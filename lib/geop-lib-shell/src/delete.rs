//! [`delete_faces`]: faces taken out of a solid, and what is left either
//! open, a sheet ([`Heal::Open`]), or healed over by the faces around the
//! gap grown until they meet ([`Heal::Extend`]).
//!
//! Healing is decided by topology, then built by [`rebuilt`]:
//!
//! - **A hole closed.** Faces that fill holes of their neighbours — a
//!   boss standing on a face, a hole through it, a pocket — are bounded
//!   by whole hole loops of the faces around them. Taking them out and
//!   those holes away leaves the neighbours as they were otherwise, and
//!   nothing moves.
//! - **Faces grown over.** Faces between others — a chamfer, a fillet,
//!   a chain of them, a cut-off corner — leave a gap bounded by sides on
//!   their neighbours' loops, which close over it once they meet. Each
//!   three of the neighbours meet at a corner, each two sharing two
//!   corners along a new edge: a triangulation of the gap's polygon of
//!   sides. A gap of three sides leaves one corner; of four, an edge
//!   between the two faces across it; of more, a tree of edges. Which
//!   faces meet is not decided by kind but by trying every way, from the
//!   first the rebuild accepts: the others fail where they would make
//!   faces meet that never do, as a box's two ends, or edges that turn
//!   around.
//!
//! Anything else is refused, by name.

use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    search::PROJECT_ITERATIONS,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, FaceId, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

use crate::rebuild::rebuilt;

/// What [`delete_faces`] leaves where the faces were.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Heal {
    /// Nothing: the solid is left open, a sheet of the faces that stay.
    Open,
    /// The faces around the gap, grown until they meet (see the module
    /// docs).
    Extend,
}

/// Takes the faces `faces` out of their solid of `part` and leaves the gap
/// as `heal` says — see the module docs — and returns what is left: the
/// healed solid, named `namer`'s root, or the sheet. Every face, edge and
/// vertex that stays keeps its name; of two vertices made one, the one the
/// face's loop reaches first.
pub fn delete_faces(
    part: &mut NamedModel,
    namer: &Namer,
    faces: &[FaceId],
    whole_feature: bool,
    heal: Heal,
) -> GeopResult<Body> {
    let ctx = with_context!(
        "delete_faces({}, faces={faces:?}, heal={heal:?})",
        namer.root()
    );
    let Some(&first) = faces.first() else {
        return Err(GeopError::new("pick the faces to delete")).with_context(ctx);
    };
    let solid = part
        .solid_of_face(first, "only faces of a solid can be deleted")
        .with_context(ctx)?;
    let model = part.topology();
    let all = model.solid_faces(solid).with_context(ctx)?;
    if let Some(&f) = faces.iter().find(|f| !all.contains(f)) {
        return Err(GeopError::new(format!(
            "face {} is not of the same solid as face {}: delete the faces of one solid at a time",
            part.name(f)?,
            part.name(first)?
        )))
        .with_context(ctx);
    }
    if model.get_solid(solid).with_context(ctx)?.shells.len() != 1 {
        return Err(GeopError::new(
            "the solid has a void: deleting its faces is not supported",
        ))
        .with_context(ctx);
    }
    let (spec, sources) = model.body_spec(&all, true).with_context(ctx)?;
    let names = BodyNames {
        solid: Some(namer.root()),
        ..part.source_names(&sources)?
    };
    let mut deleted: Vec<bool> = sources.faces.iter().map(|f| faces.contains(f)).collect();
    if whole_feature {
        deleted = feature_of(&spec, &names, &deleted).with_context(ctx)?;
    }
    if deleted.iter().all(|&d| d) {
        return Err(GeopError::new(
            "every face of the solid is picked: delete the body instead",
        ))
        .with_context(ctx);
    }
    match heal {
        Heal::Open => {
            let mut edited = Edited::new(&spec, &names);
            edited.delete = deleted;
            let (sheet, sheet_names, _) = edited.compacted(false)?;
            part.delete_bodies(&[solid.into()]).with_context(ctx)?;
            part.build_body(sheet, sheet_names)
                .with_context(ctx)?
                .body()
                .with_context(ctx)
        }
        Heal::Extend => {
            let (healed, healed_names) =
                healed(&spec, &names, namer, &deleted).with_context(ctx)?;
            part.replace_with_solid(&[solid.into()], healed, healed_names)
                .map(Body::Solid)
                .with_context(ctx)
        }
    }
}

/// The faces of the features the faces marked `picked` are part of: every
/// face reached from one of them across an edge that lies on no face's hole
/// — a hole's wall and its bottom, a boss's wall and top — for a feature
/// that fills holes of the faces around it, and so can be taken out with
/// them (see the module docs). Refused, naming the face, where what is
/// reached so is no such feature: a face of the body the features stand
/// on, whose own holes stop the search.
fn feature_of(spec: &BodySpec, names: &BodyNames, picked: &[bool]) -> GeopResult<Vec<bool>> {
    let mut on_hole = vec![false; spec.edges.len()];
    for face in &spec.faces {
        for c in face.holes.iter().flatten() {
            if let CoedgeOn::Edge(e, _) = c.on {
                on_hole[e] = true;
            }
        }
    }
    let uses = spec.edge_uses();
    let mut feature = picked.to_vec();
    let mut todo: Vec<usize> = (0..picked.len()).filter(|&f| picked[f]).collect();
    while let Some(f) = todo.pop() {
        for c in spec.faces[f].loops().into_iter().flatten() {
            let CoedgeOn::Edge(e, _) = c.on else { continue };
            if on_hole[e] {
                continue;
            }
            for &(g, ..) in &uses[e] {
                if !feature[g] {
                    feature[g] = true;
                    todo.push(g);
                }
            }
        }
    }
    if let Some(f) = (0..picked.len()).find(|&f| picked[f])
        && feature.iter().all(|&inside| inside)
    {
        return Err(GeopError::new(format!(
            "face {} is part of no feature filling a hole of the faces around it: every face of the solid is reached from it",
            names.faces[f]
        )));
    }
    // Its boundary has to be holes of the faces around it.
    for (e, at) in uses.iter().enumerate() {
        let [(f, ..), (g, ..)] = at.as_slice() else {
            continue;
        };
        if feature[*f] == feature[*g] {
            continue;
        }
        let (inside, outside) = if feature[*f] { (*f, *g) } else { (*g, *f) };
        let on_outside_hole = (spec.faces[outside].holes.iter().flatten())
            .any(|c| matches!(c.on, CoedgeOn::Edge(x, _) if x == e));
        if !on_outside_hole {
            return Err(GeopError::new(format!(
                "face {} is part of no feature filling a hole of the faces around it: edge {} between it and face {} lies on no hole of that face",
                names.faces[inside], names.edges[e], names.faces[outside]
            )));
        }
    }
    Ok(feature)
}

/// The solid `spec`, named `names`, with the faces marked `deleted` taken
/// out and the gap healed over (see the module docs). What it creates is
/// named by `namer`.
fn healed(
    spec: &BodySpec,
    names: &BodyNames,
    namer: &Namer,
    deleted: &[bool],
) -> GeopResult<(BodySpec, BodyNames)> {
    // Every loop of a face that stays, and which of its coedges run along
    // an edge of a face deleted.
    let uses = spec.edge_uses();
    let across = |e: usize| uses[e].iter().any(|&(f, ..)| deleted[f]);
    let mut closed_holes: Vec<(usize, usize)> = Vec::new();
    let mut crossing = false;
    for (f, face) in spec.faces.iter().enumerate() {
        if deleted[f] {
            continue;
        }
        for (l, lp) in face.loops().into_iter().enumerate() {
            let along = (lp.iter())
                .filter(|c| matches!(c.on, CoedgeOn::Edge(e, _) if across(e)))
                .count();
            if along == 0 {
                continue;
            }
            if l > 0 && along == lp.len() {
                closed_holes.push((f, l - 1));
            } else {
                crossing = true;
            }
        }
    }
    let listed = || -> Vec<&str> {
        (0..spec.faces.len())
            .filter(|&f| deleted[f])
            .map(|f| names.faces[f].as_str())
            .collect()
    };
    if !crossing {
        let mut edited = Edited::new(spec, names);
        edited.delete = deleted.to_vec();
        edited.closed = closed_holes;
        return edited.compacted(true).map(|(spec, names, _)| (spec, names));
    }
    if !closed_holes.is_empty() {
        return Err(GeopError::new(format!(
            "the faces {:?} both fill holes and cross their neighbours' outer loops: healing over them at once is not supported",
            listed()
        )));
    }
    let sides = gap_sides(spec, names, deleted)?;
    let n = sides.len();
    if !(3..=MOST_SIDES).contains(&n) {
        return Err(GeopError::new(format!(
            "the gap the faces {:?} leave has {n} sides: healing over a gap of fewer than 3 or more than {MOST_SIDES} sides is not supported",
            listed()
        )));
    }
    let mut failures = Vec::new();
    for triangles in triangulations(n) {
        let tried = Grown::new(spec, names, namer, deleted, &sides, &triangles)
            .and_then(|grown| grown.compacted())
            .and_then(|(grown, grown_names, reseat)| {
                let replaced = vec![false; grown.faces.len()];
                Ok((
                    rebuilt(&grown, &grown_names, &replaced, &reseat)?,
                    grown_names,
                ))
            });
        match tried {
            Ok(done) => return Ok(done),
            Err(e) => failures.push(format!("{triangles:?}: {}", e.root_message())),
        }
    }
    let around: Vec<&str> = sides.iter().map(|s| names.faces[s.face].as_str()).collect();
    Err(GeopError::new(format!(
        "the faces around the gap, {around:?} in order, meet in none of the {} ways they could close it, each three of them at a corner — {}",
        failures.len(),
        failures.join("; ")
    )))
    .map_err(|e| e.with_context(format!("healing over the faces {:?}", listed())))
}

/// The most sides a gap may have to be healed over: every way its faces
/// could close it is tried, and their number grows as the Catalan numbers
/// (429 for nine sides).
const MOST_SIDES: usize = 8;

/// A side of the gap the faces deleted leave: the coedges of one loop of a
/// face that stays, `face`'s loop `lp`, that run along edges of the faces
/// deleted, one after the other — at positions `run` of the loop — from
/// the vertex `start` to `end`.
struct Side {
    face: usize,
    lp: usize,
    run: Vec<usize>,
    start: usize,
    end: usize,
}

/// The sides of the gap the faces marked `deleted` leave, in order round
/// it: each starts where the one before ends, at a corner of the gap.
/// Refused, by name, where the gap has more than one boundary — the faces
/// deleted have a hole, or enclose a face that stays — or touches itself
/// at a vertex.
fn gap_sides(spec: &BodySpec, names: &BodyNames, deleted: &[bool]) -> GeopResult<Vec<Side>> {
    let uses = spec.edge_uses();
    let across = |e: usize| uses[e].iter().any(|&(f, ..)| deleted[f]);
    let mut sides = Vec::new();
    for (f, face) in spec.faces.iter().enumerate() {
        if deleted[f] {
            continue;
        }
        for (l, lp) in face.loops().into_iter().enumerate() {
            let along: Vec<bool> = (lp.iter())
                .map(|c| matches!(c.on, CoedgeOn::Edge(e, _) if across(e)))
                .collect();
            let n = lp.len();
            for k in 0..n {
                if !along[k] || along[(k + n - 1) % n] {
                    continue;
                }
                let mut run = Vec::new();
                let mut j = k;
                while along[j] && run.len() < n {
                    run.push(j);
                    j = (j + 1) % n;
                }
                let start = spec.ends(lp[run[0]].on)?.0;
                let end = spec.ends(lp[run[run.len() - 1]].on)?.1;
                sides.push(Side {
                    face: f,
                    lp: l,
                    run,
                    start,
                    end,
                });
            }
        }
    }
    // Chained, each from where the one before ends.
    let mut ordered: Vec<Side> = Vec::with_capacity(sides.len());
    let total = sides.len();
    let Some(first) = sides.pop() else {
        return Err(GeopError::new(
            "the faces deleted border no face that stays",
        ));
    };
    ordered.push(first);
    loop {
        let end = ordered[ordered.len() - 1].end;
        let next: Vec<usize> = (0..sides.len())
            .filter(|&k| sides[k].start == end)
            .collect();
        match next.as_slice() {
            [] if end == ordered[0].start => break,
            [k] => ordered.push(sides.swap_remove(*k)),
            _ => {
                return Err(GeopError::new(format!(
                    "the gap the faces deleted leave touches itself at vertex {}: healing over it is not supported",
                    names.vertices[end]
                )));
            }
        }
    }
    if ordered.len() != total {
        return Err(GeopError::new(format!(
            "the gap the faces deleted leave has more than one boundary — faces {:?} lie along another: healing over it is not supported",
            sides
                .iter()
                .map(|s| names.faces[s.face].as_str())
                .collect::<Vec<_>>()
        )));
    }
    Ok(ordered)
}

/// Every triangulation of the polygon of corners `0..n`: the ways the `n`
/// faces around a gap can close it, each three of them meeting at a
/// corner, each two not side by side along an edge.
fn triangulations(n: usize) -> Vec<Vec<[usize; 3]>> {
    fn of(corners: &[usize]) -> Vec<Vec<[usize; 3]>> {
        if corners.len() < 3 {
            return vec![Vec::new()];
        }
        let (first, last) = (corners[0], corners[corners.len() - 1]);
        let mut out = Vec::new();
        for m in 1..corners.len() - 1 {
            for left in of(&corners[..=m]) {
                for right in of(&corners[m..]) {
                    let mut triangles = vec![[first, corners[m], last]];
                    triangles.extend(left.iter().copied());
                    triangles.extend(right.iter().copied());
                    out.push(triangles);
                }
            }
        }
        out
    }
    of(&(0..n).collect::<Vec<_>>())
}

/// A solid with the faces deleted taken out and the faces around the gap
/// grown over it in one way, `triangles`: each three of those faces — the
/// faces of three sides of the gap — meeting at a corner; each two sharing
/// two corners meeting along a new edge between them; each two sides side
/// by side meeting where they met, along the edge at the gap's corner
/// between them, carried on to where the third face of their corner meets
/// them. The gap's corners move to the corners of the triangles, a
/// triangle with none gets a new one, and the faces' sides along the gap
/// are their new edges to the faces they meet.
///
/// The old entities keep their index, the new ones come after them;
/// [`Grown::compacted`] numbers them anew.
struct Grown<'a> {
    spec: &'a BodySpec,
    names: &'a BodyNames,
    /// Every vertex — the old, then the new — with the point to start
    /// looking for it from, its name, and whether it moves.
    vertices: Vec<(Vector3, String, bool)>,
    /// Per old vertex: the vertex it becomes.
    vertex: Vec<usize>,
    /// The new edges, between the triangles' corners, and their names.
    edges: Vec<(EdgeSpec, String)>,
    /// Per face that stays: its loops, the sides along the gap replaced.
    faces: Vec<(usize, Vec<Vec<CoedgeSpec>>)>,
}

impl<'a> Grown<'a> {
    fn new(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        namer: &Namer,
        deleted: &[bool],
        sides: &[Side],
        triangles: &[[usize; 3]],
    ) -> GeopResult<Self> {
        let n = sides.len();
        let face_name = |s: usize| names.faces[sides[s].face].as_str();
        // Two faces meeting along a new edge have to be two.
        let adjacent = |a: usize, b: usize| (a + 1) % n == b || (b + 1) % n == a;
        for t in triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[0], t[2])] {
                if !adjacent(a, b) && sides[a].face == sides[b].face {
                    return Err(GeopError::new(format!(
                        "face {} would meet itself across the gap",
                        face_name(a)
                    )));
                }
            }
        }
        let around = spec.vertex_faces()?;
        let mut grown = Grown {
            spec,
            names,
            vertices: (spec.vertices.iter().zip(&names.vertices))
                .map(|(p, name)| (*p, name.clone(), false))
                .collect(),
            vertex: (0..spec.vertices.len()).collect(),
            edges: Vec::new(),
            faces: Vec::new(),
        };
        // The corner of each triangle: the gap's corners it closes made one,
        // at their middle, or a new one in the middle of its sides' ends.
        let corner = |s: usize| sides[s].end;
        let closes = |t: &[usize; 3], s: usize| t.contains(&s) && t.contains(&((s + 1) % n));
        let mut corners = Vec::with_capacity(triangles.len());
        for t in triangles {
            let closed: Vec<usize> = (0..n).filter(|&s| closes(t, s)).map(corner).collect();
            let near: Vec<usize> = if closed.is_empty() {
                (t.iter())
                    .flat_map(|&s| [sides[s].start, sides[s].end])
                    .collect()
            } else {
                closed.clone()
            };
            let mut sum = Vector3::zero();
            for &v in &near {
                sum = sum.add(&spec.vertices[v]);
            }
            let share = Scalar::from_ratio(1, near.len() as i64)?;
            let seed = sum.prod_scalar(share).sharpen();
            let index = match closed.first() {
                Some(&keep) => {
                    for &v in &closed {
                        grown.vertex[v] = keep;
                    }
                    keep
                }
                None => {
                    let faces: Vec<&str> = t.iter().map(|&s| face_name(s)).collect();
                    grown.vertices.push((seed, namer.name(&faces), true));
                    grown.vertices.len() - 1
                }
            };
            grown.vertices[index].0 = seed;
            grown.vertices[index].2 = true;
            corners.push(index);
        }
        // Where a triangle's corner is on the face of side `s`: where a
        // gap's corner it closes was, if one was on that face, or else its
        // seed's foot on the face's surface, from where the side starts.
        let uv_on = |s: usize, t: usize| -> GeopResult<Vector2> {
            let f = sides[s].face;
            for v in (0..spec.vertices.len()).filter(|&v| grown.vertex[v] == corners[t]) {
                if let Some(&(_, uv)) = around[v].iter().find(|&&(g, _)| g == f) {
                    return Ok(uv);
                }
            }
            let start = (around[sides[s].start].iter())
                .find(|&&(g, _)| g == f)
                .map(|&(_, uv)| uv)
                .ok_or_else(|| GeopError::new("a side's start is not on its face"))?;
            let (u, v) = spec.faces[f].surface.project_in_domain(
                grown.vertices[corners[t]].0,
                start[0].sharpen(),
                start[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            Ok(Vector2::from_array([u.sharpen(), v.sharpen()]))
        };
        // Per side, the triangles round its face, from the one at the
        // gap's corner it starts at to the one at the corner it ends at:
        // by how far round the gap their other sides are, furthest first.
        let fan = |s: usize| -> Vec<usize> {
            let offset = |x: usize| (x + n - s) % n;
            let mut round: Vec<usize> = (0..triangles.len())
                .filter(|&t| triangles[t].contains(&s))
                .collect();
            round
                .sort_by_key(|&t| std::cmp::Reverse(triangles[t].iter().map(|&x| offset(x)).max()));
            round
        };
        // The new edges: one per pair of triangles sharing two sides.
        let mut diagonal: Vec<((usize, usize), usize)> = Vec::new();
        let mut used_names: Vec<usize> = Vec::new();
        for s in 0..n {
            let round = fan(s);
            for pair in round.windows(2) {
                let (t1, t2) = (pair[0], pair[1]);
                let x = *(triangles[t1].iter())
                    .find(|&&x| x != s && triangles[t2].contains(&x))
                    .expect("two triangles round a side share another");
                let key = (s.min(x), s.max(x));
                if diagonal.iter().any(|(k, _)| *k == key) {
                    continue;
                }
                let (a, b) = (grown.vertices[corners[t1]].0, grown.vertices[corners[t2]].0);
                // Named as the edge of a side of one edge it takes the place
                // of, where there is one, as a box's edge a chamfer took.
                let single = |side: usize| match sides[side].run.as_slice() {
                    [k] => match spec.faces[sides[side].face].loops()[sides[side].lp][*k].on {
                        CoedgeOn::Edge(e, _) if !used_names.contains(&e) => Some(e),
                        _ => None,
                    },
                    _ => None,
                };
                let name = match single(s).or_else(|| single(x)) {
                    Some(e) => {
                        used_names.push(e);
                        names.edges[e].clone()
                    }
                    None => namer.name(&[face_name(key.0), face_name(key.1)]),
                };
                grown.edges.push((
                    EdgeSpec {
                        curve: NurbCurve3D::line(a, b)?,
                        start: corners[t1],
                        end: corners[t2],
                    },
                    name,
                ));
                diagonal.push((key, spec.edges.len() + grown.edges.len() - 1));
            }
        }
        // Each face's loops, each side along the gap replaced by the new
        // edges round its fan.
        for (f, face) in spec.faces.iter().enumerate() {
            if deleted[f] {
                continue;
            }
            let mut loops = Vec::new();
            for (l, lp) in face.loops().into_iter().enumerate() {
                let mut kept = Vec::with_capacity(lp.len());
                for (k, c) in lp.iter().enumerate() {
                    let Some(s) = (0..n).find(|&s| {
                        sides[s].face == f && sides[s].lp == l && sides[s].run.contains(&k)
                    }) else {
                        kept.push(c.clone());
                        continue;
                    };
                    if sides[s].run[0] != k {
                        continue;
                    }
                    let round = fan(s);
                    for pair in round.windows(2) {
                        let (t1, t2) = (pair[0], pair[1]);
                        let x = *(triangles[t1].iter())
                            .find(|&&x| x != s && triangles[t2].contains(&x))
                            .expect("two triangles round a side share another");
                        let e = (diagonal.iter())
                            .find(|(key, _)| *key == (s.min(x), s.max(x)))
                            .map(|&(_, e)| e)
                            .expect("every pair round a fan has its edge");
                        let edge = &grown.edges[e - spec.edges.len()].0;
                        let sense = if edge.start == corners[t1] {
                            Sense::Forward
                        } else {
                            Sense::Reversed
                        };
                        let pcurve = NurbCurve2D::line(uv_on(s, t1)?, uv_on(s, t2)?)?;
                        kept.push(CoedgeSpec {
                            on: CoedgeOn::Edge(e, sense),
                            pcurve,
                        });
                    }
                }
                loops.push(kept);
            }
            grown.faces.push((f, loops));
        }
        Ok(grown)
    }

    /// The solid, its entities numbered anew, named, and per vertex whether
    /// it moves.
    fn compacted(&self) -> GeopResult<(BodySpec, BodyNames, Vec<bool>)> {
        let old = self.spec.edges.len();
        let edge = |e: usize| -> (EdgeSpec, &str) {
            if e < old {
                let edge = &self.spec.edges[e];
                let spec = EdgeSpec {
                    curve: edge.curve.clone(),
                    start: self.vertex[edge.start],
                    end: self.vertex[edge.end],
                };
                (spec, self.names.edges[e].as_str())
            } else {
                let (spec, name) = &self.edges[e - old];
                (spec.clone(), name.as_str())
            }
        };
        let faces = (self.faces.iter())
            .map(|(f, loops)| {
                let loops = (loops.iter())
                    .map(|lp| {
                        (lp.iter())
                            .map(|c| CoedgeSpec {
                                on: match c.on {
                                    CoedgeOn::Vertex(v) => CoedgeOn::Vertex(self.vertex[v]),
                                    on => on,
                                },
                                pcurve: c.pcurve.clone(),
                            })
                            .collect()
                    })
                    .collect();
                (*f, loops)
            })
            .collect();
        renumbered(
            self.spec,
            self.names,
            faces,
            edge,
            |v| {
                (
                    self.vertices[v].0,
                    self.vertices[v].1.as_str(),
                    self.vertices[v].2,
                )
            },
            true,
        )
    }
}

/// The solid of the faces `faces` of `spec` — each by its index and its
/// loops, which run along the edges `edge(e)` gives and the vertices
/// `vertex(v)` gives, each as its geometry and name, a vertex also whether
/// it moves — with only the edges and vertices those loops use, numbered
/// anew in the order the loops reach them.
fn renumbered<'n>(
    spec: &BodySpec,
    names: &BodyNames,
    faces: Vec<(usize, Vec<Vec<CoedgeSpec>>)>,
    edge: impl Fn(usize) -> (EdgeSpec, &'n str),
    vertex: impl Fn(usize) -> (Vector3, &'n str, bool),
    solid: bool,
) -> GeopResult<(BodySpec, BodyNames, Vec<bool>)> {
    let mut edge_index: std::collections::HashMap<usize, usize> = Default::default();
    let mut vertex_index: std::collections::HashMap<usize, usize> = Default::default();
    let mut out = BodySpec::new(solid);
    let mut out_names = BodyNames {
        solid: names.solid.clone().filter(|_| solid),
        ..Default::default()
    };
    let mut reseat = Vec::new();
    let mut number_vertex = |v: usize, out: &mut BodySpec, out_names: &mut BodyNames| {
        *vertex_index.entry(v).or_insert_with(|| {
            let (point, name, moves) = vertex(v);
            out.vertices.push(point);
            out_names.vertices.push(name.to_string());
            reseat.push(moves);
            out.vertices.len() - 1
        })
    };
    let mut new_faces = Vec::with_capacity(faces.len());
    for (f, loops) in faces {
        let mut renumbered = Vec::with_capacity(loops.len());
        for lp in loops {
            let mut coedges = Vec::with_capacity(lp.len());
            for c in lp {
                let on = match c.on {
                    CoedgeOn::Edge(e, sense) => {
                        let index = match edge_index.get(&e) {
                            Some(&i) => i,
                            None => {
                                let (mut spec_edge, name) = edge(e);
                                spec_edge.start =
                                    number_vertex(spec_edge.start, &mut out, &mut out_names);
                                spec_edge.end =
                                    number_vertex(spec_edge.end, &mut out, &mut out_names);
                                out.edges.push(spec_edge);
                                out_names.edges.push(name.to_string());
                                edge_index.insert(e, out.edges.len() - 1);
                                out.edges.len() - 1
                            }
                        };
                        CoedgeOn::Edge(index, sense)
                    }
                    CoedgeOn::Vertex(v) => {
                        CoedgeOn::Vertex(number_vertex(v, &mut out, &mut out_names))
                    }
                };
                coedges.push(CoedgeSpec {
                    on,
                    pcurve: c.pcurve,
                });
            }
            renumbered.push(coedges);
        }
        if renumbered.iter().any(Vec::is_empty) {
            return Err(GeopError::new(format!(
                "a loop of face {} would be left with no edge",
                names.faces[f]
            )));
        }
        let outer = renumbered.remove(0);
        new_faces.push(FaceSpec {
            surface: spec.faces[f].surface.clone(),
            outer,
            holes: renumbered,
        });
        out_names.faces.push(names.faces[f].clone());
    }
    out.faces = new_faces;
    out.shells = vec![(0..out.faces.len()).collect()];
    out.check()?;
    Ok((out, out_names, reseat))
}

/// A solid described as a [`BodySpec`], and the faces taken out of it and
/// the holes closed.
struct Edited<'a> {
    spec: &'a BodySpec,
    names: &'a BodyNames,
    /// Per face: whether it is taken out.
    delete: Vec<bool>,
    /// The holes closed: a face, and which of its holes.
    closed: Vec<(usize, usize)>,
}

impl<'a> Edited<'a> {
    fn new(spec: &'a BodySpec, names: &'a BodyNames) -> Self {
        Self {
            spec,
            names,
            delete: vec![false; spec.faces.len()],
            closed: Vec::new(),
        }
    }

    /// The edited solid — or sheet, unless `solid` — its entities numbered
    /// anew and named as they were, and per vertex whether it moves.
    fn compacted(&self, solid: bool) -> GeopResult<(BodySpec, BodyNames, Vec<bool>)> {
        let spec = self.spec;
        let faces = (spec.faces.iter().enumerate())
            .filter(|&(f, _)| !self.delete[f])
            .map(|(f, face)| {
                let loops = (face.loops().into_iter().enumerate())
                    .filter(|&(l, _)| l == 0 || !self.closed.contains(&(f, l - 1)))
                    .map(|(_, lp)| lp.clone())
                    .collect();
                (f, loops)
            })
            .collect();
        renumbered(
            spec,
            self.names,
            faces,
            |e| (spec.edges[e].clone(), self.names.edges[e].as_str()),
            |v| (spec.vertices[v], self.names.vertices[v].as_str(), false),
            solid,
        )
    }
}
