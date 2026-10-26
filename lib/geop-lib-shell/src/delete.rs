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
//! - **A face grown over.** A face between others — a chamfer, a fillet,
//!   a cut-off corner — is bounded by edges on its neighbours' outer
//!   loops, which close over it once they meet. A face of three sides
//!   leaves a corner: its three vertices become one, where the three
//!   neighbours now meet. A face of four sides leaves an edge: two of its
//!   opposite sides, the rails, become one edge where the faces across
//!   them now meet, and the two ends shrink to a vertex each, where those
//!   faces meet the face across the end. Which two sides are the rails is
//!   tried both ways; the rebuild of the other comes out first where it
//!   fails (two faces across that never meet, as a box's two ends).
//!
//! Anything else is refused, by name.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    vector::Vector3,
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
    let deleted: Vec<bool> = sources.faces.iter().map(|f| faces.contains(f)).collect();
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
            let (healed, healed_names) = healed(&spec, &names, &deleted).with_context(ctx)?;
            part.replace_with_solid(&[solid.into()], healed, healed_names)
                .map(Body::Solid)
                .with_context(ctx)
        }
    }
}

/// The solid `spec`, named `names`, with the faces marked `deleted` taken
/// out and the gap healed over (see the module docs).
fn healed(
    spec: &BodySpec,
    names: &BodyNames,
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
    if !crossing {
        let mut edited = Edited::new(spec, names);
        edited.delete = deleted.to_vec();
        for &(f, h) in &closed_holes {
            edited.closed.push((f, h));
        }
        return edited.compacted(true).map(|(spec, names, _)| (spec, names));
    }
    let picked: Vec<usize> = (0..spec.faces.len()).filter(|&f| deleted[f]).collect();
    let [f] = picked.as_slice() else {
        let listed: Vec<&str> = picked.iter().map(|&f| names.faces[f].as_str()).collect();
        return Err(GeopError::new(format!(
            "the faces {listed:?} cross their neighbours' outer loops: healing over several faces at once is only supported where they fill holes"
        )));
    };
    let face = &spec.faces[*f];
    if !face.holes.is_empty() {
        return Err(GeopError::new(format!(
            "face {} has holes: healing over it is not supported",
            names.faces[*f]
        )));
    }
    let rebuild = |edited: Edited| -> GeopResult<(BodySpec, BodyNames)> {
        let (spec, names, reseat) = edited.compacted(true)?;
        let replaced = vec![false; spec.faces.len()];
        let result = rebuilt(&spec, &names, &replaced, &reseat)?;
        Ok((result, names))
    };
    let sides = (face.outer.iter())
        .filter(|c| matches!(c.on, CoedgeOn::Edge(..)))
        .count();
    match sides {
        3 => rebuild(Edited::grown_over(spec, names, *f, None)?),
        4 => {
            let mut failures = Vec::new();
            for rails in [[0, 2], [1, 3]] {
                match Edited::grown_over(spec, names, *f, Some(rails)).and_then(rebuild) {
                    Ok(done) => return Ok(done),
                    Err(e) => failures.push(e),
                }
            }
            Err(GeopError::new(format!(
                "the faces across neither pair of its opposite sides meet — across sides 0 and 2: {}; across sides 1 and 3: {}",
                failures[0].root_message(),
                failures[1].root_message()
            )))
            .map_err(|e| e.with_context(format!("healing over face {}", names.faces[*f])))
        }
        n => Err(GeopError::new(format!(
            "face {} has {n} sides: healing over a face of other than three or four sides is not supported",
            names.faces[*f]
        ))),
    }
}

/// A solid described as a [`BodySpec`], and how its topology is to change:
/// faces taken out, holes closed, edges dropped or standing in for others,
/// vertices made one.
struct Edited<'a> {
    spec: &'a BodySpec,
    names: &'a BodyNames,
    /// Per face: whether it is taken out.
    delete: Vec<bool>,
    /// The holes closed: a face, and which of its holes.
    closed: Vec<(usize, usize)>,
    /// Per edge: whether it goes, its coedges with it.
    dropped: Vec<bool>,
    /// Edges standing in for others: `(e, by)`, every coedge on `e` then on
    /// `by`, in the sense that runs it the same way.
    swapped: Vec<(usize, usize)>,
    /// Per vertex: the vertex it is made one with (itself, if none).
    vertex: Vec<usize>,
    /// Per vertex: where to start looking for it, if it moves.
    seeds: Vec<Option<Vector3>>,
}

impl<'a> Edited<'a> {
    fn new(spec: &'a BodySpec, names: &'a BodyNames) -> Self {
        Self {
            spec,
            names,
            delete: vec![false; spec.faces.len()],
            closed: Vec::new(),
            dropped: vec![false; spec.edges.len()],
            swapped: Vec::new(),
            vertex: (0..spec.vertices.len()).collect(),
            seeds: vec![None; spec.vertices.len()],
        }
    }

    /// The face `f` taken out and its neighbours grown over it (see the
    /// module docs): of three sides if `rails` is none, of four with the
    /// sides `rails` of its outer loop the rails.
    fn grown_over(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        f: usize,
        rails: Option<[usize; 2]>,
    ) -> GeopResult<Self> {
        let mut edited = Self::new(spec, names);
        edited.delete[f] = true;
        // Its sides, the edges of its outer loop: a coedge at a pole,
        // where the face comes to a point, is no side.
        let sides: Vec<CoedgeOn> = (spec.faces[f].outer.iter())
            .map(|c| c.on)
            .filter(|on| matches!(on, CoedgeOn::Edge(..)))
            .collect();
        let n = sides.len();
        let edges: Vec<usize> = (sides.iter())
            .map(|on| match on {
                CoedgeOn::Edge(e, _) => *e,
                CoedgeOn::Vertex(_) => unreachable!("filtered"),
            })
            .collect();
        for &e in &edges {
            edited.dropped[e] = true;
        }
        // Where each side starts.
        let starts: Vec<usize> = (sides.iter())
            .map(|&on| Ok(spec.ends(on)?.0))
            .collect::<GeopResult<_>>()?;
        let groups: Vec<Vec<usize>> = match rails {
            None => vec![(0..n).collect()],
            Some([a, b]) => {
                let (r1, r2) = (edges[a], edges[b]);
                if r1 == r2 {
                    return Err(GeopError::new(format!(
                        "face {} runs along edge {} on both sides: healing over it is not supported",
                        names.faces[f], names.edges[r1]
                    )));
                }
                edited.dropped[r1] = false;
                edited.swapped.push((r2, r1));
                vec![
                    vec![(a + 1) % n, (a + 2) % n],
                    vec![(b + 1) % n, (b + 2) % n],
                ]
            }
        };
        for group in groups {
            let vertices: Vec<usize> = group.iter().map(|&k| starts[k]).collect();
            let keep = vertices[0];
            let mut sum = Vector3::zero();
            for &v in &vertices {
                edited.vertex[v] = keep;
                sum = sum.add(&spec.vertices[v]);
            }
            // Where to start looking for it: their middle, a free choice.
            let share = geop_core_math::scalars::Scalar::from_ratio(1, vertices.len() as i64)?;
            edited.seeds[keep] = Some(sum.prod_scalar(share).sharpen());
        }
        Ok(edited)
    }

    /// The edited solid, its entities numbered anew and named as they
    /// were, and per vertex whether it moves.
    fn compacted(&self, solid: bool) -> GeopResult<(BodySpec, BodyNames, Vec<bool>)> {
        let spec = self.spec;
        let rep = |v: usize| self.vertex[v];
        let by: Vec<usize> = (0..spec.edges.len())
            .map(|e| {
                (self.swapped.iter())
                    .find(|&&(from, _)| from == e)
                    .map_or(e, |&(_, to)| to)
            })
            .collect();
        // The faces that stay, their loops on the edges that stay.
        let mut faces: Vec<(usize, Vec<Vec<CoedgeSpec>>)> = Vec::new();
        for (f, face) in spec.faces.iter().enumerate() {
            if self.delete[f] {
                continue;
            }
            let mut loops = Vec::new();
            for (l, lp) in face.loops().into_iter().enumerate() {
                if l > 0 && self.closed.contains(&(f, l - 1)) {
                    continue;
                }
                let mut kept = Vec::with_capacity(lp.len());
                for c in lp {
                    let on = match c.on {
                        CoedgeOn::Edge(e, _) if by[e] != e => {
                            // Runs the edge standing in for `e` the way it
                            // ran `e`, between the vertices made one.
                            let (x, y) = spec.ends(c.on)?;
                            let to = &spec.edges[by[e]];
                            let sense = if (rep(to.start), rep(to.end)) == (rep(x), rep(y)) {
                                Sense::Forward
                            } else if (rep(to.end), rep(to.start)) == (rep(x), rep(y)) {
                                Sense::Reversed
                            } else {
                                return Err(GeopError::new(format!(
                                    "edge {} cannot stand in for edge {}: they no longer join the same vertices",
                                    self.names.edges[by[e]], self.names.edges[e]
                                )));
                            };
                            CoedgeOn::Edge(by[e], sense)
                        }
                        CoedgeOn::Edge(e, _) if self.dropped[e] => continue,
                        on => on,
                    };
                    kept.push(CoedgeSpec {
                        on,
                        pcurve: c.pcurve.clone(),
                    });
                }
                if kept.is_empty() {
                    return Err(GeopError::new(format!(
                        "a loop of face {} would be left with no edge",
                        self.names.faces[f]
                    )));
                }
                loops.push(kept);
            }
            faces.push((f, loops));
        }
        // The edges and vertices those loops use, numbered anew.
        let mut edge_index = vec![None; spec.edges.len()];
        let mut vertex_index = vec![None; spec.vertices.len()];
        let mut out = BodySpec::new(solid);
        let mut names = BodyNames {
            solid: self.names.solid.clone().filter(|_| solid),
            ..Default::default()
        };
        let mut reseat = Vec::new();
        let mut vertex = |v: usize, out: &mut BodySpec, names: &mut BodyNames| {
            let v = rep(v);
            *vertex_index[v].get_or_insert_with(|| {
                out.vertices.push(self.seeds[v].unwrap_or(spec.vertices[v]));
                names.vertices.push(self.names.vertices[v].clone());
                reseat.push(self.seeds[v].is_some());
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
                            let index = match edge_index[e] {
                                Some(i) => i,
                                None => {
                                    let edge = &spec.edges[e];
                                    let start = vertex(edge.start, &mut out, &mut names);
                                    let end = vertex(edge.end, &mut out, &mut names);
                                    out.edges.push(EdgeSpec {
                                        curve: edge.curve.clone(),
                                        start,
                                        end,
                                    });
                                    names.edges.push(self.names.edges[e].clone());
                                    edge_index[e] = Some(out.edges.len() - 1);
                                    out.edges.len() - 1
                                }
                            };
                            CoedgeOn::Edge(index, sense)
                        }
                        CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertex(v, &mut out, &mut names)),
                    };
                    coedges.push(CoedgeSpec {
                        on,
                        pcurve: c.pcurve,
                    });
                }
                renumbered.push(coedges);
            }
            let outer = renumbered.remove(0);
            new_faces.push(FaceSpec {
                surface: spec.faces[f].surface.clone(),
                outer,
                holes: renumbered,
            });
            names.faces.push(self.names.faces[f].clone());
        }
        out.faces = new_faces;
        out.shells = vec![(0..out.faces.len()).collect()];
        out.check()?;
        Ok((out, names, reseat))
    }
}
