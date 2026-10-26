//! [`shell`]: a solid hollowed out to walls of one thickness — or of one
//! per face — open where faces are taken away.
//!
//! The hollow is built directly, not cut out by a boolean: the walls'
//! inner side is the solid's own boundary moved inward, face for face, so
//! it has exactly the solid's topology, and everything about how the inner
//! and the outer side fit together is known without intersecting
//! anything.
//!
//! **The inner side.** Every face kept is offset inward by the thickness
//! (see [`NurbSurface3D::offset`]: any surface, fitted and enclosed where it
//! is no plane or revolution), every face taken away stays where it is:
//! its inner copy is what the opening leaves of it. The inner side is the
//! solid rebuilt on those surfaces ([`crate::rebuilt`]): a vertex moves to
//! where the offsets of the faces around it meet, an edge to where the
//! offsets of its two faces meet, whatever curve that is. A vertex on the
//! axis of a revolved face, at its pole, moves to the pole of the face's
//! inner copy: for a cone's apex, the offset cone's apex.
//!
//! **Faces that collapse.** A face whose offset collapses — every point
//! of it past its centre of curvature, a blend narrower than the walls are
//! thick, or the corner patch where such blends meet — has no inner copy.
//! The solid is contracted before it is rebuilt ([`Contracted`]): a
//! collapsed blend's edges along the faces it rounds, where it is tangent
//! to them, share one copy, where those faces' offsets meet — a sharp inner
//! corner; its edges across it vanish, and their ends share one vertex.
//!
//! **The opening.** Where a face `F` is taken away, its inner copy `F'`
//! lies on `F`'s own surface, inside it, and what is left of `F` is `F`
//! without `F'`: a rim, the walls' cut edge. Its boundary is `F`'s
//! boundary and `F'`'s, run the other way. Two faces taken away side by
//! side share an edge, and their inner copies share the middle of it — the
//! part between the walls; what is left of that edge is a short piece at
//! each end, where a wall reaches it, and the rims run along those. A rim
//! is put together from these pieces by following them end to end, and may
//! come out as several faces — a face taken away together with the faces
//! on two opposite sides of it leaves two strips.
//!
//! A face taken away that every face kept around it runs smoothly into —
//! the top of a box whose top edges are rounded — leaves no rim: the walls
//! end where it was, each of its edges a free edge of the face kept there,
//! with an end face along it as a sheet's border has ([`smoothly_open`]).
//!
//! **The result.** The faces kept, their inner copies turned around, and
//! the rims, as one solid. Without an opening, the inner side is a void of
//! its own: a second shell.
//!
//! **A sheet** — faces standing on their own — is shelled the same way
//! ([`thicken`]): its faces are the outer side, their inner copies the
//! inner one, and along every free edge, used by one face only, a wall
//! joins the edge to its inner copy — the ruled surface between them, with
//! a straight edge from each of its vertices to that vertex's inner copy.
//! The inner side alone is the sheet offset ([`offset_faces`]).
//!
//! What it does not do (yet): take away a face tangent to faces kept along
//! some of its edges and not others, take away a face that meets one kept
//! at an inward corner — a pocket's floor, a step, the face a boss stands
//! on: the wall kept there is thickened away from the face, so the inner
//! copy reaches past it and the opening is no longer the face less its
//! inner copy — or open a solid that already has a void. Walls so thick
//! that an inner edge turns around or shrinks to a point, or that an edge
//! of the result crosses one of its faces, are refused; two faces crossing
//! along a curve no edge reaches are not noticed.

use std::collections::HashMap;

use geop_core_geometry::{
    intersection::curve_surface_overlaps,
    nurb_curve::{NurbCurve2D, NurbCurve3D, loop_signed_area},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::walk::{Walked, walk},
    vector::{Vector, Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, Curve2, Curve3, FaceId, Model, Sense, ShellId, SolidId,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
    contains::face::{PointClassification, pcurves_contain},
    validation::{ValidationParameters, body_crossing},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

use crate::rebuild::{along, rebuilt, room, smooth_along};

/// Seed of the ray casting that finds which outer loop a hole lies in:
/// any gives the same answer (see `pcurves_contain`), a constant keeps it
/// reproducible.
const SEED: u64 = 0x5E11_0000_0000_0001;

/// Hollows the solid `solid` of `part` out to walls `thickness` thick —
/// measured inward from its faces, or outward for a negative thickness —
/// those of the faces in `thicker` as thick as it says instead — open where
/// its faces `open` were — see the module docs — and returns the hollowed
/// solid, which replaces it and is named `namer`'s root.
///
/// What stays of the solid keeps its names. What is new is named after
/// what it was made from, with `Scalar` the operation:
///
/// - `shell(Scalar,X)`: the inner copy of the face, edge or vertex `X` kept; for
///   a face `X` taken away, the rim it leaves — `shell(Scalar,X,0)`,
///   `shell(Scalar,X,1)`, ... if that is several faces;
/// - `shell(Scalar,E,start)` / `shell(Scalar,E,end)`: what is left of an edge `E`
///   between two faces taken away, at its start and its end.
pub fn shell(
    part: &mut NamedModel,
    namer: &Namer,
    solid: SolidId,
    open: &[FaceId],
    thickness: Scalar,
    thicker: &[(FaceId, Scalar)],
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "shell({}, {solid}, open={open:?}, {thickness:?}, thicker={thicker:?})",
        namer.root()
    );
    // Outward, the walls grow from the faces out: the solid turned inside
    // out and shelled inward, then turned back.
    let outward = thickness.definitely_less(Scalar::ZERO);
    let sign = |t: Scalar| if outward { t.neg() } else { t };
    for &t in std::iter::once(&thickness).chain(thicker.iter().map(|(_, t)| t)) {
        if !sign(t).definitely_greater(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "a shell's walls need a thickness other than zero, all inward or all outward, not {t:?}"
            )))
            .with_context(ctx);
        }
    }
    let model = part.topology();
    if model.get_solid(solid).with_context(ctx)?.shells.len() != 1 {
        return Err(GeopError::new(
            "the solid already has a void; shelling it is not supported",
        ))
        .with_context(ctx);
    }
    let faces = model.solid_faces(solid).with_context(ctx)?;
    if let Some(f) = open.iter().find(|f| !faces.contains(f)) {
        return Err(GeopError::new(format!(
            "face {f} is not a face of the solid being shelled"
        )))
        .with_context(ctx);
    }
    let (mut spec, sources) = model.body_spec(&faces, true).with_context(ctx)?;
    if outward {
        for face in &mut spec.faces {
            *face = face.reversed();
        }
    }
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed: Vec<bool> = sources.faces.iter().map(|f| open.contains(f)).collect();
    if let Some((f, g, e)) = inward_corner(&spec, &removed).with_context(ctx)? {
        return Err(GeopError::new(format!(
            "face {} is taken away where it meets face {} at an inward corner, along edge {}: its opening would reach past it, which is not supported",
            names.faces[f], names.faces[g], names.edges[e]
        )))
        .with_context(ctx);
    }

    if let Some((f, _)) = thicker.iter().find(|(f, _)| !faces.contains(f)) {
        return Err(GeopError::new(format!(
            "face {f} is not a face of the solid being shelled"
        )))
        .with_context(ctx);
    }
    // Per face, how thick its wall is: a face of one thickness and its
    // neighbour of another still meet where their offsets do.
    let thickness: Vec<Scalar> = (sources.faces.iter())
        .map(|f| {
            sign(
                (thicker.iter())
                    .find(|(g, _)| g == f)
                    .map_or(thickness, |&(_, t)| t),
            )
        })
        .collect();
    let hollow = Hollow::new(&spec, &names, removed, &thickness).with_context(ctx)?;
    let (mut result, result_names) = hollow.result(namer).with_context(ctx)?;
    if outward {
        // The faces turned inside out face the void they bound now, and
        // their offsets, turned around as inner copies are, face out: the
        // offsets are the outside, listed first.
        result.shells.reverse();
    }
    refuse_crossing(&result, &result_names).with_context(ctx)?;
    part.replace_with_solid(&[solid.into()], result, result_names)
        .with_context(ctx)
}

/// Thickens the sheet `sheet` of `part` into a solid of walls `thickness`
/// thick — against its faces' normals, or along them if `along_normal` —
/// and returns it: the sheet shelled (see the module docs), which replaces
/// it and is named `namer`'s root.
///
/// The sheet's faces, edges and vertices keep their names, their inner
/// copies are named as [`shell`] names them, and along a free edge `E` of
/// the sheet the wall is `N(E,side)`, with `N` the operation, the straight
/// edge from a vertex `V` of it to its inner copy `N(V,side)`.
pub fn thicken(
    part: &mut NamedModel,
    namer: &Namer,
    sheet: ShellId,
    thickness: Scalar,
    along_normal: bool,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "thicken({}, sheet {sheet}, {thickness:?}, along_normal={along_normal})",
        namer.root()
    );
    if !thickness.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "a thickness greater than zero is needed, not {thickness:?}"
        )))
        .with_context(ctx);
    }
    let model = part.topology();
    let faces = model.body_faces(Body::Sheet(sheet)).with_context(ctx)?;
    let (mut spec, sources) = model.body_spec(&faces, false).with_context(ctx)?;
    if along_normal {
        // Turned around, the walls go inward from the other side.
        for face in &mut spec.faces {
            *face = face.reversed();
        }
    }
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let thickness = vec![thickness; spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, &thickness).with_context(ctx)?;
    let (result, result_names) = hollow.result(namer).with_context(ctx)?;
    refuse_crossing(&result, &result_names).with_context(ctx)?;
    part.replace_with_solid(&[Body::Sheet(sheet)], result, result_names)
        .with_context(ctx)
}

/// Copies the faces `faces` of `part` — of a solid or of a sheet — into a
/// sheet `distance` along their normals (against them for a negative
/// distance), and returns it: the inner side of the faces shelled (see the
/// module docs), sharing nothing with them, which stay as they are.
///
/// The copy of each face, edge and vertex `X` is named `N(X)`, with `N`
/// the operation.
pub fn offset_faces(
    part: &mut NamedModel,
    namer: &Namer,
    faces: &[FaceId],
    distance: Scalar,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!("offset_faces({}, {faces:?}, {distance:?})", namer.root());
    if distance.could_be_equal(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "an offset needs a distance other than zero, not {distance:?}"
        )))
        .with_context(ctx);
    }
    let (spec, sources) = part.topology().body_spec(faces, false).with_context(ctx)?;
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let thickness = vec![distance.neg(); spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, &thickness).with_context(ctx)?;
    let (result, result_names) = hollow.inner_side(namer).with_context(ctx)?;
    refuse_crossing(&result, &result_names).with_context(ctx)?;
    part.build_body(result, result_names).with_context(ctx)
}

/// Refuses the body `spec`, named `names`, if it would run into itself:
/// an edge of it that crosses one of its faces away from every vertex.
/// The walls are built face by face from the faces' offsets, which say
/// nothing about where faces apart from each other end up: a hole closer
/// to a side than the walls are thick moves its wall through the side's.
/// Asked of the body built on its own, so the part is left as it was.
fn refuse_crossing(spec: &BodySpec, names: &BodyNames) -> GeopResult<()> {
    let mut model = Model::new();
    let built = model.build_body(spec.clone())?;
    let params = ValidationParameters::default();
    let Some((edge, face, point)) = body_crossing(&params, &model, built.body()?)? else {
        return Ok(());
    };
    let edge = built
        .edges
        .iter()
        .position(|&e| e == edge)
        .expect("an edge built");
    let face = built
        .faces
        .iter()
        .position(|&f| f == face)
        .expect("a face built");
    Err(GeopError::new(format!(
        "the walls run into each other: edge {} crosses face {} at {point:?}: the walls are too thick for this part, which is not supported",
        names.edges[edge], names.faces[face]
    )))
}

/// A face taken away, `f`, that meets a face kept, `g`, at an inward
/// corner — along the edge `e` the material between them turns more than
/// a half turn — if there is one: `(f, g, e)`.
///
/// The opening of a face taken away is what its inner copy leaves of it,
/// which assumes the inner copy lies inside the face. At an inward corner
/// it does not: the wall kept there is thickened away from the face, and
/// its end reaches past the face's edge. Decided halfway along the edge,
/// from the faces' outward normals there — those of the surfaces of a
/// [`BodySpec`] of a solid. A corner that could be tangent is no inward
/// one: it is left to the check that walls cannot end on a smooth surface.
fn inward_corner(spec: &BodySpec, removed: &[bool]) -> GeopResult<Option<(usize, usize, usize)>> {
    for (e, at) in spec.edge_uses().iter().enumerate() {
        let [(f, f_sense, f_pcurve), (g, _, g_pcurve)] = match *at.as_slice() {
            [a, b] if removed[a.0] && !removed[b.0] => [a, b],
            [a, b] if removed[b.0] && !removed[a.0] => [b, a],
            _ => continue,
        };
        let curve = &spec.edges[e].curve;
        let t = curve.mid_parameter()?;
        let point = curve.evaluate(t)?;
        let tangent = curve.tangent(t)?;
        let along = match f_sense {
            Sense::Forward => tangent,
            Sense::Reversed => tangent.neg(),
        };
        let normal = |face: usize, pcurve: &Curve2| -> GeopResult<Vector3> {
            let surface = &spec.faces[face].surface;
            let seed = pcurve.evaluate(pcurve.mid_parameter()?)?;
            let (u, v) = surface.project(
                point,
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            surface.normal(u, v)
        };
        let (n_f, n_g) = (normal(f, f_pcurve)?, normal(g, g_pcurve)?);
        // Running into `f`, square to the edge: in front of `g` at an
        // inward corner (see the fillet's bend).
        if n_f
            .prod_cross(&along)
            .prod_dot(&n_g)
            .definitely_greater(Scalar::ZERO)
        {
            return Ok(Some((f, g, e)));
        }
    }
    Ok(None)
}

/// What the inner copy of an edge is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Copy {
    /// A copy of its own.
    Own,
    /// None: the edge is of a collapsed face, across it — where a blend
    /// meets a face square to it, or another collapsed one — and its ends'
    /// copies are one point.
    Vanishes,
    /// The copy of the edge given: of a collapsed face's two edges along
    /// it, where it is tangent to the faces it rounds, the second shares
    /// the first's copy, where those faces' offsets meet.
    As(usize),
}

/// The inner side's topology where faces collapse (see the module docs):
/// `spec` without its collapsed faces, the edges across them contracted to
/// points and the two along each merged into one. Each entity of `spec`
/// is mapped to its copy here.
struct Contracted {
    spec: BodySpec,
    names: BodyNames,
    /// Per face of `spec`'s, whether the face it copies is kept.
    kept: Vec<bool>,
    /// Per vertex: its copy.
    vertex: Vec<usize>,
    /// Per edge: its copy, run the way given — `None` where it vanishes.
    edge: Vec<Option<(usize, Sense)>>,
    /// Per face: its copy, `None` where it collapses.
    face: Vec<Option<usize>>,
    /// Per coedge: where its copy is in its face's copy, loop and place.
    coedge: HashMap<At, (usize, usize)>,
}

/// What the copy of each edge of `spec` is, given which faces collapse (see
/// [`Copy`]). An edge of a collapsed face vanishes, unless the face is
/// tangent along it to a face that does not collapse: a blend narrower than
/// the walls are thick leaves the two faces it rounds meeting in a sharp
/// inner corner along it, and across it — where it meets a face square to
/// it, or another collapsed blend — nothing. Of the two edges along it, the
/// second takes the first's copy. A collapsed face with other than two such
/// edges, a free edge or a seam is refused: what would take its place is
/// not that corner.
fn collapse_edges(spec: &BodySpec, names: &BodyNames, collapsed: &[bool]) -> GeopResult<Vec<Copy>> {
    let uses = spec.edge_uses();
    let mut copy = vec![Copy::Own; spec.edges.len()];
    let mut along: Vec<Vec<usize>> = vec![Vec::new(); spec.faces.len()];
    for (e, at) in uses.iter().enumerate() {
        let [(f, ..), (g, ..)] = match *at.as_slice() {
            [a, b] if a.0 != b.0 => [a, b],
            _ => {
                if at.iter().any(|&(f, ..)| collapsed[f]) {
                    return Err(GeopError::new(format!(
                        "the walls are too thick: face {} collapses, and its edge {} is free or a seam, which is not supported",
                        names.faces[at[0].0], names.edges[e]
                    )));
                }
                continue;
            }
        };
        match (collapsed[f], collapsed[g]) {
            (false, false) => {}
            (true, true) => copy[e] = Copy::Vanishes,
            _ => {
                let gone = if collapsed[f] { f } else { g };
                if smooth_along(spec, &spec.edges[e], f, g)? {
                    along[gone].push(e);
                } else {
                    copy[e] = Copy::Vanishes;
                }
            }
        }
    }
    for (f, edges) in along.iter().enumerate() {
        match *edges.as_slice() {
            [] => {}
            [first, second] => copy[second] = Copy::As(first),
            _ => {
                return Err(GeopError::new(format!(
                    "the walls are too thick: face {} collapses, and is tangent along {} of its edges to faces that stay, where a blend is along two: not supported",
                    names.faces[f],
                    edges.len()
                )));
            }
        }
    }
    Ok(copy)
}

/// `spec` — the faces on their inner surfaces — contracted where faces
/// collapse (see [`Contracted`]), the edges' copies `copy`, `kept` and
/// `names` per entity of `spec`.
fn contract(
    spec: &BodySpec,
    names: &BodyNames,
    kept: &[bool],
    gone: &[bool],
    copy: &[Copy],
) -> GeopResult<Contracted> {
    // Vertices joined by an edge that vanishes share their copy.
    let mut group: Vec<usize> = (0..spec.vertices.len()).collect();
    fn root(group: &mut [usize], v: usize) -> usize {
        let mut r = v;
        while group[r] != r {
            r = group[r];
        }
        group[v] = r;
        r
    }
    for (e, edge) in spec.edges.iter().enumerate() {
        if copy[e] == Copy::Vanishes {
            let (a, b) = (root(&mut group, edge.start), root(&mut group, edge.end));
            group[a.max(b)] = a.min(b);
        }
    }
    let mut out = BodySpec::new(spec.solid);
    let mut out_names = BodyNames {
        solid: names.solid.clone(),
        ..Default::default()
    };
    let mut index = vec![None; spec.vertices.len()];
    let mut vertex = Vec::with_capacity(spec.vertices.len());
    for v in 0..spec.vertices.len() {
        let r = root(&mut group, v);
        vertex.push(*index[r].get_or_insert_with(|| {
            out.vertices.push(spec.vertices[r]);
            out_names.vertices.push(names.vertices[r].clone());
            out.vertices.len() - 1
        }));
    }
    let mut edge = vec![None; spec.edges.len()];
    for (e, original) in spec.edges.iter().enumerate() {
        if copy[e] == Copy::Own {
            edge[e] = Some((out.edges.len(), Sense::Forward));
            out.edges.push(EdgeSpec {
                curve: original.curve.clone(),
                start: vertex[original.start],
                end: vertex[original.end],
            });
            out_names.edges.push(names.edges[e].clone());
        }
    }
    for (e, original) in spec.edges.iter().enumerate() {
        if let Copy::As(first) = copy[e] {
            let (i, _) = edge[first].expect("the first edge along a blend has its own copy");
            let ends = (vertex[original.start], vertex[original.end]);
            let sense = if ends == (out.edges[i].start, out.edges[i].end) {
                Sense::Forward
            } else if ends == (out.edges[i].end, out.edges[i].start) {
                Sense::Reversed
            } else {
                return Err(GeopError::new(format!(
                    "the walls are too thick: edges {} and {} along a collapsing blend do not end where each other's copies do: not supported",
                    names.edges[first], names.edges[e]
                )));
            };
            edge[e] = Some((i, sense));
        }
    }
    let mut face = vec![None; spec.faces.len()];
    let mut coedge = HashMap::new();
    let mut out_kept = Vec::new();
    for (f, original) in spec.faces.iter().enumerate() {
        if gone[f] {
            continue;
        }
        let mut loops = Vec::new();
        for (l, lp) in original.loops().into_iter().enumerate() {
            let mut copied = Vec::new();
            for (k, c) in lp.iter().enumerate() {
                let on = match c.on {
                    CoedgeOn::Edge(e, sense) => match edge[e] {
                        None => continue,
                        Some((i, along)) => CoedgeOn::Edge(
                            i,
                            if along == Sense::Forward {
                                sense
                            } else {
                                sense.opposite()
                            },
                        ),
                    },
                    CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertex[v]),
                };
                coedge.insert(At { f, l, k }, (l, copied.len()));
                copied.push(CoedgeSpec {
                    on,
                    pcurve: c.pcurve.clone(),
                });
            }
            if copied.is_empty() {
                return Err(GeopError::new(format!(
                    "the walls are too thick: every edge of a loop of face {} vanishes, and its inner copy with it",
                    names.faces[f]
                )));
            }
            loops.push(copied);
        }
        face[f] = Some(out.faces.len());
        let mut loops = loops.into_iter();
        out.faces.push(FaceSpec {
            surface: original.surface.clone(),
            outer: loops.next().expect("a face has an outer loop"),
            holes: loops.collect(),
        });
        out_names.faces.push(names.faces[f].clone());
        out_kept.push(kept[f]);
    }
    out.shells = spec
        .shells
        .iter()
        .map(|shell| shell.iter().filter_map(|&f| face[f]).collect())
        .collect();
    Ok(Contracted {
        spec: out,
        names: out_names,
        kept: out_kept,
        vertex,
        edge,
        face,
        coedge,
    })
}

/// Per face taken away, whether it is smoothly open: tangent along every
/// edge to the face kept there. Its walls do not stand on it, they end
/// where it was — each edge a free edge of the face kept, whose wall gets
/// an end face along it as a sheet's border does — and nothing is left of
/// it: the open top of a box whose top edges are rounded. A face taken
/// away tangent to faces kept along some edges and not others, or one
/// that meets another face taken away, is refused: its rim would end on a
/// smooth surface.
fn smoothly_open(spec: &BodySpec, names: &BodyNames, removed: &[bool]) -> GeopResult<Vec<bool>> {
    let mut tangent = vec![0usize; spec.faces.len()];
    let mut other = vec![0usize; spec.faces.len()];
    for (e, at) in spec.edge_uses().iter().enumerate() {
        let [a, b] = match *at.as_slice() {
            [a, b] if a.0 != b.0 => [a.0, b.0],
            _ => continue,
        };
        match (removed[a], removed[b]) {
            (false, false) => {}
            (true, true) => {
                other[a] += 1;
                other[b] += 1;
            }
            _ => {
                let gone = if removed[a] { a } else { b };
                if smooth_along(spec, &spec.edges[e], a, b)? {
                    tangent[gone] += 1;
                } else {
                    other[gone] += 1;
                }
            }
        }
    }
    (0..spec.faces.len())
        .map(|f| {
            if tangent[f] == 0 {
                return Ok(false);
            }
            if other[f] > 0 {
                return Err(GeopError::new(format!(
                    "face {} is taken away tangent to a face kept along {} of its edges, and not along {}: the walls cannot end on a smooth surface, which is not supported",
                    names.faces[f], tangent[f], other[f]
                )));
            }
            Ok(true)
        })
        .collect()
}

/// A coedge of the solid: face `f`, its loop `l` (0 the outer one), and
/// the `k`-th coedge of that.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct At {
    f: usize,
    l: usize,
    k: usize,
}

/// What is left of an edge between two faces taken away: a piece at its
/// start and one at its end, each only where a wall reaches it.
type Pieces<T> = (Option<T>, Option<T>);

/// Where the entities of the solid went in the result, by their index
/// there: each walled vertex and its inner copy, each edge that is not
/// open and its inner copy, and what is left of each open one.
struct Numbering {
    vertex: Vec<Option<usize>>,
    inner_vertex: Vec<Option<usize>>,
    edge: Vec<Option<usize>>,
    inner_edge: Vec<Option<usize>>,
    pieces: Vec<Pieces<usize>>,
}

/// The pieces in the order a coedge of `sense` comes across them.
fn in_order<T>(pieces: (T, T), sense: Sense) -> (T, T) {
    match sense {
        Sense::Forward => pieces,
        Sense::Reversed => (pieces.1, pieces.0),
    }
}

/// The solid described as [`BodySpec`], and what hollowing it moves where.
struct Hollow<'a> {
    spec: &'a BodySpec,
    names: &'a BodyNames,
    /// Per face: whether it is taken away.
    removed: Vec<bool>,
    /// Per vertex: whether a face kept meets it. Where none does, the inner
    /// copy of the vertex is the vertex itself, and nothing of the solid
    /// is left around it.
    walled: Vec<bool>,
    /// Per edge: whether both its faces are taken away.
    open_edge: Vec<bool>,
    /// Per edge: whether it is free, used by one face only — an edge of a
    /// sheet's border — or one of a face smoothly open: it gets a wall.
    free_edge: Vec<bool>,
    /// Per face: whether it is taken away smoothly open (see
    /// [`smoothly_open`]): nothing is left of it, and it has no copy.
    smooth: Vec<bool>,
    /// The inner side: the solid rebuilt with every face kept on its offset
    /// and every face taken away on its own surface, each with room around
    /// it — its vertices, edges and pcurves where the walled ones go — and
    /// contracted where faces collapse (see [`Contracted`]).
    inner: BodySpec,
    /// Where each entity of the solid's copy is in `inner`.
    copies: Contracted,
    /// Per open edge: what is left of it at its start and at its end, where
    /// those are walled.
    pieces: Vec<Pieces<Curve3>>,
}

impl<'a> Hollow<'a> {
    fn new(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        removed: Vec<bool>,
        thickness: &[Scalar],
    ) -> GeopResult<Self> {
        let mut walled = vec![false; spec.vertices.len()];
        let mut open_edge = vec![true; spec.edges.len()];
        let mut uses = vec![0usize; spec.edges.len()];
        for (f, face) in spec.faces.iter().enumerate() {
            for lp in face.loops() {
                for c in lp {
                    let (_, end) = spec.ends(c.on)?;
                    walled[end] |= !removed[f];
                    if let CoedgeOn::Edge(e, _) = c.on {
                        open_edge[e] &= removed[f];
                        uses[e] += 1;
                    }
                }
            }
        }
        // The inner side is the solid rebuilt with every face kept on its
        // offset, every face taken away on its own surface (see
        // [`rebuilt`]). One taken away cannot run smoothly into one kept:
        // the walls would have to end on a smooth surface.
        let kept: Vec<bool> = removed.iter().map(|&r| !r).collect();
        let smooth = smoothly_open(spec, names, &removed)?;
        let mut offsets = spec.clone();
        let mut collapsed = vec![false; spec.faces.len()];
        for (f, face) in offsets.faces.iter_mut().enumerate() {
            if kept[f] {
                // The offset of the face given room, not the offset given
                // room afterwards: a fitted offset's end span carried on is
                // a polynomial extrapolated, and its width with it.
                match room(face.surface.clone())?
                    .offset(thickness[f].neg())
                    .map_err(|e| e.with_context(format!("offsetting face {}", names.faces[f])))?
                {
                    Some(surface) => face.surface = surface,
                    None => collapsed[f] = true,
                }
            }
        }
        let copy = collapse_edges(spec, names, &collapsed)?;
        let gone: Vec<bool> = (0..spec.faces.len())
            .map(|f| collapsed[f] || smooth[f])
            .collect();
        let copies = contract(&offsets, names, &kept, &gone, &copy)?;
        let reseat = vec![false; copies.spec.vertices.len()];
        let inner = rebuilt(&copies.spec, &copies.names, &copies.kept, &reseat)
            .map_err(|e| e.with_context("building the walls' inner side"))?;
        let mut hollow = Self {
            spec,
            names,
            removed,
            walled,
            open_edge,
            // An edge of a face smoothly open is a free edge of the face kept
            // there: its wall ends in an end face along it.
            free_edge: (0..spec.edges.len())
                .map(|e| uses[e] == 1 || spec.edge_uses()[e].iter().any(|&(f, ..)| smooth[f]))
                .collect(),
            smooth,
            inner,
            copies,
            pieces: Vec::new(),
        };
        hollow.pieces = (0..spec.edges.len())
            .map(|e| hollow.pieces_of(e))
            .collect::<GeopResult<_>>()?;
        Ok(hollow)
    }

    /// The coedges of the solid, each with where it is.
    fn coedges(&self) -> impl Iterator<Item = (At, &'a CoedgeSpec)> + 'a {
        let spec = self.spec;
        spec.faces.iter().enumerate().flat_map(|(f, face)| {
            face.loops()
                .into_iter()
                .enumerate()
                .flat_map(move |(l, lp)| {
                    lp.iter().enumerate().map(move |(k, c)| (At { f, l, k }, c))
                })
        })
    }

    /// The coedge before `at` in its loop.
    fn previous(&self, at: At) -> At {
        let n = self.spec.faces[at.f].loops()[at.l].len();
        At {
            k: (at.k + n - 1) % n,
            ..at
        }
    }

    fn inner_point(&self, v: usize) -> Vector3 {
        self.inner.vertices[self.copies.vertex[v]]
    }

    /// The curve of the inner copy of edge `e`, run the way `e` runs, if it
    /// has one.
    fn inner_curve(&self, e: usize) -> Option<Curve3> {
        self.copies.edge[e].map(|(i, sense)| sense.orient(&self.inner.edges[i].curve))
    }

    /// What is left of the open edge `e` at its start and at its end: from
    /// a walled end to its inner copy, which lies on the edge.
    fn pieces_of(&self, e: usize) -> GeopResult<Pieces<Curve3>> {
        if !self.open_edge[e] {
            return Ok((None, None));
        }
        let edge = &self.spec.edges[e];
        let piece = |from: Vector3, to: Vector3| along(&edge.curve, from, to);
        let (s, t) = (edge.start, edge.end);
        Ok((
            self.walled[s]
                .then(|| piece(self.spec.vertices[s], self.inner_point(s)))
                .transpose()?,
            self.walled[t]
                .then(|| piece(self.inner_point(t), self.spec.vertices[t]))
                .transpose()?,
        ))
    }

    /// The inner copy of the coedge `at`, on its face's inner surface —
    /// its edge and vertex numbered as in `inner` — if it has one.
    fn inner_coedge(&self, at: At) -> Option<CoedgeSpec> {
        let face = self.copies.face[at.f]?;
        let &(l, k) = self.copies.coedge.get(&at)?;
        Some(self.inner.faces[face].loops()[l][k].clone())
    }

    /// Where the inner copy of the coedge `at` ends on its face's inner
    /// surface: of the last coedge up to it that has a copy.
    fn inner_end(&self, at: At) -> GeopResult<Vector2> {
        let mut here = at;
        loop {
            if let Some(c) = self.inner_coedge(here) {
                return Ok(c.pcurve.ends()?.1);
            }
            here = self.previous(here);
            if here == at {
                return Err(GeopError::new("no coedge of the loop has a copy"));
            }
        }
    }

    /// The hollowed solid, described whole, and the names of what it is
    /// made of.
    fn result(&self, namer: &Namer) -> GeopResult<(BodySpec, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec::new(true);
        let mut names = BodyNames {
            solid: Some(namer.root()),
            ..Default::default()
        };
        let inner_name = |name: &str| namer.name(&[name]);

        let mut at = Numbering {
            vertex: vec![None; spec.vertices.len()],
            inner_vertex: vec![None; self.inner.vertices.len()],
            edge: vec![None; spec.edges.len()],
            inner_edge: vec![None; self.inner.edges.len()],
            pieces: vec![(None, None); spec.edges.len()],
        };
        for v in 0..spec.vertices.len() {
            if self.walled[v] {
                at.vertex[v] = Some(out.vertices.len());
                out.vertices.push(spec.vertices[v]);
                names.vertices.push(self.names.vertices[v].clone());
            }
        }
        // The inner vertices, each once: vertices joined where a face
        // collapses share one, named after the first.
        for v in 0..spec.vertices.len() {
            let i = self.copies.vertex[v];
            if self.walled[v] && at.inner_vertex[i].is_none() {
                at.inner_vertex[i] = Some(out.vertices.len());
                out.vertices.push(self.inner.vertices[i]);
                names.vertices.push(inner_name(&self.names.vertices[v]));
            }
        }
        let inner_vertex_of = |v: usize| at.inner_vertex[self.copies.vertex[v]];
        let inner_vertex: Vec<Option<usize>> =
            (0..spec.vertices.len()).map(inner_vertex_of).collect();
        let vertex = &at.vertex;
        let mut push_edge = |curve: Curve3, start: usize, end: usize, name: String| {
            out.edges.push(EdgeSpec { curve, start, end });
            names.edges.push(name);
            Some(out.edges.len() - 1)
        };
        for (e, original) in spec.edges.iter().enumerate() {
            let name = &self.names.edges[e];
            let (s, t) = (original.start, original.end);
            if !self.open_edge[e] {
                at.edge[e] = push_edge(
                    original.curve.clone(),
                    vertex[s].unwrap(),
                    vertex[t].unwrap(),
                    name.clone(),
                );
                // Its inner copy, once for the edges sharing it.
                if let Some((i, Sense::Forward)) = self.copies.edge[e]
                    && at.inner_edge[i].is_none()
                {
                    let copy = &self.inner.edges[i];
                    at.inner_edge[i] = push_edge(
                        copy.curve.clone(),
                        at.inner_vertex[copy.start].unwrap(),
                        at.inner_vertex[copy.end].unwrap(),
                        inner_name(name),
                    );
                }
            }
            let (first, second) = &self.pieces[e];
            if let Some(curve) = first {
                at.pieces[e].0 = push_edge(
                    curve.clone(),
                    vertex[s].unwrap(),
                    inner_vertex[s].unwrap(),
                    namer.name(&[name, "start"]),
                );
            }
            if let Some(curve) = second {
                at.pieces[e].1 = push_edge(
                    curve.clone(),
                    inner_vertex[t].unwrap(),
                    vertex[t].unwrap(),
                    namer.name(&[name, "end"]),
                );
            }
        }
        let map = |on: CoedgeOn, edges: &[Option<usize>], vertices: &[Option<usize>]| match on {
            CoedgeOn::Edge(e, sense) => CoedgeOn::Edge(edges[e].unwrap(), sense),
            CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertices[v].unwrap()),
        };
        let inner_vertex = &inner_vertex;

        let mut outer_faces = Vec::new();
        let mut inner_faces = Vec::new();
        for (f, face) in spec.faces.iter().enumerate() {
            let name = &self.names.faces[f];
            if self.smooth[f] {
                continue;
            }
            if self.removed[f] {
                let rims = self.rims(f, &at)?;
                let several = rims.len() > 1;
                for (k, rim) in rims.into_iter().enumerate() {
                    outer_faces.push(out.faces.len());
                    out.faces.push(rim);
                    names.faces.push(if several {
                        namer.name(&[name, &k.to_string()])
                    } else {
                        namer.name(&[name])
                    });
                }
                continue;
            }
            let remap = |lp: &Vec<CoedgeSpec>| -> Vec<CoedgeSpec> {
                lp.iter()
                    .map(|c| CoedgeSpec {
                        on: map(c.on, &at.edge, &at.vertex),
                        pcurve: c.pcurve.clone(),
                    })
                    .collect()
            };
            outer_faces.push(out.faces.len());
            out.faces.push(FaceSpec {
                surface: face.surface.clone(),
                outer: remap(&face.outer),
                holes: face.holes.iter().map(remap).collect(),
            });
            names.faces.push(name.clone());

            let Some(copy) = self.copies.face[f] else {
                continue;
            };
            let copy = &self.inner.faces[copy];
            let inner_loop = |lp: &Vec<CoedgeSpec>| -> Vec<CoedgeSpec> {
                lp.iter()
                    .map(|c| CoedgeSpec {
                        on: map(c.on, &at.inner_edge, &at.inner_vertex),
                        pcurve: c.pcurve.clone(),
                    })
                    .collect()
            };
            let inner = FaceSpec {
                surface: copy.surface.clone(),
                outer: inner_loop(&copy.outer),
                holes: copy.holes.iter().map(inner_loop).collect(),
            };
            inner_faces.push(out.faces.len());
            out.faces.push(inner.reversed());
            names.faces.push(inner_name(name));
        }
        // Along every free edge, a wall from the edge to its inner copy,
        // with a straight edge from each of its vertices to that vertex's
        // inner copy, `side[v]`.
        let mut side: Vec<Option<usize>> = vec![None; spec.vertices.len()];
        let mut walls = Vec::new();
        for (here, c) in self.coedges() {
            let CoedgeOn::Edge(e, sense) = c.on else {
                continue;
            };
            if !self.free_edge[e] || self.removed[here.f] {
                continue;
            }
            let (x, y) = spec.ends(c.on)?;
            for v in [x, y] {
                if side[v].is_none() {
                    side[v] = Some(out.edges.len());
                    out.edges.push(EdgeSpec {
                        curve: NurbCurve3D::line(spec.vertices[v], self.inner_point(v))?,
                        start: at.vertex[v].unwrap(),
                        end: inner_vertex[v].unwrap(),
                    });
                    names
                        .edges
                        .push(namer.name(&[&self.names.vertices[v], "side"]));
                }
            }
            let outer = sense.orient(&spec.edges[e].curve);
            let inner = sense.orient(&self.inner_curve(e).expect("a free edge has a copy"));
            let (copy, along) = self.copies.edge[e].expect("a free edge has a copy");
            let copy_sense = if along == Sense::Forward {
                sense
            } else {
                sense.opposite()
            };
            // From the inner copy, at `v = 0`, out to the edge, at `v = 1`:
            // running round it counter-clockwise, the wall runs the edge
            // the other way than its face does, and faces away from it.
            let lp = [
                CoedgeOn::Edge(at.inner_edge[copy].unwrap(), copy_sense),
                CoedgeOn::Edge(side[y].unwrap(), Sense::Reversed),
                CoedgeOn::Edge(at.edge[e].unwrap(), sense.opposite()),
                CoedgeOn::Edge(side[x].unwrap(), Sense::Forward),
            ];
            walls.push(out.faces.len());
            let surface = NurbSurface3D::ruled(&inner, &outer).map_err(|err| {
                err.with_context(format!("the wall along edge {}", self.names.edges[e]))
            })?;
            out.faces.push(FaceSpec::on_unit_square(surface, lp)?);
            names
                .faces
                .push(namer.name(&[&self.names.edges[e], "side"]));
        }
        out.shells = if self.removed.iter().any(|&r| r) || !walls.is_empty() {
            vec![
                outer_faces
                    .into_iter()
                    .chain(inner_faces)
                    .chain(walls)
                    .collect(),
            ]
        } else {
            vec![outer_faces, inner_faces]
        };
        Ok((out, names))
    }

    /// The inner side alone, as a sheet of its own: the inner copy of every
    /// face, facing the way the face does, and of every edge and vertex,
    /// each copy of `X` named `N(X)` for `namer`'s `N` — of the first `X`
    /// where several share one.
    fn inner_side(&self, namer: &Namer) -> GeopResult<(BodySpec, BodyNames)> {
        let mut out = self.inner.clone();
        out.solid = false;
        out.shells = vec![(0..out.faces.len()).collect()];
        let names = BodyNames {
            vertices: self
                .copies
                .names
                .vertices
                .iter()
                .map(|n| namer.name(&[n]))
                .collect(),
            edges: self
                .copies
                .names
                .edges
                .iter()
                .map(|n| namer.name(&[n]))
                .collect(),
            faces: self
                .copies
                .names
                .faces
                .iter()
                .map(|n| namer.name(&[n]))
                .collect(),
            solid: None,
        };
        Ok((out, names))
    }

    /// What is left of the face `f`, taken away: `f` without its inner copy
    /// (see the module docs), as one face or several, the result's entities
    /// numbered `at`.
    fn rims(&self, f: usize, at: &Numbering) -> GeopResult<Vec<FaceSpec>> {
        let spec = self.spec;
        let vertex = &at.vertex;
        let inner_vertex: Vec<Option<usize>> = (0..spec.vertices.len())
            .map(|v| at.inner_vertex[self.copies.vertex[v]])
            .collect();
        let surface = &spec.faces[f].surface;
        let unsupported = |v: usize| {
            GeopError::new(format!(
                "face {} is taken away at vertex {}, where its surface comes to a point: not supported",
                self.names.faces[f], self.names.vertices[v]
            ))
        };
        // Every coedge of the rim, with the result's vertices it runs from
        // and to.
        let mut coedges: Vec<(CoedgeSpec, usize, usize)> = Vec::new();
        for (here, c) in self.coedges().filter(|(here, _)| here.f == f) {
            let CoedgeOn::Edge(e, sense) = c.on else {
                let CoedgeOn::Vertex(v) = c.on else {
                    unreachable!()
                };
                if self.walled[v] {
                    return Err(unsupported(v));
                }
                continue;
            };
            let (x, y) = spec.ends(c.on)?;
            if !self.open_edge[e] {
                // `f`'s own boundary where a wall stands on it...
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(at.edge[e].unwrap(), sense),
                        pcurve: c.pcurve.clone(),
                    },
                    vertex[x].unwrap(),
                    vertex[y].unwrap(),
                ));
                // ...and its inner copy, the other way, where it has one.
                if let Some(inner) = self.inner_coedge(here) {
                    let CoedgeOn::Edge(copy, copy_sense) = inner.on else {
                        unreachable!("the copy of an edge");
                    };
                    let (from, to) = self.inner.ends(inner.on)?;
                    coedges.push((
                        CoedgeSpec {
                            on: CoedgeOn::Edge(at.inner_edge[copy].unwrap(), copy_sense.opposite()),
                            pcurve: inner.pcurve.reverse(),
                        },
                        at.inner_vertex[to].unwrap(),
                        at.inner_vertex[from].unwrap(),
                    ));
                }
                continue;
            }
            // An edge to another face taken away: what is left of it, from
            // `x` to its inner copy and from `y`'s to `y`.
            let (orig_start, orig_end) = c.pcurve.ends()?;
            let (start_piece, end_piece) = in_order(at.pieces[e], sense);
            let curves = &self.pieces[e];
            let (start_curve, end_curve) = in_order((&curves.0, &curves.1), sense);
            if let (Some(piece), Some(curve)) = (start_piece, start_curve) {
                let to = self.inner_end(self.previous(here))?;
                let curve = sense.orient(curve);
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(piece, sense),
                        pcurve: pcurve(surface, &c.pcurve, Some(&curve), orig_start, to)?,
                    },
                    vertex[x].unwrap(),
                    inner_vertex[x].unwrap(),
                ));
            }
            if let (Some(piece), Some(curve)) = (end_piece, end_curve) {
                let from = self.inner_end(here)?;
                let curve = sense.orient(curve);
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(piece, sense),
                        pcurve: pcurve(surface, &c.pcurve, Some(&curve), from, orig_end)?,
                    },
                    inner_vertex[y].unwrap(),
                    vertex[y].unwrap(),
                ));
            }
        }

        // Follow the coedges end to end into loops.
        let mut starting: HashMap<usize, usize> = HashMap::new();
        for (i, (_, start, _)) in coedges.iter().enumerate() {
            if starting.insert(*start, i).is_some() {
                return Err(GeopError::new(format!(
                    "what is left of face {} touches itself at a vertex: not supported",
                    self.names.faces[f]
                )));
            }
        }
        let mut used = vec![false; coedges.len()];
        let mut loops: Vec<Vec<usize>> = Vec::new();
        for first in 0..coedges.len() {
            if used[first] {
                continue;
            }
            // On from each coedge to the one starting where it ends. One
            // running into a loop already followed runs round that loop
            // into itself.
            let around = walk(first, |&i| {
                let next = starting.get(&coedges[i].2).ok_or_else(|| {
                    GeopError::new(format!(
                        "what is left of face {} does not close up",
                        self.names.faces[f]
                    ))
                })?;
                Ok::<_, GeopError>(Some(*next))
            })?;
            if around.end != Walked::Closed {
                return Err(GeopError::new(format!(
                    "what is left of face {} runs into itself",
                    self.names.faces[f]
                )));
            }
            for &i in &around.nodes {
                used[i] = true;
            }
            loops.push(around.nodes);
        }

        // Outer loops run counter-clockwise in `(u, v)`, holes clockwise;
        // each hole lies in one outer loop.
        let mut outers: Vec<(Vec<usize>, Scalar)> = Vec::new();
        let mut holes: Vec<Vec<usize>> = Vec::new();
        for lp in loops {
            let area = loop_signed_area(lp.iter().map(|&i| &coedges[i].0.pcurve))?;
            if area.definitely_greater(Scalar::ZERO) {
                outers.push((lp, area));
            } else if area.definitely_less(Scalar::ZERO) {
                holes.push(lp);
            } else {
                return Err(GeopError::new(format!(
                    "cannot tell which way a loop of what is left of face {} runs",
                    self.names.faces[f]
                )));
            }
        }
        let mut faces: Vec<FaceSpec> = outers
            .iter()
            .map(|(lp, _)| FaceSpec {
                surface: surface.clone(),
                outer: lp.iter().map(|&i| coedges[i].0.clone()).collect(),
                holes: Vec::new(),
            })
            .collect();
        // A part of the rim may lie inside another's outer loop — the ring
        // around a hole inside the ring around the walls — so a hole is the
        // innermost one's: of the outer loops around it, the smallest. Which
        // one a hole lies in is asked of a point in the middle of one of its
        // pcurves, by ray casting against the outer loop's pcurves.
        for lp in &holes {
            let first = &coedges[lp[0]].0.pcurve;
            let point = first.evaluate(first.mid_parameter()?)?;
            let mut owner: Option<usize> = None;
            for (i, (outer, area)) in outers.iter().enumerate() {
                let pcurves: Vec<&Curve2> = outer.iter().map(|&k| &coedges[k].0.pcurve).collect();
                let inside = match pcurves_contain(
                    surface,
                    &pcurves,
                    point,
                    MAX_NODES,
                    min_subdivision_size(),
                    SEED,
                )? {
                    PointClassification::Inside => true,
                    PointClassification::Outside => false,
                    PointClassification::OnVertex | PointClassification::OnCoedge => {
                        return Err(GeopError::new(format!(
                            "a hole of what is left of face {} meets one of its parts' outer loops",
                            self.names.faces[f]
                        )));
                    }
                };
                if inside && owner.is_none_or(|o| area.definitely_less(outers[o].1)) {
                    owner = Some(i);
                }
            }
            let owner = owner.ok_or_else(|| {
                GeopError::new(format!(
                    "a hole of what is left of face {} lies in none of its parts",
                    self.names.faces[f]
                ))
            })?;
            faces[owner]
                .holes
                .push(lp.iter().map(|&i| coedges[i].0.clone()).collect());
        }
        Ok(faces)
    }
}

/// The first of the faces `faces`, indices into `surfaces`, whose surface
/// `curve` does not lie on all along, with a point that shows it: the
/// middle of the first stretch of the curve that is off the surface. `None`
/// if the curve could lie on all of them. A curve built for an edge between
/// moved faces is only right if it does.
///
/// Whether it lies on a surface is what [`curve_surface_overlaps`] finds:
/// a stretch of it on the surface runs between two points where the curve
/// or the surface ends, so all of the curve lies on it if one stretch runs
/// from the curve's start to its end.
pub fn surface_missed(
    curve: &NurbCurve3D,
    faces: impl IntoIterator<Item = usize>,
    surfaces: &[NurbSurface3D],
) -> GeopResult<Option<(usize, Vector3)>> {
    let (t0, t1) = curve.domain();
    for f in faces {
        let overlaps =
            curve_surface_overlaps(curve, &surfaces[f], MAX_NODES, min_subdivision_size())?;
        if overlaps
            .iter()
            .any(|o| o.start.t.could_be_equal(t0) && o.end.t.could_be_equal(t1))
        {
            continue;
        }
        // The first stretch off it: up to the first overlap, or after it.
        let (from, to) = match overlaps.first() {
            Some(o) if o.start.t.could_be_equal(t0) => (
                o.end.t.upper(),
                overlaps.get(1).map_or(t1, |n| n.start.t.lower()),
            ),
            Some(o) => (t0, o.start.t.lower()),
            None => (t0, t1),
        };
        let off = curve.evaluate(from.add(to).div(Scalar::TWO)?.sharpen())?;
        return Ok(Some((f, off)));
    }
    Ok(None)
}

/// The pcurve on `surface` of a coedge that runs along `curve` — or sits at
/// a vertex, without one — from `(u, v)` `start` to `end`. Exact where it
/// can be: on a surface affine in `(u, v)`, a plane, the curve mapped
/// ([`NurbSurface3D::affine_pcurve`]); and straight where the coedge it is
/// a copy of ran along an iso-line, `original`, and its copy does too —
/// the line keeps its `u` (or `v`) — as a cylinder's rim or seam does.
/// Fitted otherwise: a sphere's meridian cut by a plane is straight, but
/// its inner copy, where the offset plane cuts the offset sphere, is a
/// small circle no straight pcurve follows.
pub fn pcurve(
    surface: &NurbSurface3D,
    original: &Curve2,
    curve: Option<&NurbCurve3D>,
    start: Vector2,
    end: Vector2,
) -> GeopResult<Curve2> {
    let Some(curve) = curve else {
        return NurbCurve2D::line(start, end);
    };
    if let Some(mut mapped) = surface.affine_pcurve(curve)? {
        // Its ends and the junctions' `(u, v)` both enclose the ends' foot
        // points: so does what they share.
        let (a, b) = mapped.ends()?;
        if a.could_be_equal(&start) && b.could_be_equal(&end) {
            let last = mapped.control_points.len() - 1;
            for (k, at, given) in [(0, a, start), (last, b, end)] {
                let w = mapped.control_points[k][2];
                let pinned = |c: usize| {
                    let x = at[c].intersect(given[c]);
                    if w.is_exactly(Scalar::ONE) {
                        x
                    } else {
                        x.mul(w)
                    }
                };
                mapped.control_points[k] = Vector3::from_array([pinned(0), pinned(1), w]);
            }
            return Ok(mapped);
        }
    }
    let straight = original.degree == 1 && original.control_points.len() == 2 && {
        let (a, b) = original.ends()?;
        let keeps = |k: usize| a[k].could_be_equal(b[k]) && start[k].could_be_equal(end[k]);
        keeps(0) || keeps(1)
    };
    if straight {
        return NurbCurve2D::line(start, end);
    }
    surface.fit_pcurve(
        curve,
        Some(start),
        Some(end),
        MAX_NODES,
        min_subdivision_size(),
    )
}

/// `surface` over three times its domain in each direction it is straight
/// in — degree 1, two rows of control points of equal weights — extended
/// linearly: the same points at the same `(u, v)`, and more around them.
/// It is not extended past a row that is a single point, a pole like a
/// disc's center: the pole would end up inside the patch, where it has no
/// normal, and the inner copy of a vertex there is that pole.
///
/// A face's inner copy can reach past the face where they meet at an
/// inward corner — or, for a plane cut off by the box around the profile,
/// past where the patch ends — so its surface needs room around the face.
/// How much is a free choice; the face's own size is room enough unless
/// the walls are as thick as the face is wide.
pub fn extended(mut surface: NurbSurface3D) -> GeopResult<NurbSurface3D> {
    for along_u in [true, false] {
        let (degree, num, other) = if along_u {
            (surface.degree_u, surface.num_u, surface.num_v)
        } else {
            (surface.degree_v, surface.num_v, surface.num_u)
        };
        if degree != 1 || num != 2 {
            continue;
        }
        let index = |row: usize, j: usize| {
            if along_u {
                row * surface.num_v + j
            } else {
                j * surface.num_v + row
            }
        };
        let cps = &surface.control_points;
        if !(0..other).all(|j| cps[index(0, j)][3].could_be_equal(cps[index(1, j)][3])) {
            continue;
        }
        let pole = |row: usize| -> GeopResult<bool> {
            let first = cps[index(row, 0)].dehomogenized()?;
            for j in 1..other {
                if !cps[index(row, j)].dehomogenized()?.could_be_equal(&first) {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        let (start, end) = (!pole(0)?, !pole(1)?);
        if !start && !end {
            continue;
        }
        let mut control_points = cps.clone();
        for j in 0..other {
            let (a, b) = (cps[index(0, j)], cps[index(1, j)]);
            let two = |p: &Vector<4>| p.prod_scalar(Scalar::TWO);
            if start {
                control_points[index(0, j)] = two(&a).sub(&b);
            }
            if end {
                control_points[index(1, j)] = two(&b).sub(&a);
            }
        }
        let knots = if along_u {
            &surface.knot_vector_u
        } else {
            &surface.knot_vector_v
        };
        let (lo, hi) = (knots[1], knots[2]);
        let width = hi.sub(lo);
        let lo = if start { lo.sub(width) } else { lo };
        let hi = if end { hi.add(width) } else { hi };
        let knots = vec![lo, lo, hi, hi];
        surface = if along_u {
            NurbSurface3D::try_new(
                1,
                surface.degree_v,
                control_points,
                knots,
                surface.knot_vector_v.clone(),
            )?
        } else {
            NurbSurface3D::try_new(
                surface.degree_u,
                1,
                control_points,
                surface.knot_vector_u.clone(),
                knots,
            )?
        };
    }
    Ok(surface)
}

#[cfg(test)]
mod tests;
