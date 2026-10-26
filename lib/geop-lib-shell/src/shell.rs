//! [`shell`]: a solid hollowed out to walls of one thickness, open where
//! faces are taken away.
//!
//! The hollow is built directly, not cut out by a boolean: the walls'
//! inner side is the solid's own boundary moved inward, face for face, so
//! it has exactly the solid's topology, and everything about how the inner
//! and the outer side fit together is known without intersecting
//! anything.
//!
//! **The inner side.** Every face kept is offset inward by the thickness
//! (see [`NurbSurface3D::offset`]: exact for planes and revolutions, fitted
//! and enclosed for any other), every face taken away stays where it is:
//! its inner copy is what the opening leaves of it. A vertex moves to where
//! the offsets of the faces around it meet — the point at the thickness
//! from each face, found on the faces themselves — an edge to where the offsets of
//! its two faces meet — a line from one moved vertex to the other for a
//! straight edge, an arc around the same axis for a circular one, and for
//! any other the branch where the two offsets cross between the moved
//! vertices, found at the edge's own stations and fitted through them. A
//! vertex on the axis of a revolved face, at its pole, moves to the pole of
//! the face's inner copy: for a cone's apex, the offset cone's apex.
//!
//! **Blends narrower than the walls.** A face whose offset collapses —
//! every point of it past its centre of curvature, a blend of a radius
//! less than the thickness — has no inner copy. Its edges along the faces
//! it rounds, where it is tangent to them, share one copy: where those
//! faces' offsets meet, a sharp inner corner. Its edges across it vanish,
//! and the copies of their ends are one point, where the faces around them
//! meet. A corner patch of such blends, whose offset cannot be found at
//! its pole, collapses with them, once the walls' corner is found to be as
//! far from it as the walls are thick (see `collapse_edges`).
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
//! **The result.** The faces kept, their inner copies turned around, and
//! the rims, as one solid. Without an opening, the inner side is a void of
//! its own: a second shell.
//!
//! **A sheet** — faces standing on their own — is shelled the same way
//! ([`thicken`]): its faces are the outer side, their inner copies the
//! inner one, and along every free edge, used by one face only, a wall
//! joins the edge to its inner copy — the ruled surface between them, with
//! a straight edge from each of its vertices to that vertex's inner copy.
//! A free edge's copy, like a seam's, is the edge carried along its face's
//! normal. The inner side alone is the sheet offset ([`offset_faces`]).
//!
//! What it does not do (yet): take away part of a smooth surface — a face
//! tangent to one that stays — take away
//! a face that meets one kept at an inward corner — a pocket's floor, a
//! step, the face a boss stands on: the wall kept there is thickened away
//! from the face, so the inner copy reaches past it and the opening is no
//! longer the face less its inner copy — or open a solid that already has
//! a void. Walls so thick that an edge's inner copy turns around, or that
//! an edge of the result crosses one of its faces, are refused; two faces
//! crossing along a curve no edge reaches are not noticed.

use std::collections::HashMap;

use geop_core_geometry::{
    intersection::{
        curve_surface_overlaps,
        surface_surface::{OnBoth, fit_branch, predictor_corrector_step},
    },
    meet::{Offset, meet, tangent_groups},
    nurb_curve::{NurbCurve2D, NurbCurve3D, loop_signed_area},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
    shape::{Arc, Circle},
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
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec, FacesAround},
    contains::face::{PointClassification, pcurves_contain},
    validation::{ValidationParameters, body_crossing},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

/// Seed of the ray casting that finds which outer loop a hole lies in:
/// any gives the same answer (see `pcurves_contain`), a constant keeps it
/// reproducible.
const SEED: u64 = 0x5E11_0000_0000_0001;

/// Hollows the solid `solid` of `part` out to walls `thickness` thick,
/// open where its faces `open` were — see the module docs — and returns
/// the hollowed solid, which replaces it and is named `namer`'s root.
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
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "shell({}, {solid}, open={open:?}, {thickness:?})",
        namer.root()
    );
    if !thickness.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "a shell's walls need a thickness greater than zero, not {thickness:?}"
        )))
        .with_context(ctx);
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
    let (spec, sources) = model.body_spec(&faces, true).with_context(ctx)?;
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed: Vec<bool> = sources.faces.iter().map(|f| open.contains(f)).collect();
    if let Some((f, g, e)) = inward_corner(&spec, &removed).with_context(ctx)? {
        return Err(GeopError::new(format!(
            "face {} is taken away where it meets face {} at an inward corner, along edge {}: its opening would reach past it, which is not supported",
            names.faces[f], names.faces[g], names.edges[e]
        )))
        .with_context(ctx);
    }

    let hollow = Hollow::new(&spec, &names, removed, thickness).with_context(ctx)?;
    let (result, result_names) = hollow.result(namer).with_context(ctx)?;
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
    let hollow = Hollow::new(&spec, &names, removed, thickness).with_context(ctx)?;
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
    let hollow = Hollow::new(&spec, &names, removed, distance.neg()).with_context(ctx)?;
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

/// What the inner copy of each edge is, given which faces collapse, and
/// the faces each copy lies on (see [`InnerEdge`], `Hollow::between`).
///
/// An edge of a collapsed face vanishes, unless the face is tangent along
/// it to a face that does not collapse: a blend narrower than the walls
/// are thick leaves the two faces it rounds meeting in a sharp inner
/// corner, along it, and across it — where it meets a face square to it,
/// or another collapsed blend — nothing. Which is asked of the faces'
/// normals halfway along the edge. Of the two edges along it, the second
/// takes the first's copy, which lies where the rounded faces' offsets
/// meet. A collapsed face with other than two such edges, or with a free
/// edge or a seam, is refused: what would take its place is not that
/// corner.
fn collapse_edges(
    spec: &BodySpec,
    names: &BodyNames,
    collapsed: &[bool],
    surfaces: &[NurbSurface3D],
) -> GeopResult<(Vec<InnerEdge>, Vec<Vec<usize>>)> {
    let uses = spec.edge_uses();
    let mut copy = vec![InnerEdge::Own; spec.edges.len()];
    let mut between: Vec<Vec<usize>> = uses
        .iter()
        .map(|u| {
            let mut faces: Vec<usize> = u.iter().map(|&(f, ..)| f).collect();
            faces.dedup();
            faces
        })
        .collect();
    let mut along: Vec<Vec<usize>> = vec![Vec::new(); spec.faces.len()];
    for (e, at) in uses.iter().enumerate() {
        let [(f, _, f_pcurve), (g, _, g_pcurve)] = match *at.as_slice() {
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
        if !collapsed[f] && !collapsed[g] {
            continue;
        }
        if collapsed[f] && collapsed[g] {
            copy[e] = InnerEdge::Vanishes;
            continue;
        }
        let (kept, gone) = if collapsed[f] { (g, f) } else { (f, g) };
        // Tangent along the edge: the normals halfway along it parallel.
        let curve = &spec.edges[e].curve;
        let point = curve.evaluate(curve.mid_parameter()?)?;
        let normal = |face: usize, pcurve: &Curve2| -> GeopResult<Vector3> {
            let seed = pcurve.evaluate(pcurve.mid_parameter()?)?;
            let (u, v) = surfaces[face].project(
                point,
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            surfaces[face].normal(u, v)
        };
        let (n_f, n_g) = (normal(f, f_pcurve)?, normal(g, g_pcurve)?);
        if n_f.prod_cross(&n_g).norm_sq().could_be_equal(Scalar::ZERO) {
            along[gone].push(e);
            between[e] = vec![kept];
        } else {
            copy[e] = InnerEdge::Vanishes;
        }
    }
    for (f, edges) in along.iter().enumerate() {
        match *edges.as_slice() {
            [] => {}
            [first, second] => {
                let (kept_first, kept_second) = (between[first][0], between[second][0]);
                // Which way the second runs along the first is told by
                // their ends once the vanishing edges have joined them.
                copy[second] = InnerEdge::As(first, Sense::Forward);
                between[first] = vec![kept_first, kept_second];
                between[second] = vec![kept_first, kept_second];
            }
            _ => {
                return Err(GeopError::new(format!(
                    "the walls are too thick: face {} collapses, and is tangent along {} of its edges to faces that stay, where a blend is along two: not supported",
                    names.faces[f],
                    edges.len()
                )));
            }
        }
    }
    Ok((copy, between))
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

/// What the inner copy of an edge is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InnerEdge {
    /// A copy of its own (none for an edge between faces taken away).
    Own,
    /// None: the edge is of a collapsed face, across it — where a blend
    /// meets a face square to it, or another collapsed one — and its ends'
    /// copies are one point.
    Vanishes,
    /// The copy of the edge given, run the way given: of a collapsed
    /// face's two edges along it, where it is tangent to the faces it
    /// rounds, the second shares the first's copy — where those faces'
    /// offsets meet.
    As(usize, Sense),
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
    /// sheet's border, which gets a wall.
    free_edge: Vec<bool>,
    /// Per vertex, each face around it once, with where the vertex is on
    /// the face's surface.
    around: Vec<FacesAround>,
    /// Per face: whether it is kept, but its offset collapses — a blend
    /// narrower than the walls are thick — so that it has no inner copy.
    collapsed: Vec<bool>,
    /// Per vertex: the vertex whose inner copy it shares — itself, unless
    /// an edge of a collapsed face between them vanishes (see
    /// [`InnerEdge::Vanishes`]).
    group: Vec<usize>,
    /// Per edge: what its inner copy is.
    copy: Vec<InnerEdge>,
    /// Per edge: the faces whose inner surfaces its copy lies on — its own,
    /// but for the copy that stands for a collapsed face's two tangent
    /// edges, which lies where its neighbours' meet.
    between: Vec<Vec<usize>>,
    /// Per face: the surface its inner copy lies on — for a face taken
    /// away, its own — with room around the face (see [`extended`]). A
    /// collapsed face has none, and keeps its own here.
    inner_surfaces: Vec<NurbSurface3D>,
    /// Per face: how far its inner surface is from it along its normal —
    /// against it, by the thickness, for a face kept; nothing for a face
    /// taken away.
    offsets: Vec<Scalar>,
    /// Per vertex: its inner copy, where it is walled.
    inner_points: Vec<Option<Vector3>>,
    /// Per edge: its inner copy's curve, unless it is open.
    inner_curves: Vec<Option<Curve3>>,
    /// Per open edge: what is left of it at its start and at its end, where
    /// those are walled.
    pieces: Vec<Pieces<Curve3>>,
    /// Per coedge, the `(u, v)` where it ends on its face's inner surface —
    /// where it ends at a walled vertex.
    inner_ends: HashMap<At, Vector2>,
}

impl<'a> Hollow<'a> {
    fn new(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        removed: Vec<bool>,
        thickness: Scalar,
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
        // Per vertex, each face around it once, with where the vertex is on
        // the face's surface.
        let around = spec.vertex_faces()?;

        let face_offsets: Vec<Scalar> = removed
            .iter()
            .map(|&r| if r { Scalar::ZERO } else { thickness.neg() })
            .collect();
        let mut collapsed = vec![false; spec.faces.len()];
        let offsets = spec
            .faces
            .iter()
            .enumerate()
            .map(|(f, face)| {
                if removed[f] {
                    return Ok(Some(face.surface.clone()));
                }
                face.surface
                    .offset(thickness.neg())
                    .map_err(|e| e.with_context(format!("offsetting face {}", names.faces[f])))
            })
            .collect::<Vec<_>>();
        // A face whose offset is no surface — a corner patch of blends,
        // its pole leaving no normal to offset along — collapses with the
        // blends around it, if every one of them does: a ball rolling round
        // a corner has the blends' radius. That it really has no copy is
        // checked once the corner's copy is known (`collapsed_by`).
        let mut collapsed_by_neighbours = vec![false; spec.faces.len()];
        for (f, offset) in offsets.iter().enumerate() {
            collapsed[f] = matches!(offset, Ok(None));
        }
        let edge_uses = spec.edge_uses();
        for (f, offset) in offsets.iter().enumerate() {
            if offset.is_ok() {
                continue;
            }
            let mut neighbours = edge_uses
                .iter()
                .filter(|at| at.iter().any(|&(g, ..)| g == f))
                .flat_map(|at| at.iter().map(|&(g, ..)| g).filter(|&g| g != f))
                .peekable();
            if neighbours.peek().is_some() && neighbours.all(|g| collapsed[g]) {
                collapsed_by_neighbours[f] = true;
            }
        }
        let inner_surfaces = spec
            .faces
            .iter()
            .zip(offsets)
            .enumerate()
            .map(|(f, (face, offset))| match offset {
                Ok(Some(surface)) => extended(surface),
                Ok(None) => extended(face.surface.clone()),
                Err(_) if collapsed_by_neighbours[f] => {
                    collapsed[f] = true;
                    extended(face.surface.clone())
                }
                Err(e) => Err(e),
            })
            .collect::<GeopResult<Vec<_>>>()?;

        let roomy = spec
            .faces
            .iter()
            .map(|face| extended(face.surface.clone()))
            .collect::<GeopResult<Vec<_>>>()?;
        let (mut copy, between) = collapse_edges(spec, names, &collapsed, &roomy)?;
        // Vertices joined by an edge that vanishes share their inner copy.
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
            if copy[e] == InnerEdge::Vanishes {
                let (a, b) = (root(&mut group, edge.start), root(&mut group, edge.end));
                group[a.max(b)] = a.min(b);
            }
        }
        for v in 0..group.len() {
            group[v] = root(&mut group, v);
        }
        // A copy shared runs the way its ends say: an edge runs along the
        // first edge's copy if its ends' copies are the first's, in order.
        for (e, b) in spec.edges.iter().enumerate() {
            if let InnerEdge::As(first, _) = copy[e] {
                let a = &spec.edges[first];
                let ends = |edge: &EdgeSpec| (group[edge.start], group[edge.end]);
                let sense = if ends(b) == ends(a) {
                    Sense::Forward
                } else if ends(b) == (ends(a).1, ends(a).0) {
                    Sense::Reversed
                } else {
                    return Err(GeopError::new(format!(
                        "the walls are too thick: edges {} and {} of a collapsing blend do not end where each other's copies do: not supported",
                        names.edges[first], names.edges[e]
                    )));
                };
                copy[e] = InnerEdge::As(first, sense);
            }
        }
        // Each group's faces: those around its vertices, but the collapsed.
        let mut group_faces: Vec<FacesAround> = vec![Vec::new(); spec.vertices.len()];
        for v in 0..spec.vertices.len() {
            for &(f, uv) in &around[v] {
                let faces = &mut group_faces[group[v]];
                if !collapsed[f] && !faces.iter().any(|&(g, _)| g == f) {
                    faces.push((f, uv));
                }
            }
        }
        let group_points = (0..spec.vertices.len())
            .map(|v| {
                if !walled[v] || group[v] != v {
                    return Ok(None);
                }
                let around = &group_faces;
                // Where the inner surfaces around it meet, nearest to it.
                // Surfaces tangent to each other there are pieces of one
                // smooth surface — the quarters of a revolved face meeting
                // at its pole — and so one condition. One whose face is
                // taken away cannot be tangent to one whose face is kept:
                // the walls would have to end on a smooth surface.
                if around[v].is_empty() {
                    return Err(GeopError::new(format!(
                        "the walls are too thick: every face at vertex {} collapses",
                        names.vertices[v]
                    )));
                }
                let pieces: Vec<_> = around[v]
                    .iter()
                    .map(|&(f, uv)| (&inner_surfaces[f], (uv[0], uv[1])))
                    .collect();
                // A vertex at a pole of its faces — on the axis of a
                // revolved face — moves to the pole of their inner copies:
                // the topology puts it there. A cone's apex has no normal,
                // and the nearest points of the inner cone to it are a
                // ring round the axis, not its apex.
                let poles: Vec<_> = pieces.iter().map(|(s, (u, v))| s.pole_at(*u, *v)).collect();
                if let Some(pole) = poles.iter().flatten().next() {
                    let removed_at = |i: usize| removed[around[v][i].0];
                    if (0..pieces.len()).any(|i| removed_at(i) != removed_at(0)) {
                        return Err(GeopError::new(format!(
                            "at vertex {}, faces taken away and faces kept meet at a pole: opening part of a revolved face round its axis is not supported",
                            names.vertices[v]
                        )));
                    }
                    if !poles.iter().all(|p| p.is_some_and(|p| p.could_be_equal(pole))) {
                        return Err(GeopError::new(format!(
                            "vertex {} is at a pole of some of its faces' inner copies and not of others, or of poles apart: {poles:?}",
                            names.vertices[v]
                        )));
                    }
                    return Ok(Some(*pole));
                }
                // The conditions are on the faces themselves, each at its
                // offset: exact, where an inner surface fitted to a
                // free-form face is only as tight as its fit, and where a
                // foot point on the face of a vertex of it is the vertex.
                // With room around each face, as its inner copy has: at an
                // inward corner the copy's foot point lies past the face.
                let outer: Vec<_> = around[v]
                    .iter()
                    .map(|&(f, uv)| (&roomy[f], (uv[0], uv[1])))
                    .collect();
                let mut offsets = Vec::new();
                for group in tangent_groups(&outer)? {
                    let f = |i: usize| around[v][i].0;
                    if group.iter().any(|&i| removed[f(i)] != removed[f(group[0])]) {
                        return Err(GeopError::new(format!(
                            "at vertex {}, a face taken away is tangent to one kept: the walls cannot end on a smooth surface",
                            names.vertices[v]
                        )));
                    }
                    offsets.push(Offset {
                        pieces: group.iter().map(|&i| outer[i]).collect(),
                        offset: face_offsets[f(group[0])],
                    });
                }
                meet(spec.vertices[v], &mut offsets, &[])
                    .map(Some)
                    .map_err(|e| {
                        e.with_context(format!("moving vertex {} inward", names.vertices[v]))
                    })
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let inner_points: Vec<Option<Vector3>> = (0..spec.vertices.len())
            .map(|v| {
                if walled[v] {
                    group_points[group[v]]
                } else {
                    None
                }
            })
            .collect();
        // A face taken to collapse with the blends around it has no copy
        // only if the walls' corner its vertices move to is as far from it
        // as the walls are thick: nearer, the face would still reach into
        // the hollow, and its copy would be there.
        for (f, &by_neighbours) in collapsed_by_neighbours.iter().enumerate() {
            if !by_neighbours {
                continue;
            }
            for &(v, uv) in around
                .iter()
                .enumerate()
                .flat_map(|(v, faces)| {
                    faces
                        .iter()
                        .filter(|&&(g, _)| g == f)
                        .map(move |&(_, uv)| (v, uv))
                })
                .collect::<Vec<_>>()
                .iter()
            {
                let Some(point) = inner_points[v] else {
                    continue;
                };
                let surface = &spec.faces[f].surface;
                let (u, w) = surface.project_in_domain(
                    point,
                    uv[0].sharpen(),
                    uv[1].sharpen(),
                    PROJECT_ITERATIONS,
                )?;
                let distance = surface.evaluate(u, w)?.sub(&point).norm();
                if distance.could_be_less(thickness) {
                    return Err(GeopError::new(format!(
                        "the walls are too thick: the blends round face {} collapse, and it would not: its offset could not be found, and the walls' corner at vertex {} is {distance:?} from it, nearer than the thickness",
                        names.faces[f], names.vertices[v]
                    )));
                }
            }
        }

        // Walls thicker than a face is wide turn its inner copy inside out,
        // and walls as thick as an edge is long take all of its inner copy:
        // a straight edge's inner copy then runs the other way, or is a point.
        for (e, edge) in spec.edges.iter().enumerate() {
            let (Some(start), Some(end)) = (&inner_points[edge.start], &inner_points[edge.end])
            else {
                continue;
            };
            let along = spec.vertices[edge.end].sub(&spec.vertices[edge.start]);
            if copy[e] == InnerEdge::Own
                && edge.curve.as_line()?.is_some()
                && !end
                    .sub(start)
                    .prod_dot(&along)
                    .definitely_greater(Scalar::ZERO)
            {
                return Err(GeopError::new(format!(
                    "the walls are too thick: the inner copy of edge {} has no length or turns around",
                    names.edges[e]
                )));
            }
        }

        let mut hollow = Self {
            spec,
            names,
            removed,
            walled,
            open_edge,
            free_edge: uses.iter().map(|&n| n == 1).collect(),
            around,
            collapsed,
            group,
            copy,
            between,
            inner_surfaces,
            offsets: face_offsets,
            inner_points,
            inner_curves: Vec::new(),
            pieces: Vec::new(),
            inner_ends: HashMap::new(),
        };
        hollow.inner_curves = (0..spec.edges.len())
            .map(|e| hollow.inner_curve(e))
            .collect::<GeopResult<_>>()?;
        hollow.pieces = (0..spec.edges.len())
            .map(|e| hollow.pieces_of(e))
            .collect::<GeopResult<_>>()?;
        hollow.inner_ends = hollow.inner_ends()?;
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

    fn coedge(&self, at: At) -> &'a CoedgeSpec {
        &self.spec.faces[at.f].loops()[at.l][at.k]
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
        self.inner_points[v].expect("a walled vertex has an inner copy")
    }

    /// Where vertex `v` is on face `f`'s surface: at it, or at another
    /// vertex sharing its inner copy.
    fn uv_at(&self, f: usize, v: usize) -> GeopResult<Vector2> {
        (0..self.group.len())
            .filter(|&w| self.group[w] == self.group[v])
            .flat_map(|w| self.around[w].iter())
            .find(|&&(g, _)| g == f)
            .map(|&(_, uv)| uv)
            .ok_or_else(|| {
                GeopError::new(format!(
                    "face {} is not at vertex {}",
                    self.names.faces[f], self.names.vertices[v]
                ))
            })
    }

    /// The curve of the inner copy of edge `e`, unless it is open — then it
    /// lies on the edge itself, and is no part of the result — or has no
    /// copy of its own (see [`InnerEdge`]).
    fn inner_curve(&self, e: usize) -> GeopResult<Option<Curve3>> {
        if self.open_edge[e] || self.copy[e] != InnerEdge::Own {
            return Ok(None);
        }
        let edge = &self.spec.edges[e];
        let ctx = |err: GeopError| {
            err.with_context(format!("the inner copy of edge {}", self.names.edges[e]))
        };
        let (start, end) = (self.inner_point(edge.start), self.inner_point(edge.end));
        let faces = self.between[e].clone();
        // A copy shaped like the edge is exact where it is right: a line
        // where two planes' offsets meet, an arc round a revolution's
        // axis. Whether it is right is asked, not assumed — a straight
        // edge of a bump is no straight line once carried along its
        // normal — and where it is not, the copy is found as any other.
        if let Some(curve) = shaped_like(
            &edge.curve,
            start,
            end,
            self.group[edge.start] == self.group[edge.end],
        )
        .transpose()
        .map_err(ctx)?
            && surface_missed(&curve, faces.iter().copied(), &self.inner_surfaces)
                .map_err(ctx)?
                .is_none()
        {
            return Ok(Some(curve));
        }
        let curve = self.traced_inner_curve(e, start, end).map_err(ctx)?;
        // It is only right if it lies on both inner surfaces: say so if not,
        // rather than build an edge off its faces.
        if let Some((f, middle)) =
            surface_missed(&curve, faces, &self.inner_surfaces).map_err(ctx)?
        {
            return Err(ctx(GeopError::new(format!(
                "its curve from {start:?} to {end:?} is not on the offset of face {} at {middle:?}: offsetting this kind of edge is not supported",
                self.names.faces[f]
            ))));
        }
        Ok(Some(curve))
    }

    /// What is left of the open edge `e` at its start and at its end: from
    /// a walled end to its inner copy, which lies on the edge.
    fn pieces_of(&self, e: usize) -> GeopResult<Pieces<Curve3>> {
        if !self.open_edge[e] {
            return Ok((None, None));
        }
        // Both faces stay where they are, so the inner copy of each walled
        // end lies on the edge itself, and what is left is the edge's own
        // curve from the end to it.
        let edge = &self.spec.edges[e];
        let ctx = |err: GeopError| {
            err.with_context(format!("what is left of edge {}", self.names.edges[e]))
        };
        let (s, t) = (edge.start, edge.end);
        let (t0, t1) = edge.curve.domain();
        let at = |point: Vector3, seed: Scalar| parameter_at(&edge.curve, point, seed).map_err(ctx);
        Ok((
            self.walled[s]
                .then(|| edge.curve.sub_curve(t0, at(self.inner_point(s), t0)?))
                .transpose()?,
            self.walled[t]
                .then(|| edge.curve.sub_curve(at(self.inner_point(t), t1)?, t1))
                .transpose()?,
        ))
    }

    /// The inner copy of edge `e` of face `f` alone — free, or a seam where
    /// `f` meets itself — from `start` to `end`: each point of the edge carried onto
    /// `f`'s inner surface. An offset keeps `(u, v)`, so a point of the
    /// edge is carried to the inner surface at its foot point on the face.
    /// Carried as the point moved along the face's normal there, not as the
    /// inner surface at the foot point: the foot point is as wide as the
    /// edge, and the surface evaluated over it spreads that width over
    /// every coordinate, so each copy of a copy came out wider. Moving the
    /// point keeps its own width, and the normal hardly varies over the
    /// foot point's box. The point lies on the face, so the true copy is
    /// the point plus the offset times a normal in that box.
    ///
    /// Carried from the edge's own curve, not the pcurve: the pcurve of a
    /// traced edge is as wide as its fit to the curve, and evaluating the
    /// surface along it spreads that width over the copy. The copy is
    /// fitted through the edge's breakpoints carried, enclosing it between
    /// them too (see [`NurbCurve3D::fit_enclosing`]).
    fn lifted_inner_curve(
        &self,
        e: usize,
        f: usize,
        start: Vector3,
        end: Vector3,
    ) -> GeopResult<Curve3> {
        let edge = &self.spec.edges[e];
        let curve = &edge.curve;
        let (outer, offset) = (&self.spec.faces[f].surface, self.offsets[f]);
        let seed = self.uv_at(f, edge.start)?;
        // The point of the edge at `t`, carried, and its foot point.
        let carried = |t: Scalar, seed: Vector2| -> GeopResult<(Vector3, Vector2)> {
            let point = curve.evaluate(t)?;
            let (u, v) = outer.project(
                point,
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            let normal = outer.normal(u, v)?;
            Ok((
                point.add(&normal.prod_scalar(offset)),
                Vector2::from_array([u, v]),
            ))
        };
        let params = curve.breakpoints();
        let last = params.len() - 1;
        let mut stations = Vec::with_capacity(params.len());
        let mut foot = seed;
        for (i, &t) in params.iter().enumerate() {
            let (point, uv) = carried(t, foot)?;
            foot = uv;
            // The moved vertices are where the copy starts and ends: on the
            // edge's ends carried, or the vertex and the face disagree.
            let point = match i {
                _ if i == 0 || i == last => {
                    let vertex = if i == 0 { start } else { end };
                    if !point.could_be_equal(&vertex) {
                        return Err(GeopError::new(format!(
                            "the copy of a vertex, {vertex:?}, is not where the edge's end is carried onto the face's inner surface, {point:?}"
                        )));
                    }
                    vertex
                }
                _ => point,
            };
            stations.push((point, (t, uv)));
        }
        let mut widest = None;
        NurbCurve3D::fit_enclosing(
            &stations,
            |&(_, (from, uv)), &(_, (to, _)), frac| {
                let t = from.add(to.sub(from).mul(frac)).sharpen();
                let (point, uv) = carried(t, uv)?;
                Ok((point, (t, uv)))
            },
            &mut widest,
            &|err| err,
        )
        .map_err(|err| {
            err.with_context(format!(
                "carrying the edge onto the face's inner surface; the widest point between stations: {widest:?}"
            ))
        })
    }

    /// The inner copy of edge `e`, from `start` to `end`, where shaping it
    /// like the edge does not say what it is: the branch where the inner
    /// surfaces of the edge's two faces meet, between the moved vertices.
    ///
    /// The branch is found where the edge is, not marched to: each station
    /// of the edge — its breakpoints and the middles of its spans — is
    /// carried onto both inner surfaces at once on the plane through it
    /// square to the edge, by the corrector the booleans trace with. Which
    /// branch is the copy is then a question the topology answers: it is
    /// the one through the moved vertices at both ends, and between them
    /// it runs the way the edge does. A station whose copy runs the other
    /// way is where the walls turn the copy inside out, and is refused.
    /// The curve is fitted through the stations, enclosing the branch
    /// between them too ([`fit_branch`]); the caller checks it lies on both
    /// inner surfaces.
    fn traced_inner_curve(&self, e: usize, start: Vector3, end: Vector3) -> GeopResult<Curve3> {
        let edge = &self.spec.edges[e];
        let (f, g) = match self.between[e].as_slice() {
            &[f, g] if f != g => (f, g),
            &[f, ..] => return self.lifted_inner_curve(e, f, start, end),
            [] => unreachable!("an edge of a face"),
        };
        let (outer_f, outer_g) = (&self.spec.faces[f].surface, &self.spec.faces[g].surface);
        let (inner_f, inner_g) = (&self.inner_surfaces[f], &self.inner_surfaces[g]);
        // Where the edge starts on each face, as a seed.
        let (mut seed_f, mut seed_g) = (self.uv_at(f, edge.start)?, self.uv_at(g, edge.start)?);
        let onto = |surface: &NurbSurface3D, point: Vector3, seed: &mut Vector2| {
            let (u, v) = surface.project_in_domain(
                point,
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            *seed = Vector2::from_array([u, v]);
            Ok::<_, GeopError>((u, v))
        };

        let breakpoints = edge.curve.breakpoints();
        let mut stations = Vec::new();
        for pair in breakpoints.windows(2) {
            stations.push(pair[0].add(pair[1]).div(Scalar::TWO)?.sharpen());
            stations.push(pair[1]);
        }
        stations.pop();

        let on_both = |point: Vector3, sf: &mut Vector2, sg: &mut Vector2| -> GeopResult<OnBoth> {
            let (uf, vf) = onto(inner_f, point, sf)?;
            let (ug, vg) = onto(inner_g, point, sg)?;
            Ok((uf, vf, ug, vg))
        };
        let mut points = vec![start];
        let mut uvs = vec![on_both(start, &mut seed_f, &mut seed_g)?];
        for t in stations {
            let ctx = |err: GeopError| err.with_context(format!("the station at t={t:?}"));
            let point = edge.curve.evaluate(t).map_err(ctx)?;
            let along = edge
                .curve
                .tangent(t)
                .and_then(|d| d.normalize())
                .map_err(ctx)?;
            let seeds = on_both(point, &mut seed_f, &mut seed_g).map_err(ctx)?;
            let (copy, (uf, vf, ug, vg)) = predictor_corrector_step(
                inner_f,
                inner_g,
                point,
                along,
                seeds.0,
                seeds.1,
                seeds.2,
                seeds.3,
                Scalar::ZERO,
            )
            .map(|(p, uf, vf, ug, vg)| (p, (uf, vf, ug, vg)))
            .map_err(ctx)?;
            // The edge runs along `n_f × n_g` one way; its copy, where the
            // inner surfaces face the way the faces do, runs the same way.
            let (puf, pvf) = outer_f
                .project(point, uf.sharpen(), vf.sharpen(), PROJECT_ITERATIONS)
                .map_err(ctx)?;
            let (pug, pvg) = outer_g
                .project(point, ug.sharpen(), vg.sharpen(), PROJECT_ITERATIONS)
                .map_err(ctx)?;
            let way = |a: Vector3, b: Vector3| a.prod_cross(&b).prod_dot(&along);
            let outer_way = way(
                outer_f.normal(puf, pvf).map_err(ctx)?,
                outer_g.normal(pug, pvg).map_err(ctx)?,
            );
            let inner_way = way(
                inner_f.normal(uf, vf).map_err(ctx)?,
                inner_g.normal(ug, vg).map_err(ctx)?,
            );
            let same = (outer_way.definitely_greater(Scalar::ZERO)
                && inner_way.definitely_greater(Scalar::ZERO))
                || (outer_way.definitely_less(Scalar::ZERO)
                    && inner_way.definitely_less(Scalar::ZERO));
            if !same {
                return Err(GeopError::new(format!(
                    "the walls are too thick: the inner copy of the edge at {copy:?} could run the other way than the edge at {point:?} ({inner_way:?} against {outer_way:?})"
                )));
            }
            points.push(copy);
            uvs.push((uf, vf, ug, vg));
            seed_f = Vector2::from_array([uf, vf]);
            seed_g = Vector2::from_array([ug, vg]);
        }
        points.push(end);
        uvs.push(on_both(end, &mut seed_f, &mut seed_g)?);
        let mut widest = None;
        let ctx = |err: GeopError| err;
        fit_branch(inner_f, inner_g, &points, &uvs, &mut widest, &ctx).map_err(|err| {
            err.with_context(format!(
                "fitting the copy through {} stations; the widest true point between them: {widest:?}",
                points.len()
            ))
        })
    }

    /// Where every coedge's inner copy ends on its face's inner surface, at
    /// a walled vertex: the inner vertex projected onto it, from where the
    /// coedge ends on the face — one `(u, v)` per junction of a loop, so
    /// the coedges meeting there agree on it exactly.
    fn inner_ends(&self) -> GeopResult<HashMap<At, Vector2>> {
        let mut out = HashMap::new();
        for (at, c) in self.coedges() {
            let (_, end) = self.spec.ends(c.on)?;
            if !self.walled[end] || self.collapsed[at.f] {
                continue;
            }
            let seed = c.pcurve.ends()?.1;
            let surface = &self.inner_surfaces[at.f];
            let (u, v) = surface.project_in_domain(
                self.inner_point(end),
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            out.insert(at, Vector2::from_array([u, v]));
        }
        Ok(out)
    }

    /// Whether the coedge `at` has an inner copy: not along an edge whose
    /// copy vanishes.
    fn has_copy(&self, at: At) -> bool {
        match self.coedge(at).on {
            CoedgeOn::Edge(e, _) => self.copy[e] != InnerEdge::Vanishes,
            CoedgeOn::Vertex(_) => true,
        }
    }

    /// Where the inner copy of the coedge `at` starts and ends on its face's
    /// inner surface: where the copy of the coedge before it that has one
    /// ends, so that the copies of a loop join exactly.
    fn inner_pins(&self, at: At) -> (Vector2, Vector2) {
        let mut before = self.previous(at);
        while before != at && !self.has_copy(before) {
            before = self.previous(before);
        }
        (self.inner_ends[&before], self.inner_ends[&at])
    }

    /// The inner copy of the coedge `at`, on its face's inner surface.
    fn inner_coedge(&self, at: At) -> GeopResult<CoedgeSpec> {
        let c = self.coedge(at);
        let (start, end) = self.inner_pins(at);
        let (on, curve) = match c.on {
            CoedgeOn::Edge(e, sense) => {
                let (e, sense) = match self.copy[e] {
                    InnerEdge::As(first, along) => (
                        first,
                        if along == Sense::Forward {
                            sense
                        } else {
                            sense.opposite()
                        },
                    ),
                    _ => (e, sense),
                };
                let curve = self.inner_curves[e].as_ref().expect("an edge with a copy");
                (CoedgeOn::Edge(e, sense), Some(sense.orient(curve)))
            }
            CoedgeOn::Vertex(_) => (c.on, None),
        };
        Ok(CoedgeSpec {
            on,
            pcurve: pcurve(
                &self.inner_surfaces[at.f],
                &c.pcurve,
                curve.as_ref(),
                start,
                end,
            )?,
        })
    }

    /// The inner copy of loop `l` of face `f`, its coedges numbered by
    /// `edges` and `vertices`: the copy of each of its coedges that has
    /// one.
    fn inner_loop(
        &self,
        f: usize,
        l: usize,
        edges: &[Option<usize>],
        vertices: &[Option<usize>],
    ) -> GeopResult<Vec<CoedgeSpec>> {
        let mut out = Vec::new();
        for k in 0..self.spec.faces[f].loops()[l].len() {
            let here = At { f, l, k };
            if !self.has_copy(here) {
                continue;
            }
            let mut c = self.inner_coedge(here)?;
            c.on = match c.on {
                CoedgeOn::Edge(e, sense) => CoedgeOn::Edge(edges[e].expect("a copy"), sense),
                CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertices[v].expect("walled")),
            };
            out.push(c);
        }
        if out.is_empty() {
            return Err(GeopError::new(format!(
                "the walls are too thick: every edge of a loop of face {} vanishes, and its inner copy with it",
                self.names.faces[f]
            )));
        }
        Ok(out)
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
            inner_vertex: vec![None; spec.vertices.len()],
            edge: vec![None; spec.edges.len()],
            inner_edge: vec![None; spec.edges.len()],
            pieces: vec![(None, None); spec.edges.len()],
        };
        for v in 0..spec.vertices.len() {
            if self.walled[v] {
                at.vertex[v] = Some(out.vertices.len());
                out.vertices.push(spec.vertices[v]);
                names.vertices.push(self.names.vertices[v].clone());
                if self.group[v] == v {
                    at.inner_vertex[v] = Some(out.vertices.len());
                    out.vertices.push(self.inner_point(v));
                    names.vertices.push(inner_name(&self.names.vertices[v]));
                }
            }
        }
        for v in 0..spec.vertices.len() {
            at.inner_vertex[v] = at.inner_vertex[self.group[v]];
        }
        let (vertex, inner_vertex) = (&at.vertex, &at.inner_vertex);
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
            }
            if let Some(curve) = &self.inner_curves[e] {
                at.inner_edge[e] = push_edge(
                    curve.clone(),
                    inner_vertex[s].unwrap(),
                    inner_vertex[t].unwrap(),
                    inner_name(name),
                );
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

        let mut outer_faces = Vec::new();
        let mut inner_faces = Vec::new();
        for (f, face) in spec.faces.iter().enumerate() {
            let name = &self.names.faces[f];
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

            if self.collapsed[f] {
                continue;
            }
            let inner_loop = |l: usize| -> GeopResult<Vec<CoedgeSpec>> {
                self.inner_loop(f, l, &at.inner_edge, &at.inner_vertex)
            };
            let inner = FaceSpec {
                surface: self.inner_surfaces[f].clone(),
                outer: inner_loop(0)?,
                holes: (1..=face.holes.len())
                    .map(inner_loop)
                    .collect::<GeopResult<_>>()?,
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
        for (_, c) in self.coedges() {
            let CoedgeOn::Edge(e, sense) = c.on else {
                continue;
            };
            if !self.free_edge[e] {
                continue;
            }
            let (x, y) = spec.ends(c.on)?;
            for v in [x, y] {
                if side[v].is_none() {
                    side[v] = Some(out.edges.len());
                    out.edges.push(EdgeSpec {
                        curve: NurbCurve3D::line(spec.vertices[v], self.inner_point(v))?,
                        start: at.vertex[v].unwrap(),
                        end: at.inner_vertex[v].unwrap(),
                    });
                    names
                        .edges
                        .push(namer.name(&[&self.names.vertices[v], "side"]));
                }
            }
            let outer = sense.orient(&spec.edges[e].curve);
            let inner = sense.orient(self.inner_curves[e].as_ref().expect("not open"));
            // From the inner copy, at `v = 0`, out to the edge, at `v = 1`:
            // running round it counter-clockwise, the wall runs the edge
            // the other way than its face does, and faces away from it.
            let lp = [
                CoedgeOn::Edge(at.inner_edge[e].unwrap(), sense),
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
    /// each copy of `X` named `N(X)` for `namer`'s `N`.
    fn inner_side(&self, namer: &Namer) -> GeopResult<(BodySpec, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec::new(false);
        let mut names = BodyNames::default();
        let mut vertex = vec![None; spec.vertices.len()];
        for (v, name) in self.names.vertices.iter().enumerate() {
            if self.group[v] == v {
                vertex[v] = Some(out.vertices.len());
                out.vertices.push(self.inner_point(v));
                names.vertices.push(namer.name(&[name]));
            }
        }
        for v in 0..spec.vertices.len() {
            vertex[v] = vertex[self.group[v]];
        }
        let mut edge = vec![None; spec.edges.len()];
        for (e, original) in spec.edges.iter().enumerate() {
            if self.inner_curves[e].is_none() {
                continue;
            }
            edge[e] = Some(out.edges.len());
            out.edges.push(EdgeSpec {
                curve: self.inner_curves[e].clone().expect("not open"),
                start: vertex[original.start].unwrap(),
                end: vertex[original.end].unwrap(),
            });
            names.edges.push(namer.name(&[&self.names.edges[e]]));
        }
        for (f, face) in spec.faces.iter().enumerate() {
            if self.collapsed[f] {
                continue;
            }
            let inner_loop = |l: usize| self.inner_loop(f, l, &edge, &vertex);
            out.faces.push(FaceSpec {
                surface: self.inner_surfaces[f].clone(),
                outer: inner_loop(0)?,
                holes: (1..=face.holes.len())
                    .map(inner_loop)
                    .collect::<GeopResult<_>>()?,
            });
            names.faces.push(namer.name(&[&self.names.faces[f]]));
        }
        out.shells = vec![(0..out.faces.len()).collect()];
        Ok((out, names))
    }

    /// What is left of the face `f`, taken away: `f` without its inner copy
    /// (see the module docs), as one face or several, the result's entities
    /// numbered `at`.
    fn rims(&self, f: usize, at: &Numbering) -> GeopResult<Vec<FaceSpec>> {
        let spec = self.spec;
        let (vertex, inner_vertex) = (&at.vertex, &at.inner_vertex);
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
                if !self.has_copy(here) {
                    continue;
                }
                let inner = self.inner_coedge(here)?;
                let CoedgeOn::Edge(copy, copy_sense) = inner.on else {
                    unreachable!("the copy of an edge");
                };
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(at.inner_edge[copy].unwrap(), copy_sense.opposite()),
                        pcurve: inner.pcurve.reverse(),
                    },
                    inner_vertex[y].unwrap(),
                    inner_vertex[x].unwrap(),
                ));
                continue;
            }
            // An edge to another face taken away: what is left of it, from
            // `x` to its inner copy and from `y`'s to `y`.
            let (orig_start, orig_end) = c.pcurve.ends()?;
            let (start_piece, end_piece) = in_order(at.pieces[e], sense);
            let curves = &self.pieces[e];
            let (start_curve, end_curve) = in_order((&curves.0, &curves.1), sense);
            if let (Some(piece), Some(curve)) = (start_piece, start_curve) {
                let to = self.inner_ends[&self.previous(here)];
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
                let from = self.inner_ends[&here];
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

/// A curve shaped like `curve` from `start` to `end` — all the way round
/// a circle back to `start`, if `closed`: a line for a straight one, an arc
/// around the same axis for a circular one, exact. `None` for any other
/// curve, whose copy is traced instead (see `Hollow::traced_inner_curve`);
/// an error for an `end` not on the circle around that axis through
/// `start`.
fn shaped_like(
    curve: &Curve3,
    start: Vector3,
    end: Vector3,
    closed: bool,
) -> Option<GeopResult<Curve3>> {
    match curve.as_line() {
        Err(e) => return Some(Err(e)),
        Ok(Some(_)) => return Some(NurbCurve3D::line(start, end)),
        Ok(None) => {}
    }
    match curve.as_arc() {
        Err(e) => Some(Err(e)),
        Ok(None) => None,
        Ok(Some(arc)) => Some(arc_like(curve, &arc, start, end, closed)),
    }
}

/// [`shaped_like`] for a circular `curve`, the arc `arc`.
fn arc_like(
    curve: &Curve3,
    arc: &Arc,
    start: Vector3,
    end: Vector3,
    closed: bool,
) -> GeopResult<Curve3> {
    let axis = arc.circle.axis();
    let center = axis.project(&start);
    let radius = start.sub(&center).norm();
    if !closed
        && !(axis.project(&end).could_be_equal(&center)
            && end.sub(&center).norm().could_be_equal(radius))
    {
        return Err(GeopError::new(format!(
            "{end:?} is not on the circle around {axis:?} through {start:?}"
        )));
    }
    // Where the copy spans the same angles as the arc — a cylinder's rim
    // moved along its axis, a disc's rim shrunk — it is the arc's own
    // control points moved along the axis and scaled about it, weights and
    // knots kept: the same parametrization, so the ruled surface between
    // the two, a sheet's wall, is the cylinder, cone or ring between them,
    // not a twisted one. Where it spans other angles — a sphere's meridian
    // cut by an offset plane — it is the arc between its ends.
    let scale = radius.div(arc.circle.radius)?;
    let control_points = curve
        .control_points
        .iter()
        .map(|cp| {
            let w = cp[3];
            let p = cp.dehomogenized()?;
            let moved = center.add(&p.sub(&arc.circle.center).prod_scalar(scale));
            Ok(Vector::from_array([
                moved[0].mul(w),
                moved[1].mul(w),
                moved[2].mul(w),
                w,
            ]))
        })
        .collect::<GeopResult<Vec<_>>>()?;
    let moved = NurbCurve3D::try_new(curve.degree, control_points, curve.knot_vector.clone())?;
    let (from, to) = moved.ends()?;
    let end = if closed { start } else { end };
    if from.could_be_equal(&start) && to.could_be_equal(&end) {
        return Ok(moved);
    }
    Arc {
        circle: Circle {
            center,
            normal: arc.circle.normal,
            radius,
        },
        start,
        end,
    }
    .to_curve()
}

/// The parameter of `curve` at `point`, a point of it, found from `seed`:
/// refined until it pins the point down, and checked to be at it.
fn parameter_at(curve: &Curve3, point: Vector3, seed: Scalar) -> GeopResult<Scalar> {
    let t = curve.closest_parameter_from(&point, seed)?;
    let t = curve.refine_parameter_at_point(t, &point);
    let at = curve.evaluate(t)?;
    if !at.could_be_equal(&point) {
        return Err(GeopError::new(format!(
            "{point:?} is not on the curve: the nearest point is {at:?}, at t={t:?}"
        )));
    }
    Ok(t)
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
/// a vertex, without one — from `(u, v)` `start` to `end`. Straight where
/// the coedge it is a copy of had a straight pcurve, `original`, and its
/// inner copy is straight too: on a plane, whose `(u, v)` are affine, or
/// along the same iso-line — the line keeps its `u` (or `v`) — as a
/// cylinder's rim or seam does. Fitted otherwise: a sphere's meridian cut
/// by a plane is straight, but its inner copy, where the offset plane cuts
/// the offset sphere, is a small circle no straight pcurve follows.
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
    let straight = original.degree == 1 && original.control_points.len() == 2 && {
        let (a, b) = original.ends()?;
        let keeps = |k: usize| a[k].could_be_equal(b[k]) && start[k].could_be_equal(end[k]);
        surface.as_plane()?.is_some() || keeps(0) || keeps(1)
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
