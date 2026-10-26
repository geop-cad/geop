//! Remesh every edge x face pair between two solids: wherever an edge of
//! one solid lies *within* a face of the other, imprint it there as a new
//! boundary; wherever an edge instead pierces a face transversally at a
//! single point, split the edge there and trace the intersection curve
//! that begins at that point until it reaches an already-known vertex,
//! then splice the traced curve in as a new shared edge.
//!
//! Pipeline, in order (each phase runs to a fixed point, in *both*
//! `(solid_a, solid_b)` and `(solid_b, solid_a)` directions before the next
//! phase starts, since either solid's edges might pierce or lie within the
//! other's faces):
//!
//! 1. [`split_piercing_crossings`] — split every edge at every point it
//!    genuinely (transversally) crosses a face of the other solid.
//! 2. [`imprint_coincident_pairs`] — imprint every edge that lies entirely
//!    within a face of the other solid as that face's own new dangling
//!    boundary loop.
//! 3. [`find_tracing_start_points`] — once splitting/imprinting has fully
//!    settled, scan the *final* state for every vertex that sits exactly
//!    on a face it doesn't already have a boundary coedge on (i.e. every
//!    piercing point step 1 just created) and record it as a tracing
//!    start point. Deliberately a fresh scan of the settled model, not
//!    something threaded through step 1's own mutations — a start point
//!    recorded mid-loop could be silently invalidated by a *later* split
//!    further along the very same edge.
//! 4. For each start point, trace the intersection curve outward from it
//!    (see [`trace_from_start_point`]) and splice in whatever new edge
//!    that produces. The march finds which vertex the curve ends at, and
//!    the edge's curve is a cubic fitted through the march, whatever the
//!    two surfaces are (see [`fit_branch`]): no pair of surface
//!    kinds is special-cased.
//!    Before tracing, a vertex at each node inside both faces, where
//!    branches cross with the faces tangent, for the traces arriving there
//!    to stop at (see [`insert_face_points`]).
//! 5. Last, the closed loops that no edge pierces, so that no start point
//!    leads to: a vertex at each of their turning points along a direction,
//!    found by the same subdivision search over each pair of faces, and the
//!    loops traced from them as in 4.

use geop_core_geometry::{
    contains::{curve::curve_could_contain, surface::surface_could_contain},
    intersection::{
        Intersections,
        branch::{fit_branch, predictor_corrector_step},
        curve_surface_intersect, refine_crossing,
        surface_surface::{TurningPoints, refine_node, turning_points},
    },
    nurb_curve::NurbCurve,
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::{
        bisect::bisect,
        probe::{Probe, attempts},
        repeat::{self, repeat, rewrite},
    },
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, CoedgeGeometry, Edge, EdgeId, FaceId, Model, VertexId,
    contains::face::{PointClassification, face_contains, face_interior_point},
};
use geop_ops_model::NamedModel;

use crate::naming::BooleanNaming;
use crate::remesh::{RemeshParams, remesh_edges_x_edges::find_vertex_at_point, split_at_crossing};

/// A vertex where an edge of one solid meets a face of the other without
/// running along it — where the edge pierces the face, or ends on it —
/// recorded so the intersection curves through it can be traced once every
/// such point across both solids has been found (see this module's own top
/// doc comment for why that's a separate pass, not done inline while
/// splitting).
///
/// What is recorded is the vertex, not the edge that led to it. Every face
/// of either solid holding the vertex may carry a curve leaving it, and the
/// edge that happened to find it borders only some of them: several edges
/// meet at a vertex, and a face holding it can be split by an earlier trace
/// so that a given edge borders only one half. Which faces a curve can leave
/// between is a property of the vertex — see [`trace_from_start_point`].
struct TracingStartPoint {
    vertex: VertexId,
    /// The solid whose edge ends at `vertex`, so `vertex` lies on the
    /// boundary of its faces there.
    edge_solid: Body,
    /// The face of the other solid found holding `vertex`, as of when the
    /// point was found — only a record of where to look: the vertex may lie
    /// on none of that solid's boundaries, and an earlier trace may have
    /// split the face since (see [`faces_at_vertex`]).
    face: FaceId,
    /// The solid `face` belongs to.
    face_solid: Body,
}

/// Fixed PRNG seed for `face_contains`' ray casting — the classification it
/// returns is seed-independent (it retries until it finds a ray that grazes
/// no vertex), so a constant keeps tracing reproducible run to run.
const FACE_CONTAINS_SEED: u64 = 0x9E37_79B9_7F4A_7C15;

/// How many marching steps to spend on a full revolution of the tighter of
/// the two surfaces' curvature. A step has to be short compared to how fast
/// the curve is turning, or the straight-line predictor leaves the surface
/// and the corrector drags it back somewhere else entirely — which is what
/// makes a trace wander off instead of following the curve.
const STEPS_PER_REVOLUTION: usize = 64;

/// How many marching steps to spend across the smaller of the two surfaces
/// a trace runs between (see [`NurbSurface3D::size`]), where neither
/// curves. The curve lies on both, so this is the scale of the features it
/// can pass: the stride grows and shrinks with the part instead of being
/// fixed in model units. A fixed 0.1 took 400 steps along a 40 mm cut, and
/// the volume integrated along the cubic fitted through them came out wider
/// than `long_bars_overlap` allows; it was also more than a cut 0.05 wide.
/// Like [`STEPS_PER_REVOLUTION`] this bounds effort and the fit's width,
/// not whether a trace finds its end — a stride reaches a vertex wherever
/// one lies on it (see [`candidate_within`]).
const STEPS_ACROSS_PATCH: usize = 16;

/// The marching step to use at `(u_a, v_a)` / `(u_b, v_b)`, marching along
/// `dir`: a [`STEPS_ACROSS_PATCH`]th of the smaller surface, shortened so a
/// full turn of whichever surface is curving harder there would take
/// [`STEPS_PER_REVOLUTION`] steps. A surface that's locally flat — or runs
/// straight the way the curve does — reports no curvature radius and so
/// imposes no limit. Sharp: where a stride lands is a free choice.
///
/// A surface that runs straight the way the curve does bends it by nothing,
/// however it bends the other way. Limited by that other bend, a trace
/// running along a generator into the apex of a cone took a step a fraction
/// of its distance to the apex — the circle round the axis shrinks with it —
/// and never arrived: 359 steps, spaced geometrically, for a straight line.
/// Where a surface does bend the curve, if only a little, the tightest bend
/// still limits the step, not the bend along the curve here: a curve that
/// bends a little here may bend sharply a stride on (the blades of
/// `turbine_blade`, whose pattern's trace left its patch).
fn adaptive_step_size(
    surf_a: &NurbSurface3D,
    surf_b: &NurbSurface3D,
    u_a: Scalar,
    v_a: Scalar,
    u_b: Scalar,
    v_b: Scalar,
    dir: &Vector3,
) -> GeopResult<Scalar> {
    let arc = Scalar::TWO
        .mul(Scalar::PI)
        .div(Scalar::from_i64(STEPS_PER_REVOLUTION as i64))?;
    let mut step = surf_a
        .size()?
        .min(surf_b.size()?)
        .div(Scalar::from_i64(STEPS_ACROSS_PATCH as i64))?
        .sharpen();
    // A surface that runs straight the way the curve does bends it by
    // nothing, however it bends the other way; any other bends it, and is
    // limited by its tightest bend, as ever: a curve whose surfaces bend it
    // a little here may bend sharply a stride on.
    let radius_along =
        |surface: &NurbSurface3D, u: Scalar, v: Scalar| -> GeopResult<Option<Scalar>> {
            match surface.curvature_radius_along(u, v, dir) {
                Ok(None) => Ok(None),
                _ => surface.curvature_radius(u, v),
            }
        };
    for radius in [
        radius_along(surf_a, u_a, v_a)?,
        radius_along(surf_b, u_b, v_b)?,
    ]
    .into_iter()
    .flatten()
    {
        let limit = radius.abs().mul(arc);
        if limit.definitely_less(step) {
            step = limit.sharpen();
        }
    }
    Ok(step)
}

/// Remesh every edge x face pair between `solid_a` and `solid_b` — see this
/// module's own top doc comment for the full pipeline.
///
/// Every search here, pairwise or single-shape, runs to `params`'
/// `curve_curve_min_subdivision_size`; `max_trace_steps` bounds how many
/// marching steps a single trace may take before it's considered to have
/// failed to find a terminating vertex (see [`adaptive_step_size`] for how
/// long each is).
pub fn remesh_edges_x_faces(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    solid_a: Body,
    solid_b: Body,
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!(
        "remesh_edges_x_faces(solid_a={solid_a}, solid_b={solid_b}, params={params:?})"
    );

    split_piercing_crossings(model, naming, solid_a, solid_b, params).with_context(&ctx)?;
    split_piercing_crossings(model, naming, solid_b, solid_a, params).with_context(&ctx)?;

    imprint_coincident_pairs(model, naming, solid_a, solid_b, params).with_context(&ctx)?;
    imprint_coincident_pairs(model, naming, solid_b, solid_a, params).with_context(&ctx)?;

    // One scan, one pass. Every piercing point either solid's edges make on the
    // other's faces exists by now — that is exactly what the two phases above
    // were run to a fixed point for — so the settled topology already names every
    // place an intersection curve can begin. Tracing does not create new ones:
    // a traced curve runs between two vertices that are already here, and
    // splitting a face along it moves the trim boundaries around without moving
    // any surface, so nothing that was not already a crossing becomes one.
    //
    // Re-scanning used to find more work only because tracing was skipping
    // curves it should have drawn (see `trace_one_side`); a second round
    // rediscovered them. Curing that is what makes the single pass sufficient,
    // and keeping the pass single is what keeps the omission visible instead of
    // papered over.
    let topology = model.topology();
    let mut starts =
        find_tracing_start_points(topology, solid_a, solid_b, params).with_context(&ctx)?;
    // A vertex found from both sides is one start point: tracing from it
    // already tries every face pair through it.
    let found: std::collections::HashSet<VertexId> = starts.iter().map(|s| s.vertex).collect();
    starts.extend(
        find_tracing_start_points(topology, solid_b, solid_a, params)
            .with_context(&ctx)?
            .into_iter()
            .filter(|s| !found.contains(&s.vertex)),
    );

    // Every vertex of the two bodies is a candidate endpoint, not just the
    // start points. A traced curve does run from one piercing point to another in
    // the common case, but it can equally end at a vertex that was already
    // there before this pass — a corner where the two solids' edges were
    // merged by `remesh_vertices`, say, which is a perfectly good place for
    // an intersection curve to terminate yet never becomes a piercing point
    // and so never appears in `starts`. Restricting the search to `starts`
    // makes those traces run to the edge of the patch and fail, having
    // walked right past the vertex they should have stopped at.
    // Only theirs: a traced curve lies on a face of each, so it ends at a
    // vertex of one of them, never at one of a third body — another solid,
    // a wire — that lies near. Sorted: the order candidates are tried in
    // must not depend on hash order, or neither would the traced curves.
    let mut candidates: Vec<VertexId> = topology
        .iter_body_vertices(solid_a)
        .with_context(&ctx)?
        .chain(topology.iter_body_vertices(solid_b).with_context(&ctx)?)
        .collect();
    candidates.sort_by_key(|v| v.0);
    candidates.dedup();

    // What no edge leads to, searched over each pair of the two solids'
    // surfaces (see `geop_core_geometry::intersection::surface_surface`).
    // First the nodes inside both faces, where branches of the curve cross
    // with the faces tangent: a vertex each, where the traces arriving
    // stop, since no corrector can march through one.
    let pairs = search_surface_pairs(model.topology(), naming, solid_a, solid_b, params)
        .with_context(&ctx)?;
    let nodes = insert_face_points(
        model,
        naming,
        &pairs,
        FacePoint::Node,
        solid_a,
        solid_b,
        params,
    )
    .with_context(&ctx)?;
    candidates.extend(nodes.iter().map(|s| s.vertex));

    for start in &starts {
        trace_from_start_point(model, naming, start, &candidates, params).with_context(&ctx)?;
    }

    // Last, the closed loops no edge leads to: a vertex on each of their
    // turning points, and the loops traced between them like any other
    // curve. After the traces above, so that a turning point of a curve
    // already traced lies on its edge, on the boundary of both faces, and
    // is not taken again.
    let loops = insert_face_points(
        model,
        naming,
        &pairs,
        FacePoint::Turning,
        solid_a,
        solid_b,
        params,
    )
    .with_context(&ctx)?;
    candidates.extend(loops.iter().map(|s| s.vertex));
    for start in &loops {
        trace_from_start_point(model, naming, start, &candidates, params).with_context(&ctx)?;
    }

    Ok(())
}

/// A pair of surfaces of the two solids, each that of the faces split from
/// one face of the solid (its origin, see `naming`), and what the search
/// over the pair found.
struct SurfacePair {
    origins: [String; 2],
    surfaces: [NurbSurface3D; 2],
    found: TurningPoints,
}

/// The faces of `solid` by their origin: one surface each.
fn faces_by_origin(
    model: &Model,
    naming: &BooleanNaming,
    solid: Body,
) -> GeopResult<std::collections::BTreeMap<String, Vec<FaceId>>> {
    let mut groups: std::collections::BTreeMap<String, Vec<FaceId>> = Default::default();
    let mut faces = model.body_faces(solid)?;
    faces.sort_by_key(|f| f.0);
    for face in faces {
        groups
            .entry(naming.face_origin(face)?.to_string())
            .or_default()
            .push(face);
    }
    Ok(groups)
}

/// The search for turning points and nodes over every pair of `solid_a`'s
/// and `solid_b`'s surfaces whose boxes overlap: once per pair of the faces
/// the two solids had before the remesh, whose pieces share their surface.
fn search_surface_pairs(
    model: &Model,
    naming: &BooleanNaming,
    solid_a: Body,
    solid_b: Body,
    params: &RemeshParams,
) -> GeopResult<Vec<SurfacePair>> {
    let (groups_a, groups_b) = (
        faces_by_origin(model, naming, solid_a)?,
        faces_by_origin(model, naming, solid_b)?,
    );
    let mut pairs = Vec::new();
    for (origin_a, faces_a) in &groups_a {
        let surf_a = &model.get_face(faces_a[0])?.surface;
        for (origin_b, faces_b) in &groups_b {
            let surf_b = &model.get_face(faces_b[0])?.surface;
            if !surf_a.could_overlap(surf_b) {
                continue;
            }
            let found = turning_points(surf_a, surf_b, params.max_nodes).with_context(
                with_context!("searching the faces {origin_a:?} and {origin_b:?}"),
            )?;
            pairs.push(SurfacePair {
                origins: [origin_a.clone(), origin_b.clone()],
                surfaces: [surf_a.clone(), surf_b.clone()],
                found,
            });
        }
    }
    Ok(pairs)
}

/// Which of the points a [`SurfacePair`]'s search found to make vertices of.
#[derive(Clone, Copy, Debug, PartialEq)]
enum FacePoint {
    /// Isolated points where the faces touch, branches crossing there.
    Node,
    /// Points where a curve the faces meet in turns back.
    Turning,
}

/// Where a point a [`SurfacePair`]'s search found lies on one solid: inside
/// one of its faces, or on one of its edges, at a parameter strictly inside
/// it.
#[derive(Clone, Copy, Debug)]
enum Placed {
    Inside(FaceId),
    OnEdge(EdgeId, Scalar),
}

/// A vertex at every point of `which` kind the search over `pairs` found
/// that lies inside a face of each solid, with the faces it lies in as
/// where to trace from. Each candidate is refined first: a node by
/// [`refine_node`], which claims none where the surfaces touch along a
/// curve or over an area; a turning point onto the curve, with the
/// corrector the march uses, across it. A point on a curve already traced
/// lies on that edge, which bounds both faces, so it is not inside them.
/// Two points that could be one are one vertex.
///
/// A node may lie on an edge of one solid instead — on a seam the faces
/// touch across, which touches the other face there without crossing it,
/// so no piercing split it: that edge is split there, the vertex named as
/// where it meets the other face, and the traces arriving stop at it.
fn insert_face_points(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    pairs: &[SurfacePair],
    which: FacePoint,
    solid_a: Body,
    solid_b: Body,
    params: &RemeshParams,
) -> GeopResult<Vec<TracingStartPoint>> {
    let topology = model.topology();
    let (groups_a, groups_b) = (
        faces_by_origin(topology, naming, solid_a)?,
        faces_by_origin(topology, naming, solid_b)?,
    );
    // Where `point`, at `(u, v)` on their surface, lies among `faces`.
    let placed =
        |faces: &[FaceId], u: Scalar, v: Scalar, point: &Vector3| -> GeopResult<Option<Placed>> {
            let mut on_coedge = false;
            for &face in faces {
                let class = face_contains(
                    topology,
                    face,
                    u,
                    v,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                    FACE_CONTAINS_SEED,
                )?;
                match class {
                    PointClassification::Inside => return Ok(Some(Placed::Inside(face))),
                    PointClassification::OnCoedge => on_coedge = true,
                    _ => {}
                }
            }
            if !on_coedge || which != FacePoint::Node {
                return Ok(None);
            }
            let mut edges: Vec<EdgeId> =
                faces.iter().flat_map(|&f| topology.face_edges(f)).collect();
            edges.sort_by_key(|e| e.0);
            edges.dedup();
            for edge_id in edges {
                let curve = &topology.get_edge(edge_id)?.curve;
                let Some(t) = curve_could_contain(
                    curve,
                    point,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                )?
                else {
                    continue;
                };
                let (lo, hi) = curve.domain();
                if t.definitely_greater(lo) && t.definitely_less(hi) {
                    return Ok(Some(Placed::OnEdge(edge_id, t)));
                }
            }
            Ok(None)
        };
    let mut found: Vec<(Vector3, Placed, Placed, Scalar)> = Vec::new();
    for pair in pairs {
        let [surf_a, surf_b] = &pair.surfaces;
        let ctx = with_context!(
            "insert_face_points({which:?}, faces {:?} and {:?})",
            pair.origins[0],
            pair.origins[1]
        );
        let (Some(faces_a), Some(faces_b)) = (
            groups_a.get(&pair.origins[0]),
            groups_b.get(&pair.origins[1]),
        ) else {
            continue;
        };
        let seeds = match which {
            FacePoint::Node => &pair.found.tangent,
            FacePoint::Turning => &pair.found.candidates,
        };
        for candidate in seeds {
            let refined = match which {
                FacePoint::Node => refine_node(surf_a, surf_b, candidate.seed)
                    .map(|([u_a, v_a, u_b, v_b], point)| (point, u_a, v_a, u_b, v_b)),
                FacePoint::Turning => refine_turning_point(surf_a, surf_b, candidate.seed),
            };
            let Some((point, u_a, v_a, u_b, v_b)) = refined else {
                continue;
            };
            let (Some(on_a), Some(on_b)) = (
                placed(faces_a, u_a, v_a, &point).with_context(&ctx)?,
                placed(faces_b, u_b, v_b, &point).with_context(&ctx)?,
            ) else {
                continue;
            };
            if matches!((on_a, on_b), (Placed::OnEdge(..), Placed::OnEdge(..))) {
                // Where two edges cross: the edge phases put a vertex there.
                continue;
            }
            if found.iter().any(|(p, ..)| p.could_be_equal(&point)) {
                continue;
            }
            found.push((point, on_a, on_b, point.prod_dot(&pair.found.direction)));
        }
    }
    let mut starts = Vec::with_capacity(found.len());
    for (point, on_a, on_b, along) in found {
        match (on_a, on_b) {
            (Placed::Inside(face_a), Placed::Inside(face_b)) => {
                let vertex = model.insert_vertex(point, naming.provisional())?;
                naming.face_point(vertex, face_a, face_b, along)?;
                starts.push(TracingStartPoint {
                    vertex,
                    edge_solid: solid_a,
                    face: face_b,
                    face_solid: solid_b,
                });
            }
            (Placed::OnEdge(edge, t), Placed::Inside(face))
            | (Placed::Inside(face), Placed::OnEdge(edge, t)) => {
                split_at_crossing(model, naming, edge, t, None, point, face, params)?;
                let vertex = model.topology().get_edge(edge)?.end_vertex;
                let (edge_solid, face_solid) = match on_a {
                    Placed::OnEdge(..) => (solid_a, solid_b),
                    Placed::Inside(_) => (solid_b, solid_a),
                };
                starts.push(TracingStartPoint {
                    vertex,
                    edge_solid,
                    face,
                    face_solid,
                });
            }
            (Placed::OnEdge(..), Placed::OnEdge(..)) => unreachable!("set aside above"),
        }
    }
    Ok(starts)
}

/// A point of the curve `surf_a` and `surf_b` meet in near `seed`, where
/// they cross, and its parameters on both: the corrector the march uses,
/// landing on the plane through `surf_a` at the seed square to the curve
/// there. `None` where it brings them onto no common point — the boxes the
/// seed came from only said one could be — or onto one where they touch.
fn refine_turning_point(
    surf_a: &NurbSurface3D,
    surf_b: &NurbSurface3D,
    [u_a, v_a, u_b, v_b]: [Scalar; 4],
) -> Option<(Vector3, Scalar, Scalar, Scalar, Scalar)> {
    let start = surf_a.evaluate(u_a, v_a).ok()?;
    let along = surf_a
        .normal(u_a, v_a)
        .ok()?
        .prod_cross(&surf_b.normal(u_b, v_b).ok()?)
        .normalize()
        .ok()?;
    let (point, u_a, v_a, u_b, v_b) = predictor_corrector_step(
        surf_a,
        surf_b,
        start,
        along,
        u_a,
        v_a,
        u_b,
        v_b,
        Scalar::ZERO,
    )
    .ok()?;
    let n_a = surf_a.normal(u_a, v_a).ok()?;
    let n_b = surf_b.normal(u_b, v_b).ok()?;
    n_a.prod_cross(&n_b).normalize().ok()?;
    Some((point, u_a, v_a, u_b, v_b))
}

/// Whether `edge_id` already has a boundary coedge on `face_id` — if so,
/// it's already been imprinted or traced there; nothing more to do for
/// this pair.
fn edge_is_boundary_of_face(model: &Model, edge_id: EdgeId, face_id: FaceId) -> bool {
    model.iterate_face_coedges(face_id).any(|coedge_id| {
        model.coedges.get(&coedge_id).map(|c| c.geometry) == Some(CoedgeGeometry::Edge(edge_id))
    })
}

// ── Phase 1: split every genuine (transversal) piercing crossing ───────────

/// The first genuine interior piercing crossing found — `edge_id` from
/// `edge_solid`, on a face from `face_solid` — with the `t` to split at and
/// either an already-existing vertex there or the `point` to create one at.
/// `None` once every remaining `(edge, face)` pair is either disjoint,
/// coincident (see [`find_coincident_pair`] instead), or only touches at an
/// edge endpoint that's already a vertex (a normal shared corner, not an
/// interior piercing).
///
/// `settled` are the pairs already found to have no piercing, which are
/// not asked again, and every pair found so is added to it. That stays
/// true while the search goes on: splitting an edge at a piercing keeps
/// its id for the first piece, and a piece of a curve crosses nothing the
/// whole did not — so the search is not started over from every pair
/// after every split, which made a boolean of many crossings, a gear's
/// teeth or a pulley's grooves, quadratic in them.
#[allow(clippy::type_complexity)]
fn find_piercing_crossing(
    model: &Model,
    settled: &mut std::collections::HashSet<(EdgeId, FaceId)>,
    edge_solid: Body,
    face_solid: Body,
    params: &RemeshParams,
) -> GeopResult<Option<(EdgeId, Scalar, Option<VertexId>, Vector3, FaceId)>> {
    let ctx = with_context!(
        "find_piercing_crossing(edge_solid={edge_solid}, face_solid={face_solid}, params={params:?})"
    );

    for edge_id in model.iter_body_edges(edge_solid).with_context(&ctx)? {
        for face_id in model.body_faces(face_solid).with_context(&ctx)? {
            if settled.contains(&(edge_id, face_id))
                || edge_is_boundary_of_face(model, edge_id, face_id)
            {
                continue;
            }

            let pair_ctx = with_context!("edge={edge_id}, face={face_id}");

            let edge = model.get_edge(edge_id).with_context(&ctx)?;
            let face = model.get_face(face_id).with_context(&ctx)?;
            let crossings = curve_surface_intersect(
                &edge.curve,
                &face.surface,
                params.max_edge_intersections,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(&pair_ctx)?;

            let Intersections::Found(crossings) = crossings else {
                // Coincident — handled by `find_coincident_pair` instead.
                continue;
            };

            let start_pt = model
                .get_vertex(edge.start_vertex)
                .with_context(&ctx)?
                .point;
            let end_pt = model.get_vertex(edge.end_vertex).with_context(&ctx)?.point;

            let (domain_lo, domain_hi) = edge.curve.domain();
            for (t, uv) in crossings {
                // This parameter is about to become a *split* parameter, so
                // its width matters in a way it does not for callers that
                // only count crossings: `NurbCurve::split` cannot absorb a
                // `min_subdivision_size`-wide one, and the vertex point
                // evaluated from it would be just as wide. Polish it before
                // anything downstream looks at it.
                let (box_t, box_uv) = (t, uv);
                let (t, _) = refine_crossing(&edge.curve, &face.surface, t, uv);
                // Only a transversal crossing is a piercing. Where the curve
                // runs along the surface's tangent plane the crossing is not
                // regular, Newton cannot pin it down, and the box is what the
                // search left at its handoff threshold — not a point the
                // curve is known to meet the patch at. On
                // `three_turned_boxes_one_a_turned_copy_joined` a top edge
                // lying in another box's top plane, ending on that cap's
                // boundary, met the patch only at its own end vertex; the
                // search's last box stopped 1.4e-4 short of it, 1.4e-5
                // outside the cap, and was split at as a crossing, leaving a
                // vertex 5e-5 wide that a later imprint could not get past.
                // A curve tangent to a face is the coincidence phase's and the
                // tangent branches' to handle. Where the surface has no normal
                // to ask (a pole, whose parametrization collapses), nothing
                // says the crossing is tangential, and it is kept.
                //
                // Asked over the box the search proved the crossing lies
                // in, not over Newton's answer: at a tangential contact
                // Newton converges only linearly, and stops beside the
                // contact, where the curve and the surface are still a
                // rounding apart and the tangent already leans off the
                // plane. Two spheres touching at a point had each meridian
                // through the contact "pierce" the other sphere 1e-8 from
                // it, as soon as curve tangents came out a little tighter.
                // A crossing is regular only if the curve is transversal
                // all over the box its root may be in.
                //
                // Running along the face is not yet touching it, though: at
                // an inflection the curve is tangent to the face and still
                // crosses it, and the curves the faces meet in end there. So
                // a tangential box is a crossing after all where the curve
                // lies on definitely different sides of the face just
                // outside it (see `odd_crossing`), at the parameter that
                // separates the sides.
                let t = if let (Ok(tangent), Ok(normal)) = (
                    edge.curve.tangent(box_t),
                    face.surface.normal(box_uv[0], box_uv[1]),
                ) && tangent.prod_dot(&normal).could_be_equal(Scalar::ZERO)
                {
                    match odd_crossing(&edge.curve, &face.surface, box_t, box_uv)
                        .with_context(&pair_ctx)?
                    {
                        Some(t) => t,
                        None => continue,
                    }
                } else {
                    t
                };
                // A `t` that isn't *definitely* strictly inside the domain
                // could be the domain bound itself — i.e. the crossing may
                // be the edge's own start or end vertex, which is a shared
                // corner and not an interior piercing. Splitting there is
                // both meaningless and impossible (`split` rightly refuses
                // a parameter it can't place strictly inside), so skip it
                // on the parameter, exactly as the point check below skips
                // it on the geometry.
                if !t.definitely_greater(domain_lo) || !t.definitely_less(domain_hi) {
                    continue;
                }
                let point = edge
                    .curve
                    .evaluate(t)
                    .with_context(&ctx)
                    .with_context(&pair_ctx)?;
                if point.could_be_equal(&start_pt) || point.could_be_equal(&end_pt) {
                    // Already a vertex — a normal shared corner, not an
                    // interior piercing needing a split.
                    continue;
                }
                let vertex = find_vertex_at_point(model, [edge_solid, face_solid], &point)
                    .with_context(&ctx)?;
                return Ok(Some((edge_id, t, vertex, point, face_id)));
            }
            settled.insert((edge_id, face_id));
        }
    }
    Ok(None)
}

/// Where `curve` crosses `surface` within the parameter box `box_t`, if it
/// does there while running along the surface's tangent plane: a bracket
/// of parameters on whose ends the curve lies on definitely different sides
/// of the surface, each end the last one told on its side. `None` where it
/// touches without crossing
/// — the same side on both ends, two spheres meeting at a point — or where
/// the sides cannot be told at all: the curve lying on the surface, which
/// is the coincidence phase's to imprint, or a box reaching an end of the
/// curve, which is a vertex.
///
/// The side of `C(t)` is the sign of `(C(t) - S(f)) · n` at its foot
/// point `f`, projected from `uv`, with `n` the normal at the sharp middle
/// `m` of the foot point's box: `C(t) - S(f)` runs along the normal at `f`,
/// which points the way `n` does. `S(f)` is taken in its mean-value form,
/// `S(m) + S_u (f_u - m_u) + S_v (f_v - m_v)`, the derivatives over the box:
/// across `n` they are as small as the box, so the box's width enters the
/// offset squared. Evaluated over the box directly, `S` carries the box's
/// width along the surface into every coordinate, and the offset a hundred
/// times the width of the data — at a contact of order three, a vertex 1e-4
/// wide. The parameters asked are the
/// box's outer bounds and the bisection's middles, all sharp: where a side
/// is asked is a free choice. The bracket's ends are not the answer to the
/// question "where does it cross"; the bracket is, and its width is the
/// honest one: at a contact of order three the side flips by `t³`, and
/// stays told until that sinks below the width the evaluation carries.
fn odd_crossing(
    curve: &NurbCurve<4>,
    surface: &NurbSurface3D,
    box_t: Scalar,
    uv: Vector2,
) -> GeopResult<Option<Scalar>> {
    const PROJECTION_ITERATIONS: usize = 20;
    let (t0, t1) = curve.domain();
    let (lo, hi) = (box_t.lower(), box_t.upper());
    if !lo.definitely_greater(t0) || !hi.definitely_less(t1) {
        return Ok(None);
    }
    let mut seed = (uv[0].midpoint(), uv[1].midpoint());
    // A side that cannot be computed — no foot point, no normal — is one
    // that cannot be told.
    let mut side = |t: Scalar| -> Option<bool> {
        let point = curve.evaluate(t).ok()?;
        let (u, v) = (surface.project(point, seed.0, seed.1, PROJECTION_ITERATIONS)).ok()?;
        let (mu, mv) = (u.sharpen(), v.sharpen());
        let normal = surface.normal(mu, mv).ok()?;
        let (su, sv) = surface.derivatives(u, v).ok()?;
        let offset = (point.sub(&surface.evaluate(mu, mv).ok()?).prod_dot(&normal))
            .sub(su.prod_dot(&normal).mul(u.sub(mu)))
            .sub(sv.prod_dot(&normal).mul(v.sub(mv)));
        seed = (mu, mv);
        offset.definite_sign()
    };
    let (Some(below), Some(above)) = (side(lo), side(hi)) else {
        return Ok(None);
    };
    if below == above {
        return Ok(None);
    }
    // From each side in turn: the last parameter definitely on that side,
    // a middle not told so counted with the other. One bisection that
    // stopped at the first middle it could not tell would keep a bracket
    // as wide as the steps before it, however near that middle the
    // crossing is.
    let below_until = bisect(lo, hi, usize::MAX, true, |t| {
        Ok::<_, GeopError>(Some(side(t) == Some(below)))
    })?
    .lo;
    let above_from = bisect(lo, hi, usize::MAX, true, |t| {
        Ok::<_, GeopError>(Some(side(t) != Some(above)))
    })?
    .hi;
    Ok(Some(below_until.union(above_from)))
}

fn split_piercing_crossings(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    edge_solid: Body,
    face_solid: Body,
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!(
        "split_piercing_crossings(edge_solid={edge_solid}, face_solid={face_solid}, params={params:?})"
    );

    let mut settled = std::collections::HashSet::new();
    rewrite(
        model,
        params.max_rewrites,
        |model| {
            find_piercing_crossing(
                model.topology(),
                &mut settled,
                edge_solid,
                face_solid,
                params,
            )
        },
        |model, (edge_id, t, vertex, point, face_id)| {
            split_at_crossing(model, naming, edge_id, t, vertex, point, face_id, params)
        },
    )
    .with_context(&ctx)?
    .done("splitting edges where they pierce faces")
    .with_context(&ctx)?;
    Ok(())
}

// ── Phase 2: imprint every coincident pair ──────────────────────────────────

/// `true` if `face_a` and `face_b` lie in the same plane: their normals
/// (each sampled at a genuine interior point, via [`face_interior_point`])
/// are parallel, and a point of one lies in the other's plane. Any failure
/// to even ask the question (e.g. a face with no real interior to sample)
/// answers `false` — the conservative direction here, since this is only
/// ever used to *skip* a candidate as redundant (see
/// [`is_internal_seam_edge`]); answering `false` just falls back to the
/// pre-existing, already-correct (if slower) full check.
fn faces_are_coplanar(
    model: &Model,
    face_a: FaceId,
    face_b: FaceId,
    params: &RemeshParams,
) -> GeopResult<bool> {
    let Ok((ua, va)) = face_interior_point(
        model,
        face_a,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        FACE_CONTAINS_SEED,
    ) else {
        return Ok(false);
    };
    let Ok((ub, vb)) = face_interior_point(
        model,
        face_b,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        FACE_CONTAINS_SEED,
    ) else {
        return Ok(false);
    };
    let fa = model.get_face(face_a)?;
    let fb = model.get_face(face_b)?;
    let pa = fa.surface.evaluate(ua, va)?;
    let pb = fb.surface.evaluate(ub, vb)?;
    let na = fa.surface.normal(ua, va)?;
    let nb = fb.surface.normal(ub, vb)?;

    let cross = na.prod_cross(&nb);
    let normals_parallel = cross[0].could_be_equal(Scalar::ZERO)
        && cross[1].could_be_equal(Scalar::ZERO)
        && cross[2].could_be_equal(Scalar::ZERO);
    if !normals_parallel {
        return Ok(false);
    }
    let offset = pb.sub(&pa).prod_dot(&na);
    Ok(offset.could_be_equal(Scalar::ZERO))
}

/// One of `edge_id`'s two neighboring faces — *within the solid `edge_id`
/// itself belongs to* — if those two are coplanar with each other, i.e.
/// `edge_id` is a purely internal seam between two mutually-flush patches
/// of that one solid, not a genuine outer boundary edge.
///
/// The motivating case: [`geop_lib_sweep::revolve_at_oriented`]
/// builds any flat cap (e.g. a cylinder's) as a fan of pie-wedge faces
/// meeting at a center pole, the same technique its doubly-curved callers
/// (spheres) genuinely need — but a flat cap doesn't, and when that cap
/// ends up flush against another solid's face, [`find_coincident_pair`]
/// used to try imprinting *every* wedge's radial "spoke" edge into the
/// other face (indistinguishable, to a per-edge coincidence test, from the
/// genuine rim boundary) — both a severe slowdown (many redundant
/// candidates, each an expensive `curve_surface_intersect`, with the face
/// count climbing every imprint) and, worse, a wrong result (interleaved
/// imprints of two different flat caps' spokes could corrupt each other's
/// face-splitting bookkeeping in `Model::splice_edge_into_face`, leaving
/// one cap's hole not actually cut — see
/// `boolean::tests::cube_minus_z_cylinder_with_coplanar_cap_is_fast_and_correct`).
///
/// Skipping such a seam is right only for a face *flush with* the seam's
/// own plane: there the seam carries nothing that plane's outer edges do
/// not, and the region it borders is reached through them (a wedge's rim
/// arc, in the motivating case). For a face that *crosses* that plane the
/// seam carries everything: it is exactly where the two planes meet — the
/// intersection curve — and nothing else supplies it. It cannot be traced
/// either, since every step along it lands on the seam itself. A revolved
/// flat ring cut by a plate's face along its seam is the case that showed
/// it: `ring_revolved_onto_plate`.
fn seam_plane(model: &Model, edge_id: EdgeId, params: &RemeshParams) -> GeopResult<Option<FaceId>> {
    let coedges = model.coedges_of_edge(edge_id);
    let [c1, c2] = coedges[..] else {
        return Ok(None);
    };
    let f1 = model.get_coedge(c1)?.face;
    let f2 = model.get_coedge(c2)?.face;
    if f1 == f2 {
        return Ok(None);
    }
    Ok(faces_are_coplanar(model, f1, f2, params)?.then_some(f1))
}

/// The first `(edge, face)` pair found where `edge`'s curve coincides with
/// `face`'s surface (`curve_surface_intersect` finds an overlap:
/// [`Intersections::Coincident`]) and enters the face's interior, and `edge`
/// isn't already a boundary coedge of `face`.
///
/// `settled` are the pairs already found not to be such, which are not
/// asked again, and every pair found so is added to it. That stays true
/// while imprinting goes on: an imprint splits a face, keeping its id for
/// one piece, and splits no edge; a piece of a face holds no more of a
/// curve's points inside it than the whole did, and the new piece's pairs
/// are new. So the search is not started over from every pair after every
/// imprint, as [`find_piercing_crossing`]'s is not after every split.
fn find_coincident_pair(
    model: &Model,
    settled: &mut std::collections::HashSet<(EdgeId, FaceId)>,
    edge_solid: Body,
    face_solid: Body,
    params: &RemeshParams,
) -> GeopResult<Option<(EdgeId, FaceId)>> {
    let ctx = with_context!(
        "find_coincident_pair(edge_solid={edge_solid}, face_solid={face_solid}, params={params:?})"
    );

    for edge_id in model.iter_body_edges(edge_solid).with_context(&ctx)? {
        let seam = seam_plane(model, edge_id, params).with_context(&ctx)?;
        for face_id in model.body_faces(face_solid).with_context(&ctx)? {
            if settled.contains(&(edge_id, face_id))
                || edge_is_boundary_of_face(model, edge_id, face_id)
            {
                continue;
            }
            // A seam within a plane is that plane's own business only for a
            // face flush with it (see `seam_plane`).
            if let Some(plane) = seam
                && faces_are_coplanar(model, plane, face_id, params).with_context(&ctx)?
            {
                continue;
            }
            let edge = model.get_edge(edge_id).with_context(&ctx)?;
            let face = model.get_face(face_id).with_context(&ctx)?;
            let crossings = curve_surface_intersect(
                &edge.curve,
                &face.surface,
                params.max_edge_intersections,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(with_context!("edge={edge_id}, face={face_id}"))?;
            if let Intersections::Coincident(crossings) = crossings {
                // On the face's *untrimmed* surface. That was enough when a
                // face could carry any number of loops, because imprinting
                // never created another face with the same surface. Now that
                // `splice_edge_into_face` splits a face in two, both halves
                // share one surface, so every edge already imprinted into the
                // original is still "on the surface" of the half it does not
                // belong to — it would be imprinted again, splitting again,
                // without end.
                //
                // Proper face topology is what lets us ask the right question
                // instead: is the edge inside *this* face's trim? Tested at
                // the curve's midpoint, since a coincident edge lies wholly
                // within one face's region. `Inside` only — an edge whose
                // midpoint is `OnCoedge` already runs along this face's
                // boundary and is represented there, so imprinting it would
                // duplicate a coedge rather than add one.
                // Judged on the crossings the search already returned, not on
                // the curve's midpoint alone. An edge can run along this
                // face's boundary for part of its length and cut through the
                // interior elsewhere, and the midpoint then lands on the
                // boundary and reports nothing to imprint — while
                // `disjointness_check` looks at every crossing and sees the
                // interior ones. The two disagreed, and such an edge fell
                // between the phases entirely: `find_piercing_crossing`
                // deferred it here as coincident, and this declined it.
                //
                // Reusing the crossings costs nothing and makes the imprint
                // decision ask exactly the question validation asks.
                let mut enters_interior = false;
                for (_, uv) in &crossings {
                    if matches!(
                        face_contains(
                            model,
                            face_id,
                            uv[0],
                            uv[1],
                            params.max_nodes,
                            params.curve_curve_min_subdivision_size,
                            FACE_CONTAINS_SEED,
                        )
                        .with_context(&ctx)?,
                        PointClassification::Inside
                    ) {
                        enters_interior = true;
                        break;
                    }
                }
                if enters_interior {
                    return Ok(Some((edge_id, face_id)));
                }
            }
            settled.insert((edge_id, face_id));
        }
    }
    Ok(None)
}

fn imprint_coincident_edge(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    edge_id: EdgeId,
    face_id: FaceId,
    params: &RemeshParams,
) -> GeopResult<()> {
    let names = [
        model.name_of(edge_id).map(str::to_string),
        model.name_of(face_id).map(str::to_string),
    ];
    let ctx = with_context!(
        "imprint_coincident_edge(edge={edge_id} ({:?}), face={face_id} ({:?}))",
        names[0],
        names[1]
    );
    let new_face = model
        .splice_edge_into_face(
            edge_id,
            face_id,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            naming.provisional(),
        )
        .with_context(ctx)?;
    naming.face_split(face_id, edge_id, new_face)
}

fn imprint_coincident_pairs(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    edge_solid: Body,
    face_solid: Body,
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!(
        "imprint_coincident_pairs(edge_solid={edge_solid}, face_solid={face_solid}, params={params:?})"
    );

    let mut settled = std::collections::HashSet::new();
    rewrite(
        model,
        params.max_rewrites,
        |model| {
            find_coincident_pair(
                model.topology(),
                &mut settled,
                edge_solid,
                face_solid,
                params,
            )
        },
        |model, (edge_id, face_id)| {
            imprint_coincident_edge(model, naming, edge_id, face_id, params)
        },
    )
    .with_context(&ctx)?
    .done("imprinting edges lying in faces")
    .with_context(&ctx)?;
    Ok(())
}

// ── Phase 3: find tracing start points ──────────────────────────────────────

/// Every point where an edge of `edge_solid` pierces a face of
/// `face_solid` in the model's *current* (fully split/imprinted) state:
/// for each edge's each endpoint vertex, for each face it doesn't already
/// have a boundary coedge on, if that vertex's point lies on the face's
/// surface, that's a piercing point. Deduped on the vertex (an edge's two
/// endpoint-vertex checks, two different edges meeting at the same vertex,
/// or two faces holding it would otherwise rediscover the same point).
///
/// Like `curve_surface_intersect`/`surface_could_contain` throughout this
/// codebase, this checks against a face's *untrimmed* surface function,
/// not its actual trimmed region — a vertex that merely happens to lie on
/// two faces' coincident-but-unrelated underlying surfaces (e.g. two
/// separate coplanar faces sharing one infinite plane) would show up here
/// too. Accepted as a pre-existing, codebase-wide limitation (the same one
/// `disjointness_check::check_edges_and_faces_consistent` already lives
/// with), not something this module solves fresh.
fn find_tracing_start_points(
    model: &Model,
    edge_solid: Body,
    face_solid: Body,
    params: &RemeshParams,
) -> GeopResult<Vec<TracingStartPoint>> {
    let ctx = with_context!(
        "find_tracing_start_points(edge_solid={edge_solid}, face_solid={face_solid}, params={params:?})"
    );

    let mut seen: std::collections::HashSet<VertexId> = std::collections::HashSet::new();
    let mut starts = Vec::new();

    for edge_id in model.iter_body_edges(edge_solid).with_context(&ctx)? {
        let edge = model.get_edge(edge_id).with_context(&ctx)?;
        for &vertex_id in &[edge.start_vertex, edge.end_vertex] {
            let point = model.get_vertex(vertex_id).with_context(&ctx)?.point;
            for face_id in model.body_faces(face_solid).with_context(&ctx)? {
                // Rejected before the pair is claimed, not after. This edge
                // running along `face_id`'s boundary says nothing about
                // whether the *vertex* is a place an intersection curve
                // starts — it only makes this edge the wrong one to derive
                // the curve's other face from. Claiming the pair first and
                // then rejecting it burns the vertex for every other edge
                // meeting there, and the start point is lost: on
                // `box_grid_n0p50_n0p50_0p00`, where the two boxes' top faces
                // are coplanar and so share edges, that silently dropped the
                // whole segment along which one box's face crosses the
                // other's, leaving a face straddling the other solid's
                // boundary that no boolean can classify.
                if edge_is_boundary_of_face(model, edge_id, face_id) {
                    continue;
                }
                if seen.contains(&vertex_id) {
                    continue;
                }
                let surface = &model.get_face(face_id).with_context(&ctx)?.surface;
                if surface_could_contain(
                    surface,
                    &point,
                    params.max_nodes,
                    params.curve_curve_min_subdivision_size,
                )
                .with_context(&ctx)?
                .is_some()
                {
                    seen.insert(vertex_id);
                    starts.push(TracingStartPoint {
                        vertex: vertex_id,
                        edge_solid,
                        face: face_id,
                        face_solid,
                    });
                }
            }
        }
    }
    Ok(starts)
}

/// The nearest candidate endpoint within `radius` of `point` that actually
/// lies on *both* traced surfaces, excluding `origin` (the vertex the trace
/// started from).
///
/// Proximity alone is not enough, and using it alone was a bug. A traced
/// curve is the intersection of `surf_a` and `surf_b`, so the vertex it
/// terminates at is by definition a point of both — but an oversized or
/// overhanging solid puts plenty of *unrelated* vertices within one marching
/// stride of the curve's end. Adopting one of those ends the edge somewhere
/// that is not on either face, and the two coedges then spliced onto those
/// faces get pcurve endpoints that cannot possibly match: `fit_pcurve` has
/// nothing to project onto, so it returns the nearest foot point instead,
/// ~8e-3 away. That surfaced far downstream as 32 `validate_fast` errors
/// about pcurve endpoints not matching their 3-D points, on 4 faces, always
/// with exactly one endpoint of each edge off-surface.
///
/// The membership test is the same `surface_could_contain` the rest of this
/// module uses, so a vertex that genuinely lies on both faces still
/// terminates the trace exactly as before — this only rejects the ones that
/// never belonged.
///
/// Lying on both surfaces is still not lying on the stretch of curve a
/// stride runs along. Once the march has committed to a `stride` — a
/// direction, and the distance along it to the plane the corrector lands
/// on — the vertex it reaches is one between `point` and that plane. One
/// definitely behind `point` is not it: the march left it behind, or it
/// lies on the curve's other side of the start. On `narrow_groove` a trace
/// leaving the cut's inner corner along the wall found, one stride later,
/// the vertex 0.05 *behind* its start, where the cut's outer floor edge
/// crosses the wall's plane outside the wall, nearer than anything ahead —
/// and spliced a spur out of the wall to it. One definitely past the plane
/// is the next stride's to reach.
#[allow(clippy::too_many_arguments)]
fn candidate_within(
    model: &Model,
    candidates: &[VertexId],
    origin: VertexId,
    point: &Vector3,
    radius: Scalar,
    stride: Option<(&Vector3, Scalar)>,
    surf_a: &NurbSurface3D,
    surf_b: &NurbSurface3D,
    params: &RemeshParams,
) -> GeopResult<Option<VertexId>> {
    let radius_sq = radius.mul(radius);
    let mut best: Option<(VertexId, Scalar)> = None;
    for &candidate in candidates {
        if candidate == origin {
            continue;
        }
        let candidate_point = model.get_vertex(candidate)?.point;
        let offset = candidate_point.sub(point);
        let d = offset.norm_sq();
        if !d.could_be_less(radius_sq) {
            continue;
        }
        if let Some((heading, length)) = stride {
            let along = offset.prod_dot(heading);
            if along.definitely_less(Scalar::ZERO) || along.definitely_greater(length) {
                continue;
            }
        }
        if best.is_some() && !d.could_be_less(best.expect("checked").1) {
            continue;
        }
        let on_both = surface_could_contain(
            surf_a,
            &candidate_point,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
        )?
        .is_some()
            && surface_could_contain(
                surf_b,
                &candidate_point,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )?
            .is_some();
        if !on_both {
            continue;
        }
        best = Some((candidate, d));
    }
    Ok(best.map(|(id, _)| id))
}

// ── Phase 4: trace each start point ─────────────────────────────────────────

/// Trace the intersection curve(s) starting at `start.vertex`: every curve
/// leaving it runs between one face of `start.edge_solid` and one of
/// `start.face_solid`, both holding the vertex, so each such pair gets its
/// own, separate trace attempt (most find no direction and cost one
/// predictor-corrector step each way).
///
/// Both sides are looked up afresh before every pair, not once: a trace
/// splits the faces it runs between, so after one curve has left the vertex
/// a face holding it may be two, with the vertex on both halves. A curve
/// leaving into the half no earlier lookup named is then never tried —
/// silently, since a wrong pair just finds no direction. So this runs until
/// every pair of faces currently at the vertex has been tried once.
///
/// Measured twice. On `cube_minus_two_crossing_bores`, with the pierced
/// face taken from when the start point was found: the second bore's lower
/// curve in one quadrant went missing, the face it should have split was
/// dropped whole, and the solid was left open. And on
/// `cube_minus_two_inscribed_bores`, with both sides looked up once per
/// start point: at a cube-edge midpoint the first pair traced the circle
/// where the second bore leaves the cube, splitting its wall there, and the
/// Steinmetz arc leaving into the inner half was tried against the outer one,
/// rejected, and never traced.
fn trace_from_start_point(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    start: &TracingStartPoint,
    candidates: &[VertexId],
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!(
        "trace_from_start_point(vertex={}, edge_solid={}, face={})",
        start.vertex,
        start.edge_solid,
        start.face
    );

    let mut tried: std::collections::HashSet<(FaceId, FaceId)> = std::collections::HashSet::new();
    repeat(params.max_rewrites, || -> GeopResult<_> {
        let topology = model.topology();
        let faces_a = faces_at_vertex(topology, start.edge_solid, start.vertex, None, params)
            .with_context(&ctx)?;
        let faces_b = faces_at_vertex(
            topology,
            start.face_solid,
            start.vertex,
            Some(start.face),
            params,
        )
        .with_context(&ctx)?;
        let untried = faces_a
            .iter()
            .flat_map(|&face_a| faces_b.iter().map(move |&face_b| (face_a, face_b)))
            .find(|pair| !tried.contains(pair));
        let Some((face_a, face_b)) = untried else {
            return Ok(repeat::Step::Done(()));
        };
        tried.insert((face_a, face_b));
        trace_one_side(
            model,
            naming,
            start.vertex,
            face_a,
            face_b,
            candidates,
            params,
        )
        .with_context(with_context!("face_a={face_a}, face_b={face_b}"))
        .with_context(&ctx)?;
        Ok(repeat::Step::Again)
    })?
    .done("tracing from every pair of faces at the start point")
    .with_context(&ctx)
}

/// Every face of `solid` whose closure holds `vertex`, starting from
/// `recorded` (the face the vertex was found piercing, if any, which may
/// since have been split).
///
/// Topology answers this almost always: if the vertex lies on some faces'
/// boundaries, those are exactly the faces holding it — within one closed
/// shell a point on an edge is interior to no other face. Only a vertex on no
/// boundary needs geometry, and it is then interior to exactly one face: the
/// recorded one unless a split moved it into the other half, so that is
/// tested first and the rest only if it no longer holds the vertex.
fn faces_at_vertex(
    model: &Model,
    solid: Body,
    vertex: VertexId,
    recorded: Option<FaceId>,
    params: &RemeshParams,
) -> GeopResult<Vec<FaceId>> {
    let faces = model.body_faces(solid)?;
    let on_boundary: Vec<FaceId> = faces
        .iter()
        .copied()
        .filter(|&face_id| {
            model.iterate_face_coedges(face_id).any(|coedge_id| {
                match model.coedges.get(&coedge_id).map(|c| c.geometry) {
                    Some(CoedgeGeometry::Edge(edge_id)) => model
                        .edges
                        .get(&edge_id)
                        .is_some_and(|e| e.start_vertex == vertex || e.end_vertex == vertex),
                    Some(CoedgeGeometry::Vertex(v)) => v == vertex,
                    None => false,
                }
            })
        })
        .collect();
    if !on_boundary.is_empty() {
        return Ok(on_boundary);
    }

    let point = model.get_vertex(vertex)?.point;
    let holds = |face_id: FaceId| -> GeopResult<bool> {
        let surface = &model.get_face(face_id)?.surface;
        Ok(
            match surface_could_contain(
                surface,
                &point,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )? {
                Some((u, v)) => matches!(
                    face_contains(
                        model,
                        face_id,
                        u,
                        v,
                        params.max_nodes,
                        params.curve_curve_min_subdivision_size,
                        FACE_CONTAINS_SEED
                    )?,
                    PointClassification::Inside
                ),
                None => false,
            },
        )
    };
    if let Some(recorded) = recorded.filter(|f| faces.contains(f))
        && holds(recorded)?
    {
        return Ok(vec![recorded]);
    }
    for face_id in faces {
        if Some(face_id) != recorded && holds(face_id)? {
            return Ok(vec![face_id]);
        }
    }
    Ok(vec![])
}

/// Trace one branch of the intersection curve of `face_a` x `face_b`,
/// starting at vertex `v` (already known to lie on both), until it reaches
/// an existing vertex, then splice the result in as a new shared edge (see
/// [`splice_dangling_edge`]). A no-op if neither marching direction is
/// viable (see the per-direction rejection rules below) — that's a
/// legitimate outcome (e.g. a glancing touch), not an error.
#[allow(clippy::too_many_arguments)]
fn trace_one_side(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    v: VertexId,
    face_a: FaceId,
    face_b: FaceId,
    candidates: &[VertexId],
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!("trace_one_side(v={v}, face_a={face_a}, face_b={face_b})");

    let topology = model.topology();
    let point = topology.get_vertex(v).with_context(&ctx)?.point;
    let surf_a = topology
        .get_face(face_a)
        .with_context(&ctx)?
        .surface
        .clone();
    let surf_b = topology
        .get_face(face_b)
        .with_context(&ctx)?
        .surface
        .clone();

    let (u_a0, v_a0) = surface_could_contain(
        &surf_a,
        &point,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
    )
    .with_context(&ctx)?
    .ok_or_else(|| GeopError::new("trace_one_side: start vertex does not lie on face_a's surface"))
    .with_context(&ctx)?;
    let (u_b0, v_b0) = surface_could_contain(
        &surf_b,
        &point,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
    )
    .with_context(&ctx)?
    .ok_or_else(|| GeopError::new("trace_one_side: start vertex does not lie on face_b's surface"))
    .with_context(&ctx)?;
    // The normals are taken over the honest boxes, not sharpened ones: at a
    // pole only the box still touches the collapsed row, which is what
    // `normal` needs to recognise it (see `NurbSurface::normal`).
    //
    // A start point where either surface has no normal at all — the apex of
    // a cone, whose spokes span a cone rather than a plane — has no single
    // tangent direction for a curve to leave along: the curves through it
    // are generators, and no pair of normals can say which. Nothing is traced from here, and nothing is lost:
    // every intersection curve has two ends, both start points, so a curve
    // leaving an apex is traced from its other end, where it arrives at the
    // apex as a known vertex (see `candidate_within`). `(u, v)` came from
    // `surface_could_contain`, so it is in domain: a failing `normal` here
    // can only mean there is none.
    let (Ok(normal_a), Ok(normal_b)) = (surf_a.normal(u_a0, v_a0), surf_b.normal(u_b0, v_b0))
    else {
        return Ok(());
    };
    let (u_a0, v_a0) = (u_a0.sharpen(), v_a0.sharpen());
    let (u_b0, v_b0) = (u_b0.sharpen(), v_b0.sharpen());
    // A zero (or near-zero) cross product means `face_a`/`face_b` are
    // (locally) parallel here — there's no single well-defined tangent
    // direction for an intersection curve between two parallel surfaces,
    // so there's nothing to trace on this side. Not an error: `face_a` is
    // just one of up to two faces attached to the pierced edge, tried
    // speculatively — the *other* attached face (if any) may still be a
    // perfectly genuine transversal case.
    let Ok(axis) = normal_a.prod_cross(&normal_b).normalize() else {
        return Ok(());
    };

    // Pick the direction to march in. Only the *first* step needs this:
    // the intersection curve leaves `v` along +/-`axis`, and exactly one of
    // those two runs into the shared interior of the two trimmed faces
    // (the other immediately leaves at least one of them). `face_contains`
    // is the check for "is this `(u, v)` inside the *trimmed* face", as
    // opposed to merely inside the underlying surface patch's parameter
    // domain — a distinction the raw domain bounds can't make. Once a
    // direction is committed the march just follows the curve; re-testing
    // containment every step would only re-derive the same answer.
    let first_step =
        adaptive_step_size(&surf_a, &surf_b, u_a0, v_a0, u_b0, v_b0, &axis).with_context(&ctx)?;
    let mut last_rejection: Option<(PointClassification, PointClassification)> = None;
    // The trial step that decides the direction must not overshoot the curve.
    // A step leaving either trimmed face is rejected whichever way it goes, so
    // a curve shorter than one stride — its end vertex nearer than the step —
    // would be silently abandoned: on `flush_bored_cube_plus_inscribed_sphere_section`
    // a 0.0025-long segment of the section plane across the cube's top face
    // never got traced, the face was never split there, and a whole corner of
    // it was dropped with it. So when a vertex lying on both surfaces is
    // within the first step, a step of half its distance is tried first — it
    // stays inside both faces along such a curve, and the march then finds
    // that vertex within reach at once. One extra attempt, and only then;
    // halving the step blindly cost every start point no curve leaves a dozen.
    let near = candidate_within(
        topology, candidates, v, &point, first_step, None, &surf_a, &surf_b, params,
    )
    .with_context(&ctx)?;
    // That vertex lies on both surfaces, so on the curve, on one side of `v`:
    // the curve leaving that way ends there. The full step must not decide
    // that side — it lands past the vertex, and where the half step was
    // rejected because the curve runs along a boundary up to the vertex, it
    // would carry the trace over the vertex and splice a duplicate of that
    // boundary.
    let mut trials = Vec::with_capacity(2);
    let mut toward_near = None;
    if let Some(w) = near {
        let offset = topology.get_vertex(w).with_context(&ctx)?.point.sub(&point);
        trials.push((offset.norm().div(Scalar::TWO).with_context(&ctx)?, None));
        let along = offset.prod_dot(&axis);
        toward_near = if along.definitely_greater(Scalar::ZERO) {
            Some(true)
        } else if along.definitely_less(Scalar::ZERO) {
            Some(false)
        } else {
            None
        };
    }
    trials.push((first_step, toward_near));
    // Every trial step, each way it may go, in turn: the first that lands
    // inside both faces decides the direction.
    let tries: Vec<(Scalar, Scalar)> = (trials.iter())
        .flat_map(|&(trial, not_toward)| {
            [(true, Scalar::ONE), (false, Scalar::ONE.neg())]
                .into_iter()
                .filter(move |&(forward, _)| not_toward != Some(forward))
                .map(move |(_, sign)| (trial, sign))
        })
        .collect();
    let chosen = attempts(tries.len(), |&k| -> GeopResult<_> {
        let (trial, sign) = tries[k];
        let dir = axis.prod_scalar(sign);
        // A step in the wrong direction leaves one of the two patches, so its
        // two projections land somewhere different and
        // `predictor_corrector_step` rejects them as disagreeing. During
        // *selection* that isn't a failure — it's precisely the signal that
        // this is the wrong way to go — so try the other sign. Once a
        // direction is committed the same disagreement is a real error (the
        // march lost the curve), and is propagated below.
        let (next_point, na, va, nb, vb) = match predictor_corrector_step(
            &surf_a, &surf_b, point, dir, u_a0, v_a0, u_b0, v_b0, trial,
        ) {
            Ok(step) => step,
            Err(_) => return Ok(Probe::Next),
        };
        let inside_a = face_contains(
            topology,
            face_a,
            na,
            va,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            FACE_CONTAINS_SEED,
        )
        .with_context(&ctx)?;
        let inside_b = face_contains(
            topology,
            face_b,
            nb,
            vb,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            FACE_CONTAINS_SEED,
        )
        .with_context(&ctx)?;
        if inside_a == PointClassification::Inside && inside_b == PointClassification::Inside {
            return Ok(Probe::Found((
                next_point, na, va, nb, vb, dir, inside_a, inside_b,
            )));
        }
        last_rejection = Some((inside_a, inside_b));
        Ok(Probe::Next)
    })?;

    // Neither direction viable means no intersection curve emanates from
    // this start point, which is a legitimate and common outcome rather
    // than a failure: `find_tracing_start_points` deliberately tests each
    // face's *untrimmed* surface (see its own doc comment), so it reports
    // start points that no curve actually leaves — two cubes meeting at a
    // single corner produce one for every face pair through that corner,
    // and marching either way immediately leaves one of the two patches.
    //
    // The cost of returning quietly here is that a trace which *should*
    // have produced a curve but didn't looks identical to this. That is a
    // real hazard — it once made a whole sweep look like it was passing
    // while every trace gave up — so it is guarded from the outside
    // instead: `box_cylinder_blind_hole_remesh_succeeds` asserts that
    // tracing produced edges shared between both solids' faces, which a
    // dead trace cannot fake. Do not weaken that assertion to a plain edge
    // count; splitting alone creates edges, so a count still passes when
    // nothing is traced at all.
    let Some((mut cur_point, mut u_a, mut v_a, mut u_b, mut v_b, mut dir, first_a, first_b)) =
        chosen
    else {
        return Ok(());
    };
    // Which classification committed this direction — the fact that decides
    // whether a curve is entitled to be spliced at all. A curve that runs
    // along a face's boundary rather than through it cannot have been
    // `Inside`, so if a splice below reports a degenerate split, this says
    // whether the direction choice was wrong or something later moved the
    // curve onto the boundary.
    let (first_ua, first_va, first_ub, first_vb, first_dir) = (u_a, v_a, u_b, v_b, dir);
    let ctx = |e: GeopError| {
        ctx(e).with_context(format!(
            "from {point:?} along {first_dir:?}, direction chosen with face_a={first_a:?} at uv=({first_ua:?}, {first_va:?}), face_b={first_b:?} at uv=({first_ub:?}, {first_vb:?}), rejected direction saw {last_rejection:?}"
        ))
    };

    let mut points = vec![point, cur_point];
    // Each point's `(u, v)` on both surfaces, kept alongside so the true
    // intersection between two points can be located again after the march
    // (for `interpolate_enclosing` below).
    let mut uvs = vec![(u_a0, v_a0, u_b0, v_b0), (u_a, v_a, u_b, v_b)];

    // March until we reach another vertex: one on the stretch of curve the
    // next stride runs along, from `cur_point` up to the plane at `step`
    // along `dir` that the corrector lands it on (see `candidate_within`).
    let marched = repeat(params.max_trace_steps, || -> GeopResult<_> {
        let step =
            adaptive_step_size(&surf_a, &surf_b, u_a, v_a, u_b, v_b, &dir).with_context(&ctx)?;
        let reached = |radius: Scalar| {
            candidate_within(
                topology,
                candidates,
                v,
                &cur_point,
                radius,
                Some((&dir, step)),
                &surf_a,
                &surf_b,
                params,
            )
        };
        // Every point of the plane the stride lands on is at least `step`
        // from `cur_point`, so a vertex ahead within `step` is reached
        // whatever the stride does — and is found without taking it, which
        // matters where the curve ends at the edge of a patch the stride
        // would have to leave.
        if let Some(hit) = reached(step).with_context(&ctx)? {
            return Ok(repeat::Step::Done(hit));
        }
        // Failing, it says which vertex on both surfaces lies nearest within
        // a few strides, if any: where the curve leaves a patch it should end
        // at one, and one missing, or out of reach, is what lost the trace.
        let ahead = |e: GeopError| {
            let nearest = reached(step.mul(Scalar::from_f64(4.0)));
            let seen = match nearest {
                Ok(Some(w)) => match topology.get_vertex(w) {
                    Ok(vertex) => {
                        let offset = vertex.point.sub(&cur_point);
                        format!(
                            "the nearest vertex on both surfaces within four strides is {w} at {:?}, {:?} away, {:?} along the stride",
                            vertex.point,
                            offset.norm(),
                            offset.prod_dot(&dir)
                        )
                    }
                    Err(_) => format!("{w}"),
                },
                Ok(None) => "no vertex on both surfaces lies within four strides".to_string(),
                Err(found) => format!("looking for a vertex ahead failed: {found}"),
            };
            ctx(e.with_context(seen))
        };
        let stride =
            predictor_corrector_step(&surf_a, &surf_b, cur_point, dir, u_a, v_a, u_b, v_b, step);
        let (next_point, na, va, nb, vb) = match stride {
            Ok(stride) => stride,
            // Where the curve bends, it can end at a vertex short of the
            // plane yet farther than `step`, and leave a patch there that
            // the stride, landing on the plane, would have to leave too. So
            // when the stride fails, a vertex short of the plane counts
            // within the chord of a curve turning less than an eighth of a
            // revolution over it, as the check after a stride counts it
            // within the chord it took. A tapered thread's flank met a
            // hole's wall `5e-5` outside the ball at the end of a quarter.
            Err(e) => match reached(step.mul(Scalar::from_f64(std::f64::consts::SQRT_2)))
                .with_context(&ctx)?
            {
                Some(hit) => return Ok(repeat::Step::Done(hit)),
                None => return Err(ahead(e)),
            },
        };
        // The rest of the stride: where the curve bends away from `dir` it
        // meets the plane farther than `step` away, and a vertex in between
        // lies ahead of the plane but outside a ball of radius `step`. Within
        // the chord, though, since along an arc turning less than half a
        // revolution the distance from its start only grows. Checking the
        // ball alone, the stride stepped over such a vertex: on
        // `chained_differences_block_with_two_slots_and_a_sphere` the
        // sphere's circle across the first slot's floor ran past the corner
        // of the second slot it ends at.
        if let Some(hit) = reached(next_point.sub(&cur_point).norm()).with_context(&ctx)? {
            return Ok(repeat::Step::Done(hit));
        }
        cur_point = next_point;
        (u_a, v_a, u_b, v_b) = (na, va, nb, vb);
        points.push(cur_point);
        uvs.push((na, va, nb, vb));

        // Re-derive direction from the new normals for the next step,
        // keeping continuity with the previous direction.
        if let (Ok(na), Ok(nb)) = (surf_a.normal(u_a, v_a), surf_b.normal(u_b, v_b))
            && let Ok(new_axis) = na.prod_cross(&nb).normalize()
        {
            dir = if new_axis.prod_dot(&dir).could_be_greater(Scalar::ZERO) {
                new_axis
            } else {
                new_axis.neg()
            };
        }
        Ok(repeat::Step::Again)
    })?;

    // An intersection curve of two faces has to end at a vertex — every
    // place it can end is a point where one face's boundary pierces the
    // other, which this pass has already split a vertex into. Ending
    // anywhere else means the trace lost the curve.
    let hit_vertex = marched.done_or(|_| {
        ctx(GeopError::new(format!(
            "traced {} step(s) from {cur_point:?} without reaching another vertex",
            points.len()
        )))
    })?;
    // Land the traced polyline exactly on the vertex it ends at, so the
    // fitted curve's endpoint and the edge's `end_vertex` agree.
    let hit_point = topology.get_vertex(hit_vertex).with_context(&ctx)?.point;
    *points.last_mut().expect("points is never empty") = hit_point;
    // Which marched point is the widest, and how wide: a trace that ends
    // wide went wrong somewhere along it, and this says where.
    let widest = points
        .iter()
        .enumerate()
        .map(|(i, p)| (i, (0..3).map(|k| p[k].width().to_f64()).fold(0.0, f64::max)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("points is never empty");
    let ctx = |e: GeopError| {
        ctx(e).with_context(format!(
            "traced {} step(s) to vertex {hit_vertex} at {hit_point:?}; its widest point is {} at {:?}, {:e} wide",
            points.len(),
            widest.0,
            points[widest.0],
            widest.1,
        ))
    };

    // Is this curve already in the topology? Every intersection branch has two
    // ends and both are piercing points, so both land in `starts` and each
    // branch is traced twice — the second time it must find the first rather
    // than lay a second edge along the same path, which would splice to a
    // sliver of no area.
    //
    // Asked *after* marching, because only the finished curve answers it. The
    // question is whether some existing edge runs along the same intersection
    // branch, and one point near `v` cannot say: two branches crossing at `v`
    // agree there to within a step, and an edge shorter than one step has
    // nothing near the corrected point at all. Both ends and an interior point
    // together pin the branch down — the endpoints separate branches that go
    // different places, and the interior point separates two distinct branches
    // that happen to share both ends (an arc and its complement).
    //
    // Tracing first and discarding the result is the price of asking a
    // geometric question geometrically. It costs a march, and only in the
    // duplicate case.
    let mid = points[points.len() / 2];
    let duplicate = topology
        .edges
        .iter()
        .map(|(&id, edge)| {
            let same_ends = (edge.start_vertex == v && edge.end_vertex == hit_vertex)
                || (edge.start_vertex == hit_vertex && edge.end_vertex == v);
            if !same_ends {
                return Ok(None);
            }
            curve_could_contain(
                &edge.curve,
                &mid,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )
            .map(|hit| hit.map(|_| id))
        })
        .collect::<GeopResult<Vec<_>>>()
        .with_context(&ctx)?
        .into_iter()
        .flatten()
        // Lowest id, not whichever the hash map yields first: choosing
        // arbitrarily would make remesh depend on hash order.
        .min_by_key(|id| id.0);

    // The widest point of the branch located between the marched ones —
    // leg, fraction, point, width — of the last attempt to fit it: the curve
    // is widened to hold it, so one that is wide makes the whole curve wide,
    // and this says which.
    let mut widest_between: Option<(usize, usize, Vector3, f64)> = None;
    let mut fitted = String::new();
    let edge_id = match duplicate {
        Some(id) => id,
        None => {
            let curve = fit_branch(&surf_a, &surf_b, &points, &uvs, &mut widest_between, &ctx)?;
            let widest = curve
                .control_points
                .iter()
                .enumerate()
                .map(|(i, p)| (i, (0..4).map(|k| p[k].width().to_f64()).fold(0.0, f64::max)))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .expect("a curve has control points");
            fitted = format!(
                "{} control points, the widest {} of them, {:e} wide",
                curve.control_points.len(),
                widest.0,
                widest.1
            );
            let edge = model
                .insert_edge(
                    Edge {
                        curve,
                        start_vertex: v,
                        end_vertex: hit_vertex,
                    },
                    naming.provisional(),
                )
                .with_context(&ctx)?;
            naming.trace(edge, face_a, face_b, [v, hit_vertex])?;
            edge
        }
    };

    let ctx = |e: GeopError| {
        ctx(e).with_context(format!(
            "the curve through {} marched points is {fitted}; its widest true point between marched ones: {widest_between:?}",
            points.len()
        ))
    };

    // Common to both: whichever edge carries this curve, each face needs a
    // coedge for it, and only if it does not already have one. A traced edge
    // has neither yet; an existing one may already be imprinted on one face
    // and not the other — the splitting stage creates edges that were never
    // imprinted, and a curve traced from the other end carries only its own
    // two faces.
    for face in [face_a, face_b] {
        if edge_is_boundary_of_face(model.topology(), edge_id, face) {
            continue;
        }
        let new_face = model
            .splice_edge_into_face(
                edge_id,
                face,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
                naming.provisional(),
            )
            .with_context(&ctx)?;
        naming.face_split(face, edge_id, new_face)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::remesh::{
        remesh_edges_x_edges::remesh_edges_x_edges, remesh_vertices::remesh_vertices,
        remesh_vertices_x_edges::remesh_vertices_x_edges,
    };
    use crate::scenes::unit_cube_at;
    use geop_ops_model::Namer;

    /// Two unit cubes offset by half a cube-width along x: a simple, fully
    /// planar scene (no curved surfaces at all) exercising every phase —
    /// coincident faces get imprinted, and the two solids' silhouettes
    /// (where one cube's edge pierces the other's face) get traced.
    #[test]
    fn box_grid_offset_x_half_edges_x_faces() {
        let mut model = NamedModel::new();
        let solid_a = unit_cube_at(&mut model, "a", 0.0, 0.0, 0.0).into();
        let solid_b = unit_cube_at(&mut model, "b", 0.5, 0.0, 0.0).into();
        let namer = Namer::new("boolean", "ab").unwrap();
        let mut naming = BooleanNaming::new(&model, &namer, &[solid_a, solid_b]).unwrap();
        let params = RemeshParams {
            max_edge_intersections: 17,
            max_trace_steps: 200,
            ..RemeshParams::default()
        };

        remesh_vertices(&mut model, solid_a, solid_b).unwrap();
        remesh_vertices_x_edges(&mut model, &mut naming, solid_a, solid_b, &params).unwrap();
        remesh_vertices_x_edges(&mut model, &mut naming, solid_b, solid_a, &params).unwrap();
        remesh_edges_x_edges(&mut model, &mut naming, solid_a, solid_b, &params).unwrap();

        let edges_before = model.topology().edges.len();
        let coedges_before = model.topology().coedges.len();

        remesh_edges_x_faces(&mut model, &mut naming, solid_a, solid_b, &params).unwrap();
        naming.finish(&mut model).unwrap();
        model.check_names().unwrap();
        let topology = model.topology();

        eprintln!(
            "edges: {} -> {}, coedges: {} -> {}",
            edges_before,
            topology.edges.len(),
            coedges_before,
            topology.coedges.len()
        );

        std::fs::create_dir_all("outputs").unwrap();
        match geop_core_rasterize::debug::rasterize_topology(topology, 12) {
            Ok(render) => render
                .save_to_file("outputs/remesh_edges_x_faces_box_grid_offset_x_half.html")
                .unwrap(),
            Err(e) => eprintln!("rasterize failed (tracing itself succeeded): {e}"),
        }
    }
}

#[cfg(test)]
mod splice_regression_tests {
    use crate::scenes::{remeshed, scene, validation};
    use geop_core_topology::validation::{validate, validate_fast};

    /// Isolated from the 175-scene sweep: a cylinder drilled part-way into
    /// a box, so the cylinder's curved wall genuinely intersects the box's
    /// flat faces — the case where marching has to follow a *curved*
    /// intersection curve rather than a straight one, and where a fixed
    /// step size lets the trace wander off the surfaces entirely.
    #[test]
    fn box_cylinder_blind_hole_remesh_succeeds() {
        let scene = remeshed("box_cylinder_blind_hole");

        // The cylinder wall genuinely cuts the box's faces here, so tracing
        // *must* have produced intersection edges. The decisive signature of
        // a traced edge — as opposed to one merely split out of an existing
        // edge by `split_piercing_crossings` — is that it is spliced into a
        // face of *each* solid, so its coedges span both. A plain edge count
        // wouldn't distinguish the two, and so would still pass if every
        // trace gave up, which is indistinguishable from success in
        // `remesh`'s return value alone.
        let faces_a = scene.model.topology().solid_faces(scene.solid_a).unwrap();
        let faces_b = scene.model.topology().solid_faces(scene.solid_b).unwrap();
        let shared = scene
            .model
            .topology()
            .edges
            .keys()
            .filter(|&&edge_id| {
                let faces: Vec<_> = scene
                    .model
                    .topology()
                    .coedges_of_edge(edge_id)
                    .into_iter()
                    .filter_map(|c| scene.model.topology().get_coedge(c).ok().map(|c| c.face))
                    .collect();
                faces.iter().any(|f| faces_a.contains(f))
                    && faces.iter().any(|f| faces_b.contains(f))
            })
            .count();
        assert!(
            shared > 0,
            "no edge is shared between the two solids' faces — tracing produced nothing"
        );
    }

    /// Remesh must *split* a face that an intersection curve cuts across,
    /// not merely hang another loop on it. With proper face topology a loop
    /// joining two points of a face's outer boundary divides the material in
    /// two, so both halves have to become faces of their own — the old
    /// generic-boundary model instead left one face carrying several
    /// disjoint patches, which is not a valid trimmed face.
    ///
    /// Checked structurally rather than by counting: every face must be
    /// bounded by a real loop (not a bare vertex), and no face may be left
    /// holding a second outer-like ring, which is exactly what
    /// `validate_fast` plus the face-count growth below assert.
    #[test]
    fn remesh_splits_faces_rather_than_accumulating_loops() {
        let name = "box_cylinder_drilled_hole_through";
        let before = scene(name).model.topology().faces.len();
        let scene = remeshed(name);
        let after = scene.model.topology().faces.len();

        assert!(
            after > before,
            "remesh must split faces the intersection curve cuts across: {before} faces before, {after} after"
        );
        for (&face_id, face) in &scene.model.topology().faces {
            assert!(
                matches!(
                    face.outer,
                    geop_core_topology::boundary::BoundaryType::Loop(_)
                ),
                "face {face_id} is left bounded by {:?} rather than a loop",
                face.outer
            );
        }
    }

    /// Isolated from the sweep: the last scene whose model still fails
    /// `validate_fast` after a successful remesh. A cylinder passes through
    /// the neck of a figure-8 profile and overhangs it, so edges meet faces
    /// well outside the region the two solids actually share.
    #[test]
    fn figure8_cylinder_through_neck_oversized_is_valid_after_remesh() {
        let scene = remeshed("figure8_cylinder_through_neck_oversized");
        if let Err(errors) = validate_fast(&validation(), scene.model.topology()) {
            let all: Vec<String> = errors.iter().map(|e| format!("{e}")).collect();
            panic!(
                "{} validate_fast error(s):\n{}",
                errors.len(),
                all.join("\n---\n")
            );
        }
    }

    /// The two `figure8_cylinder_engulfing` arrangements, held to the **full**
    /// `validate` rather than `validate_fast`.
    ///
    /// Both render as a mess after remesh — coedges that stop half-way,
    /// missing edges (V437 to V453 in `thin_slice`), a hole that never
    /// closes — while passing every structural check. That is the point of
    /// running the full validation here: `validate_fast` only asks whether
    /// the pointers, loops and pcurve endpoints are self-consistent, which a
    /// model missing an edge entirely still is. The geometric checks —
    /// pairwise disjointness, face x face intersection, holes inside their
    /// outer loop — are what can notice that two faces still cross where no
    /// edge records it.
    ///
    /// A cylinder engulfing a thin slice of one lobe, and one engulfing both
    /// lobes: in each the cylinder swallows a whole piece of the figure-8, so
    /// the intersection curves close on themselves rather than running
    /// between piercing points, which is the case tracing is weakest at.
    fn check_engulfing_scene_fully_valid(name: &str) {
        let scene = remeshed(name);
        if let Err(errors) = validate(&validation(), scene.model.topology()) {
            let all: Vec<String> = errors.iter().map(|e| format!("{e}")).collect();
            panic!(
                "{name}: {} validate error(s):\n{}",
                errors.len(),
                all.join("\n---\n")
            );
        }
    }

    #[test]
    #[ignore = "slow: 4 s — run with `cargo test -- --ignored`"]
    fn figure8_cylinder_engulfing_thin_slice_is_fully_valid_after_remesh() {
        check_engulfing_scene_fully_valid("figure8_cylinder_engulfing_thin_slice");
    }

    #[test]
    fn figure8_cylinder_engulfing_both_lobes_is_fully_valid_after_remesh() {
        check_engulfing_scene_fully_valid("figure8_cylinder_engulfing_both_lobes");
    }

    /// This was the regression case for `split_edge_at_vertex` splitting at a
    /// *sharpened* `edge_t`: it validated the vertex against the wide
    /// parameter but cut at that interval's midpoint, so the two halves met
    /// at `C(t_mid)` rather than at `C(t*) = vertex_point` — a different
    /// point on the same curve, off by `|t_mid - t*| x |C'(t)|` (~1.8e-8
    /// here). Simply dropping the sharpen did not fix it, because Boehm
    /// insertion cannot take a `min_subdivision_size`-wide parameter either.
    /// Newton-refining both parameters against what located them is what
    /// resolved both halves at once.
    #[test]
    fn box_cylinder_corner_quarter_overlap_is_valid_after_remesh() {
        let scene = remeshed("box_cylinder_corner_quarter_overlap");
        if let Err(errors) = validate_fast(&validation(), scene.model.topology()) {
            panic!("{} validate_fast error(s): {}", errors.len(), errors[0]);
        }
    }

    /// Two unit cubes offset by `(-1, -0.5, -0.5)`: they meet face-to-face
    /// along `x`, so the pair is entirely planar and coincident, exercising
    /// the imprint path (and `splice_edge_into_face`'s loop restructuring)
    /// with no curved geometry involved at all.
    #[test]
    fn box_grid_n1p00_n0p50_n0p50_is_valid_after_remesh() {
        let scene = remeshed("box_grid_n1p00_n0p50_n0p50");
        if let Err(errors) = validate_fast(&validation(), scene.model.topology()) {
            panic!("{} validate_fast error(s): {errors:?}", errors.len());
        }
    }
}
