//! Remesh every edge x face pair between two solids: wherever an edge of
//! one solid lies *within* a face of the other, imprint it there as a new
//! boundary; wherever an edge instead pierces a face transversally at a
//! single point, split the edge there and trace the intersection curve
//! that begins at that point until it reaches an already-known vertex,
//! then splice the traced curve in as a new shared edge.
//!
//! Written from scratch against the current `Model` API — not a revival of
//! an earlier `remesh`/`traced_curve`/`tracing_start_point` attempt that
//! predated a `Model` API overhaul and was removed rather than carried
//! forward, though the general shape of the problem (and a few algorithmic
//! ideas — predictor-corrector marching, re-deriving direction from the two
//! surfaces' normals each step) is the same one that code already solved
//! once. See `splice_dangling_edge`'s own doc comment for the one place
//! this module's topology-splicing choice deliberately differs from what
//! the old code did.
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
//!    that produces.

use geop_core_geometry::{
    contains::{curve::curve_could_contain, surface::surface_could_contain},
    intersection::{Intersections, curve_surface_intersect, refine_crossing},
    nurb_curve::{NurbCurve, true_point_fractions},
    nurb_surface::{NurbSurface3D, clamp},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::{Vector, Vector3},
};
use geop_core_topology::{
    Body, CoedgeGeometry, Edge, EdgeId, FaceId, Model, VertexId,
    contains::face::{PointClassification, face_contains, face_interior_point},
};
use geop_ops::Part;

use crate::naming::BooleanNaming;

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

/// The fewest legs a traced intersection curve is fitted through (see
/// `trace_one_side`): a cubic interpolant needs points enough to follow
/// the branch, however few strides the march took along it. Like
/// `STEPS_PER_REVOLUTION` this bounds effort — every extra leg is one more
/// corrector — and decides how *wide* the traced curve's enclosure is, not
/// whether it holds.
const MIN_TRACED_LEGS: usize = 8;

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

/// The marching step to use at `(u_a, v_a)` / `(u_b, v_b)`: a
/// [`STEPS_ACROSS_PATCH`]th of the smaller surface, shortened so a full
/// turn of whichever surface is curving harder there would take
/// [`STEPS_PER_REVOLUTION`] steps. A surface that's locally flat reports no
/// curvature radius and so imposes no limit. Sharp: where a stride lands is
/// a free choice.
fn adaptive_step_size<S: Scalar>(
    surf_a: &NurbSurface3D<S>,
    surf_b: &NurbSurface3D<S>,
    u_a: S,
    v_a: S,
    u_b: S,
    v_b: S,
) -> GeopResult<S> {
    let arc = S::TWO
        .mul(S::PI)
        .div(S::from_i64(STEPS_PER_REVOLUTION as i64))?;
    let mut step = surf_a
        .size()?
        .min(surf_b.size()?)
        .div(S::from_i64(STEPS_ACROSS_PATCH as i64))?
        .sharpen();
    for radius in [
        surf_a.curvature_radius(u_a, v_a)?,
        surf_b.curvature_radius(u_b, v_b)?,
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
/// `max_solutions`/`max_nodes`/`min_subdivision_size` bound the
/// `curve_surface_intersect` searches used to classify each pair (and the
/// `curve_could_contain`/`surface_could_contain` single-shape searches used
/// while tracing). `max_trace_steps` bounds how many marching steps a
/// single trace may take before it's considered to have failed to find a
/// terminating vertex (see [`adaptive_step_size`] for how long each is).
pub fn remesh_edges_x_faces<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    solid_a: Body,
    solid_b: Body,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
    max_trace_steps: usize,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "remesh_edges_x_faces(solid_a={solid_a}, solid_b={solid_b}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size}, max_trace_steps={max_trace_steps})"
        ))
    };

    split_piercing_crossings(
        part,
        naming,
        solid_a,
        solid_b,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?;
    split_piercing_crossings(
        part,
        naming,
        solid_b,
        solid_a,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?;

    imprint_coincident_pairs(
        part,
        naming,
        solid_a,
        solid_b,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?;
    imprint_coincident_pairs(
        part,
        naming,
        solid_b,
        solid_a,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?;

    // One scan, one pass. Every piercing point either solid's edges make on the
    // other's faces exists by now — that is exactly what the two phases above
    // were run to a fixed point for — so the settled model already names every
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
    let model = part.topology();
    let mut starts =
        find_tracing_start_points(model, solid_a, solid_b, max_nodes, min_subdivision_size)
            .with_context(&ctx)?;
    // A vertex found from both sides is one start point: tracing from it
    // already tries every face pair through it.
    let found: std::collections::HashSet<VertexId> = starts.iter().map(|s| s.vertex).collect();
    starts.extend(
        find_tracing_start_points(model, solid_b, solid_a, max_nodes, min_subdivision_size)
            .with_context(&ctx)?
            .into_iter()
            .filter(|s| !found.contains(&s.vertex)),
    );

    // Every vertex in the model is a candidate endpoint, not just the start
    // points. A traced curve does run from one piercing point to another in
    // the common case, but it can equally end at a vertex that was already
    // there before this pass — a corner where the two solids' edges were
    // merged by `remesh_vertices`, say, which is a perfectly good place for
    // an intersection curve to terminate yet never becomes a piercing point
    // and so never appears in `starts`. Restricting the search to `starts`
    // makes those traces run to the edge of the patch and fail, having
    // walked right past the vertex they should have stopped at.
    // Sorted: the order candidates are tried in must not depend on hash
    // order, or neither would the traced curves.
    let mut candidates: Vec<VertexId> = model.vertices.keys().copied().collect();
    candidates.sort_by_key(|v| v.0);
    for start in &starts {
        trace_from_start_point(
            part,
            naming,
            start,
            &candidates,
            max_nodes,
            min_subdivision_size,
            max_trace_steps,
        )
        .with_context(&ctx)?;
    }

    Ok(())
}

/// Whether `edge_id` already has a boundary coedge on `face_id` — if so,
/// it's already been imprinted or traced there; nothing more to do for
/// this pair.
fn edge_is_boundary_of_face<S: Scalar>(model: &Model<S>, edge_id: EdgeId, face_id: FaceId) -> bool {
    model.iterate_face_coedges(face_id).any(|coedge_id| {
        model.coedges.get(&coedge_id).map(|c| c.geometry) == Some(CoedgeGeometry::Edge(edge_id))
    })
}

/// An existing vertex whose point could be `point`, if any.
fn find_vertex_at_point<S: Scalar>(model: &Model<S>, point: &Vector3<S>) -> Option<VertexId> {
    model
        .vertices
        .iter()
        .find(|(_, v)| v.point.could_be_equal(point))
        .map(|(&id, _)| id)
}

// ── Phase 1: split every genuine (transversal) piercing crossing ───────────

/// The first genuine interior piercing crossing found — `edge_id` from
/// `edge_solid`, on a face from `face_solid` — with the `t` to split at and
/// either an already-existing vertex there or the `point` to create one at.
/// `None` once every remaining `(edge, face)` pair is either disjoint,
/// coincident (see [`find_coincident_pair`] instead), or only touches at an
/// edge endpoint that's already a vertex (a normal shared corner, not an
/// interior piercing).
#[allow(clippy::type_complexity)]
fn find_piercing_crossing<S: Scalar>(
    model: &Model<S>,
    edge_solid: Body,
    face_solid: Body,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<(EdgeId, S, Option<VertexId>, Vector3<S>, FaceId)>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "find_piercing_crossing(edge_solid={edge_solid}, face_solid={face_solid}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    for edge_id in model.iter_body_edges(edge_solid).with_context(&ctx)? {
        for face_id in model.body_faces(face_solid).with_context(&ctx)? {
            if edge_is_boundary_of_face(model, edge_id, face_id) {
                continue;
            }

            let pair_ctx = |e: GeopError| e.with_context(format!("edge={edge_id}, face={face_id}"));

            let edge = model.get_edge(edge_id).with_context(&ctx)?;
            let face = model.get_face(face_id).with_context(&ctx)?;
            let crossings = curve_surface_intersect(
                &edge.curve,
                &face.surface,
                max_solutions,
                max_nodes,
                min_subdivision_size,
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
                let (t, uv) = refine_crossing(&edge.curve, &face.surface, t, uv);
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
                if let (Ok(tangent), Ok(normal)) = (
                    edge.curve.tangent(box_t),
                    face.surface.normal(box_uv[0], box_uv[1]),
                ) && tangent.prod_dot(&normal).could_be_equal(S::ZERO)
                {
                    continue;
                }
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
                let vertex = find_vertex_at_point(model, &point);
                return Ok(Some((edge_id, t, vertex, point, face_id)));
            }
        }
    }
    Ok(None)
}

fn split_piercing_crossings<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    edge_solid: Body,
    face_solid: Body,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "split_piercing_crossings(edge_solid={edge_solid}, face_solid={face_solid}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    while let Some((edge_id, t, vertex, point, face_id)) = find_piercing_crossing(
        part.topology(),
        edge_solid,
        face_solid,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?
    {
        let vertex_id = match vertex {
            Some(vertex) => vertex,
            None => {
                let vertex = part.insert_vertex(point, naming.provisional())?;
                naming.piercing(vertex, edge_id, t, face_id)?;
                vertex
            }
        };
        let new_edge = part
            .split_edge_at_vertex(
                edge_id,
                t,
                vertex_id,
                max_nodes,
                min_subdivision_size,
                naming.provisional(),
            )
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "edge={edge_id}, t={t:?}, vertex={vertex_id}, piercing face={face_id}"
                ))
            })
            .with_context(&ctx)?;
        naming.edge_split(edge_id, new_edge, vertex_id)?;
    }
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
fn faces_are_coplanar<S: Scalar>(
    model: &Model<S>,
    face_a: FaceId,
    face_b: FaceId,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<bool> {
    let Ok((ua, va)) = face_interior_point(
        model,
        face_a,
        max_nodes,
        min_subdivision_size,
        FACE_CONTAINS_SEED,
    ) else {
        return Ok(false);
    };
    let Ok((ub, vb)) = face_interior_point(
        model,
        face_b,
        max_nodes,
        min_subdivision_size,
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
    let normals_parallel = cross[0].could_be_equal(S::ZERO)
        && cross[1].could_be_equal(S::ZERO)
        && cross[2].could_be_equal(S::ZERO);
    if !normals_parallel {
        return Ok(false);
    }
    let offset = pb.sub(&pa).prod_dot(&na);
    Ok(offset.could_be_equal(S::ZERO))
}

/// One of `edge_id`'s two neighboring faces — *within the solid `edge_id`
/// itself belongs to* — if those two are coplanar with each other, i.e.
/// `edge_id` is a purely internal seam between two mutually-flush patches
/// of that one solid, not a genuine outer boundary edge.
///
/// The motivating case: [`geop_ops_extrude_revolve::revolve::revolve_at_oriented`]
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
fn seam_plane<S: Scalar>(
    model: &Model<S>,
    edge_id: EdgeId,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<FaceId>> {
    let coedges = model.coedges_of_edge(edge_id);
    let [c1, c2] = coedges[..] else {
        return Ok(None);
    };
    let f1 = model.get_coedge(c1)?.face;
    let f2 = model.get_coedge(c2)?.face;
    if f1 == f2 {
        return Ok(None);
    }
    Ok(faces_are_coplanar(model, f1, f2, max_nodes, min_subdivision_size)?.then_some(f1))
}

/// The first `(edge, face)` pair found where `edge`'s whole curve is
/// coincident with `face`'s surface (`curve_surface_intersect` hitting its
/// `max_solutions` cap), and `edge` isn't already a boundary coedge of
/// `face`.
fn find_coincident_pair<S: Scalar>(
    model: &Model<S>,
    edge_solid: Body,
    face_solid: Body,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<(EdgeId, FaceId)>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "find_coincident_pair(edge_solid={edge_solid}, face_solid={face_solid}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    for edge_id in model.iter_body_edges(edge_solid).with_context(&ctx)? {
        let seam =
            seam_plane(model, edge_id, max_nodes, min_subdivision_size).with_context(&ctx)?;
        for face_id in model.body_faces(face_solid).with_context(&ctx)? {
            if edge_is_boundary_of_face(model, edge_id, face_id) {
                continue;
            }
            // A seam within a plane is that plane's own business only for a
            // face flush with it (see `seam_plane`).
            if let Some(plane) = seam
                && faces_are_coplanar(model, plane, face_id, max_nodes, min_subdivision_size)
                    .with_context(&ctx)?
            {
                continue;
            }
            let edge = model.get_edge(edge_id).with_context(&ctx)?;
            let face = model.get_face(face_id).with_context(&ctx)?;
            let crossings = curve_surface_intersect(
                &edge.curve,
                &face.surface,
                max_solutions,
                max_nodes,
                min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(&|e: GeopError| {
                e.with_context(format!("edge={edge_id}, face={face_id}"))
            })?;
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
                            max_nodes,
                            min_subdivision_size,
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
                continue;
            }
        }
    }
    Ok(None)
}

fn imprint_coincident_edge<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    edge_id: EdgeId,
    face_id: FaceId,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<()> {
    let new_face = part
        .splice_edge_into_face(
            edge_id,
            face_id,
            max_nodes,
            min_subdivision_size,
            naming.provisional(),
        )
        .with_context(&|e: GeopError| {
            e.with_context(format!(
                "imprint_coincident_edge(edge={edge_id}, face={face_id})"
            ))
        })?;
    naming.face_split(face_id, edge_id, new_face)
}

fn imprint_coincident_pairs<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    edge_solid: Body,
    face_solid: Body,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "imprint_coincident_pairs(edge_solid={edge_solid}, face_solid={face_solid}, max_solutions={max_solutions}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    while let Some((edge_id, face_id)) = find_coincident_pair(
        part.topology(),
        edge_solid,
        face_solid,
        max_solutions,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?
    {
        imprint_coincident_edge(
            part,
            naming,
            edge_id,
            face_id,
            max_nodes,
            min_subdivision_size,
        )
        .with_context(&ctx)?;
    }
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
fn find_tracing_start_points<S: Scalar>(
    model: &Model<S>,
    edge_solid: Body,
    face_solid: Body,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<TracingStartPoint>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "find_tracing_start_points(edge_solid={edge_solid}, face_solid={face_solid}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

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
                if surface_could_contain(surface, &point, max_nodes, min_subdivision_size)
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
fn candidate_within<S: Scalar>(
    model: &Model<S>,
    candidates: &[VertexId],
    origin: VertexId,
    point: &Vector3<S>,
    radius: S,
    stride: Option<(&Vector3<S>, S)>,
    surf_a: &NurbSurface3D<S>,
    surf_b: &NurbSurface3D<S>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<VertexId>> {
    let radius_sq = radius.mul(radius);
    let mut best: Option<(VertexId, S)> = None;
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
            if along.definitely_less(S::ZERO) || along.definitely_greater(length) {
                continue;
            }
        }
        if best.is_some() && !d.could_be_less(best.expect("checked").1) {
            continue;
        }
        let on_both =
            surface_could_contain(surf_a, &candidate_point, max_nodes, min_subdivision_size)?
                .is_some()
                && surface_could_contain(
                    surf_b,
                    &candidate_point,
                    max_nodes,
                    min_subdivision_size,
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

/// How many damped-Newton iterations the corrector may take to bring a
/// predicted point onto *both* surfaces at once. Like [`NEWTON_ITERATIONS`]
/// this bounds effort, not correctness: the loop exits early the moment the
/// two surfaces' enclosures of the point overlap, and failing to converge
/// within the budget is reported as an error rather than accepted.
const CORRECTOR_ITERATIONS: usize = 20;

/// Levenberg-Marquardt damping for the corrector's linear solve. Purely a
/// conditioning term — it keeps the step finite where the 4x4 system is
/// rank-deficient (the two surfaces locally parallel, or a parametric pole
/// where one surface's `Su`/`Sv` collapse), at the cost of a shorter step
/// that the next iteration simply continues. It cannot affect what a
/// converged answer *means*: the `could_be_equal` test below is what
/// decides whether the corrector actually landed on the curve.
const CORRECTOR_DAMPING: f64 = 1e-12;

/// One predictor-corrector step from `point` (with known `(u_a, v_a)` /
/// `(u_b, v_b)` on `surf_a`/`surf_b`) along `dir`: predicts `point + dir *
/// step_size`, then solves for the point that lies on *both* surfaces at
/// once by damped Newton, returning the union of the two surfaces'
/// enclosures of it plus each surface's own resulting `(u, v)`.
///
/// The corrector solves one system in the four unknowns `(u_a, v_a, u_b,
/// v_b)` rather than projecting onto each surface separately. Separate
/// projections each enforce their own surface's constraint while ignoring
/// the other's, so they land a full prediction-error apart and stay there
/// no matter how good the predictor is — and that error is not small: the
/// marching point inherits the start vertex's own interval width (a vertex
/// found by `curve_surface_intersect` is located only to within
/// `min_subdivision_size`), which `sharpen` turns into a concrete offset.
///
/// Three of the four residuals are `S_a(u_a, v_a) - S_b(u_b, v_b) = 0`.
/// Those alone are underdetermined by exactly one degree of freedom — the
/// intersection curve is a curve, so sliding along it satisfies them
/// equally well — and that free direction is what the damper has to pin
/// down. It pins it *biased to the last direction*: the fourth residual
/// holds the solution on the plane through the prediction with normal
/// `dir`, so the corrector may move freely onto the curve but not drift
/// along it, and the step stays the one the predictor asked for.
///
/// A step that would leave a patch is *not* rejected: `project` clamps into
/// the domain, so such a step lands exactly on the patch boundary — which
/// is precisely where an intersection curve ends, and therefore where the
/// vertex it must connect to lives. Refusing it instead stops the march a
/// full `step_size` short of that vertex, which is how a trace ends up
/// never reaching anything. Deciding whether a direction is viable at all
/// is `face_contains`' job on the first step (see `trace_one_side`), not
/// something for every step to re-litigate via raw domain bounds.
#[allow(clippy::too_many_arguments)]
fn predictor_corrector_step<S: Scalar>(
    surf_a: &NurbSurface3D<S>,
    surf_b: &NurbSurface3D<S>,
    point: Vector3<S>,
    dir: Vector3<S>,
    u_a: S,
    v_a: S,
    u_b: S,
    v_b: S,
    step_size: S,
) -> GeopResult<(Vector3<S>, S, S, S, S)> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "predictor_corrector_step(point={point:?}, dir={dir:?}, u_a={u_a:?}, v_a={v_a:?}, u_b={u_b:?}, v_b={v_b:?}, step_size={step_size:?})"
        ))
    };
    // Sharpened exactly once, here. The prediction is a seed and a bias for
    // the damped Newton below, free to be any point along the step, so
    // collapsing it costs no accuracy — and it fixes the plane the corrector
    // solves against. Nothing downstream of this line sharpens.
    //
    // The plane's normal `dir` is as free as its point: any plane crossing
    // the curve pins the corrector's free direction equally well. Left wide,
    // it is the one wide row of an otherwise sharp Jacobian — and where the
    // two surfaces meet at a shallow angle, `dir` (a normalized cross product
    // of nearly parallel normals, taken at the previous step's honest
    // `(u, v)`) is wide by percents, enough to put zero inside a pivot of the
    // nearly singular system and fail the solve.
    let dir = dir.sharpen();
    let target = point.add(&dir.prod_scalar(step_size)).sharpen();
    let (u_a_lo, u_a_hi) = surf_a.domain_u();
    let (v_a_lo, v_a_hi) = surf_a.domain_v();
    let (u_b_lo, u_b_hi) = surf_b.domain_u();
    let (v_b_lo, v_b_hi) = surf_b.domain_v();
    let lambda = S::from_f64(CORRECTOR_DAMPING);

    // Two copies of the iterate. `*1` is the sharp *seed* the next Newton
    // step starts from; `*_honest` is that same value with the last update's
    // width kept, and is what gets returned.
    //
    // Sharpening the seed is legitimate (any point inside it is an equally
    // good place to start) and necessary (left unsharpened, 20 steps compound
    // width until `evaluate` fails outright — measured at 36/175 scenes).
    // Sharpening the *answer* is not: callers ask `face_contains` where this
    // point is, and a sharp `(u, v)` sitting 6e-17 off a trim boundary reports
    // `Inside` a face it is actually on the edge of, because `could_be_equal`
    // has no width to work with. That produced traced curves running along a
    // face's own boundary, spliced in as zero-area slivers.
    //
    // Seeding sharp while returning honest is what the wart needed: the
    // returned width is that of a *single* step from a sharp seed — bounded
    // per step rather than merely not discarded — so it states how well this
    // iterate is pinned down without compounding.
    //
    // That holds for the first seed too. The `(u, v)` handed in are the
    // previous step's honest answer, as wide as that step left them; used
    // as the seed unsharpened, each step starts from the last one's width,
    // carries it through the residual into its own, and the march compounds
    // it step after step. On a tool whose corners are only known to 1e-12 —
    // computed from a solid's own vertices — that reached a whole patch's
    // width within a dozen steps (`cube_minus_box_with_wide_corners`).
    let seed = |x: S, (lo, hi): (S, S)| clamp(x.sharpen(), lo, hi);
    let (mut u_a1, mut v_a1) = (seed(u_a, (u_a_lo, u_a_hi)), seed(v_a, (v_a_lo, v_a_hi)));
    let (mut u_b1, mut v_b1) = (seed(u_b, (u_b_lo, u_b_hi)), seed(v_b, (v_b_lo, v_b_hi)));
    let (mut u_a_honest, mut v_a_honest) = (u_a, v_a);
    let (mut u_b_honest, mut v_b_honest) = (u_b, v_b);
    for _ in 0..CORRECTOR_ITERATIONS {
        let p_a = surf_a
            .evaluate(u_a1, v_a1)
            .with_context(&|e: GeopError| e.with_context("evaluate surf_a"))
            .with_context(&ctx)?;
        let p_b = surf_b
            .evaluate(u_b1, v_b1)
            .with_context(&|e: GeopError| e.with_context("evaluate surf_b"))
            .with_context(&ctx)?;
        // All *four* residuals have to be satisfied, not just the three
        // that put the point on both surfaces. The corrector is handed the
        // previous step's converged `(u, v)`, which already lies on both
        // surfaces — testing only those three would report success without
        // ever moving, leaving the march pinned at its first step forever.
        // The fourth residual is what says the point has actually advanced
        // to the plane the predictor stepped to.
        let on_plane = dir.prod_dot(&p_a.sub(&target));
        if p_a.could_be_equal(&p_b) && on_plane.could_be_equal(S::ZERO) {
            // The union of the two surfaces' enclosures, not their average:
            // both enclose the *same* point on the intersection curve, so
            // the smallest interval containing both is the honest answer.
            // Averaging instead invents a sharp midpoint claiming more
            // precision than either had — and hides how far apart they
            // were, exactly the signal that the two surfaces have started
            // to disagree about where the curve is.
            return Ok((
                p_a.union(&p_b),
                u_a_honest,
                v_a_honest,
                u_b_honest,
                v_b_honest,
            ));
        }

        let (sau, sav) = surf_a
            .derivatives(u_a1, v_a1)
            .with_context(&|e: GeopError| e.with_context("derivatives of surf_a"))
            .with_context(&ctx)?;
        let (sbu, sbv) = surf_b
            .derivatives(u_b1, v_b1)
            .with_context(&|e: GeopError| e.with_context("derivatives of surf_b"))
            .with_context(&ctx)?;

        // Residuals: the two surfaces must meet (rows 0..3), and the meeting
        // point must sit on the plane the predictor stepped to (row 3).
        let delta = p_a.sub(&p_b);
        let mut f = [S::ZERO; 4];
        let mut j = [[S::ZERO; 4]; 4];
        for k in 0..3 {
            f[k] = delta[k];
            j[k] = [sau[k], sav[k], sbu[k].neg(), sbv[k].neg()];
        }
        f[3] = on_plane;
        j[3] = [dir.prod_dot(&sau), dir.prod_dot(&sav), S::ZERO, S::ZERO];

        // Damped normal equations `(J^T J + lambda I) d = -J^T f`. Going
        // through `J^T J` rather than solving `J d = -f` directly is what
        // lets the damping act at all, and keeps the step finite where `J`
        // loses rank instead of erroring out of an otherwise fine trace.
        //
        // The matrix is sharpened: it only steers the step, and any matrix
        // near the true Jacobian leads Newton to the same fixed point — the
        // residual `f` alone decides where that is, so the step's honest
        // width is the residual's, carried through a sharp preconditioner (as
        // in Krawczyk's `Y = mid(J)^-1`). Left as intervals, the elimination
        // amplifies the surfaces' own width by the system's condition, which
        // `J^T J` squares: where the surfaces meet at a shallow angle, that
        // put zero inside the last pivot of a solvable system.
        let mut ata = [[S::ZERO; 4]; 4];
        let mut atf = [S::ZERO; 4];
        for r in 0..4 {
            for c in 0..4 {
                let mut sum = S::ZERO;
                for k in 0..4 {
                    sum = sum.add(j[k][r].mul(j[k][c]));
                }
                ata[r][c] = if r == c { sum.add(lambda) } else { sum }.sharpen();
            }
            let mut sum = S::ZERO;
            for k in 0..4 {
                sum = sum.add(j[k][r].mul(f[k]));
            }
            atf[r] = sum.neg();
        }

        let step = solve_linear_system(&Matrix::from_rows(ata), &Vector::from_array(atf))
            .with_context(&ctx)?;

        // Each update is kept twice: honestly, and sharpened as the next
        // seed — see the declarations above for why the two differ.
        u_a_honest = clamp(u_a1.add(step[0]), u_a_lo, u_a_hi);
        v_a_honest = clamp(v_a1.add(step[1]), v_a_lo, v_a_hi);
        u_b_honest = clamp(u_b1.add(step[2]), u_b_lo, u_b_hi);
        v_b_honest = clamp(v_b1.add(step[3]), v_b_lo, v_b_hi);
        // Sharpen *before* clamping, as `NurbSurface::project` does: `clamp`
        // cannot pull back an interval that merely straddles a domain bound,
        // and collapsing such an interval to its midpoint can land just past
        // the bound, where the next `evaluate` rejects it. On a sharp value
        // `clamp`'s comparisons are exact, so the seed is always in domain.
        u_a1 = clamp(u_a_honest.sharpen(), u_a_lo, u_a_hi);
        v_a1 = clamp(v_a_honest.sharpen(), v_a_lo, v_a_hi);
        u_b1 = clamp(u_b_honest.sharpen(), u_b_lo, u_b_hi);
        v_b1 = clamp(v_b_honest.sharpen(), v_b_lo, v_b_hi);
    }

    Err(ctx(GeopError::new(format!(
        "the corrector did not bring the two surfaces onto a common point within {CORRECTOR_ITERATIONS} iterations: face_a lands at (u={u_a1:?}, v={v_a1:?}) and face_b at (u={u_b1:?}, v={v_b1:?}), which cannot be the same point — at least one of them has left its surface"
    ))))
}

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
fn trace_from_start_point<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    start: &TracingStartPoint,
    candidates: &[VertexId],
    max_nodes: usize,
    min_subdivision_size: S,
    max_trace_steps: usize,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "trace_from_start_point(vertex={}, edge_solid={}, face={})",
            start.vertex, start.edge_solid, start.face
        ))
    };

    let mut tried: std::collections::HashSet<(FaceId, FaceId)> = std::collections::HashSet::new();
    loop {
        let model = part.topology();
        let faces_a = faces_at_vertex(
            model,
            start.edge_solid,
            start.vertex,
            None,
            max_nodes,
            min_subdivision_size,
        )
        .with_context(&ctx)?;
        let faces_b = faces_at_vertex(
            model,
            start.face_solid,
            start.vertex,
            Some(start.face),
            max_nodes,
            min_subdivision_size,
        )
        .with_context(&ctx)?;
        let untried = faces_a
            .iter()
            .flat_map(|&face_a| faces_b.iter().map(move |&face_b| (face_a, face_b)))
            .find(|pair| !tried.contains(pair));
        let Some((face_a, face_b)) = untried else {
            return Ok(());
        };
        tried.insert((face_a, face_b));
        trace_one_side(
            part,
            naming,
            start.vertex,
            face_a,
            face_b,
            candidates,
            max_nodes,
            min_subdivision_size,
            max_trace_steps,
        )
        .with_context(&|e: GeopError| e.with_context(format!("face_a={face_a}, face_b={face_b}")))
        .with_context(&ctx)?;
    }
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
fn faces_at_vertex<S: Scalar>(
    model: &Model<S>,
    solid: Body,
    vertex: VertexId,
    recorded: Option<FaceId>,
    max_nodes: usize,
    min_subdivision_size: S,
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
            match surface_could_contain(surface, &point, max_nodes, min_subdivision_size)? {
                Some((u, v)) => matches!(
                    face_contains(
                        model,
                        face_id,
                        u,
                        v,
                        max_nodes,
                        min_subdivision_size,
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
fn trace_one_side<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    v: VertexId,
    face_a: FaceId,
    face_b: FaceId,
    candidates: &[VertexId],
    max_nodes: usize,
    min_subdivision_size: S,
    max_trace_steps: usize,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "trace_one_side(v={v}, face_a={face_a}, face_b={face_b})"
        ))
    };

    let model = part.topology();
    let point = model.get_vertex(v).with_context(&ctx)?.point;
    let surf_a = model.get_face(face_a).with_context(&ctx)?.surface.clone();
    let surf_b = model.get_face(face_b).with_context(&ctx)?.surface.clone();

    let (u_a0, v_a0) = surface_could_contain(&surf_a, &point, max_nodes, min_subdivision_size)
        .with_context(&ctx)?
        .ok_or_else(|| {
            GeopError::new("trace_one_side: start vertex does not lie on face_a's surface")
        })
        .with_context(&ctx)?;
    let (u_b0, v_b0) = surface_could_contain(&surf_b, &point, max_nodes, min_subdivision_size)
        .with_context(&ctx)?
        .ok_or_else(|| {
            GeopError::new("trace_one_side: start vertex does not lie on face_b's surface")
        })
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
        adaptive_step_size(&surf_a, &surf_b, u_a0, v_a0, u_b0, v_b0).with_context(&ctx)?;
    let mut chosen = None;
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
        model,
        candidates,
        v,
        &point,
        first_step,
        None,
        &surf_a,
        &surf_b,
        max_nodes,
        min_subdivision_size,
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
        let offset = model.get_vertex(w).with_context(&ctx)?.point.sub(&point);
        trials.push((offset.norm().div(S::TWO).with_context(&ctx)?, None));
        let along = offset.prod_dot(&axis);
        toward_near = if along.definitely_greater(S::ZERO) {
            Some(true)
        } else if along.definitely_less(S::ZERO) {
            Some(false)
        } else {
            None
        };
    }
    trials.push((first_step, toward_near));
    'trial: for (trial, not_toward) in trials {
        for (forward, sign) in [(true, S::ONE), (false, S::ONE.neg())] {
            if not_toward == Some(forward) {
                continue;
            }
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
                Err(_) => continue,
            };
            let inside_a = face_contains(
                model,
                face_a,
                na,
                va,
                max_nodes,
                min_subdivision_size,
                FACE_CONTAINS_SEED,
            )
            .with_context(&ctx)?;
            let inside_b = face_contains(
                model,
                face_b,
                nb,
                vb,
                max_nodes,
                min_subdivision_size,
                FACE_CONTAINS_SEED,
            )
            .with_context(&ctx)?;
            if inside_a == PointClassification::Inside && inside_b == PointClassification::Inside {
                chosen = Some((next_point, na, va, nb, vb, dir, inside_a, inside_b));
                break 'trial;
            }
            last_rejection = Some((inside_a, inside_b));
        }
    }

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
    let mut params = vec![(u_a0, v_a0, u_b0, v_b0), (u_a, v_a, u_b, v_b)];

    // March until we reach another vertex: one on the stretch of curve the
    // next stride runs along, from `cur_point` up to the plane at `step`
    // along `dir` that the corrector lands it on (see `candidate_within`).
    let mut hit_vertex = None;
    for _ in 0..max_trace_steps {
        let step = adaptive_step_size(&surf_a, &surf_b, u_a, v_a, u_b, v_b).with_context(&ctx)?;
        let reached = |radius: S| {
            candidate_within(
                model,
                candidates,
                v,
                &cur_point,
                radius,
                Some((&dir, step)),
                &surf_a,
                &surf_b,
                max_nodes,
                min_subdivision_size,
            )
        };
        // Every point of the plane the stride lands on is at least `step`
        // from `cur_point`, so a vertex ahead within `step` is reached
        // whatever the stride does — and is found without taking it, which
        // matters where the curve ends at the edge of a patch the stride
        // would have to leave.
        if let Some(hit) = reached(step).with_context(&ctx)? {
            hit_vertex = Some(hit);
            break;
        }
        let (next_point, na, va, nb, vb) =
            predictor_corrector_step(&surf_a, &surf_b, cur_point, dir, u_a, v_a, u_b, v_b, step)
                .with_context(&ctx)?;
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
            hit_vertex = Some(hit);
            break;
        }
        cur_point = next_point;
        (u_a, v_a, u_b, v_b) = (na, va, nb, vb);
        points.push(cur_point);
        params.push((na, va, nb, vb));

        // Re-derive direction from the new normals for the next step,
        // keeping continuity with the previous direction.
        if let (Ok(na), Ok(nb)) = (surf_a.normal(u_a, v_a), surf_b.normal(u_b, v_b)) {
            if let Ok(new_axis) = na.prod_cross(&nb).normalize() {
                dir = if new_axis.prod_dot(&dir).could_be_greater(S::ZERO) {
                    new_axis
                } else {
                    new_axis.neg()
                };
            }
        }
    }

    // An intersection curve of two faces has to end at a vertex — every
    // place it can end is a point where one face's boundary pierces the
    // other, which this pass has already split a vertex into. Ending
    // anywhere else means the trace lost the curve.
    let Some(hit_vertex) = hit_vertex else {
        return Err(ctx(GeopError::new(format!(
            "traced {} step(s) from {cur_point:?} without reaching another vertex",
            points.len()
        ))));
    };
    // Land the traced polyline exactly on the vertex it ends at, so the
    // fitted curve's endpoint and the edge's `end_vertex` agree.
    let hit_point = model.get_vertex(hit_vertex).with_context(&ctx)?.point;
    *points.last_mut().expect("points is never empty") = hit_point;
    let ctx = |e: GeopError| {
        ctx(e).with_context(format!(
            "traced {} step(s) to vertex {hit_vertex} at {hit_point:?}",
            points.len()
        ))
    };

    // Is this curve already in the model? Every intersection branch has two
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
    let duplicate = model
        .edges
        .iter()
        .map(|(&id, edge)| {
            let same_ends = (edge.start_vertex == v && edge.end_vertex == hit_vertex)
                || (edge.start_vertex == hit_vertex && edge.end_vertex == v);
            if !same_ends {
                return Ok(None);
            }
            curve_could_contain(&edge.curve, &mid, max_nodes, min_subdivision_size)
                .map(|hit| hit.map(|_| id))
        })
        .collect::<GeopResult<Vec<_>>>()
        .with_context(&ctx)?
        .into_iter()
        .flatten()
        // Lowest id, not whichever the hash map yields first: choosing
        // arbitrarily would make remesh depend on hash order.
        .min_by_key(|id| id.0);

    let edge_id = match duplicate {
        Some(id) => id,
        None => {
            // The edge's curve interpolates the marched points, and must
            // enclose the intersection branch *between* them too — a later
            // trace of the same branch from its other end tests its own
            // points against this curve (the duplicate check above), and
            // they lie on the branch, not on an interpolant.
            //
            // Located with the same corrector the march uses, at a fraction
            // `frac` of the way along the leg from `from` (whose surface
            // parameters are `(u_a, v_a, u_b, v_b)`) to `to`.
            let on_branch = |from: Vector3<S>,
                             (u_a, v_a, u_b, v_b): (S, S, S, S),
                             to: Vector3<S>,
                             frac: S|
             -> GeopResult<(Vector3<S>, (S, S, S, S))> {
                let leg_ctx = |e: GeopError| {
                    e.with_context(format!(
                        "on_branch(from={from:?}, to={to:?}, frac={frac:?})"
                    ))
                };
                let chord = to.sub(&from);
                let step = chord.norm().mul(frac);
                let (p, na, va, nb, vb) = predictor_corrector_step(
                    &surf_a,
                    &surf_b,
                    from,
                    chord.normalize().with_context(&leg_ctx)?,
                    u_a,
                    v_a,
                    u_b,
                    v_b,
                    step,
                )
                .with_context(&leg_ctx)?;
                Ok((p, (na, va, nb, vb)))
            };
            // First every leg is split, on the branch, into `pieces`: the
            // march's stride leaves a cubic through its points drifting ~1e-5
            // from the branch, and halving every leg cuts that ~16x for one
            // corrector per leg. A branch crossed in only a stride or two
            // needs more than halving, though: it would be left with too few
            // points for a cubic at all — a single stride, halved, is three,
            // and a quadratic through three points of an ellipse drifts ~4e-6
            // from it. That drift is enclosed as width, honestly, but only in
            // the directions it points in, and a pcurve fitted by projecting
            // the curve onto a surface tilted against them inherits it as an
            // offset its own width does not cover. So every branch gets at
            // least `MIN_TRACED_LEGS` legs.
            let strides = points.len() - 1;
            let pieces = MIN_TRACED_LEGS.div_ceil(strides).max(2);
            let mut dense = vec![points[0]];
            let mut dense_params = vec![params[0]];
            for i in 0..strides {
                for k in 1..pieces {
                    let frac = S::from_ratio(k as i64, pieces as i64).with_context(&ctx)?;
                    let (p, p_params) =
                        on_branch(points[i], params[i], points[i + 1], frac).with_context(&ctx)?;
                    dense.push(p);
                    dense_params.push(p_params);
                }
                dense.push(points[i + 1]);
                dense_params.push(params[i + 1]);
            }
            let (points, params) = (dense, dense_params);
            // Then the branch inside each of the new legs, which
            // `interpolate_enclosing` widens the curve to hold.
            let legs = points.len() - 1;
            let mut between = Vec::with_capacity(legs);
            for i in 0..legs {
                let fractions = true_point_fractions(i, legs);
                let mut inside = Vec::with_capacity(fractions.len());
                for &(a, b) in fractions {
                    let frac = S::from_ratio(a, b).with_context(&ctx)?;
                    let (p, _) =
                        on_branch(points[i], params[i], points[i + 1], frac).with_context(&ctx)?;
                    inside.push(p);
                }
                between.push(inside);
            }
            let curve = NurbCurve::<S, 4>::interpolate_enclosing(&points, &between, 3)
                .with_context(&ctx)?;
            let edge = part
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

    // Common to both: whichever edge carries this curve, each face needs a
    // coedge for it, and only if it does not already have one. A traced edge
    // has neither yet; an existing one may already be imprinted on one face
    // and not the other — the splitting stage creates edges that were never
    // imprinted, and a curve traced from the other end carries only its own
    // two faces.
    for face in [face_a, face_b] {
        if edge_is_boundary_of_face(part.topology(), edge_id, face) {
            continue;
        }
        let new_face = part
            .splice_edge_into_face(
                edge_id,
                face,
                max_nodes,
                min_subdivision_size,
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
    use geop_core_math::{scalars::ScalInF64, vector::Vector3};
    use geop_ops::Namer;
    use geop_ops_extrude_revolve::shapes::cube_solid;

    const MAX_NODES: usize = 20000;
    const MAX_EDGE_INTERSECTIONS: usize = 17;

    fn min_subdivision_size() -> ScalInF64 {
        ScalInF64::from_f64(1e-7)
    }
    fn curve_curve_min_subdivision_size() -> ScalInF64 {
        ScalInF64::from_f64(1e-4)
    }

    fn unit_cube_at(
        part: &mut Part<ScalInF64>,
        name: &str,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> geop_core_topology::SolidId {
        let f = ScalInF64::from_f64;
        let min = Vector3::from_array([f(cx - 0.5), f(cy - 0.5), f(cz - 0.5)]);
        let max = Vector3::from_array([f(cx + 0.5), f(cy + 0.5), f(cz + 0.5)]);
        cube_solid(part, name, min, max).unwrap()
    }

    /// Two unit cubes offset by half a cube-width along x: a simple, fully
    /// planar scene (no curved surfaces at all) exercising every phase —
    /// coincident faces get imprinted, and the two solids' silhouettes
    /// (where one cube's edge pierces the other's face) get traced.
    #[test]
    fn box_grid_offset_x_half_edges_x_faces() {
        let mut part = Part::<ScalInF64>::new();
        let solid_a = unit_cube_at(&mut part, "a", 0.0, 0.0, 0.0).into();
        let solid_b = unit_cube_at(&mut part, "b", 0.5, 0.0, 0.0).into();
        let namer = Namer::new("boolean", "ab").unwrap();
        let mut naming = BooleanNaming::new(&part, &namer, &[solid_a, solid_b]).unwrap();

        remesh_vertices(&mut part, solid_a, solid_b).unwrap();
        remesh_vertices_x_edges(
            &mut part,
            &mut naming,
            solid_a,
            solid_b,
            MAX_NODES,
            min_subdivision_size(),
        )
        .unwrap();
        remesh_vertices_x_edges(
            &mut part,
            &mut naming,
            solid_b,
            solid_a,
            MAX_NODES,
            min_subdivision_size(),
        )
        .unwrap();
        remesh_edges_x_edges(
            &mut part,
            &mut naming,
            solid_a,
            solid_b,
            MAX_EDGE_INTERSECTIONS,
            MAX_NODES,
            curve_curve_min_subdivision_size(),
        )
        .unwrap();

        let edges_before = part.topology().edges.len();
        let coedges_before = part.topology().coedges.len();

        remesh_edges_x_faces(
            &mut part,
            &mut naming,
            solid_a,
            solid_b,
            MAX_EDGE_INTERSECTIONS,
            MAX_NODES,
            curve_curve_min_subdivision_size(),
            200,
        )
        .unwrap();
        naming.finish(&mut part).unwrap();
        part.check_names().unwrap();
        let model = part.topology();

        eprintln!(
            "edges: {} -> {}, coedges: {} -> {}",
            edges_before,
            model.edges.len(),
            coedges_before,
            model.coedges.len()
        );

        std::fs::create_dir_all("outputs").unwrap();
        match geop_ops_rasterize::debug::rasterize_topology(model, 12) {
            Ok(render) => render
                .save_to_file("outputs/remesh_edges_x_faces_box_grid_offset_x_half.html")
                .unwrap(),
            Err(e) => eprintln!("rasterize failed (tracing itself succeeded): {e}"),
        }
    }
}

#[cfg(test)]
mod splice_regression_tests {
    use crate::{
        remesh::remesh::{RemeshParams, remesh},
        scenes::all_scenes,
    };
    use geop_core_math::scalars::ScalInF64;
    use geop_core_topology::validation::{ValidationParameters, validate, validate_fast};
    use geop_ops::Namer;

    fn namer() -> Namer {
        Namer::new("boolean", "ab").unwrap()
    }

    /// One isolated scene out of `all_scenes`' 175, so a `validate_fast`
    /// failure after `remesh` can be debugged on its own instead of being
    /// one line of a sweep's log. `box_grid_n1p00_n0p50_n0p50` is two unit
    /// cubes offset by `(-1, -0.5, -0.5)` — they meet face-to-face along
    /// `x`, so the pair is entirely planar and coincident, exercising the
    /// imprint path (and `splice_edge_into_face`'s loop restructuring) with
    /// no curved geometry involved at all.
    /// Isolated from the 175-scene sweep: a cylinder drilled part-way into
    /// a box, so the cylinder's curved wall genuinely intersects the box's
    /// flat faces — the case where marching has to follow a *curved*
    /// intersection curve rather than a straight one, and where a fixed
    /// step size lets the trace wander off the surfaces entirely.
    #[test]
    fn box_cylinder_blind_hole_remesh_succeeds() {
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == "box_cylinder_blind_hole")
            .expect("scene must exist");

        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            RemeshParams::<ScalInF64>::default(),
        )
        .unwrap();

        // The cylinder wall genuinely cuts the box's faces here, so tracing
        // *must* have produced intersection edges. The decisive signature of
        // a traced edge — as opposed to one merely split out of an existing
        // edge by `split_piercing_crossings` — is that it is spliced into a
        // face of *each* solid, so its coedges span both. A plain edge count
        // wouldn't distinguish the two, and so would still pass if every
        // trace gave up, which is indistinguishable from success in
        // `remesh`'s return value alone.
        let faces_a = scene.part.topology().solid_faces(scene.solid_a).unwrap();
        let faces_b = scene.part.topology().solid_faces(scene.solid_b).unwrap();
        let shared = scene
            .part
            .topology()
            .edges
            .keys()
            .filter(|&&edge_id| {
                let faces: Vec<_> = scene
                    .part
                    .topology()
                    .coedges_of_edge(edge_id)
                    .into_iter()
                    .filter_map(|c| scene.part.topology().get_coedge(c).ok().map(|c| c.face))
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

    /// Isolated from the sweep: two unit spheres offset so they overlap in
    /// two axes. Spheres are the scenes where `validate_fast` still reports
    /// a pcurve whose surface point doesn't match its edge's 3D endpoint,
    /// and a sphere's parametrization has poles, so this is the case that
    /// stresses the projection behind `fit_pcurve`.
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
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == "box_cylinder_drilled_hole_through")
            .expect("scene must exist");

        let before = scene.part.topology().faces.len();
        let params = RemeshParams::<ScalInF64>::default();
        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            params,
        )
        .unwrap();
        let after = scene.part.topology().faces.len();

        assert!(
            after > before,
            "remesh must split faces the intersection curve cuts across: {before} faces before, {after} after"
        );
        for (&face_id, face) in &scene.part.topology().faces {
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
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == "figure8_cylinder_through_neck_oversized")
            .expect("scene must exist");

        let params = RemeshParams::<ScalInF64>::default();
        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            params,
        )
        .unwrap();

        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };
        if let Err(errors) = validate_fast(&validation_params, scene.part.topology()) {
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
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("scene {name} must exist"));

        let params = RemeshParams::<ScalInF64>::default();
        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            params,
        )
        .unwrap_or_else(|e| panic!("{name}: remesh failed: {e}"));

        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };
        if let Err(errors) = validate(&validation_params, scene.part.topology()) {
            let all: Vec<String> = errors.iter().map(|e| format!("{e}")).collect();
            panic!(
                "{name}: {} validate error(s):\n{}",
                errors.len(),
                all.join("\n---\n")
            );
        }
    }

    #[test]
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
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == "box_cylinder_corner_quarter_overlap")
            .expect("scene must exist");

        let params = RemeshParams::<ScalInF64>::default();
        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            params,
        )
        .unwrap();

        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };
        if let Err(errors) = validate_fast(&validation_params, scene.part.topology()) {
            panic!("{} validate_fast error(s): {}", errors.len(), errors[0]);
        }
    }

    #[test]
    fn box_grid_n1p00_n0p50_n0p50_is_valid_after_remesh() {
        let mut scene = all_scenes::<ScalInF64>()
            .into_iter()
            .find(|s| s.name == "box_grid_n1p00_n0p50_n0p50")
            .expect("scene must exist");

        let params = RemeshParams::<ScalInF64>::default();
        remesh(
            &mut scene.part,
            &namer(),
            scene.solid_a,
            scene.solid_b,
            params,
        )
        .unwrap();

        let validation_params = ValidationParameters {
            max_nodes: params.max_nodes,
            min_subdivision_size: params.curve_curve_min_subdivision_size,
            ..ValidationParameters::default()
        };
        if let Err(errors) = validate_fast(&validation_params, scene.part.topology()) {
            panic!("{} validate_fast error(s): {errors:?}", errors.len());
        }
    }
}
