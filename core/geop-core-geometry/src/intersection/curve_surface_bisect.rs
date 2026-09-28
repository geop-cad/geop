//! The previous curve–surface search — convex hull tests and bisection,
//! with coincidence read from reaching `max_solutions` — kept only as the
//! baseline for `examples/intersection_bench.rs`. The kernel uses
//! [`super::curve_surface`].

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::{
    aabb::aabb_could_overlap,
    fat_axis::{curve_fat_axes_separate, surface_fat_plane_separates},
    nurb_curve::{NurbCurve, dehomogenize},
    nurb_surface::NurbSurface,
};
use geop_core_math::{
    disjoint_set::DisjointSet,
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector2,
};

use super::Intersections;

/// An entry in the outer-loop priority queue: a `(curve segment, surface
/// patch)` pair awaiting a DFS dive, ordered by `level` (shallower first) —
/// see `curve_surface_intersect`'s own doc comment for why popping the
/// shallowest pending pair first, rather than a plain LIFO stack, matters.
struct QueueEntry<S: Scalar> {
    level: usize,
    curve_seg: NurbCurve<S, 4>,
    surf_patch: NurbSurface<S, 4>,
}

impl<S: Scalar> PartialEq for QueueEntry<S> {
    fn eq(&self, other: &Self) -> bool {
        self.level == other.level
    }
}
impl<S: Scalar> Eq for QueueEntry<S> {}
impl<S: Scalar> PartialOrd for QueueEntry<S> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<S: Scalar> Ord for QueueEntry<S> {
    fn cmp(&self, other: &Self) -> Ordering {
        // `BinaryHeap` is a max-heap; reverse so the smallest `level` is popped first.
        other.level.cmp(&self.level)
    }
}

/// Result of a single recursive DFS dive.
enum DfsOutcome<S: Scalar> {
    /// This subtree's hulls definitely cannot overlap; nothing found.
    NoSolution,
    /// A converged `(t, uv)` candidate — `t0.union(t1)` / `u0.union(u1)` /
    /// `v0.union(v1)` of whichever segment/patch it converged on — along
    /// with the sibling subtrees skipped on the way to it (each tagged with
    /// its own depth, for the outer-loop priority queue).
    Found {
        solution: (S, Vector2<S>),
        unexplored: Vec<(NurbCurve<S, 4>, NurbSurface<S, 4>, usize)>,
    },
}

/// Recursively narrow `(curve_seg, surf_patch)` until either the hulls
/// definitely cannot overlap (`NoSolution`), or both the curve chord and the
/// patch span are no longer definitely greater than `min_subdivision_size`
/// (`Found`). Always dives into the *left* half of whichever side it split,
/// stashing the right half in `unexplored` rather than recursing into it
/// directly — the outer loop (`curve_surface_intersect`) is what actually
/// explores those, via its own priority queue, so that work spreads evenly
/// across the whole domain instead of this dive exhaustively finishing one
/// side first.
///
/// `explored` is a node-visit counter shared across the *entire* search
/// (threaded through every dive, not reset per call) — exceeding
/// `max_nodes` aborts the whole search with an error rather than silently
/// returning a possibly-incomplete result; see
/// `curve_surface_intersect`'s own doc comment for why that distinction
/// matters to callers.
fn dfs<S: Scalar>(
    curve_seg: NurbCurve<S, 4>,
    surf_patch: NurbSurface<S, 4>,
    level: usize,
    min_subdivision_size: S,
    explored: &mut usize,
    max_nodes: usize,
) -> GeopResult<DfsOutcome<S>> {
    *explored += 1;
    if *explored > max_nodes {
        return Err(GeopError::new(
            "curve_surface_intersect: exhausted max_nodes before the search converged",
        ));
    }

    // No artificial padding — see `curve_curve::dfs`'s own `found_here`
    // for the full reasoning: the segment's/patch's domains are already
    // honest enclosures of whatever converged here, so inflating them adds
    // no safety, only imprecision that callers then have to compensate for
    // with a tolerance of their own.
    let found_here = |curve_seg: &NurbCurve<S, 4>, surf_patch: &NurbSurface<S, 4>| {
        let (t0, t1) = curve_seg.domain();
        let (u0, u1) = surf_patch.domain_u();
        let (v0, v1) = surf_patch.domain_v();
        DfsOutcome::Found {
            solution: (
                t0.union(t1),
                Vector2::from_array([u0.union(u1), v0.union(v1)]),
            ),
            unexplored: vec![],
        }
    };

    // Cheap prefilter: see `curve_curve::dfs`'s identical check. Both
    // `NurbCurve<S, 4>` and `NurbSurface<S, 4>` are 3-D (`c = 3`), so all 3
    // cached axes are meaningful here.
    if !aabb_could_overlap(&curve_seg.aabb, &surf_patch.aabb, 3) {
        return Ok(DfsOutcome::NoSolution);
    }

    // Second cheap prefilter, tried before the expensive iterative GJK
    // check below — see `curve_curve::dfs`'s identical addition and
    // `fat_axis`'s own module doc. Tried in both directions: the surface's
    // own fat plane (its 3 corners) against the curve's points, and the
    // curve's own fat axis/axes against the surface's points.
    let pts_c = dehomogenize::<S, 4, 3>(&curve_seg.control_points);
    let pts_s = dehomogenize::<S, 4, 3>(&surf_patch.control_points);
    if surface_fat_plane_separates(&surf_patch, &pts_c) || curve_fat_axes_separate(&curve_seg, &pts_s)
    {
        return Ok(DfsOutcome::NoSolution);
    }

    if curve_seg.convex_hull().definitely_no_overlap(&surf_patch.convex_hull()) {
        return Ok(DfsOutcome::NoSolution);
    }

    let (curve_size, surf_size) = (curve_seg.size(), surf_patch.size());

    // A side counts as converged once *either* its physical size or its own
    // parameter-domain width is no longer definitely greater than
    // `min_subdivision_size` — see `curve_curve::dfs` for the same rule and
    // why `size` alone isn't enough: `size` is derived from control points
    // that can carry real interval width, so it can stay stubbornly large
    // for a side whose domain has already narrowed to nothing left to
    // subdivide. Without the domain-width half, such a side is picked as
    // "still too big" forever, the dive keeps manufacturing distinct tiny
    // leaves, and the outer loop reads the resulting flood of solutions as
    // coincidence — which is exactly how a merely-hard-to-converge pair
    // ends up misreported as an overlapping one.
    let (t0, t1) = curve_seg.domain();
    let curve_width = t1.sub(t0);
    let (u0, u1) = surf_patch.domain_u();
    let (v0, v1) = surf_patch.domain_v();
    let surf_width = u1.sub(u0).union(v1.sub(v0));
    let curve_converged = !curve_size.definitely_greater(min_subdivision_size)
        || !curve_width.definitely_greater(min_subdivision_size);
    let surf_converged = !surf_size.definitely_greater(min_subdivision_size)
        || !surf_width.definitely_greater(min_subdivision_size);

    if curve_converged && surf_converged {
        return Ok(found_here(&curve_seg, &surf_patch));
    }

    // Split whichever side still isn't converged; if both still aren't,
    // split whichever is physically larger (ties go to the surface). Cannot
    // split and hasn't converged — report whatever's here; there's nothing
    // more to do with this pair.
    let split_curve = if curve_converged {
        false
    } else if surf_converged {
        true
    } else {
        curve_size.definitely_greater(surf_size)
    };
    let (left, right) = if split_curve {
        match curve_seg.split_mid() {
            Ok((l, r)) => ((l, surf_patch.clone()), (r, surf_patch)),
            Err(_) => return Ok(found_here(&curve_seg, &surf_patch)),
        }
    } else {
        match surf_patch.split_mid() {
            Ok((s0, s1)) => ((curve_seg.clone(), s0), (curve_seg, s1)),
            Err(_) => return Ok(found_here(&curve_seg, &surf_patch)),
        }
    };

    match dfs(
        left.0,
        left.1,
        level + 1,
        min_subdivision_size,
        explored,
        max_nodes,
    )? {
        DfsOutcome::Found {
            solution,
            mut unexplored,
        } => {
            unexplored.push((right.0, right.1, level + 1));
            Ok(DfsOutcome::Found {
                solution,
                unexplored,
            })
        }
        DfsOutcome::NoSolution => dfs(
            right.0,
            right.1,
            level + 1,
            min_subdivision_size,
            explored,
            max_nodes,
        ),
    }
}

/// Points where `curve` crosses (or, in the coplanar case, lies within)
/// `surface`.
///
/// A DFS-with-priority-queue search (restored, against the current
/// [`DisjointSet`]-based solution representation, from an earlier
/// implementation removed by commit `d75bff5`): the outer loop always dives
/// from the *shallowest* still-unexplored `(curve segment, surface patch)`
/// pair (a level-ordered [`BinaryHeap`], so the search spreads laterally
/// across the whole domain before going deep anywhere), each dive following
/// [`dfs`]'s own leftmost-branch-first policy and stashing every sibling
/// subtree it skips along the way back onto the queue at its own depth. For
/// an isolated, genuine crossing this behaves essentially like a plain
/// stack-based DFS (hull-overlap pruning quickly discards everything but
/// the local neighborhood of the crossing, regardless of traversal order).
/// But for a coincident or coplanar pair — where hull-overlap pruning can't
/// narrow anything down, since the curve overlaps the surface almost
/// everywhere along the shared region — a plain LIFO stack tends to
/// exhaustively refine one small neighborhood (feeding [`DisjointSet`] a
/// long run of adjacent, `could_be_equal` candidates that all merge into
/// one ever-widening solution) before ever reaching a genuinely different
/// part of the domain. The breadth-first-by-level ordering here instead
/// guarantees an evenly-spread set of solutions across the *whole* shared
/// region — which is what actually lets a caller reliably treat "found
/// `max_solutions` distinct solutions" as a coincidence signal in the first
/// place; see e.g. `booleans::remesh::remesh_edges_x_faces`.
///
/// The search stops once `max_solutions` distinct solutions have been
/// found, or the queue empties (every subtree explored, genuinely fewer
/// solutions than `max_solutions`) — either way, `Ok`. If it instead
/// exhausts `max_nodes` mid-dive without reaching either of those, that's a
/// genuinely unknown result: this returns an error rather than silently
/// reporting a possibly-incomplete solution set as if it were final. A
/// caller checking `len() >= max_solutions` to detect coincidence has no
/// way to tell "genuinely converged short of the budget" apart from "ran
/// out of nodes early" otherwise — and the latter is, if anything, itself
/// evidence of extended overlap (an isolated crossing converges in a
/// handful of nodes; only a broad shared region burns through a real
/// budget), so callers relying on that heuristic should treat this error
/// the same way they'd treat hitting `max_solutions`.
pub fn curve_surface_intersect<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface: &NurbSurface<S, 4>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Intersections<(S, Vector2<S>)>> {
    let mut queue: BinaryHeap<QueueEntry<S>> = BinaryHeap::new();
    queue.push(QueueEntry {
        level: 0,
        curve_seg: curve.clone(),
        surf_patch: surface.clone(),
    });

    let mut explored = 0usize;
    let mut solutions: DisjointSet<(S, Vector2<S>)> = DisjointSet::new();

    while solutions.len() < max_solutions {
        let Some(entry) = queue.pop() else { break };

        match dfs(
            entry.curve_seg,
            entry.surf_patch,
            entry.level,
            min_subdivision_size,
            &mut explored,
            max_nodes,
        )? {
            DfsOutcome::NoSolution => continue,
            DfsOutcome::Found {
                solution,
                unexplored,
            } => {
                solutions.insert(solution);
                for (c, s, lvl) in unexplored {
                    queue.push(QueueEntry {
                        level: lvl,
                        curve_seg: c,
                        surf_patch: s,
                    });
                }
            }
        }
    }

    let result = solutions.into_vec();
    Ok(if max_solutions > 0 && result.len() >= max_solutions {
        Intersections::Coincident(result)
    } else {
        Intersections::Found(result)
    })
}

#[cfg(test)]
mod tests {
    use super::curve_surface_intersect;
    use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    const MAX_NODES: usize = 2000;

    fn ptc<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    fn pts<S: Scalar>(x: f64, y: f64, z: f64) -> Vector4<S> {
        Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
    }

    /// Flat unit patch in the xy-plane (z = 0), x,y ∈ [0,1].
    fn flat_xy<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pts(0., 0., 0.),
                pts(0., 1., 0.),
                pts(1., 0., 0.),
                pts(1., 1., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Straight line crossing `flat_xy` once at (0.5, 0.5, 0).
    fn vertical_crossing_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0.5, 0.5, -1., 1.), ptc(0.5, 0.5, 1., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Straight line entirely above `flat_xy` (z ∈ [1, 2]) — never crosses.
    fn line_above_surface<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0.5, 0.5, 1., 1.), ptc(0.5, 0.5, 2., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Quadratic Bézier dipping below z=0 and back, crossing `flat_xy` twice.
    /// y = 0.3 is deliberately not the midpoint of flat_xy's y range [0,1],
    /// avoiding the "both halves always survive" tie pathology that exact
    /// midpoints trigger (see `crossing_vyz` in `surface_surface.rs`).
    fn double_dip_curve<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                ptc(0.2, 0.3, 1.0, 1.),
                ptc(0.5, 0.3, -2.0, 1.),
                ptc(0.8, 0.3, 1.0, 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Straight line lying *in* the `flat_xy` plane (z = 0), spanning part of
    /// its footprint: from (0.2, 0.5, 0) to (0.8, 0.5, 0).
    fn coplanar_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0.2, 0.5, 0., 1.), ptc(0.8, 0.5, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Coplanar line spanning x ∈ [-0.5, 0.5] at y=0.5 — only the x ∈ [0, 0.5]
    /// half (t ∈ [0.5, 1]) overlaps `flat_xy`'s footprint (x,y ∈ [0,1]).
    fn partial_overlap_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(-0.5, 0.5, 0., 1.), ptc(0.5, 0.5, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Coplanar line spanning x ∈ [-1, 2] at y=0.5 — much larger than
    /// `flat_xy`'s x-extent [0,1], extending beyond it on both sides. Only
    /// x ∈ [0,1] (t ∈ [1/3, 2/3]) overlaps the surface.
    fn oversized_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(-1.0, 0.5, 0., 1.), ptc(2.0, 0.5, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Coplanar line spanning x ∈ [0,1] at y=0.5 — exactly matches
    /// `flat_xy`'s x-extent.
    fn full_width_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0., 0.5, 0., 1.), ptc(1., 0.5, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Homogeneous control point with weight `w`, given its Cartesian
    /// position `(x, y, z)`.
    fn ptw<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x * w),
            S::from_f64(y * w),
            S::from_f64(z * w),
            S::from_f64(w),
        ])
    }

    /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
    /// x,y ∈ [0,2]. Its v=0.5 ridge line sits at z = 0.5.
    fn bent_surface<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pts(0., 0., 0.),
                pts(0., 2., 1.),
                pts(2., 0., 1.),
                pts(2., 2., 0.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Quadratic Bézier running along `bent_surface`'s ridge line (y = 1),
    /// dipping from z=-1 up to z=3 and back to z=-1 -- crossing the ridge's
    /// z=0.5 height at two distinct points (t ≈ 0.25 and t ≈ 0.75).
    fn bent_curve<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                ptc(0.2, 1.0, -1.0, 1.),
                ptc(1.0, 1.0, 3.0, 1.),
                ptc(1.8, 1.0, -1.0, 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Degree-(2,2) rational patch covering one octant of the unit sphere
    /// (x,y,z >= 0), built by revolving a quarter-circle meridian (in the
    /// xz-plane) by a quarter turn around the z axis. Its v=0 edge is exactly
    /// `equator_quarter_circle`.
    fn sphere_octant_patch<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        let w = 1.0 / 2.0_f64.sqrt();
        NurbSurface::try_new(
            2,
            2,
            vec![
                // u = 0 (azimuth 0deg)
                ptw(1., 0., 0., 1.),
                ptw(1., 0., 1., w),
                ptw(0., 0., 1., 1.),
                // u = 1 (azimuth 45deg)
                ptw(1., 1., 0., w),
                ptw(1., 1., 1., 0.5),
                ptw(0., 0., 1., w),
                // u = 2 (azimuth 90deg)
                ptw(0., 1., 0., 1.),
                ptw(0., 1., 1., w),
                ptw(0., 0., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Quarter circle from (1,0,0) to (0,1,0) in the xy-plane -- exactly the
    /// v=0 edge of `sphere_octant_patch`, i.e. coincident with that surface.
    fn equator_quarter_circle<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        let w = 1.0 / 2.0_f64.sqrt();
        NurbCurve::try_new(
            2,
            vec![ptw(1., 0., 0., 1.), ptw(1., 1., 0., w), ptw(0., 1., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    const EPS: f64 = 1e-2;

    // ── Single crossing ───────────────────────────────────────────────────────

    fn check_single_crossing_curve_has_one_solution<S: Scalar>() {
        let curve = vertical_crossing_line::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result.len(), 1);
    }
    #[test]
    fn single_crossing_curve_has_one_solution() {
        for_all_scalars!(check_single_crossing_curve_has_one_solution);
    }

    // ── No crossing ───────────────────────────────────────────────────────────

    fn check_curve_missing_surface_has_no_solution<S: Scalar>() {
        let curve = line_above_surface::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(result.is_empty());
    }
    #[test]
    fn curve_missing_surface_has_no_solution() {
        for_all_scalars!(check_curve_missing_surface_has_no_solution);
    }

    // ── Budget ────────────────────────────────────────────────────────────────

    fn check_max_solutions_zero_returns_empty<S: Scalar>() {
        let curve = vertical_crossing_line::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 0, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(result.is_empty());
    }
    #[test]
    fn max_solutions_zero_returns_empty() {
        for_all_scalars!(check_max_solutions_zero_returns_empty);
    }

    fn check_max_nodes_exhausted_errors<S: Scalar>() {
        let curve = coplanar_line::<S>();
        let surf = flat_xy::<S>();
        // A coincident pair searching for far more solutions than a tiny
        // node budget can possibly separate must error, not silently
        // return a truncated/misleading result.
        let result = curve_surface_intersect(&curve, &surf, 1000, 3, S::from_f64(EPS));
        assert!(result.is_err());
    }
    #[test]
    fn max_nodes_exhausted_errors() {
        for_all_scalars!(check_max_nodes_exhausted_errors);
    }

    // ── Two crossings ─────────────────────────────────────────────────────────

    fn check_two_crossings_found_when_budget_allows<S: Scalar>() {
        let curve = double_dip_curve::<S>();
        let surf = flat_xy::<S>();
        let result = curve_surface_intersect(&curve, &surf, 2, MAX_NODES, S::from_f64(EPS))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 2);
        assert!(
            !result[0].0.could_be_equal(result[1].0),
            "the two crossings should remain distinct after unification"
        );
    }
    #[test]
    fn two_crossings_found_when_budget_allows() {
        for_all_scalars!(check_two_crossings_found_when_budget_allows);
    }

    fn check_max_solutions_one_caps_at_one_even_with_two_crossings<S: Scalar>() {
        let curve = double_dip_curve::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result.len(), 1);
    }
    #[test]
    fn max_solutions_one_caps_at_one_even_with_two_crossings() {
        for_all_scalars!(check_max_solutions_one_caps_at_one_even_with_two_crossings);
    }

    // ── min_subdivision_size controls precision ────────────────────────────

    fn check_min_subdivision_size_controls_precision<S: Scalar>() {
        let curve = vertical_crossing_line::<S>();
        let surf = flat_xy::<S>();
        let result = curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(1e-3))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 1);

        let (_, uv) = result[0];
        assert!(
            uv[0]
                .sub(S::from_f64(0.5))
                .abs()
                .could_be_less(S::from_f64(1e-2))
        );
        assert!(
            uv[1]
                .sub(S::from_f64(0.5))
                .abs()
                .could_be_less(S::from_f64(1e-2))
        );
    }
    #[test]
    fn min_subdivision_size_controls_precision() {
        for_all_scalars!(check_min_subdivision_size_controls_precision);
    }

    // ── Coplanar curve: must terminate ───────────────────────────────────────

    fn check_coplanar_curve_terminates<S: Scalar>() {
        let curve = coplanar_line::<S>();
        let surf = flat_xy::<S>();

        // A single dive must converge to exactly one result.
        let result_one =
            curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result_one.len(), 1);

        // Asking for more solutions still terminates, with at most that many
        // (possibly fewer after unification) segments along the coplanar overlap.
        let result_many =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result_many.is_empty());
        assert!(result_many.len() <= 5);
    }
    #[test]
    fn coplanar_curve_terminates() {
        for_all_scalars!(check_coplanar_curve_terminates);
    }

    // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

    fn check_coincident_curve_reaches_max_solutions<S: Scalar>() {
        let curve = full_width_line::<S>();
        let surf = flat_xy::<S>();
        // A curve running the *entire* width of the surface it's coincident
        // with, with a generous node budget, should reliably reach the
        // requested solution count via the evenly-spread search — this is
        // exactly the property `max_solutions` saturating is meant to
        // signal "coincident" to a caller in the first place.
        let result = curve_surface_intersect(&curve, &surf, 5, 5000, S::from_f64(1e-3)).unwrap();
        assert!(result.is_coincident());
        assert_eq!(result.len(), 5);
    }
    #[test]
    fn coincident_curve_reaches_max_solutions() {
        for_all_scalars!(check_coincident_curve_reaches_max_solutions);
    }

    // ── Partial overlap: only part of the curve lies over the surface ───────

    fn check_partial_overlap_coplanar_line<S: Scalar>() {
        let curve = partial_overlap_line::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result.is_empty());
        assert!(result.len() <= 5);

        // Every solution must lie within the overlapping half of the curve
        // (x >= 0, i.e. t >= 0.5), up to a small tolerance.
        let lower_bound = S::from_f64(0.5 - EPS);
        for &(t, _) in result.as_slice() {
            assert!(!t.definitely_less(lower_bound));
        }
    }
    #[test]
    fn partial_overlap_coplanar_line() {
        for_all_scalars!(check_partial_overlap_coplanar_line);
    }

    // ── Curve much larger than the surface ───────────────────────────────────

    fn check_curve_larger_than_surface_terminates<S: Scalar>() {
        let curve = oversized_line::<S>();
        let surf = flat_xy::<S>();
        let result =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result.is_empty());
        assert!(result.len() <= 5);

        // Every solution must lie within the overlapping middle third of the
        // curve (x ∈ [0,1], i.e. t ∈ [1/3, 2/3]), up to a small tolerance.
        let lower_bound = S::from_f64(1.0 / 3.0 - EPS);
        let upper_bound = S::from_f64(2.0 / 3.0 + EPS);
        for &(t, _) in result.as_slice() {
            assert!(!t.definitely_less(lower_bound));
            assert!(!t.definitely_greater(upper_bound));
        }
    }
    #[test]
    fn curve_larger_than_surface_terminates() {
        for_all_scalars!(check_curve_larger_than_surface_terminates);
    }

    // ── Curve exactly the same size as the surface ──────────────────────────

    fn check_curve_same_size_as_surface_terminates<S: Scalar>() {
        let curve = full_width_line::<S>();
        let surf = flat_xy::<S>();

        let result_one =
            curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result_one.len(), 1);

        let result_many =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result_many.is_empty());
        assert!(result_many.len() <= 5);
    }
    #[test]
    fn curve_same_size_as_surface_terminates() {
        for_all_scalars!(check_curve_same_size_as_surface_terminates);
    }

    // ── Bent surface / bent curve: distinct crossings ────────────────────────

    fn check_bent_surface_bent_curve_two_distinct_crossings<S: Scalar>() {
        let curve = bent_curve::<S>();
        let surf = bent_surface::<S>();
        let result = curve_surface_intersect(&curve, &surf, 4, MAX_NODES, S::from_f64(EPS))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 2);
        assert!(
            !result[0].0.could_be_equal(result[1].0),
            "the two crossings of a bent curve through a bent surface should be distinct"
        );
    }
    #[test]
    fn bent_surface_bent_curve_two_distinct_crossings() {
        for_all_scalars!(check_bent_surface_bent_curve_two_distinct_crossings);
    }

    // ── Coincident circle on a spherical patch: must terminate ───────────────

    fn check_coincident_circle_on_sphere_patch_terminates<S: Scalar>() {
        let curve = equator_quarter_circle::<S>();
        let surf = sphere_octant_patch::<S>();

        let result_one =
            curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result_one.len(), 1);

        let result_many =
            curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result_many.is_empty());
        assert!(result_many.len() <= 5);
    }
    #[test]
    fn coincident_circle_on_sphere_patch_terminates() {
        for_all_scalars!(check_coincident_circle_on_sphere_patch_terminates);
    }
}
