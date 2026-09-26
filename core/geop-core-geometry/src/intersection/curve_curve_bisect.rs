//! The previous curve–curve search — convex hull tests and bisection, with
//! coincidence read from reaching `max_solutions` — kept only as the
//! baseline for `examples/intersection_bench.rs`. The kernel uses
//! [`super::curve_curve`].

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::{
    aabb::aabb_could_overlap,
    fat_axis::HasFatAxes,
    nurb_curve::{HasConvexHull, NurbCurve, dehomogenize},
};
use geop_core_math::{
    disjoint_set::DisjointSet,
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::Intersections;

/// An entry in the outer-loop priority queue: a `(seg_a, seg_b)` pair
/// awaiting a DFS dive, ordered by `level` (shallower first) — see
/// `curve_curve_intersect`'s own doc comment for why popping the shallowest
/// pending pair first, rather than a plain LIFO stack, matters.
struct QueueEntry<S: Scalar, const D: usize> {
    level: usize,
    seg_a: NurbCurve<S, D>,
    seg_b: NurbCurve<S, D>,
}

impl<S: Scalar, const D: usize> PartialEq for QueueEntry<S, D> {
    fn eq(&self, other: &Self) -> bool {
        self.level == other.level
    }
}
impl<S: Scalar, const D: usize> Eq for QueueEntry<S, D> {}
impl<S: Scalar, const D: usize> PartialOrd for QueueEntry<S, D> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<S: Scalar, const D: usize> Ord for QueueEntry<S, D> {
    fn cmp(&self, other: &Self) -> Ordering {
        // `BinaryHeap` is a max-heap; reverse so the smallest `level` is popped first.
        other.level.cmp(&self.level)
    }
}

/// Result of a single recursive DFS dive.
enum DfsOutcome<S: Scalar, const D: usize> {
    /// This subtree's hulls definitely cannot overlap; nothing found.
    NoSolution,
    /// A converged `(t_a, t_b)` candidate — each the [`Scalar::union`] of
    /// whichever segment it converged on — along with the sibling subtrees
    /// skipped on the way to it (each tagged with its own depth, for the
    /// outer-loop priority queue).
    Found {
        solution: (S, S),
        unexplored: Vec<(NurbCurve<S, D>, NurbCurve<S, D>, usize)>,
    },
}

/// Recursively narrow `(seg_a, seg_b)` until either the hulls definitely
/// cannot overlap (`NoSolution`), or neither segment's chord is definitely
/// greater than `min_subdivision_size` (`Found`). Always dives into the
/// *left* half of whichever side it split, stashing the right half in
/// `unexplored` rather than recursing into it directly — the outer loop
/// (`curve_curve_intersect`) is what actually explores those, via its own
/// priority queue, so that work spreads evenly across the whole domain
/// instead of this dive exhaustively finishing one side first.
///
/// `explored` is a node-visit counter shared across the *entire* search
/// (threaded through every dive, not reset per call) — exceeding
/// `max_nodes` aborts the whole search with an error rather than silently
/// returning a possibly-incomplete result; see `curve_curve_intersect`'s
/// own doc comment for why that distinction matters to callers.
fn dfs<S: Scalar, const D: usize, const C: usize>(
    seg_a: NurbCurve<S, D>,
    seg_b: NurbCurve<S, D>,
    level: usize,
    min_subdivision_size: S,
    explored: &mut usize,
    max_nodes: usize,
) -> GeopResult<DfsOutcome<S, D>>
where
    NurbCurve<S, D>: HasConvexHull<S, C> + HasFatAxes<S, C>,
{
    *explored += 1;
    if *explored > max_nodes {
        return Err(GeopError::new(
            "curve_curve_intersect: exhausted max_nodes before the search converged",
        ));
    }

    // No artificial padding: `seg_a`/`seg_b`'s own domains are already
    // honest interval bounds (that's the whole point of interval
    // arithmetic), so a converged solution's true position is already
    // guaranteed to lie within them — inflating it further isn't adding
    // safety, just needless imprecision that callers then have to account
    // for themselves (e.g. `disjointness_check::coincides_with_a_vertex`
    // used to need its own separate distance-epsilon fudge to compensate
    // for this padding before comparing against a vertex's own tight
    // bound). `DisjointSet::insert` merging only on an exact
    // `Scalar::could_be_equal` overlap is the right, epsilon-free behavior;
    // if a near-tangential touch converges to several adjacent-but-not-
    // quite-overlapping leaves instead of one, that's real information (the
    // touch genuinely isn't pinned down tighter than that yet), not
    // something to paper over here.
    let found_here = |seg_a: &NurbCurve<S, D>, seg_b: &NurbCurve<S, D>| {
        let (a0, a1) = seg_a.domain();
        let (b0, b1) = seg_b.domain();
        DfsOutcome::Found {
            solution: (a0.union(a1), b0.union(b1)),
            unexplored: vec![],
        }
    };

    // Cheap prefilter: each segment's cached axis-aligned bounding box (see
    // `aabb::compute_aabb`) is far quicker to compare than building a convex
    // hull and running GJK, and just as sound — if the boxes can't overlap,
    // neither can the (tighter-fitting) hulls they contain. This alone
    // resolves most dfs nodes; the hull/GJK check below only runs when it
    // doesn't.
    if !aabb_could_overlap(&seg_a.aabb, &seg_b.aabb, C) {
        return Ok(DfsOutcome::NoSolution);
    }

    // Second cheap prefilter, tried before the expensive iterative GJK
    // check below: each segment's own fat line/plane (through its
    // endpoints, for a curve) is often a far more effective separating
    // axis than the world-axis-aligned AABB for a diagonal segment — and
    // testing along one fixed axis is a handful of dot products, not
    // GJK's up-to-64-iteration simplex search. Sound for the same reason
    // the AABB prefilter is: a fixed axis proving separation is a
    // sufficient (if not exhaustive) condition, so `false` here is as
    // trustworthy as `hull.definitely_no_overlap` — see `fat_axis`'s own
    // module doc. Tried in both directions since either segment's own
    // axis might be the one that resolves it.
    if let (Ok(pts_a), Ok(pts_b)) = (
        dehomogenize::<S, D, C>(&seg_a.control_points),
        dehomogenize::<S, D, C>(&seg_b.control_points),
    ) {
        if seg_a.fat_axes_separate(&pts_b) || seg_b.fat_axes_separate(&pts_a) {
            return Ok(DfsOutcome::NoSolution);
        }
    }

    let (hull_a, hull_b) = match (seg_a.convex_hull(), seg_b.convex_hull()) {
        (Ok(a), Ok(b)) => (a, b),
        // Degenerate segment (zero weight): can't bound or split it any
        // further — report whatever's here rather than silently dropping it.
        _ => return Ok(found_here(&seg_a, &seg_b)),
    };
    if hull_a.definitely_no_overlap(&hull_b) {
        return Ok(DfsOutcome::NoSolution);
    }

    let (size_a, size_b) = match (seg_a.size(), seg_b.size()) {
        (Ok(a), Ok(b)) => (a, b),
        _ => return Ok(found_here(&seg_a, &seg_b)),
    };

    // A segment counts as converged once *either* its physical chord
    // (`size`) or its own parameter-domain width is no longer definitely
    // greater than `min_subdivision_size` — not just `size` alone. `size`
    // is a *physical* chord length, derived from the (possibly
    // Boehm-insertion-noise-inflated) control points; the domain width is a
    // direct, purely-parametric measure of how much room is even left to
    // subdivide, immune to that noise. Relying on `size` alone lets a
    // segment whose domain has already narrowed to (or past) what's
    // representable keep getting picked as "still needs splitting" forever,
    // since its noisy `size` never registers as small — this domain-width
    // check is a second, independent way to recognize "nothing more to
    // gain here" even when `size` is lying.
    let (a0, a1) = seg_a.domain();
    let width_a = a1.sub(a0);
    let (b0, b1) = seg_b.domain();
    let width_b = b1.sub(b0);
    let a_converged = !size_a.definitely_greater(min_subdivision_size)
        || !width_a.definitely_greater(min_subdivision_size);
    let b_converged = !size_b.definitely_greater(min_subdivision_size)
        || !width_b.definitely_greater(min_subdivision_size);

    if a_converged && b_converged {
        return Ok(found_here(&seg_a, &seg_b));
    }

    // Split whichever of the two segments still isn't converged; if both
    // still aren't, split whichever is larger (ties go to `b`), same as
    // before. Cannot split and hasn't converged — report whatever's here.
    let split_a = if a_converged {
        false
    } else if b_converged {
        true
    } else {
        size_a.definitely_greater(size_b)
    };
    let (left, right) = if split_a {
        match seg_a.split_mid() {
            Ok((l, r)) => ((l, seg_b.clone()), (r, seg_b)),
            Err(_) => return Ok(found_here(&seg_a, &seg_b)),
        }
    } else {
        match seg_b.split_mid() {
            Ok((l, r)) => ((seg_a.clone(), l), (seg_a, r)),
            Err(_) => return Ok(found_here(&seg_a, &seg_b)),
        }
    };

    match dfs::<S, D, C>(
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
        DfsOutcome::NoSolution => dfs::<S, D, C>(
            right.0,
            right.1,
            level + 1,
            min_subdivision_size,
            explored,
            max_nodes,
        ),
    }
}

/// Points where `curve_a` crosses (or, in the coincident case, overlaps)
/// `curve_b`.
///
/// A DFS-with-priority-queue search (restored, against the current
/// [`DisjointSet`]-based solution representation, from an earlier
/// implementation removed by commit `d75bff5`): the outer loop always dives
/// from the *shallowest* still-unexplored `(seg_a, seg_b)` pair (a
/// level-ordered [`BinaryHeap`], so the search spreads laterally across the
/// whole domain before going deep anywhere), each dive following [`dfs`]'s
/// own leftmost-branch-first policy and stashing every sibling subtree it
/// skips along the way back onto the queue at its own depth. For an
/// isolated, genuine crossing this behaves essentially like a plain
/// stack-based DFS (hull-overlap pruning quickly discards everything but
/// the local neighborhood of the crossing, regardless of traversal order).
/// But for a coincident pair — where hull-overlap pruning can't narrow
/// anything down, since the two curves overlap almost everywhere along the
/// shared region — a plain LIFO stack tends to exhaustively refine one
/// small neighborhood (feeding [`DisjointSet`] a long run of adjacent,
/// `could_be_equal` candidates that all merge into one ever-widening
/// solution) before ever reaching a genuinely different part of the domain.
/// The breadth-first-by-level ordering here instead guarantees an
/// evenly-spread set of solutions across the *whole* shared region — which
/// is what actually lets a caller reliably treat "found `max_solutions`
/// distinct solutions" as a coincidence signal in the first place.
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
/// evidence of extended overlap, so callers relying on that heuristic
/// should treat this error the same way they'd treat hitting
/// `max_solutions`.
///
/// Generic over the curves' shared homogeneous dimension `D` (e.g. `D=4`
/// for 3-D curves, `D=3` for 2-D pcurves) — both curves must share the same
/// `D`.
pub fn curve_curve_intersect<S: Scalar, const D: usize, const C: usize>(
    curve_a: &NurbCurve<S, D>,
    curve_b: &NurbCurve<S, D>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Intersections<(S, S)>>
where
    NurbCurve<S, D>: HasConvexHull<S, C> + HasFatAxes<S, C>,
{
    let mut queue: BinaryHeap<QueueEntry<S, D>> = BinaryHeap::new();
    queue.push(QueueEntry {
        level: 0,
        seg_a: curve_a.clone(),
        seg_b: curve_b.clone(),
    });

    let mut explored = 0usize;
    let mut solutions: DisjointSet<(S, S)> = DisjointSet::new();

    while solutions.len() < max_solutions {
        let Some(entry) = queue.pop() else { break };

        match dfs::<S, D, C>(
            entry.seg_a,
            entry.seg_b,
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
                for (a, b, lvl) in unexplored {
                    queue.push(QueueEntry {
                        level: lvl,
                        seg_a: a,
                        seg_b: b,
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
    use super::curve_curve_intersect;
    use crate::intersection::curve_curve::refine_crossing;
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    const MAX_NODES: usize = 2000;

    fn ptc<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    /// Horizontal line along the x axis, y=0.3, x ∈ [0,1].
    fn horizontal_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Vertical line along the y axis at x=0.5, y ∈ [-1,1] -- crosses
    /// `horizontal_line` once at (0.5, 0.3, 0).
    fn vertical_crossing_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0.5, -1., 0., 1.), ptc(0.5, 1., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Vertical line along the y axis at x=2.0, y ∈ [-1,1] -- never crosses
    /// `horizontal_line` (x ∈ [0,1]).
    fn vertical_missing_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(2.0, -1., 0., 1.), ptc(2.0, 1., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Quadratic Bézier dipping below y=0.3 and back, crossing
    /// `horizontal_line` twice. x=0.3 is deliberately not the midpoint of
    /// `horizontal_line`'s x range [0,1], avoiding the "both halves always
    /// survive" tie pathology that exact midpoints trigger.
    fn double_dip_curve<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                ptc(0.3, 1.0, 0., 1.),
                ptc(0.5, -2.0, 0., 1.),
                ptc(0.7, 1.0, 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Straight line lying *on* `horizontal_line` (same y=0.3, z=0), spanning
    /// only part of its x range: from (0.5, 0.3, 0) to (1.5, 0.3, 0). The
    /// overlap with `horizontal_line` (x ∈ [0,1]) is x ∈ [0.5, 1].
    fn coincident_overlap_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0.5, 0.3, 0., 1.), ptc(1.5, 0.3, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Straight line lying *on* `horizontal_line` exactly (same domain,
    /// x ∈ [0,1], y=0.3, z=0) — fully coincident, not just partially.
    fn full_coincident_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    const EPS: f64 = 1e-6;

    // ── Single crossing ───────────────────────────────────────────────────────

    fn check_single_crossing_curves_have_one_solution<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = vertical_crossing_line::<S>();
        let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result.len(), 1);
    }
    #[test]
    fn single_crossing_curves_have_one_solution() {
        for_all_scalars!(check_single_crossing_curves_have_one_solution);
    }

    // ── No crossing ───────────────────────────────────────────────────────────

    fn check_curves_missing_each_other_have_no_solution<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = vertical_missing_line::<S>();
        let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(result.is_empty());
    }
    #[test]
    fn curves_missing_each_other_have_no_solution() {
        for_all_scalars!(check_curves_missing_each_other_have_no_solution);
    }

    // ── Budget ────────────────────────────────────────────────────────────────

    fn check_max_solutions_zero_returns_empty<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = vertical_crossing_line::<S>();
        let result = curve_curve_intersect(&a, &b, 0, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(result.is_empty());
    }
    #[test]
    fn max_solutions_zero_returns_empty() {
        for_all_scalars!(check_max_solutions_zero_returns_empty);
    }

    fn check_max_nodes_exhausted_errors<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = full_coincident_line::<S>();
        // A coincident pair searching for far more solutions than a tiny
        // node budget can possibly separate must error, not silently
        // return a truncated/misleading result.
        let result = curve_curve_intersect(&a, &b, 1000, 3, S::from_f64(EPS));
        assert!(result.is_err());
    }
    #[test]
    fn max_nodes_exhausted_errors() {
        for_all_scalars!(check_max_nodes_exhausted_errors);
    }

    // ── Two crossings ─────────────────────────────────────────────────────────

    fn check_two_crossings_found_when_budget_allows<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = double_dip_curve::<S>();
        let result = curve_curve_intersect(&a, &b, 2, MAX_NODES, S::from_f64(EPS))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 2);
        assert!(
            !result[0].0.could_be_equal(result[1].0),
            "the two crossings should remain distinct"
        );
    }
    #[test]
    fn two_crossings_found_when_budget_allows() {
        for_all_scalars!(check_two_crossings_found_when_budget_allows);
    }

    fn check_max_solutions_one_caps_at_one_even_with_two_crossings<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = double_dip_curve::<S>();
        let result = curve_curve_intersect(&a, &b, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result.len(), 1);
    }
    #[test]
    fn max_solutions_one_caps_at_one_even_with_two_crossings() {
        for_all_scalars!(check_max_solutions_one_caps_at_one_even_with_two_crossings);
    }

    // ── min_subdivision_size controls precision ────────────────────────────

    fn check_min_subdivision_size_controls_precision<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = vertical_crossing_line::<S>();
        let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 1);

        let (t_a, _) = result[0];
        assert!(
            t_a.sub(S::from_f64(0.5))
                .abs()
                .could_be_less(S::from_f64(1e-2))
        );
    }
    #[test]
    fn min_subdivision_size_controls_precision() {
        for_all_scalars!(check_min_subdivision_size_controls_precision);
    }

    // ── Coincident overlap: must terminate ───────────────────────────────────

    fn check_coincident_overlap_terminates<S: Scalar + 'static>() {
        let a = horizontal_line::<S>();
        let b = coincident_overlap_line::<S>();

        // A single dive must converge to exactly one result.
        let result_one = curve_curve_intersect(&a, &b, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert_eq!(result_one.len(), 1);

        // Asking for more solutions still terminates, with at most that many
        // (possibly fewer after merging) segments along the overlap, and
        // each solution lying within the overlapping x ∈ [0.5, 1] range.
        let result_many = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
        assert!(!result_many.is_empty());
        assert!(result_many.len() <= 5);

        let lower_bound = S::from_f64(0.5 - EPS);
        for &(t_a, _) in result_many.as_slice() {
            assert!(t_a.could_be_greater(lower_bound));
        }
    }
    #[test]
    fn coincident_overlap_terminates() {
        for_all_scalars!(check_coincident_overlap_terminates);
    }

    // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

    fn check_full_coincidence_reaches_max_solutions<S: Scalar>() {
        let a = horizontal_line::<S>();
        let b = full_coincident_line::<S>();
        // Two curves coincident over their *entire* shared domain, with a
        // generous node budget, should reliably reach the requested
        // solution count via the evenly-spread search -- and be reported
        // via the explicit `Coincident` variant, not just inferred from
        // hitting the length cap.
        let result = curve_curve_intersect(&a, &b, 5, 5000, S::from_f64(1e-3)).unwrap();
        assert!(result.is_coincident());
        assert_eq!(result.len(), 5);
    }
    #[test]
    fn full_coincidence_reaches_max_solutions() {
        for_all_scalars!(check_full_coincidence_reaches_max_solutions);
    }

    // ── 2-D (D=3) pcurve intersection — the motivating use case ──────────────

    fn pt2<S: Scalar>(x: f64, y: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE])
    }

    /// Horizontal 2-D segment y=0.3, x ∈ [0,1].
    fn horizontal_line_2d<S: Scalar>() -> crate::nurb_curve::NurbCurve2D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![pt2(0., 0.3), pt2(1., 0.3)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Vertical 2-D segment x=0.5, y ∈ [-1,1] — crosses the horizontal line
    /// once at (0.5, 0.3).
    fn vertical_crossing_line_2d<S: Scalar>() -> crate::nurb_curve::NurbCurve2D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![pt2(0.5, -1.), pt2(0.5, 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_single_crossing_2d<S: Scalar>() {
        let a = horizontal_line_2d::<S>();
        let b = vertical_crossing_line_2d::<S>();
        let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS))
            .unwrap()
            .into_vec();
        assert_eq!(result.len(), 1);
        let (t_a, _) = result[0];
        let hit = a.evaluate(t_a).unwrap();
        assert!(
            hit[0]
                .sub(S::from_f64(0.5))
                .abs()
                .could_be_less(S::from_f64(1e-3))
        );
        assert!(
            hit[1]
                .sub(S::from_f64(0.3))
                .abs()
                .could_be_less(S::from_f64(1e-3))
        );
    }
    #[test]
    fn single_crossing_2d() {
        for_all_scalars!(check_single_crossing_2d);
    }

    // ── Hard-to-intersect curves: quartic tangencies ─────────────────────────
    //
    // Both curves below are built by converting a monomial `(2t-1)^n` (in
    // `x = 2t-1`, over the standard `t ∈ [0,1]` NURBS domain) to its exact
    // Bernstein/Bezier control points, so `y` is *exactly* `x^n` along the
    // curve, not merely close to it — an honest, analytically-known worst
    // case rather than an approximation of one.

    /// Degree-2 Bezier tracing `y = x^2` exactly (`x = 2t-1`, `t ∈ [0,1]`),
    /// touching `y = 0` at `t = 0.5` with **order-2** contact (an ordinary
    /// parabola-tangent-to-a-line case) — the case one level of
    /// cross-product deflation is built to resolve.
    fn quadratic_tangent_to_x_axis<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                ptc(-1., 1., 0., 1.),
                ptc(0., -1., 0., 1.),
                ptc(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Degree-4 Bezier tracing `y = x^4` exactly (`x = 2t-1`, `t ∈ [0,1]`),
    /// touching `y = 0` at `t = 0.5` with **order-4** contact — flatter than
    /// cross-product deflation (which resolves order-2) can fully
    /// regularize: at the touch point both the tangent cross product *and*
    /// its first derivative vanish (`y = 16s^4` near `s = t-0.5` has
    /// `y'' = 192s^2 = 0` at `s = 0` too). The deliberately hard case: does
    /// refinement stay *sound* (never claims a narrower, wrong answer) when
    /// it cannot fully converge, rather than just being tight when it can.
    fn quartic_tangent_to_x_axis<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            4,
            vec![
                ptc(-1., 1., 0., 1.),
                ptc(-0.5, -1., 0., 1.),
                ptc(0., 1., 0., 1.),
                ptc(0.5, -1., 0., 1.),
                ptc(1., 1., 0., 1.),
            ],
            vec![
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
            ],
        )
        .unwrap()
    }

    /// The x axis, x ∈ [-1,1] — the common tangent line for both curves
    /// above, touched at (0,0,0).
    fn x_axis_line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![ptc(-1., 0., 0., 1.), ptc(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// An order-2 tangency: with Krawczyk-verified deflation currently
    /// disabled (see the module comment near the top of the file),
    /// `refine_crossing` is plain Newton, whose Jacobian is *also* singular
    /// right at a tangential contact — so this is a soundness check, same
    /// shape as the order-4 case below, not a tightness one. (Once deflation
    /// is re-enabled, this is exactly the case it's meant to resolve to
    /// machine/fixed-point precision instead.)
    fn check_refine_crossing_stays_sound_for_order_2_tangency<S: Scalar>() {
        let a = quadratic_tangent_to_x_axis::<S>();
        let b = x_axis_line::<S>();

        let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
            .unwrap()
            .into_vec();
        assert_eq!(
            found.len(),
            1,
            "a single tangential touch, not a crossing pair"
        );
        let (t_a, t_b) = found[0];

        // Soundness: the search's own (loose) box must already bracket the
        // true touch point.
        assert!(t_a.could_be_equal(S::from_f64(0.5)));
        assert!(t_b.could_be_equal(S::from_f64(0.5)));

        let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

        // Soundness: refinement must still contain the true parameter.
        assert!(ra.could_be_equal(S::from_f64(0.5)));
        assert!(rb.could_be_equal(S::from_f64(0.5)));
        // Never worse than the input: refinement can only tighten (or leave
        // it unchanged, which is what happens here since plain Newton's own
        // Jacobian is singular at this tangency too).
        assert!(ra.is_subset_of(t_a));
        assert!(rb.is_subset_of(t_b));
    }
    #[test]
    fn refine_crossing_stays_sound_for_order_2_tangency() {
        for_all_scalars!(check_refine_crossing_stays_sound_for_order_2_tangency);
    }

    /// Order-4 tangency: deflation's own Jacobian is *also* singular right
    /// at the touch point, so refinement is expected to stall -- the
    /// requirement under test is that it stays sound (still encloses the
    /// true parameter, never claims a narrower box than it actually proved)
    /// rather than that it achieves full precision.
    fn check_refine_crossing_stays_sound_for_order_4_tangency<S: Scalar>() {
        let a = quartic_tangent_to_x_axis::<S>();
        let b = x_axis_line::<S>();

        let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
            .unwrap()
            .into_vec();
        assert!(!found.is_empty(), "the touch point must still be found");

        for (t_a, t_b) in found {
            // Soundness of the search itself.
            assert!(t_a.could_be_equal(S::from_f64(0.5)));
            assert!(t_b.could_be_equal(S::from_f64(0.5)));

            let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

            // The refinement guarantee under test: whatever comes back
            // still encloses the true touch point (order-4 flatness may
            // well mean it comes back completely unchanged -- that is a
            // pass, not a failure, per `refine_crossing`'s own "can only
            // tighten, never fail" contract).
            assert!(
                ra.could_be_equal(S::from_f64(0.5)),
                "refine_crossing must never lose the true root: got {ra:?}"
            );
            assert!(rb.could_be_equal(S::from_f64(0.5)));
            // And never claim a box the search didn't already prove.
            assert!(ra.is_subset_of(t_a));
            assert!(rb.is_subset_of(t_b));
        }
    }
    #[test]
    fn refine_crossing_stays_sound_for_order_4_tangency() {
        for_all_scalars!(check_refine_crossing_stays_sound_for_order_4_tangency);
    }

    /// A transversal crossing very close to a quartic's flat spot (two
    /// distinct roots of `x^4 = 0.0001`, i.e. `x = ±0.1`, extremely close
    /// together and ill-conditioned near `x=0`) — not tangential at all,
    /// but numerically adversarial: checks the search still separates and
    /// soundly encloses both nearby crossings instead of merging or losing
    /// one.
    fn quartic_minus_epsilon<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        // y = x^4 - 0.0001, same control-point construction as
        // `quartic_tangent_to_x_axis` with every y-coordinate shifted down
        // by the constant 0.0001 (Bezier control points are affine in the
        // curve's own coordinates, so a constant shift is just a shift of
        // every control point's y).
        let dy = 0.0001;
        NurbCurve::try_new(
            4,
            vec![
                ptc(-1., 1. - dy, 0., 1.),
                ptc(-0.5, -1. - dy, 0., 1.),
                ptc(0., 1. - dy, 0., 1.),
                ptc(0.5, -1. - dy, 0., 1.),
                ptc(1., 1. - dy, 0., 1.),
            ],
            vec![
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
            ],
        )
        .unwrap()
    }

    fn check_close_transversal_crossings_near_quartic_flat_spot<S: Scalar>() {
        let a = quartic_minus_epsilon::<S>();
        let b = x_axis_line::<S>();

        let found = curve_curve_intersect(&a, &b, 4, MAX_NODES, S::from_f64(1e-4))
            .unwrap()
            .into_vec();
        assert_eq!(
            found.len(),
            2,
            "two distinct, separated crossings near x=±0.1"
        );
        assert!(
            !found[0].0.could_be_equal(found[1].0),
            "the two nearby crossings must remain distinct"
        );

        // x = 2t-1 = ±0.1 -> t = 0.45 or t = 0.55.
        for &(t_a, t_b) in &found {
            let near_left = t_a
                .sub(S::from_f64(0.45))
                .abs()
                .could_be_less(S::from_f64(1e-2));
            let near_right = t_a
                .sub(S::from_f64(0.55))
                .abs()
                .could_be_less(S::from_f64(1e-2));
            assert!(near_left || near_right, "crossing at unexpected t={t_a:?}");

            let (ra, _) = refine_crossing(&a, &b, t_a, t_b);
            assert!(ra.is_subset_of(t_a), "refinement must only ever tighten");
        }
    }
    #[test]
    fn close_transversal_crossings_near_quartic_flat_spot() {
        for_all_scalars!(check_close_transversal_crossings_near_quartic_flat_spot);
    }
}
