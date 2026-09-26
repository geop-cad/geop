//! The previous curve/point containment — convex hull test and bisection —
//! kept only as the baseline for `examples/curve_contains_bench.rs`. The
//! kernel uses [`super::curve`].

use std::collections::VecDeque;

use crate::{
    aabb::aabb_could_contain,
    nurb_curve::{HasConvexHull, NurbCurve},
};
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

/// Folds `domain` into the running solution: the first one found stands
/// as-is, every subsequent one widens it via [`Scalar::union`] — so the
/// final result covers every converged segment found, not just whichever
/// one the BFS happened to reach first.
fn union_domain<S: Scalar>(solution: Option<S>, domain: S) -> S {
    match solution {
        Some(existing) => existing.union(domain),
        None => domain,
    }
}

/// BFS over subdivisions of `curve`, exploring every node up to the
/// `max_nodes` budget (never stopping early at the first hit) and returning
/// the [`Scalar::union`] of every converged segment's own domain — a segment
/// converges once its convex hull could contain `point` and its chord length
/// is no longer definitely greater than `min_subdivision_size`. `None` if no
/// segment converged within budget.
///
/// Exploring to completion (rather than returning on the first match)
/// matters because more than one segment can independently converge on
/// `point` — e.g. near a curve self-intersection, or simply because more
/// than one leaf of the subdivision tree ends up within tolerance of it —
/// and stopping early would silently narrow the answer to whichever one the
/// BFS happened to visit first instead of the true (possibly wider) set of
/// parameters that could contain it.
///
/// `epsilon` is compared against the Euclidean length of each segment's
/// chord (first to last control point), so it should be in the same units as
/// the control points.
///
/// Generic over the curve's homogeneous dimension `D` (e.g. `D=4` for 3-D
/// curves, `D=3` for 2-D pcurves); `point` is in the matching Cartesian
/// dimension `C = D - 1`.
pub fn curve_could_contain<S: Scalar, const D: usize, const C: usize>(
    curve: &NurbCurve<S, D>,
    point: &Vector<S, C>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<S>>
where
    NurbCurve<S, D>: HasConvexHull<S, C>,
{
    let mut queue: VecDeque<NurbCurve<S, D>> = VecDeque::new();
    queue.push_back(curve.clone());

    let mut explored = 0usize;
    let mut solution: Option<S> = None;

    while let Some(seg) = queue.pop_front() {
        if explored >= max_nodes {
            break;
        }
        explored += 1;

        // Cheap prefilter: see `intersection::curve_curve::dfs`'s identical
        // check — the cached bounding box is far quicker to compare than
        // building a convex hull and running GJK, and just as sound.
        if !aabb_could_contain(&seg.aabb, point) {
            continue;
        }

        let hull = match seg.convex_hull() {
            Ok(hull) => hull,
            // Degenerate segment (zero weight): can't be ruled out, so its
            // whole domain conservatively folds into the solution instead of
            // aborting the rest of the search.
            Err(_) => {
                solution = Some(union_domain(solution, seg.domain_as_scalar()));
                continue;
            }
        };

        if !hull.could_contain(point) {
            continue;
        }

        // Segment could contain the point — is the chord short enough?
        let chord_len = hull.points[hull.points.len() - 1]
            .sub(&hull.points[0])
            .norm();
        if !chord_len.definitely_greater(min_subdivision_size) {
            solution = Some(union_domain(solution, seg.domain_as_scalar()));
            continue;
        }

        // Subdivide at the parameter-domain midpoint.
        let (start_t, end_t) = seg.domain();
        // A self-chosen subdivision point: any value in the interval cuts
        // it equally well, so sharpening loses no accuracy and keeps
        // repeated splits from compounding width (see AGENTS.md).
        let mid_t = start_t.add(end_t).div(S::TWO)?.sharpen();

        match seg.split(mid_t) {
            Ok((left, right)) => {
                queue.push_back(left);
                queue.push_back(right);
            }
            // Cannot split (e.g. midpoint already at multiplicity p+1).
            Err(_) => solution = Some(union_domain(solution, seg.domain_as_scalar())),
        }
    }

    Ok(solution)
}

#[cfg(test)]
mod tests {
    use super::curve_could_contain;
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    const MAX: usize = 500;
    const EPS: f64 = 1e-3;

    // ── Curve constructors ────────────────────────────────────────────────────

    /// Degree-1 line from (0,0,0) to (1,0,0).
    fn line<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    // ── Line: points on the curve ─────────────────────────────────────────────

    fn check_line_contains_start<S: Scalar>() {
        let c = line::<S>();
        let p = c.evaluate(S::ZERO).unwrap();
        assert!(
            curve_could_contain(&c, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn line_contains_start() {
        for_all_scalars!(check_line_contains_start);
    }

    fn check_line_contains_midpoint<S: Scalar>() {
        let c = line::<S>();
        let p = c.evaluate(S::from_f64(0.5)).unwrap();
        assert!(
            curve_could_contain(&c, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn line_contains_midpoint() {
        for_all_scalars!(check_line_contains_midpoint);
    }

    fn check_line_contains_end<S: Scalar>() {
        let c = line::<S>();
        let p = c.evaluate(S::ONE).unwrap();
        assert!(
            curve_could_contain(&c, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn line_contains_end() {
        for_all_scalars!(check_line_contains_end);
    }
}
