//! Curve/point containment by per-axis fat line clipping — the derivation
//! is in `curve.md` next to this file; this module implements its Clip B
//! ("fat line") together with the iteration of its section 5.
//!
//! Same contract as [`super::curve_bisect::curve_could_contain`] (a sound
//! enclosure of every parameter at which the curve could pass through the
//! point, `None` when it definitely doesn't), but instead of only asking
//! "could this segment's hull contain the point?" and bisecting, every
//! segment is *clipped*: each Cartesian axis `k` yields a scalar spline
//! `g_k(t) = X_k(t) - p_k W(t)` whose zeros are exactly the parameters where
//! that coordinate matches, and a fat line around its graph's control
//! polygon bounds those zeros to an interval. Intersecting the axes'
//! intervals shrinks the segment directly towards the solution, so a
//! transversal hit converges in a handful of clips instead of one
//! bisection per bit of precision.

use std::collections::VecDeque;

use crate::{
    aabb::aabb_could_contain,
    fat_line::{fat_line_zeros, greville_abscissae},
    nurb_curve::{NurbCurve, ParameterRefinable},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

/// Clip `seg` against `point`, axis by axis: the intersection of every
/// axis's [`fat_line_zeros`] with `seg`'s domain, or `None` if some axis
/// proves the segment misses the point.
///
/// Axis `k`'s coefficients `d_i = P_{i,k} - p_k P_{i,w}` come straight from
/// the homogeneous control points, with no division: since
/// `NurbCurve::try_new` guarantees positive weights, `W(t) > 0` and
/// `C_k(t) = p_k ⇔ X_k(t) - p_k W(t) = 0`.
fn clip<S: Scalar, const D: usize, const C: usize>(
    seg: &NurbCurve<S, D>,
    xi: &[S],
    point: &Vector<S, C>,
) -> Option<S> {
    let mut t_hat = seg.domain_as_scalar();
    let mut d = Vec::with_capacity(seg.control_points.len());
    for k in 0..C {
        d.clear();
        d.extend(
            seg.control_points
                .iter()
                .map(|q| q[k].sub(point[k].mul(q[D - 1]))),
        );
        let zeros = fat_line_zeros(xi, &d)?;
        // Both constraints hold at once, so their intersection encloses
        // the solution — but `intersect` of disjoint enclosures is not
        // empty, it returns an input, so disjointness is checked first.
        if !zeros.could_be_equal(t_hat) {
            return None;
        }
        t_hat = t_hat.intersect(zeros);
    }
    Some(t_hat)
}

/// Euclidean distance between the segment's first and last (dehomogenized)
/// control points — the same convergence measure `curve_bisect::curve_could_contain`
/// uses.
fn chord_length<S: Scalar, const D: usize, const C: usize>(seg: &NurbCurve<S, D>) -> GeopResult<S> {
    let dehom = |q: &Vector<S, D>| -> GeopResult<Vector<S, C>> {
        let inv_w = S::ONE.div(q[D - 1])?;
        let mut v = Vector::<S, C>::zero();
        for c in 0..C {
            v[c] = q[c].mul(inv_w);
        }
        Ok(v)
    };
    let first = dehom(&seg.control_points[0])?;
    let last = dehom(&seg.control_points[seg.control_points.len() - 1])?;
    Ok(last.sub(&first).norm())
}

/// Enclosure of `{C(t) : t ∈ t_hat}`, or `None` if it can't be had cheaply.
///
/// De Boor evaluation with an interval parameter encloses the polynomial
/// of the *one* knot span `find_knot_span` picks, over the whole interval —
/// so it is only an enclosure of the curve if `t_hat` lies definitely inside
/// a single span. Interior knots must therefore be definitely outside
/// `t_hat`: even one touching `t_hat`'s upper end makes `find_knot_span`
/// pick the span to its right and extrapolate that polynomial over the
/// rest. The domain's own end knots may touch; `find_knot_span` handles
/// both ends explicitly.
fn evaluate_over<S: Scalar, const D: usize, const C: usize>(
    seg: &NurbCurve<S, D>,
    t_hat: S,
) -> Option<Vector<S, C>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let (lo, hi) = (t_hat.lower(), t_hat.upper());
    let interior_knots = &seg.knot_vector[seg.degree + 1..seg.control_points.len()];
    let single_span = interior_knots
        .iter()
        .all(|&u| u.definitely_less(lo) || u.definitely_greater(hi));
    if !single_span {
        return None;
    }
    seg.evaluate_cartesian(t_hat).ok()
}

/// Fat-line-clipping counterpart of [`super::curve_bisect::curve_could_contain`],
/// with the same tunables: the [`Scalar::union`] of every converged
/// segment's clipped parameter interval, exploring breadth-first up to
/// `max_nodes` segments. `None` only when every part of the domain was
/// rejected: unlike `curve_bisect`, running out of budget is never read as "not
/// contained" (nor as "contained") — it is an error, since the search is
/// incomplete (see `surface.md` §5).
///
/// Each segment is clipped (see [`clip`]). An empty clip rejects it. A
/// segment converges — and reports its *clipped* interval, tighter than its
/// domain and still an enclosure — once either the curve evaluated over that
/// interval contains the point within `min_subdivision_size` in every axis,
/// or its chord is no longer definitely greater than `min_subdivision_size`.
/// Otherwise, if the clip removed at least 20% of the domain, the segment is
/// restricted to the clip and clipped again; if it didn't — several
/// solutions in one segment, or a tangency, where clipping stalls — it is
/// bisected instead, exactly as the hull search always does. (20% is the
/// usual Bézier/fat line clipping rule, Sederberg & Nishita.)
///
/// Restriction cuts at the clip's *outer* bounds ([`Scalar::lower`] /
/// [`Scalar::upper`]), which are free choices only on the outside: any cut
/// outside the enclosure loses nothing, a cut through it would. Bisection
/// uses the sharpened midpoint as always.
pub fn curve_could_contain<S: Scalar, const D: usize, const C: usize>(
    curve: &NurbCurve<S, D>,
    point: &Vector<S, C>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<S>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    debug_assert_eq!(
        C + 1,
        D,
        "point dimension must match the curve's Cartesian dimension"
    );
    let min_progress = S::from_ratio(4, 5)?;

    let mut queue: VecDeque<NurbCurve<S, D>> = VecDeque::new();
    queue.push_back(curve.clone());

    let mut explored = 0usize;
    let mut solution: Option<S> = None;
    let mut report = |t: S| {
        solution = Some(match solution {
            Some(existing) => existing.union(t),
            None => t,
        });
    };

    while let Some(seg) = queue.pop_front() {
        if explored >= max_nodes {
            return Err(GeopError::new(format!(
                "curve_could_contain (clipping): exhausted max_nodes={max_nodes} with {} \
                 segments pending; the result would be incomplete",
                queue.len() + 1
            )));
        }
        explored += 1;

        // Cheap prefilter against the cached bounding box, as in `curve_bisect`.
        if !aabb_could_contain(&seg.aabb, point) {
            continue;
        }

        let Some(xi) = greville_abscissae(&seg.knot_vector, seg.degree, seg.control_points.len())?
        else {
            // Degree 0 has no graph polygon to clip; the box test above is
            // all we can say.
            report(seg.domain_as_scalar());
            continue;
        };
        let Some(t_hat) = clip(&seg, &xi, point) else {
            continue;
        };

        // `t_hat` encloses every solution in this segment, but a narrow
        // `t_hat` alone doesn't mean the point is near the curve: a single
        // axis can pin it down while the others were never checked at that
        // precision. So a narrow `t_hat` is settled by evaluating the curve
        // over it — an enclosure of `C(t_hat)`. Missing the point proves
        // there is no solution; containing it with a physically small
        // enclosure is the same statement the chord test below makes. This
        // is also what ends a search whose solution sits exactly on a domain
        // end, where the clip collapses but no cut can be made.
        if !t_hat.width().definitely_greater(min_subdivision_size) {
            if let Some(on_curve) = evaluate_over(&seg, t_hat) {
                if !on_curve.could_be_equal(point) {
                    continue;
                }
                if (0..C).all(|c| !on_curve[c].width().definitely_greater(min_subdivision_size)) {
                    report(t_hat);
                    continue;
                }
            }
        }

        let (t0, t1) = seg.domain();
        let domain_width = seg.domain_as_scalar().width();
        if !chord_length::<S, D, C>(&seg)?.definitely_greater(min_subdivision_size) {
            report(t_hat);
            continue;
        }

        if t_hat
            .width()
            .definitely_less(domain_width.mul(min_progress))
        {
            let (lo, hi) = (t_hat.lower(), t_hat.upper());
            // `sub_curve` only cuts at bounds strictly inside the domain; if
            // neither is (e.g. a clip collapsed onto a domain end), no cut is
            // possible and we fall through to bisection — re-queuing an
            // uncut segment would just repeat this node until the budget.
            let inside = |t: S| t.definitely_greater(t0) && t.definitely_less(t1);
            if inside(lo) || inside(hi) {
                if let Ok(restricted) = seg.sub_curve(lo, hi) {
                    queue.push_back(restricted);
                    continue;
                }
            }
        }

        match seg.split_mid() {
            Ok((left, right)) => {
                queue.push_back(left);
                queue.push_back(right);
            }
            // Cannot split (e.g. midpoint already at multiplicity p+1).
            Err(_) => report(t_hat),
        }
    }

    Ok(solution)
}

#[cfg(test)]
mod tests {
    use super::curve_could_contain;
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

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

    /// Exact rational quarter circle of radius 1 in the xy-plane.
    fn quarter_circle<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbCurve::try_new(
            2,
            vec![pt(1., 0., 0., 1.), pt(w, w, 0., w), pt(0., 1., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// Cubic B-spline with two interior knots, wiggling through 3-D.
    fn wiggle<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            3,
            vec![
                pt(0., 0., 0., 1.),
                pt(1., 2., 0., 1.),
                pt(2., -1., 1., 1.),
                pt(3., 2., 0., 1.),
                pt(4., 0., -1., 1.),
                pt(5., 1., 0., 1.),
            ],
            vec![
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(0.3),
                f(0.6),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
            ],
        )
        .unwrap()
    }

    /// Point at `t` must be found, and the returned enclosure must contain `t`.
    fn assert_contains_at<S: Scalar>(c: &NurbCurve<S, 4>, t: f64) {
        let t = S::from_f64(t);
        let p = c.evaluate(t).unwrap();
        let found = curve_could_contain(c, &p, MAX, S::from_f64(EPS))
            .unwrap()
            .unwrap_or_else(|| panic!("point at t={t:?} not found"));
        assert!(
            found.could_be_equal(t),
            "enclosure {found:?} misses t={t:?}"
        );
    }

    fn check_line_contains<S: Scalar>() {
        for t in [0., 0.25, 0.5, 1.] {
            assert_contains_at(&line::<S>(), t);
        }
    }
    #[test]
    fn line_contains() {
        for_all_scalars!(check_line_contains);
    }

    fn check_quarter_circle_contains<S: Scalar>() {
        for t in [0., 0.1, 0.5, 0.77, 1.] {
            assert_contains_at(&quarter_circle::<S>(), t);
        }
    }
    #[test]
    fn quarter_circle_contains() {
        for_all_scalars!(check_quarter_circle_contains);
    }

    fn check_wiggle_contains<S: Scalar>() {
        for t in [0., 0.05, 0.3, 0.42, 0.6, 0.9, 1.] {
            assert_contains_at(&wiggle::<S>(), t);
        }
    }
    #[test]
    fn wiggle_contains() {
        for_all_scalars!(check_wiggle_contains);
    }

    /// Clipping narrows a transversal hit well below `min_subdivision_size`
    /// instead of stopping at a bisection leaf.
    fn check_result_is_tight<S: Scalar>() {
        let c = wiggle::<S>();
        let t = S::from_f64(0.42);
        let p = c.evaluate(t).unwrap();
        let found = curve_could_contain(&c, &p, MAX, S::from_f64(EPS))
            .unwrap()
            .unwrap();
        assert!(found.width().definitely_less(S::from_f64(EPS)), "{found:?}");
    }
    #[test]
    fn result_is_tight() {
        for_all_scalars!(check_result_is_tight);
    }

    fn check_misses_off_curve_points<S: Scalar>() {
        let f = S::from_f64;
        let off = |x, y, z| Vector3::from_array([f(x), f(y), f(z)]);
        // Inside the circle's bounding box and control polygon, off the arc.
        assert!(
            curve_could_contain(&quarter_circle::<S>(), &off(0.6, 0.6, 0.), MAX, f(EPS))
                .unwrap()
                .is_none()
        );
        // Beside the line, beyond its end.
        assert!(
            curve_could_contain(&line::<S>(), &off(1.5, 0., 0.), MAX, f(EPS))
                .unwrap()
                .is_none()
        );
        assert!(
            curve_could_contain(&line::<S>(), &off(0.5, 0.1, 0.), MAX, f(EPS))
                .unwrap()
                .is_none()
        );
        assert!(
            curve_could_contain(&wiggle::<S>(), &off(2.5, 0.5, 0.5), MAX, f(EPS))
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn misses_off_curve_points() {
        for_all_scalars!(check_misses_off_curve_points);
    }
}
