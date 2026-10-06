//! Curve/point containment by per-axis fat line clipping — the derivation
//! is in `curve.md` next to this file; this module implements its Clip B
//! ("fat line") together with the iteration of its section 5.
//!
//! The result is a sound enclosure of every parameter at which the curve
//! could pass through the point, `None` when it definitely doesn't. Instead
//! of only asking "could this segment contain the point?" and bisecting,
//! every segment is *clipped*: each Cartesian axis `k` yields a scalar spline
//! `g_k(t) = X_k(t) - p_k W(t)` whose zeros are exactly the parameters where
//! that coordinate matches, and a fat line around its graph's control
//! polygon bounds those zeros to an interval. Intersecting the axes'
//! intervals shrinks the segment directly towards the solution, so a
//! transversal hit converges in a handful of clips instead of one
//! bisection per bit of precision.

use std::collections::VecDeque;

use crate::{
    aabb::{aabb_could_contain, curve_could_meet_aabb},
    fat_line::{
        Stalled, carried_width, extent, fat_line_zeros, greville_abscissae, restriction, stalled,
    },
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

/// Every parameter at which `curve` could pass through `point`: the
/// [`Scalar::union`] of every converged segment's clipped parameter
/// interval, exploring breadth-first up to `max_nodes` segments. `None` only
/// when every part of the domain was rejected: running out of budget is
/// never read as "not contained" (nor as "contained") — it is an error, since the search is
/// incomplete (see `surface.md` §5).
///
/// Each segment is clipped (see [`clip`]). An empty clip rejects it. While
/// the clip keeps shrinking the segment it is restricted and clipped again
/// ([`restriction`]). Once it stalls — several solutions or a tangency in
/// one segment, or the resolution of the data reached — the segment is
/// reported with its *clipped* interval (tighter than its domain, and still
/// an enclosure) if it has converged, and bisected otherwise ([`stalled`]).
/// `min_subdivision_size` thus only decides when to stop *bisecting*: a
/// segment that clipping keeps shrinking is never cut short, so a point
/// merely within `min_subdivision_size` of the curve is still rejected.
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
    // A straight segment is tested as itself, not its box, before anything
    // is cloned (see `curve_could_meet_aabb`).
    let mut point_box = [S::ENTIRE; 3];
    point_box[..C].copy_from_slice(&point.to_array());
    if !curve_could_meet_aabb(curve, &point_box, C) {
        return Ok(None);
    }
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

        // Cheap prefilter against the cached bounding box.
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

        let ranges = [seg.domain()];
        if let Some(bounds) = restriction(&[t_hat], &ranges)?
            && let Ok(restricted) = seg.sub_curve(bounds[0].0, bounds[0].1)
        {
            queue.push_back(restricted);
            continue;
        }

        let sizes = [extent([seg.control_points.iter().copied()])];
        // The query point's own width counts too, like a second object's.
        let point_width = (0..C)
            .map(|k| point[k].width().to_f64())
            .fold(0.0, f64::max);
        let carried = carried_width(&seg.control_points).max(point_width);
        let halves = match stalled(&ranges, &sizes, carried, min_subdivision_size) {
            Stalled::Converged => None,
            Stalled::Bisect(order) if order.is_empty() => None,
            // Cannot split (e.g. midpoint already at multiplicity p+1).
            Stalled::Bisect(_) => seg.split_mid().ok(),
        };
        match halves {
            Some((left, right)) => {
                queue.push_back(left);
                queue.push_back(right);
            }
            None => report(t_hat),
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
    const MIN_SUBDIVISION_SIZE: f64 = 1e-3;

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
        let found = curve_could_contain(c, &p, MAX, S::from_f64(MIN_SUBDIVISION_SIZE))
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
        let found = curve_could_contain(&c, &p, MAX, S::from_f64(MIN_SUBDIVISION_SIZE))
            .unwrap()
            .unwrap();
        assert!(
            found
                .width()
                .definitely_less(S::from_f64(MIN_SUBDIVISION_SIZE)),
            "{found:?}"
        );
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
            curve_could_contain(
                &quarter_circle::<S>(),
                &off(0.6, 0.6, 0.),
                MAX,
                f(MIN_SUBDIVISION_SIZE)
            )
            .unwrap()
            .is_none()
        );
        // Beside the line, beyond its end.
        assert!(
            curve_could_contain(
                &line::<S>(),
                &off(1.5, 0., 0.),
                MAX,
                f(MIN_SUBDIVISION_SIZE)
            )
            .unwrap()
            .is_none()
        );
        assert!(
            curve_could_contain(
                &line::<S>(),
                &off(0.5, 0.1, 0.),
                MAX,
                f(MIN_SUBDIVISION_SIZE)
            )
            .unwrap()
            .is_none()
        );
        assert!(
            curve_could_contain(
                &wiggle::<S>(),
                &off(2.5, 0.5, 0.5),
                MAX,
                f(MIN_SUBDIVISION_SIZE)
            )
            .unwrap()
            .is_none()
        );
    }
    #[test]
    fn misses_off_curve_points() {
        for_all_scalars!(check_misses_off_curve_points);
    }

    /// `min_subdivision_size` bounds bisection, not accuracy: a point off the
    /// arc by far less than it is still rejected, because clipping keeps
    /// shrinking the segment until it proves the miss. (The offset stays well
    /// above `ScalInFPA64`'s 2^-32 resolution, below which a report is the
    /// honest answer.)
    fn check_misses_points_closer_than_min_subdivision_size<S: Scalar>() {
        let f = S::from_f64;
        let r = 1. + 1e-7;
        let (x, y) = (r * 0.6, r * 0.8);
        let p = Vector3::from_array([f(x), f(y), f(0.)]);
        let found =
            curve_could_contain(&quarter_circle::<S>(), &p, MAX, f(MIN_SUBDIVISION_SIZE)).unwrap();
        assert!(found.is_none(), "{found:?}");
    }
    #[test]
    fn misses_points_closer_than_min_subdivision_size() {
        for_all_scalars!(check_misses_points_closer_than_min_subdivision_size);
    }
}
