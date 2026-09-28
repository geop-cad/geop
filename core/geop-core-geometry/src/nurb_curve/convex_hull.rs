use geop_core_math::{convex_hull::ConvexHull, scalars::Scalar, vector::Vector};

use super::NurbCurve;

/// Dehomogenize `control_points` (`D`-dimensional homogeneous, last component
/// the weight) into `C`-dimensional Cartesian points (`C` = `D - 1`, passed
/// explicitly since Rust's stable const generics can't express `D - 1` in a
/// single generic parameter's bound).
///
/// Infallible: every weight of a `NurbCurve`/`NurbSurface` is definitely
/// positive by construction (see `NurbCurve::try_new`), so the division
/// cannot fail. `pub(crate)`, not private: `fat_axis` and the surface hull
/// need the exact same points.
pub(crate) fn dehomogenize<S: Scalar, const D: usize, const C: usize>(
    control_points: &[Vector<S, D>],
) -> Vec<Vector<S, C>> {
    control_points
        .iter()
        .map(|p| {
            let inv_w = S::ONE
                .div(p[D - 1])
                .expect("weights are definitely positive by construction");
            let mut pt = Vector::<S, C>::zero();
            for c in 0..C {
                pt[c] = p[c].mul(inv_w);
            }
            pt
        })
        .collect()
}

fn hull_size<S: Scalar, const N: usize>(hull: &ConvexHull<S, N>) -> S {
    hull.points[hull.points.len() - 1]
        .sub(&hull.points[0])
        .norm()
}

/// Bridges `NurbCurve<S, D>`'s pair of concrete `convex_hull()`/`size()`
/// impls (`D=4` → `ConvexHull<S,3>`, `D=3` → `ConvexHull<S,2>`) so code
/// generic over `D` (e.g. `curve_curve_intersect`, `curve_could_contain`) can
/// call them via a `where NurbCurve<S, D>: HasConvexHull<S, C>` bound —
/// Rust's stable const generics can't express `C = D - 1` directly in a
/// single generic function.
pub trait HasConvexHull<S: Scalar, const C: usize> {
    fn convex_hull(&self) -> ConvexHull<S, C>;
    fn size(&self) -> S;
}

impl<S: Scalar> NurbCurve<S, 4> {
    /// Convex hull of the curve's Cartesian (dehomogenized) control points.
    ///
    /// By the convex-hull property of the NURBS basis, every point on the
    /// curve lies within this hull.
    pub fn convex_hull(&self) -> ConvexHull<S, 3> {
        ConvexHull::new(dehomogenize(&self.control_points))
    }

    /// Chord length of the curve's convex hull, used as a convergence
    /// measure for subdivision algorithms.
    pub fn size(&self) -> S {
        hull_size(&self.convex_hull())
    }
}

impl<S: Scalar> HasConvexHull<S, 3> for NurbCurve<S, 4> {
    fn convex_hull(&self) -> ConvexHull<S, 3> {
        self.convex_hull()
    }
    fn size(&self) -> S {
        self.size()
    }
}

impl<S: Scalar> NurbCurve<S, 3> {
    /// Convex hull of the pcurve's Cartesian (dehomogenized) control points.
    ///
    /// By the convex-hull property of the NURBS basis, every point on the
    /// curve lies within this hull.
    pub fn convex_hull(&self) -> ConvexHull<S, 2> {
        ConvexHull::new(dehomogenize(&self.control_points))
    }

    /// Chord length of the pcurve's convex hull, used as a convergence
    /// measure for subdivision algorithms.
    pub fn size(&self) -> S {
        hull_size(&self.convex_hull())
    }
}

impl<S: Scalar> HasConvexHull<S, 2> for NurbCurve<S, 3> {
    fn convex_hull(&self) -> ConvexHull<S, 2> {
        self.convex_hull()
    }
    fn size(&self) -> S {
        self.size()
    }
}

#[cfg(test)]
mod tests {
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

    fn check_hull_contains_curve_points<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(0.5, 1., 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let hull = c.convex_hull();
        let p = c.evaluate(S::from_f64(0.5)).unwrap();
        assert!(hull.could_contain(&p));
    }
    #[test]
    fn hull_contains_curve_points() {
        for_all_scalars!(check_hull_contains_curve_points);
    }

    fn check_hull_points_match_dehomogenized_control_points<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(0.5, 2., 0., 2.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let hull = c.convex_hull();
        assert!(hull.points[0][0].could_be_equal(S::ZERO));
        // (0.5, 2, 0, 2) dehomogenizes to (0.25, 1, 0)
        assert!(hull.points[1][0].could_be_equal(S::from_f64(0.25)));
        assert!(hull.points[1][1].could_be_equal(S::ONE));
        assert!(hull.points[2][0].could_be_equal(S::ONE));
    }
    #[test]
    fn hull_points_match_dehomogenized_control_points() {
        for_all_scalars!(check_hull_points_match_dehomogenized_control_points);
    }

    fn check_hull_excludes_far_point<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let hull = c.convex_hull();
        assert!(
            hull.definitely_not_contains(&geop_core_math::vector::Vector3::from_array([
                f(0.5),
                f(5.),
                f(0.),
            ]))
        );
    }
    #[test]
    fn hull_excludes_far_point() {
        for_all_scalars!(check_hull_excludes_far_point);
    }

    fn check_2d_hull_contains_pcurve_points<S: Scalar>() {
        let f = S::from_f64;
        let c: crate::nurb_curve::NurbCurve2D<S> = NurbCurve::try_new(
            1,
            vec![
                geop_core_math::vector::Vector3::from_array([f(0.), f(0.), f(1.)]),
                geop_core_math::vector::Vector3::from_array([f(1.), f(1.), f(1.)]),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let hull = c.convex_hull();
        assert!(
            hull.could_contain(&geop_core_math::vector::Vector2::from_array([
                f(0.5),
                f(0.5)
            ]))
        );
        assert!(
            hull.definitely_not_contains(&geop_core_math::vector::Vector2::from_array([
                f(5.),
                f(5.)
            ]))
        );
    }
    #[test]
    fn hull_2d_contains_pcurve_points() {
        for_all_scalars!(check_2d_hull_contains_pcurve_points);
    }
}
