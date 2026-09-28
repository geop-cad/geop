use geop_core_math::{convex_hull::ConvexHull, geop_error::GeopResult, scalars::Scalar};

use super::NurbSurface;
use crate::nurb_curve::dehomogenize;

impl<S: Scalar> NurbSurface<S, 4> {
    /// Convex hull of the surface patch's Cartesian (dehomogenized) control
    /// points, stored row-major (matching [`Self::control_points`]).
    ///
    /// By the convex-hull property of the NURBS basis, every point on the
    /// patch lies within this hull.
    pub fn convex_hull(&self) -> ConvexHull<S, 3> {
        ConvexHull::new(dehomogenize::<S, 4, 3>(&self.control_points))
    }

    /// `(u_size, v_size)`: the control net's own extent along each
    /// dimension, each the *larger* of the two corner-to-corner edges
    /// running that direction (u: the `v=0` row's own span *and* the
    /// `v=max` row's own span; v: the `u=0` column's *and* the `u=max`
    /// column's). Sampling only one row/column (as an earlier version of
    /// this did) badly underestimates that dimension's true extent for any
    /// patch that includes a coordinate-singular pole as one of its own
    /// `u`/`v` boundaries (e.g. a `revolve`d disk cap row very close to its
    /// own apex): every point along that *particular* row collapses to
    /// nearly the same 3-D point regardless of how wide its own parameter
    /// range still is, while the *other* row does still spread out — so a
    /// single-row/column measurement can report a dimension as
    /// already-converged when it isn't, and a subdivision search relying on
    /// that (see `intersection::curve_surface_intersect`) never picks that
    /// dimension to split, degrading into combinatorial blowup along
    /// whatever it splits instead.
    fn extents(&self) -> (S, S) {
        let hull = self.convex_hull();
        let (nu, nv) = (self.num_u(), self.num_v());
        let (u0v0, u0vn, unv0, unvn) = (
            hull.points[0],
            hull.points[nv - 1],
            hull.points[(nu - 1) * nv],
            hull.points[(nu - 1) * nv + (nv - 1)],
        );
        let u_size_at_v0 = unv0.sub(&u0v0).norm();
        let u_size_at_vmax = unvn.sub(&u0vn).norm();
        let v_size_at_u0 = u0vn.sub(&u0v0).norm();
        let v_size_at_umax = unvn.sub(&unv0).norm();
        let u_size = if u_size_at_vmax.definitely_greater(u_size_at_v0) {
            u_size_at_vmax
        } else {
            u_size_at_v0
        };
        let v_size = if v_size_at_umax.definitely_greater(v_size_at_u0) {
            v_size_at_umax
        } else {
            v_size_at_u0
        };
        (u_size, v_size)
    }

    /// `max(u_size, v_size)`, measured via the convex hull's control-net
    /// edges (see [`Self::extents`]), used as a convergence measure for
    /// subdivision algorithms.
    pub fn size(&self) -> S {
        let (u_size, v_size) = self.extents();
        if v_size.definitely_greater(u_size) {
            v_size
        } else {
            u_size
        }
    }

    /// Split along the longer of the u/v dimensions (measured via
    /// [`Self::extents`]) at that dimension's domain midpoint.
    pub fn split_mid(&self) -> GeopResult<(NurbSurface<S, 4>, NurbSurface<S, 4>)> {
        let (u_size, v_size) = self.extents();
        let along_u = u_size.definitely_greater(v_size);
        if along_u {
            self.split_u_mid()
        } else {
            self.split_v_mid()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::NurbSurface;
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

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn flat_patch<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn lifted_patch<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 1., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_flat_patch_contains_surface_points<S: Scalar>() {
        let s = flat_patch::<S>();
        let hull = s.convex_hull();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(hull.could_contain(&p));
    }
    #[test]
    fn flat_patch_contains_surface_points() {
        for_all_scalars!(check_flat_patch_contains_surface_points);
    }

    fn check_flat_patch_excludes_elevated_point<S: Scalar>() {
        let hull = flat_patch::<S>().convex_hull();
        assert!(hull.definitely_not_contains(&v3(0.5, 0.5, 5.)));
    }
    #[test]
    fn flat_patch_excludes_elevated_point() {
        for_all_scalars!(check_flat_patch_excludes_elevated_point);
    }

    fn check_flat_patch_excludes_point_outside_bounds<S: Scalar>() {
        let hull = flat_patch::<S>().convex_hull();
        assert!(hull.definitely_not_contains(&v3(2., 0.5, 0.)));
    }
    #[test]
    fn flat_patch_excludes_point_outside_bounds() {
        for_all_scalars!(check_flat_patch_excludes_point_outside_bounds);
    }

    fn check_lifted_patch_contains_midheight_point<S: Scalar>() {
        let hull = lifted_patch::<S>().convex_hull();
        // Centroid of the lifted patch's control points lies at height 0.25.
        assert!(hull.could_contain(&v3(0.5, 0.5, 0.25)));
    }
    #[test]
    fn lifted_patch_contains_midheight_point() {
        for_all_scalars!(check_lifted_patch_contains_midheight_point);
    }

    fn check_lifted_patch_excludes_far_above<S: Scalar>() {
        let hull = lifted_patch::<S>().convex_hull();
        assert!(hull.definitely_not_contains(&v3(0.5, 0.5, 5.)));
    }
    #[test]
    fn lifted_patch_excludes_far_above() {
        for_all_scalars!(check_lifted_patch_excludes_far_above);
    }
}
