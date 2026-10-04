use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::NurbSurface;
use crate::spline::{de_boor, find_span};

// ── 3-D surface ──────────────────────────────────────────────────────────────

impl<S: Scalar> NurbSurface<S, 4> {
    /// Evaluate the surface at `(u, v)`, returning a 3-D Cartesian point.
    ///
    /// Only the `(p + 1) × (q + 1)` control points acting on the spans of
    /// `u` and `v` are touched: each column of them is evaluated at `u`,
    /// and those at `v`, every de Boor run over its local points with the
    /// knots shifted to match (the same arithmetic as over all of them).
    pub fn evaluate(&self, u: S, v: S) -> GeopResult<Vector3<S>> {
        let p = self.degree_u;
        let q = self.degree_v;
        let nu = self.num_u;
        let nv = self.num_v;

        let span_u = find_span(p, &self.knot_vector_u, nu - 1, u)?;
        let span_v = find_span(q, &self.knot_vector_v, nv - 1, v)?;
        let (base_u, base_v) = (span_u - p, span_v - q);

        let col_pts: Vec<Vector<S, 4>> = (base_v..=span_v)
            .map(|j| {
                let column: Vec<Vector<S, 4>> = (base_u..=span_u)
                    .map(|i| self.control_points[i * nv + j])
                    .collect();
                de_boor(p, &self.knot_vector_u[base_u..], &column, u, p)
            })
            .collect();
        let hw = de_boor(q, &self.knot_vector_v[base_v..], &col_pts, v, q);

        let w = hw[3];
        if w.could_be_equal(S::ZERO) {
            return Err(GeopError::new("weight is zero at evaluation point"));
        }
        let inv_w = S::ONE.div(w)?;
        let mut result = Vector3::zero();
        for c in 0..3 {
            result[c] = hw[c].mul(inv_w);
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::super::NurbSurface;
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

    fn bilinear<S: Scalar>() -> NurbSurface<S, 4> {
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

    fn check_bilinear_corner_00<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::ZERO, S::ZERO).unwrap();
        assert!(p[0].could_be_equal(S::ZERO));
        assert!(p[1].could_be_equal(S::ZERO));
        assert!(p[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn bilinear_corner_00() {
        for_all_scalars!(check_bilinear_corner_00);
    }

    fn check_bilinear_corner_10<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::ONE, S::ZERO).unwrap();
        assert!(p[0].could_be_equal(S::ONE));
        assert!(p[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn bilinear_corner_10() {
        for_all_scalars!(check_bilinear_corner_10);
    }

    fn check_bilinear_corner_01<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::ZERO, S::ONE).unwrap();
        assert!(p[0].could_be_equal(S::ZERO));
        assert!(p[1].could_be_equal(S::ONE));
    }
    #[test]
    fn bilinear_corner_01() {
        for_all_scalars!(check_bilinear_corner_01);
    }

    fn check_bilinear_corner_11<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::ONE, S::ONE).unwrap();
        assert!(p[0].could_be_equal(S::ONE));
        assert!(p[1].could_be_equal(S::ONE));
    }
    #[test]
    fn bilinear_corner_11() {
        for_all_scalars!(check_bilinear_corner_11);
    }

    fn check_bilinear_center<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(0.5)));
        assert!(p[1].could_be_equal(S::from_f64(0.5)));
        assert!(p[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn bilinear_center() {
        for_all_scalars!(check_bilinear_center);
    }

    fn check_bilinear_mid_u_edge<S: Scalar>() {
        let s = bilinear::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::ZERO).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(0.5)));
        assert!(p[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn bilinear_mid_u_edge() {
        for_all_scalars!(check_bilinear_mid_u_edge);
    }

    fn check_out_of_domain_u_returns_err<S: Scalar>() {
        let s = bilinear::<S>();
        assert!(s.evaluate(S::from_f64(-0.1), S::from_f64(0.5)).is_err());
        assert!(s.evaluate(S::from_f64(1.1), S::from_f64(0.5)).is_err());
    }
    #[test]
    fn out_of_domain_u_returns_err() {
        for_all_scalars!(check_out_of_domain_u_returns_err);
    }

    fn check_out_of_domain_v_returns_err<S: Scalar>() {
        let s = bilinear::<S>();
        assert!(s.evaluate(S::from_f64(0.5), S::from_f64(-0.1)).is_err());
        assert!(s.evaluate(S::from_f64(0.5), S::from_f64(1.1)).is_err());
    }
    #[test]
    fn out_of_domain_v_returns_err() {
        for_all_scalars!(check_out_of_domain_v_returns_err);
    }

    fn check_quadratic_u_midpoint<S: Scalar>() {
        let f = S::from_f64;
        let s = NurbSurface::try_new(
            2,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(0.5, 0., 1., 1.),
                pt(0.5, 1., 1., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let p = s.evaluate(f(0.5), f(0.)).unwrap();
        assert!(p[0].could_be_equal(f(0.5)));
        assert!(p[1].could_be_equal(f(0.)));
        assert!(p[2].could_be_equal(f(0.5)));
    }
    #[test]
    fn quadratic_u_midpoint() {
        for_all_scalars!(check_quadratic_u_midpoint);
    }

    fn check_everything_matches_any_point<S: Scalar>() {
        let s = NurbSurface::<S, 4>::everything();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(-3.0)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(123.456)));
        assert!(p[1].could_be_equal(S::from_f64(-9.0)));
        assert!(p[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn everything_matches_any_point() {
        for_all_scalars!(check_everything_matches_any_point);
    }
}
