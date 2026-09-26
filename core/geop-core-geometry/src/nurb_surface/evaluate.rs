use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::NurbSurface;

/// Find the knot span: last index k in [degree, n] where knots[k] <= t < knots[k+1].
pub(super) fn find_span<S: Scalar>(
    degree: usize,
    knots: &[S],
    n: usize,
    t: S,
) -> GeopResult<usize> {
    let p = degree;
    if t.definitely_less(knots[p]) || t.definitely_greater(knots[n + 1]) {
        return Err(GeopError::new(format!(
            "parameter t={} out of domain [{}, {}]",
            t,
            knots[p],
            knots[n + 1]
        )));
    }
    if !t.definitely_less(knots[n + 1]) {
        for k in (p..=n).rev() {
            if knots[k].definitely_less(knots[n + 1]) {
                return Ok(k);
            }
        }
        return Ok(p);
    }
    for k in p..=n {
        if !t.definitely_less(knots[k]) && t.definitely_less(knots[k + 1]) {
            return Ok(k);
        }
    }
    Err(GeopError::new("could not find knot span"))
}

/// De Boor triangular recursion in homogeneous space; generic over CP dimension D.
pub(super) fn de_boor<S: Scalar, const D: usize>(
    degree: usize,
    knots: &[S],
    points: &[Vector<S, D>],
    t: S,
    span: usize,
) -> Vector<S, D> {
    let p = degree;
    let mut d: Vec<Vector<S, D>> = (0..=p).map(|j| points[span - p + j]).collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = span - p + j;
            let denom = knots[i + p - r + 1].sub(knots[i]);
            let alpha = if denom.could_be_equal(S::ZERO) {
                S::ZERO
            } else {
                t.sub(knots[i]).div(denom).unwrap_or(S::ZERO)
            };
            d[j] = Vector::interpolate(&d[j - 1], &d[j], alpha);
        }
    }
    d[p]
}

// ── 3-D surface ──────────────────────────────────────────────────────────────

impl<S: Scalar> NurbSurface<S, 4> {
    /// Evaluate the surface at `(u, v)`, returning a 3-D Cartesian point.
    pub fn evaluate(&self, u: S, v: S) -> GeopResult<Vector3<S>> {
        let p = self.degree_u;
        let q = self.degree_v;
        let nu = self.num_u;
        let nv = self.num_v;

        let span_u = find_span(p, &self.knot_vector_u, nu - 1, u)?;

        let mut col_pts: Vec<Vector<S, 4>> = Vec::with_capacity(nv);
        for j in 0..nv {
            let row: Vec<Vector<S, 4>> = (0..nu).map(|i| self.control_points[i * nv + j]).collect();
            col_pts.push(de_boor(p, &self.knot_vector_u, &row, u, span_u));
        }

        let span_v = find_span(q, &self.knot_vector_v, nv - 1, v)?;
        let hw = de_boor(q, &self.knot_vector_v, &col_pts, v, span_v);

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

impl<S: Scalar> geop_core_math::primitives::scene::RasterizableSurface<S> for NurbSurface<S, 4> {
    fn eval_at(&self, u: S, v: S) -> GeopResult<Vector3<S>> {
        self.evaluate(u, v)
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
