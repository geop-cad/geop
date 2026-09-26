use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

use super::NurbCurve;
use crate::aabb::compute_aabb;

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Return the derivative of this curve as a new `NurbCurve` of degree `p − 1`.
    ///
    /// The returned curve is the derivative of the *homogeneous* B-spline.
    /// Evaluating it at `t` yields the homogeneous tangent vector.  To obtain
    /// the Cartesian tangent `C′(t)`, apply the quotient rule:
    ///
    /// ```text
    /// C′(t) = (A′(t) − w′(t)·C(t)) / w(t)
    /// ```
    ///
    /// Errors if `self.degree == 0`.
    pub fn derivative(&self) -> GeopResult<NurbCurve<S, D>> {
        let p = self.degree;
        if p == 0 {
            return Err(GeopError::new(
                "derivative is undefined for a degree-0 curve",
            ));
        }

        let n = self.control_points.len() - 1;
        let u = &self.knot_vector;
        let p_s = S::from_i64(p as i64);

        let mut deriv_pts: Vec<Vector<S, D>> = Vec::with_capacity(n);
        for i in 0..n {
            let den = u[i + p + 1].sub(u[i + 1]);
            let mut d = Vector::<S, D>::zero();
            if den.definitely_not_equal(S::ZERO) {
                let scale = p_s.div(den).unwrap_or(S::ZERO);
                for c in 0..D {
                    d[c] = self.control_points[i + 1][c]
                        .sub(self.control_points[i][c])
                        .mul(scale);
                }
            }
            deriv_pts.push(d);
        }

        let deriv_knots: Vec<S> = u[1..=n + p].to_vec();

        // Computed the same way every other constructor here does (see
        // `compute_aabb`'s own doc comment) — this matches what
        // `NurbCurve::evaluate` already does with a derivative curve's own
        // "weight" component (it has no separate rational-vs-derivative
        // special case; it just divides by the last coordinate like it
        // would for any other curve), so this stays consistent with that
        // rather than falling back to a match-anything placeholder that
        // would silently disable any future caller relying on `.aabb`
        // (not just today's intersection-search prefilters).
        let aabb = compute_aabb(&deriv_pts);

        Ok(NurbCurve {
            degree: p - 1,
            control_points: deriv_pts,
            knot_vector: deriv_knots,
            aabb,
        })
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

    fn check_degree_zero_errors<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let d = c.derivative().unwrap();
        assert!(d.derivative().is_err());
    }
    #[test]
    fn degree_zero_errors() {
        for_all_scalars!(check_degree_zero_errors);
    }

    fn check_structural_properties<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            3,
            vec![
                pt(0., 0., 0., 1.),
                pt(1., 1., 0., 1.),
                pt(2., 1., 0., 1.),
                pt(3., 0., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(0.), f(1.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let d1 = c.derivative().unwrap();
        assert_eq!(d1.degree, 2);
        assert_eq!(d1.control_points.len(), 3);
        assert_eq!(
            d1.knot_vector.len(),
            d1.control_points.len() + d1.degree + 1
        );

        let d2 = d1.derivative().unwrap();
        assert_eq!(d2.degree, 1);
        assert_eq!(d2.control_points.len(), 2);
        assert_eq!(
            d2.knot_vector.len(),
            d2.control_points.len() + d2.degree + 1
        );

        let d3 = d2.derivative().unwrap();
        assert_eq!(d3.degree, 0);
        assert_eq!(d3.control_points.len(), 1);
        assert_eq!(
            d3.knot_vector.len(),
            d3.control_points.len() + d3.degree + 1
        );
    }
    #[test]
    fn structural_properties() {
        for_all_scalars!(check_structural_properties);
    }

    fn check_quadratic_derivative_control_points<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(0.5, 0.5, 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let d = c.derivative().unwrap();
        assert_eq!(d.degree, 1);
        assert_eq!(d.control_points.len(), 2);

        assert!(d.control_points[0][0].could_be_equal(S::ONE));
        assert!(d.control_points[0][1].could_be_equal(S::ONE));
        assert!(d.control_points[0][2].could_be_equal(S::ZERO));
        assert!(d.control_points[0][3].could_be_equal(S::ZERO));

        assert!(d.control_points[1][0].could_be_equal(S::ONE));
        assert!(d.control_points[1][1].could_be_equal(S::ONE.neg()));
        assert!(d.control_points[1][2].could_be_equal(S::ZERO));
        assert!(d.control_points[1][3].could_be_equal(S::ZERO));
    }
    #[test]
    fn quadratic_derivative_control_points() {
        for_all_scalars!(check_quadratic_derivative_control_points);
    }

    fn check_knot_vector_is_trimmed<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(0.5, 0.5, 0., 1.), pt(1., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let d = c.derivative().unwrap();
        let expected = [0., 0., 1., 1.];
        for (got, &exp) in d.knot_vector.iter().zip(expected.iter()) {
            assert!(
                got.could_be_equal(f(exp)),
                "expected {exp}, got {}",
                got.to_f64()
            );
        }
    }
    #[test]
    fn knot_vector_is_trimmed() {
        for_all_scalars!(check_knot_vector_is_trimmed);
    }

    fn check_weighted_derivative_is_evaluatable<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(1., 0., 0., 2.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();

        let d = c.derivative().unwrap();
        assert_eq!(d.degree, 0);

        assert!(d.control_points[0][0].could_be_equal(S::ONE));
        assert!(d.control_points[0][3].could_be_equal(S::ONE));

        let a_prime = d.evaluate(f(0.5)).unwrap();
        assert!(a_prime[0].could_be_equal(S::ONE));
        assert!(a_prime[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn weighted_derivative_is_evaluatable() {
        for_all_scalars!(check_weighted_derivative_is_evaluatable);
    }
}
