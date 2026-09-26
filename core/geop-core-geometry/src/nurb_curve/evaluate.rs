use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3},
};

use super::NurbCurve;

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Returns span index k where u[k] <= t < u[k+1].
    pub(super) fn find_knot_span(&self, t: S) -> GeopResult<usize> {
        let n = self.control_points.len() - 1;
        let p = self.degree;
        let u = &self.knot_vector;

        if t.definitely_less(u[p]) || t.definitely_greater(u[n + 1]) {
            return Err(GeopError::new(format!(
                "parameter t={} out of domain [{}, {}]",
                t,
                u[p],
                u[n + 1]
            )));
        }

        if !t.definitely_less(u[n + 1]) {
            for k in (p..=n).rev() {
                if u[k].definitely_less(u[n + 1]) {
                    return Ok(k);
                }
            }
            return Ok(p);
        }

        for k in p..=n {
            if !t.definitely_less(u[k]) && t.definitely_less(u[k + 1]) {
                return Ok(k);
            }
        }

        Err(GeopError::new("could not find knot span"))
    }

    /// De Boor triangular recursion; returns the homogeneous result `Vector<S, D>`.
    pub(super) fn de_boor(&self, t: S, span: usize) -> Vector<S, D> {
        let p = self.degree;
        let u = &self.knot_vector;
        let mut d: Vec<Vector<S, D>> = (0..=p).map(|j| self.control_points[span - p + j]).collect();

        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = span - p + j;
                let denom = u[i + p - r + 1].sub(u[i]);
                let alpha = if denom.could_be_equal(S::ZERO) {
                    S::ZERO
                } else {
                    t.sub(u[i]).div(denom).unwrap_or(S::ZERO)
                };
                d[j] = Vector::interpolate(&d[j - 1], &d[j], alpha);
            }
        }
        d[p]
    }
}

// ── 3-D curve: evaluate returns Vector3 ─────────────────────────────────────

impl<S: Scalar> NurbCurve<S, 4> {
    /// Evaluate the 3-D NURBS curve at `t`, returning a Cartesian `Vector3`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector3<S>> {
        let span = self.find_knot_span(t)?;
        let hw = self.de_boor(t, span);
        let w = hw[3];
        if w.could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "weight is zero at evaluation point (t={t:?}, span={span}, homogeneous de_boor result w={w:?}, degree={}, knot_vector={:?}, control_points={:?})",
                self.degree, self.knot_vector, self.control_points
            )));
        }
        let inv_w = S::ONE.div(w).with_context(&|e: GeopError| {
            e.with_context(format!(
                "NurbCurve::evaluate(t={t}): degree={}, knot_vector={:?}, control_points={:?}",
                self.degree, self.knot_vector, self.control_points
            ))
        })?;
        let mut result = Vector3::zero();
        for c in 0..3 {
            result[c] = hw[c].mul(inv_w);
        }
        Ok(result)
    }
}

impl<S: Scalar> geop_core_math::primitives::scene::RasterizableCurve<S> for NurbCurve<S, 4> {
    fn eval_at(&self, t: S) -> GeopResult<Vector3<S>> {
        self.evaluate(t)
    }
}

// ── 2-D curve (pcurve): evaluate returns Vector2 ─────────────────────────────

impl<S: Scalar> NurbCurve<S, 3> {
    /// Evaluate the 2-D pcurve at `t`, returning a Cartesian `Vector2`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector2<S>> {
        let span = self.find_knot_span(t)?;
        let hw = self.de_boor(t, span);
        let w = hw[2];
        if w.could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "weight is zero at evaluation point (t={t:?}, span={span}, homogeneous de_boor result w={w:?}, degree={}, knot_vector={:?}, control_points={:?})",
                self.degree, self.knot_vector, self.control_points
            )));
        }
        let inv_w = S::ONE.div(w).with_context(&|e: GeopError| {
            e.with_context(format!(
                "NurbCurve::evaluate(t={t}): degree={}, knot_vector={:?}, control_points={:?}",
                self.degree, self.knot_vector, self.control_points
            ))
        })?;
        let mut result = Vector2::zero();
        for c in 0..2 {
            result[c] = hw[c].mul(inv_w);
        }
        Ok(result)
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

    fn line_curve<S: Scalar>() -> NurbCurve<S, 4> {
        NurbCurve::try_new(
            1,
            vec![pt(0.0, 0.0, 0.0, 1.0), pt(1.0, 0.0, 0.0, 1.0)],
            vec![
                S::from_f64(0.0),
                S::from_f64(0.0),
                S::from_f64(1.0),
                S::from_f64(1.0),
            ],
        )
        .unwrap()
    }

    fn check_line_at_start<S: Scalar>() {
        let c = line_curve::<S>();
        let p = c.evaluate(S::ZERO).unwrap();
        assert!(p[0].could_be_equal(S::ZERO));
        assert!(p[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn line_at_start() {
        for_all_scalars!(check_line_at_start);
    }

    fn check_line_at_end<S: Scalar>() {
        let c = line_curve::<S>();
        let p = c.evaluate(S::ONE).unwrap();
        assert!(p[0].could_be_equal(S::ONE));
        assert!(p[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn line_at_end() {
        for_all_scalars!(check_line_at_end);
    }

    fn check_line_at_midpoint<S: Scalar>() {
        let c = line_curve::<S>();
        let p = c.evaluate(S::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(0.5)));
    }
    #[test]
    fn line_at_midpoint() {
        for_all_scalars!(check_line_at_midpoint);
    }

    fn check_out_of_domain_returns_err<S: Scalar>() {
        let c = line_curve::<S>();
        assert!(c.evaluate(S::from_f64(-0.1)).is_err());
        assert!(c.evaluate(S::from_f64(1.1)).is_err());
    }
    #[test]
    fn out_of_domain_returns_err() {
        for_all_scalars!(check_out_of_domain_returns_err);
    }

    fn check_quadratic_midpoint<S: Scalar>() {
        let curve = NurbCurve::try_new(
            2,
            vec![
                pt(0.0, 0.0, 0.0, 1.0),
                pt(0.5, 0.0, 0.0, 1.0),
                pt(1.0, 0.0, 0.0, 1.0),
            ],
            vec![
                S::from_f64(0.0),
                S::from_f64(0.0),
                S::from_f64(0.0),
                S::from_f64(1.0),
                S::from_f64(1.0),
                S::from_f64(1.0),
            ],
        )
        .unwrap();
        let p = curve.evaluate(S::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(0.5)));
        assert!(p[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn quadratic_midpoint() {
        for_all_scalars!(check_quadratic_midpoint);
    }

    /// Regression check for a `weight is zero at evaluation point` failure
    /// observed from `remesh` on a degree-1, weight-1-constant, [0,0,1,1]
    /// curve — data that on paper cannot produce a near-zero interpolated
    /// weight (both endpoint weights are exactly 1). Reproduces the exact
    /// control points/knots/`t` from that failure's error context to check
    /// whether `evaluate` itself is at fault, independent of `remesh`.
    fn check_weight_one_constant_line_does_not_report_zero_weight<S: Scalar>() {
        let curve = NurbCurve::try_new(
            1,
            vec![pt(0.5, 0.5, 0.5, 1.0), pt(0.5, 0.5, -0.5, 1.0)],
            vec![
                S::from_f64(0.0),
                S::from_f64(0.0),
                S::from_f64(1.0),
                S::from_f64(1.0),
            ],
        )
        .unwrap();
        let p = curve.evaluate(S::from_f64(0.972)).unwrap();
        assert!(
            p[2].could_be_equal(S::from_f64(0.5 - 1.0 * 0.972)),
            "p={p:?}"
        );
    }
    #[test]
    fn weight_one_constant_line_does_not_report_zero_weight() {
        for_all_scalars!(check_weight_one_constant_line_does_not_report_zero_weight);
    }

    fn check_everything_matches_any_point<S: Scalar>() {
        let c = NurbCurve::<S, 4>::everything();
        let p = c.evaluate(S::from_f64(7.0)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(123.456)));
        assert!(p[1].could_be_equal(S::from_f64(-9.0)));
        assert!(p[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn everything_matches_any_point() {
        for_all_scalars!(check_everything_matches_any_point);
    }
}
