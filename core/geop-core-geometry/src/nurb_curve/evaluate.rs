use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3},
};

use super::NurbCurve;
use crate::spline::{de_boor, find_spans};

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// The Cartesian point at `t`, in `C = D - 1` coordinates: the union of
    /// its value on every knot span `t` reaches (see [`find_spans`]).
    fn cartesian<const C: usize>(&self, t: S) -> GeopResult<Vector<S, C>> {
        let n = self.control_points.len() - 1;
        let mut out: Option<Vector<S, C>> = None;
        for (span, piece) in find_spans(self.degree, &self.knot_vector, n, t)? {
            let hw = de_boor(
                self.degree,
                &self.knot_vector,
                &self.control_points,
                piece,
                span,
            );
            let w = hw[D - 1];
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
            let mut point = Vector::<S, C>::zero();
            for c in 0..C {
                point[c] = hw[c].mul(inv_w);
            }
            out = Some(match out {
                Some(other) => other.union(&point),
                None => point,
            });
        }
        out.ok_or_else(|| GeopError::new(format!("NurbCurve::evaluate(t={t:?}): no knot span")))
    }
}

// ── 3-D curve: evaluate returns Vector3 ─────────────────────────────────────

impl<S: Scalar> NurbCurve<S, 4> {
    /// Evaluate the 3-D NURBS curve at `t`, returning a Cartesian `Vector3`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector3<S>> {
        self.cartesian(t)
    }
}

// ── 2-D curve (pcurve): evaluate returns Vector2 ─────────────────────────────

impl<S: Scalar> NurbCurve<S, 3> {
    /// Evaluate the 2-D pcurve at `t`, returning a Cartesian `Vector2`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector2<S>> {
        self.cartesian(t)
    }
}

#[cfg(test)]
mod tests {
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

    /// Over an interval reaching across many knot spans, a curve is the union
    /// of its spans' parts, not one span's cubic extrapolated over all of
    /// them: a quarter circle of radius 2.25 interpolated through 49 points
    /// came out 57 wide over its whole domain, and over an eighth of it
    /// eight times as wide as that eighth.
    fn check_a_wide_parameter_encloses_the_curve_tightly<S: Scalar>() {
        let r = 2.25;
        let points: Vec<_> = (0..=48)
            .map(|k| {
                let a = std::f64::consts::FRAC_PI_2 * k as f64 / 48.0;
                Vector3::from_array([r * a.cos(), r * a.sin(), 0.0].map(S::from_f64))
            })
            .collect();
        let curve = NurbCurve::<S, 4>::interpolate(&points, 3).unwrap();
        let (t0, t1) = curve.domain();
        for (lo, hi) in [
            (t0, t1),
            (t0, S::from_f64(0.125)),
            (S::from_f64(0.3), S::from_f64(0.55)),
        ] {
            let box_ = curve.evaluate(lo.union(hi)).unwrap();
            for k in 0..=64 {
                let t = match k {
                    0 => lo,
                    64 => hi,
                    _ => S::interpolate(lo, hi, S::from_ratio(k, 64).unwrap()).sharpen(),
                };
                let p = curve.evaluate(t).unwrap();
                assert!(box_.could_be_equal(&p), "{t:?}: {p:?} outside {box_:?}");
            }
            // The arc between `lo` and `hi` spans at most its length on
            // either axis; each span's part, evaluated over all of it in
            // interval arithmetic, comes out somewhat wider than that.
            let length = r * std::f64::consts::FRAC_PI_2 * (hi.to_f64() - lo.to_f64());
            for c in 0..2 {
                let width = box_[c].width().to_f64();
                assert!(width < 1.5 * length, "{c}: {width} over {lo:?}..{hi:?}");
            }
        }
    }
    #[test]
    fn a_wide_parameter_encloses_the_curve_tightly() {
        for_all_scalars!(check_a_wide_parameter_encloses_the_curve_tightly);
    }
}
