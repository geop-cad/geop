use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::NurbCurve;
use crate::{
    knot_insertion::pinned_clamped_end,
    spline::{centered, de_boor, find_spans},
};

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// The Cartesian point at `t`, its `D − 1` coordinates written to `out`:
    /// the union of its value on every knot span `t` reaches (see
    /// [`find_spans`]).
    ///
    /// On each span it is evaluated twice and intersected — both are
    /// enclosures of the one point: `A / W` as it is, and relative to a
    /// control point of the span ([`centered`]), `origin + A / W`. The first
    /// divides two enclosures interval arithmetic cannot correlate, so its
    /// width grows with `|A|`: an interval `t` gives a point as wide as the
    /// curve is far from the origin, rather than as wide as the stretch of
    /// curve it covers. The second is as wide as that stretch, but moving
    /// away and back costs a rounding of the origin, which near the origin
    /// is wider than the first — and wider than the control points, which
    /// containment searches clip against, say the curve is.
    ///
    /// At a clamped end, pinned sharply, the curve is its end control point.
    fn evaluate_into(&self, t: S, out: &mut [S]) -> GeopResult<()> {
        let p = self.degree;
        let n = self.control_points.len();
        let inverse = |w: S, span: usize| -> GeopResult<S> {
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "NurbCurve::evaluate(t={t:?}, span={span}): degree={}, knot_vector={:?}, control_points={:?}",
                    self.degree, self.knot_vector, self.control_points
                ))
            };
            if w.could_be_equal(S::ZERO) {
                return Err(ctx(GeopError::new(format!(
                    "weight is zero at evaluation point (homogeneous de_boor result w={w:?})"
                ))));
            }
            S::ONE.div(w).with_context(&ctx)
        };
        if let Some(first) = pinned_clamped_end(t, &self.knot_vector, n, p) {
            let end = self.control_points[if first { 0 } else { n - 1 }];
            let inv_w = inverse(end[D - 1], if first { p } else { n - 1 })?;
            for (c, o) in out.iter_mut().enumerate() {
                *o = end[c].mul(inv_w);
            }
            return Ok(());
        }
        let mut reached = false;
        for (span, piece) in find_spans(p, &self.knot_vector, n - 1, t)? {
            let local = &self.control_points[span - p..=span];
            let knots = &self.knot_vector[span - p..];
            let absolute = de_boor(p, knots, local, piece, p);
            let (moved, origin) = centered(local);
            let relative = de_boor(p, knots, &moved, piece, p);
            // The weights are not moved: one `W` for both.
            let inv_w = inverse(absolute[D - 1], span)?;
            for (c, o) in out.iter_mut().enumerate() {
                let near = origin[c].add(relative[c].mul(inv_w));
                let value = absolute[c].mul(inv_w).intersect(near);
                *o = if reached { o.union(value) } else { value };
            }
            reached = true;
        }
        if !reached {
            return Err(GeopError::new(format!(
                "NurbCurve::evaluate(t={t:?}): no knot span"
            )));
        }
        Ok(())
    }
}

// ── 3-D curve: evaluate returns Vector3 ─────────────────────────────────────

impl<S: Scalar> NurbCurve<S, 4> {
    /// Evaluate the 3-D NURBS curve at `t`, returning a Cartesian `Vector3`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector3<S>> {
        let mut out = [S::ZERO; 3];
        self.evaluate_into(t, &mut out)?;
        Ok(Vector3::from_array(out))
    }
}

// ── 2-D curve (pcurve): evaluate returns Vector2 ─────────────────────────────

impl<S: Scalar> NurbCurve<S, 3> {
    /// Evaluate the 2-D pcurve at `t`, returning a Cartesian `Vector2`.
    pub fn evaluate(&self, t: S) -> GeopResult<Vector2<S>> {
        let mut out = [S::ZERO; 2];
        self.evaluate_into(t, &mut out)?;
        Ok(Vector2::from_array(out))
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

    /// A quarter circle of radius 3 in the plane `z = 50`, centred at
    /// `(1000, -500)`: the kernel's rational arc, far from the origin.
    fn far_arc<S: Scalar>() -> NurbCurve<S, 4> {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let (cx, cy, z) = (1000.0, -500.0, 50.0);
        NurbCurve::try_new(
            2,
            [(3.0, 0.0, 1.0), (3.0, 3.0, h), (0.0, 3.0, 1.0)]
                .iter()
                .map(|&(x, y, w)| pt((cx + x) * w, (cy + y) * w, z * w, w))
                .collect(),
            [0., 0., 0., 1., 1., 1.].map(S::from_f64).to_vec(),
        )
        .unwrap()
    }

    /// Evaluated over a short parameter box, the arc far from the origin is
    /// as wide as the stretch of it the box covers (about 0.05 long), and as
    /// flat as the plane it lies in. Evaluated as `A / W` in absolute
    /// coordinates its width grew with its distance from the origin instead.
    fn check_an_arc_far_from_the_origin_is_as_wide_as_its_stretch<S: Scalar>() {
        let f = S::from_f64;
        let c = far_arc::<S>();
        let t = f(0.33).union(f(0.34));
        let p = c.evaluate(t).unwrap();
        for k in 0..2 {
            assert!(p[k].width().to_f64() < 0.2, "{p:?}");
        }
        assert!(p[2].could_be_equal(f(50.0)), "{p:?}");
        assert!(p[2].width().to_f64() < 1e-6, "{p:?}");
        // Its tangent too lies in the plane, and is as wide as it turns.
        let d = c.tangent(t).unwrap();
        assert!(d[2].could_be_equal(S::ZERO), "{d:?}");
        assert!(d[2].width().to_f64() < 1e-6, "{d:?}");
        assert!(d[0].width().to_f64() < 1.0, "{d:?}");
    }
    #[test]
    fn an_arc_far_from_the_origin_is_as_wide_as_its_stretch() {
        for_all_scalars!(check_an_arc_far_from_the_origin_is_as_wide_as_its_stretch);
    }

    /// The same for a pcurve: a quarter circle in `(u, v)` around
    /// `(1000, -500)`, as on a surface parameterized in millimetres.
    fn check_a_pcurve_far_from_the_origin_is_as_wide_as_its_stretch<S: Scalar>() {
        let f = S::from_f64;
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let c = NurbCurve::<S, 3>::try_new(
            2,
            [(3.0, 0.0, 1.0), (3.0, 3.0, h), (0.0, 3.0, 1.0)]
                .iter()
                .map(|&(x, y, w)| {
                    geop_core_math::vector::Vector3::from_array([
                        f((1000.0 + x) * w),
                        f((-500.0 + y) * w),
                        f(w),
                    ])
                })
                .collect(),
            [0., 0., 0., 1., 1., 1.].map(f).to_vec(),
        )
        .unwrap();
        let t = f(0.33).union(f(0.34));
        let p = c.evaluate(t).unwrap();
        for k in 0..2 {
            assert!(p[k].width().to_f64() < 0.2, "{p:?}");
        }
        let d = c.tangent(t).unwrap();
        assert!(d[0].width().to_f64() < 1.0, "{d:?}");
    }
    #[test]
    fn a_pcurve_far_from_the_origin_is_as_wide_as_its_stretch() {
        for_all_scalars!(check_a_pcurve_far_from_the_origin_is_as_wide_as_its_stretch);
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
