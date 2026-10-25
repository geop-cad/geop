use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::NurbCurve;
use crate::{
    knot_insertion::pinned_clamped_end,
    spline::{centered, de_boor, find_spans, nearer_end},
};

impl<const D: usize> NurbCurve<D> {
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
    pub(super) fn evaluate_into(&self, t: Scalar, out: &mut [Scalar]) -> GeopResult<()> {
        let p = self.degree;
        let n = self.control_points.len();
        let inverse = |w: Scalar, span: usize| -> GeopResult<Scalar> {
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "NurbCurve::evaluate(t={t:?}, span={span}): degree={}, knot_vector={:?}, control_points={:?}",
                    self.degree, self.knot_vector, self.control_points
                ))
            };
            if w.could_be_equal(Scalar::ZERO) {
                return Err(ctx(GeopError::new(format!(
                    "weight is zero at evaluation point (homogeneous de_boor result w={w:?})"
                ))));
            }
            Scalar::ONE.div(w).with_context(&ctx)
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
            let near = if nearer_end(&self.knot_vector, span, piece) { p } else { 0 };
            let (moved, origin) = centered(local, near);
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

impl NurbCurve<4> {
    /// Evaluate the 3-D NURBS curve at `t`, returning a Cartesian `Vector3`.
    pub fn evaluate(&self, t: Scalar) -> GeopResult<Vector3> {
        let mut out = [Scalar::ZERO; 3];
        self.evaluate_into(t, &mut out)?;
        Ok(Vector3::from_array(out))
    }

    /// Where it starts and where it ends.
    pub fn ends(&self) -> GeopResult<(Vector3, Vector3)> {
        let (t0, t1) = self.domain();
        Ok((self.evaluate(t0)?, self.evaluate(t1)?))
    }
}

// ── 2-D curve (pcurve): evaluate returns Vector2 ─────────────────────────────

impl NurbCurve<3> {
    /// Evaluate the 2-D pcurve at `t`, returning a Cartesian `Vector2`.
    pub fn evaluate(&self, t: Scalar) -> GeopResult<Vector2> {
        let mut out = [Scalar::ZERO; 2];
        self.evaluate_into(t, &mut out)?;
        Ok(Vector2::from_array(out))
    }

    /// Where it starts and where it ends.
    pub fn ends(&self) -> GeopResult<(Vector2, Vector2)> {
        let (t0, t1) = self.domain();
        Ok((self.evaluate(t0)?, self.evaluate(t1)?))
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
    use crate::test_fixtures::hom;
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    fn line_curve() -> NurbCurve<4> {
        NurbCurve::try_new(
            1,
            vec![hom(0.0, 0.0, 0.0, 1.0), hom(1.0, 0.0, 0.0, 1.0)],
            vec![
                Scalar::from_f64(0.0),
                Scalar::from_f64(0.0),
                Scalar::from_f64(1.0),
                Scalar::from_f64(1.0),
            ],
        )
        .unwrap()
    }

    #[test]
    fn line_at_start() {
        let c = line_curve();
        let p = c.evaluate(Scalar::ZERO).unwrap();
        assert!(p[0].could_be_equal(Scalar::ZERO));
        assert!(p[1].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn line_at_end() {
        let c = line_curve();
        let p = c.evaluate(Scalar::ONE).unwrap();
        assert!(p[0].could_be_equal(Scalar::ONE));
        assert!(p[1].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn line_at_midpoint() {
        let c = line_curve();
        let p = c.evaluate(Scalar::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(0.5)));
    }

    #[test]
    fn out_of_domain_returns_err() {
        let c = line_curve();
        assert!(c.evaluate(Scalar::from_f64(-0.1)).is_err());
        assert!(c.evaluate(Scalar::from_f64(1.1)).is_err());
    }

    #[test]
    fn quadratic_midpoint() {
        let curve = NurbCurve::try_new(
            2,
            vec![
                hom(0.0, 0.0, 0.0, 1.0),
                hom(0.5, 0.0, 0.0, 1.0),
                hom(1.0, 0.0, 0.0, 1.0),
            ],
            vec![
                Scalar::from_f64(0.0),
                Scalar::from_f64(0.0),
                Scalar::from_f64(0.0),
                Scalar::from_f64(1.0),
                Scalar::from_f64(1.0),
                Scalar::from_f64(1.0),
            ],
        )
        .unwrap();
        let p = curve.evaluate(Scalar::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(0.5)));
        assert!(p[1].could_be_equal(Scalar::ZERO));
    }

    /// Regression check for a `weight is zero at evaluation point` failure
    /// observed from `remesh` on a degree-1, weight-1-constant, [0,0,1,1]
    /// curve — data that on paper cannot produce a near-zero interpolated
    /// weight (both endpoint weights are exactly 1). Reproduces the exact
    /// control points/knots/`t` from that failure's error context to check
    /// whether `evaluate` itself is at fault, independent of `remesh`.
    #[test]
    fn weight_one_constant_line_does_not_report_zero_weight() {
        let curve = NurbCurve::try_new(
            1,
            vec![hom(0.5, 0.5, 0.5, 1.0), hom(0.5, 0.5, -0.5, 1.0)],
            vec![
                Scalar::from_f64(0.0),
                Scalar::from_f64(0.0),
                Scalar::from_f64(1.0),
                Scalar::from_f64(1.0),
            ],
        )
        .unwrap();
        let p = curve.evaluate(Scalar::from_f64(0.972)).unwrap();
        assert!(
            p[2].could_be_equal(Scalar::from_f64(0.5 - 1.0 * 0.972)),
            "p={p:?}"
        );
    }

    /// A quarter circle of radius 3 in the plane `z = 50`, centred at
    /// `(1000, -500)`: the kernel's rational arc, far from the origin.
    fn far_arc() -> NurbCurve<4> {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let (cx, cy, z) = (1000.0, -500.0, 50.0);
        NurbCurve::try_new(
            2,
            [(3.0, 0.0, 1.0), (3.0, 3.0, h), (0.0, 3.0, 1.0)]
                .iter()
                .map(|&(x, y, w)| hom((cx + x) * w, (cy + y) * w, z * w, w))
                .collect(),
            [0., 0., 0., 1., 1., 1.].map(Scalar::from_f64).to_vec(),
        )
        .unwrap()
    }

    /// Evaluated over a short parameter box, the arc far from the origin is
    /// as wide as the stretch of it the box covers (about 0.05 long), and as
    /// flat as the plane it lies in. Evaluated as `A / W` in absolute
    /// coordinates its width grew with its distance from the origin instead.
    #[test]
    fn an_arc_far_from_the_origin_is_as_wide_as_its_stretch() {
        let f = Scalar::from_f64;
        let c = far_arc();
        let t = f(0.33).union(f(0.34));
        let p = c.evaluate(t).unwrap();
        for k in 0..2 {
            assert!(p[k].width().to_f64() < 0.2, "{p:?}");
        }
        assert!(p[2].could_be_equal(f(50.0)), "{p:?}");
        assert!(p[2].width().to_f64() < 1e-6, "{p:?}");
        // Its tangent too lies in the plane, and is as wide as it turns.
        let d = c.tangent(t).unwrap();
        assert!(d[2].could_be_equal(Scalar::ZERO), "{d:?}");
        assert!(d[2].width().to_f64() < 1e-6, "{d:?}");
        assert!(d[0].width().to_f64() < 1.0, "{d:?}");
    }

    /// The same for a pcurve: a quarter circle in `(u, v)` around
    /// `(1000, -500)`, as on a surface parameterized in millimetres.
    #[test]
    fn a_pcurve_far_from_the_origin_is_as_wide_as_its_stretch() {
        let f = Scalar::from_f64;
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let c = NurbCurve::<3>::try_new(
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
    fn everything_matches_any_point() {
        let c = NurbCurve::<4>::everything();
        let p = c.evaluate(Scalar::from_f64(7.0)).unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(123.456)));
        assert!(p[1].could_be_equal(Scalar::from_f64(-9.0)));
        assert!(p[2].could_be_equal(Scalar::ZERO));
    }

    /// Over an interval reaching across many knot spans, a curve is the union
    /// of its spans' parts, not one span's cubic extrapolated over all of
    /// them: a quarter circle of radius 2.25 interpolated through 49 points
    /// came out 57 wide over its whole domain, and over an eighth of it
    /// eight times as wide as that eighth.
    #[test]
    fn a_wide_parameter_encloses_the_curve_tightly() {
        let r = 2.25;
        let points: Vec<_> = (0..=48)
            .map(|k| {
                let a = std::f64::consts::FRAC_PI_2 * k as f64 / 48.0;
                Vector3::from_array([r * a.cos(), r * a.sin(), 0.0].map(Scalar::from_f64))
            })
            .collect();
        let curve = NurbCurve::<4>::interpolate(&points, 3).unwrap();
        let (t0, t1) = curve.domain();
        for (lo, hi) in [
            (t0, t1),
            (t0, Scalar::from_f64(0.125)),
            (Scalar::from_f64(0.3), Scalar::from_f64(0.55)),
        ] {
            let box_ = curve.evaluate(lo.union(hi)).unwrap();
            for k in 0..=64 {
                let t = match k {
                    0 => lo,
                    64 => hi,
                    _ => Scalar::interpolate(lo, hi, Scalar::from_ratio(k, 64).unwrap()).sharpen(),
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
}
