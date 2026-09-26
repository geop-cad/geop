use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

use super::NurbCurve;
use crate::{aabb::compute_aabb, knot_insertion};

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Split at parameter `t` (must be strictly inside the domain).
    /// Returns `(left, right)` sharing the junction value.
    ///
    /// `t` is used exactly as given — it is **not** sharpened here. Whether
    /// `t`'s width may be discarded belongs to whoever produced it, since
    /// only they know what it means, so this function neither assumes nor
    /// imposes an answer.
    ///
    /// What `t` must be is *narrow*, not sharp. Boehm insertion cannot absorb
    /// a wide parameter: its width flows into `alpha = (t - e) / (s - e)` — `s - e` shrinks with every successive
    /// split while an unsharpened width does not, so their ratio widens
    /// without bound — and on into the sub-curves' control points, until
    /// downstream subdivision searches stop converging.
    ///
    /// Callers used to meet that by sharpening, which was a real geometric
    /// error: it moved the cut to the interval's midpoint rather than the
    /// point actually located, off by `|t_mid - t*| x |C'(t)|`, which is how
    /// an edge endpoint ended up ~1e-8 from the vertex it was anchored to.
    /// They now Newton-refine instead — see
    /// [`NurbCurve::refine_parameter_at_point`] and
    /// `intersection::curve_surface::refine_crossing` — which yields a
    /// parameter that is narrow *and* still an honest enclosure. Subdivision
    /// isolates the solution, Newton polishes it; neither does the other's
    /// job. No caller on the split path sharpens any more.
    ///
    /// See "Sharpen only where the value is a free choice" in `AGENTS.md`.
    pub fn split(&self, t: S) -> GeopResult<(NurbCurve<S, D>, NurbCurve<S, D>)> {
        let p = self.degree;
        let mut knots = self.knot_vector.clone();
        let mut rows = [self.control_points.clone()];
        let (right_knots, right_rows) = knot_insertion::split(&mut knots, &mut rows, p, t)?;
        let [pts] = rows;
        let right_pts = right_rows.into_iter().next().unwrap_or_default();
        Ok((
            NurbCurve {
                aabb: compute_aabb(&pts),
                degree: p,
                control_points: pts,
                knot_vector: knots,
            },
            NurbCurve {
                aabb: compute_aabb(&right_pts),
                degree: p,
                control_points: right_pts,
                knot_vector: right_knots,
            },
        ))
    }

    /// This curve restricted to `[t0, t1]`, cut in one pass: each bound that
    /// lies strictly inside the domain (`definitely_greater` the start /
    /// `definitely_less` the end) is inserted to full multiplicity and the
    /// outside is dropped; a bound that doesn't is not cut at, so the result
    /// always covers at least `[t0, t1] ∩ domain`. Cheaper than two
    /// [`Self::split`]s: no discarded piece or intermediate curve is built.
    ///
    /// Like `split`, the bounds are used exactly as given and must be narrow.
    pub fn sub_curve(&self, t0: S, t1: S) -> GeopResult<NurbCurve<S, D>> {
        let p = self.degree;
        let mut knots = self.knot_vector.clone();
        let mut rows = [self.control_points.clone()];
        knot_insertion::restrict(&mut knots, &mut rows, p, t0, t1)?;
        let [pts] = rows;
        Ok(NurbCurve {
            aabb: compute_aabb(&pts),
            degree: p,
            control_points: pts,
            knot_vector: knots,
        })
    }

    /// Split at the midpoint of the parameter domain.
    pub fn split_mid(&self) -> GeopResult<(NurbCurve<S, D>, NurbCurve<S, D>)> {
        let (t0, t1) = self.domain();
        // A self-chosen subdivision point: any value in the interval cuts
        // it equally well, so sharpening loses no accuracy and keeps
        // repeated splits from compounding width (see AGENTS.md).
        let mid = t0.add(t1).div(S::TWO)?.sharpen();
        self.split(mid)
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

    fn line<S: Scalar>() -> NurbCurve<S, 4> {
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

    fn check_split_line_halves_domain<S: Scalar>() {
        let (left, right) = line::<S>().split(S::from_f64(0.5)).unwrap();

        let l0 = left.evaluate(S::ZERO).unwrap();
        assert!(l0[0].could_be_equal(S::ZERO));

        let l1 = left.evaluate(S::from_f64(0.5)).unwrap();
        let r0 = right.evaluate(S::from_f64(0.5)).unwrap();
        assert!(l1[0].could_be_equal(r0[0]));
        assert!(l1[0].could_be_equal(S::from_f64(0.5)));

        let r1 = right.evaluate(S::ONE).unwrap();
        assert!(r1[0].could_be_equal(S::ONE));
    }
    #[test]
    fn split_line_halves_domain() {
        for_all_scalars!(check_split_line_halves_domain);
    }

    fn check_split_preserves_points_on_curve<S: Scalar>() {
        let curve = line::<S>();
        let (left, right) = curve.split(S::from_f64(0.25)).unwrap();

        let orig = curve.evaluate(S::from_f64(0.1)).unwrap();
        let from_left = left.evaluate(S::from_f64(0.1)).unwrap();
        assert!(orig[0].could_be_equal(from_left[0]));

        let orig2 = curve.evaluate(S::from_f64(0.75)).unwrap();
        let from_right = right.evaluate(S::from_f64(0.75)).unwrap();
        assert!(orig2[0].could_be_equal(from_right[0]));
    }
    #[test]
    fn split_preserves_points_on_curve() {
        for_all_scalars!(check_split_preserves_points_on_curve);
    }

    fn check_split_at_boundary_returns_err<S: Scalar>() {
        let c = line::<S>();
        assert!(c.split(S::ZERO).is_err());
        assert!(c.split(S::ONE).is_err());
    }
    #[test]
    fn split_at_boundary_returns_err() {
        for_all_scalars!(check_split_at_boundary_returns_err);
    }

    fn check_split_cubic_at_existing_knot<S: Scalar>() {
        let f = S::from_f64;
        let curve = NurbCurve::try_new(
            3,
            vec![
                pt(0.0, 0.0, 0.0, 1.0),
                pt(0.25, 1.0, 0.0, 1.0),
                pt(0.5, 0.0, 0.0, 1.0),
                pt(0.75, 1.0, 0.0, 1.0),
                pt(1.0, 0.0, 0.0, 1.0),
            ],
            vec![
                f(0.0),
                f(0.0),
                f(0.0),
                f(0.0),
                f(0.5),
                f(1.0),
                f(1.0),
                f(1.0),
                f(1.0),
            ],
        )
        .unwrap();

        let t_split = f(0.5);
        let (left, right) = curve.split(t_split).unwrap();

        let orig_at_split = curve.evaluate(t_split).unwrap();
        let left_at_split = left.evaluate(t_split).unwrap();
        let right_at_split = right.evaluate(t_split).unwrap();

        for c in 0..3 {
            assert!(
                orig_at_split[c].could_be_equal(left_at_split[c]),
                "left junction mismatch at coord {c}"
            );
            assert!(
                orig_at_split[c].could_be_equal(right_at_split[c]),
                "right junction mismatch at coord {c}"
            );
        }
    }
    #[test]
    fn split_cubic_at_existing_knot() {
        for_all_scalars!(check_split_cubic_at_existing_knot);
    }
}
