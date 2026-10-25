use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

use super::NurbCurve;
use crate::{aabb::compute_aabb, knot_insertion};

impl<const D: usize> NurbCurve<D> {
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
    pub fn split(&self, t: Scalar) -> GeopResult<(NurbCurve<D>, NurbCurve<D>)> {
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
    pub fn sub_curve(&self, t0: Scalar, t1: Scalar) -> GeopResult<NurbCurve<D>> {
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

    /// The curve cut at every interior knot: its polynomial (or rational)
    /// pieces, in order.
    pub fn bezier_pieces(&self) -> GeopResult<Vec<Self>> {
        let end = self.domain().1;
        let interior = &self.knot_vector[self.degree + 1..self.control_points.len()];
        let mut rest = self.clone();
        let mut pieces = Vec::new();
        for &k in interior {
            if k.definitely_greater(rest.domain().0) && k.definitely_less(end) {
                let (left, right) = rest.split(k)?;
                pieces.push(left);
                rest = right;
            }
        }
        pieces.push(rest);
        Ok(pieces)
    }

    /// Split at the midpoint of the parameter domain.
    pub fn split_mid(&self) -> GeopResult<(NurbCurve<D>, NurbCurve<D>)> {
        // A self-chosen subdivision point, sharp, so repeated splits do not
        // compound width.
        self.split(self.mid_parameter()?)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
    use crate::test_fixtures::{hom, line};
    use geop_core_math::scalars::Scalar;

    #[test]
    fn split_line_halves_domain() {
        let (left, right) = line([0.; 3], [1., 0., 0.])
            .split(Scalar::from_f64(0.5))
            .unwrap();

        let l0 = left.evaluate(Scalar::ZERO).unwrap();
        assert!(l0[0].could_be_equal(Scalar::ZERO));

        let l1 = left.evaluate(Scalar::from_f64(0.5)).unwrap();
        let r0 = right.evaluate(Scalar::from_f64(0.5)).unwrap();
        assert!(l1[0].could_be_equal(r0[0]));
        assert!(l1[0].could_be_equal(Scalar::from_f64(0.5)));

        let r1 = right.evaluate(Scalar::ONE).unwrap();
        assert!(r1[0].could_be_equal(Scalar::ONE));
    }

    #[test]
    fn split_preserves_points_on_curve() {
        let curve = line([0.; 3], [1., 0., 0.]);
        let (left, right) = curve.split(Scalar::from_f64(0.25)).unwrap();

        let orig = curve.evaluate(Scalar::from_f64(0.1)).unwrap();
        let from_left = left.evaluate(Scalar::from_f64(0.1)).unwrap();
        assert!(orig[0].could_be_equal(from_left[0]));

        let orig2 = curve.evaluate(Scalar::from_f64(0.75)).unwrap();
        let from_right = right.evaluate(Scalar::from_f64(0.75)).unwrap();
        assert!(orig2[0].could_be_equal(from_right[0]));
    }

    #[test]
    fn split_at_boundary_returns_err() {
        let c = line([0.; 3], [1., 0., 0.]);
        assert!(c.split(Scalar::ZERO).is_err());
        assert!(c.split(Scalar::ONE).is_err());
    }

    #[test]
    fn split_cubic_at_existing_knot() {
        let f = Scalar::from_f64;
        let curve = NurbCurve::try_new(
            3,
            vec![
                hom(0.0, 0.0, 0.0, 1.0),
                hom(0.25, 1.0, 0.0, 1.0),
                hom(0.5, 0.0, 0.0, 1.0),
                hom(0.75, 1.0, 0.0, 1.0),
                hom(1.0, 0.0, 0.0, 1.0),
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
}
