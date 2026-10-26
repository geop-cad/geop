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
    /// A wide `t` gives every piece the curve could be cut into there, in
    /// one: its width flows into `alpha = (t - e) / (s - e)`, into the
    /// pieces' control points and into their end knot. A piece cut again
    /// is then found wider at the next cut, and its pieces wider still.
    /// To end a piece at a point, cut at a sharp parameter of the refined
    /// enclosure of where the curve reaches it
    /// ([`NurbCurve::split_parameter_at_point`]): which one is a free
    /// choice, and the pieces then carry no more than the rounding of
    /// the cut.
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

    /// A quarter of a disc's rim cut again and again, as slots cut into a
    /// disc one after another cut it: each cut where a plane crosses the
    /// piece left by the cut before, at the vertex the piece gives there,
    /// as `Model::split_edge_at_vertex` cuts. Cut once, the piece is
    /// `2e-15` wide; each further cut adds one subdivision's rounding and
    /// nothing more. Cut at the whole refined parameter, the width grew
    /// threefold per cut, to `0.08` by the 27th, and the 28th failed.
    #[test]
    fn a_rim_cut_many_times_stays_as_narrow_as_cut_once() {
        use crate::contains::curve::curve_could_contain;
        use geop_core_math::vector::Vector3;
        let f = Scalar::from_f64;
        let v = |x: f64, y: f64, z: f64| Vector3::from_array([f(x), f(y), f(z)]);
        let mut piece = NurbCurve::<4>::arc(
            v(0.0, -4.0, 1.0),
            v(4.0, -4.0, 1.0),
            v(4.0, 0.0, 1.0),
            crate::nurb_curve::sqrt2_over_2(),
        )
        .unwrap();
        for k in 1..=40 {
            let angle = -std::f64::consts::FRAC_PI_2 * (1.0 - k as f64 / 41.0);
            let crossing = v(4.0 * angle.cos(), 4.0 * angle.sin(), 1.0);
            let t = curve_could_contain(&piece, &crossing, 10_000, f(1e-4))
                .unwrap()
                .unwrap();
            let t = piece.refine_parameter_at_point(t, &crossing);
            let vertex = piece.evaluate(t).unwrap();
            let t = piece.split_parameter_at_point(t, &vertex).unwrap();
            piece = piece.split(t).unwrap().1;
            let widest = piece
                .control_points
                .iter()
                .flat_map(|p| (0..4).map(move |c| p[c].width().to_f64()))
                .fold(0.0, f64::max);
            assert!(widest < 1e-13, "cut {k}: {widest:e}");
            let z = piece.evaluate(piece.mid_parameter().unwrap()).unwrap()[2];
            assert!(
                z.could_be_equal(Scalar::ONE) && z.width().to_f64() < 1e-13,
                "cut {k}: {z:?}"
            );
        }
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
