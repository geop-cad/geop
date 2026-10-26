use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

use super::NurbCurve;
use geop_core_math::solvers::newton;

impl<const D: usize> NurbCurve<D> {
    /// Refine `t` — a parameter enclosure produced by a subdivision search —
    /// into the tightest enclosure of the parameter at which this curve
    /// passes through `point`: Gauss-Newton on `|C(t) - P|²`, by
    /// [`newton::refine`], which says why and how honestly.
    ///
    /// A search's parameter is as wide as the search's tolerance; the
    /// refined one is as narrow as the curve and the point pin it down, and
    /// still an honest enclosure. [`Self::split_parameter_at_point`] cuts
    /// within it.
    ///
    /// A point nowhere near the curve, or a vanishing tangent, leaves `t`
    /// as it is.
    pub fn refine_parameter_at_point<const C: usize>(&self, t: Scalar, point: &Vector<C>) -> Scalar
    where
        NurbCurve<D>: ParameterRefinable<C>,
    {
        let [t] = newton::refine([t], [self.domain()], |[current]| {
            let position = self.evaluate_cartesian(current).ok()?;
            let tangent = self.tangent_cartesian(current).ok()?;
            // The step that zeroes the directional residual along the
            // tangent.
            let residual = position.sub(point);
            let numerator = residual.prod_dot(&tangent);
            let denominator = tangent.prod_dot(&tangent);
            let step = numerator.div(denominator).ok()?;
            Some([current.sub(step)])
        });
        t
    }

    /// The parameter to [`split`](Self::split) this curve at so a piece
    /// ends at `point`, from a parameter `t` that isolates it: `t` refined
    /// ([`Self::refine_parameter_at_point`]), then the sharp middle of it.
    ///
    /// Where in the refined enclosure the curve is cut is a free choice.
    /// `point` is an enclosure, and every parameter in the refined one
    /// gives a piece that lies on the curve and ends within `point`, so
    /// any of them serves; the check below asks it of the one chosen.
    /// Cutting at the whole enclosure instead keeps *every* such piece in
    /// one: the junction's uncertainty along the curve becomes width in the
    /// pieces' control points (through `α = (t - a) / (b - a)` in Boehm
    /// insertion) and in their end knots. The next cut's parameter is then
    /// found on the wider piece, comes out wider, and widens the next
    /// pieces more: a quarter arc of radius 4 cut forty times went from
    /// `3e-15` wide to `0.08` (three times wider per cut), and a disc's rim
    /// cut by thirty-two slots ended `4e-10` off its cap. A sharp cut adds
    /// the rounding of one subdivision per cut, nothing more.
    ///
    /// Refused if `t` could be an end of the domain — then `point` is an
    /// end of the curve, nothing to cut — or if the curve at the chosen
    /// parameter does not reach `point`.
    pub fn split_parameter_at_point<const C: usize>(
        &self,
        t: Scalar,
        point: &Vector<C>,
    ) -> GeopResult<Scalar>
    where
        NurbCurve<D>: ParameterRefinable<C>,
    {
        let (lo, hi) = self.domain();
        let refined = self.refine_parameter_at_point(t, point);
        if !refined.definitely_greater(lo) || !refined.definitely_less(hi) {
            return Err(GeopError::new(format!(
                "the parameter {refined:?} of {point:?} could be an end of the domain ({lo:?}, {hi:?}): the point is an end of the curve, not inside it"
            )));
        }
        let cut = refined.sharpen();
        let reached = self.evaluate_cartesian(cut)?;
        if !reached.could_be_equal(point) {
            return Err(GeopError::new(format!(
                "the curve at {cut:?}, the middle of the refined parameter {refined:?}, is at {reached:?}, not at {point:?}"
            )));
        }
        Ok(cut)
    }
}

/// Bridges a `NurbCurve<D>`'s Cartesian evaluation (`D = 4` -> 3-D points,
/// `D = 3` -> 2-D pcurve points) so [`NurbCurve::refine_parameter_at_point`]
/// can be written once for both — stable Rust's const generics cannot express
/// `C = D - 1` directly.
pub trait ParameterRefinable<const C: usize> {
    fn evaluate_cartesian(&self, t: Scalar) -> GeopResult<Vector<C>>;
    /// The Cartesian tangent, via each dimension's own `tangent` (the
    /// quotient rule over the homogeneous `derivative()`).
    fn tangent_cartesian(&self, t: Scalar) -> GeopResult<Vector<C>>;
}

impl ParameterRefinable<3> for NurbCurve<4> {
    fn evaluate_cartesian(&self, t: Scalar) -> GeopResult<Vector<3>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: Scalar) -> GeopResult<Vector<3>> {
        self.tangent(t)
    }
}

impl ParameterRefinable<2> for NurbCurve<3> {
    fn evaluate_cartesian(&self, t: Scalar) -> GeopResult<Vector<2>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: Scalar) -> GeopResult<Vector<2>> {
        self.tangent(t)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
    use crate::test_fixtures::hom;
    use geop_core_math::scalars::Scalar;

    /// Degree-2 arc, so the parametrization is genuinely nonlinear.
    fn arc() -> NurbCurve<4> {
        let f = Scalar::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                hom(0.0, 0.0, 0.0, 1.),
                hom(1.0, 2.0, 0.0, 1.),
                hom(2.0, 0.0, 0.0, 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A search-width parameter refines to a far tighter one that still
    /// encloses the true parameter, and still lands on the same point.
    #[test]
    fn refines_a_wide_parameter() {
        let curve = arc();
        let exact = Scalar::from_f64(0.375);
        let target = curve.evaluate(exact).unwrap();

        // What a 1e-4 subdivision search would hand back.
        let wide = Scalar::from_f64(0.3745).union(Scalar::from_f64(0.3755));
        let refined = curve.refine_parameter_at_point(wide, &target);

        assert!(
            refined.could_be_equal(exact),
            "refined parameter must still enclose the true one: {refined:?}"
        );
        let landed = curve.evaluate(refined).unwrap();
        for c in 0..3 {
            assert!(landed[c].could_be_equal(target[c]), "coord {c} moved");
        }
    }

    /// A point nowhere near the curve must not drag the parameter somewhere
    /// arbitrary — the incoming enclosure comes back untouched.
    #[test]
    fn off_curve_point_falls_back() {
        let curve = arc();
        let target = geop_core_math::vector::Vector3::from_array([
            Scalar::from_f64(50.0),
            Scalar::from_f64(50.0),
            Scalar::from_f64(50.0),
        ]);
        let wide = Scalar::from_f64(0.3745).union(Scalar::from_f64(0.3755));
        let refined = curve.refine_parameter_at_point(wide, &target);
        assert!(refined.could_be_equal(wide));
    }
}
