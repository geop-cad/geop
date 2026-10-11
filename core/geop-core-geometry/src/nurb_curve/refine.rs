use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

use super::NurbCurve;
use geop_core_math::solvers::newton;

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Refine `t` — a parameter enclosure produced by a subdivision search —
    /// into the tightest enclosure of the parameter at which this curve
    /// passes through `point`: Gauss-Newton on `|C(t) - P|²`, by
    /// [`newton::refine`], which says why and how honestly.
    ///
    /// This is what lets [`NurbCurve::split`] be called without sharpening:
    /// sharpening a search's parameter moved the split to the interval's
    /// midpoint rather than the point actually located — a silent geometric
    /// error of `|t_mid - t*| x |C'(t)|`, and the reason an edge endpoint
    /// could land ~1e-8 from the vertex it is anchored to. A refined
    /// parameter is narrow *and* still an honest enclosure.
    ///
    /// A point nowhere near the curve, or a vanishing tangent, leaves `t`
    /// as it is.
    pub fn refine_parameter_at_point<const C: usize>(&self, t: S, point: &Vector<S, C>) -> S
    where
        NurbCurve<S, D>: ParameterRefinable<S, C>,
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
}

/// Bridges a `NurbCurve<S, D>`'s Cartesian evaluation (`D = 4` -> 3-D points,
/// `D = 3` -> 2-D pcurve points) so [`NurbCurve::refine_parameter_at_point`]
/// can be written once for both — stable Rust's const generics cannot express
/// `C = D - 1` directly.
pub trait ParameterRefinable<S: Scalar, const C: usize> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, C>>;
    /// The Cartesian tangent, via each dimension's own `tangent` (the
    /// quotient rule over the homogeneous `derivative()`).
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, C>>;
}

impl<S: Scalar> ParameterRefinable<S, 3> for NurbCurve<S, 4> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, 3>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, 3>> {
        self.tangent(t)
    }
}

impl<S: Scalar> ParameterRefinable<S, 2> for NurbCurve<S, 3> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, 2>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, 2>> {
        self.tangent(t)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64) -> Vector4<S> {
        Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
    }

    /// Degree-2 arc, so the parametrization is genuinely nonlinear.
    fn arc<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![pt(0.0, 0.0, 0.0), pt(1.0, 2.0, 0.0), pt(2.0, 0.0, 0.0)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A search-width parameter refines to a far tighter one that still
    /// encloses the true parameter, and still lands on the same point.
    fn check_refines_a_wide_parameter<S: Scalar>() {
        let curve = arc::<S>();
        let exact = S::from_f64(0.375);
        let target = curve.evaluate(exact).unwrap();

        // What a 1e-4 subdivision search would hand back.
        let wide = S::from_f64(0.3745).union(S::from_f64(0.3755));
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
    #[test]
    fn refines_a_wide_parameter() {
        for_all_scalars!(check_refines_a_wide_parameter);
    }

    /// A point nowhere near the curve must not drag the parameter somewhere
    /// arbitrary — the incoming enclosure comes back untouched.
    fn check_off_curve_point_falls_back<S: Scalar>() {
        let curve = arc::<S>();
        let target = geop_core_math::vector::Vector3::from_array([
            S::from_f64(50.0),
            S::from_f64(50.0),
            S::from_f64(50.0),
        ]);
        let wide = S::from_f64(0.3745).union(S::from_f64(0.3755));
        let refined = curve.refine_parameter_at_point(wide, &target);
        assert!(refined.could_be_equal(wide));
    }
    #[test]
    fn off_curve_point_falls_back() {
        for_all_scalars!(check_off_curve_point_falls_back);
    }
}
