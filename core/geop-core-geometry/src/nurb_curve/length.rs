//! [`NurbCurve::length`]: how long a 3-D curve is.

use geop_core_math::{
    geop_error::GeopResult,
    quadrature::{Quadrature, integrate},
    scalars::Scalar,
};

use super::NurbCurve;

impl<S: Scalar> NurbCurve<S, 4> {
    /// The curve's length, `∫ |C'(t)| dt` over its domain, by adaptive
    /// Gauss–Kronrod from knot to knot — an enclosure in the sense of
    /// [`geop_core_math::quadrature`]: the rule's value enclosed, widened by
    /// its estimated truncation error — and whether that estimate came
    /// within the tolerance.
    pub fn length(&self) -> GeopResult<(S, bool)> {
        let speed = |t: S| Ok(vec![self.tangent(t)?.norm()]);
        let integral = integrate(speed, &self.breakpoints(), 1, &Quadrature::default())?;
        Ok((integral.value[0], integral.converged))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        scalars::{Ring, ScalInF64 as S, Scalar},
        vector::Vector4,
    };

    use crate::nurb_curve::NurbCurve;

    /// A quarter circle of radius 2 is pi long, enclosed.
    #[test]
    fn a_quarter_circle_is_pi_long() {
        let w = S::from_f64(std::f64::consts::FRAC_1_SQRT_2);
        let p = |x: f64, y: f64, w: S| {
            Vector4::from_array([S::from_f64(x).mul(w), S::from_f64(y).mul(w), S::ZERO, w])
        };
        let arc = NurbCurve::try_new(
            2,
            vec![p(2.0, 0.0, S::ONE), p(2.0, 2.0, w), p(0.0, 2.0, S::ONE)],
            vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
        )
        .unwrap();
        let (length, converged) = arc.length().unwrap();
        assert!(converged);
        assert!(length.could_be_equal(S::PI), "{length:?}");
        assert!(length.width().to_f64() < 1e-8, "{length:?}");
    }
}
