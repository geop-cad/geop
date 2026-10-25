//! [`NurbCurve::length`]: how long a curve is, in space or in a plane.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    quadrature::{Quadrature, integrate},
    scalars::Scalar,
};

use super::NurbCurve;

impl<const D: usize> NurbCurve<D> {
    /// `∫ speed(t) dt` over the curve's domain, by adaptive Gauss–Kronrod
    /// from knot to knot — an enclosure in the sense of
    /// [`geop_core_math::quadrature`]: the rule's value enclosed, widened by
    /// its estimated truncation error — and whether that estimate came
    /// within the tolerance.
    fn integrated(
        &self,
        mut speed: impl FnMut(Scalar) -> GeopResult<Scalar>,
    ) -> GeopResult<(Scalar, bool)> {
        let integral = integrate(
            |t| Ok(vec![speed(t)?]),
            &self.breakpoints(),
            1,
            &Quadrature::default(),
        )?;
        Ok((integral.value[0], integral.converged))
    }
}

impl NurbCurve<4> {
    /// The curve's length, `∫ |C'(t)| dt` over its domain, enclosed (see
    /// [`Self::integrated`]), and whether the quadrature converged.
    pub fn length(&self) -> GeopResult<(Scalar, bool)> {
        self.integrated(|t| Ok(self.tangent(t)?.norm()))
    }
}

impl NurbCurve<3> {
    /// The planar curve's length, `∫ |C'(t)| dt` over its domain, enclosed
    /// (see [`Self::integrated`]), and whether the quadrature converged.
    pub fn length(&self) -> GeopResult<(Scalar, bool)> {
        self.integrated(|t| Ok(self.tangent(t)?.norm()))
    }

    /// The parameter `t` at which the fraction `fraction` of the curve's
    /// length, from its start, has been run, for `fraction` in `(0, 1)`:
    /// a sharp value, for a caller choosing where to cut the curve, found
    /// by bisection on the length up to `t`, which grows with `t`. An
    /// error for a fraction outside `(0, 1)`.
    pub fn parameter_at_length_fraction(&self, fraction: f64) -> GeopResult<Scalar> {
        if !(0.0 < fraction && fraction < 1.0) {
            return Err(GeopError::new(format!(
                "parameter_at_length_fraction: {fraction} is not inside (0, 1)"
            )));
        }
        let (t0, t1) = self.domain();
        let target = fraction * self.length()?.0.to_f64();
        let (mut lo, mut hi) = (t0.to_f64(), t1.to_f64());
        // Halving the bracket to the last bit of an `f64`.
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            let run = self
                .sub_curve(t0, Scalar::from_f64(mid))?
                .length()?
                .0
                .to_f64();
            if run < target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Ok(Scalar::from_f64(0.5 * (lo + hi)))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    use crate::nurb_curve::NurbCurve;

    /// A quarter circle of radius 2 is pi long, enclosed.
    #[test]
    fn a_quarter_circle_is_pi_long() {
        let w = Scalar::from_f64(std::f64::consts::FRAC_1_SQRT_2);
        let p = |x: f64, y: f64, w: Scalar| {
            Vector4::from_array([
                Scalar::from_f64(x).mul(w),
                Scalar::from_f64(y).mul(w),
                Scalar::ZERO,
                w,
            ])
        };
        let arc = NurbCurve::try_new(
            2,
            vec![
                p(2.0, 0.0, Scalar::ONE),
                p(2.0, 2.0, w),
                p(0.0, 2.0, Scalar::ONE),
            ],
            vec![
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ONE,
                Scalar::ONE,
                Scalar::ONE,
            ],
        )
        .unwrap();
        let (length, converged) = arc.length().unwrap();
        assert!(converged);
        assert!(length.could_be_equal(Scalar::PI), "{length:?}");
        assert!(length.width().to_f64() < 1e-8, "{length:?}");
    }
}
