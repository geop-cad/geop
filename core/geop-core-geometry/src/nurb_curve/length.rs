//! [`NurbCurve::length`]: how long a curve is, in space or in a plane, and
//! where along it a fraction of that length is run.

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

/// A curve with a length: in space, or in a plane.
pub trait Length: Sized {
    /// Its length, `∫ |C'(t)| dt` over its domain, enclosed (see
    /// `NurbCurve::integrated`), and whether the quadrature converged.
    fn length(&self) -> GeopResult<(Scalar, bool)>;
}

impl Length for NurbCurve<4> {
    fn length(&self) -> GeopResult<(Scalar, bool)> {
        self.integrated(|t| Ok(self.tangent(t)?.norm()))
    }
}

impl Length for NurbCurve<3> {
    fn length(&self) -> GeopResult<(Scalar, bool)> {
        self.integrated(|t| Ok(self.tangent(t)?.norm()))
    }
}

impl<const D: usize> NurbCurve<D>
where
    Self: Length,
{
    /// The curve's length, `∫ |C'(t)| dt` over its domain, enclosed, and
    /// whether the quadrature converged (see [`Length`]).
    pub fn length(&self) -> GeopResult<(Scalar, bool)> {
        Length::length(self)
    }

    /// The parameter `t` at which the fraction `fraction` of the curve's
    /// length, from its start, has been run, for `fraction` in `(0, 1)`:
    /// a sharp value, for a caller choosing where to cut the curve or put
    /// something on it, found by bisection on the length up to `t`, which
    /// grows with `t`. An error for a fraction outside `(0, 1)`.
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
