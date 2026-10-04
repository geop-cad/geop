use core::fmt::Display;

use super::{Field, Ring, Scalar};
use crate::geop_error::{GeopError, GeopResult};

// ── IEEE 754 outward-rounding helpers ─────────────────────────────────────────

#[inline]
fn next_up(x: f64) -> f64 {
    if x.is_nan() {
        return x;
    }
    if x == f64::INFINITY {
        return f64::INFINITY;
    }
    if x == 0.0 {
        return f64::MIN_POSITIVE;
    }
    let bits = x.to_bits();
    let bits = if x > 0.0 { bits + 1 } else { bits - 1 };
    f64::from_bits(bits)
}

#[inline]
fn next_down(x: f64) -> f64 {
    if x.is_nan() {
        return x;
    }
    if x == f64::NEG_INFINITY {
        return f64::NEG_INFINITY;
    }
    if x == 0.0 {
        return -f64::MIN_POSITIVE;
    }
    let bits = x.to_bits();
    let bits = if x < 0.0 { bits + 1 } else { bits - 1 };
    f64::from_bits(bits)
}

// ── Interval trigonometry ─────────────────────────────────────────────────────

/// True iff `[lo, hi]` contains some `target + k * period` for an integer `k`
/// — i.e. whether the interval passes through one of `sin`/`cos`'s
/// extrema, which sampling only the endpoints could miss entirely.
#[inline]
fn contains_periodic(lo: f64, hi: f64, target: f64, period: f64) -> bool {
    let k = ((lo - target) / period).ceil();
    target + k * period <= hi
}

/// Outward-rounded enclosure of `sin`/`cos([lo, hi])`, correct even though
/// `f64::sin`/`f64::cos` are not guaranteed correctly rounded: the true
/// extrema at the given `max_at`/`min_at` phases are recognized structurally
/// (`could_be_periodic`) rather than hunted for numerically, and the sampled
/// endpoint values are widened outward by a further ULP against roundoff in
/// the libm call itself.
fn interval_trig(lo: f64, hi: f64, f: impl Fn(f64) -> f64, max_at: f64, min_at: f64) -> (f64, f64) {
    use std::f64::consts::TAU;
    if !lo.is_finite() || !hi.is_finite() || hi - lo >= TAU {
        return (-1.0, 1.0);
    }
    let (a, b) = (f(lo), f(hi));
    let mut out_lo = next_down(a.min(b));
    let mut out_hi = next_up(a.max(b));
    if contains_periodic(lo, hi, max_at, TAU) {
        out_hi = 1.0;
    }
    if contains_periodic(lo, hi, min_at, TAU) {
        out_lo = -1.0;
    }
    (out_lo.max(-1.0), out_hi.min(1.0))
}

// ── Type ──────────────────────────────────────────────────────────────────────

/// Interval f64 scalar: `[lo, hi]` with outward-rounded arithmetic.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ScalInF64 {
    pub lo: f64,
    pub hi: f64,
}

impl ScalInF64 {
    /// Create a proper interval. Panics in debug if lo > hi.
    #[inline]
    pub fn new(lo: f64, hi: f64) -> Self {
        debug_assert!(lo <= hi, "ScalInF64::new: lo ({lo}) > hi ({hi})");
        ScalInF64 { lo, hi }
    }

    /// Degenerate (point) interval.
    #[inline]
    pub fn degenerate(v: f64) -> Self {
        ScalInF64 { lo: v, hi: v }
    }
}

impl Display for ScalInF64 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:.3}", self.to_f64())
    }
}

impl Ring for ScalInF64 {
    fn add(self, other: Self) -> Self {
        ScalInF64::new(next_down(self.lo + other.lo), next_up(self.hi + other.hi))
    }

    fn sub(self, other: Self) -> Self {
        ScalInF64::new(next_down(self.lo - other.hi), next_up(self.hi - other.lo))
    }

    fn mul(self, other: Self) -> Self {
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        let lo = products.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = products.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        ScalInF64::new(next_down(lo), next_up(hi))
    }

    fn neg(self) -> Self {
        ScalInF64::new(-self.hi, -self.lo)
    }
}

impl Field for ScalInF64 {
    fn div(self, other: Self) -> GeopResult<Self> {
        if other.lo <= 0.0 && other.hi >= 0.0 {
            return Err(GeopError::new(
                "ScalInF64::div: divisor interval contains zero",
            ));
        }
        // [lo,hi] * [1/hi', 1/lo']
        let inv_lo = next_down(1.0 / other.hi);
        let inv_hi = next_up(1.0 / other.lo);
        let inv = ScalInF64::new(inv_lo, inv_hi);
        Ok(self.mul(inv))
    }
}

// ── Scalar impl ───────────────────────────────────────────────────────────────

impl Scalar for ScalInF64 {
    const ZERO: Self = ScalInF64 { lo: 0.0, hi: 0.0 };
    const ONE: Self = ScalInF64 { lo: 1.0, hi: 1.0 };
    const TWO: Self = ScalInF64 { lo: 2.0, hi: 2.0 };

    // π ≈ 3.141592653589793   (next_down/up computed at compile time as literals)
    const PI: Self = ScalInF64 {
        lo: std::f64::consts::PI,   // rounded value is a lower bound for π
        hi: 3.1415926535897936_f64, // next_up(π)
    };
    const E: Self = ScalInF64 {
        lo: std::f64::consts::E, // rounded value is a lower bound for e
        hi: 2.7182818284590455_f64,
    };
    const INFINITY: Self = ScalInF64 {
        lo: f64::INFINITY,
        hi: f64::INFINITY,
    };
    const ENTIRE: Self = ScalInF64 {
        lo: f64::NEG_INFINITY,
        hi: f64::INFINITY,
    };

    fn from_i64(v: i64) -> Self {
        ScalInF64::degenerate(v as f64)
    }
    fn from_f64(v: f64) -> Self {
        ScalInF64::degenerate(v)
    }

    fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        if den == 0 {
            return Err(GeopError::new("ScalInF64::from_ratio: denominator is zero"));
        }
        let exact = num as f64 / den as f64;
        Ok(ScalInF64::new(next_down(exact), next_up(exact)))
    }

    fn abs(self) -> Self {
        if self.lo >= 0.0 {
            self
        } else if self.hi <= 0.0 {
            ScalInF64::new(-self.hi, -self.lo)
        } else {
            ScalInF64::new(0.0, self.lo.abs().max(self.hi.abs()))
        }
    }

    fn sqrt(self) -> GeopResult<Self> {
        if self.hi < 0.0 {
            return Err(GeopError::new(
                "ScalInF64::sqrt: interval is definitely negative",
            ));
        }
        let lo_clamped = if self.lo < 0.0 { 0.0 } else { self.lo };
        Ok(ScalInF64::new(
            next_down(lo_clamped.sqrt()),
            next_up(self.hi.sqrt()),
        ))
    }

    fn sin(self) -> Self {
        use std::f64::consts::FRAC_PI_2;
        let (lo, hi) = interval_trig(self.lo, self.hi, f64::sin, FRAC_PI_2, -FRAC_PI_2);
        ScalInF64::new(lo, hi)
    }

    fn cos(self) -> Self {
        use std::f64::consts::PI;
        let (lo, hi) = interval_trig(self.lo, self.hi, f64::cos, 0.0, PI);
        ScalInF64::new(lo, hi)
    }

    fn acos(self) -> GeopResult<Self> {
        if self.hi < -1.0 || self.lo > 1.0 {
            return Err(GeopError::new(format!(
                "ScalInF64::acos: {self:?} lies outside [-1, 1]"
            )));
        }
        // Decreasing: the upper bound gives the smaller angle. Widened by
        // an ULP against roundoff in the libm call, like `sin`/`cos`.
        let (lo, hi) = (self.lo.max(-1.0), self.hi.min(1.0));
        Ok(ScalInF64::new(
            next_down(hi.acos()).max(0.0),
            next_up(lo.acos()).min(next_up(std::f64::consts::PI)),
        ))
    }

    fn could_be_equal(self, other: Self) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    fn definitely_not_equal(self, other: Self) -> bool {
        self.hi < other.lo || self.lo > other.hi
    }

    fn could_be_greater(self, other: Self) -> bool {
        self.hi > other.lo
    }

    fn definitely_greater(self, other: Self) -> bool {
        self.lo > other.hi
    }

    fn could_be_less(self, other: Self) -> bool {
        self.lo < other.hi
    }

    fn definitely_less(self, other: Self) -> bool {
        self.hi < other.lo
    }

    fn is_infinite(self) -> bool {
        self.lo == f64::NEG_INFINITY || self.hi == f64::INFINITY
    }

    fn is_finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    fn midpoint(self) -> Self {
        let m = (self.lo + self.hi) / 2.0;
        ScalInF64::degenerate(m)
    }

    fn is_sharp(self) -> bool {
        self.lo == self.hi
    }

    fn lower(self) -> Self {
        ScalInF64::degenerate(self.lo)
    }

    fn upper(self) -> Self {
        ScalInF64::degenerate(self.hi)
    }

    fn width(self) -> Self {
        // Non-negative and sharp, so it can serve as a decidable threshold.
        let w = (self.hi - self.lo).max(0.0);
        ScalInF64::new(w, w)
    }

    fn intersect(self, other: Self) -> Self {
        let lo = self.lo.max(other.lo);
        let hi = self.hi.min(other.hi);
        if lo <= hi {
            ScalInF64::new(lo, hi)
        } else if self.hi - self.lo <= other.hi - other.lo {
            self
        } else {
            other
        }
    }

    fn to_f64(self) -> f64 {
        (self.lo + self.hi) / 2.0
    }

    fn union(self, other: Self) -> Self {
        ScalInF64::new(self.lo.min(other.lo), self.hi.max(other.hi))
    }

    fn is_subset_of(self, other: Self) -> bool {
        other.lo <= self.lo && self.hi <= other.hi
    }
}

impl core::ops::Add for ScalInF64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Ring::add(self, rhs)
    }
}
impl core::ops::Sub for ScalInF64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Ring::sub(self, rhs)
    }
}
impl core::ops::Mul for ScalInF64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Ring::mul(self, rhs)
    }
}
impl core::ops::Neg for ScalInF64 {
    type Output = Self;
    fn neg(self) -> Self {
        Ring::neg(self)
    }
}

impl From<i64> for ScalInF64 {
    fn from(v: i64) -> Self {
        ScalInF64::from_i64(v)
    }
}

impl From<f64> for ScalInF64 {
    fn from(v: f64) -> Self {
        ScalInF64::degenerate(v)
    }
}

impl Default for ScalInF64 {
    fn default() -> Self {
        ScalInF64::ZERO
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(v: f64) -> ScalInF64 {
        ScalInF64::degenerate(v)
    }

    fn iv(lo: f64, hi: f64) -> ScalInF64 {
        ScalInF64::new(lo, hi)
    }

    #[test]
    fn arithmetic_add() {
        let r = pt(1.0).add(pt(2.0));
        assert!(r.could_be_equal(pt(3.0)));
    }

    #[test]
    fn arithmetic_sub() {
        let r = pt(5.0).sub(pt(3.0));
        assert!(r.could_be_equal(pt(2.0)));
    }

    #[test]
    fn arithmetic_mul() {
        let r = pt(3.0).mul(pt(4.0));
        assert!(r.could_be_equal(pt(12.0)));
    }

    #[test]
    fn arithmetic_div() {
        let r = pt(10.0).div(pt(2.0)).unwrap();
        assert!(r.could_be_equal(pt(5.0)));
    }

    #[test]
    fn arithmetic_sqrt_4() {
        let r = pt(4.0).sqrt().unwrap();
        assert!(r.could_be_equal(pt(2.0)));
    }

    #[test]
    fn arithmetic_sqrt_9() {
        let r = pt(9.0).sqrt().unwrap();
        assert!(r.could_be_equal(pt(3.0)));
    }

    #[test]
    fn arithmetic_abs() {
        assert!(pt(-3.0).abs().could_be_equal(pt(3.0)));
    }

    #[test]
    fn arithmetic_neg() {
        assert!(pt(-5.0).neg().could_be_equal(pt(5.0)));
    }

    #[test]
    fn sin_at_a_point() {
        assert!(pt(0.0).sin().could_be_equal(pt(0.0)));
        assert!(
            pt(std::f64::consts::FRAC_PI_2)
                .sin()
                .could_be_equal(pt(1.0))
        );
    }

    #[test]
    fn cos_at_a_point() {
        assert!(pt(0.0).cos().could_be_equal(pt(1.0)));
    }

    #[test]
    fn sin_over_a_peak_reaches_exactly_one() {
        // [0, pi] straddles the sin maximum at pi/2; the lower bound stays
        // outward-rounded (sin(0) = 0 exactly, but sin(pi) is a hair above
        // zero in f64, and the enclosure must not round that away).
        let r = iv(0.0, std::f64::consts::PI).sin();
        assert!(r.hi == 1.0 && r.lo >= -1e-15);
    }

    #[test]
    fn cos_over_a_full_turn_is_entire_range() {
        let r = iv(0.0, std::f64::consts::TAU).cos();
        assert!(r.lo == -1.0 && r.hi == 1.0);
    }

    #[test]
    fn sin_narrow_interval_encloses_true_value() {
        // A narrow enclosure of 0.5 radians; sin(0.5) should lie in the result.
        let r = iv(0.5 - 1e-9, 0.5 + 1e-9).sin();
        let truth = 0.5_f64.sin();
        assert!(r.lo <= truth && truth <= r.hi);
    }

    #[test]
    fn err_div_by_zero() {
        assert!(pt(1.0).div(ScalInF64::ZERO).is_err());
    }

    #[test]
    fn err_sqrt_negative() {
        assert!(pt(-1.0).sqrt().is_err());
    }

    #[test]
    fn sqrt_straddles_zero_ok() {
        // Interval [-1, 4] straddles zero — sqrt should succeed and contain 2
        let r = iv(-1.0, 4.0).sqrt().unwrap();
        assert!(r.could_be_equal(pt(2.0)));
    }

    #[test]
    fn acos_encloses_the_angle() {
        let r = iv(-0.5, 0.5).acos().unwrap();
        assert!(r.could_be_equal(pt(std::f64::consts::FRAC_PI_2)));
        assert!(r.lo <= std::f64::consts::FRAC_PI_3 && r.hi >= 2.0 * std::f64::consts::FRAC_PI_3);
        assert!(pt(-1.0).acos().unwrap().could_be_equal(ScalInF64::PI));
        assert!(pt(1.5).acos().is_err());
    }

    #[test]
    fn overflow_infinity() {
        let big = pt(f64::MAX);
        let result = big.mul(big);
        assert!(result.is_infinite());
    }

    #[test]
    fn cmp_definitely_greater() {
        assert!(pt(3.0).definitely_greater(pt(2.0)));
    }

    #[test]
    fn cmp_could_be_less_false() {
        assert!(!pt(3.0).definitely_less(pt(2.0)));
    }

    #[test]
    fn cmp_overlapping_intervals_could_be_equal() {
        assert!(iv(1.0, 3.0).could_be_equal(iv(2.0, 4.0)));
    }

    #[test]
    fn cmp_disjoint_intervals_definitely_not_equal() {
        assert!(iv(1.0, 2.0).definitely_not_equal(iv(3.0, 4.0)));
    }

    #[test]
    fn entire_could_be_equal_point() {
        assert!(ScalInF64::ENTIRE.could_be_equal(pt(42.0)));
        assert!(pt(-1e300).could_be_equal(ScalInF64::ENTIRE));
    }

    #[test]
    fn entire_could_be_equal_interval() {
        assert!(ScalInF64::ENTIRE.could_be_equal(iv(-5.0, 5.0)));
    }

    #[test]
    fn entire_could_be_greater_and_less() {
        assert!(ScalInF64::ENTIRE.could_be_greater(pt(1e300)));
        assert!(ScalInF64::ENTIRE.could_be_less(pt(-1e300)));
    }

    #[test]
    fn entire_never_definitely() {
        assert!(!ScalInF64::ENTIRE.definitely_not_equal(pt(0.0)));
        assert!(!ScalInF64::ENTIRE.definitely_greater(pt(1e300)));
        assert!(!ScalInF64::ENTIRE.definitely_less(pt(-1e300)));
    }

    #[test]
    fn fp_enclosure_01_plus_02() {
        // 0.1 + 0.2 is famously not exactly 0.3; the interval should still enclose the true sum
        let a = iv(next_down(0.1), next_up(0.1));
        let b = iv(next_down(0.2), next_up(0.2));
        let c = iv(next_down(0.3), next_up(0.3));
        let sum = a.add(b);
        assert!(sum.could_be_equal(c));
    }
}
