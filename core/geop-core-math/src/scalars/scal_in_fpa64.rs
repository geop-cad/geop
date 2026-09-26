use super::{Field, Ring, Scalar};
use crate::geop_error::{GeopError, GeopResult};

/// Number of fractional bits.
const F: u32 = 32;
/// Scale factor: 2^F as i128 for widened arithmetic.
const SCALE: i128 = 1_i128 << F;

// ── OVERFLOW sentinel values ──────────────────────────────────────────────────
const OV_LO: i64 = i64::MIN;
const OV_HI: i64 = i64::MAX;

// ── Directed-rounding integer helpers ────────────────────────────────────────

/// Floor division for signed integers (rounds toward −∞).
#[inline]
fn floor_div(n: i128, d: i128) -> i128 {
    let q = n / d;
    let r = n % d;
    // If remainder is non-zero and signs differ, subtract 1
    if r != 0 && (n ^ d) < 0 { q - 1 } else { q }
}

/// Ceiling division for signed integers (rounds toward +∞).
#[inline]
fn ceil_div(n: i128, d: i128) -> i128 {
    -floor_div(-n, d)
}

/// Saturate an i128 into i64, treating extremes as the OVERFLOW sentinel.
#[inline]
fn sat_lo(v: i128) -> i64 {
    if v <= i64::MIN as i128 {
        OV_LO
    } else if v >= i64::MAX as i128 {
        OV_HI
    } else {
        v as i64
    }
}

#[inline]
fn sat_hi(v: i128) -> i64 {
    sat_lo(v)
}

// ── next_up / next_down for f64 (mirrored from scal_in_f64) ──────────────────

#[inline]
fn next_up_f64(x: f64) -> f64 {
    if x.is_nan() || x == f64::INFINITY {
        return x;
    }
    if x == 0.0 {
        return f64::MIN_POSITIVE;
    }
    let bits = x.to_bits();
    f64::from_bits(if x > 0.0 { bits + 1 } else { bits - 1 })
}

#[inline]
fn next_down_f64(x: f64) -> f64 {
    if x.is_nan() || x == f64::NEG_INFINITY {
        return x;
    }
    if x == 0.0 {
        return -f64::MIN_POSITIVE;
    }
    let bits = x.to_bits();
    f64::from_bits(if x < 0.0 { bits + 1 } else { bits - 1 })
}

// ── Interval trigonometry (widened to f64, like `sqrt` above) ────────────────

/// True iff `[lo, hi]` contains some `target + k * period` for an integer `k`.
#[inline]
fn contains_periodic(lo: f64, hi: f64, target: f64, period: f64) -> bool {
    let k = ((lo - target) / period).ceil();
    target + k * period <= hi
}

/// Outward-rounded enclosure of `sin`/`cos([lo, hi])`, as f64 bounds — mirrors
/// `scal_in_f64::interval_trig`.
fn interval_trig(lo: f64, hi: f64, f: impl Fn(f64) -> f64, max_at: f64, min_at: f64) -> (f64, f64) {
    use std::f64::consts::TAU;
    if !lo.is_finite() || !hi.is_finite() || hi - lo >= TAU {
        return (-1.0, 1.0);
    }
    let (a, b) = (f(lo), f(hi));
    let mut out_lo = next_down_f64(a.min(b));
    let mut out_hi = next_up_f64(a.max(b));
    if contains_periodic(lo, hi, max_at, TAU) {
        out_hi = 1.0;
    }
    if contains_periodic(lo, hi, min_at, TAU) {
        out_lo = -1.0;
    }
    (out_lo.max(-1.0), out_hi.min(1.0))
}

// ── Type ──────────────────────────────────────────────────────────────────────

/// Interval fixed-point scalar: `[lo, hi]` at 2^-32 resolution (i64 units).
#[derive(Copy, Clone, PartialEq)]
pub struct ScalInFPA64 {
    pub lo: i64,
    pub hi: i64,
}

impl ScalInFPA64 {
    #[inline]
    pub fn new(lo: i64, hi: i64) -> Self {
        debug_assert!(lo <= hi, "ScalInFPA64::new: lo ({lo}) > hi ({hi})");
        ScalInFPA64 { lo, hi }
    }

    #[inline]
    pub fn degenerate(v: i64) -> Self {
        ScalInFPA64 { lo: v, hi: v }
    }

    #[inline]
    fn is_overflow(self) -> bool {
        self.lo == OV_LO || self.hi == OV_HI
    }

    /// Convert a real-valued f64 to fixed-point, rounding outward.
    pub fn from_f64_outward(lo_f: f64, hi_f: f64) -> Self {
        let lo_i = (lo_f * SCALE as f64).floor() as i128;
        let hi_i = (hi_f * SCALE as f64).ceil() as i128;
        ScalInFPA64::new(sat_lo(lo_i), sat_hi(hi_i))
    }
}

impl core::fmt::Debug for ScalInFPA64 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let lo_f = self.lo as f64 / SCALE as f64;
        let hi_f = self.hi as f64 / SCALE as f64;
        write!(f, "[{lo_f}, {hi_f}]")
    }
}

impl core::fmt::Display for ScalInFPA64 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let lo_f = self.lo as f64 / SCALE as f64;
        let hi_f = self.hi as f64 / SCALE as f64;
        write!(f, "{:.3}", (lo_f + hi_f) / 2.0)
    }
}

// ── Constants (pre-computed bit patterns) ─────────────────────────────────────
//
// π * 2^32 = 13493037704.92…  → floor = 13493037704, ceil = 13493037705
// e * 2^32 = 11674931555.08…  → floor = 11674931555, ceil = 11674931556

const PI_LO: i64 = 13493037704;
const PI_HI: i64 = 13493037705;
const E_LO: i64 = 11674931555;
const E_HI: i64 = 11674931556;

impl Ring for ScalInFPA64 {
    fn add(self, other: Self) -> Self {
        let lo = self.lo.saturating_add(other.lo);
        let hi = self.hi.saturating_add(other.hi);
        ScalInFPA64::new(lo, hi)
    }

    fn sub(self, other: Self) -> Self {
        let lo = self.lo.saturating_sub(other.hi);
        let hi = self.hi.saturating_sub(other.lo);
        ScalInFPA64::new(lo, hi)
    }

    fn mul(self, other: Self) -> Self {
        let products = [
            (self.lo as i128) * (other.lo as i128),
            (self.lo as i128) * (other.hi as i128),
            (self.hi as i128) * (other.lo as i128),
            (self.hi as i128) * (other.hi as i128),
        ];
        let raw_lo = *products.iter().min().unwrap();
        let raw_hi = *products.iter().max().unwrap();
        let lo = floor_div(raw_lo, SCALE);
        let hi = ceil_div(raw_hi, SCALE);
        ScalInFPA64::new(sat_lo(lo), sat_hi(hi))
    }

    fn neg(self) -> Self {
        ScalInFPA64::new(self.hi.saturating_neg(), self.lo.saturating_neg())
    }
}

impl Field for ScalInFPA64 {
    fn div(self, other: Self) -> GeopResult<Self> {
        if other.lo <= 0 && other.hi >= 0 {
            return Err(GeopError::new(
                "ScalInFPA64::div: divisor interval contains zero",
            ));
        }
        let a_lo = (self.lo as i128) << F;
        let a_hi = (self.hi as i128) << F;
        let b_lo = other.lo as i128;
        let b_hi = other.hi as i128;

        let candidates = [
            floor_div(a_lo, b_hi),
            floor_div(a_lo, b_lo),
            floor_div(a_hi, b_hi),
            floor_div(a_hi, b_lo),
        ];
        let candidates_hi = [
            ceil_div(a_lo, b_hi),
            ceil_div(a_lo, b_lo),
            ceil_div(a_hi, b_hi),
            ceil_div(a_hi, b_lo),
        ];
        let lo = *candidates.iter().min().unwrap();
        let hi = *candidates_hi.iter().max().unwrap();
        Ok(ScalInFPA64::new(sat_lo(lo), sat_hi(hi)))
    }
}

// ── Scalar impl ───────────────────────────────────────────────────────────────

impl Scalar for ScalInFPA64 {
    const ZERO: Self = ScalInFPA64 { lo: 0, hi: 0 };
    const ONE: Self = ScalInFPA64 {
        lo: SCALE as i64,
        hi: SCALE as i64,
    };
    const TWO: Self = ScalInFPA64 {
        lo: 2 * SCALE as i64,
        hi: 2 * SCALE as i64,
    };
    const PI: Self = ScalInFPA64 {
        lo: PI_LO,
        hi: PI_HI,
    };
    const E: Self = ScalInFPA64 { lo: E_LO, hi: E_HI };
    const INFINITY: Self = ScalInFPA64 {
        lo: OV_LO,
        hi: OV_HI,
    };
    const ENTIRE: Self = ScalInFPA64 {
        lo: i64::MIN,
        hi: i64::MAX,
    };

    fn from_f64(v: f64) -> Self {
        ScalInFPA64::from_f64_outward(v, v)
    }
    fn from_i64(v: i64) -> Self {
        let shifted = (v as i128).checked_mul(SCALE);
        match shifted {
            Some(s) if s >= i64::MIN as i128 && s <= i64::MAX as i128 => {
                ScalInFPA64::degenerate(s as i64)
            }
            _ => ScalInFPA64::INFINITY,
        }
    }

    fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        if den == 0 {
            return Err(GeopError::new(
                "ScalInFPA64::from_ratio: denominator is zero",
            ));
        }
        let n = (num as i128) << F;
        let d = den as i128;
        let lo = floor_div(n, d);
        let hi = ceil_div(n, d);
        Ok(ScalInFPA64::new(sat_lo(lo), sat_hi(hi)))
    }

    fn abs(self) -> Self {
        if self.lo >= 0 {
            self
        } else if self.hi <= 0 {
            ScalInFPA64::new(self.hi.saturating_neg(), self.lo.saturating_neg())
        } else {
            let hi = self.lo.saturating_neg().max(self.hi);
            ScalInFPA64::new(0, hi)
        }
    }

    fn sqrt(self) -> GeopResult<Self> {
        if self.hi < 0 {
            return Err(GeopError::new(
                "ScalInFPA64::sqrt: interval is definitely negative",
            ));
        }
        let lo_clamped = if self.lo < 0 { 0i64 } else { self.lo };
        // Convert to f64, sqrt with outward rounding, convert back
        let lo_f = (lo_clamped as f64) / SCALE as f64;
        let hi_f = (self.hi as f64) / SCALE as f64;
        let sqrt_lo = next_down_f64(lo_f.sqrt());
        let sqrt_hi = next_up_f64(hi_f.sqrt());
        // Convert back to fixed-point with outward rounding
        let lo_fixed = (sqrt_lo * SCALE as f64).floor() as i64;
        let hi_fixed = (sqrt_hi * SCALE as f64).ceil() as i64;
        Ok(ScalInFPA64::new(lo_fixed, hi_fixed))
    }

    fn sin(self) -> Self {
        // No native fixed-point trig: widen to f64 (as `sqrt` already does
        // above), reuse the f64 interval routine, then round outward back
        // into fixed point. An overflowed operand converts to a huge (but
        // finite) f64 span, which the `hi - lo >= TAU` check below still
        // correctly collapses to the full `[-1, 1]` range.
        use std::f64::consts::FRAC_PI_2;
        let (lo, hi) = interval_trig(
            self.lo as f64 / SCALE as f64,
            self.hi as f64 / SCALE as f64,
            f64::sin,
            FRAC_PI_2,
            -FRAC_PI_2,
        );
        ScalInFPA64::from_f64_outward(lo, hi)
    }

    fn cos(self) -> Self {
        use std::f64::consts::PI;
        let (lo, hi) = interval_trig(
            self.lo as f64 / SCALE as f64,
            self.hi as f64 / SCALE as f64,
            f64::cos,
            0.0,
            PI,
        );
        ScalInFPA64::from_f64_outward(lo, hi)
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
        self.is_overflow()
    }

    fn is_finite(self) -> bool {
        !self.is_overflow()
    }

    fn midpoint(self) -> Self {
        // Avoid overflow: (lo + hi) / 2 using i128
        let m = ((self.lo as i128 + self.hi as i128) / 2) as i64;
        ScalInFPA64::degenerate(m)
    }

    fn is_sharp(self) -> bool {
        self.lo == self.hi
    }

    fn lower(self) -> Self {
        ScalInFPA64::degenerate(self.lo)
    }

    fn upper(self) -> Self {
        ScalInFPA64::degenerate(self.hi)
    }

    fn width(self) -> Self {
        let w = self.hi.saturating_sub(self.lo).max(0);
        ScalInFPA64::new(w, w)
    }

    fn intersect(self, other: Self) -> Self {
        let lo = self.lo.max(other.lo);
        let hi = self.hi.min(other.hi);
        if lo <= hi {
            ScalInFPA64::new(lo, hi)
        } else if self.hi.saturating_sub(self.lo) <= other.hi.saturating_sub(other.lo) {
            self
        } else {
            other
        }
    }

    fn to_f64(self) -> f64 {
        let m = (self.lo as i128 + self.hi as i128) / 2;
        m as f64 / SCALE as f64
    }

    fn union(self, other: Self) -> Self {
        ScalInFPA64::new(self.lo.min(other.lo), self.hi.max(other.hi))
    }

    fn is_subset_of(self, other: Self) -> bool {
        other.lo <= self.lo && self.hi <= other.hi
    }
}

impl core::ops::Add for ScalInFPA64 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Ring::add(self, rhs)
    }
}
impl core::ops::Sub for ScalInFPA64 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Ring::sub(self, rhs)
    }
}
impl core::ops::Mul for ScalInFPA64 {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Ring::mul(self, rhs)
    }
}
impl core::ops::Neg for ScalInFPA64 {
    type Output = Self;
    fn neg(self) -> Self {
        Ring::neg(self)
    }
}

impl From<i64> for ScalInFPA64 {
    fn from(v: i64) -> Self {
        ScalInFPA64::from_i64(v)
    }
}

impl From<f64> for ScalInFPA64 {
    fn from(v: f64) -> Self {
        ScalInFPA64::from_f64(v)
    }
}

impl Default for ScalInFPA64 {
    fn default() -> Self {
        ScalInFPA64::ZERO
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(v: i64) -> ScalInFPA64 {
        ScalInFPA64::from_i64(v)
    }

    fn iv(lo: i64, hi: i64) -> ScalInFPA64 {
        ScalInFPA64::new(lo << F, hi << F)
    }

    #[test]
    fn arithmetic_add() {
        let r = pt(1).add(pt(2));
        assert!(r.could_be_equal(pt(3)));
    }

    #[test]
    fn arithmetic_sub() {
        let r = pt(5).sub(pt(3));
        assert!(r.could_be_equal(pt(2)));
    }

    #[test]
    fn arithmetic_mul() {
        let r = pt(3).mul(pt(4));
        assert!(r.could_be_equal(pt(12)));
    }

    #[test]
    fn arithmetic_div() {
        let r = pt(10).div(pt(2)).unwrap();
        assert!(r.could_be_equal(pt(5)));
    }

    #[test]
    fn arithmetic_sqrt_4() {
        let r = pt(4).sqrt().unwrap();
        assert!(r.could_be_equal(pt(2)));
    }

    #[test]
    fn arithmetic_sqrt_9() {
        let r = pt(9).sqrt().unwrap();
        assert!(r.could_be_equal(pt(3)));
    }

    #[test]
    fn arithmetic_abs() {
        assert!(pt(-3).abs().could_be_equal(pt(3)));
    }

    #[test]
    fn arithmetic_neg() {
        assert!(pt(-5).neg().could_be_equal(pt(5)));
    }

    #[test]
    fn sin_at_a_point() {
        assert!(pt(0).sin().could_be_equal(pt(0)));
    }

    #[test]
    fn cos_at_a_point() {
        assert!(pt(0).cos().could_be_equal(pt(1)));
    }

    #[test]
    fn sin_over_a_peak_reaches_exactly_one() {
        let lo = ScalInFPA64::from_f64(0.0);
        let hi = ScalInFPA64::from_f64(std::f64::consts::PI);
        let r = ScalInFPA64::new(lo.lo, hi.hi).sin();
        assert!(r.hi == ScalInFPA64::ONE.hi && r.lo >= -1);
    }

    #[test]
    fn err_div_by_zero() {
        assert!(pt(1).div(ScalInFPA64::ZERO).is_err());
    }

    #[test]
    fn err_sqrt_negative() {
        assert!(pt(-1).sqrt().is_err());
    }

    #[test]
    fn sqrt_straddles_zero_ok() {
        // [-1, 4] straddles zero; sqrt should succeed and contain 2
        let neg_one = -(1_i64 << F);
        let four = 4_i64 << F;
        let r = ScalInFPA64::new(neg_one, four).sqrt().unwrap();
        assert!(r.could_be_equal(pt(2)));
    }

    #[test]
    fn overflow_infinity() {
        let big = ScalInFPA64::new(i64::MAX / 2, i64::MAX / 2);
        let result = big.mul(big);
        assert!(result.is_infinite());
    }

    #[test]
    fn cmp_definitely_greater() {
        assert!(pt(3).definitely_greater(pt(2)));
    }

    #[test]
    fn cmp_could_be_less_false() {
        assert!(!pt(3).definitely_less(pt(2)));
    }

    #[test]
    fn cmp_overlapping_intervals_could_be_equal() {
        assert!(iv(1, 3).could_be_equal(iv(2, 4)));
    }

    #[test]
    fn cmp_disjoint_intervals_definitely_not_equal() {
        assert!(iv(1, 2).definitely_not_equal(iv(3, 4)));
    }

    #[test]
    fn entire_could_be_equal_point() {
        assert!(ScalInFPA64::ENTIRE.could_be_equal(pt(42)));
        assert!(pt(-1_000_000).could_be_equal(ScalInFPA64::ENTIRE));
    }

    #[test]
    fn entire_could_be_equal_interval() {
        assert!(ScalInFPA64::ENTIRE.could_be_equal(iv(-5, 5)));
    }

    #[test]
    fn entire_could_be_greater_and_less() {
        assert!(ScalInFPA64::ENTIRE.could_be_greater(pt(1_000_000)));
        assert!(ScalInFPA64::ENTIRE.could_be_less(pt(-1_000_000)));
    }

    #[test]
    fn entire_never_definitely() {
        assert!(!ScalInFPA64::ENTIRE.definitely_not_equal(pt(0)));
        assert!(!ScalInFPA64::ENTIRE.definitely_greater(pt(1_000_000)));
        assert!(!ScalInFPA64::ENTIRE.definitely_less(pt(-1_000_000)));
    }
}
