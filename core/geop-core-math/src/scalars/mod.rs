//! The kernel's number: an interval `[lo, hi]` of `f64`, every operation
//! rounded outward, so that a value encloses every real it could be and
//! comparisons are three-valued (`could_be_equal`, `definitely_greater`).
//! [`Number`] is what a residual of the constraint solvers is written in:
//! a [`Scalar`], or a [`crate::dual::Dual`] of one carrying derivatives.

mod number;

use core::fmt::Display;

pub use number::Number;

use crate::geop_error::{GeopError, GeopResult};

// ── IEEE 754 outward-rounding helpers ─────────────────────────────────────────

#[inline]
pub(crate) fn next_up(x: f64) -> f64 {
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
pub(crate) fn next_down(x: f64) -> f64 {
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

/// A number as the kernel computes with it: an interval `[lo, hi]` of
/// `f64` with outward-rounded arithmetic. The default is zero, `[0, 0]`.
/// A plain value, borrowing nothing: what is worked out in one can be kept
/// as any other value — a drawing's views, in the session editing it.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Scalar {
    pub lo: f64,
    pub hi: f64,
}

impl Display for Scalar {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:.3}", self.to_f64())
    }
}

#[allow(
    clippy::should_implement_trait,
    reason = "the arithmetic is fallible (`div`) and spelled as `Number` spells it, so a residual reads the same over either"
)]
impl Scalar {
    pub const ZERO: Self = Scalar { lo: 0.0, hi: 0.0 };
    pub const ONE: Self = Scalar { lo: 1.0, hi: 1.0 };
    pub const TWO: Self = Scalar { lo: 2.0, hi: 2.0 };

    // π ≈ 3.141592653589793   (next_down/up computed at compile time as literals)
    pub const PI: Self = Scalar {
        lo: std::f64::consts::PI,   // rounded value is a lower bound for π
        hi: 3.1415926535897936_f64, // next_up(π)
    };
    /// Saturation sentinel — set on overflow.
    pub const INFINITY: Self = Scalar {
        lo: f64::INFINITY,
        hi: f64::INFINITY,
    };
    /// The "entire" interval `(-inf, inf)` — the top element of the interval
    /// lattice. `could_be_equal`/`could_be_greater`/`could_be_less` against
    /// it are always `true`, and it never satisfies `definitely_*`. Used to
    /// represent a value or a whole curve/surface whose position is not yet
    /// known — an unsharp placeholder that automatically passes any
    /// overlap/equality check made against it.
    pub const ENTIRE: Self = Scalar {
        lo: f64::NEG_INFINITY,
        hi: f64::INFINITY,
    };

    // ── Construction ─────────────────────────────────────────────────────────

    /// Create a proper interval. Panics in debug if lo > hi.
    #[inline]
    pub fn new(lo: f64, hi: f64) -> Self {
        debug_assert!(lo <= hi, "Scalar::new: lo ({lo}) > hi ({hi})");
        Scalar { lo, hi }
    }

    /// Degenerate (point) interval.
    #[inline]
    pub fn degenerate(v: f64) -> Self {
        Scalar { lo: v, hi: v }
    }

    #[inline]
    pub fn from_i64(v: i64) -> Self {
        Scalar::degenerate(v as f64)
    }

    #[inline]
    pub fn from_f64(v: f64) -> Self {
        Scalar::degenerate(v)
    }

    #[inline]
    pub fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        if den == 0 {
            return Err(GeopError::new("Scalar::from_ratio: denominator is zero"));
        }
        let exact = num as f64 / den as f64;
        Ok(Scalar::new(next_down(exact), next_up(exact)))
    }

    /// Approximate f64 midpoint, `(lo + hi) / 2`. Used only for
    /// rendering/debugging.
    #[inline]
    pub fn to_f64(self) -> f64 {
        (self.lo + self.hi) / 2.0
    }

    // ── Arithmetic ───────────────────────────────────────────────────────────

    #[inline]
    pub fn add(self, other: Self) -> Self {
        Scalar::new(next_down(self.lo + other.lo), next_up(self.hi + other.hi))
    }

    #[inline]
    pub fn sub(self, other: Self) -> Self {
        Scalar::new(next_down(self.lo - other.hi), next_up(self.hi - other.lo))
    }

    #[inline]
    pub fn mul(self, other: Self) -> Self {
        let products = [
            self.lo * other.lo,
            self.lo * other.hi,
            self.hi * other.lo,
            self.hi * other.hi,
        ];
        let lo = products.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = products.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Scalar::new(next_down(lo), next_up(hi))
    }

    #[inline]
    pub fn neg(self) -> Self {
        Scalar::new(-self.hi, -self.lo)
    }

    /// Fails where the divisor could be zero.
    #[inline]
    pub fn div(self, other: Self) -> GeopResult<Self> {
        if other.lo <= 0.0 && other.hi >= 0.0 {
            return Err(GeopError::new(
                "Scalar::div: divisor interval contains zero",
            ));
        }
        // [lo,hi] * [1/hi', 1/lo']
        let inv_lo = next_down(1.0 / other.hi);
        let inv_hi = next_up(1.0 / other.lo);
        let inv = Scalar::new(inv_lo, inv_hi);
        Ok(self.mul(inv))
    }

    // ── Real-valued operations ───────────────────────────────────────────────

    #[inline]
    pub fn abs(self) -> Self {
        if self.lo >= 0.0 {
            self
        } else if self.hi <= 0.0 {
            Scalar::new(-self.hi, -self.lo)
        } else {
            Scalar::new(0.0, self.lo.abs().max(self.hi.abs()))
        }
    }

    #[inline]
    pub fn sqrt(self) -> GeopResult<Self> {
        if self.hi < 0.0 {
            return Err(GeopError::new(
                "Scalar::sqrt: interval is definitely negative",
            ));
        }
        let lo_clamped = if self.lo < 0.0 { 0.0 } else { self.lo };
        Ok(Scalar::new(
            next_down(lo_clamped.sqrt()),
            next_up(self.hi.sqrt()),
        ))
    }

    /// Outward-rounded enclosure of `sin` over the whole interval
    /// (radians). Total — never fails, even for [`Scalar::ENTIRE`] or an
    /// [`Scalar::INFINITY`]-adjacent value, which just widen to `[-1, 1]`.
    #[inline]
    pub fn sin(self) -> Self {
        use std::f64::consts::FRAC_PI_2;
        let (lo, hi) = interval_trig(self.lo, self.hi, f64::sin, FRAC_PI_2, -FRAC_PI_2);
        Scalar::new(lo, hi)
    }

    /// Outward-rounded enclosure of `cos` (see [`Scalar::sin`]).
    #[inline]
    pub fn cos(self) -> Self {
        use std::f64::consts::PI;
        let (lo, hi) = interval_trig(self.lo, self.hi, f64::cos, 0.0, PI);
        Scalar::new(lo, hi)
    }

    /// Outward-rounded enclosure of `acos` over the interval, in `[0, π]`:
    /// the angle whose cosine it is. The part of the interval beyond
    /// `[-1, 1]` is left out; an interval with nothing in `[-1, 1]` fails.
    #[inline]
    pub fn acos(self) -> GeopResult<Self> {
        if self.hi < -1.0 || self.lo > 1.0 {
            return Err(GeopError::new(format!(
                "Scalar::acos: {self:?} lies outside [-1, 1]"
            )));
        }
        // Decreasing: the upper bound gives the smaller angle. Widened by
        // an ULP against roundoff in the libm call, like `sin`/`cos`.
        let (lo, hi) = (self.lo.max(-1.0), self.hi.min(1.0));
        Ok(Scalar::new(
            next_down(hi.acos()).max(0.0),
            next_up(lo.acos()).min(next_up(std::f64::consts::PI)),
        ))
    }

    /// The angle `self`, given in degrees, in radians.
    pub fn degrees_to_radians(self) -> GeopResult<Self> {
        self.mul(Self::PI).div(Self::from_i64(180))
    }

    /// An enclosure of the angle of the point `(x, self)` from the `x` axis,
    /// in radians, in `(-pi, pi]`: of `atan2(y, x)` for every `y` in `self`
    /// and `x` in `x`. All of `[-pi, pi]` where that is not one range of
    /// angles: the box `(x, self)` holds the origin, or reaches across the
    /// negative `x` axis, where the angle jumps.
    ///
    /// Over a box clear of the origin the angle is least and greatest at
    /// corners. Those are taken in `f64` — free choices of where to put the
    /// bounds — and then each is proven, not trusted: a bound `b` holds when
    /// every point of the box lies on the far side of the ray at angle `b`,
    /// `cos(b) y - sin(b) x` definitely of the right sign, evaluated in the
    /// scalar's own outward-rounded arithmetic. Where rounding leaves that
    /// undecided the bound moves outward, by steps that double, until it
    /// holds.
    pub fn atan2(self, x: Self) -> Self {
        let y = self;
        let whole = Self::PI.neg().union(Self::PI);
        // On the negative `x` axis itself the angle is `pi`, just below it
        // near `-pi`: a box reaching below it from there jumps too.
        if (y.could_be_equal(Self::ZERO) && x.could_be_equal(Self::ZERO))
            || (x.could_be_less(Self::ZERO)
                && y.could_be_less(Self::ZERO)
                && !y.definitely_less(Self::ZERO))
        {
            return whole;
        }
        // `+ 0.0` turns a zero's sign positive: on the negative `x` axis
        // the angle is `pi`, never `-pi`.
        let ys = [y.lower().to_f64() + 0.0, y.upper().to_f64() + 0.0];
        let xs = [x.lower().to_f64(), x.upper().to_f64()];
        let corners = ys.iter().flat_map(|&a| xs.iter().map(move |&b| a.atan2(b)));
        let (low, high) = corners.fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), a| {
            (l.min(a), h.max(a))
        });
        // Whether every point of the box is at an angle `>= b` (`sign = 1`)
        // or `<= b` (`sign = -1`).
        let holds = |b: f64, sign: f64| {
            let b = Self::from_f64(b);
            let side = b.cos().mul(y).sub(b.sin().mul(x)).mul(Self::from_f64(sign));
            !side.could_be_less(Self::ZERO)
        };
        let prove = |start: f64, sign: f64, limit: f64| -> Option<f64> {
            let mut b = start;
            let mut step = f64::EPSILON * start.abs().max(1.0);
            for _ in 0..64 {
                if holds(b, sign) {
                    return Some(b);
                }
                b -= sign * step;
                step *= 2.0;
                if (b - limit) * sign < 0.0 {
                    return None;
                }
            }
            None
        };
        let pi = std::f64::consts::PI;
        match (prove(low, 1.0, -pi - 1.0), prove(high, -1.0, pi + 1.0)) {
            (Some(lo), Some(hi)) => Self::from_f64(lo)
                .union(Self::from_f64(hi))
                .intersect(whole),
            _ => whole,
        }
    }

    // ── Three-valued comparisons ─────────────────────────────────────────────

    #[inline]
    pub fn could_be_equal(self, other: Self) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    #[inline]
    pub fn definitely_not_equal(self, other: Self) -> bool {
        self.hi < other.lo || self.lo > other.hi
    }

    #[inline]
    pub fn could_be_greater(self, other: Self) -> bool {
        self.hi > other.lo
    }

    #[inline]
    pub fn definitely_greater(self, other: Self) -> bool {
        self.lo > other.hi
    }

    #[inline]
    pub fn could_be_less(self, other: Self) -> bool {
        self.lo < other.hi
    }

    #[inline]
    pub fn definitely_less(self, other: Self) -> bool {
        self.hi < other.lo
    }

    /// Whether it is definitely zero — no value it could be is less or more
    /// than zero: an exact zero, such as the entry of a column a row has
    /// nothing in.
    pub fn is_zero(self) -> bool {
        !self.could_be_less(Self::ZERO) && !self.could_be_greater(Self::ZERO)
    }

    /// The sign it definitely has: `Some(true)` if it is definitely
    /// positive, `Some(false)` if definitely negative, `None` where it could
    /// be zero.
    pub fn definite_sign(self) -> Option<bool> {
        if self.definitely_greater(Self::ZERO) {
            Some(true)
        } else if self.definitely_less(Self::ZERO) {
            Some(false)
        } else {
            None
        }
    }

    // ── Finiteness ───────────────────────────────────────────────────────────

    #[inline]
    pub fn is_infinite(self) -> bool {
        self.lo == f64::NEG_INFINITY || self.hi == f64::INFINITY
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    // ── Set-valued helpers ───────────────────────────────────────────────────

    #[inline]
    pub fn midpoint(self) -> Self {
        let m = (self.lo + self.hi) / 2.0;
        Scalar::degenerate(m)
    }

    /// True iff this value carries no width — it's a single, exactly-known
    /// point, not a genuine range of possibility.
    #[inline]
    pub fn is_sharp(self) -> bool {
        self.lo == self.hi
    }

    /// How much possibility this enclosure carries: `hi - lo`, as a **sharp,
    /// non-negative** value. Zero exactly when [`Scalar::is_sharp`].
    ///
    /// This is how much a computed quantity is *not* known. Being sharp
    /// itself is what makes it usable as a threshold — comparing an uncertain
    /// width against an uncertain bound could never be decided three-valuedly
    /// (see `validation::numerical_accuracy`).
    #[inline]
    pub fn width(self) -> Self {
        // Non-negative and sharp, so it can serve as a decidable threshold.
        let w = (self.hi - self.lo).max(0.0);
        Scalar::new(w, w)
    }

    /// The sharp lower endpoint of this enclosure. Every value `self` could
    /// be is `>= lower()` and `<= upper()`, so these are the *outer* bounds
    /// to cut at when a search restricts a domain to an enclosure of its
    /// answer: a cut there never loses a solution (unlike
    /// [`Scalar::sharpen`], which would cut through the enclosure).
    #[inline]
    pub fn lower(self) -> Self {
        Self::degenerate(self.lo)
    }

    /// The sharp upper endpoint (see [`Scalar::lower`]).
    #[inline]
    pub fn upper(self) -> Self {
        Self::degenerate(self.hi)
    }

    /// Collapse to a single representative point (currently the midpoint,
    /// like [`Scalar::midpoint`], but named for its distinct *purpose*: use
    /// this only when you are free to pick *any* value within `self` and
    /// don't need to preserve which one — e.g. choosing where to place a
    /// new knot when subdividing a curve at an arbitrary interior point.
    /// **Never** use this to compress a value that represents a genuinely
    /// uncertain physical quantity (a search's converged bound, a measured
    /// position) — that would silently discard real uncertainty rather than
    /// making an arbitrary, harmless choice.
    ///
    /// Exists to break a specific class of interval blowup: repeatedly
    /// re-deriving a split point as `(t0 + t1) / 2` from an already-widened
    /// domain propagates and compounds that width forever, even though nothing
    /// downstream actually cares *which* interior point was chosen — only
    /// that some valid one was. Sharpening throws that unneeded width away
    /// at the source instead of letting every later `alpha = (t - e) / (s - e)`
    /// division amplify it further.
    pub fn sharpen(self) -> Self {
        self.midpoint()
    }

    /// The point a fraction `f` of the way from `lo` to `hi`, sharp: a
    /// sample position, a free choice (see [`Scalar::sharpen`]) — not an
    /// enclosure of the point there, which [`Scalar::interpolate`] is.
    pub fn fraction(lo: Self, hi: Self, f: Self) -> Self {
        lo.add(hi.sub(lo).mul(f)).sharpen()
    }

    /// Point a fraction `alpha` of the way from `a` to `b`: `a` at
    /// `alpha=0`, `b` at `alpha=1`.
    ///
    /// Deliberately `a.add(alpha.mul(b.sub(a)))`, *not* the equally-valid
    /// `a.mul(Scalar::ONE.sub(alpha)).add(b.mul(alpha))` — both give the
    /// same exact real result, but the latter computes `alpha` and
    /// `1-alpha` as two *decorrelated* intervals before ever relating `a`
    /// and `b`, so interval arithmetic can't recognize when they cancel.
    /// This form computes `b.sub(a)` first: when `a` and `b` are honestly
    /// the same value (e.g. a weight that should stay exactly `1.0` across
    /// many subdivisions), that subtraction is exactly `0` regardless of
    /// `alpha`'s own width, and the whole expression collapses to exactly
    /// `a` instead of needlessly widening with every call.
    pub fn interpolate(a: Self, b: Self, alpha: Self) -> Self {
        // Two algebraically identical forms with *opposite* numerical
        // strengths, so this evaluates both and keeps their intersection —
        // both are honest enclosures of the same exact value, so the
        // narrower parts of each are jointly valid, and no magic tolerance
        // is involved in preferring them.
        //
        // - `a + alpha*(b - a)` mentions `a` twice (so `a`'s own width is
        //   counted twice, decorrelated) but computes `b - a` first: when
        //   `a` and `b` are honestly equal — a weight that should stay
        //   exactly `1.0` across many subdivisions, say — that difference
        //   is exactly zero and the whole thing collapses to exactly `a`,
        //   no matter how wide `alpha` is.
        // - `(1 - alpha)*a + alpha*b` mentions each of `a`/`b` once, so
        //   wide control points don't get double-counted, but it splits
        //   `alpha` into two decorrelated factors and so can't see the
        //   `a == b` cancellation at all.
        //
        // Neither dominates: the first is what a repeatedly-split curve's
        // weights need, the second is what a surface patch with genuinely
        // wide control points needs.
        let via_delta = a.add(alpha.mul(b.sub(a)));
        let via_weights = Self::ONE.sub(alpha).mul(a).add(alpha.mul(b));
        via_delta.intersect(via_weights)
    }

    /// The largest value contained in *both* `self` and `other` — the dual
    /// of [`Scalar::union`]. Callers must only intersect two enclosures of
    /// the same underlying exact value (as [`Scalar::interpolate`] does);
    /// given that, the result is still an honest enclosure, just a tighter
    /// one. Returns the narrower input if the two somehow don't overlap,
    /// rather than fabricating an empty/inverted interval.
    #[inline]
    pub fn intersect(self, other: Self) -> Self {
        let lo = self.lo.max(other.lo);
        let hi = self.hi.min(other.hi);
        if lo <= hi {
            Scalar::new(lo, hi)
        } else if self.hi - self.lo <= other.hi - other.lo {
            self
        } else {
            other
        }
    }

    /// The smallest value definitely containing both `self` and `other` —
    /// the scalar-level analog of `Set::union`.
    #[inline]
    pub fn union(self, other: Self) -> Self {
        Self::new(self.lo.min(other.lo), self.hi.max(other.hi))
    }

    /// True iff `self` is contained in `other` as sets: `other.lo <= self.lo`
    /// and `self.hi <= other.hi`. This is the rigorous existence/uniqueness
    /// test a Krawczyk-style contraction relies on (`K(X) ⊆ X`) — distinct
    /// from [`Scalar::could_be_equal`], which only asks whether the two
    /// enclosures *overlap*.
    #[inline]
    pub fn is_subset_of(self, other: Self) -> bool {
        other.lo <= self.lo && self.hi <= other.hi
    }

    /// Whether it is exactly `other`, as a set: both the same single value,
    /// or both the same range. A weight that is exactly one is skipped
    /// rather than multiplied by, since the product would only be widened
    /// by rounding.
    pub fn is_exactly(self, other: Self) -> bool {
        self.is_subset_of(other) && other.is_subset_of(self)
    }

    /// The larger of the two: an enclosure of `max(a, b)` for every `a` and
    /// `b` the two could be — sharp where both are.
    pub fn max(self, other: Self) -> Self {
        let pick = |a: Self, b: Self| if b.definitely_greater(a) { b } else { a };
        pick(self.lower(), other.lower()).union(pick(self.upper(), other.upper()))
    }

    /// The smaller of the two (see [`Scalar::max`]).
    pub fn min(self, other: Self) -> Self {
        self.neg().max(other.neg()).neg()
    }

    /// Clamped into `[lo, hi]`: moved to a bound only where it is
    /// definitely beyond it, else left as it is.
    pub fn clamped(self, lo: Self, hi: Self) -> Self {
        if self.definitely_less(lo) {
            lo
        } else if self.definitely_greater(hi) {
            hi
        } else {
            self
        }
    }
}

// ── Serialization ─────────────────────────────────────────────────────────────

/// `#[serde(with = "geop_core_math::scalars::as_f64")]`: a scalar as a plain
/// number — its midpoint written, a sharp scalar read — the way a
/// [`crate::vector::Vector`] serializes, for what travels to and from a
/// viewer.
pub mod as_f64 {
    use super::Scalar;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<Ser: Serializer>(x: &Scalar, serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
        serializer.serialize_f64(x.to_f64())
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Scalar, D::Error> {
        f64::deserialize(deserializer).map(Scalar::from_f64)
    }

    /// `#[serde(with = "geop_core_math::scalars::as_f64::vec")]`: a list of
    /// scalars, each as [`super::as_f64`] has it.
    pub mod vec {
        use super::super::Scalar;
        use serde::{Deserialize, Deserializer, Serializer, ser::SerializeSeq};

        pub fn serialize<Ser: Serializer>(
            xs: &[Scalar],
            serializer: Ser,
        ) -> Result<Ser::Ok, Ser::Error> {
            let mut seq = serializer.serialize_seq(Some(xs.len()))?;
            for x in xs {
                seq.serialize_element(&x.to_f64())?;
            }
            seq.end()
        }

        pub fn deserialize<'de, D: Deserializer<'de>>(
            deserializer: D,
        ) -> Result<Vec<Scalar>, D::Error> {
            Vec::<f64>::deserialize(deserializer)
                .map(|xs| xs.into_iter().map(Scalar::from_f64).collect())
        }
    }

    /// `#[serde(with = "geop_core_math::scalars::as_f64::option")]`: a
    /// scalar that may be missing, as [`super::as_f64`] has it, or `null`.
    pub mod option {
        use super::super::Scalar;
        use serde::{Deserialize, Deserializer, Serializer};

        pub fn serialize<Ser: Serializer>(
            x: &Option<Scalar>,
            serializer: Ser,
        ) -> Result<Ser::Ok, Ser::Error> {
            match x {
                Some(x) => serializer.serialize_some(&x.to_f64()),
                None => serializer.serialize_none(),
            }
        }

        pub fn deserialize<'de, D: Deserializer<'de>>(
            deserializer: D,
        ) -> Result<Option<Scalar>, D::Error> {
            Option::<f64>::deserialize(deserializer).map(|x| x.map(Scalar::from_f64))
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(v: f64) -> Scalar {
        Scalar::degenerate(v)
    }

    fn iv(lo: f64, hi: f64) -> Scalar {
        Scalar::new(lo, hi)
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
        assert!(pt(1.0).div(Scalar::ZERO).is_err());
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
        assert!(pt(-1.0).acos().unwrap().could_be_equal(Scalar::PI));
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
        assert!(Scalar::ENTIRE.could_be_equal(pt(42.0)));
        assert!(pt(-1e300).could_be_equal(Scalar::ENTIRE));
    }

    #[test]
    fn entire_could_be_equal_interval() {
        assert!(Scalar::ENTIRE.could_be_equal(iv(-5.0, 5.0)));
    }

    #[test]
    fn entire_could_be_greater_and_less() {
        assert!(Scalar::ENTIRE.could_be_greater(pt(1e300)));
        assert!(Scalar::ENTIRE.could_be_less(pt(-1e300)));
    }

    #[test]
    fn entire_never_definitely() {
        assert!(!Scalar::ENTIRE.definitely_not_equal(pt(0.0)));
        assert!(!Scalar::ENTIRE.definitely_greater(pt(1e300)));
        assert!(!Scalar::ENTIRE.definitely_less(pt(-1e300)));
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

    /// `atan2` encloses the true angle tightly in every quadrant, on the
    /// axes, and right at the jump on the negative `x` axis.
    #[test]
    fn atan2_encloses_the_angle() {
        let f = Scalar::from_f64;
        for degrees in (-179..=180).step_by(7).chain([0, 90, 180, -90, 45]) {
            let a = (degrees as f64).to_radians();
            let angle = f(a.sin()).atan2(f(a.cos()));
            assert!(angle.could_be_equal(f(a)), "{degrees}: {angle:?}");
            assert!(angle.width().to_f64() < 1e-8, "{degrees}: {angle:?}");
        }
        // pi itself, enclosed.
        let pi = Scalar::ZERO.atan2(f(-1.0));
        assert!(pi.could_be_equal(Scalar::PI), "{pi:?}");
        assert!(pi.width().to_f64() < 1e-8, "{pi:?}");
        // A box across the jump, or round the origin: all angles.
        let across = f(-1e-3).union(f(1e-3)).atan2(f(-1.0));
        assert!(across.could_be_equal(Scalar::PI) && across.could_be_equal(Scalar::PI.neg()));
        let round = f(-1.0).union(f(1.0)).atan2(f(-1.0).union(f(1.0)));
        assert!(round.could_be_equal(Scalar::PI.neg()) && round.could_be_equal(Scalar::PI));
        // A wide box clear of the origin: its corners' angles, enclosed.
        let wide = f(1.0).union(f(2.0)).atan2(f(1.0).union(f(3.0)));
        assert!(wide.could_be_equal(f(1.0f64.atan2(3.0))));
        assert!(wide.could_be_equal(f(2.0f64.atan2(1.0))));
        assert!(!wide.could_be_equal(f(0.3)) && !wide.could_be_equal(f(1.11)));
    }
}
