pub mod scal_in_f64;
pub mod scal_in_fpa64;

use core::fmt::Display;

pub use scal_in_f64::ScalInF64;
pub use scal_in_fpa64::ScalInFPA64;

use crate::geop_error::GeopResult;

// ── Algebraic traits ─────────────────────────────────────────────────────────

pub trait Ring: Clone + core::fmt::Debug + Send + Sync + 'static {
    fn add(self, other: Self) -> Self;
    fn sub(self, other: Self) -> Self;
    fn mul(self, other: Self) -> Self;
    fn neg(self) -> Self;
}

pub trait Field: Ring {
    fn div(self, other: Self) -> GeopResult<Self>;
}

// ── Core trait ────────────────────────────────────────────────────────────────
pub trait Scalar: Field + Copy + Display + Default {
    // Constants
    const ZERO: Self;
    const ONE: Self;
    const TWO: Self;
    const E: Self;
    const PI: Self;
    /// Saturation sentinel — set on overflow.
    const INFINITY: Self;
    /// The "entire" interval `(-inf, inf)` — the top element of the interval
    /// lattice. `could_be_equal`/`could_be_greater`/`could_be_less` against
    /// it are always `true`, and it never satisfies `definitely_*`. Used to
    /// represent a value or a whole curve/surface whose position is not yet
    /// known — an unsharp placeholder that automatically passes any
    /// overlap/equality check made against it.
    const ENTIRE: Self;

    // Construction
    fn from_i64(v: i64) -> Self;
    fn from_f64(v: f64) -> Self;
    fn from_ratio(num: i64, den: i64) -> GeopResult<Self>;

    /// Approximate f64 midpoint. For point scalars returns the value; for
    /// interval scalars returns (lo + hi) / 2. Used only for rendering/debugging.
    fn to_f64(self) -> f64;

    // Real-valued operations
    fn abs(self) -> Self;
    fn sqrt(self) -> GeopResult<Self>;
    /// Outward-rounded enclosure of `sin`/`cos` over the whole interval
    /// (radians). Total — never fails, even for [`Scalar::ENTIRE`] or an
    /// [`Scalar::INFINITY`]-adjacent value, which just widen to `[-1, 1]`.
    fn sin(self) -> Self;
    fn cos(self) -> Self;

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
    fn atan2(self, x: Self) -> Self {
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

    // Three-valued comparisons
    fn could_be_equal(self, other: Self) -> bool;
    fn definitely_not_equal(self, other: Self) -> bool;
    fn could_be_greater(self, other: Self) -> bool;
    fn definitely_greater(self, other: Self) -> bool;
    fn could_be_less(self, other: Self) -> bool;
    fn definitely_less(self, other: Self) -> bool;

    // Finiteness
    fn is_infinite(self) -> bool;
    fn is_finite(self) -> bool;

    // Set-valued helpers
    fn midpoint(self) -> Self;

    /// True iff this value carries no width — it's a single, exactly-known
    /// point, not a genuine range of possibility.
    fn is_sharp(self) -> bool;

    /// How much possibility this enclosure carries: `hi - lo`, as a **sharp,
    /// non-negative** value. Zero exactly when [`Scalar::is_sharp`].
    ///
    /// This is how much a computed quantity is *not* known. Being sharp
    /// itself is what makes it usable as a threshold — comparing an uncertain
    /// width against an uncertain bound could never be decided three-valuedly
    /// (see `validation::numerical_accuracy`).
    fn width(self) -> Self;

    /// The sharp lower / upper endpoint of this enclosure. Every value
    /// `self` could be is `>= lower()` and `<= upper()`, so these are the
    /// *outer* bounds to cut at when a search restricts a domain to an
    /// enclosure of its answer: a cut there never loses a solution (unlike
    /// [`Scalar::sharpen`], which would cut through the enclosure).
    fn lower(self) -> Self;
    fn upper(self) -> Self;

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
    fn sharpen(self) -> Self {
        self.midpoint()
    }

    /// Point a fraction `alpha` of the way from `a` to `b`: `a` at
    /// `alpha=0`, `b` at `alpha=1`.
    ///
    /// Deliberately `a.add(alpha.mul(b.sub(a)))`, *not* the equally-valid
    /// `a.mul(S::ONE.sub(alpha)).add(b.mul(alpha))` — both give the same
    /// exact real result, but the latter computes `alpha` and `1-alpha` as
    /// two *decorrelated* intervals before ever relating `a` and `b`, so
    /// interval arithmetic can't recognize when they cancel. This form
    /// computes `b.sub(a)` first: when `a` and `b` are honestly the same
    /// value (e.g. a weight that should stay exactly `1.0` across many
    /// subdivisions), that subtraction is exactly `0` regardless of
    /// `alpha`'s own width, and the whole expression collapses to exactly
    /// `a` instead of needlessly widening with every call.
    fn interpolate(a: Self, b: Self, alpha: Self) -> Self {
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
    /// one. Implementations may return either input if the two somehow
    /// don't overlap, rather than fabricating an empty/inverted interval.
    fn intersect(self, other: Self) -> Self;

    /// The smallest value definitely containing both `self` and `other` —
    /// the scalar-level analog of `Set::union`.
    fn union(self, other: Self) -> Self;

    /// True iff `self` is contained in `other` as sets: `other.lo <= self.lo`
    /// and `self.hi <= other.hi`. This is the rigorous existence/uniqueness
    /// test a Krawczyk-style contraction relies on (`K(X) ⊆ X`) — distinct
    /// from [`Scalar::could_be_equal`], which only asks whether the two
    /// enclosures *overlap*. `self.intersect(other).could_be_equal(self)`
    /// would answer the same question but at the cost of rebuilding an
    /// enclosure just to throw it away; implementations should compare
    /// bounds directly.
    fn is_subset_of(self, other: Self) -> bool;

    /// The larger of the two: an enclosure of `max(a, b)` for every `a` and
    /// `b` the two could be — sharp where both are.
    fn max(self, other: Self) -> Self {
        let pick = |a: Self, b: Self| if b.definitely_greater(a) { b } else { a };
        pick(self.lower(), other.lower()).union(pick(self.upper(), other.upper()))
    }

    /// The smaller of the two (see [`Scalar::max`]).
    fn min(self, other: Self) -> Self {
        self.neg().max(other.neg()).neg()
    }

    /// The same value in the scalar type `T`: the smallest enclosure there
    /// of everything `self` could be — exactly `self` where `T` can hold it.
    fn cast<T: Scalar>(self) -> T {
        T::from_f64(self.lower().to_f64()).union(T::from_f64(self.upper().to_f64()))
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

    pub fn serialize<S: Scalar, Ser: Serializer>(
        x: &S,
        serializer: Ser,
    ) -> Result<Ser::Ok, Ser::Error> {
        serializer.serialize_f64(x.to_f64())
    }

    pub fn deserialize<'de, S: Scalar, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<S, D::Error> {
        f64::deserialize(deserializer).map(S::from_f64)
    }

    /// `#[serde(with = "geop_core_math::scalars::as_f64::vec")]`: a list of
    /// scalars, each as [`super::as_f64`] has it.
    pub mod vec {
        use super::super::Scalar;
        use serde::{Deserialize, Deserializer, Serializer, ser::SerializeSeq};

        pub fn serialize<S: Scalar, Ser: Serializer>(
            xs: &[S],
            serializer: Ser,
        ) -> Result<Ser::Ok, Ser::Error> {
            let mut seq = serializer.serialize_seq(Some(xs.len()))?;
            for x in xs {
                seq.serialize_element(&x.to_f64())?;
            }
            seq.end()
        }

        pub fn deserialize<'de, S: Scalar, D: Deserializer<'de>>(
            deserializer: D,
        ) -> Result<Vec<S>, D::Error> {
            Vec::<f64>::deserialize(deserializer)
                .map(|xs| xs.into_iter().map(S::from_f64).collect())
        }
    }
}

// ── Test helper trait ─────────────────────────────────────────────────────────

/// Invoke a generic test function once for each concrete scalar
/// implementation.  Inside a `#[test]` function, write:
///
/// ```rust,ignore
/// fn my_check<S: Scalar + ScalarTestHelper>() { /* … */ }
/// #[test] fn my_test() { for_all_scalars!(my_check); }
/// ```
#[macro_export]
macro_rules! for_all_scalars {
    ($fn:ident) => {{
        $fn::<$crate::scalars::ScalInF64>();
        $fn::<$crate::scalars::ScalInFPA64>();
    }};
}

#[cfg(test)]
mod tests {
    use super::Scalar;

    /// `atan2` encloses the true angle tightly in every quadrant, on the
    /// axes, and right at the jump on the negative `x` axis.
    fn check_atan2_encloses_the_angle<S: Scalar>() {
        let f = S::from_f64;
        for degrees in (-179..=180).step_by(7).chain([0, 90, 180, -90, 45]) {
            let a = (degrees as f64).to_radians();
            let angle = f(a.sin()).atan2(f(a.cos()));
            assert!(angle.could_be_equal(f(a)), "{degrees}: {angle:?}");
            // Fixed point resolves 2^-32, plain intervals far finer.
            assert!(angle.width().to_f64() < 1e-8, "{degrees}: {angle:?}");
        }
        // pi itself, enclosed.
        let pi = S::ZERO.atan2(f(-1.0));
        assert!(pi.could_be_equal(S::PI), "{pi:?}");
        assert!(pi.width().to_f64() < 1e-8, "{pi:?}");
        // A box across the jump, or round the origin: all angles.
        let across = f(-1e-3).union(f(1e-3)).atan2(f(-1.0));
        assert!(across.could_be_equal(S::PI) && across.could_be_equal(S::PI.neg()));
        let round = f(-1.0).union(f(1.0)).atan2(f(-1.0).union(f(1.0)));
        assert!(round.could_be_equal(S::PI.neg()) && round.could_be_equal(S::PI));
        // A wide box clear of the origin: its corners' angles, enclosed.
        let wide = f(1.0).union(f(2.0)).atan2(f(1.0).union(f(3.0)));
        assert!(wide.could_be_equal(f(1.0f64.atan2(3.0))));
        assert!(wide.could_be_equal(f(2.0f64.atan2(1.0))));
        assert!(!wide.could_be_equal(f(0.3)) && !wide.could_be_equal(f(1.11)));
    }
    #[test]
    fn atan2_encloses_the_angle() {
        for_all_scalars!(check_atan2_encloses_the_angle);
    }
}
