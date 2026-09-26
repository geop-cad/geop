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
