//! Forward-mode automatic differentiation, for the residuals of the
//! constraint solvers (sketches, assemblies).
//!
//! Every residual is written once, generically over [`Number`], and
//! evaluated either with a plain [`Scalar`] value (the value only) or with
//! [`Dual`] (the value plus its exact partial derivatives with respect to
//! the constraint's own variables). A constraint touches only a handful of variables, so a dual
//! number carries a small fixed-size gradient rather than one entry per
//! solver variable.
//!
//! Building `Dual` over the interval [`Scalar`] rather than a bare `f64`
//! means a residual's value is a rigorous enclosure of the true real number
//! it stands for, not a float that silently absorbs whatever rounding `sin`,
//! `sqrt` or `PI` introduced along the way — the sketch solver's own
//! uncertainty about an irrational quantity shows up as interval width
//! instead of vanishing.

use crate::{
    geop_error::GeopResult,
    scalars::{Number, Scalar},
};

/// A value together with its gradient with respect to up to `N` seeded
/// variables — as many as the residual it is computed for depends on.
#[derive(Clone, Copy, Debug)]
pub struct Dual<const N: usize> {
    pub v: Scalar,
    pub d: [Scalar; N],
}

impl<const N: usize> core::fmt::Display for Dual<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(&self.v, f)
    }
}

impl<const N: usize> Default for Dual<N> {
    fn default() -> Self {
        Dual::cst(Scalar::ZERO)
    }
}

impl<const N: usize> Dual<N> {
    /// A constant, with no dependence on any variable.
    pub const fn cst(v: Scalar) -> Self {
        Dual {
            v,
            d: [Scalar::ZERO; N],
        }
    }

    /// The `slot`-th independent variable, with value `v`.
    pub fn var(v: Scalar, slot: usize) -> Self {
        let mut d = [Scalar::ZERO; N];
        d[slot] = Scalar::ONE;
        Dual { v, d }
    }

    /// The same value in a gradient of `M` variables: its own first `vars`
    /// the ones from `slot` on — and no dependence on any other (its own
    /// beyond `vars` it has none of).
    pub fn embed<const M: usize>(self, slot: usize, vars: usize) -> Dual<M> {
        let mut d = [Scalar::ZERO; M];
        d[slot..slot + vars].copy_from_slice(&self.d[..vars]);
        Dual { v: self.v, d }
    }

    /// `f(self)` for a scalar function with value `v` and derivative `df =
    /// f'(self.v)`: `d <- d * df` by the chain rule.
    pub fn chain(self, v: Scalar, df: Scalar) -> Self {
        let mut d = self.d;
        for x in &mut d {
            *x = x.mul(df);
        }
        Dual { v, d }
    }
}

impl<const N: usize> Number for Dual<N> {
    const ZERO: Self = Self::cst(Scalar::ZERO);
    const ONE: Self = Self::cst(Scalar::ONE);
    const TWO: Self = Self::cst(Scalar::TWO);
    const PI: Self = Self::cst(Scalar::PI);

    fn from_i64(v: i64) -> Self {
        Dual::cst(Scalar::from_i64(v))
    }

    fn from_f64(v: f64) -> Self {
        Dual::cst(Scalar::from_f64(v))
    }

    fn from_scalar(s: Scalar) -> Self {
        Dual::cst(s)
    }

    fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        Ok(Dual::cst(Scalar::from_ratio(num, den)?))
    }

    fn to_f64(self) -> f64 {
        self.v.to_f64()
    }

    fn add(self, o: Self) -> Self {
        let mut d = self.d;
        for (x, y) in d.iter_mut().zip(o.d) {
            *x = x.add(y);
        }
        Dual {
            v: self.v.add(o.v),
            d,
        }
    }

    fn sub(self, o: Self) -> Self {
        let mut d = self.d;
        for (x, y) in d.iter_mut().zip(o.d) {
            *x = x.sub(y);
        }
        Dual {
            v: self.v.sub(o.v),
            d,
        }
    }

    fn mul(self, o: Self) -> Self {
        let mut d = [Scalar::ZERO; N];
        for (i, x) in d.iter_mut().enumerate() {
            *x = self.d[i].mul(o.v).add(o.d[i].mul(self.v));
        }
        Dual {
            v: self.v.mul(o.v),
            d,
        }
    }

    fn neg(self) -> Self {
        self.chain(self.v.neg(), Scalar::ZERO.sub(Scalar::ONE))
    }

    fn div(self, o: Self) -> GeopResult<Self> {
        let v = self.v.div(o.v)?;
        let mut d = [Scalar::ZERO; N];
        for (i, x) in d.iter_mut().enumerate() {
            // Quotient rule: (self' - v * o') / o.v.
            *x = self.d[i].sub(v.mul(o.d[i])).div(o.v)?;
        }
        Ok(Dual { v, d })
    }

    // `could_be_equal`/`is_infinite`/etc. below act on the value alone,
    // ignoring the gradient — a residual's dual number is compared and
    // reported on the same terms a plain [`Scalar`] would be, and the
    // interval-lattice operations (`intersect`, `midpoint`, `width`, ...)
    // exist on [`Dual`] only because [`Number`] requires them, not because
    // a constraint ever calls them: the sketch solver only ever adds,
    // multiplies, divides, and takes `sqrt`/`sin`/`cos`/`abs` of a residual.

    fn abs(self) -> Self {
        // Not differentiable at a sign change; 0 is the subgradient (the
        // straddling case falls through to the `+1` branch, matching
        // `Scalar::abs`'s own choice to keep the value non-negative there).
        if self.v.definitely_less(Scalar::ZERO) {
            self.chain(self.v.abs(), Scalar::ZERO.sub(Scalar::ONE))
        } else {
            self.chain(self.v.abs(), Scalar::ONE)
        }
    }

    fn sqrt(self) -> GeopResult<Self> {
        let s = self.v.sqrt()?;
        // `sqrt` is not differentiable at 0. Residuals only take roots of
        // squared lengths, whose gradient vanishes there too, so 0 is the
        // right subgradient: it leaves the other terms in charge.
        let df = if s.definitely_greater(Scalar::ZERO) {
            Scalar::ONE.div(Scalar::TWO.mul(s))?
        } else {
            Scalar::ZERO
        };
        Ok(self.chain(s, df))
    }

    fn sin(self) -> Self {
        self.chain(self.v.sin(), self.v.cos())
    }

    fn cos(self) -> Self {
        let df = self.v.sin().neg();
        self.chain(self.v.cos(), df)
    }

    /// `d acos(x) = -1 / sqrt(1 - x^2)`, which fails where it is infinite,
    /// at `x = ±1`.
    fn acos(self) -> GeopResult<Self> {
        let root = Scalar::ONE.sub(self.v.mul(self.v)).sqrt()?;
        let df = Scalar::ONE.div(root)?.neg();
        Ok(self.chain(self.v.acos()?, df))
    }

    fn could_be_equal(self, other: Self) -> bool {
        self.v.could_be_equal(other.v)
    }

    fn definitely_not_equal(self, other: Self) -> bool {
        self.v.definitely_not_equal(other.v)
    }

    fn could_be_greater(self, other: Self) -> bool {
        self.v.could_be_greater(other.v)
    }

    fn definitely_greater(self, other: Self) -> bool {
        self.v.definitely_greater(other.v)
    }

    fn could_be_less(self, other: Self) -> bool {
        self.v.could_be_less(other.v)
    }

    fn definitely_less(self, other: Self) -> bool {
        self.v.definitely_less(other.v)
    }

    /// A single, exactly known point: a sharp value that does not vary
    /// with any variable. A value that is sharp but moves with a variable is
    /// no point — taking it for one drops the gradient.
    fn is_sharp(self) -> bool {
        self.v.is_sharp() && self.d.iter().all(|g| g.is_zero())
    }

    fn is_exactly(self, other: Self) -> bool {
        self.is_sharp() && other.is_sharp() && self.v.is_exactly(other.v)
    }

    /// The union of the values, with `self`'s derivatives: the width a
    /// series' remainder adds (see [`crate::primitives::Pose::exp`]).
    fn union(self, other: Self) -> Self {
        Dual {
            v: self.v.union(other.v),
            d: self.d,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every operation's derivative against a central difference.
    #[test]
    fn dual_matches_finite_differences() {
        fn f<T: Number>(x: T, y: T) -> T {
            let numerator = x.mul(y).add(x.sin().mul(y.cos()));
            let denom = x.mul(x).add(T::ONE).sqrt().unwrap();
            numerator.div(denom).unwrap().sub(y.sub(x).abs())
        }
        let (x, y) = (0.7, -1.3);
        let d = f(
            Dual::<2>::var(Scalar::from_f64(x), 0),
            Dual::<2>::var(Scalar::from_f64(y), 1),
        );
        let h = 1e-6;
        let fv = |x: f64, y: f64| f(Scalar::from_f64(x), Scalar::from_f64(y)).to_f64();
        let dx = (fv(x + h, y) - fv(x - h, y)) / (2.0 * h);
        let dy = (fv(x, y + h) - fv(x, y - h)) / (2.0 * h);
        assert!((d.v.to_f64() - fv(x, y)).abs() < 1e-15);
        assert!((d.d[0].to_f64() - dx).abs() < 1e-8, "{} vs {dx}", d.d[0]);
        assert!((d.d[1].to_f64() - dy).abs() < 1e-8, "{} vs {dy}", d.d[1]);
    }
}
