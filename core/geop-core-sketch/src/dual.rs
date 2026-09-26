//! Forward-mode automatic differentiation for constraint residuals.
//!
//! Every residual is written once, generically over [`Scalar`] (the same
//! trait the rest of the kernel runs on), and evaluated either with a plain
//! [`Scalar`] value (the value only) or with [`Dual`] (the value plus its
//! exact partial derivatives with respect to the constraint's own
//! variables). A constraint touches only a handful of variables, so a dual
//! number carries a small fixed-size gradient rather than one entry per
//! sketch variable.
//!
//! Building `Dual` over an interval [`Scalar`] rather than a bare `f64`
//! means a residual's value is a rigorous enclosure of the true real number
//! it stands for, not a float that silently absorbs whatever rounding `sin`,
//! `sqrt` or `PI` introduced along the way — the sketch solver's own
//! uncertainty about an irrational quantity shows up as interval width
//! instead of vanishing.

use geop_core_math::{
    geop_error::GeopResult,
    scalars::{Field, Ring, Scalar},
};

/// The most variables a single constraint may depend on. The largest
/// constraints (tangency or equality between two arcs) touch two arcs of
/// five variables each.
pub const MAX_LOCAL_VARS: usize = 10;

/// A value together with its gradient with respect to up to
/// [`MAX_LOCAL_VARS`] seeded variables.
#[derive(Clone, Copy, Debug)]
pub struct Dual<S: Scalar> {
    pub v: S,
    pub d: [S; MAX_LOCAL_VARS],
}

impl<S: Scalar> core::fmt::Display for Dual<S> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(&self.v, f)
    }
}

impl<S: Scalar> Default for Dual<S> {
    fn default() -> Self {
        Dual::cst(S::default())
    }
}

impl<S: Scalar> Dual<S> {
    /// A constant, with no dependence on any variable.
    pub fn cst(v: S) -> Self {
        Dual {
            v,
            d: [S::ZERO; MAX_LOCAL_VARS],
        }
    }

    /// The `slot`-th independent variable, with value `v`.
    pub fn var(v: S, slot: usize) -> Self {
        let mut d = [S::ZERO; MAX_LOCAL_VARS];
        d[slot] = S::ONE;
        Dual { v, d }
    }

    /// `f(self)` for a scalar function with value `v` and derivative `df =
    /// f'(self.v)`: `d <- d * df` by the chain rule.
    fn chain(self, v: S, df: S) -> Self {
        let mut d = self.d;
        for x in &mut d {
            *x = x.mul(df);
        }
        Dual { v, d }
    }
}

impl<S: Scalar> Ring for Dual<S> {
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
        let mut d = [S::ZERO; MAX_LOCAL_VARS];
        for (i, x) in d.iter_mut().enumerate() {
            *x = self.d[i].mul(o.v).add(o.d[i].mul(self.v));
        }
        Dual {
            v: self.v.mul(o.v),
            d,
        }
    }

    fn neg(self) -> Self {
        self.chain(self.v.neg(), S::ZERO.sub(S::ONE))
    }
}

impl<S: Scalar> Field for Dual<S> {
    fn div(self, o: Self) -> GeopResult<Self> {
        let v = self.v.div(o.v)?;
        let mut d = [S::ZERO; MAX_LOCAL_VARS];
        for (i, x) in d.iter_mut().enumerate() {
            // Quotient rule: (self' - v * o') / o.v.
            *x = self.d[i].sub(v.mul(o.d[i])).div(o.v)?;
        }
        Ok(Dual { v, d })
    }
}

// `could_be_equal`/`is_infinite`/etc. below act on the value alone, ignoring
// the gradient — a residual's dual number is compared and reported on the
// same terms a plain [`Scalar`] would be, and these interval-lattice
// operations (`intersect`, `midpoint`, `width`, ...) exist on [`Dual`] only
// because [`Scalar`] requires them, not because a constraint ever calls
// them: the sketch solver only ever adds, multiplies, divides, and takes
// `sqrt`/`sin`/`cos`/`abs` of a residual.
impl<S: Scalar> Scalar for Dual<S> {
    const ZERO: Self = Dual {
        v: S::ZERO,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const ONE: Self = Dual {
        v: S::ONE,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const TWO: Self = Dual {
        v: S::TWO,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const E: Self = Dual {
        v: S::E,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const PI: Self = Dual {
        v: S::PI,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const INFINITY: Self = Dual {
        v: S::INFINITY,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };
    const ENTIRE: Self = Dual {
        v: S::ENTIRE,
        d: [S::ZERO; MAX_LOCAL_VARS],
    };

    fn from_i64(v: i64) -> Self {
        Dual::cst(S::from_i64(v))
    }

    fn from_f64(v: f64) -> Self {
        Dual::cst(S::from_f64(v))
    }

    fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        Ok(Dual::cst(S::from_ratio(num, den)?))
    }

    fn to_f64(self) -> f64 {
        self.v.to_f64()
    }

    fn abs(self) -> Self {
        // Not differentiable at a sign change; 0 is the subgradient (the
        // straddling case falls through to the `+1` branch, matching
        // `Scalar::abs`'s own choice to keep the value non-negative there).
        if self.v.definitely_less(S::ZERO) {
            self.chain(self.v.abs(), S::ZERO.sub(S::ONE))
        } else {
            self.chain(self.v.abs(), S::ONE)
        }
    }

    fn sqrt(self) -> GeopResult<Self> {
        let s = self.v.sqrt()?;
        // `sqrt` is not differentiable at 0. Residuals only take roots of
        // squared lengths, whose gradient vanishes there too, so 0 is the
        // right subgradient: it leaves the other terms in charge.
        let df = if s.definitely_greater(S::ZERO) {
            S::ONE.div(S::TWO.mul(s))?
        } else {
            S::ZERO
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

    fn is_infinite(self) -> bool {
        self.v.is_infinite()
    }

    fn is_finite(self) -> bool {
        self.v.is_finite()
    }

    fn midpoint(self) -> Self {
        Dual {
            v: self.v.midpoint(),
            d: self.d,
        }
    }

    fn is_sharp(self) -> bool {
        self.v.is_sharp()
    }

    fn width(self) -> Self {
        Dual::cst(self.v.width())
    }

    fn lower(self) -> Self {
        Dual {
            v: self.v.lower(),
            d: self.d,
        }
    }

    fn upper(self) -> Self {
        Dual {
            v: self.v.upper(),
            d: self.d,
        }
    }

    fn intersect(self, other: Self) -> Self {
        Dual {
            v: self.v.intersect(other.v),
            d: self.d,
        }
    }

    fn union(self, other: Self) -> Self {
        Dual {
            v: self.v.union(other.v),
            d: self.d,
        }
    }

    fn is_subset_of(self, other: Self) -> bool {
        self.v.is_subset_of(other.v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::scalars::scal_in_f64::ScalInF64;

    /// Every operation's derivative against a central difference.
    #[test]
    fn dual_matches_finite_differences() {
        fn f<S: Scalar>(x: S, y: S) -> S {
            let numerator = x.mul(y).add(x.sin().mul(y.cos()));
            let denom = x.mul(x).add(S::ONE).sqrt().unwrap();
            numerator.div(denom).unwrap().sub(y.sub(x).abs())
        }
        let (x, y) = (0.7, -1.3);
        let d = f(
            Dual::<ScalInF64>::var(ScalInF64::from_f64(x), 0),
            Dual::<ScalInF64>::var(ScalInF64::from_f64(y), 1),
        );
        let h = 1e-6;
        let fv = |x: f64, y: f64| f(ScalInF64::from_f64(x), ScalInF64::from_f64(y)).to_f64();
        let dx = (fv(x + h, y) - fv(x - h, y)) / (2.0 * h);
        let dy = (fv(x, y + h) - fv(x, y - h)) / (2.0 * h);
        assert!((d.v.to_f64() - fv(x, y)).abs() < 1e-15);
        assert!((d.d[0].to_f64() - dx).abs() < 1e-8, "{} vs {dx}", d.d[0]);
        assert!((d.d[1].to_f64() - dy).abs() < 1e-8, "{} vs {dy}", d.d[1]);
    }
}
