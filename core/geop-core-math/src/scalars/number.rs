//! What a residual of the constraint solvers is written in.

use core::fmt::{Debug, Display};

use super::Scalar;
use crate::geop_error::GeopResult;

/// A number a residual computes with: a [`Scalar`], giving the residual's
/// value, or a [`crate::dual::Dual`] of one, giving its value and its
/// derivatives by the variables it depends on. Every residual is written
/// once, over `T: Number`, and evaluated with either. The comparisons act
/// on the value alone.
pub trait Number: Copy + Clone + Debug + Display + Default + Send + Sync + 'static {
    const ZERO: Self;
    const ONE: Self;
    const TWO: Self;
    const PI: Self;

    fn from_i64(v: i64) -> Self;
    fn from_f64(v: f64) -> Self;
    /// The scalar `s` as a constant: depending on no variable.
    fn from_scalar(s: Scalar) -> Self;
    fn from_ratio(num: i64, den: i64) -> GeopResult<Self>;
    /// The value's midpoint, for rendering and debugging.
    fn to_f64(self) -> f64;

    fn add(self, other: Self) -> Self;
    fn sub(self, other: Self) -> Self;
    fn mul(self, other: Self) -> Self;
    fn neg(self) -> Self;
    /// Fails where the divisor could be zero.
    fn div(self, other: Self) -> GeopResult<Self>;

    fn abs(self) -> Self;
    fn sqrt(self) -> GeopResult<Self>;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn acos(self) -> GeopResult<Self>;

    /// The angle `self`, given in degrees, in radians.
    fn degrees_to_radians(self) -> GeopResult<Self> {
        self.mul(Self::PI).div(Self::from_i64(180))
    }

    fn could_be_equal(self, other: Self) -> bool;
    fn definitely_not_equal(self, other: Self) -> bool;
    fn could_be_greater(self, other: Self) -> bool;
    fn definitely_greater(self, other: Self) -> bool;
    fn could_be_less(self, other: Self) -> bool;
    fn definitely_less(self, other: Self) -> bool;

    /// Whether it is a single, exactly known point: a value that depends
    /// on no variable, and carries no width.
    fn is_sharp(self) -> bool;
    /// Whether its value is exactly `other`'s, as a set (see
    /// [`Scalar::is_exactly`]).
    fn is_exactly(self, other: Self) -> bool;
    /// The smallest value containing both (see [`Scalar::union`]).
    fn union(self, other: Self) -> Self;
}

impl Number for Scalar {
    const ZERO: Self = Scalar::ZERO;
    const ONE: Self = Scalar::ONE;
    const TWO: Self = Scalar::TWO;
    const PI: Self = Scalar::PI;

    fn from_i64(v: i64) -> Self {
        Scalar::from_i64(v)
    }
    fn from_f64(v: f64) -> Self {
        Scalar::from_f64(v)
    }
    fn from_scalar(s: Scalar) -> Self {
        s
    }
    fn from_ratio(num: i64, den: i64) -> GeopResult<Self> {
        Scalar::from_ratio(num, den)
    }
    fn to_f64(self) -> f64 {
        Scalar::to_f64(self)
    }

    fn add(self, other: Self) -> Self {
        Scalar::add(self, other)
    }
    fn sub(self, other: Self) -> Self {
        Scalar::sub(self, other)
    }
    fn mul(self, other: Self) -> Self {
        Scalar::mul(self, other)
    }
    fn neg(self) -> Self {
        Scalar::neg(self)
    }
    fn div(self, other: Self) -> GeopResult<Self> {
        Scalar::div(self, other)
    }

    fn abs(self) -> Self {
        Scalar::abs(self)
    }
    fn sqrt(self) -> GeopResult<Self> {
        Scalar::sqrt(self)
    }
    fn sin(self) -> Self {
        Scalar::sin(self)
    }
    fn cos(self) -> Self {
        Scalar::cos(self)
    }
    fn acos(self) -> GeopResult<Self> {
        Scalar::acos(self)
    }
    fn degrees_to_radians(self) -> GeopResult<Self> {
        Scalar::degrees_to_radians(self)
    }

    fn could_be_equal(self, other: Self) -> bool {
        Scalar::could_be_equal(self, other)
    }
    fn definitely_not_equal(self, other: Self) -> bool {
        Scalar::definitely_not_equal(self, other)
    }
    fn could_be_greater(self, other: Self) -> bool {
        Scalar::could_be_greater(self, other)
    }
    fn definitely_greater(self, other: Self) -> bool {
        Scalar::definitely_greater(self, other)
    }
    fn could_be_less(self, other: Self) -> bool {
        Scalar::could_be_less(self, other)
    }
    fn definitely_less(self, other: Self) -> bool {
        Scalar::definitely_less(self, other)
    }

    fn is_sharp(self) -> bool {
        Scalar::is_sharp(self)
    }
    fn is_exactly(self, other: Self) -> bool {
        Scalar::is_exactly(self, other)
    }
    fn union(self, other: Self) -> Self {
        Scalar::union(self, other)
    }
}
