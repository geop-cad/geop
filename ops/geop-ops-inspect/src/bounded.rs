//! [`Bounded`]: a number as it is shown — a value, and how far the truth
//! may be from it.

use geop_core_math::scalars::Scalar;
use serde::Serialize;

/// An enclosure as it is shown: its midpoint, and its half-width — the
/// truth lies within `error` of `value`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Bounded {
    pub value: f64,
    pub error: f64,
}

impl Bounded {
    /// The enclosure `x`, as shown.
    pub fn of<S: Scalar>(x: S) -> Self {
        let (lo, hi) = (x.lower().to_f64(), x.upper().to_f64());
        Self {
            value: 0.5 * (lo + hi),
            error: 0.5 * (hi - lo),
        }
    }

    /// Whether `exact` lies within the bounds.
    pub fn contains(&self, exact: f64) -> bool {
        (exact - self.value).abs() <= self.error
    }
}
