use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::part::State;

pub use geop_core_math::solvers::system::Bound;

/// What an operation requires of the state: residuals that are zero — or,
/// for a limit, at most zero — where it holds, each a length, and how they
/// change as the values of the state they depend on move.
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint<S: State> {
    pub residuals: Vec<S::Scalar>,
    /// For each state the residuals depend on, one row per residual: its
    /// derivative by that state's step (see
    /// [`geop_core_math::solvers::system::Variable::retract`]), as many
    /// numbers as the state's dimension.
    pub jacobian: BTreeMap<String, Vec<Vec<S::Scalar>>>,
    pub bound: Bound,
}

impl<S: State> Constraint<S> {
    /// Checks that the jacobian has a row per residual, of as many numbers
    /// as a step of the state takes, for states the constraint read —
    /// `dimension` saying how many, `None` for one it did not read: a
    /// derivative by a state it never looked at cannot be right.
    pub(crate) fn check(
        &self,
        name: &str,
        dimension: impl Fn(&str) -> Option<usize>,
    ) -> GeopResult<()> {
        for (state, rows) in &self.jacobian {
            let Some(columns) = dimension(state) else {
                return Err(GeopError::new(format!(
                    "constraint `{name}` gives a derivative by state `{state}`, which it does not read"
                )));
            };
            if rows.len() != self.residuals.len() {
                return Err(GeopError::new(format!(
                    "constraint `{name}` has {} residuals but {} rows of derivatives by state `{state}`",
                    self.residuals.len(),
                    rows.len()
                )));
            }
            if let Some(row) = rows.iter().find(|row| row.len() != columns) {
                return Err(GeopError::new(format!(
                    "constraint `{name}` gives a derivative of {} numbers by state `{state}`, whose step takes {columns}",
                    row.len()
                )));
            }
        }
        Ok(())
    }

    /// Whether it holds: every residual within `tolerance` of zero — or,
    /// for a limit, not definitely above `tolerance`.
    pub fn holds(&self, tolerance: S::Scalar) -> bool {
        (self.residuals.iter()).all(|&r| self.bound.holds(r, tolerance))
    }
}
