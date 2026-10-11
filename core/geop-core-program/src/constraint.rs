use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::part::State;

/// What an operation requires of the state: residuals that are zero where
/// it holds, and how they change as the values of the state they depend on
/// move.
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint<S: State> {
    pub residuals: Vec<S::Scalar>,
    /// For each state the residuals depend on, one row per residual: its
    /// derivative by that state, a tangent of it.
    pub jacobian: BTreeMap<String, Vec<S::Tangent>>,
}

impl<S: State> Constraint<S> {
    /// Checks that the jacobian has a row per residual, for states the
    /// constraint read: a derivative by a state it never looked at cannot be
    /// right.
    pub(crate) fn check(&self, name: &str, mut read: impl FnMut(&str) -> bool) -> GeopResult<()> {
        for (state, rows) in &self.jacobian {
            if !read(state) {
                return Err(GeopError::new(format!(
                    "constraint `{name}` gives a derivative by state `{state}`, which it does not read"
                )));
            }
            if rows.len() != self.residuals.len() {
                return Err(GeopError::new(format!(
                    "constraint `{name}` has {} residuals but {} rows of derivatives by state `{state}`",
                    self.residuals.len(),
                    rows.len()
                )));
            }
        }
        Ok(())
    }
}
