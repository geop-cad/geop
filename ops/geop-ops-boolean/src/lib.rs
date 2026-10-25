//! The [`Boolean`] and [`Split`] operations of a program: combining two
//! solids of the part, and cutting one into pieces with a face standing on
//! its own. The algorithms are `geop_lib_booleans`'; this crate resolves
//! the solids and faces a step names and offers the forms that pick them.

#[cfg(test)]
mod example_tests;
#[cfg(any(test, feature = "examples"))]
pub mod examples;
mod icons;
mod operation;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod tutorial_regression_tests;

pub use operation::{Boolean, BooleanArgs, Split, SplitArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: combining bodies and
/// splitting them, and the examples of them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let r = r.operation(Boolean)?.operation(Split)?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
