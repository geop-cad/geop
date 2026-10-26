//! The [`Boolean`], [`Split`], [`Imprint`] and [`Section`] operations of a
//! program: combining two solids of the part, cutting one into pieces with
//! a face standing on its own, dividing two bodies where they cross, and
//! the curves where they cross as a wire. The algorithms are
//! `geop_lib_booleans`'; this crate resolves the bodies, faces and planes a
//! step names and offers the forms that pick them.

#[cfg(test)]
mod editor_tests;
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

pub use operation::{
    Boolean, BooleanArgs, Imprint, ImprintArgs, Section, SectionArgs, SectionWith, Split, SplitArgs,
};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: combining, splitting,
/// imprinting and sectioning bodies, and the examples of them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let r = (r.operation(Boolean)?.operation(Split)?)
        .operation(Imprint)?
        .operation(Section)?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
