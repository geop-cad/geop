//! The [`Shell`] operation: a solid of a program hollowed out to walls of
//! one thickness, open where faces are picked, by [`geop_lib_shell::shell()`].

mod icons;
mod operation;
#[cfg(test)]
mod program_tests;

pub use operation::{Shell, ShellArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: shelling a solid.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Shell)
}
