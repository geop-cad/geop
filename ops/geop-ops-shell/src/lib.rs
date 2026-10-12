//! Shelling: hollowing a solid out to walls of one thickness, open where
//! faces are taken away ([`shell::shell`]), and the [`Shell`] operation of a
//! program built on it.

mod icons;
pub mod operation;
#[cfg(test)]
mod program_tests;
pub mod shell;

pub use operation::{Shell, ShellArgs};
