//! What an interactive CAD editor needs from the kernel beyond the
//! operations themselves: which operations it offers, and the programs
//! written in them ([`operations`]), and example programs ([`examples`]).

pub mod examples;
pub mod operations;

pub use operations::{PartOperation, Program, ProgramEdit, ProgramRunner, Step};
