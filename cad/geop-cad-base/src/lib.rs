//! What an interactive CAD editor needs from the kernel beyond the
//! operations themselves: which operations it offers, and the programs
//! written in them ([`operations`]); example programs ([`examples`]); and
//! picking entities in the viewport ([`pick`]).

pub mod examples;
pub mod operations;
pub mod pick;

pub use operations::{PartOperation, Program, ProgramEdit, ProgramRunner, Step};
