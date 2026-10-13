//! What a part is given rather than built: the parameters its design is
//! given by ([`parameters`]), resolved to the numbers a run reads as its
//! inputs; what its program says of it besides — its file, colour,
//! material, title, the rows of its tables — and the files around it, as
//! targets the run's given step defines ([`about`]); its state
//! ([`state`]); and the program as a file holds all of that
//! ([`program_file`]).

// Library code reports what goes wrong as an error, never by panicking;
// tests may unwrap.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable
    )
)]

pub mod about;
pub mod parameters;
pub mod program_file;
pub mod state;

pub use about::{About, Files, MaterialOf, Shared, about, files, read_about, read_files};
pub use geop_ops_model::Design;
pub use parameters::{COLOR, Material, Parameter, ParameterKind, Parameters, Setting, Settings};
pub use program_file::{ProgramFile, Runner, StateValues};
pub use state::{StateValue, pose_state};

/// A part as a program builds it: numbers its inputs, its state of
/// [`StateValue`]s.
pub type BuildPart = geop_core_program::part::Part<Design, StateValue>;

/// A part as a step reads it (see [`geop_core_program::part::Reader`]).
pub type PartReader<'a> = geop_core_program::part::Reader<'a, Design, StateValue>;

/// What a step of a part is edited against (see
/// [`geop_core_editor::editable::Context`]).
pub type StepContext<'a> = geop_core_editor::editable::Context<'a, Design, StateValue, Design>;

/// The operations programs of parts are written in.
pub type Registry = geop_core_program::operation_registry::OperationRegistry<Design, StateValue>;

#[cfg(test)]
mod tests;
