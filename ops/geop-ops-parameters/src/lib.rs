//! What a part says of itself besides its shape: the parameters its design
//! is given by ([`parameters`]) and its properties — title, designation,
//! colour, material — as steps of its program defining targets
//! ([`steps`]), set by a program placing it through the run's inputs
//! ([`Setting`]s); its state ([`state`]) and where the parts of its
//! instances are placed by it ([`instance`](mod@instance)); the files a field
//! picks from ([`files`]); and the program as a file holds all of that
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

pub mod files;
pub mod instance;
pub mod parameters;
pub mod program_file;
pub mod state;
pub mod steps;

pub use files::file_choices;
pub use geop_ops_model::Design;
pub use instance::{Factor, Instance, Placement, define_instance, instance, instances, placement};
pub use parameters::{COLOR, Material, Parameter, ParameterKind, Parameters, Setting, Settings};
pub use program_file::{ProgramFile, Runner, StateValues, parameters_of, set_parameters};
pub use state::{PoseOf, StateValue, compose, located, pose_in, pose_state};
pub use steps::{
    DefineParameters, DefineParametersArgs, MaterialOf, PartProperties, PartPropertiesArgs,
    Properties, read_parameter_values, read_properties, stem,
};

/// A part as a program builds it: what its parameters are set to its
/// inputs, its state of [`StateValue`]s.
pub type BuildPart = geop_core_program::part::Part<Setting, StateValue>;

/// A part as a step reads it (see [`geop_core_program::part::Reader`]).
pub type PartReader<'a> = geop_core_program::part::Reader<'a, Setting, StateValue>;

/// What a step of a part is edited against (see
/// [`geop_core_editor::editable::Context`]).
pub type StepContext<'a> = geop_core_editor::editable::Context<'a, Setting, StateValue, Design>;

/// The operations programs of parts are written in.
pub type Registry = geop_core_program::operation_registry::OperationRegistry<Setting, StateValue>;

#[cfg(test)]
mod tests;
