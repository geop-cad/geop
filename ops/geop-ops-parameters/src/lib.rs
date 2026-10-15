//! What a part says of itself besides its shape: the parameters its design
//! is given by ([`parameters`]) — numbers, and the tables a part family
//! looks its size up in — and its properties — title, designation,
//! colour, material — as steps of its program producing targets
//! ([`steps`]), set by a program placing it through the run's inputs
//! ([`Setting`](geop_core_program::values::Setting)s), and read, set and
//! renamed in a program ([`program`]); where the parts of its instances are
//! placed by it ([`instance`](mod@instance)); and the files a field picks
//! from ([`files`]).

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
pub mod program;
pub mod steps;

pub use files::file_choices;
pub use instance::{
    Factor, Instance, Placement, PoseOf, compose, define_instance, instance, instances, located,
    placement,
};
pub use parameters::{COLOR, Material, NumberParameter, Parameter, ParameterKind, Parameters};
pub use program::{
    parameter_numbers, parameter_uses, parameters_of, rename_parameter, set_parameters,
};
pub use steps::{
    DefineParameters, DefineParametersArgs, LookupTable, LookupTableArgs, MaterialOf,
    PartProperties, PartPropertiesArgs, Properties, read_properties, stem,
};

#[cfg(test)]
mod tests;
