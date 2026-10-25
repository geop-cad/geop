//! What a part says of itself besides its shape: the parameters its design
//! is given by ([`definition`]) — numbers, and the tables a part family
//! looks its size up in — and its properties — title, designation,
//! colour, material — as steps of its program producing targets
//! ([`steps`]), set by a program placing it through the run's inputs
//! ([`Setting`](geop_core_program::values::Setting)s), and read and set in
//! a program ([`program`]); where the parts of its instances are
//! placed by it ([`instance`](mod@instance)); and the files a field picks
//! from ([`files`]).

pub mod definition;
pub mod files;
mod icons;
pub mod instance;
pub mod program;
pub mod steps;

pub use definition::{
    COLOR, Material, NumberParameter, Parameter, ParameterKind, Parameters, Row, validate_color,
};
pub use files::file_choices;
pub use instance::{
    Factor, Instance, Placement, PoseOf, compose, define_instance, instance, instances, located,
    placement,
};
pub use program::{parameter_numbers, parameters_of, set_parameters};
pub use steps::{
    DefineParameters, DefineParametersArgs, LookupTable, LookupTableArgs, PartColor,
    PartProperties, PartPropertiesArgs, Properties, read_properties, stem,
};

#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod tests;
