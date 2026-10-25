//! What a part says of itself besides its shape: the parameters its design
//! is given by ([`parameters`]) — numbers, and the tables a part family
//! looks its size up in — and its properties — title, designation,
//! colour, material — as steps of its program producing targets
//! ([`steps`]), set by a program placing it through the run's inputs
//! ([`Setting`](geop_core_program::values::Setting)s), and read, set and
//! renamed in a program ([`program`]); where the parts of its instances are
//! placed by it ([`instance`](mod@instance)); and the files a field picks
//! from ([`files`]); the parameters panel of an editor and the colour a
//! part is drawn in ([`panel`]), which [`register`] offers.

pub mod definition;
pub mod files;
mod icons;
pub mod instance;
pub mod panel;
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
pub use program::{
    parameter_numbers, parameter_uses, parameters_of, rename_parameter, set_parameters,
};
pub use steps::{
    DefineParameters, DefineParametersArgs, LookupTable, LookupTableArgs, MaterialOf,
    PartProperties, PartPropertiesArgs, Properties, read_properties, stem,
};

pub use panel::{
    ParametersCommand, ParametersPanel, ParametersPanelState, ParametersState, PartColor,
};

#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod tests;
