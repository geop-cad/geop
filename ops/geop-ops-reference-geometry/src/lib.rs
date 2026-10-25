//! Reference geometry as an operation of a program: [`AddDatum`] builds a
//! point, an axis, a plane or a coordinate system from elements picked in
//! the part ([`Selected`]), in one of the ways CAD systems commonly offer
//! (see [`Construction`]), and adds it to the part's datums (see
//! [`geop_ops_model::datums`]). What it is built from is picked as any
//! element of the part ([`geop_ops_model::selection::Selected`]).

mod add_datum;
mod editor;
#[cfg(test)]
mod editor_tests;
pub mod examples;
mod icons;
#[cfg(test)]
mod program_tests;

pub use add_datum::{
    AddDatum, AddDatumArgs, CONSTRUCTIONS, Construction, ConstructionSchema, Param, ParamKind,
    fitting_constructions,
};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: the step adding a datum.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(AddDatum)?.examples(examples::set())
}
