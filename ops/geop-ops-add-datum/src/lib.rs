//! Reference geometry as an operation of a program: [`AddDatum`] builds a
//! point, an axis, a plane or a coordinate system from elements picked in
//! the part ([`Selected`]), in one of the ways CAD systems commonly offer
//! (see [`Construction`]), and adds it to the part's datums (see
//! [`geop_ops_datums`]).

mod add_datum;
mod editor;
mod icons;
#[cfg(test)]
mod program_tests;
pub mod selection;

pub use add_datum::{
    AddDatum, AddDatumArgs, CONSTRUCTIONS, Construction, ConstructionSchema, Param, ParamKind,
    fitting_constructions,
};
pub use selection::{Role, Selected, Shape};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: the step adding a datum.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(AddDatum)
}
