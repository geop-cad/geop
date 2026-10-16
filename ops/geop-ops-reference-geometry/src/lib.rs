//! Reference geometry as an operation of a program: [`AddDatum`] builds a
//! point, an axis, a plane or a coordinate system from elements picked in
//! the part ([`Selected`]), in one of the ways CAD systems commonly offer
//! (see [`Construction`]), and adds it to the part's datums (see
//! [`geop_ops_datums`]).
//!
//! [`Selected`] is this crate's public pick vocabulary: one element of a
//! part, picked in the viewport, with the [`Role`] it plays and the
//! [`Shape`] it has. The crate sits above every crate it picks from (the
//! model, the sketches, the datums), so a crate that lets the user pick
//! any element of a part, the assembly's mates for one, reads the pick
//! through `Selected` rather than naming each kind of element itself.

mod add_datum;
mod editor;
pub mod examples;
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
    r.operation(AddDatum)?.examples(examples::set())
}
