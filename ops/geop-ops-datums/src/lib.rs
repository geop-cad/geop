//! Reference geometry as an operation of a program (see
//! [`geop_ops::operation`]): [`AddDatum`] builds a point, an axis, a plane
//! or a coordinate system from entities picked in the part, in one of the
//! ways CAD systems commonly offer (see [`Construction`]).

mod add_datum;
mod editor;
mod geometry;

pub use add_datum::{
    AddDatum, AddDatumArgs, CONSTRUCTIONS, Construction, ConstructionSchema, Param, ParamKind,
    SelectionFit, inspect_selection,
};
pub use geometry::{Geometry, Role};
