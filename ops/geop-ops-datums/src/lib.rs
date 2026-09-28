//! Reference geometry as an operation of a program (see
//! [`geop_ops::operation`]): [`AddDatum`] builds a point, an axis, a plane
//! or a coordinate system from entities picked in the part, in one of the
//! ways CAD systems commonly offer (see [`Construction`]).

mod add_datum;

pub use add_datum::{
    AddDatum, AddDatumArgs, CONSTRUCTIONS, Construction, SelectionFit, inspect_selection,
};
