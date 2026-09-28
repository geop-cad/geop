//! Extrude and revolve as operations of a program (see
//! [`geop_ops::operation`]): the sketches of a part swept into solids, each
//! kept as a new body or combined with one the part has.

mod extrude;
mod revolve;

pub use extrude::{Extrude, ExtrudeArgs};
pub use revolve::{Revolve, RevolveArgs};
