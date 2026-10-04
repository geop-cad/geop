//! Assemblies as an operation of a program (see [`geop_ops::operation`]):
//! [`AddPart`] places the part another program file builds, at a pose, and
//! mates it to what is already there — solving every mate of the part
//! anew, which may move the parts placed before it too. Dragging the
//! placed part moves it as far as its mates let it (see [`editor`]).
//! [`PartPattern`] places copies of a placed part in a row or round an
//! axis.

mod add_part;
pub mod editor;
mod part_pattern;

pub use add_part::{AddPart, AddPartArgs};
pub use part_pattern::{Layout, PartPattern, PartPatternArgs};
