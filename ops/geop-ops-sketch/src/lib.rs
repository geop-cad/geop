//! Sketches as an operation of a program (see [`geop_ops::operation`]):
//! [`AddSketch`] places a sketch on a plane of the part, and edits it —
//! drawing with tools that snap by constraints, constraining, dragging (see
//! [`editor`]).

mod add_sketch;
mod constraints;
pub mod editor;
mod geometry;

pub use add_sketch::{AddSketch, AddSketchArgs};
