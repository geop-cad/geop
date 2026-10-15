//! Sketches as an operation of a program: [`AddSketch`] places a sketch on
//! a plane of the part ([`SketchPlane`]), and edits it —
//! drawing with tools that snap by constraints, constraining, dragging (see
//! [`editor`]) — with reference geometry: its own origin and axes, and what
//! it projects of the part (see [`references`]). The sketch it draws is a
//! target of the part ([`placed`]), and what other steps build on.

mod add_sketch;
mod constraints;
pub mod editor;
pub mod fields;
mod geometry;
mod icons;
pub mod placed;
#[cfg(test)]
mod program_tests;
pub mod references;

pub use add_sketch::{AddSketch, AddSketchArgs, SketchPlane};
pub use placed::{
    PlacedSketch, SketchRef, chain_of_pieces, piece_names, sketch, sketch_element, whole_sketch,
};

use geop_core_program::values::Design;

/// A sketch as a program holds it.
pub type Sketch = geop_core_sketch::Sketch<Design>;
/// A sketch constraint as a program holds it.
pub type Constraint = geop_core_sketch::Constraint<Design>;
/// A sketch curve's kind as a program holds it.
pub type CurveKind = geop_core_sketch::CurveKind<Design>;
