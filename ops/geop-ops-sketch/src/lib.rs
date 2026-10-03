//! Sketches as an operation of a program (see [`geop_ops::operation`]):
//! [`AddSketch`] places a sketch on a plane of the part, and edits it —
//! drawing with tools that snap by constraints, constraining, dragging (see
//! [`editor`]).

mod add_sketch;
mod constraints;
pub mod editor;
mod geometry;

pub use add_sketch::{AddSketch, AddSketchArgs};

use geop_ops::Design;

/// A sketch as a program holds it.
pub type Sketch = geop_core_sketch::Sketch<Design>;
/// A sketch constraint as a program holds it.
pub type Constraint = geop_core_sketch::Constraint<Design>;
/// A sketch curve's kind as a program holds it.
pub type CurveKind = geop_core_sketch::CurveKind<Design>;
