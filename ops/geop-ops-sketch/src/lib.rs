//! Sketches as an operation of a program: [`AddSketch`] places a sketch on
//! a plane of the part ([`SketchPlane`]), and edits it —
//! drawing with tools that snap by constraints, constraining, dragging (see
//! [`editor`]) — with reference geometry: its own origin and axes, and what
//! it projects of the part (see [`references`]). The sketch it draws is a
//! target of the part ([`geop_ops_model::sketch`]), and what other steps
//! build on.

mod add_sketch;
mod constraints;
pub mod editor;
mod geometry;
mod icons;
pub mod references;

pub use add_sketch::{AddSketch, AddSketchArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what sketches offer an editor to `r`: the step adding one.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(AddSketch)
}
