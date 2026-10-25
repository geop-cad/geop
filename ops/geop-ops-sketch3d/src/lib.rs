//! 3-D sketches as an operation of a program: [`AddSketch3d`] draws points, lines, arcs and splines in space — placed
//! on the part's vertices, edges, faces and datums where the pointer hits
//! them, or at typed coordinates — and constrains them (see
//! [`geop_core_sketch::space`]). What it builds is a path for a sweep to run
//! along, or a rail for one to follow: the sketch is a target of the part
//! ([`geop_ops_model::sketch3d`]), a path as a whole.

mod add_sketch3d;
mod editor;
mod icons;
#[cfg(test)]
mod regression_tests;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
#[cfg(test)]
mod tests;

pub use add_sketch3d::{AddSketch3d, AddSketch3dArgs, Reference3d, Sketch3dEditing, Target};
pub use editor::{Sketch3dSession, Tool};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: the step adding a 3-D
/// sketch.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(AddSketch3d)
}
