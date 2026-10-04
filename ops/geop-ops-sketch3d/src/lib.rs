//! 3-D sketches as an operation of a program (see [`geop_ops::operation`]):
//! [`AddSketch3d`] draws points, lines, arcs and splines in space — placed
//! on the part's vertices, edges, faces and datums where the pointer hits
//! them, or at typed coordinates — and constrains them (see
//! [`geop_core_sketch::space`]). What it builds is a path for a sweep to run
//! along, or a rail for one to follow.

mod add_sketch3d;
mod editor;
#[cfg(test)]
mod tests;

pub use add_sketch3d::{AddSketch3d, AddSketch3dArgs, Reference3d, Target};
pub use editor::{Sketch3dSession, Tool};

use geop_ops::Design;

/// A 3-D sketch as a program holds it.
pub type Sketch3d = geop_core_sketch::space::Sketch3d<Design>;
/// A 3-D sketch's constraint as a program holds it.
pub type Constraint3d = geop_core_sketch::space::Constraint3d<Design>;
