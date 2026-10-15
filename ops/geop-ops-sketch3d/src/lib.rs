//! 3-D sketches as an operation of a program: [`AddSketch3d`] draws points, lines, arcs and splines in space — placed
//! on the part's vertices, edges, faces and datums where the pointer hits
//! them, or at typed coordinates — and constrains them (see
//! [`geop_core_sketch::space`]). What it builds is a path for a sweep to run
//! along, or a rail for one to follow: the sketch is a target of the part
//! ([`placed`]), a path as a whole.

mod add_sketch3d;
mod editor;
mod icons;
pub mod placed;
#[cfg(test)]
mod tests;

pub use add_sketch3d::{
    AddSketch3d, AddSketch3dArgs, CurveSource, LineSource, PointSource, Reference3d,
    Sketch3dEditing, Target,
};
pub use editor::{Sketch3dSession, Tool};
pub use placed::{PlacedSketch3d, sketch3d, sketch3d_path};

use geop_core_program::values::Design;

/// A 3-D sketch as a program holds it.
pub type Sketch3d = geop_core_sketch::space::Sketch3d<Design>;
/// A 3-D sketch's constraint as a program holds it.
pub type Constraint3d = geop_core_sketch::space::Constraint3d<Design>;
