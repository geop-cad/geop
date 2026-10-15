//! Subdivision-surface modelling: a freeform body shaped by a control cage
//! and built as the cage's Catmull–Clark limit surface, a B-rep of
//! B-spline faces every other operation works with.
//!
//! - [`cage`]: the cage a step holds — vertices, faces, creased edges — and
//!   the same indexed, mirrored and checked.
//! - [`edit`]: the cages to start from, and the edits that shape them.
//! - [`subdivide`]: one Catmull–Clark step with creases, after which every
//!   face is a quad.
//! - [`limit`]: the limit surface of that as bicubic Bézier patches sharing
//!   their boundary curves.
//! - [`brep`]: those patches as a closed solid, or an open sheet, with the
//!   cage's own topology and names.
//! - [`operation`]: the [`Subd`] operation of a program, and how it is
//!   edited.

pub mod brep;
pub mod cage;
pub mod edit;
pub mod limit;
pub mod operation;
pub mod subdivide;

mod icons;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod tests;

pub use cage::{Cage, CageFace, CageVertex, Mirror};
pub use operation::{Subd, SubdArgs, SubdEditing, SubdSession, build_subd};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: subdivision surfaces.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Subd)
}
