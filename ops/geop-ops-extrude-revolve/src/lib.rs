//! Extrude, revolve, sweep and loft as operations of a program: a sketch's
//! one area swept into a solid — along its plane's normal, around an axis,
//! along another sketch's curves, or through other sketches' profiles —
//! kept as a new body or combined with one the model has; or its curves
//! swept into a sheet, faces standing on their own. Which of these, each
//! says by an [`Output`].
//!
//! Extrude and revolve say how far they go the same way, by an [`Extents`]:
//! one side, both sides alike, or each side its own way — a length (an
//! angle), up to the next face of the solid they are combined with, or
//! through all of it.
//!
//! The operations resolve what their arguments refer to — the sketch, the
//! axis, the path — into geometry, and sweep it with `geop_lib_sweep`.
//!
//! [`examples`] are the programs an editor offers that exercise them.
//!
//! With the `testing` feature — which only other crates' tests turn on —
//! `testing` draws the sketches and pushes the steps their programs start
//! from.

#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod example_tests;
pub mod examples;
mod extrude;
mod icons;
mod loft;
#[cfg(test)]
mod parameters_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
mod revolve;
#[cfg(test)]
mod sketch3d_program_tests;
mod sweep;
mod sweeping;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

pub use extrude::{Extrude, ExtrudeArgs};
pub use geop_ops_model::Extent;
pub use loft::{Loft, LoftArgs};
pub use revolve::{Revolve, RevolveArgs, RevolveAxis};
pub use sweep::{Sweep, SweepArgs};
pub use sweeping::{Extents, Output, PathRef};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: extrusions, revolutions,
/// sweeps and lofts, and the examples of them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Extrude)?
        .operation(Revolve)?
        .operation(Sweep)?
        .operation(Loft)?
        .examples(examples::set())
}
