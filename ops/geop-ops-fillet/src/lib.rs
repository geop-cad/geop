//! Blending a solid's edges: rounding them ([`Fillet`]) and bevelling them
//! ([`Chamfer`]).
//!
//! `blend` builds, for every edge, the tool that holds the material between
//! the edge and its blend face — a ball rolled along it (`rolling`), exact
//! where its section is carried along the edge unchanged — and cuts it away
//! with the boolean difference of `geop_lib_booleans`, or fills it in with
//! their union. Where every edge of a corner is filleted, the corner is
//! rounded by a ball's piece (`corner`); tools joined there are built whole
//! as one solid (`tool`). `operation` makes
//! that the [`Fillet`] and [`Chamfer`] operations of a program.

mod blend;
mod corner;
#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod example_tests;
#[cfg(any(test, feature = "examples"))]
pub mod examples;
pub mod operation;
mod rolling;
mod tool;

pub use operation::{Chamfer, ChamferArgs, Fillet, FilletArgs, VertexRadius};

#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod rolling_tests;
#[cfg(test)]
mod stress_tests;
#[cfg(test)]
mod subd_regression_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tutorial_regression_tests;

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: fillets and chamfers, and
/// the examples of them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let r = r.operation(Fillet)?.operation(Chamfer)?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
