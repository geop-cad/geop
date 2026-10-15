//! Blending a solid's edges: rounding them ([`Fillet`]) and bevelling them
//! ([`Chamfer`]).
//!
//! `blend` builds, for every edge, the tool that holds the material between
//! the edge and its blend face — swept from the edge's cross-section, along
//! a straight edge or around a circular one, or, for any other edge and for
//! a radius that varies, rolled along it (`rolling`) — and cuts it away
//! with the boolean difference of `geop_ops_booleans`, or fills it in with
//! their union. Where every edge of a corner is filleted, the corner is
//! rounded by a ball's piece (`corner`); tools joined there, or mitred at an
//! inward corner, are built whole as one solid (`tool`). `operation` makes
//! that the [`Fillet`] and [`Chamfer`] operations of a program.

mod blend;
mod corner;
pub mod operation;
mod rolling;
mod tool;

pub use operation::{Chamfer, ChamferArgs, Fillet, FilletArgs, VertexRadius};

#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod rolling_tests;
#[cfg(test)]
mod tests;

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: fillets and chamfers.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Fillet)?.operation(Chamfer)
}
