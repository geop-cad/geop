//! Wire harnesses: bundles of wires routed between connectors on placed
//! parts, through clips — the route a tangent-continuous chain of lines and
//! arcs ([`route`]), its bends checked against the bundle's minimum bend
//! radius, the bundle swept along it, and the length to cut every wire to
//! recorded as the part's target [`cables`] — and the [`Route`]
//! operation of a program built on it.

pub mod cable;
pub mod operation;
pub mod route;
pub mod wire;

pub use cable::{Cable, Cables, cable, cables};
pub use operation::{Route, RouteArgs, RouteEditing, RouteSession};
pub use wire::{Wire, WireSize};

#[cfg(test)]
mod editor_tests;
mod icons;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod tests;

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: routing cables.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Route)
}
