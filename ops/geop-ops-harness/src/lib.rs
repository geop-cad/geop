//! Wire harnesses: bundles of wires routed between connectors on placed
//! parts, through clips — the route a tangent-continuous chain of lines and
//! arcs ([`route`]), its bends checked against the bundle's minimum bend
//! radius, the bundle swept along it, and the length to cut every wire to
//! recorded on the part ([`geop_ops::part::Cable`]) — and the [`Route`]
//! operation of a program built on it.

pub mod operation;
pub mod route;
pub mod wire;

pub use operation::{Route, RouteArgs};
pub use wire::{Wire, WireSize};

#[cfg(test)]
mod tests;
