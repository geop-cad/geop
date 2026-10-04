//! Inspecting a part: what an engineer asks of a design without changing
//! it — none of it is an operation, and none of it adds a step.
//!
//! - [`mass`]: the mass properties of every solid of a part and of the parts
//!   placed in it, each of its own part's material, and of all of them
//!   together ([`mass_report`]).
//! - [`measure`]: distances, angles, lengths, areas and radii of picked
//!   entities ([`measure()`]).
//! - [`interference`]: which solids overlap, by how much, and which only
//!   touch ([`interference_report`]).
//!
//! Every number is computed from the exact geometry and carries how well it
//! is known ([`Bounded`]): an interval enclosure, widened — for the
//! integrals — by the quadrature's estimate of its truncation error (see
//! [`geop_core_math::quadrature`]). Lengths are millimetres, masses
//! kilograms (see [`geop_ops::parameters::Material`]).

pub mod bodies;
pub mod bounded;
pub mod interference;
pub mod mass;
pub mod measure;

pub use bounded::Bounded;
pub use interference::{InterferenceReport, interference_report};
pub use mass::{MassReport, mass_report};
pub use measure::{Measurement, measure};

#[cfg(test)]
mod tests;
