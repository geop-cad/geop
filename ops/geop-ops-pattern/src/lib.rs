//! Repeating and moving bodies, as operations of a program (see
//! [`geop_ops::operation`]):
//!
//! - [`LinearPattern`]: copies in a row along a direction, or in a grid
//!   along two.
//! - [`CircularPattern`]: copies turned around an axis.
//! - [`Mirror`]: a mirror image in a plane.
//! - [`MoveBody`]: bodies, or a copy of them, turned and shifted.
//!
//! Each acts on solids, and on sheets — faces standing on their own — and
//! keeps what it makes as new bodies, or combines it with a solid as an
//! extrude does ([`geop_ops_booleans::Combine`]): joined to the bodies
//! themselves, or cut from another solid. The copies are moved exactly as
//! the bodies are (see `geop_core_topology::Model::transform_body`): every
//! surface and curve by the same motion, each pcurve as it was.
//!
//! # Patterning a feature
//!
//! A program has no "pattern the cut of step `E`": a step only sees the
//! part the steps before it built, never their arguments, and an extrude
//! that cuts keeps nothing of its tool — the solid it cut with is consumed
//! the moment it is combined. What a feature pattern repeats is that tool,
//! so the program builds it as a body of its own and patterns *that*: an
//! extrude kept as a new body, then a pattern cutting it — and every copy —
//! from the plate. The tool is design data like any other body, so the
//! holes follow when it changes.

mod circular;
mod common;
mod features;
mod linear;
mod mirror;
mod move_body;
#[cfg(test)]
mod tests;

pub use circular::{CircularPattern, CircularPatternArgs};
pub use common::Spacing;
pub use linear::{Direction, LinearPattern, LinearPatternArgs};
pub use mirror::{Mirror, MirrorArgs};
pub use move_body::{MoveBody, MoveBodyArgs};
