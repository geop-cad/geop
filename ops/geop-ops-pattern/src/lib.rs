//! Repeating and moving bodies, as operations of a program (see
//! [`geop_ops::operation`]):
//!
//! - [`LinearPattern`]: copies in a row along a direction, or in a grid
//!   along two.
//! - [`CircularPattern`]: copies turned around an axis.
//! - [`Mirror`]: a mirror image in a plane.
//!
//! These three repeat bodies or features.
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
//! Patterns and mirrors also repeat features — a hole cut by an extrude, a
//! boss joined to a plate, a tapped hole with its thread — picked by a face
//! each made. A step only sees the part the steps before it built, never
//! their arguments; what it sees of a feature is what the feature's step
//! recorded on the part as it combined its tools (see
//! [`geop_ops::Feature`]). A pattern copies those tools, moves them, and
//! combines each copy with the solid the feature lies on as the feature
//! combined it: why that, rather than running the step again, is said in
//! `features.rs`.

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
