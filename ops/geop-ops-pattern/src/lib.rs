//! Repeating and moving bodies, as operations of a program (see
//! [`geop_core_editor::part_operation::PartOperation`]):
//!
//! - [`LinearPattern`]: copies in a row along a direction, or in a grid
//!   along two.
//! - [`CircularPattern`]: copies turned around an axis.
//! - [`Mirror`]: a mirror image in a plane.
//! - [`MoveBody`]: bodies, or a copy of them, turned and shifted.
//! - [`ScaleBody`]: bodies scaled about a point, evenly or along each axis.
//! - [`DeformBody`]: bodies tapered, twisted or bent about an axis.
//!
//! The first three repeat bodies or features.
//!
//! Each acts on solids, and on sheets — faces standing on their own — and
//! keeps what it makes as new bodies, or combines it with a solid as an
//! extrude does ([`geop_ops_model::Combine`]): joined to the bodies
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
//! [`geop_ops_model::Feature`]), and a feature is picked by a face it made
//! ([`Feature`]). A pattern copies those tools, moves them, and
//! combines each copy with the solid the feature lies on as the feature
//! combined it: why that, rather than running the step again, is said in
//! `features.rs`.
//!
//! # Spacing
//!
//! How instances are spaced in a row — each a step from the last, or spread
//! over an extent — and how many there are is
//! [`geop_ops_model::spacing`]'s, shared with every operation that lays out
//! instances (a placed part's copies, too).

mod circular;
mod common;
mod deform_body;
#[cfg(test)]
mod editor_tests;
#[cfg(any(test, feature = "examples"))]
pub mod examples;
mod features;
mod icons;
mod linear;
mod mirror;
mod move_body;
#[cfg(test)]
mod program_tests;
mod references;
#[cfg(test)]
mod regression_tests;
mod scale_body;
#[cfg(test)]
mod tests;

pub use circular::{CircularPattern, CircularPatternArgs};
pub use common::{BodyRef, Seeds, body};
pub use deform_body::{Deform, DeformBody, DeformBodyArgs};
pub use features::{Feature, FeatureRef, feature};
pub use linear::{Direction, LinearPattern, LinearPatternArgs};
pub use mirror::{Mirror, MirrorArgs};
pub use move_body::{MoveBody, MoveBodyArgs};
pub use references::{PatternDirection, TurnAxis};
pub use scale_body::{ScaleBody, ScaleBodyArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: patterns, mirrors and
/// moved bodies.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let r = r
        .operation(LinearPattern)?
        .operation(CircularPattern)?
        .operation(Mirror)?
        .operation(MoveBody)?
        .operation(ScaleBody)?
        .operation(DeformBody)?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
