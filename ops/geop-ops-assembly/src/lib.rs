//! Assemblies: parts placed in a part ([`placed`]) — another program
//! file's part, loaded through the runner and placed by [`AddPart`] at a
//! pose its program's state holds — and mated to what is already there
//! ([`mates`]). Where every placed part is, is the program's state, which
//! the editor solves the mates for ([`mechanism`], and [`solving`], which
//! solves it); dragging the placed part moves it as far as its mates let it
//! (see [`editor`]). [`PartPattern`] places copies of a placed part in a
//! row or round an axis.
//!
//! Also what reads a part with the parts placed in it: their solids, each
//! of its own part's material ([`solids`]), and an STL file of them all
//! ([`Stl`]).

mod add_part;
pub mod editor;
pub mod frame;
#[cfg(test)]
mod frame_tests;
mod icons;
pub mod joints;
pub mod mates;
pub mod mechanism;
mod part_pattern;
#[cfg(any(test, feature = "testing"))]
#[allow(clippy::expect_used, reason = "tests' helpers fail where a test would")]
pub mod testing;
pub mod placed;
pub mod residuals;
pub mod solids;
pub mod solving;
mod stl;

pub use add_part::{AddPart, AddPartArgs};
pub use frame::MateEntity;
pub use mates::{
    Anchor, CouplingKind, JointKind, Kind, Mate, MateKind, Motion, On, Resolved, Transmission,
    joint_parameter, mate, mate_name,
};
pub use mechanism::mates_resolved;
pub use part_pattern::{Layout, PartPattern, PartPatternArgs, PatternAxis};
pub use placed::{Drawn, define_placed, draw, draw_instances, drawn, placed_parts, solid_meshes};
pub use stl::{STL_QUALITY, Stl, StlOptions, stl_bytes};
