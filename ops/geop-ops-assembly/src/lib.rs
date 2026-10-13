//! Assemblies: parts placed in a part ([`placed`]) — another program
//! file's part, built once by a [`Library`] and placed by [`AddPart`] at a
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
mod part_pattern;
pub mod joints;
pub mod mates;
pub mod mechanism;
pub mod placed;
pub mod residuals;
pub mod solids;
pub mod solving;
mod stl;

pub use add_part::{AddPart, AddPartArgs};
pub use frame::{MateEntity, PlacedElement};
pub use mates::{
    Anchor, CouplingKind, JointKind, Kind, Mate, MateKind, Motion, On, Resolved, Transmission,
    joint_parameter, mate, mate_name,
};
pub use mechanism::mates_resolved;
pub use placed::{
    Component, GivenLibrary, Instance, Library, Placement, define_library, draw_instances, instance,
    instance_path, library, placed_parts, read_library, solid_meshes,
};
pub use part_pattern::{Layout, PartPattern, PartPatternArgs, PatternAxis};
pub use stl::{STL_QUALITY, Stl, StlOptions, stl_bytes};
