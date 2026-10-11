//! Assemblies as an operation of a program (see [`geop_core_program::operation`]):
//! [`AddPart`] places the part another program file builds, at a pose, and
//! mates it to what is already there. Where every placed part is, is the
//! program's state, which the editor solves the mates for (see
//! [`mechanism`], and [`geop_ops::solving`], which solves it); dragging the
//! placed part moves it as far as its mates let it (see [`editor`]). [`PartPattern`] places copies of a placed part
//! in a row or round an axis.

mod add_part;
pub mod editor;
mod icons;
pub mod joints;
pub mod mates;
pub mod mechanism;
mod part_pattern;
pub mod residuals;

pub use add_part::{AddPart, AddPartArgs};
pub use mates::{
    Anchor, CouplingKind, JointKind, Kind, Mate, MateKind, Motion, On, Resolved, Transmission,
    joint_parameter, mate, mate_name,
};
pub use mechanism::mates_resolved;
pub use part_pattern::{Layout, PartPattern, PartPatternArgs};
