//! Edits of the bodies a part already has, as operations of a program (see
//! [`geop_core_editor::part_operation`]):
//!
//! - [`DeleteBody`]: delete solids and faces standing on their own.
//! - [`ExtractFace`]: copy a face out of its body into a face standing on
//!   its own — what a solid can be split with.
//! - [`ProjectCurve`]: project a sketch's curves along its plane's normal
//!   onto a face, dividing the face along them.
//! - [`MoveFace`], [`OffsetFace`], [`ReplaceFace`]: direct edits of a
//!   solid's faces — shifted, pushed or pulled along their normals, put on
//!   another face's surface — the solid rebuilt around them.

pub mod delete;
pub mod direct;
#[cfg(test)]
mod editor_tests;
pub mod extract;
mod icons;
#[cfg(test)]
mod program_tests;
pub mod project;
#[cfg(test)]
mod stress_tests;

pub use delete::{DeleteBody, DeleteBodyArgs};
pub use direct::{
    MoveFace, MoveFaceArgs, OffsetFace, OffsetFaceArgs, ReplaceFace, ReplaceFaceArgs,
};
pub use extract::{ExtractFace, ExtractFaceArgs};
pub use project::{ProjectCurve, ProjectCurveArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: deleting bodies,
/// extracting faces, projecting curves, and moving, offsetting and
/// replacing faces.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(DeleteBody)?
        .operation(ExtractFace)?
        .operation(ProjectCurve)?
        .operation(MoveFace)?
        .operation(OffsetFace)?
        .operation(ReplaceFace)
}
