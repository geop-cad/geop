//! Edits of the bodies a part already has, as operations of a program (see
//! [`geop_core_editor::part_operation`]):
//!
//! - [`DeleteBody`]: delete solids and faces standing on their own.
//! - [`ExtractFace`]: copy a face out of its body into a face standing on
//!   its own — what a solid can be split with.
//! - [`MoveFace`], [`OffsetFace`], [`ReplaceFace`], [`DeleteFace`]: direct
//!   edits of a solid's faces — shifted, pushed or pulled along their
//!   normals, put on another face's surface, taken out and healed over or
//!   left open — the solid rebuilt around them.

pub mod delete;
pub mod direct;
#[cfg(test)]
mod editor_tests;
pub mod extract;
mod icons;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod stress_tests;

pub use delete::{DeleteBody, DeleteBodyArgs};
pub use direct::{
    DeleteFace, DeleteFaceArgs, HealGap, MergeFaces, MergeFacesArgs, MoveFace, MoveFaceArgs,
    OffsetFace, OffsetFaceArgs, ReplaceFace, ReplaceFaceArgs,
};
pub use extract::{ExtractFace, ExtractFaceArgs};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: deleting bodies,
/// extracting faces, and moving, offsetting and replacing faces.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(DeleteBody)?
        .operation(ExtractFace)?
        .operation(MoveFace)?
        .operation(OffsetFace)?
        .operation(ReplaceFace)?
        .operation(DeleteFace)?
        .operation(MergeFaces)
}
