//! Simple edits of the bodies a part already has, as operations of a
//! program (see [`geop_ops::operation`]):
//!
//! - [`DeleteBody`]: delete solids and faces standing on their own.
//! - [`ExtractFace`]: copy a face out of its body into a face standing on
//!   its own — what a solid can be split with.
//! - [`ProjectCurve`]: project a sketch's curves along its plane's normal
//!   onto a face, dividing the face along them.

pub mod delete;
pub mod extract;
pub mod project;

pub use delete::{DeleteBody, DeleteBodyArgs};
pub use extract::{ExtractFace, ExtractFaceArgs};
pub use project::{ProjectCurve, ProjectCurveArgs};

use geop_ops::EntityRef;

/// A face by name, as a reference field holds it: nothing, if unnamed.
fn face_ref(name: &str) -> Vec<EntityRef> {
    if name.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Face { name: name.into() }]
    }
}

/// The name of the face a reference field holds: none, if it holds none.
fn face_name(picked: &[EntityRef]) -> String {
    match picked {
        [EntityRef::Face { name }] => name.clone(),
        _ => String::new(),
    }
}
