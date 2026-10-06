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
