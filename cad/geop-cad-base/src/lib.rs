//! The CAD application, composed: everything its editor offers, one line
//! per crate ([`registry`]), an editor of all of it ([`editor`]), the
//! example programs ([`examples`]) and the standard parts every workspace
//! can place ([`stdlib`]). The editor itself is
//! [`geop_core_editor::editor`]'s; the operations are their crates'.

pub mod examples;
#[cfg(test)]
mod operations;
pub mod registry;
pub mod stdlib;

pub use geop_core_editor::editor::{Command, Editor, Update};
pub use registry::{BuiltPart, Workspace, build, editor, outputs, registry, runner, workspace};
