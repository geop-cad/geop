//! The CAD application, composed: everything its editor offers, one line
//! per crate ([`registry`]), an editor of all of it ([`editor`]) beside
//! the standard parts every workspace can place
//! ([`geop_lib_stdlib::WithStandardParts`]), and the example programs
//! ([`examples`]). The editor itself is
//! [`geop_core_editor::editor`]'s; the operations are their crates'.

pub mod examples;
#[cfg(test)]
mod operations;
pub mod registry;

pub use geop_core_editor::editor::{Command, Editor, Update};
pub use registry::{BuiltPart, Workspace, build, editor, outputs, registry, runner, workspace};
