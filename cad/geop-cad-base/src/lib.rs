//! The CAD application, composed: everything its editor offers, one line
//! per crate ([`registry`]), and an editor of all of it ([`editor`])
//! beside the standard parts every workspace can place
//! ([`geop_lib_stdlib::WithStandardParts`]). The editor itself is
//! [`geop_core_editor::editor`]'s; the operations and their examples are
//! their crates'.

pub mod registry;
#[cfg(test)]
mod tests;

pub use geop_core_editor::editor::{Command, Editor, Update};
pub use registry::{BuiltPart, Workspace, build, editor, outputs, registry, runner, workspace};
