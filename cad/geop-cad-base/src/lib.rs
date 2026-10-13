//! The CAD engine: which operations the editor offers, and the programs
//! written in them ([`operations`]); editing such a program, as one state
//! machine an editor drives ([`editor`]), with its pick tool ([`inspect`]);
//! example programs ([`examples`]); and the standard parts every workspace
//! can place ([`stdlib`]).

pub mod editor;
pub mod examples;
pub mod inspect;
pub mod operations;
pub mod part;
pub mod stdlib;
pub mod step_editor;
pub mod workspace;

pub use editor::{Command, Editor, Update};
pub use operations::{Program, Workspace, infos, outputs, registry};
