//! The CAD engine: which operations the editor offers, and the programs
//! written in them ([`operations`]); editing such a program, as one state
//! machine an editor drives ([`editor`]); example programs
//! ([`examples`]); and the standard parts every workspace can place
//! ([`stdlib`]).

pub mod editor;
pub mod examples;
pub mod operations;
pub mod stdlib;

pub use editor::{Command, Editor, Update};
pub use operations::{PartOperation, Program, ProgramRunner, Step, Workspace};
