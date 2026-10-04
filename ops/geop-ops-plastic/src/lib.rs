//! Features for designing moulded and printed housings: a [`draft`] on the
//! faces a mould slides along, a [`lip`] and the groove that takes it where
//! two halves of an enclosure meet.

pub mod band;
mod common;
pub mod draft;
pub mod lip;
pub mod operation;

pub use operation::{Draft, DraftArgs, Groove, GrooveArgs, Lip, LipArgs};

#[cfg(test)]
mod tests;
