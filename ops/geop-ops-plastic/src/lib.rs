//! Features for designing moulded and printed housings: a [`rib`] grown
//! from an open profile up to the walls around it, a [`lip`] and the groove
//! that takes it where two halves of an enclosure meet, and a [`draft`] on
//! the faces a mould slides along.

pub mod band;
mod common;
pub mod draft;
pub mod lip;
pub mod operation;
pub mod rib;

pub use operation::{
    Draft, DraftArgs, Groove, GrooveArgs, Lip, LipArgs, Rib, RibArgs, RibDirection, RibSide,
};

#[cfg(test)]
mod tests;
