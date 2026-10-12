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

mod icons;
#[cfg(test)]
mod program_tests;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
#[cfg(test)]
mod tests;
