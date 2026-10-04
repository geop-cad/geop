//! Features for designing moulded and printed housings: a [`draft`] on the
//! faces a mould slides along.

pub mod draft;
pub mod operation;

pub use operation::{Draft, DraftArgs};

#[cfg(test)]
mod tests;
