//! STEP (ISO 10303) exchange: reading the B-rep solids and sheets of a STEP
//! file into a part, and writing a part's bodies as one — so parts travel
//! between geop and other CAD systems.
//!
//! - [`part21`]: the clear-text encoding every STEP file is written in.
//! - [`import`]: a file's bodies as descriptions the kernel builds.
//! - [`export`]: a part's bodies as a file ([`write_step`]).
//! - [`ImportStep`]: the operation adding a file's bodies to a part.

pub mod export;
pub mod import;
pub mod operation;
pub mod part21;

pub use export::{Step, write_step};
pub use import::{Healing, ImportedBody, read_step};
pub use operation::{ImportStep, ImportStepArgs, Imported, add_bodies, imported, is_step_file};

#[cfg(test)]
mod tests;

#[cfg(test)]
mod corpus;
mod icons;
