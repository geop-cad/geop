//! STEP (ISO 10303) exchange: reading the B-rep solids and sheets of a STEP
//! file into a part, and writing a part's bodies as one — so parts travel
//! between geop and other CAD systems.
//!
//! - [`part21`]: the clear-text encoding every STEP file is written in.
//! - [`import`]: a file's bodies as descriptions the kernel builds.
//! - [`export`]: a part's bodies as a file.
//! - [`ImportStep`]: the operation adding a file's bodies to a part.

pub mod cache;
pub mod export;
pub mod import;
pub mod part21;

pub use export::write_step;
pub use import::{Healing, ImportedBody, read_step};
pub mod operation;

pub use operation::{ImportStep, ImportStepArgs, add_bodies, is_step_file};

#[cfg(test)]
mod tests;

#[cfg(test)]
mod corpus;
