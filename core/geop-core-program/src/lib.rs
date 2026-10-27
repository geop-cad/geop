//! What a program is: its steps and state ([`program`]), the values it is
//! run with ([`values`]), the operations its steps are of ([`operation`],
//! [`operation_registry`]), the part it builds ([`part`], [`target`],
//! [`reader`], [`element`]), the parts of other programs it places
//! ([`placed`]), how it is run ([`runner`]), the files around it
//! ([`workspace`]) and what their texts were made into ([`cache`]), and numbers given as formulas of its parameters
//! ([`numeric`]). It knows no operation: those are defined by the crates
//! built on it, and no target type but a parameter's.
//!
//! # The run model
//!
//! A part is a memo table with one kind of node: what one step produced of
//! one target, keyed by the step's id and the target's name, never by a
//! position. A run walks the program's steps in order; each step only
//! declares what it produces ([`part::StepContext::produce`]), with its
//! arguments and a plain function, its make, that computes the value
//! through a [`reader::Reader`]. Every read is recorded on the node
//! ([`target::Read`]): which producer's version of a target, that a target
//! was absent, the value of an input or of the state, a file's revision, a
//! read inside a placed part.
//!
//! A make runs when its node is new, its arguments differ, or something it
//! read changed; nothing else ever runs. A node made again into an equal
//! value keeps its version, so what read it is reused in turn.
//!
//! The producer a step sees of a target is the latest step before it in the
//! program's order that produces it. A target several steps produce — the
//! model, the parameters' values — is a chain ([`target::Target::CHAIN`]):
//! each producer starts from what the one before left
//! ([`reader::Reader::previous`]). Inserting, removing or reordering steps
//! changes which producer a reader sees, which the recorded reads catch.
//!
//! A placed part is a node whose value is a built part
//! ([`placed::PlacedPart`]), built from exactly its program, its inputs and
//! the slice of the state under its name; equal builds are shared, so a
//! thousand equal screws are one build.

// Library code reports what goes wrong as an error, never by panicking;
// tests may unwrap.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable
    )
)]

pub mod cache;
pub mod element;
pub mod kept;
pub mod numeric;
pub mod operation;
pub mod operation_registry;
pub mod part;
pub mod placed;
pub mod program;
pub mod reader;
pub mod runner;
pub mod target;
#[cfg(test)]
mod test;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod values;
pub mod workspace;
