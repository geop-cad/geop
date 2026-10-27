//! Constraints on a part's state, and the solver that moves the state until
//! they hold.
//!
//! A program's steps say what they require of the state ([`Constraint`]s,
//! [`constraint::define_constraints`]) and which values of it a solve may
//! move ([`unknown::Unknown`]s, [`unknown::declare_unknowns`]). Each of the
//! two is one target of the part, built up by the steps one revision after
//! another ([`Constraints`], [`unknown::Unknowns`]); [`solve`] reads them
//! from the part the program builds at every state it tries.

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

pub mod constraint;
pub mod solve;
pub mod unknown;

pub use constraint::{Bound, Constraint, Constraints};

use geop_core_program::part::Make;

/// A step's arguments with the generator that makes what it contributes
/// from them: the arguments of the target that runs it, since a generator
/// captures nothing. Two are equal where both are, so a version made by the
/// same generator from equal arguments is reused.
struct Generated<Args, Out> {
    args: Args,
    generator: Make<Args, Out>,
}

// By hand, since a derive would ask the same of `Out`.
impl<Args: Clone, Out> Clone for Generated<Args, Out> {
    fn clone(&self) -> Self {
        Self {
            args: self.args.clone(),
            generator: self.generator,
        }
    }
}

/// The generator by its address, which is this process's own: what read a
/// generated target is made again in another process (see
/// `geop_core_program::kept`), never taken for what another generator made.
impl<Args: std::fmt::Debug, Out> std::fmt::Debug for Generated<Args, Out> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let generator = self.generator as *const ();
        write!(f, "Generated({:?}, {generator:p})", self.args)
    }
}

impl<Args: PartialEq, Out> PartialEq for Generated<Args, Out> {
    fn eq(&self, other: &Self) -> bool {
        self.args == other.args && std::ptr::fn_addr_eq(self.generator, other.generator)
    }
}

#[cfg(test)]
mod test;
