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

use geop_core_math::{scalars::Scalar, solvers::system::Variable};
use geop_core_program::part::{Generator, State};

/// A state a solver moves until the [`Constraint`]s hold: a pose, say.
/// It moves by steps along it ([`Variable::retract`]): a number by adding
/// to it, a pose by the exponential map of a twist, and a constraint's
/// derivatives are by such a step from where the value is. Its numbers
/// ([`Variable::Scalar`]) are what the constraints on it are made of; `==`
/// on them asks whether two enclosures are the same, as a check for change
/// does, never whether two values could be equal.
pub trait Solvable: State + Variable<Scalar: Scalar + PartialEq> {}

impl<T: State + Variable<Scalar: Scalar + PartialEq>> Solvable for T {}

/// A step's arguments with the generator that makes what it contributes
/// from them: the arguments of the target that runs it, since a generator
/// captures nothing. Two are equal where both are, so a version made by the
/// same generator from equal arguments is reused.
struct Generated<I, S, Args, Out> {
    args: Args,
    generator: Generator<I, S, Args, Out>,
}

// By hand, since a derive would ask the same of `I`, `S` and `Out`.
impl<I, S, Args: Clone, Out> Clone for Generated<I, S, Args, Out> {
    fn clone(&self) -> Self {
        Self {
            args: self.args.clone(),
            generator: self.generator,
        }
    }
}

impl<I, S, Args: PartialEq, Out> PartialEq for Generated<I, S, Args, Out> {
    fn eq(&self, other: &Self) -> bool {
        self.args == other.args && std::ptr::fn_addr_eq(self.generator, other.generator)
    }
}

#[cfg(test)]
mod test;
