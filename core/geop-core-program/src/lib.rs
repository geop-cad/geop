//! What a program is: its steps and state ([`program`]), the values it is
//! run with ([`values`]), the operations its steps are of ([`operation`],
//! [`operation_registry`]), the part it builds — targets in versions,
//! reused while nothing they read changes ([`part`], [`target`],
//! [`element`]) — the parts of other programs it places ([`instance`]),
//! how it is run ([`program_runner`]), the files around it
//! ([`workspace`]), and numbers given as formulas of its parameters
//! ([`numeric`]). It knows no operation: those are defined by the crates
//! built on it, and no target type but a parameter's.

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

pub mod element;
pub mod instance;
pub mod numeric;
pub mod operation;
pub mod operation_registry;
pub mod part;
pub mod program;
pub mod program_runner;
mod revisioned_value;
pub mod target;
#[cfg(test)]
mod test;
#[cfg(test)]
mod test2;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod values;
pub mod workspace;
