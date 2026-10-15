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
pub mod operation;
pub mod operation_registry;
pub mod part;
pub mod program;
pub mod program_runner;
mod revisioned_value;
pub mod target;
pub mod workspace;
#[cfg(test)]
mod test;
#[cfg(test)]
mod test2;
