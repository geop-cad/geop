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
pub mod editing;
pub mod operation;
pub mod operation_registry;
pub mod part;
pub mod program;
pub mod program_runner;
pub mod solve;
pub mod step_editor;
pub mod target;
#[cfg(test)]
mod test;
