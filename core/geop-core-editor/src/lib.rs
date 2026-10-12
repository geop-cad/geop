//! What an operation is to an editor: the vocabulary a step's editing is
//! said in — dialogs, visuals, gizmos, events, picks ([`editing`]) — and
//! numbers typed as formulas ([`numeric_input`]), what an operation shows
//! and does with them ([`editable::Editable`]), and the operations an
//! editor offers ([`editable::Editors`]). The interaction every operation
//! shares, driving these, is the editor's own (`geop_cad_base`). A
//! program runs without any of it (see [`geop_core_program`]).

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

pub mod editable;
pub mod editing;
pub mod numeric_input;
#[cfg(test)]
mod tests;
