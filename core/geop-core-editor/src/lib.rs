//! What an operation is to an editor: the vocabulary a step's editing is
//! said in — dialogs, visuals, gizmos, events, picks ([`editing`]), number
//! fields of numbers typed as formulas
//! ([`geop_core_program::numeric`], [`edit_form::EditForm::numeric`]) —
//! what an operation shows
//! and does with them ([`editable::Editable`]), the operations an
//! editor offers ([`editable::Editors`]), what it draws of a part and picks
//! in it ([`view`]), and what is made of a part rather than in it: files
//! and answers ([`output`]). The interaction every operation
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

pub mod edit_form;
pub mod editable;
pub mod editing;
pub mod output;
pub mod part_operation;
pub mod selectable;
#[cfg(test)]
mod tests;
pub mod view;

/// What [`part_operation!`] expands to refers to, whichever crates the
/// operation's own depends on.
#[doc(hidden)]
pub use geop_core_math::geop_error::GeopResult as __GeopResult;
#[doc(hidden)]
pub use geop_core_program as __program;
