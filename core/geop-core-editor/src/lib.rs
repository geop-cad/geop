//! What editing is: the vocabulary a step's editing is said in — dialogs,
//! visuals, gizmos, events, picks ([`editing`]), number fields of numbers
//! typed as formulas ([`geop_core_program::numeric`],
//! [`edit_form::EditForm::numeric`]) — what an operation shows and does
//! with them ([`editable::Editable`]), what a part is drawn as and picked
//! in ([`view`]), and what is made of a part rather than in it: files and
//! answers ([`output`]); the interaction every step shares
//! ([`step_editor`]) and the whole of editing a program ([`editor`]), over
//! what a [`registry::Registry`] offers — operations, drawers, panels,
//! tools ([`tools`]), a settler, examples — none of which it names; and
//! the editor driven by an agent's tools ([`agent`]), which every front end
//! an agent talks to offers. A program runs without any of it (see
//! [`geop_core_program`]).

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

pub mod agent;
pub mod edit_form;
pub mod editable;
pub mod editing;
pub mod editor;
pub mod keys;
pub mod output;
pub mod part_operation;
pub mod registry;
pub mod selectable;
pub mod step_editor;
#[cfg(any(test, feature = "testing"))]
#[allow(
    clippy::expect_used,
    clippy::panic,
    reason = "tests' helpers fail where a test would"
)]
pub mod testing;
#[cfg(test)]
mod tests;
pub mod tools;
pub mod view;

/// What [`part_operation!`] expands to refers to, whichever crates the
/// operation's own depends on.
#[doc(hidden)]
pub use geop_core_math::geop_error::GeopResult as __GeopResult;
#[doc(hidden)]
pub use geop_core_program as __program;
