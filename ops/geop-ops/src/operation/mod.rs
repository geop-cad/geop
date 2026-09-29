//! What an operation is.
//!
//! An operation is a unit struct implementing [`Operation`], with an `Args`
//! struct holding everything a step of it needs and a `Session` holding the
//! temporary state of editing one, if it needs any. Built, a step maps
//!
//! ```text
//! (part, args) -> part                                            apply
//! ```
//!
//! and edited, it shows a [`Form`] — fields and visuals — whose fields the
//! editor sets:
//!
//! ```text
//! (part, args)                   -> form                           form
//! (part, args, field, value)     -> args                           set
//! ```
//!
//! Picking entities for a field and dragging handles are the editor's (see
//! [`crate::ui::StepEditor`]), the same for every operation; an operation
//! that draws in a canvas of its own — a sketch — also takes the raw
//! pointer and key events ([`Operation::event`]), with a `Session` for what
//! it keeps between them.
//!
//! Arguments are plain design data — `f64` lengths, sketches — and refer to
//! existing entities of the part only by name, never by an internal id: an
//! id only means something inside the one build that produced it, a name
//! means the same thing in every build of the same program (see the crate
//! docs). Entities a step builds on directly — a sketch's plane, what a
//! datum is built from — are [`EntityRef`]s.
//!
//! A set of operations is an enum with one variant `Name(NameArgs)` per
//! operation, implementing [`Operations`] through `#[derive(Operations)]`:
//! what a program step holds, and what serializes as `{"operation":
//! "extrude", "args": {...}}`.

mod entity;

pub use entity::{EntityRef, frame_along};

pub use geop_ops_derive::Operations;

use std::any::Any;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    Part,
    ui::{Event, Form, Value},
};

/// An operation of a set: how a step spells it, its short name and what it
/// does — what an editor offers it by.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct OperationInfo {
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
}

/// One kind of operation on a [`Part`].
pub trait Operation {
    /// Everything a step needs, as plain, serializable design data.
    type Args: Clone;
    /// The temporary state of editing a step beyond its arguments — `()`
    /// for every operation whose form says all there is to it. Starts
    /// afresh whenever an editor opens a step.
    type Session: Default + 'static;

    /// The arguments of a new step inserted after `before`: sensible
    /// defaults, taking what `before` holds into account.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> Self::Args;

    /// Applies the operation as the program step `operation_id`, consuming
    /// `part` and returning the part it produces. Everything the operation
    /// creates is named after `operation_id` (see the crate docs), so the id
    /// has to be unique within a program.
    ///
    /// On error no part is returned — a caller that still needs the
    /// original should clone it first.
    fn apply<S: Scalar>(
        &self,
        part: Part<S>,
        operation_id: &str,
        args: &Self::Args,
    ) -> GeopResult<Part<S>>;

    /// What a step shows while it is edited against `before`: its fields —
    /// numbers, choices, entities to pick — and what it draws, handles to
    /// drag among them. Never fails: an editor needs the form most exactly
    /// when the arguments do not build, so whatever is wrong is said in it.
    fn form<S: Scalar>(
        &self,
        before: &Part<S>,
        args: &Self::Args,
        session: &Self::Session,
    ) -> Form<S>;

    /// The field `key` of the form set to `value`: typed or chosen in the
    /// dialog, picked in the viewport, dragged as a handle.
    fn set<S: Scalar>(
        &self,
        before: &Part<S>,
        args: &mut Self::Args,
        session: &mut Self::Session,
        key: &str,
        value: Value,
    );

    /// A pointer or key event the editor did not take itself — a pick or a
    /// handle's drag it does. Only an operation that draws in a canvas of
    /// its own, like a sketch, needs these.
    fn event<S: Scalar>(
        &self,
        before: &Part<S>,
        args: &mut Self::Args,
        session: &mut Self::Session,
        event: &Event<S>,
    ) {
        let _ = (before, args, session, event);
    }
}

/// A set of operations, each together with its arguments: the enum a
/// [`crate::Program`]'s steps hold.
///
/// Written by `#[derive(Operations)]` on an enum whose every variant is
/// `Name(NameArgs)`, with `Name` the unit struct implementing [`Operation`]
/// with `Args = NameArgs`: it dispatches to `Name`, and converts from each
/// `NameArgs`. A variant's doc comment describes the operation, and
/// `#[operation(label = "...")]` gives its short name if that is not the
/// variant's. A step's session is the session of its operation, boxed:
/// sessions differ by operation.
pub trait Operations: Clone + std::fmt::Debug + PartialEq + Serialize + DeserializeOwned {
    /// Every operation of the set.
    fn infos() -> Vec<OperationInfo>;

    /// A new step of the operation `kind`, inserted after `before`, see
    /// [`Operation::new_args`].
    fn new_step<S: Scalar>(kind: &str, before: &Part<S>) -> GeopResult<Self>;

    /// See [`Operation::apply`].
    fn apply<S: Scalar>(&self, part: Part<S>, operation_id: &str) -> GeopResult<Part<S>>;

    /// A fresh session for editing this step.
    fn new_session(&self) -> Box<dyn Any>;

    /// See [`Operation::form`]; `session` is one [`Operations::new_session`]
    /// made for a step of the same operation.
    fn form<S: Scalar>(&self, before: &Part<S>, session: &dyn Any) -> Form<S>;

    /// See [`Operation::set`].
    fn set<S: Scalar>(&mut self, before: &Part<S>, session: &mut dyn Any, key: &str, value: Value);

    /// See [`Operation::event`].
    fn event<S: Scalar>(&mut self, before: &Part<S>, session: &mut dyn Any, event: &Event<S>);

    /// The operation's kind, as it is serialized: `extrude`.
    fn kind(&self) -> &'static str;

    /// The operation's short name: `Extrude`.
    fn label(&self) -> &'static str;
}
