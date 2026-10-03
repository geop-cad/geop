//! What an operation is.
//!
//! An operation is a unit struct implementing [`Operation`], with an `Args`
//! struct holding everything a step of it needs and a `Session` holding the
//! temporary state of editing one, if it needs any. Built, a step maps
//!
//! ```text
//! (part, args, library) -> part                                   apply
//! ```
//!
//! — the library being where it finds the parts it places, if it places
//! any — and edited, it shows a [`Form`] — fields, each with what setting it
//! does, and visuals — whose fields the editor sets:
//!
//! ```text
//! (context, args, selection)                -> form               form
//! (context, args, selection, field, value)  -> args, selection    set
//! ```
//!
//! with the [`Context`] the part before the step, the step's id and the
//! library.
//!
//! `set` is the form's: each field is described once, with its setter.
//!
//! Picking entities for a field, dragging handles, selecting visuals and
//! dragging them are the editor's (see [`crate::ui::StepEditor`]), the same
//! for every operation; an operation that draws in a canvas of its own — a
//! sketch — also takes what the pointer and the keys do beyond that
//! ([`Operation::event`]), with a `Session` for what it keeps between
//! events.
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

mod aspects;
mod entity;

pub use aspects::{Aspects, Role, describe_roles};
pub use entity::{EntityRef, INSTANCE_SEPARATOR, frame_along};

pub use geop_ops_derive::Operations;

use std::any::Any;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    Part,
    part::State,
    program::Library,
    ui::{CanvasEvent, Edit, Form, Value},
};

/// No state: what a step is edited with by default.
static NO_PARAMETERS: State = State::new();

/// What a step is edited against: the part before it, its own id — what
/// everything it builds is named after, a new step's included — the
/// library it finds the parts it places in, the program's state as
/// they are now, and what it built when it last ran.
pub struct Context<'a, S: Scalar> {
    pub before: &'a Part<S>,
    pub id: &'a str,
    pub library: &'a dyn Library<S>,
    pub state: &'a State,
    /// The part the step built, as it last ran — `None` if it has not run,
    /// or failed. A form shows what was built from this rather than
    /// building the step again itself: an editor runs a step once per
    /// change, however often it asks for its form.
    pub built: Option<&'a Part<S>>,
}

impl<'a, S: Scalar> Context<'a, S> {
    /// Editing the step `id` against `before`, not run yet, in a program
    /// without a state.
    pub fn new(before: &'a Part<S>, id: &'a str, library: &'a dyn Library<S>) -> Self {
        Self {
            before,
            id,
            library,
            state: &NO_PARAMETERS,
            built: None,
        }
    }

    /// The same, the program's state being `state`.
    pub fn state(self, state: &'a State) -> Self {
        Self { state, ..self }
    }

    /// The same, with the step having built `built`.
    pub fn built(self, built: Option<&'a Part<S>>) -> Self {
        Self { built, ..self }
    }
}

impl<S: Scalar> Clone for Context<'_, S> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<S: Scalar> Copy for Context<'_, S> {}

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

    /// Whether a pick for one of its reference fields tests against the
    /// part as the step builds it, rather than as it was before: for an
    /// operation whose references point into what it adds itself — a
    /// placed part's mates.
    const PICKS_BUILT: bool = false;

    /// The arguments of a new step inserted after `before`: sensible
    /// defaults, taking what `before` holds into account.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> Self::Args;

    /// Applies the operation as the program step `operation_id`, consuming
    /// `part` and returning the part it produces. Everything the operation
    /// creates is named after `operation_id` (see the crate docs), so the id
    /// has to be unique within a program. The parts it places, if any, are
    /// found in `library`.
    ///
    /// On error no part is returned — a caller that still needs the
    /// original should clone it first.
    fn apply<S: Scalar>(
        &self,
        part: Part<S>,
        operation_id: &str,
        args: &Self::Args,
        library: &dyn Library<S>,
    ) -> GeopResult<Part<S>>;

    /// What a step shows while it is edited in `context`: its fields —
    /// numbers, choices, entities to pick — each with what setting it does,
    /// and what it draws, with `selection` the keys of its visuals selected.
    /// Never fails: an editor needs the form most exactly when the arguments
    /// do not build, so whatever is wrong is said in it.
    ///
    /// Its setters may capture the context, never the arguments: they are
    /// handed the arguments to change (see [`Operation::set`]).
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &Self::Args,
        session: &Self::Session,
        selection: &[String],
    ) -> Form<'a, S, Self::Args, Self::Session>;

    /// The field `key` of the form set to `value`: typed or chosen in the
    /// dialog, picked in the viewport, dragged as a handle — by the setter
    /// the form gave it (see [`Form`]).
    #[allow(clippy::too_many_arguments)]
    fn set<S: Scalar>(
        &self,
        context: Context<'_, S>,
        args: &mut Self::Args,
        session: &mut Self::Session,
        selection: &mut Vec<String>,
        state: &mut State,
        key: &str,
        value: Value,
    ) {
        let form = self.form(context, args, session, selection);
        let edit = Edit {
            args,
            session,
            selection,
            state,
        };
        form.set(key, edit, value);
    }

    /// A pointer or key event the editor passed on (see [`CanvasEvent`]).
    /// Only an operation that draws in a canvas of its own, like a sketch,
    /// needs these.
    fn event<S: Scalar>(
        &self,
        context: Context<'_, S>,
        edit: Edit<'_, Self::Args, Self::Session>,
        event: &CanvasEvent<S>,
    ) {
        let _ = (context, edit, event);
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
    fn apply<S: Scalar>(
        &self,
        part: Part<S>,
        operation_id: &str,
        library: &dyn Library<S>,
    ) -> GeopResult<Part<S>>;

    /// See [`Operation::PICKS_BUILT`].
    fn picks_built(&self) -> bool;

    /// A fresh session for editing this step.
    fn new_session(&self) -> Box<dyn Any>;

    /// See [`Operation::form`], as an editor reads it; `session` is one
    /// [`Operations::new_session`] made for a step of the same operation.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        session: &dyn Any,
        selection: &[String],
    ) -> Form<'a, S>;

    /// See [`Operation::set`].
    #[allow(clippy::too_many_arguments)]
    fn set<S: Scalar>(
        &mut self,
        context: Context<'_, S>,
        session: &mut dyn Any,
        selection: &mut Vec<String>,
        state: &mut State,
        key: &str,
        value: Value,
    );

    /// See [`Operation::event`].
    fn event<S: Scalar>(
        &mut self,
        context: Context<'_, S>,
        session: &mut dyn Any,
        selection: &mut Vec<String>,
        state: &mut State,
        event: &CanvasEvent<S>,
    );

    /// The operation's kind, as it is serialized: `extrude`.
    fn kind(&self) -> &'static str;

    /// The operation's short name: `Extrude`.
    fn label(&self) -> &'static str;
}
