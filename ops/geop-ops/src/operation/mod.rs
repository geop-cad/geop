//! What an operation is.
//!
//! An operation is a unit struct implementing [`Operation`], with an `Args`
//! struct holding everything a step of it needs and a `Session` holding the
//! temporary state of editing one. Built, a step maps
//!
//! ```text
//! (part, args) -> part                                            apply
//! ```
//!
//! and edited, every user action maps
//!
//! ```text
//! (part, args, session, event) -> (args, session, presentation)   edit
//! ```
//!
//! — the editor reruns the program with the new arguments, shows the
//! presentation (see [`crate::ui`]), and sends the next event.
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

pub use entity::{EntityRef, Geometry, Role, WorldAxis, frame_along, resolve_plane, world_frame};

pub use geop_ops_derive::Operations;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    Part,
    ui::{Event, PartView, Presentation},
};

/// What an operation is edited against: the part before its step — what
/// the step is applied to — and that part as the viewport draws it, for
/// picking.
pub struct EditContext<'a, S: Scalar> {
    pub part: &'a Part<S>,
    pub view: &'a PartView,
}

/// What an edit gives: the step's new arguments, the edit's new session,
/// and what to show.
#[derive(Clone, Debug)]
pub struct Edited<A, T> {
    pub args: A,
    pub session: T,
    pub presentation: Presentation,
}

impl<A, T> Edited<A, T> {
    /// The same edit, with its arguments turned into `f(args)`.
    pub fn map_args<B>(self, f: impl FnOnce(A) -> B) -> Edited<B, T> {
        Edited {
            args: f(self.args),
            session: self.session,
            presentation: self.presentation,
        }
    }
}

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
    /// The temporary state of editing a step: never saved, and reset —
    /// to its default — whenever an editor starts editing a step afresh.
    type Session: Default + Serialize + DeserializeOwned;

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

    /// Edits a step: `event` applied to its arguments and the session, and
    /// what to show afterwards — or, without an event, only what to show.
    ///
    /// Never fails: an editor needs a dialog most exactly when the
    /// arguments do not build, so whatever goes wrong is said in the dialog.
    fn edit<S: Scalar>(
        &self,
        ctx: &EditContext<S>,
        args: Self::Args,
        session: Self::Session,
        event: Option<&Event>,
    ) -> Edited<Self::Args, Self::Session>;

    /// The arguments in one line, for a list of steps:
    /// `sketch=outline, distance=1.00`.
    fn summary(&self, args: &Self::Args) -> String;

    /// The sketches and datums a step builds on. An editor hides them once
    /// the step has used them: what was made from them shows them now.
    fn references(&self, args: &Self::Args) -> Vec<EntityRef> {
        let _ = args;
        Vec::new()
    }
}

/// [`Operation::edit`] with the session as JSON — how a set of operations,
/// whose sessions differ by operation, carries one. A session that does not
/// read as this operation's starts afresh.
pub fn edit_json<S: Scalar, O: Operation>(
    op: &O,
    ctx: &EditContext<S>,
    args: O::Args,
    session: serde_json::Value,
    event: Option<&Event>,
) -> Edited<O::Args, serde_json::Value> {
    let session = serde_json::from_value(session).unwrap_or_default();
    let edited = op.edit(ctx, args, session, event);
    Edited {
        args: edited.args,
        session: serde_json::to_value(edited.session).unwrap_or_default(),
        presentation: edited.presentation,
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
/// variant's.
pub trait Operations: Clone + std::fmt::Debug + PartialEq + Serialize + DeserializeOwned {
    /// Every operation of the set.
    fn infos() -> Vec<OperationInfo>;

    /// A new step of the operation `kind`, inserted after `before`, see
    /// [`Operation::new_args`].
    fn new_step<S: Scalar>(kind: &str, before: &Part<S>) -> GeopResult<Self>;

    /// See [`Operation::apply`].
    fn apply<S: Scalar>(&self, part: Part<S>, operation_id: &str) -> GeopResult<Part<S>>;

    /// See [`Operation::edit`], with the session as JSON (see
    /// [`edit_json`]).
    fn edit<S: Scalar>(
        &self,
        ctx: &EditContext<S>,
        session: serde_json::Value,
        event: Option<&Event>,
    ) -> Edited<Self, serde_json::Value>;

    /// See [`Operation::summary`].
    fn summary(&self) -> String;

    /// See [`Operation::references`].
    fn references(&self) -> Vec<EntityRef>;

    /// The operation's kind, as it is serialized: `extrude`.
    fn kind(&self) -> &'static str;

    /// The operation's short name: `Extrude`.
    fn label(&self) -> &'static str;
}
