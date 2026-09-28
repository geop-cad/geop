//! What an operation is.
//!
//! An operation is a unit struct implementing [`Operation`], with an `Args`
//! struct holding everything it needs. Applied to a part, it maps
//!
//! ```text
//! (part, args) -> (part, dialog, handles)
//! ```
//!
//! — [`Operation::apply`], [`Operation::dialog`] and [`Operation::handles`].
//! They are three methods, not one, because they are needed apart: a script
//! only builds, and an editor needs the dialog most exactly when the
//! arguments do not build yet.
//!
//! Arguments are plain design data — `f64` lengths, sketches — and refer to
//! existing entities of the part only by name, never by an internal id: an
//! id only means something inside the one build that produced it, a name
//! means the same thing in every build of the same program (see the crate
//! docs). Entities a step builds on directly — a sketch's plane, what a
//! datum is built from — are [`EntityRef`]s: a vertex, edge, face or datum
//! by name, or the origin, a world axis or a base plane.
//!
//! A set of operations is an enum with one variant `Name(NameArgs)` per
//! operation, implementing [`Operations`] through `#[derive(Operations)]`:
//! what a program step holds, and what serializes as `{"operation":
//! "extrude", "args": {...}}`.

mod dialog;
mod entity;
mod handle;
mod schema;

pub use dialog::{ArgDialog, Dialog};
pub use entity::{EntityRef, Geometry, Role, WorldAxis, frame_along, resolve_plane, world_frame};
pub use handle::{ArgPath, Handle, HandleGroup, HandleMotion, arg_path, to_f64};
pub use schema::{ArgKind, ArgSchema, ConstructionSchema, OperationArgs, OperationSchema};

pub use geop_ops_derive::{OperationArgs, Operations};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Serialize, de::DeserializeOwned};

use crate::Part;

/// One kind of operation on a [`Part`].
pub trait Operation<S: Scalar> {
    /// Everything the operation needs, as plain, serializable design data.
    type Args;

    /// Applies the operation as the program step `operation_id`, consuming
    /// `part` and returning the part it produces. Everything the operation
    /// creates is named after `operation_id` (see the crate docs), so the id
    /// has to be unique within a program.
    ///
    /// On error no part is returned — a caller that still needs the
    /// original should clone it first.
    fn apply(&self, part: Part<S>, operation_id: &str, args: &Self::Args) -> GeopResult<Part<S>>;

    /// What an editor shows for the step beyond its arguments' schema (see
    /// [`Dialog`]), given the part `before` it — what it is applied to.
    /// Never fails: it is needed most when the arguments do not build yet.
    fn dialog(&self, before: &Part<S>, args: &Self::Args) -> Dialog {
        let _ = (before, args);
        Dialog::default()
    }

    /// The step's handles (see [`Handle`]), given the part `before` it.
    /// None, unless the operation offers some.
    fn handles(&self, before: &Part<S>, args: &Self::Args) -> GeopResult<Vec<Handle>> {
        let _ = (before, args);
        Ok(Vec::new())
    }
}

/// A set of operations, each together with its arguments: the enum a
/// [`crate::Program`]'s steps hold.
///
/// Written by `#[derive(Operations)]` on an enum whose every variant is
/// `Name(NameArgs)`, with `Name` the unit struct implementing [`Operation`]:
/// it dispatches to `Name`, converts from each `NameArgs`, and describes
/// every operation in [`Operations::schemas`] — which is how an editor
/// learns what operations there are and what they take. A variant's doc
/// comment describes the operation, and `#[operation(label = "...")]` gives
/// its short name if that is not the variant's.
pub trait Operations: Clone + std::fmt::Debug + PartialEq + Serialize + DeserializeOwned {
    /// Applies the operation as the program step `operation_id`, see
    /// [`Operation::apply`].
    fn apply<S: Scalar>(&self, part: Part<S>, operation_id: &str) -> GeopResult<Part<S>>;

    /// The step's dialog, given the part `before` it, see
    /// [`Operation::dialog`].
    fn dialog<S: Scalar>(&self, before: &Part<S>) -> Dialog;

    /// The step's handles, given the part `before` it, see
    /// [`Operation::handles`].
    fn handles<S: Scalar>(&self, before: &Part<S>) -> GeopResult<Vec<Handle>>;

    /// The operation's kind, as it is serialized: `extrude`.
    fn kind(&self) -> &'static str;

    /// The operation's short name: `Extrude`.
    fn label(&self) -> &'static str;

    /// A description of every operation of the set and its arguments.
    fn schemas() -> Vec<OperationSchema>;
}
