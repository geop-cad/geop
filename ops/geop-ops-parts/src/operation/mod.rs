//! The operations a [`crate::Program`] is made of.
//!
//! Each is a unit struct implementing [`Operation`], with an `Args` struct
//! holding everything it needs. Arguments are plain design data — `f64`
//! lengths, sketches — and refer to existing entities of the part only by
//! name, never by an internal id: an id only means something inside the one
//! build that produced it, a name means the same thing in every build of
//! the same program (see `geop_core_part`'s crate docs).
//!
//! Entities a step builds on directly — a sketch's plane, what a datum is
//! built from — are [`EntityRef`]s: a vertex, edge, face or datum by name,
//! or the origin, a world axis or a base plane.
//!
//! [`PartOperation`] is any one of them together with its arguments: what a
//! program step holds, and what serializes as `{"operation": "extrude",
//! "args": {...}}`.

mod add_sketch;
mod boolean;
mod datum;
mod entity;
mod extrude;
mod handle;
mod revolve;
mod schema;
#[cfg(test)]
mod tests;

pub use add_sketch::{AddSketch, AddSketchArgs};
pub use boolean::{Boolean, BooleanArgs, Combine};
pub use datum::{
    AddDatum, AddDatumArgs, CONSTRUCTIONS, Construction, SelectionFit, inspect_selection,
};
pub use entity::{EntityRef, Geometry, Role, WorldAxis, resolve_plane};
pub use extrude::{Extrude, ExtrudeArgs};
pub use handle::{ArgPath, Handle, HandleGroup, HandleMotion, arg_path};
pub use revolve::{Revolve, RevolveArgs};
pub use schema::{ArgKind, ArgSchema, ConstructionSchema, OperationArgs, OperationSchema};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_part::Part;
use geop_ops_parts_derive::Operations;
use serde::{Deserialize, Serialize};

/// One kind of operation on a [`Part`].
pub trait Operation<S: Scalar> {
    /// Everything the operation needs, as plain, serializable design data.
    type Args;

    /// Applies the operation as the program step `operation_id`, consuming
    /// `part` and returning the part it produces. Everything the operation
    /// creates is named after `operation_id` (see `geop_core_part`), so the
    /// id has to be unique within a program.
    ///
    /// On error no part is returned — a caller that still needs the
    /// original should clone it first.
    fn apply(&self, part: Part<S>, operation_id: &str, args: &Self::Args) -> GeopResult<Part<S>>;

    /// The step's handles (see [`Handle`]), given the part `before` it —
    /// what it is applied to. None, unless the operation offers some.
    fn handles(&self, before: &Part<S>, args: &Self::Args) -> GeopResult<Vec<Handle>> {
        let _ = (before, args);
        Ok(Vec::new())
    }
}

/// An operation together with its arguments, not yet applied to any part.
///
/// Every operation is registered here, as `Name(NameArgs)` with `Name` its
/// [`Operation`]: `#[derive(Operations)]` generates the dispatch
/// ([`PartOperation::apply`]), the conversions from each `NameArgs`, and
/// [`PartOperation::schemas`], which is how an editor learns what
/// operations there are and what they take.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
pub enum PartOperation {
    /// Draw a sketch on a base plane or on a planar face.
    #[operation(label = "Sketch")]
    AddSketch(AddSketchArgs),
    /// Sweep a sketch's regions along its plane's normal into a solid.
    Extrude(ExtrudeArgs),
    /// Sweep a sketch's regions a full turn around one of its lines.
    Revolve(RevolveArgs),
    /// Unite, intersect or subtract two solids.
    Boolean(BooleanArgs),
    /// Add reference geometry — a point, an axis or a plane — built from
    /// selected points, edges and planes.
    #[operation(label = "Reference")]
    AddDatum(AddDatumArgs),
}
