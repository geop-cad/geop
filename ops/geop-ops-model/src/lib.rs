//! A part's topology: the one [`NamedModel`] its steps build, revised by
//! each in turn ([`model`]) — every vertex, edge, face and solid with a
//! stable name ([`Namer`]), and the features the steps made — and the
//! kinds of element a step refers to in it ([`elements`]): vertices,
//! edges, faces, solids, each picked as a dialog asks for it — and what
//! they are used as ([`geometry`]): points, lines, planes, circles,
//! curves, bodies. A step that builds into the model is a
//! [`ModelOperation`].
//!
//! # Topological naming
//!
//! A name says *how an entity came to be*, never *when*: it is built only
//! from inputs that stay the same when a part is rebuilt after an edit
//! upstream — operation ids, sketch element ids, the names of the entities
//! an operation consumed, and positions counted along those. Never from an
//! internal id or the order in which an algorithm happened to create
//! things. So a program that refers to `extrude(box,end)` keeps meaning the
//! same face when the box gets taller, and names are stable text that diffs
//! well.
//!
//! Every name has the form `kind(operation,arg,...)` (see [`Namer`]):
//! `kind` is the operation that created the entity, `operation` the id of
//! the program step that ran it, and the arguments identify the entity
//! within that step. Operation ids are restricted to
//! [`validate_operation_id`](geop_core_program::program::validate_operation_id)'s
//! alphabet, so the arguments of a name — which
//! may themselves be names — can always be told apart.

// Library code reports what goes wrong as an error, never by panicking;
// tests may unwrap.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable
    )
)]

mod build;
pub mod curves;
mod describe;
mod draw;
mod edit;
pub mod elements;
mod euler;
mod feature;
pub mod geometry;
mod ids;
mod mesh;
mod named_model;
mod names;
pub mod notes;
pub mod records;
mod resolve;
pub mod tangent;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod threads;

pub use build::{ModelOperation, build_model};
pub use curves::{Chain, ChainCurve, Profile};
pub use describe::{EdgeDescription, FaceDescription, PartDescription};
pub use draw::ModelDrawer;
pub use edit::BodyNames;
pub use elements::{ChainElement, Edge, EdgeChain, Face, ModelElement, Solid, Vertex, read_model};
pub use feature::{BooleanOp, Feature, FeatureFaces, FeatureTool, ToolReach};
pub use geometry::{
    Body, Circle, Curve, Line, Path, Plane, Point, Round, frame_along, resolve_bodies,
};
pub use ids::RefId;
pub use mesh::SolidMeshes;
pub use named_model::{NamedModel, element, model};
pub use names::{LONGEST_ARGUMENT, NameRegistry, Namer, digest, operation_of, origin_of};
pub use notes::{CutWire, SolidNote, SolidNotes, notes_of, solid_notes};
pub use records::Records;
pub use threads::{CosmeticThread, Threads, draw_threads, read_threads, threads};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what the model offers an editor to `r`: the part's model drawn, and
/// its solids and faces listed ([`ModelDrawer`]).
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.drawer(ModelDrawer)
}
