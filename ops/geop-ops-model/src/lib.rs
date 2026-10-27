//! The vocabulary every operation shares: every target a step of one
//! operation reads of another, the references into it, and how it is
//! drawn. An operation crate depends on `core/`, `lib/` and this crate
//! alone, so what one operation reads of another lives here, and the
//! operation that produces it keeps only its arguments, form and editor.
//!
//! A part's topology: the one [`NamedModel`] its steps build, revised by
//! each in turn ([`model`]) — every vertex, edge, face and solid with a
//! stable name ([`Namer`]), and the features the steps made — and the
//! kinds of element a step refers to in it ([`elements`]): vertices,
//! edges, faces, solids, each picked as a dialog asks for it — and what
//! they are used as ([`geometry`]): points, lines, planes, circles,
//! curves, bodies. A step that builds into the model is a
//! [`ModelOperation`]; one that builds a tool says how it meets the solid
//! it is for with a [`Combine`], and how far it goes with an [`Extent`].
//!
//! The other targets of a part: its sketches ([`sketch`]) and 3-D sketches
//! ([`sketch3d`]), its datums ([`datums`]), its parameters and properties
//! ([`parameters`]), the parts placed in it with their solids and bill of
//! materials ([`placed`]); and what every operation picks and lays out
//! alike: any element of the part ([`selection`]), an element of a placed
//! part with its frame ([`mate_entity`]), instances in a row ([`spacing`]).
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
mod combine;
pub mod curves;
pub mod datums;
mod describe;
mod draw;
mod edit;
pub mod elements;
mod extent;
mod feature;
pub mod geometry;
mod ids;
pub mod mate_entity;
mod mesh;
mod named_model;
mod names;
pub mod notes;
pub mod parameters;
pub mod placed;
pub mod records;
mod resolve;
pub mod selection;
pub mod sketch;
pub mod sketch3d;
pub mod spacing;
pub mod tangent;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod threads;

pub use build::{ModelOperation, build_model};
pub use combine::{Combine, SolidRef, Tool, solid};
pub use curves::{Chain, ChainCurve, Profile};
pub use describe::{EdgeDescription, FaceDescription, PartDescription};
pub use draw::ModelDrawer;
pub use edit::BodyNames;
pub use elements::{ChainElement, Edge, EdgeChain, Face, ModelElement, Solid, Vertex, read_model};
pub use extent::{End, Extent, face_hull, hull, reach_past, spread_along};
pub use feature::{BooleanOp, Feature, FeatureFaces, FeatureTool, ToolReach};
pub use geometry::{
    Body, Circle, Curve, Line, Path, Plane, Point, Round, frame_along, resolve_bodies,
};
pub use ids::RefId;
pub use mesh::{STL_QUALITY, SolidMeshes, stl_bytes};
pub use named_model::{NamedModel, element, model};
pub use names::{LONGEST_ARGUMENT, NameRegistry, Namer, digest, operation_of, origin_of};
pub use notes::{CutWire, SolidNote, SolidNotes, notes_of, solid_notes};
pub use records::Records;
pub use threads::{CosmeticThread, Threads, draw_threads, read_threads, threads};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what the model offers an editor to `r`: the part's model drawn, and
/// its solids and faces listed ([`ModelDrawer`]); its datums drawn and
/// listed ([`datums::DatumsDrawer`]); its sketches drawn and listed
/// ([`sketch::SketchDrawer`]); the steps defining its parameters,
/// tables and properties, and the part drawn in its colour (see
/// [`parameters`]).
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.drawer(ModelDrawer::default())?
        .drawer(datums::DatumsDrawer)?
        .drawer(sketch::SketchDrawer)?
        .operation(parameters::DefineParameters)?
        .operation(parameters::LookupTable)?
        .operation(parameters::PartProperties)?
        .drawer(parameters::PartColor)
}
