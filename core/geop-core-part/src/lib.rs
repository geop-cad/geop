//! A complete CAD part: a [`geop_core_topology::Model`], the sketches and
//! datums used to build it, and a stable name for every entity in it.
//!
//! [`Part`] only ever changes through its own methods, each of which forwards
//! to the identically named `Model` operation and takes the name of every
//! entity it creates. Its fields are private, so a part can't gain an entity
//! without a name, or keep the name of one that no longer exists.
//!
//! # Topological naming
//!
//! A name says *how an entity came to be*, never *when*: it is built only
//! from inputs that stay the same when a part is rebuilt after an edit
//! upstream — operation ids, sketch element ids, the names of the entities an
//! operation consumed, and positions counted along those. Never from an
//! internal id or the order in which an algorithm happened to create things.
//! So a program that refers to `extrude(box,end)` keeps meaning the same face
//! when the box gets taller, and names are stable text that diffs well.
//!
//! Every name has the form `kind(operation,arg,...)` (see [`Namer`]):
//! `kind` is the operation that created the entity, `operation` the id of the
//! program step that ran it, and the arguments identify the entity within
//! that step. The operations document their own arguments; for example:
//!
//! - `extrude(E)` is the solid extrude step `E` built, `extrude(E,start)` and
//!   `extrude(E,end)` its caps, `extrude(E,K,c3)` the side face swept by line
//!   `c3` of sketch `K`, `extrude(E,K,c3,end)` that face's edge on the end cap,
//!   and `extrude(E,K,p1)` the edge swept by sketch point `p1`.
//! - `boolean(B,E1,E2,i,n)` is the `i`-th of the `n` points where edges `E1`
//!   and `E2` (by their names before step `B`) cross, counted along `E1`.
//!
//! Operation ids are restricted to [`validate_operation_id`]'s alphabet, so
//! the arguments of a name — which may themselves be names — can always be
//! told apart.

mod datum;
mod describe;
mod edit;
mod euler;
mod ids;
mod names;
mod part;
mod resolve;
mod sketch;

pub use datum::{Datum, DatumKind};
pub use describe::{EdgeDescription, FaceDescription, PartDescription};
pub use ids::{DatumId, RefId, SketchId};
pub use names::{NameRegistry, Namer, validate_operation_id};
pub use part::Part;
pub use sketch::PlacedSketch;
