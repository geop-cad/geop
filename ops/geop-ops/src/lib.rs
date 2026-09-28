//! The elementary structures the kernel's operations work with: parts,
//! operations, and programs made of operations. It defines what an
//! operation is, but no operation itself: every operation lives in a
//! `geop-ops-*` crate of its own, built on this one as a plugin — placing
//! sketches in `geop-ops-sketch`, reference geometry in `geop-ops-datums`,
//! extrude and revolve in `geop-ops-extrude-revolve`, booleans in
//! `geop-ops-booleans`.
//!
//! - [`Part`]: a complete CAD part — a [`geop_core_topology::Model`], the
//!   sketches and datums used to build it, and a stable name for every
//!   entity in it (see [Topological naming](#topological-naming)).
//! - [`operation`]: what an operation is. An operation maps a part and its
//!   arguments to a new part, together with what an editor needs to edit
//!   the step interactively:
//!
//!   ```text
//!   (part, args) -> (part, dialog, handles)
//!   ```
//!
//!   The arguments ([`OperationArgs`]) are plain, serializable design data:
//!   numbers, choices, sketches, and references to entities of the part by
//!   name ([`EntityRef`]). Their schema ([`ArgSchema`]) tells an editor how
//!   each can be entered — a slider, a choice, a pick in the viewport — and
//!   algebraic types (a datum's [`Construction`](ArgKind::Construction), a
//!   [`Combine`](ArgKind::Combine))
//!   make it a dialog whose fields follow what was chosen. The [`Dialog`]
//!   adds what only the part can tell: which of the choices fit what is
//!   selected in it, what a picked entity can be used as. The [`Handle`]s
//!   are the step's values placed in the 3-D view: how each can be dragged,
//!   and which argument that changes.
//! - [`Operations`]: a set of operations a program can use, as one
//!   serializable enum; `#[derive(Operations)]` writes it. Which operations
//!   an application offers is its own choice, so the set is defined there,
//!   not here.
//! - [`Program`]: an ordered list of steps, each an operation with its
//!   arguments and an id of its own, that builds a part from scratch. A
//!   program is design data: it serializes to JSON and back without losing
//!   anything, and rebuilding the read-back program gives the same part,
//!   name for name. It changes only through [`Program::update`], and
//!   [`ProgramRunner`] builds it incrementally, stopping wherever an editor
//!   asks.
//!
//! # Parts
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

// The derives in `geop_ops_derive` name this crate by its path, which has
// to resolve inside it too.
extern crate self as geop_ops;

mod datum;
mod describe;
mod edit;
mod euler;
mod ids;
mod names;
pub mod operation;
mod part;
mod program;
mod resolve;
mod sketch;

pub use describe::{EdgeDescription, FaceDescription, PartDescription};
pub use ids::{DatumId, RefId, SketchId};
pub use names::{NameRegistry, Namer, validate_operation_id};
pub use part::Part;
pub use sketch::PlacedSketch;

pub use operation::{
    ArgKind, ArgSchema, Dialog, EntityRef, Handle, Operation, OperationArgs, OperationSchema,
    Operations, WorldAxis,
};
pub use program::{Program, ProgramEdit, ProgramRunner, Step, StepDialog, StepHandle, StepResult};
