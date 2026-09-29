//! The elementary structures the kernel's operations work with: parts,
//! operations, and programs made of operations. It defines what an
//! operation is, but no operation itself: every operation lives in a
//! `geop-ops-*` crate of its own, built on this one as a plugin — placing
//! sketches in `geop-ops-sketch`, reference geometry in `geop-ops-datums`,
//! extrude and revolve in `geop-ops-extrude-revolve`, booleans in
//! `geop-ops-booleans`.
//!
//! - [`part`]: a complete CAD [`Part`] — a [`geop_core_topology::Model`],
//!   the sketches and datums used to build it, starting with the frame
//!   [`ORIGIN`], and a stable name for every entity in it (see
//!   [Topological naming](#topological-naming)).
//! - [`operation`]: what an operation is. Built, a step maps a part and
//!   its arguments to a new part; edited, it shows a
//!   [`Form`](ui::Form) — fields and visuals — and has its fields set:
//!
//!   ```text
//!   (part, args)                 -> form
//!   (part, args, field, value)   -> args
//!   ```
//!
//!   The arguments are plain, serializable design data: numbers, choices,
//!   sketches, and references to entities of the part by name
//!   ([`EntityRef`]).
//! - [`ui`]: what an editor exchanges with the operations — the
//!   [`Event`](ui::Event)s it sends (a dialog field used, a click or a drag
//!   as a ray from the eye, a key) and the
//!   [`Presentation`](ui::Presentation) it gets back — and the
//!   [`StepEditor`](ui::StepEditor), which makes every operation answer
//!   them alike: picking entities for a field, dragging a handle, hit tests
//!   against visuals and against the part as drawn
//!   ([`PartView`](ui::PartView)). An editor only renders primitives and
//!   forwards raw input; every decision is made here.
//! - [`Operations`]: a set of operations a program can use, as one
//!   serializable enum; `#[derive(Operations)]` writes it. Which operations
//!   an application offers is its own choice, so the set is defined there,
//!   not here.
//! - [`program`]: a [`Program`], an ordered list of steps, each an
//!   operation with its arguments and an id of its own, that builds a part
//!   from scratch. A program is design data: it serializes to JSON and
//!   back without losing anything, and rebuilding the read-back program
//!   gives the same part, name for name. [`ProgramRunner`] builds it
//!   incrementally, stopping wherever an editor asks.
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

pub mod operation;
pub mod part;
pub mod program;
pub mod ui;

pub use part::{
    DatumId, EdgeDescription, FaceDescription, NameRegistry, Namer, ORIGIN, Part, PartDescription,
    PlacedSketch, RefId, SketchId, validate_operation_id,
};

pub use operation::{EntityRef, Operation, OperationInfo, Operations};
pub use program::{Program, ProgramRunner, Step, StepResult};

#[doc(hidden)]
/// What `#[derive(Operations)]` writes refers to, so a crate using it needs
/// no dependencies of its own for it.
pub mod __private {
    pub use geop_core_math::{
        geop_error::{GeopError, GeopResult},
        scalars::Scalar,
    };
}
