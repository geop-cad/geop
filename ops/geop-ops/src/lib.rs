//! The model layer the kernel's operations build on: what a part is made
//! of between the steps of its program, and how a step refers to it. It
//! defines no operation itself: every operation lives in a `geop-ops-*`
//! crate of its own, implementing [`geop_core_program::operation::Operation`] —
//! sketches in `geop-ops-sketch`, extrude and revolve in
//! `geop-ops-extrude-revolve`, booleans in `geop-ops-booleans`.
//!
//! A part is built as [`geop_core_program`] builds any program, target by target.
//! Its targets are of a few kinds:
//!
//! - [`model()`]: the [`NamedModel`] — topology, a stable name for every
//!   entity in it (see [Topological naming](#topological-naming)), and the
//!   features that made its faces — defined by the first step that builds
//!   a body and revised by every later one.
//! - [`sketch`]: each sketch, with the plane it was placed on, defined by
//!   its step.
//! - [`sketch3d`]: each 3-D sketch, solved, defined by its step — which
//!   also builds it into the model, as a wire.
//!
//! The program's parameters are its inputs, [`ParamValue`]s resolved from
//! its [`parameters::Parameters`]; where its placed parts are is its state.
//!
//! - [`operation`]: how a step refers to what it builds on — entities by
//!   name ([`EntityRef`]) — and what they are, resolved against a
//!   [`operation::Lookup`]: a step's generator's, which records what it
//!   read, or an editor's.
//! - [`parameters`]: the named values a program's design is given by —
//!   numbers, tables of variants, the part's colour — and the formulas that
//!   read them.
//! - [`placed`]: the parts placed in a part, and the library they are
//!   built with.
//! - [`solids`]: the solids a part shows, wherever they are placed, each
//!   of its own part's material, and their mass properties.
//! - [`solving`]: the part's state solved as an editor does — settled,
//!   dragged, set — and asked how free its parts are and what conflicts.
//! - [`ui`]: the vocabulary an editor and the operations exchange
//!   ([`geop_core_program::editing`]), building blocks for forms, and what
//!   a step shows of the part besides its model: annotations drawn on it,
//!   entries of its structure ([`ui::annotations`], [`ui::listing`]).
//! - [`output`]: what is made of a part rather than in it — files written
//!   from it, questions answered about it — by the exporters and
//!   inspectors an editor offers.
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
//!
//! A part placed in another is named by the step that placed it, and its
//! entities by their names in it behind that one:
//! `bolt/extrude(head,end)` (see [`operation::INSTANCE_SEPARATOR`]). The
//! part placed keeps every name it has in its own file, so a reference into
//! it is as stable as one into the part itself.

/// The scalar a program holds its design data in — a sketch's points, the
/// poses of the parts it places, the values of its mates: what a file
/// holds, read as sharp values and written as their midpoints. A part is
/// built in it; algorithms take any scalar.
pub type Design = geop_core_math::scalars::ScalInF64;

/// What [`part_operation!`] expands to refers to, whichever crates the
/// operation's own depends on.
#[doc(hidden)]
pub mod __program {
    pub use geop_core_math::geop_error::GeopResult as __GeopResult;
    pub use geop_core_program::*;
}
pub mod model;
pub mod operation;
pub mod output;
pub mod parameters;
pub mod placed;
pub mod program;
pub mod solids;
pub mod solving;
pub mod ui;

pub use model::{
    BodyNames, BooleanOp, CosmeticThread, EdgeDescription, FaceDescription, Feature, FeatureTool,
    NameRegistry, NamedModel, Namer, ParamValue, PartDescription, PlacedSketch, RefId, Threads,
    ToolReach, draw_threads, model, sketch, threads, validate_operation_id,
};
pub use operation::{EntityRef, ORIGIN, origin_axis, origin_plane};
