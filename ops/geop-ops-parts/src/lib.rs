//! Building a [`Part`](geop_core_part::Part) from a script of CAD operations.
//!
//! - [`operation`]: every operation a part is built with — adding a sketch,
//!   extruding or revolving one, combining two solids, adding reference
//!   geometry — as an
//!   [`Operation`] with plain, serializable arguments that refer to
//!   existing entities only by their stable names (see `geop_core_part`'s
//!   crate docs for the naming scheme).
//! - [`Program`]: an ordered list of such operations, each with an id of
//!   its own, that builds a part from scratch. A program is design data: it
//!   serializes to JSON and back without losing anything, and rebuilding
//!   the read-back program gives the same part, name for name. It changes
//!   only through [`Program::update`], and [`ProgramRunner`] builds it
//!   incrementally, stopping wherever an editor asks.
//! - [`OperationSchema`]: what every operation takes, so an editor can
//!   offer any operation without knowing it by hand.
//! - [`examples`]: a few programs built in Rust, used by the tests and as a
//!   reference for writing new ones.

// The derives in `geop_ops_parts_derive` name this crate by its path, which
// has to resolve inside it too.
extern crate self as geop_ops_parts;

pub mod examples;
pub mod operation;
mod program;

pub use operation::{
    AddDatum, AddDatumArgs, AddSketch, AddSketchArgs, ArgKind, ArgSchema, Boolean, BooleanArgs,
    Combine, Construction, EntityRef, Extrude, ExtrudeArgs, Operation, OperationArgs,
    OperationSchema, PartOperation, Revolve, RevolveArgs, WorldAxis,
};
pub use program::{Program, ProgramEdit, ProgramRunner, Step, StepHandle, StepResult};
