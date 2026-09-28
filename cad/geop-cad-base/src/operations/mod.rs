//! The operations the editor offers, as the one set its programs are
//! written in: [`PartOperation`], and the program types over it.
//!
//! Every operation is defined by a crate of its own — placing sketches in
//! `geop_ops_sketch`, datums in `geop_ops_datums`, extrude and revolve in
//! `geop_ops_extrude_revolve`, booleans in `geop_ops_booleans`. Which of them
//! an editor offers is the editor's choice, made here.

use geop_ops::Operations;
use geop_ops_booleans::{Boolean, BooleanArgs};
use geop_ops_datums::{AddDatum, AddDatumArgs};
use geop_ops_extrude_revolve::{Extrude, ExtrudeArgs, Revolve, RevolveArgs};
use geop_ops_sketch::{AddSketch, AddSketchArgs};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod datum_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod schema_tests;
#[cfg(test)]
mod tests;

/// An operation the editor offers, together with its arguments, not yet
/// applied to any part.
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

/// A program of the editor's operations.
pub type Program = geop_ops::Program<PartOperation>;
/// A step of a [`Program`].
pub type Step = geop_ops::Step<PartOperation>;
/// An edit of a [`Program`].
pub type ProgramEdit = geop_ops::ProgramEdit<PartOperation>;
/// Builds a [`Program`] incrementally.
pub type ProgramRunner<S> = geop_ops::ProgramRunner<S, PartOperation>;
