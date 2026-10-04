//! The operations the editor offers, as the one set its programs are
//! written in: [`PartOperation`], and the program types over it.
//!
//! Every operation is defined by a crate of its own — placing sketches in
//! `geop_ops_sketch`, datums in `geop_ops_datums`, extrude and revolve in
//! `geop_ops_extrude_revolve`, booleans in `geop_ops_booleans`, placed parts
//! in `geop_ops_assembly`. Which of them
//! an editor offers is the editor's choice, made here.

use geop_ops::Operations;
use geop_ops_assembly::{AddPart, AddPartArgs};
use geop_ops_booleans::{Boolean, BooleanArgs, Split, SplitArgs};
use geop_ops_datums::{AddDatum, AddDatumArgs};
use geop_ops_extrude_revolve::{
    Extrude, ExtrudeArgs, Loft, LoftArgs, Revolve, RevolveArgs, Sweep, SweepArgs,
};
use geop_ops_sketch::{AddSketch, AddSketchArgs};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod assembly_tests;
#[cfg(test)]
mod datum_tests;
#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod set_tests;
#[cfg(test)]
mod sketch_tests;
#[cfg(test)]
mod sweep_loft_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod view_tests;

/// An operation the editor offers, together with its arguments, not yet
/// applied to any part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
pub enum PartOperation {
    /// Draw a sketch on a plane of a coordinate system, a datum plane or a
    /// planar face.
    #[operation(label = "Sketch")]
    AddSketch(AddSketchArgs),
    /// Sweep a sketch's area along its plane's normal into a solid, or its
    /// curves into faces.
    Extrude(ExtrudeArgs),
    /// Sweep a sketch's area around one of its lines into a solid, or its
    /// curves into faces.
    Revolve(RevolveArgs),
    /// Sweep a sketch's area along the curves of another sketch into a
    /// solid, or its curves into faces.
    Sweep(SweepArgs),
    /// Build a solid through the areas of several sketches, or faces
    /// through their curves.
    Loft(LoftArgs),
    /// Unite, intersect or subtract two solids.
    Boolean(BooleanArgs),
    /// Cut a solid into pieces with a face standing on its own.
    Split(SplitArgs),
    /// Add reference geometry — a point, an axis, a plane or a coordinate
    /// system — built from selected points, edges and planes.
    #[operation(label = "Reference")]
    AddDatum(AddDatumArgs),
    /// Place the part another program file builds, and mate it to what is
    /// already there.
    #[operation(label = "Part")]
    AddPart(AddPartArgs),
}

/// A program of the editor's operations.
pub type Program = geop_ops::Program<PartOperation>;
/// A step of a [`Program`].
pub type Step = geop_ops::Step<PartOperation>;
/// Builds a [`Program`] incrementally.
pub type ProgramRunner<S> = geop_ops::ProgramRunner<S, PartOperation>;
/// The program files a [`Program`] places parts from.
pub type Workspace<S, F = std::collections::BTreeMap<String, String>> =
    geop_ops::Workspace<PartOperation, S, F>;
