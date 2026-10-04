//! The operations the editor offers, as the one set its programs are
//! written in: [`PartOperation`], and the program types over it.
//!
//! Every operation is defined by a crate of its own — placing sketches in
//! `geop_ops_sketch`, datums in `geop_ops_datums`, extrude and revolve in
//! `geop_ops_extrude_revolve`, booleans in `geop_ops_booleans`, fillets and
//! chamfers in `geop_ops_fillet`, shells in `geop_ops_shell`, edits of
//! existing bodies in `geop_ops_edit`, placed parts in `geop_ops_assembly`,
//! surfaces in `geop_ops_surface`.
//! Which of them an editor offers is the editor's choice, made here.

use geop_ops::Operations;
use geop_ops_assembly::{AddPart, AddPartArgs};
use geop_ops_booleans::{Boolean, BooleanArgs, Split, SplitArgs};
use geop_ops_datums::{AddDatum, AddDatumArgs};
use geop_ops_edit::{
    DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs, ProjectCurve, ProjectCurveArgs,
};
use geop_ops_extrude_revolve::{
    Extrude, ExtrudeArgs, Loft, LoftArgs, Revolve, RevolveArgs, Sweep, SweepArgs,
};
use geop_ops_fillet::{Chamfer, ChamferArgs, Fillet, FilletArgs};
use geop_ops_shell::{Shell, ShellArgs};
use geop_ops_sketch::{AddSketch, AddSketchArgs};
use geop_ops_surface::{
    BoundarySurface, BoundarySurfaceArgs, ExtendSurface, ExtendSurfaceArgs, Knit, KnitArgs,
    OffsetSurface, OffsetSurfaceArgs, Thicken, ThickenArgs, TrimSurface, TrimSurfaceArgs,
};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod assembly_tests;
#[cfg(test)]
mod datum_tests;
#[cfg(test)]
mod edit_tests;
#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod fillet_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod set_tests;
#[cfg(test)]
mod shell_tests;
#[cfg(test)]
mod sketch_tests;
#[cfg(test)]
mod stress_tests;
#[cfg(test)]
mod surface_tests;
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
    /// Delete solids, and faces standing on their own.
    #[operation(label = "Delete body")]
    DeleteBody(DeleteBodyArgs),
    /// Copy a face out of its body into a face standing on its own.
    #[operation(label = "Extract face")]
    ExtractFace(ExtractFaceArgs),
    /// Project a sketch's curves onto a face, dividing the face along them.
    #[operation(label = "Project curve")]
    ProjectCurve(ProjectCurveArgs),
    /// Round a solid's straight and circular edges.
    Fillet(FilletArgs),
    /// Bevel a solid's straight and circular edges.
    Chamfer(ChamferArgs),
    /// Hollow a solid out to walls of one thickness, open where faces are
    /// picked.
    Shell(ShellArgs),
    /// Add reference geometry — a point, an axis, a plane or a coordinate
    /// system — built from selected points, edges and planes.
    #[operation(label = "Reference")]
    AddDatum(AddDatumArgs),
    /// Place the part another program file builds, and mate it to what is
    /// already there.
    #[operation(label = "Part")]
    AddPart(AddPartArgs),
    /// Span a face standing on its own between two edges, or fill a closed
    /// loop of edges — optionally tangent to the flat faces along them.
    #[operation(label = "Boundary surface")]
    BoundarySurface(BoundarySurfaceArgs),
    /// Copy faces a distance along their normals into a face standing on
    /// its own.
    #[operation(label = "Offset surface")]
    OffsetSurface(OffsetSurfaceArgs),
    /// Make a solid of a face standing on its own, a thickness on either
    /// side of it or on both.
    Thicken(ThickenArgs),
    /// Join faces standing on their own along the edges where they meet,
    /// into a solid once they close up.
    Knit(KnitArgs),
    /// Cut a face standing on its own back to one side of another face.
    #[operation(label = "Trim surface")]
    TrimSurface(TrimSurfaceArgs),
    /// Carry a face standing on its own on past one of its edges.
    #[operation(label = "Extend surface")]
    ExtendSurface(ExtendSurfaceArgs),
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
