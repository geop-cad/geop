//! The operations the editor offers, as the one set its programs are
//! written in: [`PartOperation`], and the program types over it.
//!
//! Every operation is defined by a crate of its own — placing sketches in
//! `geop_ops_sketch`, datums in `geop_ops_datums`, extrude and revolve in
//! `geop_ops_extrude_revolve`, booleans in `geop_ops_booleans`, fillets and
//! chamfers in `geop_ops_fillet`, shells in `geop_ops_shell`, edits of
//! existing bodies in `geop_ops_edit`, placed parts and their patterns in `geop_ops_assembly`,
//! patterns, mirrors and moves of bodies in `geop_ops_pattern`,
//! wire harness routes in `geop_ops_harness`,
//! holes and threads in `geop_ops_hole`, surfaces in `geop_ops_surface`,
//! 3-D sketches in `geop_ops_sketch3d`, ribs, lips, grooves and drafts in
//! `geop_ops_plastic`, sheet metal in `geop_ops_sheetmetal`,
//! subdivision surfaces in `geop_ops_subd`, drawings in `geop_ops_drawing`,
//! STEP import in `geop_ops_step`.
//! Which of them an editor offers is the editor's choice, made here.

use geop_ops::Operations;
use geop_ops_assembly::{AddPart, AddPartArgs, PartPattern, PartPatternArgs};
use geop_ops_booleans::{Boolean, BooleanArgs, Split, SplitArgs};
use geop_ops_datums::{AddDatum, AddDatumArgs};
use geop_ops_drawing::{Drawing, DrawingArgs};
use geop_ops_edit::{
    DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs, ProjectCurve, ProjectCurveArgs,
};
use geop_ops_extrude_revolve::{
    Extrude, ExtrudeArgs, Loft, LoftArgs, Revolve, RevolveArgs, Sweep, SweepArgs,
};
use geop_ops_fillet::{Chamfer, ChamferArgs, Fillet, FilletArgs};
use geop_ops_harness::{Route, RouteArgs};
use geop_ops_hole::{Hole, HoleArgs, Thread, ThreadArgs};
use geop_ops_pattern::{
    CircularPattern, CircularPatternArgs, LinearPattern, LinearPatternArgs, Mirror, MirrorArgs,
    MoveBody, MoveBodyArgs,
};
use geop_ops_plastic::{Draft, DraftArgs, Groove, GrooveArgs, Lip, LipArgs, Rib, RibArgs};
use geop_ops_sheetmetal::{
    BaseFlange, BaseFlangeArgs, EdgeFlange, EdgeFlangeArgs, FlatPattern, FlatPatternArgs, Hem,
    HemArgs, SheetCut, SheetCutArgs,
};
use geop_ops_shell::{Shell, ShellArgs};
use geop_ops_sketch::{AddSketch, AddSketchArgs};
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs};
use geop_ops_step::{ImportStep, ImportStepArgs};
use geop_ops_subd::{Subd, SubdArgs};
use geop_ops_surface::{
    BoundarySurface, BoundarySurfaceArgs, ExtendSurface, ExtendSurfaceArgs, Knit, KnitArgs,
    OffsetSurface, OffsetSurfaceArgs, Thicken, ThickenArgs, TrimSurface, TrimSurfaceArgs,
};
use geop_ops_surface::{NetworkSurface, NetworkSurfaceArgs};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod assembly_scale_tests;
#[cfg(test)]
mod assembly_tests;
#[cfg(test)]
mod bom_tests;
#[cfg(test)]
mod datum_tests;
#[cfg(test)]
mod drawing_tests;
#[cfg(test)]
mod edit_tests;
#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod feature_pattern_tests;
#[cfg(test)]
mod fillet_tests;
#[cfg(test)]
mod harness_tests;
#[cfg(test)]
mod hole_tests;
#[cfg(test)]
mod inspect_tests;
#[cfg(test)]
mod pattern_tests;
#[cfg(test)]
mod plastic_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
pub(crate) mod regression_tests;
#[cfg(test)]
mod set_tests;
#[cfg(test)]
mod sheetmetal_tests;
#[cfg(test)]
mod shell_tests;
#[cfg(test)]
mod sketch3d_tests;
#[cfg(test)]
mod sketch_tests;
#[cfg(test)]
mod step_tests;
#[cfg(test)]
mod stress_tests;
#[cfg(test)]
mod subd_tests;
#[cfg(test)]
mod surface_tests;
#[cfg(test)]
mod sweep_loft_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod urdf_tests;
#[cfg(test)]
mod view_tests;

/// An operation the editor offers, together with its arguments, not yet
/// applied to any part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
pub enum PartOperation {
    /// Draw a sketch on a plane of a coordinate system, a datum plane or a
    /// planar face.
    #[operation(label = "Sketch", group = Sketch, tier = Big)]
    AddSketch(AddSketchArgs),
    /// Draw points, lines, arcs and splines in space: paths to sweep along.
    #[operation(label = "3-D sketch", group = Sketch, tier = Small)]
    AddSketch3d(AddSketch3dArgs),
    /// Add reference geometry — a point, an axis, a plane or a coordinate
    /// system — built from selected points, edges and planes.
    #[operation(label = "Reference", group = Sketch, tier = Small)]
    AddDatum(AddDatumArgs),
    /// Sweep a sketch's area along its plane's normal into a solid, or its
    /// curves into faces.
    #[operation(group = Solid, tier = Big)]
    Extrude(ExtrudeArgs),
    /// Sweep a sketch's area around one of its lines into a solid, or its
    /// curves into faces.
    #[operation(group = Solid, tier = Big)]
    Revolve(RevolveArgs),
    /// Sweep a sketch's area along the curves of another sketch into a
    /// solid, or its curves into faces.
    #[operation(group = Solid, tier = Small)]
    Sweep(SweepArgs),
    /// Build a solid through the areas of several sketches, or faces
    /// through their curves.
    #[operation(group = Solid, tier = Small)]
    Loft(LoftArgs),
    /// Drill simple, counterbored, countersunk or tapped holes at points on
    /// a planar face, sized by ISO tables or by hand.
    #[operation(group = Features, tier = Big)]
    Hole(HoleArgs),
    /// Put an ISO metric thread on a cylindrical face: recorded as a
    /// cosmetic thread, or modelled.
    #[operation(group = Features, tier = Small)]
    Thread(ThreadArgs),
    /// Grow a thin wall from an open sketch profile up to a solid's faces,
    /// and join it.
    #[operation(group = Features, tier = Menu)]
    Rib(RibArgs),
    /// Round a solid's straight and circular edges.
    #[operation(group = Features, tier = Big)]
    Fillet(FilletArgs),
    /// Bevel a solid's straight and circular edges.
    #[operation(group = Features, tier = Small)]
    Chamfer(ChamferArgs),
    /// Hollow a solid out to walls of one thickness, open where faces are
    /// picked.
    #[operation(group = Features, tier = Small)]
    Shell(ShellArgs),
    /// Tilt planar faces about a neutral plane, so the part comes out of
    /// its mould.
    #[operation(group = Features, tier = Menu)]
    Draft(DraftArgs),
    /// Raise a lip along the rim of one half of an enclosure.
    #[operation(group = Features, tier = Menu)]
    Lip(LipArgs),
    /// Cut the groove that takes a lip into the rim of the other half.
    #[operation(group = Features, tier = Menu)]
    Groove(GrooveArgs),
    /// Unite, intersect or subtract two solids.
    #[operation(group = Bodies, tier = Big)]
    Boolean(BooleanArgs),
    /// Cut a solid into pieces with a face standing on its own.
    #[operation(group = Bodies, tier = Menu)]
    Split(SplitArgs),
    /// Copy bodies in a row along a direction, or in a grid along two.
    #[operation(label = "Linear pattern", group = Bodies, tier = Big)]
    LinearPattern(LinearPatternArgs),
    /// Copy bodies turned around an axis.
    #[operation(label = "Circular pattern", group = Bodies, tier = Small)]
    CircularPattern(CircularPatternArgs),
    /// Mirror bodies in a plane.
    #[operation(group = Bodies, tier = Small)]
    Mirror(MirrorArgs),
    /// Move bodies, or a copy of them, turned and shifted.
    #[operation(label = "Move body", group = Bodies, tier = Menu)]
    MoveBody(MoveBodyArgs),
    /// Delete solids, and faces standing on their own.
    #[operation(label = "Delete body", group = Bodies, tier = Menu)]
    DeleteBody(DeleteBodyArgs),
    /// Span a face standing on its own between two edges, or fill a closed
    /// loop of edges — optionally tangent to the flat faces along them.
    #[operation(label = "Boundary surface", group = Surface, tier = Small)]
    BoundarySurface(BoundarySurfaceArgs),
    /// Copy faces a distance along their normals into a face standing on
    /// its own.
    #[operation(label = "Offset surface", group = Surface, tier = Menu)]
    OffsetSurface(OffsetSurfaceArgs),
    /// Make a solid of a face standing on its own, a thickness on either
    /// side of it or on both.
    #[operation(group = Surface, tier = Menu)]
    Thicken(ThickenArgs),
    /// Join faces standing on their own along the edges where they meet,
    /// into a solid once they close up.
    #[operation(group = Surface, tier = Menu)]
    Knit(KnitArgs),
    /// Cut a face standing on its own back to one side of another face.
    #[operation(label = "Trim surface", group = Surface, tier = Menu)]
    TrimSurface(TrimSurfaceArgs),
    /// Carry a face standing on its own on past one of its edges.
    #[operation(label = "Extend surface", group = Surface, tier = Menu)]
    ExtendSurface(ExtendSurfaceArgs),
    /// Copy a face out of its body into a face standing on its own.
    #[operation(label = "Extract face", group = Surface, tier = Menu)]
    ExtractFace(ExtractFaceArgs),
    /// Project a sketch's curves onto a face, dividing the face along them.
    #[operation(label = "Project curve", group = Surface, tier = Menu)]
    ProjectCurve(ProjectCurveArgs),
    /// Start a sheet-metal body: a plate from a sketch's area, or a bent
    /// strip from a chain of lines and arcs.
    #[operation(label = "Base flange", group = SheetMetal, tier = Big)]
    BaseFlange(BaseFlangeArgs),
    /// Bend a flange up from a straight edge of a sheet-metal body.
    #[operation(label = "Edge flange", group = SheetMetal, tier = Small)]
    EdgeFlange(EdgeFlangeArgs),
    /// Cut holes and notches through a sheet-metal body along a sketch,
    /// across its bends as they lie unrolled.
    #[operation(label = "Sheet-metal cut", group = SheetMetal, tier = Menu)]
    SheetCut(SheetCutArgs),
    /// Fold an edge of a sheet-metal body right back over it.
    #[operation(group = SheetMetal, tier = Menu)]
    Hem(HemArgs),
    /// Unfold a sheet-metal body into its flat pattern.
    #[operation(label = "Flat pattern", group = SheetMetal, tier = Menu)]
    FlatPattern(FlatPatternArgs),
    /// Shape a freeform body by dragging the vertices, edges and faces of a
    /// control cage, built as its smooth subdivision surface.
    #[operation(label = "SubD", group = Surface, tier = Big)]
    Subd(SubdArgs),
    /// Place the part another program file builds, and mate it to what is
    /// already there.
    #[operation(label = "Part", group = Assembly, tier = Big)]
    AddPart(AddPartArgs),
    /// Place copies of a placed part in a row along a line, or round an
    /// axis.
    #[operation(label = "Part pattern", group = Assembly, tier = Menu)]
    PartPattern(PartPatternArgs),
    /// Route a bundle of wires from a connector through clips to another
    /// connector, its bends checked and every wire's cut length reported.
    #[operation(group = Assembly, tier = Small)]
    Route(RouteArgs),
    /// Describe a 2-D drawing of the part — views with hidden lines, a
    /// section and a title block — annotated on its sheet with dimensions,
    /// notes and centre marks, and downloaded as SVG or DXF.
    #[operation(group = Output, tier = Big)]
    Drawing(DrawingArgs),
    /// Add the solids and sheets of a STEP file next to the program.
    #[operation(label = "Import STEP", group = Output, tier = Small)]
    ImportStep(ImportStepArgs),
    /// Span a face standing on its own through a network of curves — u
    /// curves crossing v curves — running along every one of them.
    #[operation(label = "UV surface", group = Surface, tier = Small)]
    NetworkSurface(NetworkSurfaceArgs),
}

/// A program of the editor's operations.
pub type Program = geop_ops::Program<PartOperation>;
/// A step of a [`Program`].
pub type Step = geop_ops::Step<PartOperation>;
/// Builds a [`Program`] incrementally.
pub type ProgramRunner<S> = geop_ops::ProgramRunner<S, PartOperation>;
/// The program files a [`Program`] places parts from: `F`'s, and the
/// standard parts (see [`crate::stdlib`]). Made with
/// `Workspace::new(WithStandardParts(files))`.
pub type Workspace<S, F = std::collections::BTreeMap<String, String>> =
    geop_ops::Workspace<PartOperation, S, crate::stdlib::WithStandardParts<F>>;
