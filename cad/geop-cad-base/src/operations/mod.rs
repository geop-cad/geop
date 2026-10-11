//! The operations the editor offers, as the one set its programs are
//! written in ([`registry`]), the program types over it, and the one way a
//! program is built outside the editor ([`build`]).
//!
//! Every operation is defined by a crate of its own — placing sketches in
//! `geop_ops_sketch`, datums in `geop_ops_datums`, 3-D sketches in
//! `geop_ops_sketch3d`, extrude, revolve, sweep and loft in
//! `geop_ops_extrude_revolve`, holes and threads in `geop_ops_hole`, fillets
//! and chamfers in `geop_ops_fillet`, shells in `geop_ops_shell`, drafts,
//! lips, grooves and ribs in `geop_ops_plastic`, booleans and splits in
//! `geop_ops_booleans`, patterns, mirrors and moves of bodies in
//! `geop_ops_pattern`, deleting bodies, extracting faces and projecting
//! curves in `geop_ops_edit`, faces standing on their own in
//! `geop_ops_surface`, subdivision surfaces in `geop_ops_subd`, placed parts
//! and their patterns in `geop_ops_assembly`, harness routes in
//! `geop_ops_harness`, sheet metal in `geop_ops_sheetmetal`, drawings in
//! `geop_ops_drawing`, STEP import in `geop_ops_step`. Which of them an
//! editor offers is the editor's choice, made here — and so are the
//! exporters and inspectors ([`outputs`]): STL, STEP, drawings, URDF, flat
//! patterns and bills of materials; measuring, weighing, interference.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    operation::{Operation, OperationInfo},
    program::ProgramStep,
    program_runner::ProgramRunner,
};
use geop_ops::{
    ParamValue,
    operation::Input,
    output::{Outputs, Stl},
    parameters::Values,
    placed::Library,
    program::{Files, ProgramFile, Registry},
};

use crate::stdlib::WithStandardParts;
use geop_ops_assembly::{AddPart, AddPartArgs, PartPattern, PartPatternArgs};
use geop_ops_bom::{BillOfMaterials, BomCsv};
use geop_ops_booleans::{Boolean, BooleanArgs, Split, SplitArgs};
use geop_ops_datums::{AddDatum, AddDatumArgs};
use geop_ops_drawing::{Drawing, DrawingArgs, DrawingExporter};
use geop_ops_edit::{
    DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs, ProjectCurve, ProjectCurveArgs,
};
use geop_ops_extrude_revolve::{
    Extrude, ExtrudeArgs, Loft, LoftArgs, Revolve, RevolveArgs, Sweep, SweepArgs,
};
use geop_ops_fillet::{Chamfer, ChamferArgs, Fillet, FilletArgs};
use geop_ops_harness::{Route, RouteArgs};
use geop_ops_hole::{Hole, HoleArgs, Thread, ThreadArgs};
use geop_ops_inspect::{Interference, MassProperties, Measure};
use geop_ops_pattern::{
    CircularPattern, CircularPatternArgs, LinearPattern, LinearPatternArgs, Mirror, MirrorArgs,
    MoveBody, MoveBodyArgs,
};
use geop_ops_plastic::{Draft, DraftArgs, Groove, GrooveArgs, Lip, LipArgs, Rib, RibArgs};
use geop_ops_sheetmetal::{
    BaseFlange, BaseFlangeArgs, EdgeFlange, EdgeFlangeArgs, FlatPattern, FlatPatternArgs,
    FlatPatternDxf, Hem, HemArgs, SheetCut, SheetCutArgs,
};
use geop_ops_shell::{Shell, ShellArgs};
use geop_ops_sketch::{AddSketch, AddSketchArgs};
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs};
use geop_ops_step::{ImportStep, ImportStepArgs, Step};
use geop_ops_subd::{Subd, SubdArgs};
use geop_ops_surface::{
    BoundarySurface, BoundarySurfaceArgs, ExtendSurface, ExtendSurfaceArgs, Knit, KnitArgs,
    NetworkSurface, NetworkSurfaceArgs, OffsetSurface, OffsetSurfaceArgs, Thicken, ThickenArgs,
    TrimSurface, TrimSurfaceArgs,
};
use geop_ops_urdf::Urdf;
use serde::Serialize;

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
mod frame_tests;
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
mod regression_tests;
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
pub(crate) mod testing;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod urdf_tests;
#[cfg(test)]
mod view_tests;

/// Arguments that say which operation they are for: what a program written
/// in Rust — an example, a standard part — pushes as a step
/// ([`Push::push`]).
pub trait StepArgs: Serialize {
    /// The operation's kind, as a program stores it.
    const KIND: &'static str;
}

/// The operations the editor offers, in the order it offers them within a
/// group and tier: the registry its programs are run with ([`registry`]),
/// what it lists ([`infos`]), and the arguments of each ([`StepArgs`]).
macro_rules! operations {
    ($($operation:ident($args:ident)),* $(,)?) => {
        /// The operations the editor offers, as its programs are run with.
        pub fn registry() -> GeopResult<Registry> {
            let registry = Registry::default();
            $(let registry = registry.with($operation)?;)*
            Ok(registry)
        }

        /// What each operation the editor offers is, in the order it offers
        /// them.
        pub fn infos() -> Vec<&'static OperationInfo> {
            vec![$(&<$operation as Operation>::INFO),*]
        }

        $(impl StepArgs for $args {
            const KIND: &'static str = <$operation as Operation>::INFO.kind;
        })*
    };
}

operations! {
    AddSketch(AddSketchArgs),
    AddDatum(AddDatumArgs),
    AddSketch3d(AddSketch3dArgs),
    Extrude(ExtrudeArgs),
    Revolve(RevolveArgs),
    Sweep(SweepArgs),
    Loft(LoftArgs),
    Hole(HoleArgs),
    Thread(ThreadArgs),
    Rib(RibArgs),
    Fillet(FilletArgs),
    Chamfer(ChamferArgs),
    Shell(ShellArgs),
    Draft(DraftArgs),
    Lip(LipArgs),
    Groove(GrooveArgs),
    Boolean(BooleanArgs),
    Split(SplitArgs),
    LinearPattern(LinearPatternArgs),
    CircularPattern(CircularPatternArgs),
    Mirror(MirrorArgs),
    MoveBody(MoveBodyArgs),
    Subd(SubdArgs),
    DeleteBody(DeleteBodyArgs),
    BoundarySurface(BoundarySurfaceArgs),
    OffsetSurface(OffsetSurfaceArgs),
    Thicken(ThickenArgs),
    Knit(KnitArgs),
    TrimSurface(TrimSurfaceArgs),
    ExtendSurface(ExtendSurfaceArgs),
    ExtractFace(ExtractFaceArgs),
    ProjectCurve(ProjectCurveArgs),
    NetworkSurface(NetworkSurfaceArgs),
    BaseFlange(BaseFlangeArgs),
    EdgeFlange(EdgeFlangeArgs),
    Hem(HemArgs),
    SheetCut(SheetCutArgs),
    FlatPattern(FlatPatternArgs),
    AddPart(AddPartArgs),
    PartPattern(PartPatternArgs),
    Route(RouteArgs),
    ImportStep(ImportStepArgs),
    Drawing(DrawingArgs),
}

/// The exporters and inspectors the editor offers (see [`geop_ops::output`]),
/// in the order it offers them: the first exporter's first format is what
/// a part is written in unless told otherwise.
pub fn outputs() -> GeopResult<Outputs> {
    Outputs::default()
        .exporter(Stl)?
        .exporter(Step)?
        .exporter(DrawingExporter)?
        .exporter(Urdf)?
        .exporter(FlatPatternDxf)?
        .exporter(BomCsv)?
        .inspector(Measure)?
        .inspector(MassProperties)?
        .inspector(Interference)?
        .inspector(BillOfMaterials)
}

/// A program as the editor edits it and a file holds it.
pub type Program = ProgramFile;

/// Appending steps to a program written in Rust.
pub trait Push {
    /// Appends the step `id`, `args` for the operation they are for.
    fn push<A: StepArgs>(&mut self, id: impl Into<String>, args: A);
}

impl Push for Program {
    fn push<A: StepArgs>(&mut self, id: impl Into<String>, args: A) {
        self.program.steps.push(ProgramStep {
            id: id.into(),
            operation: A::KIND.to_string(),
            args: serde_json::to_value(args).expect("a program's arguments are plain data"),
        });
    }
}

/// The workspace the editor builds programs in: a set of program files, and
/// the standard parts beside them.
pub type Workspace<F = BTreeMap<String, String>> =
    geop_ops::program::Workspace<WithStandardParts<F>>;

/// The runner programs are built with.
pub type Runner = ProgramRunner<Input, ParamValue>;

/// The workspace of `files`, beside the standard parts, whose programs are
/// written in the editor's operations.
pub fn workspace<F: Files + 'static>(files: F) -> Rc<Workspace<F>> {
    Workspace::new(WithStandardParts(files), registry)
}

/// A runner of the editor's operations, that has run nothing yet.
pub fn runner() -> Runner {
    ProgramRunner::new(registry().expect("the editor's operations each have a name of their own"))
}

/// Runs `program` with `library` on `runner`, which keeps what earlier
/// runs built. Fails naming the first step that does.
pub fn run(runner: &mut Runner, program: &Program, library: &Rc<dyn Library>) -> GeopResult<()> {
    let inputs = program.inputs(&Values::new(), library.clone());
    let output = runner.run(&program.program, &inputs, &program.state);
    match output.step_errors.into_iter().next() {
        Some((step, error)) => Err(error.with_context(format!("step {step:?}"))),
        None => Ok(()),
    }
}

/// Builds `program` with `library` — for a program on its own, the scope
/// of an empty [`workspace`]: the runner, holding the part it built.
pub fn build(program: &Program, library: &Rc<dyn Library>) -> GeopResult<Runner> {
    let mut runner = runner();
    run(&mut runner, program, library)?;
    Ok(runner)
}
