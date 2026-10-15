//! The operations the editor offers, as the one set its programs are
//! written in and edited with ([`editors`]), the program types over it, and
//! the one way a program is built outside the editor ([`build`]).
//!
//! Every operation is defined by a crate of its own (`geop_ops_*`, imported
//! below), as a [`geop_core_editor::part_operation::PartOperation`]. Which of them an
//! editor offers is the editor's choice, made here (the `operations!` list) — and so
//! are the exporters and inspectors ([`outputs`]).

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::editable::{Editable, Listed};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{operation::Operation, workspace::Location};
use geop_ops_model::Design;
use geop_ops_output::Outputs;
use geop_ops_parameters::{
    BuildPart, DefineParameters, PartProperties, ProgramFile, Registry, Setting, Settings,
    StateValue, StateValues,
};

pub use geop_ops_parameters::Runner;

use crate::{
    stdlib::WithStandardParts,
    workspace::{self, FileStore},
};
use geop_ops_add_datum::AddDatum;
use geop_ops_assembly::{AddPart, PartPattern, Stl};
use geop_ops_bom::{BillOfMaterials, BomCsv};
use geop_ops_booleans::{Boolean, Split};
use geop_ops_drawing::{Drawing, DrawingExporter};
use geop_ops_edit::{DeleteBody, ExtractFace, ProjectCurve};
use geop_ops_extrude_revolve::{Extrude, Loft, Revolve, Sweep};
use geop_ops_fillet::{Chamfer, Fillet};
use geop_ops_harness::Route;
use geop_ops_hole::{Hole, Thread};
use geop_ops_inspect::{Interference, MassProperties, Measure};
use geop_ops_pattern::{CircularPattern, LinearPattern, Mirror, MoveBody};
use geop_ops_plastic::{Draft, Groove, Lip, Rib};
use geop_ops_sheetmetal::{BaseFlange, EdgeFlange, FlatPattern, FlatPatternDxf, Hem, SheetCut};
use geop_ops_shell::Shell;
use geop_ops_sketch::AddSketch;
use geop_ops_sketch3d::AddSketch3d;
use geop_ops_step::{ImportStep, Step};
use geop_ops_subd::Subd;
use geop_ops_surface::{
    BoundarySurface, ExtendSurface, Knit, NetworkSurface, OffsetSurface, Thicken, TrimSurface,
};
use geop_ops_urdf::Urdf;

#[cfg(test)]
mod add_datum_editor_tests;
#[cfg(test)]
mod assembly_editor_tests;
#[cfg(test)]
mod sketch3d_editor_tests;
#[cfg(test)]
mod sketch_editor_tests;
#[cfg(test)]
mod step_export_tests;
#[cfg(test)]
mod assembly_scale_tests;
#[cfg(test)]
mod assembly_tests;
#[cfg(test)]
mod bom_tests;
#[cfg(test)]
mod drawing_tests;
#[cfg(test)]
mod editor_tests;
#[cfg(test)]
mod harness_tests;
#[cfg(test)]
mod inspect_tests;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod set_tests;
#[cfg(test)]
mod sheetmetal_tests;
#[cfg(test)]
mod sketch_tests;
#[cfg(test)]
mod step_tests;
#[cfg(test)]
mod stress_tests;
#[cfg(test)]
pub(crate) mod testing;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod urdf_tests;
#[cfg(test)]
mod view_tests;

/// The operations of the editor's programs, and how each is edited.
pub type Editors = geop_core_editor::editable::Editors<Setting, StateValue, Design>;

/// The operations the editor offers, in the order it offers them within a
/// group and tier: how its programs are edited ([`editors`]), and what it
/// lists ([`infos`]).
macro_rules! operations {
    ($($operation:ident),* $(,)?) => {
        /// The operations the editor offers, and how each is edited.
        pub fn editors() -> GeopResult<Editors> {
            let editors = Editors::default();
            $(let editors = editors.with($operation)?;)*
            Ok(editors)
        }

        /// What each operation the editor offers is, in the order it offers
        /// them.
        pub fn infos() -> Vec<Listed> {
            vec![$(Listed {
                kind: <$operation as Operation>::KIND,
                info: &<$operation as Editable>::INFO,
            }),*]
        }
    };
}

/// The operations the editor offers, as its programs are run with.
pub fn registry() -> GeopResult<Registry> {
    editors()?.operations()
}

operations! {
    AddSketch,
    AddDatum,
    AddSketch3d,
    Extrude,
    Revolve,
    Sweep,
    Loft,
    Hole,
    Thread,
    Rib,
    Fillet,
    Chamfer,
    Shell,
    Draft,
    Lip,
    Groove,
    Boolean,
    Split,
    LinearPattern,
    CircularPattern,
    Mirror,
    MoveBody,
    Subd,
    DeleteBody,
    BoundarySurface,
    OffsetSurface,
    Thicken,
    Knit,
    TrimSurface,
    ExtendSurface,
    ExtractFace,
    ProjectCurve,
    NetworkSurface,
    BaseFlange,
    EdgeFlange,
    Hem,
    SheetCut,
    FlatPattern,
    AddPart,
    PartPattern,
    Route,
    ImportStep,
    Drawing,
    DefineParameters,
    PartProperties,
}

/// The exporters and inspectors the editor offers (see [`geop_ops_output`]),
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

/// The workspace the editor builds programs in: a set of program files, and
/// the standard parts beside them.
pub type Workspace<F = BTreeMap<String, String>> = workspace::Workspace<WithStandardParts<F>>;

/// The workspace of `files`, beside the standard parts, whose programs are
/// written in the editor's operations.
pub fn workspace<F: FileStore + 'static>(files: F) -> Rc<Workspace<F>> {
    Workspace::new(WithStandardParts(files), registry)
}

/// A runner of the editor's operations, that has run nothing yet.
pub fn runner() -> Runner {
    Runner::new(registry().expect("the editor's operations each have a name of their own"))
}

/// A part a program built, and the runner that built it, to run it again.
pub struct BuiltPart {
    pub runner: Runner,
    pub part: BuildPart,
}

/// Builds `program`, the file `file` of `workspace` — for a program on its
/// own, an empty [`workspace()`] — with its parameters as it defines them.
pub fn build(
    program: &Program,
    file: &str,
    workspace: Rc<dyn geop_core_program::workspace::Workspace<StateValue>>,
) -> GeopResult<BuiltPart> {
    let mut runner = runner();
    runner.workspace = Some(workspace);
    runner.location = Location {
        file: workspace::resolve("", file),
        within: Vec::new(),
    };
    let mut part = BuildPart::default();
    program.run(&runner, &mut part, &Settings::new(), &StateValues::new())?;
    Ok(BuiltPart { runner, part })
}
