//! The operations the editor offers, as the one set its programs are
//! written in ([`registry`]), the program types over it, and the one way a
//! program is built outside the editor ([`build`]).
//!
//! Every operation is defined by a crate of its own (`geop_ops_*`, imported
//! below), as a [`geop_ops::operation::PartOperation`]. Which of them an
//! editor offers is the editor's choice, made here (the `operations!` list) — and so
//! are the exporters and inspectors ([`outputs`]).

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::GeopResult;
use geop_core_program::operation::{Operation, OperationInfo};
use geop_ops::{
    output::{Outputs, Stl},
    parameters::Values,
    placed::Library,
    program::{Files, ProgramFile, Registry},
};

pub use geop_ops::program::Runner;

use crate::stdlib::WithStandardParts;
use geop_ops_assembly::{AddPart, PartPattern};
use geop_ops_bom::{BillOfMaterials, BomCsv};
use geop_ops_booleans::{Boolean, Split};
use geop_ops_datums::AddDatum;
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

/// The operations the editor offers, in the order it offers them within a
/// group and tier: the registry its programs are run with ([`registry`]),
/// and what it lists ([`infos`]).
macro_rules! operations {
    ($($operation:ident),* $(,)?) => {
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
    };
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

/// The workspace the editor builds programs in: a set of program files, and
/// the standard parts beside them.
pub type Workspace<F = BTreeMap<String, String>> =
    geop_ops::program::Workspace<WithStandardParts<F>>;

/// The workspace of `files`, beside the standard parts, whose programs are
/// written in the editor's operations.
pub fn workspace<F: Files + 'static>(files: F) -> Rc<Workspace<F>> {
    Workspace::new(WithStandardParts(files), registry)
}

/// A runner of the editor's operations, that has run nothing yet.
pub fn runner() -> Runner {
    Runner::new(registry().expect("the editor's operations each have a name of their own"))
}

/// Builds `program` with `library` — for a program on its own, the scope
/// of an empty [`workspace`]: the runner, holding the part it built.
pub fn build(program: &Program, library: &Rc<dyn Library>) -> GeopResult<Runner> {
    let mut runner = runner();
    program.run(&mut runner, &Values::new(), library)?;
    Ok(runner)
}
