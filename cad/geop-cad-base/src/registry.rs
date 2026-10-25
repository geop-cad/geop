//! What the editor offers ([`registry`]): every crate's operations,
//! drawers, exporters, inspectors, panels, tools, settler and examples,
//! one `register` line per crate — and the one way a program
//! is built outside the editor ([`build`]), in a workspace beside the
//! standard parts ([`workspace`]).

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{editor::Editor, output::Outputs, registry::Registry};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    part::Part,
    program::Program,
    runner::Runner,
    workspace::{self, FileStore, Files, Location},
};
use geop_lib_stdlib::WithStandardParts;

/// Everything the editor offers. Operations are offered group by group,
/// and within a group and tier in the order they are registered here; the
/// exporters and inspectors in the order they are registered — the first
/// exporter's first format is what a part is written in unless told
/// otherwise — and a part is drawn and listed by the drawers in turn.
pub fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_model::register(r)?;
    let r = geop_ops_sketch::register(r)?;
    let r = geop_ops_reference_geometry::register(r)?;
    let r = geop_ops_sketch3d::register(r)?;
    let r = geop_ops_extrude_revolve::register(r)?;
    let r = geop_ops_hole::register(r)?;
    let r = geop_ops_fillet::register(r)?;
    let r = geop_ops_shell::register(r)?;
    let r = geop_ops_plastic::register(r)?;
    let r = geop_ops_boolean::register(r)?;
    let r = geop_ops_pattern::register(r)?;
    let r = geop_ops_subd::register(r)?;
    let r = geop_ops_surface::register(r)?;
    let r = geop_ops_edit::register(r)?;
    let r = geop_ops_assembly::register(r)?;
    let r = geop_ops_harness::register(r)?;
    let r = geop_ops_step::register(r)?;
    let r = geop_ops_drawing::register(r)?;
    let r = geop_ops_sheetmetal::register(r)?;
    let r = geop_ops_inspect::register(r)?;
    geop_ops_bom::register(r)
}

/// An editor of everything [`registry`] offers, the standard parts beside
/// the files it is sent.
pub fn editor() -> Editor {
    let registry = registry().expect("the editor's extensions each have a name of their own");
    Editor::new(registry, WithStandardParts(BTreeMap::new()))
}

/// The exporters and inspectors the editor offers (see
/// [`geop_core_editor::output`]), in the order it offers them.
pub fn outputs() -> GeopResult<Outputs> {
    Ok(registry()?.into_outputs())
}

/// The workspace the editor builds programs in: a set of program files, and
/// the standard parts beside them.
pub type Workspace<F = BTreeMap<String, String>> = Files<WithStandardParts<F>>;

/// The workspace of `files`, beside the standard parts, whose programs are
/// written in the editor's operations.
pub fn workspace<F: FileStore + 'static>(files: F) -> Rc<Workspace<F>> {
    let registry = registry().expect("the editor's extensions each have a name of their own");
    Workspace::new(WithStandardParts(files), Rc::new(registry.operations()))
}

/// A runner of the editor's operations, that has run nothing yet.
pub fn runner() -> Runner {
    let registry = registry().expect("the editor's extensions each have a name of their own");
    Runner::new(registry.operations())
}

/// A part a program built, and the runner that built it, to run it again.
pub struct BuiltPart {
    pub runner: Runner,
    pub part: Part,
}

/// Builds `program`, the file `file` of `workspace` — for a program on its
/// own, an empty [`workspace()`] — with its parameters as it defines them.
pub fn build(
    program: &Program,
    file: &str,
    workspace: Rc<dyn workspace::Workspace>,
) -> GeopResult<BuiltPart> {
    let mut runner = runner();
    runner.workspace = Some(workspace);
    runner.location = Location {
        file: workspace::resolve("", file),
        within: Vec::new(),
    };
    let mut part = Part::default();
    program.run(&runner, &mut part)?;
    Ok(BuiltPart { runner, part })
}
