//! What the editor offers ([`registry`]): every crate's operations,
//! drawers, exporters, inspectors, panels, tools and settler, one
//! `register` line per crate, and the examples — and the one way a program
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

use crate::examples;

/// Everything the editor offers. Operations are offered group by group,
/// and within a group and tier in the order they are registered here; the
/// exporters and inspectors in the order they are registered — the first
/// exporter's first format is what a part is written in unless told
/// otherwise — and a part is drawn and listed by the drawers in turn.
pub fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_model::register(r)?;
    let r = geop_ops_sketch::register(r)?;
    let r = geop_ops_datums::register(r)?;
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
    let r = geop_ops_urdf::register(r)?;
    let r = geop_ops_sheetmetal::register(r)?;
    let r = geop_ops_inspect::register(r)?;
    let r = geop_ops_bom::register(r)?;
    let r = geop_ops_parameters::register(r)?;
    r.examples(examples::set())
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

/// What the tests of programs share beyond what every operation's tests
/// do (`geop_ops_model::testing`, `geop_ops_extrude_revolve::testing`,
/// `geop_core_editor::testing`): building one beside the standard parts,
/// and editing it in an editor of everything.
#[cfg(test)]
pub(crate) mod testing {
    use std::collections::BTreeMap;

    use geop_core_editor::{
        editor::{Command, Editor, ProgramState},
        registry::Panel,
        testing::loaded as loaded_in,
    };
    use geop_core_math::geop_error::GeopResult;
    use geop_core_program::{part::Part, program::Program, values::Design};
    use geop_ops_assembly::{JointsCommand, JointsPanel};
    use geop_ops_extrude_revolve::testing::model_of;
    use geop_ops_model::NamedModel;
    use geop_ops_parameters::{
        Parameters, ParametersCommand, ParametersPanel, ParametersPanelState,
    };

    pub(crate) use geop_core_editor::testing::{
        Stepping, click_at, dialog, from_above, handled, pointer,
    };
    pub(crate) use geop_ops_datums::origin_axis;
    pub(crate) use geop_ops_extrude_revolve::testing::{args, edit_args};
    pub(crate) use geop_ops_model::testing::{check_valid, solid_volume};

    pub(crate) use super::{build, workspace};

    /// Building a program on its own, in an empty [`workspace`] (see
    /// [`build`]).
    pub(crate) trait Built {
        /// The part the program builds.
        fn built(&self) -> GeopResult<Part>;

        /// The model the program builds, or why a step fails.
        fn build_model(&self) -> GeopResult<NamedModel<Design>>;
    }

    impl Built for Program {
        fn built(&self) -> GeopResult<Part> {
            Ok(build(self, "", workspace(BTreeMap::new()))?.part)
        }

        fn build_model(&self) -> GeopResult<NamedModel<Design>> {
            Ok(model_of(&self.built()?))
        }
    }

    /// An editor of everything on `program`, which loads without an error.
    pub(crate) fn loaded(program: Program) -> Editor {
        loaded_in(super::editor(), program)
    }

    /// The command setting the program's parameters to `parameters`, as
    /// the parameters panel sends it.
    pub(crate) fn set_parameters(parameters: Parameters) -> Command {
        Command::panel::<ParametersPanel>(&ParametersCommand::SetParameters { parameters })
            .expect("parameters are plain data")
    }

    /// The command renaming the parameter `from` `to`, as the parameters
    /// panel sends it.
    pub(crate) fn rename_parameter(from: &str, to: &str) -> Command {
        let (from, to) = (from.into(), to.into());
        Command::panel::<ParametersPanel>(&ParametersCommand::RenameParameter { from, to })
            .expect("names are plain data")
    }

    /// The command setting the value `parameter` a step offers a user to
    /// set to `value`, as the joints panel sends it.
    pub(crate) fn set_joint(parameter: &str, value: f64) -> Command {
        let parameter = parameter.into();
        Command::panel::<JointsPanel>(&JointsCommand::Set { parameter, value })
            .expect("a joint's value is plain data")
    }

    /// What the parameters panel shows of `program`.
    pub(crate) fn parameters_panel(program: &ProgramState) -> ParametersPanelState {
        serde_json::from_value(program.panels[ParametersPanel::KEY].clone())
            .expect("the parameters panel shows its state")
    }

    /// What the joints panel shows of `program`.
    pub(crate) fn joints_panel(program: &ProgramState) -> &serde_json::Value {
        &program.panels[JointsPanel::KEY]
    }

    /// An editor of everything on the example `name`.
    pub(crate) fn example_editor(name: &str) -> Editor {
        let mut editor = super::editor();
        handled(&mut editor, Command::LoadExample { name: name.into() });
        editor
    }
}
