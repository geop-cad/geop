//! Parameters given and renamed as the parameters panel does it, over
//! the steps that read them: the extrusions and sketches of the drilled
//! box and the parametric plate (an integration test, as the extrusion
//! depends on this crate).

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::Value,
    editor::{Command, Editor},
    registry::Registry,
    testing::{dialog, editor_with, handled, panel_state},
};
use geop_core_program::operation::Operation;
use geop_ops_extrude_revolve::{
    Extent, ExtrudeArgs,
    examples::{box_with_drill_hole, parametric_plate},
    testing::args,
};
use geop_ops_model::testing::{model_shown, solid_volume};
use geop_ops_parameters::{
    DefineParameters, Parameter, Parameters, ParametersCommand, ParametersPanel, parameters_of,
};

/// What these tests edit: the parameters, and the sketches and extrusions
/// that read them.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

/// The command renaming the parameter `from` `to`, as the parameters panel
/// sends it.
fn rename_parameter(from: &str, to: &str) -> Command {
    let (from, to) = (from.into(), to.into());
    Command::panel::<ParametersPanel>(&ParametersCommand::RenameParameter { from, to })
        .expect("names are plain data")
}

/// Parameters given to a program without any, while a step is edited, put
/// their step first: the step edited, and the marker, stay at the steps they
/// were at, and the commit keeps both the parameters and the step's edit.
#[test]
fn the_first_parameter_given_while_a_step_is_edited() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
    assert!(parameters_of(editor.program()).is_empty());
    editor.handle(Command::Seek { marker: Some(2) });
    handled(
        &mut editor,
        Command::New {
            kind: "extrude".into(),
        },
    );
    editor.handle(dialog("distance", Value::Number(2.0)));
    let parameters = Parameters {
        values: vec![Parameter::number("depth", "3", 0.0, 100.0)],
        ..Parameters::default()
    };
    let set = ParametersCommand::SetParameters { parameters };
    handled(
        &mut editor,
        Command::panel::<ParametersPanel>(&set).unwrap(),
    );
    handled(&mut editor, Command::Commit);
    let steps = &editor.program().steps;
    assert_eq!(steps[0].operation, DefineParameters::KIND);
    assert_eq!(steps[3].id, "extrude1");
    let extrude: ExtrudeArgs = args(&steps[3]);
    assert_eq!(extrude.extent.side1, Extent::blind(2.0));
    let update = editor.handle(Command::Show);
    assert_eq!(update.program.unwrap().marker, 4);
}

/// What reads each parameter is sent with the program, for the parameters
/// panel to warn before one is removed; and renaming one, as its dialog's
/// name field does, renames it in every formula — the other parameters',
/// a sketch's dimensions, an extrude's length, a table's columns — so
/// nothing fails and the part is the same. A name taken, and a rename
/// while a step is edited, are refused; undo takes a rename back.
#[test]
fn renaming_a_parameter_renames_what_reads_it() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = handled(
        &mut editor,
        Command::Load {
            program: parametric_plate(),
            path: None,
        },
    );
    let uses = panel_state::<ParametersPanel>(&update.program.expect("the program is sent"))
        .parameter_uses;
    let readers = |name: &str| uses.get(name).cloned().unwrap_or_default();
    assert_eq!(readers("width"), ["depth", "outline"]);
    assert_eq!(readers("depth"), ["outline"]);
    assert_eq!(readers("thickness"), ["plate"]);
    assert_eq!(readers("screw"), ["hole_sketch"]);
    let volume = |editor: &Editor| {
        let part = model_shown(editor);
        let [name] = part.solid_names().try_into().expect("one solid");
        solid_volume(&part, &name)
    };
    let before = volume(&editor);

    for (from, to) in [
        ("width", "plate_width"),
        ("screw", "bolt"),
        ("thickness", "t"),
    ] {
        let update = editor.handle(rename_parameter(from, to));
        assert!(update.error.is_none(), "{from} -> {to}: {:?}", update.error);
        let program = update.program.expect("the program is sent");
        assert!(
            program.steps.iter().all(|s| s.error.is_none()),
            "{:?}",
            program.steps
        );
        let panel = panel_state::<ParametersPanel>(&program);
        assert!(
            panel.parameters.errors.is_empty(),
            "{:?}",
            panel.parameters.errors
        );
        assert!(
            !panel.parameter_uses.contains_key(from),
            "{from} is still read"
        );
    }
    let program = editor.program();
    let json = program.to_json().unwrap();
    for (old, new) in [
        ("\"width / 2\"", "\"plate_width / 2\""),
        ("\"screw.clearance\"", "\"bolt.clearance\""),
        (
            "{\"blind\":{\"formula\":\"thickness\"}}",
            "{\"blind\":{\"formula\":\"t\"}}",
        ),
    ] {
        let compact: String = json.split_whitespace().collect();
        assert!(
            !compact.contains(&old.replace(' ', "")),
            "{old} is left in {json}"
        );
        assert!(
            compact.contains(&new.replace(' ', "")),
            "{new} is not in {json}"
        );
    }
    assert_eq!(volume(&editor), before);

    let taken = editor.handle(rename_parameter("depth", "t"));
    let error = taken.error.expect("a name taken is refused");
    assert!(
        error.contains(r#"there is a parameter "t" already"#),
        "{error}"
    );
    editor.handle(Command::Open { id: "plate".into() });
    let editing = editor.handle(rename_parameter("depth", "plate_depth"));
    assert!(editing.error.is_some(), "renamed while a step is edited");
    editor.handle(Command::Cancel);

    editor.handle(Command::Undo);
    assert!(parameters_of(editor.program()).get("thickness").is_some());
    assert!(parameters_of(editor.program()).get("t").is_none());
}
