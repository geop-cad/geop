//! Configurations as a user makes them, through the panel the front end
//! shows ([`ConfigurationsPanel`]): added, switched, and steps suppressed
//! in one of them, each switch building the part as that configuration
//! says — and a program of configurations built, saved and built again.

use std::rc::Rc;

use geop_core_editor::{
    editor::{Command, Editor, StepInfo, Update},
    registry::{Panel, Registry},
    testing::{editor_with, handled},
};
use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    program::Program,
    testing::{assert_incremental, assert_round_trips, edit_sequence},
};

use crate::parameters::{
    CONFIGURATION, ConfigurationsCommand, ConfigurationsPanel, DefineParametersArgs,
    LookupTableArgs, NumberParameter, PartPropertiesArgs, Row, TableChoice, read_properties,
    steps::LOOKUP,
};

/// The registry of what parameters offer.
fn registry() -> Registry {
    crate::register(Registry::default()).expect("one name each")
}

/// A part of a width and a colour.
fn part() -> Program {
    let mut program = Program::new();
    program.push(
        "parameters",
        DefineParametersArgs {
            values: vec![NumberParameter::new("width", "10", 0.0, 100.0)],
        },
    );
    program.push(
        "properties",
        PartPropertiesArgs {
            color: Some("#ff0000".into()),
            ..PartPropertiesArgs::default()
        },
    );
    program
}

/// What the panel `command` changes, sent as the front end sends it.
#[track_caller]
fn send(editor: &mut Editor, command: ConfigurationsCommand) -> Update {
    handled(
        editor,
        Command::panel::<ConfigurationsPanel>(&command).unwrap(),
    )
}

/// What the panel shows, as the last update sent it.
#[track_caller]
fn tables(update: &Update) -> Vec<TableChoice> {
    let panels = &update.program.as_ref().expect("the program is sent").panels;
    let state = &panels[ConfigurationsPanel::KEY]["tables"];
    (state.as_array().unwrap().iter())
        .map(|table| TableChoice {
            step: table["step"].as_str().unwrap().into(),
            input: table["input"].as_str().unwrap().into(),
            rows: (table["rows"].as_array().unwrap().iter())
                .map(|row| row.as_str().unwrap().into())
                .collect(),
            selected: table["selected"].as_str().unwrap().into(),
        })
        .collect()
}

/// The step `id` as the timeline lists it.
#[track_caller]
fn listed<'u>(update: &'u Update, id: &str) -> &'u StepInfo {
    let steps = &update.program.as_ref().expect("the program is sent").steps;
    steps.iter().find(|step| step.id == id).expect("listed")
}

/// The colour of the part as the editor built it.
fn color(editor: &Editor) -> Option<String> {
    read_properties(&editor.part()).unwrap().color.clone()
}

/// A step suppressed with no configurations is suppressed everywhere: it
/// builds nothing — the part has no colour — and the timeline says so;
/// suppressed again, it runs again. Undo takes it back.
#[test]
fn a_step_is_suppressed_and_runs_again() {
    let mut editor = editor_with(registry(), part());
    assert_eq!(color(&editor).as_deref(), Some("#ff0000"));
    let suppress = || ConfigurationsCommand::Suppress {
        step: "properties".into(),
    };
    let update = send(&mut editor, suppress());
    assert!(listed(&update, "properties").suppressed);
    assert!(!listed(&update, "parameters").suppressed);
    assert_eq!(color(&editor), None);
    let step = editor.program().step("properties").unwrap();
    assert_eq!(step.active.as_deref(), Some("0"));

    let update = send(&mut editor, suppress());
    assert!(!listed(&update, "properties").suppressed);
    assert_eq!(color(&editor).as_deref(), Some("#ff0000"));
    assert_eq!(editor.program().step("properties").unwrap().active, None);

    let update = handled(&mut editor, Command::Undo);
    assert!(listed(&update, "properties").suppressed);
    assert_eq!(color(&editor), None);
}

/// Configurations are added, the first making a table of them before
/// everything, its first row the program as it was; each added is the one
/// built. A step suppressed in one is suppressed in that one alone, by a
/// column of the table its formula reads; switching builds each as it
/// says, and running it again in every one takes the column out.
#[test]
fn configurations_are_added_switched_and_suppress_steps_of_their_own() {
    let mut editor = editor_with(registry(), part());
    let update = send(&mut editor, ConfigurationsCommand::Add { name: None });
    assert_eq!(
        tables(&update),
        [TableChoice {
            step: "configurations".into(),
            input: CONFIGURATION.into(),
            rows: vec!["Default".into(), "Configuration 2".into()],
            selected: "Configuration 2".into(),
        }]
    );
    assert_eq!(editor.program().steps[0].operation, LOOKUP);
    let update = send(
        &mut editor,
        ConfigurationsCommand::Add {
            name: Some("light".into()),
        },
    );
    assert_eq!(tables(&update)[0].selected, "light");

    let suppress = || ConfigurationsCommand::Suppress {
        step: "properties".into(),
    };
    let update = send(&mut editor, suppress());
    assert!(listed(&update, "properties").suppressed);
    assert_eq!(color(&editor), None);
    let step = editor.program().step("properties").unwrap();
    assert_eq!(step.active.as_deref(), Some("configuration.properties"));

    let select = |row: &str| ConfigurationsCommand::Select {
        input: CONFIGURATION.into(),
        row: row.into(),
    };
    let update = send(&mut editor, select("Default"));
    assert!(!listed(&update, "properties").suppressed);
    assert_eq!(color(&editor).as_deref(), Some("#ff0000"));
    send(&mut editor, select("Configuration 2"));
    assert_eq!(color(&editor).as_deref(), Some("#ff0000"));

    // Suppressed in a second configuration, then run again in both.
    send(&mut editor, suppress());
    assert_eq!(color(&editor), None);
    let table: LookupTableArgs =
        serde_json::from_value(editor.program().steps[0].args.clone()).unwrap();
    assert_eq!(table.columns, ["properties"]);
    let values: Vec<f64> = table.rows.iter().map(|row| row.values[0]).collect();
    assert_eq!(values, [1.0, 0.0, 0.0]);
    send(&mut editor, suppress());
    send(&mut editor, select("light"));
    let update = send(&mut editor, suppress());
    assert!(!listed(&update, "properties").suppressed);
    let table: LookupTableArgs =
        serde_json::from_value(editor.program().steps[0].args.clone()).unwrap();
    assert!(table.columns.is_empty(), "{table:?}");
    assert_eq!(editor.program().step("properties").unwrap().active, None);

    // What is not there is refused, by name.
    let refused = editor.handle(Command::panel::<ConfigurationsPanel>(&select("heavy")).unwrap());
    assert!(refused.error.unwrap().contains("no row \"heavy\""));
    let refused = editor.handle(
        Command::panel::<ConfigurationsPanel>(&ConfigurationsCommand::Add {
            name: Some("light".into()),
        })
        .unwrap(),
    );
    assert!(refused.error.unwrap().contains("already"));
}

/// A program of configurations: each sets the parameters, by the columns
/// of its row, and the steps suppressed. The part is built in the one
/// selected, or the one the run's input names — as a program placing it
/// names it.
fn configured(selected: &str) -> Program {
    let mut program = Program::new();
    program.push(
        "configurations",
        LookupTableArgs {
            input: CONFIGURATION.into(),
            columns: vec!["width".into(), "colored".into()],
            rows: vec![
                Row {
                    name: "full".into(),
                    values: vec![40.0, 1.0],
                },
                Row {
                    name: "light".into(),
                    values: vec![20.0, 0.0],
                },
            ],
            selected: selected.into(),
        },
    );
    program.push(
        "parameters",
        DefineParametersArgs {
            values: vec![NumberParameter::new(
                "length",
                "2 * configuration.width",
                0.0,
                200.0,
            )],
        },
    );
    program.push(
        "properties",
        PartPropertiesArgs {
            color: Some("#00ff00".into()),
            ..PartPropertiesArgs::default()
        },
    );
    program.steps[2].active = Some("configuration.colored".into());
    program
}

/// The parameter `name` of the part `editor` built.
fn number(editor: &Editor, name: &str) -> f64 {
    let values = read_parameter_values(&editor.part()).unwrap();
    match values.0.iter().find(|(n, _)| n == name) {
        Some((_, ParameterValue::Number(n))) => *n,
        other => panic!("{name}: {other:?}"),
    }
}

/// Switching the configuration builds the part with its parameters and
/// its steps: the length follows the row's width, and the colour is there
/// only where the row says so.
#[test]
fn a_configuration_sets_parameters_and_steps() {
    let mut editor = editor_with(registry(), configured("full"));
    assert_eq!(number(&editor, "length"), 80.0);
    assert_eq!(color(&editor).as_deref(), Some("#00ff00"));
    send(
        &mut editor,
        ConfigurationsCommand::Select {
            input: CONFIGURATION.into(),
            row: "light".into(),
        },
    );
    assert_eq!(number(&editor, "length"), 40.0);
    assert_eq!(color(&editor), None);
}

/// A program of configurations is saved and read back as it was, and
/// builds as from scratch however it is edited: switched, a step
/// suppressed and run again.
#[test]
fn a_program_of_configurations_round_trips_and_builds_as_from_scratch() {
    let operations = Rc::new(registry().operations());
    for selected in ["full", "light"] {
        assert_round_trips(operations.clone(), None, &configured(selected));
    }
    let mut programs = edit_sequence(&configured("full"));
    let mut always = configured("light");
    always.steps[2].active = None;
    programs.extend([configured("light"), always, configured("full")]);
    assert_incremental(operations, None, &programs);
}
