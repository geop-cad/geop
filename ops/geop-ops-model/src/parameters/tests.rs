//! A program file run as a part is: its parameters and properties defined
//! by steps of it, set by the run's inputs, read by the steps after them,
//! and its formulas found and renamed in the steps' saved arguments.

use std::cell::Cell;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    numeric::NumericInput,
    operation::{Operation, StepArgs},
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    program::Program,
    runner::Runner,
    target::{Target, TargetReference},
    values::{Design, Setting, Settings, StateValue, pose_state},
};
use serde::{Deserialize, Serialize};

use crate::parameters::{
    COLOR, DefineParameters, LookupTable, LookupTableArgs, Parameter, ParameterKind, Parameters,
    PartProperties, parameter_numbers, parameter_uses, Row, read_properties,
    rename_parameter, set_parameters,
};

/// What a test step makes, read whole.
#[derive(Clone, Debug, PartialEq)]
struct Made<T>(T);

impl<T: PartialEq + 'static> Target for Made<T> {}

impl<T: 'static> Kind for Made<T> {
    const NAME: &'static str = "made";
}

impl<T: 'static> HasElements<Made<T>> for Made<T> {
    type Element<'a> = &'a T;

    fn element(&self, name: &str) -> GeopResult<&T> {
        Ok(&whole(self, name)?.0)
    }
}

fn made<T: Clone + 'static>(part: &Part, step: &str) -> T {
    let reader = part.current();
    let reference = ElementReference::whole(TargetReference::<Made<T>>::new(step));
    reader.retrieve_element(&reference).unwrap().clone()
}

thread_local! {
    /// How often a step's generator ran.
    static RUNS: Cell<usize> = const { Cell::new(0) };
}

/// The length a step resolves, as a target of the step's id.
struct Length;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct LengthArgs {
    length: NumericInput,
}

impl StepArgs for LengthArgs {
    const KIND: &'static str = Length::KIND;
}

impl Operation for Length {
    const KIND: &'static str = "length";
    type Args = LengthArgs;

    fn run(ctx: &mut StepContext, args: &LengthArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(
            &TargetReference::new(step),
            &args.length,
            |reader, length| {
                RUNS.set(RUNS.get() + 1);
                length.resolve(reader).map(Made)
            },
        )
    }
}

/// The part's colour, as a target of the step's id.
struct Colour;

#[derive(Serialize, Deserialize)]
struct ColourArgs;

impl StepArgs for ColourArgs {
    const KIND: &'static str = Colour::KIND;
}

impl Operation for Colour {
    const KIND: &'static str = "colour";
    type Args = ColourArgs;

    fn run(ctx: &mut StepContext, _: &ColourArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&TargetReference::new(step), &(), |reader, _| {
            RUNS.set(RUNS.get() + 1);
            Ok(Made(
                read_properties(reader)?
                    .color()
                    .unwrap_or("none")
                    .to_string(),
            ))
        })
    }
}

fn registry() -> OperationRegistry {
    (OperationRegistry::default().with(Length))
        .and_then(|r| r.with(Colour))
        .and_then(|r| r.with(DefineParameters))
        .and_then(|r| r.with(LookupTable))
        .and_then(|r| r.with(PartProperties))
        .unwrap()
}

fn runner() -> Runner {
    Runner::new(registry())
}

fn file() -> Program {
    let mut file = Program::new();
    let values = vec![
        Parameter {
            name: "screw".into(),
            kind: ParameterKind::Table {
                columns: vec!["d".into()],
                rows: vec![
                    Row {
                        name: "M3".into(),
                        values: vec![3.0],
                    },
                    Row {
                        name: "M5".into(),
                        values: vec![5.0],
                    },
                ],
                selected: "M3".into(),
            },
        },
        Parameter::number("width", "2 * screw.d", 0.0, 100.0),
    ];
    set_parameters(
        &mut file,
        Parameters {
            color: Some("#ff8800".into()),
            designation: Some("TEST".into()),
            values,
            ..Parameters::default()
        },
    );
    let length = |formula: &str| LengthArgs {
        length: NumericInput::from(formula),
    };
    file.push("half", length("width / 2"));
    file.push("plain", length("7"));
    file.push("colour", ColourArgs);
    file
}

/// Runs `file` with its parameters set to `inputs`.
fn run(runner: &Runner, part: &mut Part, file: &Program, inputs: &Settings) {
    let output = runner.run_with(part, file, inputs, None);
    if let Some((step, e)) = output.step_errors.into_iter().next() {
        panic!("step {step:?}: {e}");
    }
}

fn length(part: &Part, step: &str) -> f64 {
    made(part, step)
}

/// A step reads the parameters its formula names, and what the program
/// says of its part through its properties; a run with other inputs
/// rebuilds only the steps that read what changed.
#[test]
fn a_run_reads_the_parameters_and_properties_its_steps_define() {
    let file = file();
    let runner = runner();
    let mut part = Part::default();
    run(&runner, &mut part, &file, &Settings::new());
    assert_eq!(length(&part, "half"), 3.0);
    assert_eq!(length(&part, "plain"), 7.0);
    assert_eq!(made::<String>(&part, "colour"), "#ff8800");
    let designation = read_properties(&part.current())
        .unwrap()
        .designation
        .clone();
    assert_eq!(designation.as_deref(), Some("TEST M3 width 6"));

    RUNS.set(0);
    let m5 = Settings::from([("screw".to_string(), Setting::Text("M5".into()))]);
    run(&runner, &mut part, &file, &m5);
    assert_eq!(length(&part, "half"), 5.0);
    let designation = read_properties(&part.current())
        .unwrap()
        .designation
        .clone();
    assert_eq!(designation.as_deref(), Some("TEST M5 width 10"));
    // The length the row changed, and the colour, which reads the
    // properties — whose designation names the row: not the plain length.
    assert_eq!(RUNS.get(), 2);

    RUNS.set(0);
    let mut red = m5.clone();
    red.insert(COLOR.into(), Setting::Text("#ff0000".into()));
    run(&runner, &mut part, &file, &red);
    assert_eq!(made::<String>(&part, "colour"), "#ff0000");
    assert_eq!(RUNS.get(), 1, "only the colour");
}

/// A step copied takes the state stored under its id along (where a placed
/// part is), and pasted under a fresh id stores it under that one; state
/// merely named like it stays behind.
#[test]
fn a_pasted_step_takes_its_state_along() {
    let mut file = file();
    let pose = StateValue::Pose(geop_core_math::primitives::Pose::identity());
    file.state.insert(pose_state("half"), pose.clone());
    (file.state).insert(
        "half_width".into(),
        StateValue::Number(Design::from_f64(2.0)),
    );

    let copied = file.excerpt("half").unwrap();
    assert_eq!(copied.state.len(), 1);
    let at = file.steps.len();
    let ids = file
        .paste(copied, at, &registry(), |s| s.operation.clone())
        .unwrap();
    assert_eq!(ids, ["length1"]);
    assert_eq!(file.steps[at].id, "length1");
    assert_eq!(file.state.get(&pose_state("length1")), Some(&pose));
    assert!(!file.state.contains_key("length1_width"));
    file.validate(&registry()).unwrap();
}

/// What reads a parameter is found in the formulas of the steps' saved
/// arguments and of the other parameters, and renaming it renames them.
#[test]
fn a_parameter_is_renamed_wherever_it_is_read() {
    let mut file = file();
    let uses = parameter_uses(&file);
    assert_eq!(uses["width"], ["half"]);
    assert_eq!(uses["screw"], ["width"]);

    rename_parameter(&mut file, "width", "span").unwrap();
    assert_eq!(parameter_uses(&file)["span"], ["half"]);
    let mut part = Part::default();
    run(&runner(), &mut part, &file, &Settings::new());
    assert_eq!(length(&part, "half"), 3.0);
    assert!(rename_parameter(&mut file, "span", "screw").is_err());
    assert!(rename_parameter(&mut file, "nothing", "x").is_err());

    let saved = file.to_json().unwrap();
    assert_eq!(Program::from_json(&saved).unwrap(), file);
}

/// What the parameters come to as defined, by the names a formula reads
/// them by.
#[test]
fn the_parameters_come_to_numbers_as_defined() {
    let numbers = parameter_numbers(&file()).unwrap();
    assert_eq!(numbers["screw.d"], 3.0);
    assert_eq!(numbers["width"], 6.0);
}

/// A table is a step of its own, looking the row its input names up: its
/// row's name is the parameter, its columns `screw.d`. Read back, the
/// program's parameters list it as the table it is.
#[test]
fn a_table_is_a_lookup_step_of_its_own() {
    let file = file();
    let table = file.step("table_screw").unwrap();
    assert_eq!(table.operation, "lookup_table");
    let args: LookupTableArgs = serde_json::from_value(table.args.clone()).unwrap();
    assert_eq!(
        (args.input.as_str(), args.selected.as_str()),
        ("screw", "M3")
    );
    assert_eq!(
        file.steps.iter().position(|s| s.id == "table_screw"),
        Some(0)
    );
    let defined = crate::parameters::parameters_of(&file);
    assert!(matches!(
        defined.get("screw").map(|p| &p.kind),
        Some(ParameterKind::Table { rows, .. }) if rows.len() == 2
    ));
}

/// A text naming no row of the table fails, naming the input and the rows;
/// the steps reading the table fail with it.
#[test]
fn a_table_given_no_row_of_it_fails_by_name() {
    let file = file();
    let runner = runner();
    let mut part = Part::default();
    let m9 = Settings::from([("screw".to_string(), Setting::Text("M9".into()))]);
    let output = runner.run_with(&mut part, &file, &m9, None);
    let (step, error) = &output.step_errors[0];
    assert_eq!(step, "table_screw");
    let error = error.to_string();
    assert!(
        error.contains("\"screw\" is \"M9\", which names no row of its table: M3, M5"),
        "{error}"
    );
    assert!(output.step_errors.iter().any(|(step, _)| step == "half"));
}
