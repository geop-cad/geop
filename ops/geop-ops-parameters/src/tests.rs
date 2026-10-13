//! A program file run as a part is: its parameters resolved to the numbers
//! a step reads, what it says of its part given by the given step, and its
//! formulas found and renamed in the steps' saved arguments.

use std::{cell::Cell, rc::Rc};

use geop_core_editor::numeric_input::NumericInput;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    operation::{Operation, StepArgs},
    operation_registry::OperationRegistry,
    part::Part,
    target::TargetReference,
};
use serde::{Deserialize, Serialize};

use crate::{
    Design, Parameter, ParameterKind, ProgramFile, Runner, Setting, Settings, StateValue,
    about::{define_about, read_about},
    parameters::Row,
    pose_state,
};

/// What a test step makes, read whole.
#[derive(Clone, Debug, PartialEq)]
struct Made<T>(T);

impl<T: 'static> Kind for Made<T> {
    const NAME: &'static str = "made";
}

impl<T: 'static> HasElements<Made<T>> for Made<T> {
    type Element<'a> = &'a T;

    fn element(&self, name: &str) -> GeopResult<&T> {
        Ok(&whole(self, name)?.0)
    }
}

fn made<T: Clone + 'static>(runner: &Runner, step: &str) -> T {
    let reader = runner.part.current();
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
    type Input = Design;
    type State = StateValue;

    fn run(step: &str, part: &mut Part<Design, StateValue>, args: &LengthArgs) -> GeopResult<()> {
        part.define_target(
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
    type Input = Design;
    type State = StateValue;

    fn run(step: &str, part: &mut Part<Design, StateValue>, _: &ColourArgs) -> GeopResult<()> {
        part.define_target(&TargetReference::new(step), &(), |reader, _| {
            RUNS.set(RUNS.get() + 1);
            Ok(Made(read_about(reader)?.color().unwrap_or("none").to_string()))
        })
    }
}

fn runner() -> Runner {
    let registry = OperationRegistry::default().with(Length).unwrap().with(Colour).unwrap();
    Runner::new(registry)
}

fn file() -> ProgramFile {
    let mut file = ProgramFile::new();
    file.parameters.color = Some("#ff8800".into());
    file.parameters.values = vec![
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
    let length = |formula: &str| LengthArgs {
        length: NumericInput::from(formula),
    };
    file.push("half", length("width / 2"));
    file.push("plain", length("7"));
    file.push("colour", ColourArgs);
    file
}

/// Runs `file` with `overrides`, the given step defining what it says of
/// its part.
fn run(runner: &mut Runner, file: &ProgramFile, overrides: &Settings) {
    let about = file.about("parts/screw.geop", overrides);
    runner.given = Some(Rc::new(move |part| define_about(part, &about)));
    file.run(runner, overrides).unwrap();
}

fn length(runner: &Runner, step: &str) -> f64 {
    made(runner, step)
}

/// A step reads the numbers its formula names, and what the program says
/// of its part through the given step; a run with other settings rebuilds
/// only the steps that read what changed.
#[test]
fn a_run_reads_parameters_as_numbers_and_the_rest_as_given() {
    let file = file();
    let mut runner = runner();
    run(&mut runner, &file, &Settings::new());
    assert_eq!(length(&runner, "half"), 3.0);
    assert_eq!(length(&runner, "plain"), 7.0);
    assert_eq!(made::<String>(&runner, "colour"), "#ff8800");
    let reader = runner.part.current();
    let about = read_about(&reader).unwrap();
    assert_eq!(about.stem(), "screw");
    assert_eq!(about.chosen["screw"], Setting::Text("M3".into()));
    drop(reader);

    RUNS.set(0);
    let m5 = Settings::from([("screw".to_string(), Setting::Text("M5".into()))]);
    run(&mut runner, &file, &m5);
    assert_eq!(length(&runner, "half"), 5.0);
    // The length the row changed, and the colour, since the row is said in
    // what the part is: not the plain length.
    assert_eq!(RUNS.get(), 2);
}

/// A step copied takes the state stored under its id along (where a placed
/// part is), and pasted under a fresh id stores it under that one; state
/// merely named like it stays behind.
#[test]
fn a_pasted_step_takes_its_state_along() {
    let mut file = file();
    let pose = StateValue::Pose(geop_core_math::primitives::Pose::identity());
    file.state.insert(pose_state("half"), pose.clone());
    (file.state).insert("half_width".into(), StateValue::Number(Design::from_f64(2.0)));

    let copied = file.excerpt("half").unwrap();
    assert_eq!(copied.state.len(), 1);
    let at = file.program.steps.len();
    let ids = file.paste(copied, at, |s| s.operation.clone()).unwrap();
    assert_eq!(ids, ["length1"]);
    assert_eq!(file.program.steps[at].id, "length1");
    assert_eq!(file.state.get(&pose_state("length1")), Some(&pose));
    assert!(!file.state.contains_key("length1_width"));
    file.validate().unwrap();
}

/// What reads a parameter is found in the formulas of the steps' saved
/// arguments and of the other parameters, and renaming it renames them.
#[test]
fn a_parameter_is_renamed_wherever_it_is_read() {
    let mut file = file();
    let uses = file.parameter_uses();
    assert_eq!(uses["width"], ["half"]);
    assert_eq!(uses["screw"], ["width"]);

    file.rename_parameter("width", "span").unwrap();
    assert_eq!(file.parameter_uses()["span"], ["half"]);
    let mut runner = runner();
    run(&mut runner, &file, &Settings::new());
    assert_eq!(length(&runner, "half"), 3.0);
    assert!(file.rename_parameter("span", "screw").is_err());
    assert!(file.rename_parameter("nothing", "x").is_err());

    let saved = file.to_json().unwrap();
    assert_eq!(ProgramFile::from_json(&saved).unwrap(), file);
}

/// A whole target is read as its one element of the empty name.
#[test]
fn an_element_reference_reads_a_given_target_whole() {
    let file = file();
    let mut runner = runner();
    run(&mut runner, &file, &Settings::new());
    let reader = runner.part.current();
    let about = reader.retrieve_element(&ElementReference::whole(crate::about()));
    assert_eq!(about.unwrap().file, "parts/screw.geop");
}
