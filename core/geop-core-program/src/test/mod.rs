//! The program machinery run on toy operations ([`operations`]): what is
//! built again when a step, an input, the state or a placed part changes,
//! what a step sees of the steps before it, and how a program is checked,
//! saved and read.

pub(crate) mod instances;
mod kept;
mod operations;
mod suppress;

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::{
    primitives::Pose,
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use crate::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    placed::Located,
    program::{Program, ProgramStep},
    runner::{RunOutput, Runner},
    target::TargetReference,
    values::{Setting, Settings, StateValue, StateValues},
};
pub(crate) use operations::{
    AddTwoDatums, AddTwoDatumsArgs, AddTwoFrameArgs, AddTwoFrameOperation, Datum, Datums, Frame,
    InputFrameArgs, InputFrameOperation, MiddleDatum, MiddleDatumArgs, MiddleFrameArgs,
    MiddleFrameOperation, NumberArgs, NumberOperation, ShiftFrameArgs, ShiftFrameOperation,
    StateFrameArgs, StateFrameOperation,
};

/// The state of a body at `position`, unturned.
pub(crate) fn pose_at(position: [f64; 3]) -> StateValue {
    StateValue::Pose(Pose::identity().with_position(vector(position)))
}

/// A vector of sharp scalars, from plain numbers.
pub(crate) fn vector<const N: usize>(xs: [f64; N]) -> Vector<N> {
    Vector::from_array(xs.map(Scalar::from_f64))
}

/// Asserts that `actual` encloses the point `expected`: computed values are
/// enclosures, rounded outward, so equal to no literal.
#[track_caller]
pub(crate) fn assert_encloses<const N: usize>(actual: Vector<N>, expected: [f64; N]) {
    assert!(
        actual.could_be_equal(&vector(expected)),
        "{actual:?} does not enclose {expected:?}"
    );
}

/// The frame target `name`, whole.
pub(crate) fn whole_frame(name: &str) -> ElementReference<Frame, Frame> {
    ElementReference::whole(TargetReference::located(Located::of_key(name).unwrap()))
}

/// A reference to the datum `name`.
fn datum(name: &str) -> ElementReference<Datums, Datum> {
    ElementReference::in_singleton(name).unwrap()
}

thread_local! {
    /// The versions this test's generators built, as `target@step`, in order.
    static BUILT: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

/// The label a version is logged under when built. Generators capture
/// nothing, so it travels in their arguments.
pub(crate) fn label(target: &str, step: &str) -> String {
    format!("{target}@{step}")
}

pub(crate) fn record_built(label: &str) {
    BUILT.with(|built| built.borrow_mut().push(label.to_string()));
}

pub(crate) fn take_built() -> Vec<String> {
    BUILT.with(|built| built.take())
}

pub(crate) fn registry() -> OperationRegistry {
    OperationRegistry::default()
        .with(AddTwoFrameOperation)
        .and_then(|o| o.with(MiddleFrameOperation))
        .and_then(|o| o.with(InputFrameOperation))
        .and_then(|o| o.with(StateFrameOperation))
        .and_then(|o| o.with(ShiftFrameOperation))
        .and_then(|o| o.with(AddTwoDatums))
        .and_then(|o| o.with(MiddleDatum))
        .and_then(|o| o.with(instances::LoadOperation))
        .and_then(|o| o.with(instances::CountOperation))
        .and_then(|o| o.with(NumberOperation))
        .unwrap()
}

/// A runner of the tests' operations, and the part it runs on.
pub(crate) struct Running {
    pub(crate) runner: Runner,
    pub(crate) part: Part,
}

impl Running {
    /// Runs `program` with `inputs`, at `state`.
    pub(crate) fn run(
        &mut self,
        program: &Program,
        inputs: &Settings,
        state: &StateValues,
    ) -> RunOutput {
        self.run_until(program, program.steps.len(), inputs, state)
    }

    /// Runs the first `end` steps of `program` with `inputs`, at `state`.
    pub(crate) fn run_until(
        &mut self,
        program: &Program,
        end: usize,
        inputs: &Settings,
        state: &StateValues,
    ) -> RunOutput {
        let program = Program {
            steps: program.steps.clone(),
            state: state.clone(),
        };
        let upto = program.steps.get(end).map(|step| step.id.clone());
        (self.runner).run_with(&mut self.part, &program, inputs, upto.as_deref())
    }
}

pub(crate) fn runner() -> Running {
    let mut runner = Runner::new(registry());
    runner.workspace = Some(std::rc::Rc::new(instances::Files));
    Running {
        runner,
        part: Part::default(),
    }
}

pub(crate) fn add_two_frame(id: &str, data1: [f64; 3], data2: [f64; 3]) -> ProgramStep {
    let args = AddTwoFrameArgs {
        data1: vector(data1),
        data2: vector(data2),
    };
    ProgramStep::new::<AddTwoFrameOperation>(id, &args).unwrap()
}

pub(crate) fn middle_frame(id: &str, frame1: &str, frame2: &str) -> ProgramStep {
    let args = MiddleFrameArgs {
        frame1: whole_frame(frame1),
        frame2: whole_frame(frame2),
    };
    ProgramStep::new::<MiddleFrameOperation>(id, &args).unwrap()
}

pub(crate) fn input_frame(id: &str, target: &str, input: &str) -> ProgramStep {
    let args = InputFrameArgs {
        target: target.to_string(),
        input: input.to_string(),
    };
    ProgramStep::new::<InputFrameOperation>(id, &args).unwrap()
}

pub(crate) fn shift_frame(id: &str, target: &str, x: f64) -> ProgramStep {
    let args = ShiftFrameArgs {
        targets: vec![target.to_string()],
        by: vector([x, 0.0, 0.0]),
    };
    ProgramStep::new::<ShiftFrameOperation>(id, &args).unwrap()
}

/// The parameter `name`, `value` unless the run's input sets it.
pub(crate) fn number(id: &str, name: &str, value: f64) -> ProgramStep {
    let args = NumberArgs {
        name: name.to_string(),
        value,
    };
    ProgramStep::new::<NumberOperation>(id, &args).unwrap()
}

pub(crate) fn add_two_datums(id: &str, data1: [f64; 3], data2: [f64; 3]) -> ProgramStep {
    let args = AddTwoDatumsArgs {
        data1: vector(data1),
        data2: vector(data2),
    };
    ProgramStep::new::<AddTwoDatums>(id, &args).unwrap()
}

pub(crate) fn middle_datum(id: &str, a: &str, b: &str) -> ProgramStep {
    let args = MiddleDatumArgs {
        a: datum(a),
        b: datum(b),
    };
    ProgramStep::new::<MiddleDatum>(id, &args).unwrap()
}

pub(crate) fn inputs(values: &[(&str, f64)]) -> Settings {
    values
        .iter()
        .map(|(name, value)| (name.to_string(), Setting::Number(*value)))
        .collect()
}

pub(crate) fn no_state() -> StateValues {
    BTreeMap::new()
}

pub(crate) fn frame(runner: &Running, name: &str) -> Vector3 {
    let view = runner.part.current();
    view.retrieve_element(&whole_frame(name)).unwrap().data
}

/// The frame `name` as it was before the step `step`.
fn frame_before(runner: &Running, step: &str, name: &str) -> Vector3 {
    let view = runner.part.before(step);
    view.retrieve_element(&whole_frame(name)).unwrap().data
}

/// The datum `name`, as the part is now.
fn datum_at(runner: &Running, name: &str) -> Vector3 {
    let current = runner.part.current();
    current.retrieve_element(&datum(name)).unwrap().data
}

/// The failed steps and what went wrong at the root.
pub(crate) fn errors(output: &RunOutput) -> Vec<(String, String)> {
    output
        .step_errors
        .iter()
        .map(|(step, e)| (step.clone(), e.root_message().to_string()))
        .collect()
}

pub(crate) fn ok(output: RunOutput) {
    assert_eq!(errors(&output), vec![]);
}

#[test]
fn rebuilds_only_what_changed() {
    let mut runner = runner();
    let program = |frame1| {
        Program::from_steps(vec![
            add_two_frame("step1", frame1, [0.0, 0.0, 1.0]),
            middle_frame("step2", "step1_1", "step1_2"),
        ])
    };

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &no_state()));
    assert_eq!(
        take_built(),
        ["step1_1@step1", "step1_2@step1", "step2@step2"]
    );
    assert_encloses(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_encloses(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([1.0, 0.0, 0.0]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), ["step1_1@step1", "step2@step2"]);
    assert_encloses(frame(&runner, "step2"), [0.5, 0.0, 0.5]);
}

/// Every step revises the one target of datums, and a step builds on a
/// datum by an element reference into it: a step reading datums of an
/// earlier one is rebuilt when that one changes.
#[test]
fn datums_are_revised_by_each_step_and_read_as_elements() {
    let mut runner = runner();
    let program = |first: [f64; 3]| {
        Program::from_steps(vec![
            add_two_datums("s1", first, [0.0, 0.0, 1.0]),
            middle_datum("s2", "s1_1", "s1_2"),
        ])
    };

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), ["datums@s1", "datums@s2"]);
    assert_encloses(datum_at(&runner, "s1_1"), [0.0; 3]);
    assert_encloses(datum_at(&runner, "s1_2"), [0.0, 0.0, 1.0]);
    assert_encloses(datum_at(&runner, "s2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([1.0, 0.0, 0.0]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), ["datums@s1", "datums@s2"]);
    assert_encloses(datum_at(&runner, "s2"), [0.5, 0.0, 0.5]);
}

/// A datum that is not there is named, with the reference that asked for
/// it.
#[test]
fn a_missing_datum_is_named() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        add_two_datums("s1", [0.0; 3], [1.0, 0.0, 0.0]),
        middle_datum("m", "s1_1", "s9_2"),
    ]);
    let output = runner.run(&program, &inputs(&[]), &no_state());
    assert_eq!(
        errors(&output),
        [("m".to_string(), "there is no datum `s9_2`".to_string())]
    );
}

/// In the one target of datums a part has, a step's arguments name a datum
/// by its name alone.
#[test]
fn a_program_names_datums_by_their_names() {
    let step = middle_datum("m", "s1_1", "s1_2");
    assert_eq!(
        serde_json::to_value(&step.args).unwrap(),
        serde_json::json!({"a": "s1_1", "b": "s1_2"})
    );
}

/// A failure is not kept as a result: once its cause is gone, the failed
/// target and what reads it are rebuilt from current data.
#[test]
fn failed_target_is_rebuilt_once_its_input_returns() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        input_frame("b", "b", "y"),
        middle_frame("m", "a", "b"),
    ]);

    ok(runner.run(&program, &inputs(&[("x", 1.0), ("y", 3.0)]), &no_state()));
    assert_encloses(frame(&runner, "m"), [2.0, 0.0, 0.0]);
    take_built();

    let output = runner.run(&program, &inputs(&[("x", 2.0)]), &no_state());
    assert_eq!(
        errors(&output),
        [
            ("b".into(), "input `y` is not given".into()),
            (
                "m".into(),
                "target `b` failed in step `b`: input `y` is not given".into()
            ),
        ]
    );
    assert_eq!(take_built(), ["a@a", "b@b", "m@m"]);

    ok(runner.run(&program, &inputs(&[("x", 2.0), ("y", 3.0)]), &no_state()));
    assert_eq!(take_built(), ["b@b", "m@m"]);
    assert_encloses(frame(&runner, "m"), [2.5, 0.0, 0.0]);
}

/// Nodes are found by their step's id, not its position: swapping two
/// steps that do not read each other makes nothing again, and what reads
/// both still sees the same producers.
#[test]
fn reordered_independent_steps_are_reused() {
    let mut runner = runner();
    let (a, b) = (input_frame("a", "a", "x"), input_frame("b", "b", "y"));
    let program = |first: &ProgramStep, second: &ProgramStep| {
        Program::from_steps(vec![
            first.clone(),
            second.clone(),
            middle_frame("m", "a", "b"),
        ])
    };
    let xy = inputs(&[("x", 1.0), ("y", 3.0)]);
    ok(runner.run(&program(&a, &b), &xy, &no_state()));
    assert_eq!(take_built(), ["a@a", "b@b", "m@m"]);
    let makes = runner.runner.counts().makes;

    ok(runner.run(&program(&b, &a), &xy, &no_state()));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_eq!(runner.runner.counts().makes, makes);
    assert_encloses(frame(&runner, "m"), [2.0, 0.0, 0.0]);
}

/// A step reads only targets defined before it in the same run, never what
/// an earlier run left behind.
#[test]
fn step_reads_only_targets_defined_before_it() {
    let mut runner = runner();
    let add = || add_two_frame("step1", [0.0; 3], [0.0, 0.0, 1.0]);
    let middle = || middle_frame("step2", "step1_1", "step1_2");
    let not_defined = vec![(
        "step2".to_string(),
        "no step before here produces target `step1_1` (the targets before here: none)".to_string(),
    )];

    ok(runner.run(
        &Program::from_steps(vec![add(), middle()]),
        &inputs(&[]),
        &no_state(),
    ));

    let reordered = runner.run(
        &Program::from_steps(vec![middle(), add()]),
        &inputs(&[]),
        &no_state(),
    );
    assert_eq!(errors(&reordered), not_defined);

    let removed = runner.run(
        &Program::from_steps(vec![middle()]),
        &inputs(&[]),
        &no_state(),
    );
    assert_eq!(errors(&removed), not_defined);
    let current = runner.part.current();
    assert!(current.retrieve_element(&whole_frame("step1_1")).is_err());
}

/// A target that is no chain is produced by one step, and a second
/// producer fails, naming both, rather than replacing the first.
#[test]
fn target_that_is_no_chain_produced_twice_is_an_error() {
    let runner = Runner::new(registry());
    let mut part = Part::default();
    let a = TargetReference::<f64>::new("a");
    let given = |_: &crate::reader::Reader, &x: &f64| Ok(x);
    (runner.run_step(&mut part, "first", |ctx| ctx.produce(&a, &1.0, given))).unwrap();
    let second = runner.run_step(&mut part, "second", |ctx| ctx.produce(&a, &2.0, given));
    assert_eq!(
        second.unwrap_err().root_message(),
        "target `a` is produced by step `first` and again by step `second`: only a chain \
         target may be produced by several steps"
    );
}

/// A run's inputs are complete: one it no longer gives is gone.
#[test]
fn removed_input_is_gone() {
    let mut runner = runner();
    let program = Program::from_steps(vec![input_frame("a", "a", "x")]);

    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));
    let output = runner.run(&program, &inputs(&[]), &no_state());
    assert_eq!(
        errors(&output),
        [("a".into(), "input `x` is not given".into())]
    );
}

#[test]
fn changed_state_rebuilds_what_read_it() {
    let mut runner = runner();
    let args = StateFrameArgs {
        state: "body".to_string(),
    };
    let program = Program::from_steps(vec![
        ProgramStep::new::<StateFrameOperation>("s", &args).unwrap(),
    ]);
    let state = |x| StateValues::from([("body".to_string(), pose_at([x, 0.0, 0.0]))]);

    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert_encloses(frame(&runner, "s"), [0.0; 3]);
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    assert_eq!(take_built(), ["s@s", "s@s"]);
    assert_encloses(frame(&runner, "s"), [1.0, 0.0, 0.0]);
}

#[test]
fn unknown_operation_fails_its_step_only() {
    let mut runner = Running {
        runner: Runner::new(
            OperationRegistry::default()
                .with(InputFrameOperation)
                .unwrap(),
        ),
        part: Part::default(),
    };
    let args = StateFrameArgs {
        state: "body".to_string(),
    };
    let program = Program::from_steps(vec![
        ProgramStep::new::<StateFrameOperation>("bad", &args).unwrap(),
        input_frame("a", "a", "x"),
    ]);

    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &no_state());
    assert_eq!(
        errors(&output),
        [("bad".into(), "unknown operation `state_frame`".into())]
    );
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
}

/// Each revision of a target is a version of its own: a step reads the
/// latest one before it, and changing one revision rebuilds that version and
/// what follows, not what came before.
#[test]
fn revisions_are_versions() {
    let mut runner = runner();
    let program = |first_shift| {
        Program::from_steps(vec![
            input_frame("a", "a", "x"),
            shift_frame("s1", "a", first_shift),
            middle_frame("m", "a", "a"),
            shift_frame("s2", "a", 10.0),
        ])
    };

    ok(runner.run(&program(1.0), &inputs(&[("x", 1.0)]), &no_state()));
    assert_eq!(take_built(), ["a@a", "a@s1", "m@m", "a@s2"]);
    assert_encloses(frame(&runner, "m"), [2.0, 0.0, 0.0]);
    assert_encloses(frame(&runner, "a"), [12.0, 0.0, 0.0]);
    assert_encloses(frame_before(&runner, "s1", "a"), [1.0, 0.0, 0.0]);
    assert_encloses(frame_before(&runner, "m", "a"), [2.0, 0.0, 0.0]);
    assert_encloses(frame_before(&runner, "s2", "a"), [2.0, 0.0, 0.0]);

    ok(runner.run(&program(2.0), &inputs(&[("x", 1.0)]), &no_state()));
    assert_eq!(take_built(), ["a@s1", "m@m", "a@s2"]);
    assert_encloses(frame(&runner, "m"), [3.0, 0.0, 0.0]);
    assert_encloses(frame(&runner, "a"), [13.0, 0.0, 0.0]);
}

/// A step that revises a target reads whichever version comes before it,
/// so inserting or removing a revision before it rebuilds it, though the
/// version it read before is unchanged.
#[test]
fn inserted_revision_rebuilds_what_follows() {
    let mut runner = runner();
    let with_s1 = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 10.0),
    ]);
    let without_s1 = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        shift_frame("s2", "a", 10.0),
    ]);
    let x = inputs(&[("x", 1.0)]);

    ok(runner.run(&without_s1, &x, &no_state()));
    take_built();
    ok(runner.run(&with_s1, &x, &no_state()));
    assert_eq!(take_built(), ["a@s1", "a@s2"]);
    assert_encloses(frame(&runner, "a"), [12.0, 0.0, 0.0]);
    ok(runner.run(&without_s1, &x, &no_state()));
    assert_eq!(take_built(), ["a@s2"]);
    assert_encloses(frame(&runner, "a"), [11.0, 0.0, 0.0]);
}

#[test]
fn step_makes_one_version_of_a_target() {
    let mut runner = runner();
    let twice = ShiftFrameArgs {
        targets: vec!["a".to_string(), "a".to_string()],
        by: vector([1.0, 0.0, 0.0]),
    };
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        ProgramStep::new::<ShiftFrameOperation>("s", &twice).unwrap(),
    ]);
    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &no_state());
    assert_eq!(
        errors(&output),
        [(
            "s".into(),
            "target `a` is produced twice by step `s`".into()
        )]
    );
    assert_encloses(frame(&runner, "a"), [2.0, 0.0, 0.0]);
}

/// A revision with no earlier version revises the default: a frame no step
/// defined is shifted from the origin.
#[test]
fn revision_without_an_earlier_version_starts_from_the_default() {
    let mut runner = runner();
    let program = Program::from_steps(vec![shift_frame("s", "a", 1.0)]);
    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
}

/// A version rebuilt into an equal value counts as unchanged, so what read
/// it is reused.
#[test]
fn equal_rebuild_keeps_what_read_it() {
    let mut runner = runner();
    let program = |first, second| {
        Program::from_steps(vec![
            input_frame("a", "a", "x"),
            input_frame("b", "b", "y"),
            middle_frame("m", first, second),
            shift_frame("s", "m", 1.0),
        ])
    };
    let xy = inputs(&[("x", 1.0), ("y", 3.0)]);

    ok(runner.run(&program("a", "b"), &xy, &no_state()));
    take_built();
    ok(runner.run(&program("b", "a"), &xy, &no_state()));
    assert_eq!(take_built(), ["m@m"]);
    assert_encloses(frame(&runner, "m"), [3.0, 0.0, 0.0]);
}

/// A program is plain data: it saves as it is, and a saved program runs.
#[test]
fn program_round_trips_through_json() {
    let program = Program::from_steps(vec![input_frame("a", "a", "x"), shift_frame("s", "a", 1.0)]);

    let json = serde_json::to_value(&program).unwrap();
    assert_eq!(
        json,
        serde_json::json!({"steps": [
            {"id": "a", "operation": "input_frame", "args": {"target": "a", "input": "x"}},
            {"id": "s", "operation": "shift_frame", "args": {"targets": ["a"], "by": [1.0, 0.0, 0.0]}},
        ]})
    );
    let loaded: Program = serde_json::from_value(json).unwrap();
    assert_eq!(loaded, program);

    let mut runner = runner();
    ok(runner.run(&loaded, &inputs(&[("x", 1.0)]), &no_state()));
    assert_encloses(frame(&runner, "a"), [2.0, 0.0, 0.0]);
}

/// Arguments that do not fit the operation fail the step, naming it.
#[test]
fn malformed_arguments_fail_their_step() {
    let mut runner = runner();
    let mut step = input_frame("a", "a", "x");
    step.args = serde_json::json!({"target": 3});
    let output = runner.run(&Program::from_steps(vec![step]), &inputs(&[]), &no_state());
    let errors = errors(&output);
    assert_eq!(errors.len(), 1);
    assert!(
        errors[0]
            .1
            .starts_with("arguments of operation `input_frame`: "),
        "{errors:?}"
    );
}

/// Running up to a marker keeps what the steps after it made, so moving the
/// marker back to the end reuses them.
#[test]
fn running_to_a_marker_keeps_later_steps() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 10.0),
    ]);
    let x = inputs(&[("x", 1.0)]);

    ok(runner.run(&program, &x, &no_state()));
    take_built();
    ok(runner.run_until(&program, 1, &x, &no_state()));
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    ok(runner.run(&program, &x, &no_state()));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_encloses(frame(&runner, "a"), [12.0, 0.0, 0.0]);
}

fn body_at(x: f64) -> StateValues {
    StateValues::from([("body".to_string(), pose_at([x, 0.0, 0.0]))])
}

#[test]
fn operation_registered_twice_is_an_error() {
    let twice = OperationRegistry::default()
        .with(InputFrameOperation)
        .and_then(|o| o.with(InputFrameOperation));
    assert_eq!(
        twice.err().unwrap().root_message(),
        "operation `input_frame` is registered twice"
    );
}

/// A program is checked against the operations it is run in before it is
/// run: a step of an operation there is none of, a step id outside the id
/// alphabet or used twice, and arguments the operation does not read are
/// each refused, naming the step.
#[test]
fn a_program_is_validated_against_its_operations() {
    let registry = registry();
    let valid = Program::from_steps(vec![input_frame("a", "a", "x")]);
    valid.validate(&registry).unwrap();
    let refused = |program: &Program| program.validate(&registry).unwrap_err().to_string();

    let mut unknown = valid.clone();
    unknown.steps[0].operation = "nothing".into();
    let error = refused(&unknown);
    assert!(
        error.contains("step `a`") && error.contains("unknown operation `nothing`"),
        "{error}"
    );

    let mut badly_named = valid.clone();
    badly_named.steps[0].id = "a(b)".into();
    assert!(refused(&badly_named).contains("not a valid operation id"));

    let twice = Program::from_steps(vec![input_frame("a", "a", "x"), input_frame("a", "b", "x")]);
    assert!(refused(&twice).contains("more than one step with id \"a\""));

    let mut unreadable = valid.clone();
    unreadable.steps[0].args = serde_json::json!({"target": 3});
    let error = refused(&unreadable);
    assert!(
        error.contains("step `a`") && error.contains("arguments of operation"),
        "{error}"
    );
}

/// A program and its state read back from their file as they were saved:
/// its steps as they were, its state saved again as it was (a pose read is
/// an enclosure of the rigid motion it stands for).
#[test]
fn a_program_reads_back_as_saved() {
    let mut program = Program::from_steps(vec![input_frame("a", "a", "x")]);
    program
        .state
        .insert("body".into(), pose_at([1.0, 2.0, 3.0]));
    let saved = program.to_json().unwrap();
    let back = Program::from_json(&saved).unwrap();
    assert_eq!(back.steps, program.steps);
    assert_eq!(back.to_json().unwrap(), saved);
    let file: serde_json::Value = serde_json::from_str(&saved).unwrap();
    assert!(
        file.get("steps").is_some() && file.get("state").is_some(),
        "{file}"
    );
}

/// A file just made, empty, is a part of no steps: a part placed from it
/// before it was ever opened is built, as the editors open it.
#[test]
fn an_empty_file_reads_as_an_empty_program() {
    for text in ["", " \n"] {
        assert_eq!(Program::from_json(text).unwrap(), Program::new());
    }
}

/// Runs edited one after another on one part build what each builds from
/// scratch, failures included.
#[test]
fn edited_programs_build_what_they_build_from_scratch() {
    let program = |x: f64, middle: bool| {
        let mut steps = vec![add_two_frame("p", [x, 0.0, 0.0], [0.0, 0.0, 1.0])];
        if middle {
            steps.push(middle_frame("m", "p_1", "p_2"));
        }
        steps.push(middle_frame("n", "m", "p_1"));
        Program::from_steps(steps)
    };
    let programs = [
        program(0.0, true),
        program(1.0, true),
        program(1.0, false),
        program(1.0, true),
        program(0.0, true),
    ];
    let workspace: Rc<dyn crate::workspace::Workspace> = Rc::new(instances::Files);
    crate::testing::assert_incremental(Rc::new(registry()), Some((workspace, "")), &programs);
}

/// A step's id is what tells its versions from another step's, so a
/// program cannot use one twice.
#[test]
fn step_id_used_twice_is_an_error() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        input_frame("a", "b", "x"),
        input_frame("c", "c", "x"),
    ]);
    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &no_state());
    assert_eq!(
        errors(&output),
        [("a".into(), "step `a` appears twice in the program".into())]
    );
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    assert_encloses(frame(&runner, "c"), [1.0, 0.0, 0.0]);
    let current = runner.part.current();
    assert!(current.retrieve_element(&whole_frame("b")).is_err());
}

/// A value built up from nothing starts from its default at the first step
/// that builds it, is revised by the later ones, and a failed version is no
/// fresh start for the step after it.
#[test]
fn the_first_revision_begins_from_the_default() {
    fn grow(
        reader: &crate::reader::Reader,
        by: &Vector3,
    ) -> geop_core_math::geop_error::GeopResult<Frame> {
        let frame: Frame = reader.previous()?;
        Ok(Frame {
            data: frame.data.add(by),
        })
    }
    let runner = Runner::new(registry());
    let mut part = Part::default();
    let reference = TargetReference::<Frame>::new("grown");
    let mut step = |id: &str, by: [f64; 3]| {
        runner.run_step(&mut part, id, |ctx| {
            ctx.produce(&reference, &vector(by), grow)
        })
    };
    step("s1", [1.0, 0.0, 0.0]).unwrap();
    step("s2", [2.0, 0.0, 0.0]).unwrap();
    assert_encloses(
        (part.current())
            .retrieve_element(&ElementReference::whole(reference.clone()))
            .unwrap()
            .data,
        [3.0, 0.0, 0.0],
    );

    fn fail(_: &crate::reader::Reader, _: &()) -> geop_core_math::geop_error::GeopResult<Frame> {
        Err("broken".into())
    }
    let failed = runner.run_step(&mut part, "s3", |ctx| ctx.produce(&reference, &(), fail));
    assert!(failed.is_err());
    let after = runner.run_step(&mut part, "s4", |ctx| {
        ctx.produce(&reference, &vector([1.0, 0.0, 0.0]), grow)
    });
    assert_eq!(
        after.unwrap_err().root_message(),
        "target `grown` failed in step `s3`: broken"
    );
}

/// A revision that started from the default, no step having made its
/// target before it, starts from the version of a step inserted before it
/// once one is: that no version was made is something it read.
#[test]
fn a_revision_from_the_default_follows_a_step_inserted_before_it() {
    let mut runner = runner();
    ok(runner.run(
        &Program::from_steps(vec![shift_frame("s1", "a", 1.0)]),
        &inputs(&[]),
        &no_state(),
    ));
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    ok(runner.run(
        &Program::from_steps(vec![
            shift_frame("s0", "a", 5.0),
            shift_frame("s1", "a", 1.0),
        ]),
        &inputs(&[]),
        &no_state(),
    ));
    assert_encloses(frame(&runner, "a"), [6.0, 0.0, 0.0]);
}

/// A view lists the targets of a kind it sees, in the order their steps
/// made them: those made before it, not after.
#[test]
fn view_lists_targets_of_a_kind() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("b", "b", "x"),
        input_frame("a", "a", "x"),
        input_frame("c", "c", "x"),
    ]);
    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));
    fn names(view: crate::reader::Reader<'_>) -> Vec<String> {
        let frames = view.targets_of::<Frame>();
        frames.iter().map(|f| f.name().to_string()).collect()
    }
    assert_eq!(names(runner.part.current()), ["b", "a", "c"]);
    assert_eq!(names(runner.part.before("c")), ["b", "a"]);
    assert!(runner.part.current().targets_of::<Pose>().is_empty());
}

/// A run at a new state, as a solver runs the program at each state it
/// tries, makes again what read the state and what reads that — and
/// nothing else: a run again at that state finds it all current.
#[test]
fn a_new_state_rebuilds_only_what_read_it() {
    let mut runner = runner();
    let follow = StateFrameArgs {
        state: "body".to_string(),
    };
    let program = Program::from_steps(vec![
        ProgramStep::new::<StateFrameOperation>("s", &follow).unwrap(),
        add_two_frame("pair", [0.0; 3], [2.0, 0.0, 0.0]),
        middle_frame("m", "s", "pair_2"),
        input_frame("a", "a", "x"),
    ]);
    let x = inputs(&[("x", 1.0)]);
    ok(runner.run(&program, &x, &body_at(3.0)));
    take_built();

    ok(runner.run(&program, &x, &body_at(1.0)));
    assert_eq!(take_built(), ["s@s", "m@m"]);
    assert_encloses(frame(&runner, "m"), [1.5, 0.0, 0.0]);

    ok(runner.run(&program, &x, &body_at(1.0)));
    assert_eq!(take_built(), Vec::<String>::new());
}

/// A step copied takes the state stored under its id along (where a placed
/// part is), and pasted under a fresh id stores it under that one; state
/// merely named like it stays behind.
#[test]
fn a_pasted_step_takes_its_state_along() {
    let mut program = Program::from_steps(vec![input_frame("a", "a", "x")]);
    let id = program.steps[0].id.clone();
    let pose = pose_at([1.0, 2.0, 3.0]);
    (program.state).insert(crate::values::pose_state(&id), pose.clone());
    program.state.insert(
        format!("{id}_width"),
        StateValue::Number(Scalar::from_f64(2.0)),
    );

    let copied = program.excerpt(&id).unwrap();
    assert_eq!(copied.state.len(), 1);
    let at = program.steps.len();
    let ids = (program.paste(copied, at, &registry(), |s| s.operation.clone())).unwrap();
    assert_eq!(ids.len(), 1);
    assert_ne!(ids[0], id);
    assert_eq!(program.steps[at].id, ids[0]);
    let pasted_pose = crate::values::pose_state(&ids[0]);
    assert_eq!(program.state.get(&pasted_pose), Some(&pose));
    assert!(!program.state.contains_key(&format!("{}_width", ids[0])));
    program.validate(&registry()).unwrap();
}
