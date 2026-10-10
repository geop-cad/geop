mod add_two_frame;
mod given_frame;
mod middle_frame;

use std::{cell::RefCell, collections::BTreeMap};

use crate::{
    operation::Registered,
    part::Pose,
    program::{Program, ProgramRunner, ProgramStep, RunOutput},
    target::TargetReference,
};
use add_two_frame::{AddTwoFrameArgs, AddTwoFrameOperation};
use given_frame::{InputFrameArgs, InputFrameOperation, StateFrameArgs, StateFrameOperation};
use middle_frame::{MiddleFrameArgs, MiddleFrameOperation};

#[derive(Debug, PartialEq)]
pub struct Frame {
    pub data: [f64; 3],
}

thread_local! {
    /// The targets this test's generators built, in order.
    static BUILT: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn record_built(name: &str) {
    BUILT.with(|built| built.borrow_mut().push(name.to_string()));
}

fn take_built() -> Vec<String> {
    BUILT.with(|built| built.take())
}

fn runner() -> ProgramRunner {
    ProgramRunner::new(BTreeMap::from([
        (
            "add_two_frame".to_string(),
            Registered::<AddTwoFrameOperation>::boxed(),
        ),
        (
            "middle_frame".to_string(),
            Registered::<MiddleFrameOperation>::boxed(),
        ),
        (
            "input_frame".to_string(),
            Registered::<InputFrameOperation>::boxed(),
        ),
        (
            "state_frame".to_string(),
            Registered::<StateFrameOperation>::boxed(),
        ),
    ]))
}

fn step(id: &str, operation_name: &str, args: impl std::any::Any) -> ProgramStep {
    ProgramStep {
        id: id.to_string(),
        operation_name: operation_name.to_string(),
        args: Box::new(args),
    }
}

fn add_two_frame(id: &str, data1: [f64; 3], data2: [f64; 3]) -> ProgramStep {
    step(id, "add_two_frame", AddTwoFrameArgs { data1, data2 })
}

fn middle_frame(id: &str, frame1: &str, frame2: &str) -> ProgramStep {
    let args = MiddleFrameArgs {
        frame1ref: frame1.to_string(),
        frame2ref: frame2.to_string(),
    };
    step(id, "middle_frame", args)
}

fn input_frame(id: &str, target: &str, input: &str) -> ProgramStep {
    let args = InputFrameArgs {
        target: target.to_string(),
        input: input.to_string(),
    };
    step(id, "input_frame", args)
}

fn inputs(values: &[(&str, f64)]) -> BTreeMap<String, f64> {
    values
        .iter()
        .map(|(name, value)| (name.to_string(), *value))
        .collect()
}

fn frame(runner: &ProgramRunner, name: &str) -> [f64; 3] {
    let reference = TargetReference::<Frame>::new(name.to_string());
    runner.part.retrieve_target(&reference).unwrap().data
}

/// The failed steps and their messages.
fn errors(output: &RunOutput) -> Vec<(String, String)> {
    output
        .step_errors
        .iter()
        .map(|(step, e)| (step.clone(), e.to_string()))
        .collect()
}

fn ok(output: RunOutput) {
    assert_eq!(errors(&output), vec![]);
}

#[test]
fn rebuilds_only_what_changed() {
    let mut runner = runner();
    let program = |frame1| {
        Program::new(vec![
            add_two_frame("step1", frame1, [0.0, 0.0, 1.0]),
            middle_frame("step2", "step1_1", "step1_2"),
        ])
    };

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &BTreeMap::new()));
    assert_eq!(take_built(), ["step1_1", "step1_2", "step2"]);
    assert_eq!(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &BTreeMap::new()));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_eq!(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([1.0, 0.0, 0.0]), &inputs(&[]), &BTreeMap::new()));
    assert_eq!(take_built(), ["step1_1", "step2"]);
    assert_eq!(frame(&runner, "step2"), [0.5, 0.0, 0.5]);
}

/// A failure is not kept as a result: once its cause is gone, the failed
/// target and what reads it are rebuilt from current data.
#[test]
fn failed_target_is_rebuilt_once_its_input_returns() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        input_frame("b", "b", "y"),
        middle_frame("m", "a", "b"),
    ]);

    ok(runner.run(
        &program,
        &inputs(&[("x", 1.0), ("y", 3.0)]),
        &BTreeMap::new(),
    ));
    assert_eq!(frame(&runner, "m"), [2.0, 0.0, 0.0]);
    take_built();

    let output = runner.run(&program, &inputs(&[("x", 2.0)]), &BTreeMap::new());
    assert_eq!(
        errors(&output),
        [
            (
                "b".into(),
                "target `b` failed: input `y` is not given".into()
            ),
            (
                "m".into(),
                "target `m` failed: target `b` failed: input `y` is not given".into()
            ),
        ]
    );
    assert_eq!(take_built(), ["a", "b", "m"]);

    ok(runner.run(
        &program,
        &inputs(&[("x", 2.0), ("y", 3.0)]),
        &BTreeMap::new(),
    ));
    assert_eq!(take_built(), ["b", "m"]);
    assert_eq!(frame(&runner, "m"), [2.5, 0.0, 0.0]);
}

/// A step reads only targets defined before it in the same run, never what
/// an earlier run left behind.
#[test]
fn step_reads_only_targets_defined_before_it() {
    let mut runner = runner();
    let add = || add_two_frame("step1", [0.0; 3], [0.0, 0.0, 1.0]);
    let middle = || middle_frame("step2", "step1_1", "step1_2");

    ok(runner.run(
        &Program::new(vec![add(), middle()]),
        &inputs(&[]),
        &BTreeMap::new(),
    ));

    let reordered = runner.run(
        &Program::new(vec![middle(), add()]),
        &inputs(&[]),
        &BTreeMap::new(),
    );
    assert_eq!(
        errors(&reordered),
        [(
            "step2".into(),
            "target `step2` failed: target `step1_1` is not defined before step `step2`".into()
        )]
    );

    let removed = runner.run(
        &Program::new(vec![middle()]),
        &inputs(&[]),
        &BTreeMap::new(),
    );
    assert_eq!(
        errors(&removed),
        [(
            "step2".into(),
            "target `step2` failed: target `step1_1` is not defined before step `step2`".into()
        )]
    );
    let reference = TargetReference::<Frame>::new("step1_1".to_string());
    assert!(runner.part.retrieve_target(&reference).is_err());
}

/// Every target is defined by one step, and a second definition fails
/// rather than replacing the first.
#[test]
fn target_defined_twice_is_an_error() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("first", "a", "x"),
        input_frame("second", "a", "y"),
    ]);

    for _ in 0..2 {
        let output = runner.run(
            &program,
            &inputs(&[("x", 1.0), ("y", 2.0)]),
            &BTreeMap::new(),
        );
        assert_eq!(
            errors(&output),
            [(
                "second".into(),
                "target `a` is defined twice: by step `first` and by step `second`".into()
            )]
        );
        assert_eq!(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    }
    assert_eq!(take_built(), ["a"]);
}

/// A run's inputs are complete: one it no longer gives is gone.
#[test]
fn removed_input_is_gone() {
    let mut runner = runner();
    let program = Program::new(vec![input_frame("a", "a", "x")]);

    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &BTreeMap::new()));
    let output = runner.run(&program, &inputs(&[]), &BTreeMap::new());
    assert_eq!(
        errors(&output),
        [(
            "a".into(),
            "target `a` failed: input `x` is not given".into()
        )]
    );
}

#[test]
fn changed_state_rebuilds_what_read_it() {
    let mut runner = runner();
    let program = Program::new(vec![step(
        "s",
        "state_frame",
        StateFrameArgs {
            state: "body".to_string(),
        },
    )]);
    let state = |x| {
        BTreeMap::from([(
            "body".to_string(),
            Pose::new([x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )])
    };

    ok(runner.run(&program, &inputs(&[]), &BTreeMap::new()));
    assert_eq!(frame(&runner, "s"), [0.0; 3]);
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    assert_eq!(take_built(), ["s", "s"]);
    assert_eq!(frame(&runner, "s"), [1.0, 0.0, 0.0]);
}

#[test]
fn unknown_operation_fails_its_step_only() {
    let mut runner = runner();
    let program = Program::new(vec![
        step("bad", "no_such_operation", ()),
        input_frame("a", "a", "x"),
    ]);

    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &BTreeMap::new());
    assert_eq!(
        errors(&output),
        [("bad".into(), "unknown operation `no_such_operation`".into())]
    );
    assert_eq!(frame(&runner, "a"), [1.0, 0.0, 0.0]);
}
