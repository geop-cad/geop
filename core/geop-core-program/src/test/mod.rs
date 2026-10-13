mod add_two_frame;
mod given_frame;
mod middle_frame;
mod shift_frame;

use std::{cell::RefCell, collections::BTreeMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{ScalInF64, Scalar},
    vector::{Vector, Vector3},
};

use crate::{
    element::{ElementReference, HasElements, Kind},
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    program_runner::{ProgramRunner, RunOutput},
    target::TargetReference,
};
pub(crate) use add_two_frame::{AddTwoFrameArgs, AddTwoFrameOperation};
pub(crate) use given_frame::{
    InputFrameArgs, InputFrameOperation, StateFrameArgs, StateFrameOperation,
};
pub(crate) use middle_frame::{MiddleFrameArgs, MiddleFrameOperation};
pub(crate) use shift_frame::{ShiftFrameArgs, ShiftFrameOperation};

/// The tests' numbers.
pub type Sc = ScalInF64;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Frame {
    pub data: Vector3<Sc>,
}

/// Where a body is: what the tests' state holds — its position alone.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Pose {
    pub position: Vector3<Sc>,
}

impl Pose {
    pub fn at(position: [f64; 3]) -> Self {
        Self {
            position: vector(position),
        }
    }
}

/// A vector of sharp scalars, from plain numbers.
pub(crate) fn vector<const N: usize>(xs: [f64; N]) -> Vector<Sc, N> {
    Vector::from_array(xs.map(Sc::from_f64))
}

/// Asserts that `actual` encloses the point `expected`: computed values are
/// enclosures, rounded outward, so equal to no literal.
#[track_caller]
pub(crate) fn assert_encloses<const N: usize>(actual: Vector<Sc, N>, expected: [f64; N]) {
    assert!(
        actual.could_be_equal(&vector(expected)),
        "{actual:?} does not enclose {expected:?}"
    );
}

/// A frame is read whole: a kind of its own.
impl Kind for Frame {
    const NAME: &'static str = "frame";
}

impl HasElements<Frame> for Frame {
    type Element<'a> = &'a Frame;

    fn element(&self, name: &str) -> GeopResult<&Frame> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!("a frame has no element `{name}`"))),
        }
    }
}

/// The frame target `name`, whole.
pub(crate) fn whole_frame(name: &str) -> ElementReference<Frame, Frame> {
    ElementReference::whole(TargetReference::new(name))
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

fn registry() -> OperationRegistry<Sc, Pose> {
    OperationRegistry::default()
        .with(AddTwoFrameOperation)
        .and_then(|o| o.with(MiddleFrameOperation))
        .and_then(|o| o.with(InputFrameOperation))
        .and_then(|o| o.with(StateFrameOperation))
        .and_then(|o| o.with(ShiftFrameOperation))
        .unwrap()
}

fn runner() -> ProgramRunner<Sc, Pose> {
    ProgramRunner::new(registry())
}

fn add_two_frame(id: &str, data1: [f64; 3], data2: [f64; 3]) -> ProgramStep {
    let args = AddTwoFrameArgs {
        data1: vector(data1),
        data2: vector(data2),
    };
    ProgramStep::new::<AddTwoFrameOperation>(id, &args).unwrap()
}

fn middle_frame(id: &str, frame1: &str, frame2: &str) -> ProgramStep {
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

fn shift_frame(id: &str, target: &str, x: f64) -> ProgramStep {
    let args = ShiftFrameArgs {
        targets: vec![target.to_string()],
        by: vector([x, 0.0, 0.0]),
    };
    ProgramStep::new::<ShiftFrameOperation>(id, &args).unwrap()
}

pub(crate) fn inputs(values: &[(&str, f64)]) -> BTreeMap<String, Sc> {
    values
        .iter()
        .map(|(name, value)| (name.to_string(), Sc::from_f64(*value)))
        .collect()
}

fn no_state() -> BTreeMap<String, Pose> {
    BTreeMap::new()
}

fn frame(runner: &ProgramRunner<Sc, Pose>, name: &str) -> Vector3<Sc> {
    let view = runner.part.current();
    view.retrieve_element(&whole_frame(name)).unwrap().data
}

/// The frame `name` as it was before the step at position `step` of the
/// program.
fn frame_before(runner: &ProgramRunner<Sc, Pose>, step: usize, name: &str) -> Vector3<Sc> {
    let view = runner
        .part
        .reader(ProgramRunner::<Sc, Pose>::position(step));
    view.retrieve_element(&whole_frame(name)).unwrap().data
}

/// The failed steps and what went wrong at the root.
fn errors(output: &RunOutput) -> Vec<(String, String)> {
    output
        .step_errors
        .iter()
        .map(|(step, e)| (step.clone(), e.root_message().to_string()))
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
                "target `b` failed in the step at position 2: input `y` is not given".into()
            ),
        ]
    );
    assert_eq!(take_built(), ["a@a", "b@b", "m@m"]);

    ok(runner.run(&program, &inputs(&[("x", 2.0), ("y", 3.0)]), &no_state()));
    assert_eq!(take_built(), ["b@b", "m@m"]);
    assert_encloses(frame(&runner, "m"), [2.5, 0.0, 0.0]);
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
        "target `step1_1` is not defined by the step at position 1 or before".to_string(),
    )];

    ok(runner.run(
        &Program::new(vec![add(), middle()]),
        &inputs(&[]),
        &no_state(),
    ));

    let reordered = runner.run(
        &Program::new(vec![middle(), add()]),
        &inputs(&[]),
        &no_state(),
    );
    assert_eq!(errors(&reordered), not_defined);

    let removed = runner.run(&Program::new(vec![middle()]), &inputs(&[]), &no_state());
    assert_eq!(errors(&removed), not_defined);
    let current = runner.part.current();
    assert!(current.retrieve_element(&whole_frame("step1_1")).is_err());
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
        let output = runner.run(&program, &inputs(&[("x", 1.0), ("y", 2.0)]), &no_state());
        assert_eq!(
            errors(&output),
            [(
                "second".into(),
                "target `a` is defined twice: by the steps at positions 1 and 2".into()
            )]
        );
        assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    }
    assert_eq!(take_built(), ["a@first"]);
}

/// A run's inputs are complete: one it no longer gives is gone.
#[test]
fn removed_input_is_gone() {
    let mut runner = runner();
    let program = Program::new(vec![input_frame("a", "a", "x")]);

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
    let program = Program::new(vec![
        ProgramStep::new::<StateFrameOperation>("s", &args).unwrap(),
    ]);
    let state = |x| BTreeMap::from([("body".to_string(), Pose::at([x, 0.0, 0.0]))]);

    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert_encloses(frame(&runner, "s"), [0.0; 3]);
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    ok(runner.run(&program, &inputs(&[]), &state(1.0)));
    assert_eq!(take_built(), ["s@s", "s@s"]);
    assert_encloses(frame(&runner, "s"), [1.0, 0.0, 0.0]);
}

#[test]
fn unknown_operation_fails_its_step_only() {
    let mut runner = ProgramRunner::new(
        OperationRegistry::default()
            .with(InputFrameOperation)
            .unwrap(),
    );
    let args = StateFrameArgs {
        state: "body".to_string(),
    };
    let program = Program::new(vec![
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
        Program::new(vec![
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
    assert_encloses(frame_before(&runner, 1, "a"), [1.0, 0.0, 0.0]);
    assert_encloses(frame_before(&runner, 2, "a"), [2.0, 0.0, 0.0]);
    assert_encloses(frame_before(&runner, 3, "a"), [2.0, 0.0, 0.0]);

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
    let with_s1 = Program::new(vec![
        input_frame("a", "a", "x"),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 10.0),
    ]);
    let without_s1 = Program::new(vec![
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
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        ProgramStep::new::<ShiftFrameOperation>("s", &twice).unwrap(),
    ]);
    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &no_state());
    assert_eq!(
        errors(&output),
        [(
            "s".into(),
            "target `a` is made twice by the step at position 2".into()
        )]
    );
    assert_encloses(frame(&runner, "a"), [2.0, 0.0, 0.0]);
}

/// A revision with no earlier version revises the default: a frame no step
/// defined is shifted from the origin.
#[test]
fn revision_without_an_earlier_version_starts_from_the_default() {
    let mut runner = runner();
    let program = Program::new(vec![shift_frame("s", "a", 1.0)]);
    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
}

/// A version rebuilt into an equal value counts as unchanged, so what read
/// it is reused.
#[test]
fn equal_rebuild_keeps_what_read_it() {
    let mut runner = runner();
    let program = |first, second| {
        Program::new(vec![
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
    let program = Program::new(vec![input_frame("a", "a", "x"), shift_frame("s", "a", 1.0)]);

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
    let output = runner.run(&Program::new(vec![step]), &inputs(&[]), &no_state());
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
    let program = Program::new(vec![
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

#[test]
fn target_is_made_by_a_step() {
    let mut part = Part::<Sc, Pose>::default();
    let reference = TargetReference::<Frame>::new("a");
    let e = part
        .define_target(&reference, &(), |_, _| {
            Ok(Frame {
                data: Vector::zero(),
            })
        })
        .unwrap_err();
    assert_eq!(e.root_message(), "target `a` is made outside of a step");
}

fn pose_at(x: f64) -> BTreeMap<String, Pose> {
    BTreeMap::from([("body".to_string(), Pose::at([x, 0.0, 0.0]))])
}

#[test]
fn operation_registered_twice_is_an_error() {
    let twice = OperationRegistry::<Sc, Pose>::default()
        .with(InputFrameOperation)
        .and_then(|o| o.with(InputFrameOperation));
    assert_eq!(
        twice.err().unwrap().root_message(),
        "operation `input_frame` is registered twice"
    );
}

/// A step's id is what tells its versions from another step's, so a
/// program cannot use one twice.
#[test]
fn step_id_used_twice_is_an_error() {
    let mut runner = runner();
    let program = Program::new(vec![
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
        reader: &crate::part::Reader<Sc, Pose>,
        by: &Vector3<Sc>,
    ) -> geop_core_math::geop_error::GeopResult<Frame> {
        let frame: Frame = reader.previous()?;
        Ok(Frame {
            data: frame.data.add(by),
        })
    }
    let mut part = Part::<Sc, Pose>::default();
    let reference = TargetReference::<Frame>::new("grown");
    part.begin_run(&BTreeMap::new(), &BTreeMap::new());
    part.begin_step();
    part.revise_target(&reference, &vector([1.0, 0.0, 0.0]), grow)
        .unwrap();
    part.begin_step();
    part.revise_target(&reference, &vector([2.0, 0.0, 0.0]), grow)
        .unwrap();
    assert_encloses(
        (part.current())
            .retrieve_element(&ElementReference::whole(reference.clone()))
            .unwrap()
            .data,
        [3.0, 0.0, 0.0],
    );

    fn fail(
        _: &crate::part::Reader<Sc, Pose>,
        _: &(),
    ) -> geop_core_math::geop_error::GeopResult<Frame> {
        Err("broken".into())
    }
    part.begin_step();
    assert!(part.revise_target(&reference, &(), fail).is_err());
    part.begin_step();
    let after = part.revise_target(&reference, &vector([1.0, 0.0, 0.0]), grow);
    assert_eq!(
        after.unwrap_err().root_message(),
        "target `grown` failed in the step at position 2: broken"
    );
}

/// A revision that started from the default, no step having made its
/// target before it, starts from the version of a step inserted before it
/// once one is: that no version was made is something it read.
#[test]
fn a_revision_from_the_default_follows_a_step_inserted_before_it() {
    let mut runner = runner();
    ok(runner.run(
        &Program::new(vec![shift_frame("s1", "a", 1.0)]),
        &inputs(&[]),
        &no_state(),
    ));
    assert_encloses(frame(&runner, "a"), [1.0, 0.0, 0.0]);
    ok(runner.run(
        &Program::new(vec![
            shift_frame("s0", "a", 5.0),
            shift_frame("s1", "a", 1.0),
        ]),
        &inputs(&[]),
        &no_state(),
    ));
    assert_encloses(frame(&runner, "a"), [6.0, 0.0, 0.0]);
}

/// What a run is given besides its inputs is defined by the given step,
/// before the program's own: its steps read it like any target, and a
/// failure of it is reported as the given step's.
#[test]
fn the_given_step_defines_what_a_run_is_given() {
    let mut runner = runner();
    runner.given = Some(std::rc::Rc::new(|part| {
        let given = TargetReference::<Frame>::new("given");
        part.define_target(&given, &(), |_, _| {
            Ok(Frame {
                data: vector([4.0, 0.0, 0.0]),
            })
        })
    }));
    let program = Program::new(vec![middle_frame("m", "given", "given")]);
    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert_encloses(frame(&runner, "m"), [4.0, 0.0, 0.0]);

    runner.given = Some(std::rc::Rc::new(|_| Err("no files".into())));
    let output = runner.run(&Program::new(vec![]), &inputs(&[]), &no_state());
    assert_eq!(errors(&output), [("given".into(), "no files".into())]);
}

/// A view lists the targets of a kind it sees, in the order their steps
/// made them: those made before it, not after.
#[test]
fn view_lists_targets_of_a_kind() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("b", "b", "x"),
        input_frame("a", "a", "x"),
        input_frame("c", "c", "x"),
    ]);
    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));
    fn names(view: crate::part::Reader<'_, Sc, Pose>) -> Vec<String> {
        let frames = view.targets_of::<Frame>();
        frames.iter().map(|f| f.name().to_string()).collect()
    }
    assert_eq!(names(runner.part.current()), ["b", "a", "c"]);
    let before_c = ProgramRunner::<Sc, Pose>::position(2);
    assert_eq!(names(runner.part.reader(before_c)), ["b", "a"]);
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
    let program = Program::new(vec![
        ProgramStep::new::<StateFrameOperation>("s", &follow).unwrap(),
        add_two_frame("pair", [0.0; 3], [2.0, 0.0, 0.0]),
        middle_frame("m", "s", "pair_2"),
        input_frame("a", "a", "x"),
    ]);
    let x = inputs(&[("x", 1.0)]);
    ok(runner.run(&program, &x, &pose_at(3.0)));
    take_built();

    ok(runner.run(&program, &x, &pose_at(1.0)));
    assert_eq!(take_built(), ["s@s", "m@m"]);
    assert_encloses(frame(&runner, "m"), [1.5, 0.0, 0.0]);

    ok(runner.run(&program, &x, &pose_at(1.0)));
    assert_eq!(take_built(), Vec::<String>::new());
}
