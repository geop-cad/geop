mod add_two_frame;
mod given_frame;
mod middle_frame;
mod pin_frame;
mod shift_frame;
mod solve;
mod step_editor;

use std::{cell::RefCell, collections::BTreeMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, ScalInF64, Scalar},
    solvers::{least_squares::Rows, system::Variable},
    vector::{Vector, Vector3},
};

use crate::{
    constraint::{Bound, Constraint},
    editing::{Event, Form, Value},
    operation::{OperationGroup, OperationInfo, OperationTier},
    operation_registry::OperationRegistry,
    part::{Part, Retrieve, State},
    program::{Program, ProgramStep},
    program_runner::{ProgramRunner, RunOutput},
    target::{Revisable, TargetReference},
};
pub(crate) use add_two_frame::{AddTwoFrameArgs, AddTwoFrameOperation};
pub(crate) use given_frame::{
    InputFrameArgs, InputFrameOperation, StateFrameArgs, StateFrameOperation,
};
pub(crate) use middle_frame::{MiddleFrameArgs, MiddleFrameOperation};
pub(crate) use pin_frame::{PinFrameArgs, PinFrameOperation};
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

/// The rows of the identity of size `n`: a derivative by a step that is
/// the change itself.
pub(crate) fn identity(n: usize) -> Vec<Vec<Sc>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| if i == j { Sc::ONE } else { Sc::ZERO })
                .collect()
        })
        .collect()
}

/// A position moves by a step added to it.
impl Variable for Pose {
    type Scalar = Sc;

    fn dimension(&self) -> usize {
        3
    }

    fn retract(&self, step: &[Sc]) -> GeopResult<Self> {
        let by = three(step)?;
        Ok(Self {
            position: self.position.add(&by).map(Scalar::sharpen),
        })
    }

    fn retract_over(&self, step: &[Sc]) -> GeopResult<Self> {
        let by = three(step)?;
        Ok(Self {
            position: self.position.add(&by),
        })
    }

    fn differential(&self, step: &[Sc]) -> GeopResult<Option<Vec<Vec<Sc>>>> {
        three(step)?;
        Ok(None)
    }

    fn stay(&self, at: &Self, _: Sc) -> GeopResult<Rows<Sc>> {
        let mut rows = Rows::default();
        for (k, row) in identity(3).into_iter().enumerate() {
            rows.push(self.position[k].sub(at.position[k]), row);
        }
        Ok(rows)
    }

    fn same(&self, other: &Self) -> bool {
        self == other
    }
}

impl State for Pose {}

/// A step of a position: three numbers.
fn three(step: &[Sc]) -> GeopResult<Vector3<Sc>> {
    let [x, y, z] = step else {
        return Err(GeopError::new(format!(
            "a step of a position is three numbers, not {}",
            step.len()
        )));
    };
    Ok(Vector3::from_array([*x, *y, *z]))
}

/// A vector of sharp scalars, from plain numbers.
pub(crate) fn vector<const N: usize>(xs: [f64; N]) -> Vector<Sc, N> {
    Vector::from_array(xs.map(Sc::from_f64))
}

/// Asserts that `actual` encloses the point `expected`: computed values are
/// enclosures, rounded outward, so equal to no literal.
#[track_caller]
fn assert_encloses<const N: usize>(actual: Vector<Sc, N>, expected: [f64; N]) {
    assert!(
        actual.could_be_equal(&vector(expected)),
        "{actual:?} does not enclose {expected:?}"
    );
}

/// Three numbers share nothing worth sharing: a revision copies them.
impl Revisable for Frame {
    fn to_revise(&self) -> Self {
        self.clone()
    }
}

thread_local! {
    /// The versions this test's generators built, as `target@step`, in order.
    static BUILT: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

/// The label a version is logged under when built. Generators capture
/// nothing, so it travels in their arguments.
fn label(target: &str, step: &str) -> String {
    format!("{target}@{step}")
}

fn record_built(label: &str) {
    BUILT.with(|built| built.borrow_mut().push(label.to_string()));
}

fn take_built() -> Vec<String> {
    BUILT.with(|built| built.take())
}

/// What a test operation is to an editor: only its name matters.
pub(crate) const fn info(kind: &'static str) -> OperationInfo {
    OperationInfo::new(kind, kind, "", OperationGroup::Solid, OperationTier::Menu)
}

fn registry() -> OperationRegistry<Sc, Pose> {
    OperationRegistry::default()
        .with(AddTwoFrameOperation)
        .and_then(|o| o.with(MiddleFrameOperation))
        .and_then(|o| o.with(InputFrameOperation))
        .and_then(|o| o.with(StateFrameOperation))
        .and_then(|o| o.with(ShiftFrameOperation))
        .and_then(|o| o.with(PinFrameOperation))
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
        frame1ref: frame1.to_string(),
        frame2ref: frame2.to_string(),
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
    let reference = TargetReference::<Frame>::new(name);
    runner
        .part
        .current()
        .retrieve_target(&reference)
        .unwrap()
        .data
}

/// The frame `name` as it was before the step at `step_index`.
fn frame_before(runner: &ProgramRunner<Sc, Pose>, step: &str, name: &str) -> Vector3<Sc> {
    let reference = TargetReference::<Frame>::new(name);
    let view = runner.part.view_before(step).unwrap();
    view.retrieve_target(&reference).unwrap().data
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
    assert_eq!(runner.part.built(), 3, "the part counts what it built");
    assert_encloses(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([0.0; 3]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_eq!(runner.part.built(), 3);
    assert_encloses(frame(&runner, "step2"), [0.0, 0.0, 0.5]);

    ok(runner.run(&program([1.0, 0.0, 0.0]), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), ["step1_1@step1", "step2@step2"]);
    assert_eq!(runner.part.built(), 5);
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
                "target `b` failed in step `b`: input `y` is not given".into()
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
        "target `step1_1` is not defined in step `step2` or before".to_string(),
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
    let reference = TargetReference::<Frame>::new("step1_1");
    assert!(runner.part.current().retrieve_target(&reference).is_err());
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
                "target `a` is defined twice: by step `first` and by step `second`".into()
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
        [("s".into(), "target `a` is made twice by step `s`".into())]
    );
    assert_encloses(frame(&runner, "a"), [2.0, 0.0, 0.0]);
}

#[test]
fn revision_needs_an_earlier_definition() {
    let mut runner = runner();
    let program = Program::new(vec![shift_frame("s", "a", 1.0)]);
    let output = runner.run(&program, &inputs(&[]), &no_state());
    assert_eq!(
        errors(&output),
        [(
            "s".into(),
            "target `a` is not defined before step `s`".into()
        )]
    );
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

/// A step is edited against the part as it was before it, whichever step
/// the editor looked at last.
#[test]
fn user_event_sees_the_part_before_its_step() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 5.0),
    ]);
    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));

    for (step, by) in [("s2", -2.0), ("s1", -1.0), ("s2", -2.0)] {
        let state = runner.initial_editing(&program, step).unwrap();
        let edited = runner
            .process_user_event(
                &program,
                step,
                &*state,
                Event::Field {
                    key: "origin".into(),
                    value: Value::Press,
                },
                None,
            )
            .unwrap();
        let args: ShiftFrameArgs = serde_json::from_value(edited.args).unwrap();
        assert_encloses(args.by, [by, 0.0, 0.0]);
    }
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

/// An operation constrains the state, and the run reports the constraint
/// with its derivatives. It is rebuilt when the state it read moves, as a
/// solver moves it, and only then.
#[test]
fn constraints_on_the_state_are_reported() {
    let mut runner = runner();
    let pin = PinFrameArgs {
        frame: "a".to_string(),
        state: "body".to_string(),
    };
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        ProgramStep::new::<PinFrameOperation>("pin", &pin).unwrap(),
    ]);
    let x = inputs(&[("x", 1.0)]);
    let identity = identity(3);
    let residuals = |runner: &ProgramRunner<_, Pose>| {
        let residuals = &runner.part.constraints()["pin"].residuals;
        Vector3::from_array([residuals[0], residuals[1], residuals[2]])
    };

    let output = runner.run(&program, &x, &pose_at(3.0));
    assert_eq!(errors(&output), vec![]);
    assert_eq!(
        runner.part.constraints().into_keys().collect::<Vec<_>>(),
        ["pin"]
    );
    assert_eq!(
        runner.part.constraints()["pin"].jacobian,
        BTreeMap::from([("body".to_string(), identity)])
    );
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);
    take_built();

    runner.run(&program, &x, &pose_at(3.0));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);

    runner.run(&program, &x, &pose_at(1.0));
    assert_eq!(take_built(), ["pin@pin"]);
    assert_encloses(residuals(&runner), [0.0, 0.0, 0.0]);
}

/// A constraint's derivatives must fit it: by states it read, a row per
/// residual.
#[test]
fn inconsistent_constraint_is_an_error() {
    let mut part = Part::<Sc, Pose>::default();
    part.begin_run(&BTreeMap::new(), &pose_at(0.0));
    part.begin_step("s").unwrap();
    let unread = part.define_constraint("unread", &(), |_, _| {
        Ok(Constraint::<Pose> {
            residuals: vec![Sc::ZERO],
            jacobian: BTreeMap::from([("body".to_string(), vec![vec![Sc::ZERO; 3]])]),
            bound: Bound::Zero,
        })
    });
    assert_eq!(
        unread.unwrap_err().root_message(),
        "constraint `unread` gives a derivative by state `body`, which it does not read"
    );

    let rows = part.define_constraint("rows", &(), |part, _| {
        part.retrieve_state("body");
        Ok(Constraint::<Pose> {
            residuals: vec![Sc::ZERO; 2],
            jacobian: BTreeMap::from([("body".to_string(), vec![vec![Sc::ZERO; 3]])]),
            bound: Bound::Zero,
        })
    });
    assert_eq!(
        rows.unwrap_err().root_message(),
        "constraint `rows` has 2 residuals but 1 rows of derivatives by state `body`"
    );
    assert!(part.constraints().is_empty());
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
    let reference = TargetReference::<Frame>::new("b");
    assert!(runner.part.current().retrieve_target(&reference).is_err());
}

/// The editor finds a step's view by its id, so a program edited since the
/// last run still edits each step against its own view, and a step the last
/// run did not have is named.
#[test]
fn edited_program_finds_each_step_by_id() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 5.0),
    ]);
    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));

    let edited = Program::new(vec![
        input_frame("a", "a", "x"),
        shift_frame("new", "a", 7.0),
        shift_frame("s1", "a", 1.0),
        shift_frame("s2", "a", 5.0),
    ]);
    let click = |step| {
        let state = runner.initial_editing(&edited, step)?;
        runner.process_user_event(
            &edited,
            step,
            &*state,
            Event::Field {
                key: "origin".into(),
                value: Value::Press,
            },
            None,
        )
    };
    let edited_s2 = click("s2").unwrap();
    let args: ShiftFrameArgs = serde_json::from_value(edited_s2.args).unwrap();
    assert_encloses(args.by, [-2.0, 0.0, 0.0]);
    assert_eq!(
        click("new").err().unwrap().root_message(),
        "step `new` is not in the last run"
    );
}

/// Only `define_constraint` makes a constraint: a target that merely holds
/// one has not had its derivatives checked, and is not reported.
#[test]
fn only_defined_constraints_are_reported() {
    let mut part = Part::<Sc, Pose>::default();
    part.begin_run(&BTreeMap::new(), &pose_at(0.0));
    part.begin_step("s").unwrap();
    let unchecked = TargetReference::<Constraint<Pose>>::new("unchecked");
    part.define_target(&unchecked, &(), |_, _| {
        Ok(Constraint {
            residuals: vec![Sc::ZERO],
            jacobian: BTreeMap::from([("body".to_string(), vec![])]),
            bound: Bound::Zero,
        })
    })
    .unwrap();
    part.define_constraint("checked", &(), |_, _| {
        Ok(Constraint {
            residuals: vec![Sc::ZERO],
            jacobian: BTreeMap::new(),
            bound: Bound::Zero,
        })
    })
    .unwrap();
    assert_eq!(part.constraints().keys().collect::<Vec<_>>(), [&"checked"]);
}

/// A new step starts from its operation's defaults, inserted before a step
/// of the last run or after its last; its form is the operation's.
#[test]
fn new_step_starts_from_its_operation() {
    let mut runner = runner();
    let program = Program::new(vec![input_frame("a", "a", "x")]);
    ok(runner.run(&program, &inputs(&[("x", 1.0)]), &no_state()));

    let step = runner.new_step("shift_frame", "s", None).unwrap();
    assert_eq!(step.operation, "shift_frame");
    let args: ShiftFrameArgs = serde_json::from_value(step.args.clone()).unwrap();
    assert_eq!(args, ShiftFrameArgs::default());
    assert!(runner.new_step("shift_frame", "s", Some("a")).is_ok());
    assert_eq!(
        runner
            .new_step("nope", "s", None)
            .err()
            .unwrap()
            .root_message(),
        "unknown operation `nope`"
    );

    let editing = runner.initial_editing(&program, "a").unwrap();
    let form = runner.presentation(&program, "a", &*editing).unwrap();
    assert_eq!(form, Form::default());
}

/// A value built up from nothing starts from its default at the first step
/// that builds it, is revised by the later ones, and a failed version is no
/// fresh start for the step after it.
#[test]
fn revise_or_start_begins_from_the_default() {
    fn grow(
        frame: Frame,
        _: &mut crate::part::Reader<Sc, Pose>,
        by: &Vector3<Sc>,
    ) -> geop_core_math::geop_error::GeopResult<Frame> {
        Ok(Frame {
            data: frame.data.add(by),
        })
    }
    let mut part = Part::<Sc, Pose>::default();
    let reference = TargetReference::<Frame>::new("grown");
    part.begin_run(&BTreeMap::new(), &BTreeMap::new());
    part.begin_step("first").unwrap();
    part.revise_or_start(&reference, &vector([1.0, 0.0, 0.0]), grow)
        .unwrap();
    part.begin_step("second").unwrap();
    part.revise_or_start(&reference, &vector([2.0, 0.0, 0.0]), grow)
        .unwrap();
    assert_encloses(
        part.current().retrieve_target(&reference).unwrap().data,
        [3.0, 0.0, 0.0],
    );

    fn fail(
        _: Frame,
        _: &mut crate::part::Reader<Sc, Pose>,
        _: &(),
    ) -> geop_core_math::geop_error::GeopResult<Frame> {
        Err("broken".into())
    }
    part.begin_step("broken").unwrap();
    assert!(part.revise_or_start(&reference, &(), fail).is_err());
    part.begin_step("after").unwrap();
    let after = part.revise_or_start(&reference, &vector([1.0, 0.0, 0.0]), grow);
    assert_eq!(
        after.unwrap_err().root_message(),
        "target `grown` failed in step `broken`: broken"
    );
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
    fn names<'a>(view: crate::part::View<'a, Sc, Pose>) -> Vec<&'a str> {
        let frames = view.targets_of::<Frame>();
        frames.into_iter().map(|(name, _)| name).collect()
    }
    assert_eq!(names(runner.part.current()), ["b", "a", "c"]);
    assert_eq!(names(runner.part.view_before("c").unwrap()), ["b", "a"]);
    assert!(runner.part.current().targets_of::<Pose>().is_empty());
}

/// Between runs, a solver brings constraints up to date with a state of its
/// own: what read the state is made again — what the constraint reads
/// first — and nothing else, and the next run finds it all current.
#[test]
fn update_remakes_only_what_read_the_state() {
    let mut runner = runner();
    let follow = StateFrameArgs {
        state: "body".to_string(),
    };
    let pin = PinFrameArgs {
        frame: "a".to_string(),
        state: "body".to_string(),
    };
    let program = Program::new(vec![
        ProgramStep::new::<StateFrameOperation>("s", &follow).unwrap(),
        add_two_frame("pair", [0.0; 3], [2.0, 0.0, 0.0]),
        middle_frame("m", "s", "pair_2"),
        input_frame("a", "a", "x"),
        ProgramStep::new::<PinFrameOperation>("pin", &pin).unwrap(),
    ]);
    let x = inputs(&[("x", 1.0)]);
    ok(runner.run(&program, &x, &pose_at(3.0)));
    take_built();

    runner.part.update(&pose_at(1.0), &["pin", "m"]);
    assert_eq!(take_built(), ["pin@pin", "s@s", "m@m"]);
    let pin = &runner.part.constraints()["pin"];
    assert_encloses(
        Vector3::from_array([pin.residuals[0], pin.residuals[1], pin.residuals[2]]),
        [0.0; 3],
    );
    assert_encloses(frame(&runner, "m"), [1.5, 0.0, 0.0]);

    // The run with that state: everything is current already.
    ok(runner.run(&program, &x, &pose_at(1.0)));
    assert_eq!(take_built(), Vec::<String>::new());
}
