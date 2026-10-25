//! Fixtures — a state of poses, frames defined from inputs, an
//! operation pinning a position to a frame — and the tests of constraints
//! and unknowns as targets of a part.

mod instances;
mod solve;

use std::{cell::RefCell, collections::BTreeMap};

use geop_core_math::{
    dual::Dual,
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
    solvers::system::Mobility,
    vector::{Vector, Vector3},
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    operation::Operation,
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    program::{Program, ProgramStep},
    reader::Reader,
    runner::{RunOutput, Runner},
    target::{Target, TargetReference},
    values::{Setting, Settings, StateValue, StateValues, pose_in},
};
use serde::{Deserialize, Serialize};

use crate::{
    constraint::{Bound, Constraint, Constraints, define_constraints},
    solve::Run,
    unknown::{Unknown, Unknowns, declare_unknowns},
};

/// The pose a body's state holds: unturned at the origin before anything
/// set it.
fn pose_of(value: &StateValue) -> Pose {
    pose_in(value).unwrap_or_else(Pose::identity)
}

/// The state of a body unturned at `position`.
fn body_at(position: [f64; 3]) -> StateValue {
    StateValue::Pose(Pose::identity().with_position(vector(position)))
}

/// Where the body whose state is `value` has its origin, and the derivatives
/// of that by the body's step: one row of six per coordinate.
fn position(value: &StateValue) -> GeopResult<(Vector3, Vec<Vec<Scalar>>)> {
    type D = Dual<6>;
    let step = Pose::exp_with_derivatives(&Vector3::zero(), &Vector3::zero())?;
    let moved = pose_of(value).map(D::cst).compose(&step).position();
    let at = Vector3::from_array(moved.to_array().map(|c| c.v));
    Ok((at, moved.to_array().iter().map(|c| c.d.to_vec()).collect()))
}

/// Asserts that `rows` are the derivatives of an unturned body's origin by
/// its step: moved along by the step's translation, not by its turn.
#[track_caller]
fn assert_translation(rows: &[Vec<Scalar>]) {
    let expected: Vec<Vec<f64>> = (0..3)
        .map(|i| (0..6).map(|j| if j == i + 3 { 1.0 } else { 0.0 }).collect())
        .collect();
    let encloses = rows.len() == 3
        && (rows.iter().zip(&expected)).all(|(row, expected)| {
            row.len() == 6
                && (row.iter().zip(expected)).all(|(d, e)| d.could_be_equal(Scalar::from_f64(*e)))
        });
    assert!(
        encloses,
        "{rows:?} are not the derivatives of a translation"
    );
}

/// A vector of sharp scalars, from plain numbers.
fn vector<const N: usize>(xs: [f64; N]) -> Vector<N> {
    Vector::from_array(xs.map(Scalar::from_f64))
}

/// Asserts that `actual` encloses the point `expected`: computed values are
/// enclosures, rounded outward, so equal to no literal.
#[track_caller]
fn assert_encloses<const N: usize>(actual: Vector<N>, expected: [f64; N]) {
    assert!(
        actual.could_be_equal(&vector(expected)),
        "{actual:?} does not enclose {expected:?}"
    );
}

/// A point a step defines, which a pin holds a body to.
#[derive(Debug, Clone, Default, PartialEq)]
struct Frame {
    data: Vector3,
}

/// Defined by one step.
impl Target for Frame {}

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
fn whole_frame(name: &str) -> ElementReference<Frame, Frame> {
    ElementReference::whole(TargetReference::new(name))
}

thread_local! {
    /// The steps whose generators ran in this test, in order.
    static BUILT: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn record_built(step: &str) {
    BUILT.with(|built| built.borrow_mut().push(step.to_string()));
}

fn take_built() -> Vec<String> {
    BUILT.with(|built| built.take())
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct InputFrameArgs {
    target: String,
    input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const KIND: &'static str = "input_frame";
    type Args = InputFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let reference = TargetReference::<Frame>::new(&args.target);
        ctx.produce(&reference, args, |reader, args| {
            let x = match reader.retrieve_input(&args.input) {
                Some(Setting::Number(x)) => Scalar::from_f64(x),
                _ => {
                    return Err(GeopError::new(format!(
                        "input `{}` is not given",
                        args.input
                    )));
                }
            };
            Ok(Frame {
                data: Vector3::from_array([x, Scalar::ZERO, Scalar::ZERO]),
            })
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct PinFrameArgs {
    frame: String,
    state: String,
}

/// Constrains the origin of the pose `state` to the frame `frame`: a constraint
/// named after its step.
struct PinFrameOperation;

impl Operation for PinFrameOperation {
    const KIND: &'static str = "pin_frame";
    type Args = PinFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (step.to_string(), args.clone());
        define_constraints(ctx, &args, |reader, (step, args)| {
            record_built(step);
            let frame = reader.retrieve_element(&whole_frame(&args.frame))?;
            let (at, rows) = position(&reader.retrieve_state(&args.state))?;
            let pin = Constraint {
                residuals: at.sub(&frame.data).to_array().to_vec(),
                jacobian: BTreeMap::from([(args.state.clone(), rows)]),
                bound: Bound::Zero,
            };
            Ok(BTreeMap::from([(step.clone(), pin)]))
        })
    }
}

fn input_frame(id: &str, target: &str, input: &str) -> ProgramStep {
    let args = InputFrameArgs {
        target: target.to_string(),
        input: input.to_string(),
    };
    ProgramStep::new::<InputFrameOperation>(id, &args).unwrap()
}

fn pin(id: &str, frame: &str, state: &str) -> ProgramStep {
    let args = PinFrameArgs {
        frame: frame.to_string(),
        state: state.to_string(),
    };
    ProgramStep::new::<PinFrameOperation>(id, &args).unwrap()
}

fn inputs(values: &[(&str, f64)]) -> Settings {
    (values.iter())
        .map(|(name, value)| (name.to_string(), Setting::Number(*value)))
        .collect()
}

fn pose_at(x: f64) -> StateValues {
    StateValues::from([("body".to_string(), body_at([x, 0.0, 0.0]))])
}

/// The failed steps and what went wrong at the root.
fn errors(output: &RunOutput) -> Vec<(String, String)> {
    (output.step_errors.iter())
        .map(|(step, e)| (step.clone(), e.root_message().to_string()))
        .collect()
}

/// The operations of the fixtures.
fn registry() -> OperationRegistry {
    (OperationRegistry::default().with(InputFrameOperation))
        .and_then(|o| o.with(PinFrameOperation))
        .and_then(|o| o.with(instances::PlaceOperation))
        .and_then(|o| o.with(instances::FreeOperation))
        .unwrap()
}

/// A runner of the tests' operations, and the part it runs on.
struct Running {
    runner: Runner,
    part: Part,
}

impl Running {
    fn of(registry: OperationRegistry) -> Self {
        let mut runner = Runner::new(registry);
        runner.workspace = Some(std::rc::Rc::new(instances::Files));
        Self {
            runner,
            part: Part::default(),
        }
    }

    /// Runs `program` with `inputs`, at `state`.
    fn run(&mut self, program: &Program, inputs: &Settings, state: &StateValues) -> RunOutput {
        let program = Program {
            steps: program.steps.clone(),
            state: state.clone(),
        };
        (self.runner).run_with(&mut self.part, &program, inputs, None)
    }

    /// The program `program` with `inputs`, for a solve to run.
    fn rerun<'r>(&'r mut self, program: &'r Program, inputs: &'r Settings) -> Run<'r> {
        Run {
            runner: &self.runner,
            part: &mut self.part,
            program: program.clone(),
            inputs: inputs.clone(),
        }
    }
}

fn runner() -> Running {
    Running::of(registry())
}

/// The constraints of the part as it is now.
fn constraints_now(part: &Part) -> GeopResult<Constraints> {
    Constraints::of(&part.current())
}

/// An operation constrains the state, and the run reports the constraint
/// with its derivatives. It is made again when the state it read moves, as
/// a solver moves it, and only then. A part no step constrains has no
/// constraints.
#[test]
fn constraints_on_the_state_are_reported() {
    let mut runner = runner();
    let x = inputs(&[("x", 1.0)]);
    let frame_only = Program::from_steps(vec![input_frame("a", "a", "x")]);
    assert_eq!(errors(&runner.run(&frame_only, &x, &pose_at(3.0))), []);
    assert!(constraints_now(&runner.part).unwrap().is_empty());

    let program = Program::from_steps(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let residuals = |runner: &Running| {
        let constraints = constraints_now(&runner.part).unwrap();
        let residuals = &constraints.get("pin").unwrap().residuals;
        Vector3::from_array([residuals[0], residuals[1], residuals[2]])
    };

    assert_eq!(errors(&runner.run(&program, &x, &pose_at(3.0))), []);
    let constraints = constraints_now(&runner.part).unwrap();
    assert_eq!(
        constraints.iter().map(|(name, _)| name).collect::<Vec<_>>(),
        ["pin"]
    );
    let jacobian = &constraints.get("pin").unwrap().jacobian;
    assert_eq!(jacobian.keys().collect::<Vec<_>>(), ["body"]);
    assert_translation(&jacobian["body"]);
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);
    assert_eq!(take_built(), ["pin"]);

    runner.run(&program, &x, &pose_at(3.0));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);

    runner.run(&program, &x, &pose_at(1.0));
    assert_eq!(take_built(), ["pin"]);
    assert_encloses(residuals(&runner), [0.0, 0.0, 0.0]);
}

/// A constraint of `residuals` residuals and one row of derivatives by
/// `body`.
fn by_body(residuals: usize) -> Constraint {
    Constraint {
        residuals: vec![Scalar::ZERO; residuals],
        jacobian: BTreeMap::from([("body".to_string(), vec![vec![Scalar::ZERO; 6]])]),
        bound: Bound::Zero,
    }
}

/// A constraint's derivatives must fit it: by states it read, a row per
/// residual. A step that defines one that does not fails, and so do the
/// part's constraints.
#[test]
fn inconsistent_constraint_is_an_error() {
    let runner = Runner::new(registry());
    let mut part = Part::given(Settings::new(), pose_at(0.0));
    let unread = runner.run_step(&mut part, "s1", |ctx| {
        define_constraints(ctx, &(), |_, _| {
            Ok(BTreeMap::from([("unread".to_string(), by_body(1))]))
        })
    });
    assert_eq!(
        unread.unwrap_err().root_message(),
        "constraint `unread` gives a derivative by state `body`, which it does not read"
    );

    let rows = runner.run_step(&mut part, "s2", |ctx| {
        define_constraints(ctx, &(), |reader, _| {
            reader.retrieve_state("body");
            Ok(BTreeMap::from([("rows".to_string(), by_body(2))]))
        })
    });
    assert_eq!(
        rows.unwrap_err().root_message(),
        "constraint `rows` has 2 residuals but 1 rows of derivatives by state `body`"
    );
    assert!(constraints_now(&part).is_err());
}

/// Each step's constraints are a target of its own: one that fails leaves
/// the later steps' alone, and a solve reading them all finds the failure,
/// not a part with fewer constraints.
#[test]
fn a_failed_constraint_step_fails_the_constraints() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        pin("broken", "missing", "body"),
        pin("pin", "a", "body"),
    ]);
    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &pose_at(0.0));
    let errors = errors(&output);
    let [(broken, why_broken)] = &errors[..] else {
        panic!("{errors:?}");
    };
    assert_eq!(broken, "broken");
    assert!(
        why_broken.contains("no step before here produces target `missing`"),
        "{why_broken}"
    );
    let error = constraints_now(&runner.part).unwrap_err();
    assert!(
        error.root_message().contains("`constraints:broken` failed"),
        "{error}"
    );
}

/// A constraint's name is the part's to resolve by: a second step defining
/// it makes the part's constraints an error naming both.
#[test]
fn constraint_defined_twice_is_an_error() {
    fn define(_: &Reader, _: &()) -> GeopResult<BTreeMap<String, Constraint>> {
        let free = Constraint {
            residuals: vec![Scalar::ZERO],
            jacobian: BTreeMap::new(),
            bound: Bound::Zero,
        };
        Ok(BTreeMap::from([("pin".to_string(), free)]))
    }
    let runner = Runner::new(registry());
    let mut part = Part::default();
    (runner.run_step(&mut part, "one", |ctx| define_constraints(ctx, &(), define))).unwrap();
    (runner.run_step(&mut part, "two", |ctx| define_constraints(ctx, &(), define))).unwrap();
    assert_eq!(
        constraints_now(&part).unwrap_err().root_message(),
        "constraint `pin` is defined twice: by step `one` and by step `two`"
    );
}

/// The unknowns are one target by the state they are: of two declarations
/// of a state, one that holds it fixed wins, and otherwise the first.
#[test]
fn unknowns_merge_by_state() {
    fn declare(_: &Reader, declared: &[(String, Mobility); 2]) -> GeopResult<Vec<Unknown>> {
        Ok((declared.iter())
            .map(|(state, mobility)| Unknown {
                state: state.clone(),
                mobility: *mobility,
                seed: None,
                unless: None,
            })
            .collect())
    }
    let declared = |state: &str, mobility| (state.to_string(), mobility);
    let mobility = |part: &Part, state: &str| {
        let unknowns = Unknowns::of(&part.current()).unwrap();
        unknowns.get(state).map(|unknown| unknown.mobility)
    };

    let runner = Runner::new(registry());
    let mut part = Part::default();
    assert!(
        Unknowns::of(&part.current())
            .unwrap()
            .iter()
            .next()
            .is_none()
    );
    let first = [
        declared("body", Mobility::Held),
        declared("loose", Mobility::Held),
    ];
    (runner.run_step(&mut part, "first", |ctx| {
        declare_unknowns(ctx, &first, declare)
    }))
    .unwrap();
    let second = [
        declared("body", Mobility::Fixed),
        declared("loose", Mobility::Follows),
    ];
    (runner.run_step(&mut part, "second", |ctx| {
        declare_unknowns(ctx, &second, declare)
    }))
    .unwrap();

    assert_eq!(mobility(&part, "body"), Some(Mobility::Fixed));
    assert_eq!(mobility(&part, "loose"), Some(Mobility::Held));
    assert_eq!(mobility(&part, "other"), None);
}

/// Defines the target `<step>` as the frame at the origin of the pose
/// `state`: geometry the state drives.
#[allow(clippy::ptr_arg, reason = "a make takes its arguments as produced")]
fn follow(reader: &Reader, state: &String) -> GeopResult<Frame> {
    Ok(Frame {
        data: pose_of(&reader.retrieve_state(state)).position(),
    })
}

/// A constraint reading a target that read the state itself would close a
/// loop the solve's derivatives do not see: it is refused, naming the
/// constraint, the target and the state.
#[test]
fn a_constraint_on_geometry_the_state_drives_is_refused() {
    let runner = Runner::new(registry());
    let mut part = Part::given(Settings::new(), pose_at(1.0));
    let followed = TargetReference::<Frame>::new("followed");
    (runner.run_step(&mut part, "followed", |ctx| {
        ctx.produce(&followed, &"body".to_string(), follow)
    }))
    .unwrap();
    let pinned = runner.run_step(&mut part, "pin", |ctx| {
        let args = PinFrameArgs {
            frame: "followed".into(),
            state: "other".into(),
        };
        PinFrameOperation::run(ctx, &args)
    });
    assert_eq!(
        pinned.unwrap_err().root_message(),
        "constraint `pin` reads target `/:followed`, which itself reads the state `body`: a solve \
         could not see how it moves with the state — a constraint may read the state only itself"
    );
}
