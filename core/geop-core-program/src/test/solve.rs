//! Solving the state a program's constraints are on: pins, pulls, limits,
//! groups, conflicts and freedom, on positions.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::GeopResult,
    scalars::{Ring, Scalar},
};
use serde::{Deserialize, Serialize};

use crate::{
    constraint::{Bound, Constraint},
    editing::{Event, Form},
    operation::{Context, Operation, OperationInfo},
    operation_registry::OperationRegistry,
    part::{Part, Retrieve, View},
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    solve::{Mobility, Preference, Problem, conflicting, freedom, solve},
    test::{
        AddTwoFrameOperation, InputFrameOperation, MiddleFrameOperation, PinFrameArgs,
        PinFrameOperation, Pose, Sc, ShiftFrameOperation, StateFrameOperation, identity, info,
        input_frame, inputs, vector,
    },
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct LimitArgs {
    state: String,
    most: f64,
}

/// Keeps the `x` of the position `state` at most `most`: a limit.
struct LimitOperation;

impl Operation for LimitOperation {
    const INFO: OperationInfo = info("limit");
    type Args = LimitArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &LimitArgs) -> GeopResult<()> {
        part.define_constraint(step, args, |part, args| {
            let x = part.retrieve_state(&args.state).position[0];
            Ok(Constraint {
                residuals: vec![x.sub(Sc::from_f64(args.most))],
                jacobian: BTreeMap::from([(
                    args.state.clone(),
                    vec![vec![Sc::ONE, Sc::ZERO, Sc::ZERO]],
                )]),
                bound: Bound::AtMost,
            })
        })
    }

    fn new_args(_: &Context<Sc, Pose>) -> LimitArgs {
        LimitArgs::default()
    }

    fn presentation(_: &Context<Sc, Pose>, _: &LimitArgs, _: &()) -> Form<Sc> {
        Form::default()
    }

    fn process_user_event(
        _: &Context<Sc, Pose>,
        args: &LimitArgs,
        _: &(),
        _: Event<Sc>,
    ) -> (LimitArgs, ()) {
        (args.clone(), ())
    }
}

/// The position `state` pulled towards `to`.
struct Toward {
    state: String,
    to: [f64; 3],
}

impl Preference<Sc, Pose> for Toward {
    fn evaluate(&self, view: View<'_, Sc, Pose>) -> GeopResult<Constraint<Pose>> {
        let at = view.retrieve_state(&self.state).position;
        Ok(Constraint {
            residuals: at.sub(&vector(self.to)).to_array().to_vec(),
            jacobian: BTreeMap::from([(self.state.clone(), identity(3))]),
            bound: Bound::Zero,
        })
    }
}

fn registry() -> OperationRegistry<Sc, Pose> {
    (OperationRegistry::default().with(AddTwoFrameOperation))
        .and_then(|o| o.with(MiddleFrameOperation))
        .and_then(|o| o.with(InputFrameOperation))
        .and_then(|o| o.with(StateFrameOperation))
        .and_then(|o| o.with(ShiftFrameOperation))
        .and_then(|o| o.with(PinFrameOperation))
        .and_then(|o| o.with(LimitOperation))
        .unwrap()
}

fn pin(id: &str, frame: &str, state: &str) -> ProgramStep {
    let args = PinFrameArgs {
        frame: frame.to_string(),
        state: state.to_string(),
    };
    ProgramStep::new::<PinFrameOperation>(id, &args).unwrap()
}

fn at(points: &[(&str, [f64; 3])]) -> BTreeMap<String, Pose> {
    (points.iter())
        .map(|(name, p)| (name.to_string(), Pose::at(*p)))
        .collect()
}

fn free(names: &[&str]) -> BTreeMap<String, Mobility> {
    (names.iter())
        .map(|n| (n.to_string(), Mobility::Held))
        .collect()
}

fn problem<'p>(
    names: &[&str],
    preferences: Vec<&'p dyn Preference<Sc, Pose>>,
) -> Problem<'p, Sc, Pose> {
    Problem {
        free: free(names),
        preferences,
        scale: Sc::ONE,
    }
}

/// Where `state` puts `name`, as plain numbers.
fn where_is(state: &BTreeMap<String, Pose>, name: &str) -> [f64; 3] {
    state[name].position.to_array().map(|c| c.to_f64())
}

#[track_caller]
fn assert_near(a: [f64; 3], b: [f64; 3]) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-8),
        "{a:?} is not {b:?}"
    );
}

/// A run's constraints, solved: the body pinned to a frame goes to it, and
/// nothing else moves — and the solve encloses where it truly is.
#[test]
fn a_pinned_body_goes_to_its_frame() {
    let mut runner = ProgramRunner::new(registry());
    let program = Program::new(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let x = inputs(&[("x", 1.0)]);
    let state = at(&[("body", [3.0, 2.0, 0.0])]);
    runner.run(&program, &x, &state);
    let solved = solve(&mut runner.part, &state, &problem(&["body"], vec![])).unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    assert_eq!(solved.report.moved, ["body"]);
    assert_near(where_is(&solved.state, "body"), [1.0, 0.0, 0.0]);
    assert!(
        solved.enclosure.unproven.is_empty(),
        "{:?}",
        solved.enclosure
    );
    let enclosed = solved.enclosure.state["body"].position;
    for (k, exact) in [1.0, 0.0, 0.0].into_iter().enumerate() {
        assert!(
            Sc::from_f64(exact).is_subset_of(enclosed[k]),
            "{enclosed:?}"
        );
    }
    // The part is left at the state found.
    assert!(runner.part.constraints()["pin"].holds(Sc::from_f64(1e-9)));
}

/// A pull moves only as far as the constraints let it: a pinned body
/// stays on its frame, a free one goes where it is pulled — and where the
/// constraints hold already and nothing pulls, nothing is solved.
#[test]
fn a_pull_moves_what_the_constraints_leave_free() {
    let mut runner = ProgramRunner::new(registry());
    let program = Program::new(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let x = inputs(&[("x", 1.0)]);
    let state = at(&[("body", [1.0, 0.0, 0.0]), ("loose", [0.0; 3])]);
    runner.run(&program, &x, &state);
    let (body, loose) = (
        Toward {
            state: "body".into(),
            to: [5.0, 0.0, 0.0],
        },
        Toward {
            state: "loose".into(),
            to: [0.0, 2.0, 0.0],
        },
    );
    let both = problem(&["body", "loose"], vec![&body, &loose]);
    let solved = solve(&mut runner.part, &state, &both).unwrap();
    assert!(solved.report.converged);
    assert_near(where_is(&solved.state, "body"), [1.0, 0.0, 0.0]);
    assert_near(where_is(&solved.state, "loose"), [0.0, 2.0, 0.0]);
    assert_eq!(solved.report.moved, ["loose"]);

    let still = solve(
        &mut runner.part,
        &state,
        &problem(&["body", "loose"], vec![]),
    )
    .unwrap();
    assert!(still.report.moved.is_empty());
    assert_eq!(still.report.iterations, 0, "nothing to solve");
}

/// A limit stops a pull at it, and the report says so.
#[test]
fn a_limit_stops_a_pull() {
    let mut runner = ProgramRunner::new(registry());
    let limit = LimitArgs {
        state: "body".into(),
        most: 2.0,
    };
    let program = Program::new(vec![
        ProgramStep::new::<LimitOperation>("limit", &limit).unwrap(),
    ]);
    let state = at(&[("body", [0.0; 3])]);
    runner.run(&program, &inputs(&[]), &state);
    let pull = Toward {
        state: "body".into(),
        to: [5.0, 1.0, 0.0],
    };
    let solved = solve(&mut runner.part, &state, &problem(&["body"], vec![&pull])).unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    assert_eq!(solved.report.at_limit, ["limit"]);
    assert_near(where_is(&solved.state, "body"), [2.0, 1.0, 0.0]);
}

/// Two pins of one body to different frames cannot both hold: the solve
/// says which fail, the conflict names both, and a body pinned is held
/// fast where a body nothing holds is free in every way.
#[test]
fn conflicts_and_freedom() {
    let mut runner = ProgramRunner::new(registry());
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        input_frame("b", "b", "y"),
        pin("one", "a", "body"),
        pin("two", "b", "body"),
    ]);
    let values = inputs(&[("x", 1.0), ("y", 2.0)]);
    let state = at(&[("body", [0.0; 3]), ("loose", [0.0; 3])]);
    runner.run(&program, &values, &state);
    let both = problem(&["body", "loose"], vec![]);
    let solved = solve(&mut runner.part, &state, &both).unwrap();
    assert!(!solved.report.converged);
    assert_eq!(solved.report.failed, ["one", "two"]);
    // Nothing to enclose, and the enclosure says why, by name.
    let [(group, why)] = &solved.enclosure.unproven[..] else {
        panic!("{:?}", solved.enclosure);
    };
    assert_eq!(group, &["body"]);
    assert!(why.contains("one") && why.contains("two"), "{why}");
    assert_eq!(
        conflicting(&mut runner.part, &state, &both).unwrap(),
        ["one", "two"]
    );

    let single = Program::new(vec![input_frame("a", "a", "x"), pin("one", "a", "body")]);
    runner.run(&single, &values, &state);
    let free = freedom(&mut runner.part, &state, &both).unwrap();
    assert_eq!(free.states["body"], 0);
    assert_eq!(free.states["loose"], 3);
    assert_eq!(free.total, 3);
}
