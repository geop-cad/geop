//! Solving the state a program's constraints are on: pins, pulls, limits,
//! conflicts and freedom, on poses.

use std::collections::BTreeMap;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    operation::Operation,
    part::StepContext,
    program::{Program, ProgramStep},
    reader::Reader,
    values::StateValues,
};
use serde::{Deserialize, Serialize};

use super::{
    Running, body_at, constraints_now, input_frame, inputs, pin, pose_of, position, registry,
    vector,
};
use crate::{
    constraint::{Bound, Constraint, define_constraints},
    solve::{Mobility, Preference, Problem, conflicting, freedom, solve},
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct LimitArgs {
    state: String,
    most: f64,
}

/// Keeps the `x` of the origin of the pose `state` at most `most`: a limit, named
/// after its step.
struct LimitOperation;

impl Operation for LimitOperation {
    const KIND: &'static str = "limit";
    type Args = LimitArgs;

    fn run(ctx: &mut StepContext, args: &LimitArgs) -> GeopResult<()> {
        let step = ctx.step();
        let args = (step.to_string(), args.clone());
        define_constraints(ctx, &args, |reader, (step, args)| {
            let (at, rows) = position(&reader.retrieve_state(&args.state))?;
            let limit = Constraint {
                residuals: vec![at[0].sub(Scalar::from_f64(args.most))],
                jacobian: BTreeMap::from([(args.state.clone(), rows[..1].to_vec())]),
                bound: Bound::AtMost,
            };
            Ok(BTreeMap::from([(step.clone(), limit)]))
        })
    }
}

/// The origin of the pose `state` pulled towards `to`.
struct Toward {
    state: String,
    to: [f64; 3],
}

impl Preference for Toward {
    fn evaluate(&self, reader: &Reader<'_>) -> GeopResult<Constraint> {
        let (at, rows) = position(&reader.retrieve_state(&self.state))?;
        Ok(Constraint {
            residuals: at.sub(&vector(self.to)).to_array().to_vec(),
            jacobian: BTreeMap::from([(self.state.clone(), rows)]),
            bound: Bound::Zero,
        })
    }
}

fn runner() -> Running {
    Running::of(registry().with(LimitOperation).unwrap())
}

fn at(points: &[(&str, [f64; 3])]) -> StateValues {
    (points.iter())
        .map(|(name, p)| (name.to_string(), body_at(*p)))
        .collect()
}

fn problem<'p>(names: &[&str], preferences: Vec<&'p dyn Preference>) -> Problem<'p> {
    Problem {
        free: (names.iter())
            .map(|n| (n.to_string(), Mobility::Held))
            .collect(),
        preferences,
        scale: Scalar::ONE,
    }
}

/// Where `state` puts `name`, as plain numbers.
fn where_is(state: &StateValues, name: &str) -> [f64; 3] {
    pose_of(&state[name])
        .position()
        .to_array()
        .map(|c| c.to_f64())
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
    let mut runner = runner();
    let program = Program::from_steps(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let x = inputs(&[("x", 1.0)]);
    let state = at(&[("body", [3.0, 2.0, 0.0])]);
    let solved = solve(
        runner.rerun(&program, &x, &state),
        &state,
        &problem(&["body"], vec![]),
        true,
    )
    .unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    assert_eq!(solved.report.moved, ["body"]);
    assert_near(where_is(&solved.state, "body"), [1.0, 0.0, 0.0]);
    assert!(
        solved.enclosure.as_ref().unwrap().unproven.is_empty(),
        "{:?}",
        solved.enclosure
    );
    let enclosed = pose_of(&solved.enclosure.as_ref().unwrap().state["body"]).position();
    for (k, exact) in [1.0, 0.0, 0.0].into_iter().enumerate() {
        assert!(
            Scalar::from_f64(exact).is_subset_of(enclosed[k]),
            "{enclosed:?}"
        );
    }
    // Run at the state found, the pin holds.
    runner.run(&program, &x, &solved.state);
    let constraints = constraints_now(&runner.part).unwrap();
    assert!(
        constraints
            .get("pin")
            .unwrap()
            .holds(Scalar::from_f64(1e-9))
    );
}

/// A pull moves only as far as the constraints let it: a pinned body
/// stays on its frame, a free one goes where it is pulled — and where the
/// constraints hold already and nothing pulls, nothing is solved.
#[test]
fn a_pull_moves_what_the_constraints_leave_free() {
    let mut runner = runner();
    let program = Program::from_steps(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let x = inputs(&[("x", 1.0)]);
    let state = at(&[("body", [1.0, 0.0, 0.0]), ("loose", [0.0; 3])]);
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
    let solved = solve(runner.rerun(&program, &x, &state), &state, &both, true).unwrap();
    assert!(solved.report.converged);
    assert_near(where_is(&solved.state, "body"), [1.0, 0.0, 0.0]);
    assert_near(where_is(&solved.state, "loose"), [0.0, 2.0, 0.0]);
    assert_eq!(solved.report.moved, ["loose"]);

    let still = solve(
        runner.rerun(&program, &x, &state),
        &state,
        &problem(&["body", "loose"], vec![]),
        true,
    )
    .unwrap();
    assert!(still.report.moved.is_empty());
    assert_eq!(still.report.iterations, 0, "nothing to solve");
}

/// A limit stops a pull at it, and the report says so.
#[test]
fn a_limit_stops_a_pull() {
    let mut runner = runner();
    let limit = LimitArgs {
        state: "body".into(),
        most: 2.0,
    };
    let program = Program::from_steps(vec![
        ProgramStep::new::<LimitOperation>("limit", &limit).unwrap(),
    ]);
    let state = at(&[("body", [0.0; 3])]);
    let none = inputs(&[]);
    let pull = Toward {
        state: "body".into(),
        to: [5.0, 1.0, 0.0],
    };
    let solved = solve(
        runner.rerun(&program, &none, &state),
        &state,
        &problem(&["body"], vec![&pull]),
        true,
    )
    .unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    assert_eq!(solved.report.at_limit, ["limit"]);
    assert_near(where_is(&solved.state, "body"), [2.0, 1.0, 0.0]);
}

/// Two pins of one body to different frames cannot both hold: the solve
/// says which fail, the conflict names both, and a body pinned is held
/// fast where it is — free only to turn about it — where a body nothing
/// holds is free in every way.
#[test]
fn conflicts_and_freedom() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        input_frame("a", "a", "x"),
        input_frame("b", "b", "y"),
        pin("one", "a", "body"),
        pin("two", "b", "body"),
    ]);
    let values = inputs(&[("x", 1.0), ("y", 2.0)]);
    let state = at(&[("body", [0.0; 3]), ("loose", [0.0; 3])]);
    let both = problem(&["body", "loose"], vec![]);
    let solved = solve(runner.rerun(&program, &values, &state), &state, &both, true).unwrap();
    assert!(!solved.report.converged);
    assert_eq!(solved.report.failed, ["one", "two"]);
    // Nothing to enclose, and the enclosure says why, by name.
    let [(group, why)] = &solved.enclosure.as_ref().unwrap().unproven[..] else {
        panic!("{:?}", solved.enclosure);
    };
    assert_eq!(group, &["body"]);
    assert!(why.contains("one") && why.contains("two"), "{why}");
    assert_eq!(
        conflicting(runner.rerun(&program, &values, &state), &state, &both).unwrap(),
        ["one", "two"]
    );

    let single = Program::from_steps(vec![input_frame("a", "a", "x"), pin("one", "a", "body")]);
    let free = freedom(runner.rerun(&single, &values, &state), &state, &both).unwrap();
    assert_eq!(free.states["body"], 3);
    assert_eq!(free.states["loose"], 6);
    assert_eq!(free.total, 9);
}
