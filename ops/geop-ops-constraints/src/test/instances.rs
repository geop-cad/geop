//! A part placed in a part: its constraints and unknowns come up behind its
//! name, and the part's solve solves them.

use std::{any::Any, collections::BTreeMap, rc::Rc};

use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    solvers::system::Mobility,
};
use geop_core_program::{
    instance::{Instance, Placement, Subpart},
    operation::Operation,
    part::{Part, Reader},
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
};
use serde::{Deserialize, Serialize};

use super::{Pose, Sc, errors, identity, input_frame, inputs, pin, registry};
use crate::{
    constraint::Constraints,
    solve::{Problem, Run, solve},
    unknown::{Unknown, Unknowns, declare_unknowns},
};

/// A part a program built, as placed.
struct Built(Part<Sc, Pose>);

impl Subpart<Sc, Pose> for Built {
    fn part(&self) -> &Part<Sc, Pose> {
        &self.0
    }

    fn into_any(self: Rc<Self>) -> Rc<dyn Any> {
        self
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub(super) struct PlaceArgs {
    program: Program,
}

/// Places the part `program` builds, with the input `x` at 1, as the
/// instance named by the step.
pub(super) struct PlaceOperation;

impl Operation for PlaceOperation {
    const KIND: &'static str = "place";
    type Args = PlaceArgs;
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &PlaceArgs) -> GeopResult<()> {
        part.define_instance(step, &(step.to_string(), args.program.clone()), build)
    }
}

fn build(
    _: &Reader<Sc, Pose>,
    (step, program): &(String, Program),
    state: &BTreeMap<String, Pose>,
) -> GeopResult<Instance<Sc, Pose>> {
    let mut runner = ProgramRunner::new(registry());
    let output = runner.run(program, &inputs(&[("x", 1.0)]), state);
    assert_eq!(errors(&output), []);
    Ok(Instance {
        component: Rc::new(Built(runner.part)),
        placement: Placement::State(format!("{step}.pose")),
    })
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub(super) struct FreeArgs {
    state: String,
}

/// Lets a solve move the position `state`.
pub(super) struct FreeOperation;

impl Operation for FreeOperation {
    const KIND: &'static str = "free";
    type Args = FreeArgs;
    type Input = Sc;
    type State = Pose;

    fn run(_: &str, part: &mut Part<Sc, Pose>, args: &FreeArgs) -> GeopResult<()> {
        declare_unknowns(part, &args.state, |_, state| {
            Ok(vec![Unknown {
                state: state.clone(),
                mobility: Mobility::Held,
                seed: None,
                unless: None,
            }])
        })
    }
}

fn place(id: &str, steps: Vec<ProgramStep>) -> ProgramStep {
    let args = PlaceArgs {
        program: Program::new(steps),
    };
    ProgramStep::new::<PlaceOperation>(id, &args).unwrap()
}

fn free(id: &str, state: &str) -> ProgramStep {
    let args = FreeArgs {
        state: state.to_string(),
    };
    ProgramStep::new::<FreeOperation>(id, &args).unwrap()
}

/// An instance's constraint on its own state is the part's on the state
/// behind the instance's name, and so is what it lets a solve move. The
/// part's solve moves it until the instance's constraint holds, running
/// the instance again at every state it tries.
#[test]
fn an_instance_is_solved_in_the_part_it_is_placed_in() {
    let inner = vec![
        input_frame("a", "a", "x"),
        free("free", "body"),
        pin("pin", "a", "body"),
    ];
    let program = Program::new(vec![place("p", inner)]);
    let mut runner = ProgramRunner::new(registry());
    let given = BTreeMap::from([("p/body".to_string(), Pose::at([3.0, 0.0, 0.0]))]);
    assert_eq!(errors(&runner.run(&program, &inputs(&[]), &given)), []);

    let reader = runner.part.current();
    let constraints = Constraints::of(&reader).unwrap();
    let names: Vec<&str> = constraints.iter().map(|(name, _)| name).collect();
    assert_eq!(names, ["p/pin"]);
    let pinned = constraints.get("p/pin").unwrap();
    assert_eq!(
        pinned.jacobian,
        BTreeMap::from([("p/body".to_string(), identity(3))])
    );
    let unknowns = Unknowns::of(&reader).unwrap();
    let declared: Vec<&str> = unknowns.iter().map(|u| u.state.as_str()).collect();
    assert_eq!(declared, ["p/body"]);
    // It starts where the instance has it.
    assert_eq!(
        unknowns.get("p/body").unwrap().seed,
        Some(Pose::at([3.0, 0.0, 0.0]))
    );

    let problem = Problem {
        free: BTreeMap::from([("p/body".to_string(), Mobility::Held)]),
        preferences: Vec::new(),
        scale: Sc::ONE,
    };
    let x = inputs(&[]);
    let run = Run {
        runner: &mut runner,
        program: &program,
        inputs: &x,
    };
    let solved = solve(run, &given, &problem).unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    let at = solved.state["p/body"].position.to_array().map(|c| c.to_f64());
    assert!((at[0] - 1.0).abs() < 1e-8 && at[1].abs() < 1e-8, "{at:?}");
}
