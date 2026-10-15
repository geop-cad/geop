//! A part placed in a part: its constraints and unknowns come up behind its
//! name, and the part's solve solves them.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::system::Mobility,
};
use geop_core_program::{
    operation::Operation,
    part::Part,
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    values::{Design, StateValues},
    workspace::Workspace,
};
use serde::{Deserialize, Serialize};

use super::{
    Running, assert_translation, body_at, errors, input_frame, inputs, pin, pose_of, registry,
};
use crate::{
    constraint::Constraints,
    solve::{Problem, solve},
    unknown::{Unknown, Unknowns, declare_unknowns},
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub(super) struct PlaceArgs {
    file: String,
}

/// Places the part the program in `file` builds, with the input `x` at 1,
/// as the instance named by the step.
pub(super) struct PlaceOperation;

impl Operation for PlaceOperation {
    const KIND: &'static str = "place";
    type Args = PlaceArgs;

    fn run(
        step: &str,
        runner: &ProgramRunner,
        part: &mut Part,
        args: &PlaceArgs,
    ) -> GeopResult<()> {
        runner.load_subpart(part, step, &args.file, &inputs(&[("x", 1.0)]))
    }
}

thread_local! {
    /// The programs of the tests' workspace, by file.
    static FILES: RefCell<BTreeMap<String, Rc<Program>>> = const { RefCell::new(BTreeMap::new()) };
}

/// The tests' programs, by the file each was written to ([`place`]).
pub(super) struct Files;

impl Workspace for Files {
    fn program(&self, path: &str) -> GeopResult<Rc<Program>> {
        let program = FILES.with(|files| files.borrow().get(path).cloned());
        program.ok_or_else(|| GeopError::new(format!("there is no file {path:?}")))
    }

    fn read(&self, path: &str) -> GeopResult<Rc<str>> {
        Err(GeopError::new(format!("{path:?} is no file to read")))
    }

    fn list(&self) -> Vec<String> {
        FILES.with(|files| files.borrow().keys().cloned().collect())
    }

    fn resolve(&self, _: &str, name: &str) -> String {
        name.to_string()
    }

    fn relative(&self, _: &str, path: &str) -> String {
        path.to_string()
    }

    fn revision(&self) -> u64 {
        0
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub(super) struct FreeArgs {
    state: String,
}

/// Lets a solve move the pose `state`.
pub(super) struct FreeOperation;

impl Operation for FreeOperation {
    const KIND: &'static str = "free";
    type Args = FreeArgs;

    fn run(_: &str, _: &ProgramRunner, part: &mut Part, args: &FreeArgs) -> GeopResult<()> {
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

/// The step `id` placing the program of `steps`, written to the file
/// `<id>.geop`.
fn place(id: &str, steps: Vec<ProgramStep>) -> ProgramStep {
    let file = format!("{id}.geop");
    FILES.with(|files| {
        files
            .borrow_mut()
            .insert(file.clone(), Rc::new(Program::from_steps(steps)))
    });
    ProgramStep::new::<PlaceOperation>(id, &PlaceArgs { file }).unwrap()
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
    let program = Program::from_steps(vec![place("p", inner)]);
    let mut runner = Running::of(registry());
    let given = StateValues::from([("p/body".to_string(), body_at([3.0, 0.0, 0.0]))]);
    assert_eq!(errors(&runner.run(&program, &inputs(&[]), &given)), []);

    let reader = runner.part.current();
    let constraints = Constraints::of(&reader).unwrap();
    let names: Vec<&str> = constraints.iter().map(|(name, _)| name).collect();
    assert_eq!(names, ["p/pin"]);
    let pinned = constraints.get("p/pin").unwrap();
    assert_eq!(pinned.jacobian.keys().collect::<Vec<_>>(), ["p/body"]);
    assert_translation(&pinned.jacobian["p/body"]);
    let unknowns = Unknowns::of(&reader).unwrap();
    let declared: Vec<&str> = unknowns.iter().map(|u| u.state.as_str()).collect();
    assert_eq!(declared, ["p/body"]);
    // It starts where the instance has it.
    assert_eq!(
        unknowns.get("p/body").unwrap().seed,
        Some(body_at([3.0, 0.0, 0.0]))
    );

    let problem = Problem {
        free: BTreeMap::from([("p/body".to_string(), Mobility::Held)]),
        preferences: Vec::new(),
        scale: Design::ONE,
    };
    let x = inputs(&[]);
    let run = runner.rerun(&program, &x);
    let solved = solve(run, &given, &problem).unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    let at = pose_of(&solved.state["p/body"])
        .position()
        .to_array()
        .map(|c| c.to_f64());
    assert!((at[0] - 1.0).abs() < 1e-8 && at[1].abs() < 1e-8, "{at:?}");
}
