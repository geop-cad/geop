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
    part::StepContext,
    placed::{self, PlacedArgs},
    program::{Program, ProgramStep},
    values::StateValues,
    workspace::Workspace,
};
use serde::{Deserialize, Serialize};

use super::{
    Running, assert_translation, body_at, errors, input_frame, inputs, pin, pose_of, registry,
};
use crate::{
    constraint::{Constraints, Remade, made_again},
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

    fn run(ctx: &mut StepContext, args: &PlaceArgs) -> GeopResult<()> {
        let step = ctx.step();
        let placed = PlacedArgs {
            file: args.file.clone(),
            inputs: inputs(&[("x", 1.0)]),
        };
        placed::place(ctx, step, &placed)
    }
}

thread_local! {
    /// The programs of the tests' workspace, by file, each with the
    /// revision it was written in.
    static FILES: RefCell<BTreeMap<String, (Rc<Program>, u64)>> = const { RefCell::new(BTreeMap::new()) };
}

/// The tests' programs, by the file each was written to ([`place`]).
pub(super) struct Files;

impl Workspace for Files {
    fn program(&self, path: &str) -> GeopResult<Rc<Program>> {
        let program = FILES.with(|files| files.borrow().get(path).map(|(p, _)| p.clone()));
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

    fn revision(&self, path: &str) -> u64 {
        FILES.with(|files| {
            files
                .borrow()
                .get(path)
                .map_or(0, |(_, revision)| *revision)
        })
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

    fn run(ctx: &mut StepContext, args: &FreeArgs) -> GeopResult<()> {
        declare_unknowns(ctx, &args.state, |_, state| {
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
        let mut files = files.borrow_mut();
        let program = Program::from_steps(steps);
        if files.get(&file).is_none_or(|(kept, _)| **kept != program) {
            let revision = files.values().map(|(_, r)| *r).max().unwrap_or_default() + 1;
            files.insert(file.clone(), (Rc::new(program), revision));
        }
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
    let given = StateValues::from([("/p:body".to_string(), body_at([3.0, 0.0, 0.0]))]);
    assert_eq!(errors(&runner.run(&program, &inputs(&[]), &given)), []);

    let reader = runner.part.current();
    let constraints = Constraints::of(&reader).unwrap();
    let names: Vec<&str> = constraints.iter().map(|(name, _)| name).collect();
    assert_eq!(names, ["/p:pin"]);
    let pinned = constraints.get("/p:pin").unwrap();
    assert_eq!(pinned.jacobian.keys().collect::<Vec<_>>(), ["/p:body"]);
    assert_translation(&pinned.jacobian["/p:body"]);
    let unknowns = Unknowns::of(&reader).unwrap();
    let declared: Vec<&str> = unknowns.iter().map(|u| u.state.as_str()).collect();
    assert_eq!(declared, ["/p:body"]);
    // It starts where the instance has it.
    assert_eq!(
        unknowns.get("/p:body").unwrap().seed,
        Some(body_at([3.0, 0.0, 0.0]))
    );

    let problem = Problem {
        free: BTreeMap::from([("/p:body".to_string(), Mobility::Held)]),
        preferences: Vec::new(),
        scale: Scalar::ONE,
        structure: None,
    };
    let x = inputs(&[]);
    let run = runner.rerun(&program, &x, &given);
    let solved = solve(run, &given, &problem, true).unwrap();
    assert!(solved.report.converged, "{:?}", solved.report);
    let at = pose_of(&solved.state["/p:body"])
        .position()
        .to_array()
        .map(|c| c.to_f64());
    assert!((at[0] - 1.0).abs() < 1e-8 && at[1].abs() < 1e-8, "{at:?}");
}

/// What a solve reads at every state it tries: each constraint made again
/// by its step, of the part as built at another state, is the constraint a
/// run of the program at the state tried makes — the part's own, and an
/// instance's behind its name — without running the program. Asked again
/// where the state it read is as it was, it is not made again; where that
/// moved, only its own step's are.
#[test]
fn a_constraint_made_again_at_a_state_tried_is_the_one_a_run_there_makes() {
    let inner = vec![
        input_frame("a", "a", "x"),
        free("free", "body"),
        pin("pin", "a", "body"),
    ];
    let program = Program::from_steps(vec![
        input_frame("b", "b", "x"),
        place("q", inner),
        pin("own", "b", "body"),
    ]);
    let x = inputs(&[("x", 2.0)]);
    let at = |dx: f64| {
        StateValues::from([
            ("body".to_string(), body_at([dx, 1.0, 0.0])),
            ("/q:body".to_string(), body_at([3.0 + dx, 0.0, 0.0])),
        ])
    };
    let mut runner = Running::of(registry());
    assert_eq!(errors(&runner.run(&program, &x, &at(0.0))), []);
    let built = Constraints::of(&runner.part.current()).unwrap();
    let tried = at(5.0);
    let remade = Remade::default();
    let asked: Vec<_> = built.defining().map(|(_, at)| at).collect();
    let made_at = |state: &StateValues| -> Vec<_> {
        let reader = runner.part.current().trying(state);
        let made: GeopResult<Vec<_>> = remade.constraints(&reader, &asked).into_iter().collect();
        made.unwrap()
    };
    let count = made_again();
    let again = made_at(&tried);
    assert_eq!(made_again() - count, 2, "each step's constraints made once");
    assert_eq!(made_at(&tried), again);
    assert_eq!(made_again() - count, 2, "and not again where nothing moved");
    let mut own_moved = tried.clone();
    own_moved.insert("body".to_string(), body_at([6.0, 1.0, 0.0]));
    made_at(&own_moved);
    assert_eq!(made_again() - count, 3, "only the step whose state moved");

    assert_eq!(errors(&runner.run(&program, &x, &tried)), []);
    let ran = Constraints::of(&runner.part.current()).unwrap();
    let ran: Vec<_> = ran.iter().map(|(_, c)| c.clone()).collect();
    assert_eq!(again, ran);
    let was: Vec<_> = built.iter().map(|(_, c)| c.clone()).collect();
    assert_ne!(again, was, "the state tried moves both");
}
