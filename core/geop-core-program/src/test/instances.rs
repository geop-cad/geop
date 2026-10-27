//! Placed parts: a step places the part a program of its own builds, and
//! the part reads its elements behind the instance's name.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    operation::Operation,
    part::StepContext,
    placed::{PlacedArgs, place},
    program::{Program, ProgramStep},
    test::{
        StateFrameArgs, StateFrameOperation, assert_encloses, inputs, middle_frame, ok, pose_at,
        runner, take_built, whole_frame,
    },
    values::StateValues,
    workspace::Workspace,
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LoadArgs {
    pub file: String,
}

/// Places the part the program in `file` builds as the instance named by
/// the step.
pub struct LoadOperation;

impl Operation for LoadOperation {
    const KIND: &'static str = "load";
    type Args = LoadArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let placed = PlacedArgs {
            file: args.file.clone(),
            inputs: inputs(&[]),
            then: Vec::new(),
        };
        let step = ctx.step();
        place(ctx, step, &placed)
    }
}

thread_local! {
    /// The programs of the tests' workspace, by file, each with the
    /// revision it was written in.
    static FILES: RefCell<BTreeMap<String, (Rc<Program>, u64)>> = const { RefCell::new(BTreeMap::new()) };
}

/// The tests' programs, by the file each was written to ([`load`]).
pub(crate) struct Files;

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

thread_local! {
    /// How many times a counted step ran.
    static COUNTED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Counts that it ran, and produces nothing.
pub struct CountOperation;

impl Operation for CountOperation {
    const KIND: &'static str = "count";
    type Args = ();

    fn run(_: &mut StepContext, _: &()) -> GeopResult<()> {
        COUNTED.set(COUNTED.get() + 1);
        Ok(())
    }
}

fn count(id: &str) -> ProgramStep {
    ProgramStep::new::<CountOperation>(id, &()).unwrap()
}

/// The program of `steps`, written to the file `<name>.geop`.
pub(crate) fn write(name: &str, steps: Vec<ProgramStep>) -> String {
    let file = format!("{name}.geop");
    let program = Program::from_steps(steps);
    FILES.with(|files| {
        let mut files = files.borrow_mut();
        if files.get(&file).is_none_or(|(kept, _)| **kept != program) {
            let revision = files.values().map(|(_, r)| *r).max().unwrap_or_default() + 1;
            files.insert(file.clone(), (Rc::new(program), revision));
        }
    });
    file
}

/// The step `id` placing the part the program in `file` builds.
pub(crate) fn place_file(id: &str, file: &str) -> ProgramStep {
    let args = LoadArgs {
        file: file.to_string(),
    };
    ProgramStep::new::<LoadOperation>(id, &args).unwrap()
}

/// The step `id` placing the program of `steps`, written to the file
/// `<id>.geop` — the same file for the same steps.
pub(crate) fn load(id: &str, steps: Vec<ProgramStep>) -> ProgramStep {
    place_file(id, &write(id, steps))
}

/// A frame at the state `state`.
fn state_frame(id: &str, state: &str) -> ProgramStep {
    let args = StateFrameArgs {
        state: state.to_string(),
    };
    ProgramStep::new::<StateFrameOperation>(id, &args).unwrap()
}

fn state(values: &[(&str, [f64; 3])]) -> StateValues {
    (values.iter())
        .map(|(name, at)| (name.to_string(), pose_at(*at)))
        .collect()
}

/// An element of a placed part is read behind its name, as its own part
/// has it, and the placed part is built with the part's state under its
/// name. A value of the state it does not read builds nothing.
#[test]
fn a_placed_part_is_built_with_the_state_under_its_name() {
    let mut runner = runner();
    let program = Program::from_steps(vec![load("p", vec![state_frame("f", "s")])]);

    let at = |x, y| state(&[("/p:s", [x, 0.0, 0.0]), ("p.pose", [y, 0.0, 0.0])]);
    ok(runner.run(&program, &inputs(&[]), &at(1.0, 5.0)));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("/p:f")).unwrap();
    assert_eq!(placed.instance, "p");
    assert_encloses(placed.element.data, [1.0, 0.0, 0.0]);
    assert_eq!(take_built(), ["f@f"]);

    ok(runner.run(&program, &inputs(&[]), &at(1.0, 6.0)));
    assert_eq!(take_built(), Vec::<String>::new());

    ok(runner.run(&program, &inputs(&[]), &at(2.0, 6.0)));
    assert_eq!(take_built(), ["f@f"]);
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("/p:f")).unwrap();
    assert_encloses(placed.element.data, [2.0, 0.0, 0.0]);
}

/// A placed part whose state changed is built again from the part it was:
/// what reads what changed is made again, in it and in the part placing
/// it, the rest is reused. A value given under its name that it does not
/// read builds nothing. Its part has changed only once a build changed
/// something in it.
#[test]
fn a_change_in_a_placed_part_makes_again_only_what_reads_it() {
    let mut runner = runner();
    let frames = vec![state_frame("f", "s"), state_frame("g", "t")];
    let program = Program::from_steps(vec![load("p", frames), middle_frame("m", "/p:f", "/p:f")]);
    let mut run = |given: &[(&str, [f64; 3])]| {
        ok(runner.run(&program, &inputs(&[]), &state(given)));
        let revision = runner.part.current().placed_part("p").unwrap().revision();
        (take_built(), revision)
    };
    let (built, first) = run(&[]);
    assert_eq!(built, ["f@f", "g@g", "m@m"]);
    assert_eq!(run(&[("/p:other", [1.0, 0.0, 0.0])]), (vec![], first));
    // Given as the default it read, `f` is made again, alike: nothing
    // reading it is.
    assert_eq!(run(&[("/p:s", [0.0; 3])]), (vec!["f@f".to_string()], first));

    let (built, moved) = run(&[("/p:s", [1.0, 0.0, 0.0])]);
    assert_eq!(built, ["f@f", "m@m"]);
    assert_ne!(moved, first);
    let (built, turned) = run(&[("/p:s", [1.0, 0.0, 0.0]), ("/p:t", [1.0, 0.0, 0.0])]);
    assert_eq!(built, ["g@g"]);
    assert_ne!(turned, moved);

    let (built, _) = run(&[]);
    assert_eq!(built, ["f@f", "g@g", "m@m"]);
}

/// A placed part given everything as the last run gave it is reused: its
/// steps do not run again, however deep, and what reads it is reused.
/// Given anything otherwise, its steps run again.
#[test]
fn a_placed_part_given_nothing_new_is_reused() {
    let mut runner = runner();
    let inner = vec![
        count("c"),
        load("q", vec![count("c"), state_frame("f", "s")]),
    ];
    let program = Program::from_steps(vec![
        load("p", inner),
        middle_frame("m", "/p/q:f", "/p/q:f"),
    ]);
    let mut run = |given: &[(&str, [f64; 3])]| {
        COUNTED.set(0);
        ok(runner.run(&program, &inputs(&[]), &state(given)));
        (COUNTED.get(), take_built())
    };
    assert_eq!(run(&[]), (2, vec!["f@f".into(), "m@m".into()]));
    assert_eq!(run(&[]), (0, vec![]));
    assert_eq!(
        run(&[("/p/q:s", [1.0, 0.0, 0.0])]),
        (2, vec!["f@f".into(), "m@m".into()])
    );
    assert_eq!(run(&[("/p/q:s", [1.0, 0.0, 0.0])]), (0, vec![]));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("/p/q:f")).unwrap();
    assert_encloses(placed.element.data, [1.0, 0.0, 0.0]);
}

/// An element of a placed part is no element of the part itself.
#[test]
fn an_element_of_a_placed_part_is_not_the_parts_own() {
    let mut runner = runner();
    let program = Program::from_steps(vec![load("p", vec![state_frame("f", "s")])]);
    ok(runner.run(&program, &inputs(&[]), &state(&[])));
    let error = (runner.part.current())
        .retrieve_element(&whole_frame("/p:f"))
        .unwrap_err()
        .to_string();
    assert!(error.contains("placed part `p`"), "{error}");
}

/// A placed part places parts of its own, each built with the state under
/// both names; only the part's own are listed as its.
#[test]
fn a_part_placed_in_a_placed_part_is_read_behind_both_names() {
    let mut runner = runner();
    let inner = || vec![load("q", vec![state_frame("f", "s")])];
    let program = Program::from_steps(vec![load("p", inner()), load("c", inner())]);
    let given = state(&[("/p/q:s", [0.0, 0.0, 4.0]), ("/c/q:s", [0.0, 0.0, 6.0])]);
    ok(runner.run(&program, &inputs(&[]), &given));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("/p/q:f")).unwrap();
    assert_eq!(placed.instance, "p/q");
    assert_encloses(placed.element.data, [0.0, 0.0, 4.0]);
    let other = reader.retrieve_placed(&whole_frame("/c/q:f")).unwrap();
    assert_encloses(other.element.data, [0.0, 0.0, 6.0]);
    assert_eq!(reader.instances(), ["p", "c"]);
    assert_eq!(reader.placed_part("p").unwrap().instances(), ["q"]);
}

/// A thousand parts placed from one program, with the same inputs and no
/// state of their own, are one build: the rest share it.
#[test]
fn a_thousand_equal_placed_parts_build_once() {
    let mut runner = runner();
    let file = write("screw", vec![count("c"), state_frame("f", "s")]);
    let steps = (0..1000)
        .map(|k| place_file(&format!("screw{k}"), &file))
        .collect();
    let program = Program::from_steps(steps);
    COUNTED.set(0);
    ok(runner.run(&program, &inputs(&[]), &state(&[])));
    let counts = runner.runner.counts();
    assert_eq!(counts.builds, 1);
    assert_eq!(counts.placed, 1000);
    assert_eq!(COUNTED.get(), 1, "the screw's steps run once");
    assert_eq!(take_built(), ["f@f"]);
    assert_eq!(runner.runner.cached_builds(), 1);
}

/// Two parts placed from one program, each with a joint of its own: moving
/// one's builds that one again, from what it was, and reuses the other —
/// whose placed node is not even made again.
#[test]
fn moving_a_joint_of_one_of_two_placed_parts_builds_only_that_one() {
    let mut runner = runner();
    let file = write("arm", vec![state_frame("f", "j"), state_frame("g", "k")]);
    let program = Program::from_steps(vec![place_file("left", &file), place_file("right", &file)]);
    let at = |left: f64, right: f64| {
        state(&[
            ("/left:j", [left, 0.0, 0.0]),
            ("/right:j", [right, 0.0, 0.0]),
        ])
    };
    ok(runner.run(&program, &inputs(&[]), &at(1.0, 2.0)));
    assert_eq!(take_built(), ["f@f", "g@g", "f@f", "g@g"]);
    let before = runner.runner.counts();

    ok(runner.run(&program, &inputs(&[]), &at(3.0, 2.0)));
    let after = runner.runner.counts();
    assert_eq!(after.builds - before.builds, 1, "one part is built again");
    assert_eq!(
        after.placed - before.placed,
        1,
        "the other's node is reused"
    );
    assert_eq!(
        take_built(),
        ["f@f"],
        "only what reads the joint is made again"
    );
    let reader = runner.part.current();
    let left = reader.retrieve_placed(&whole_frame("/left:f")).unwrap();
    assert_encloses(left.element.data, [3.0, 0.0, 0.0]);
    let right = reader.retrieve_placed(&whole_frame("/right:f")).unwrap();
    assert_encloses(right.element.data, [2.0, 0.0, 0.0]);
}

/// Dragging a placed part moves its pose, the placing part's state: no
/// placed node is made again, and nothing is built.
#[test]
fn dragging_a_placed_part_by_its_pose_builds_nothing() {
    let mut runner = runner();
    let file = write("block", vec![state_frame("f", "s")]);
    let program = Program::from_steps(vec![place_file("b", &file), place_file("c", &file)]);
    ok(runner.run(&program, &inputs(&[]), &state(&[("b.pose", [0.0; 3])])));
    take_built();
    let before = runner.runner.counts();
    ok(runner.run(
        &program,
        &inputs(&[]),
        &state(&[("b.pose", [5.0, 0.0, 0.0])]),
    ));
    assert_eq!(runner.runner.counts(), before);
    assert_eq!(take_built(), Vec::<String>::new());
}

/// A file placing itself, however deep, is refused with the chain.
#[test]
fn a_file_placing_itself_is_refused() {
    let mut runner = runner();
    write("loop_b", vec![place_file("a", "loop_a.geop")]);
    write("loop_a", vec![place_file("b", "loop_b.geop")]);
    let program = Program::from_steps(vec![place_file("a", "loop_a.geop")]);
    let output = runner.run(&program, &inputs(&[]), &state(&[]));
    let error = output.step_errors[0].1.to_string();
    assert!(
        error.contains("loop_a.geop -> loop_b.geop -> loop_a.geop"),
        "{error}"
    );
}
