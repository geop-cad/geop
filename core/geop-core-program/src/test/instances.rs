//! Instances: a step places the part a program of its own builds, and the
//! part reads its elements behind the instance's name.

use std::{cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    operation::Operation,
    part::Part,
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
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

/// Loads the part the program in `file` builds as the instance named by
/// the step.
pub struct LoadOperation;

impl Operation for LoadOperation {
    const KIND: &'static str = "load";
    type Args = LoadArgs;

    fn run(
        step: &str,
        runner: &ProgramRunner,
        part: &mut Part,
        args: &Self::Args,
    ) -> GeopResult<()> {
        runner.load_subpart(part, step, &args.file, &inputs(&[]))
    }
}

thread_local! {
    /// The programs of the tests' workspace, by file.
    static FILES: RefCell<BTreeMap<String, Rc<Program>>> = const { RefCell::new(BTreeMap::new()) };
}

/// The tests' programs, by the file each was written to ([`load`]).
pub(crate) struct Files;

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

thread_local! {
    /// How many times a counted step ran.
    static COUNTED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Counts that it ran, and builds nothing.
pub struct CountOperation;

impl Operation for CountOperation {
    const KIND: &'static str = "count";
    type Args = ();

    fn run(_: &str, _: &ProgramRunner, _: &mut Part, _: &()) -> GeopResult<()> {
        COUNTED.set(COUNTED.get() + 1);
        Ok(())
    }
}

fn count(id: &str) -> ProgramStep {
    ProgramStep::new::<CountOperation>(id, &()).unwrap()
}

/// The step `id` loading the program of `steps`, written to the file
/// `<id>.geop` — the same file for the same steps.
pub(crate) fn load(id: &str, steps: Vec<ProgramStep>) -> ProgramStep {
    let file = format!("{id}.geop");
    let program = Program::from_steps(steps);
    FILES.with(|files| {
        let mut files = files.borrow_mut();
        if files.get(&file).is_none_or(|kept| **kept != program) {
            files.insert(file.clone(), Rc::new(program));
        }
    });
    ProgramStep::new::<LoadOperation>(id, &LoadArgs { file }).unwrap()
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

/// An element of an instance is read behind its name, as its own part has
/// it, and the instance is run with the part's state under its name. A
/// value of the state its part does not read builds nothing.
#[test]
fn an_instance_is_run_with_the_state_under_its_name() {
    let mut runner = runner();
    let program = Program::from_steps(vec![load("p", vec![state_frame("f", "s")])]);

    let at = |x, y| state(&[("p/s", [x, 0.0, 0.0]), ("p.pose", [y, 0.0, 0.0])]);
    ok(runner.run(&program, &inputs(&[]), &at(1.0, 5.0)));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/f")).unwrap();
    assert_eq!(placed.instance, "p");
    assert_encloses(placed.element.data, [1.0, 0.0, 0.0]);
    assert_eq!(take_built(), ["f@f"]);

    ok(runner.run(&program, &inputs(&[]), &at(1.0, 6.0)));
    assert_eq!(take_built(), Vec::<String>::new());

    ok(runner.run(&program, &inputs(&[]), &at(2.0, 6.0)));
    assert_eq!(take_built(), ["f@f"]);
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/f")).unwrap();
    assert_encloses(placed.element.data, [2.0, 0.0, 0.0]);
}

/// The part of an instance is run again in every run, from what it built
/// last: what reads what changed is built again, in it and in the part
/// placing it, the rest is reused. A value given under its name that it
/// does not read builds nothing. Its part has changed only once a run
/// changed something in it.
#[test]
fn a_change_in_an_instance_builds_again_only_what_reads_it() {
    let mut runner = runner();
    let frames = vec![state_frame("f", "s"), state_frame("g", "t")];
    let program = Program::from_steps(vec![load("p", frames), middle_frame("m", "p/f", "p/f")]);
    let mut run = |given: &[(&str, [f64; 3])]| {
        ok(runner.run(&program, &inputs(&[]), &state(given)));
        let id = runner.part.current().inside("p").unwrap().changed_at();
        (take_built(), id)
    };
    let (built, first) = run(&[]);
    assert_eq!(built, ["f@f", "g@g", "m@m"]);
    assert_eq!(run(&[("p/other", [1.0, 0.0, 0.0])]), (vec![], first));
    // Given as the default it read, `f` is built again, alike: nothing
    // reading it is.
    assert_eq!(run(&[("p/s", [0.0; 3])]), (vec!["f@f".to_string()], first));

    let (built, moved) = run(&[("p/s", [1.0, 0.0, 0.0])]);
    assert_eq!(built, ["f@f", "m@m"]);
    assert_ne!(moved, first);
    let (built, turned) = run(&[("p/s", [1.0, 0.0, 0.0]), ("p/t", [1.0, 0.0, 0.0])]);
    assert_eq!(built, ["g@g"]);
    assert_ne!(turned, moved);

    let (built, _) = run(&[]);
    assert_eq!(built, ["f@f", "g@g", "m@m"]);
}

/// The part of an instance given everything as the last run gave it is
/// carried over: its steps do not run again, however deep, and what reads
/// it is reused. Given anything otherwise, its steps run again.
#[test]
fn an_instance_given_nothing_new_is_carried_over() {
    let mut runner = runner();
    let inner = vec![
        count("c"),
        load("q", vec![count("c"), state_frame("f", "s")]),
    ];
    let program = Program::from_steps(vec![load("p", inner), middle_frame("m", "p/q/f", "p/q/f")]);
    let mut run = |given: &[(&str, [f64; 3])]| {
        COUNTED.set(0);
        ok(runner.run(&program, &inputs(&[]), &state(given)));
        (COUNTED.get(), take_built())
    };
    assert_eq!(run(&[]), (2, vec!["f@f".into(), "m@m".into()]));
    assert_eq!(run(&[]), (0, vec![]));
    assert_eq!(
        run(&[("p/q/s", [1.0, 0.0, 0.0])]),
        (2, vec!["f@f".into(), "m@m".into()])
    );
    assert_eq!(run(&[("p/q/s", [1.0, 0.0, 0.0])]), (0, vec![]));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/q/f")).unwrap();
    assert_encloses(placed.element.data, [1.0, 0.0, 0.0]);
}

/// An element of an instance is no element of the part itself.
#[test]
fn an_element_of_an_instance_is_not_the_parts_own() {
    let mut runner = runner();
    let program = Program::from_steps(vec![load("p", vec![state_frame("f", "s")])]);
    ok(runner.run(&program, &inputs(&[]), &state(&[])));
    let error = (runner.part.current())
        .retrieve_element(&whole_frame("p/f"))
        .unwrap_err()
        .to_string();
    assert!(error.contains("placed part `p`"), "{error}");
}

/// The part of an instance has instances of its own, each run with the
/// state under both names; only the part's own are listed as its.
#[test]
fn an_instance_of_an_instance_is_read_behind_both_names() {
    let mut runner = runner();
    let inner = || vec![load("q", vec![state_frame("f", "s")])];
    let program = Program::from_steps(vec![load("p", inner()), load("c", inner())]);
    let given = state(&[("p/q/s", [0.0, 0.0, 4.0]), ("c/q/s", [0.0, 0.0, 6.0])]);
    ok(runner.run(&program, &inputs(&[]), &given));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/q/f")).unwrap();
    assert_eq!(placed.instance, "p/q");
    assert_encloses(placed.element.data, [0.0, 0.0, 4.0]);
    let other = reader.retrieve_placed(&whole_frame("c/q/f")).unwrap();
    assert_encloses(other.element.data, [0.0, 0.0, 6.0]);
    assert_eq!(reader.subparts(), ["p", "c"]);
    assert_eq!(reader.inside("p").unwrap().subparts(), ["q"]);
}
