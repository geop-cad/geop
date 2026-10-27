//! Steps that do not run ([`ProgramStep::active`]): a suppressed step
//! produces nothing, the steps after it see the part as the steps before it
//! left it, and what read what it made says which step that was.

use std::{collections::BTreeMap, rc::Rc};

use crate::{
    part::Part,
    program::{Program, ProgramStep},
    runner::Runner,
    test::{
        Running, add_two_frame, errors, frame, inputs, middle_frame, no_state, number, ok,
        registry, runner, shift_frame, take_built,
    },
    testing::{Shelf, assert_incremental, assert_reopens, assert_round_trips},
    workspace::{Files, Location, Workspace},
};

/// `step`, running where `active` is not 0.
fn when(active: &str, step: ProgramStep) -> ProgramStep {
    ProgramStep {
        active: Some(active.to_string()),
        ..step
    }
}

/// Two frames, the first shifted by a step `s` that runs where `active`
/// says, and one halfway between them.
fn shifted(active: Option<&str>) -> Program {
    let shift = shift_frame("s", "a_1", 2.0);
    Program::from_steps(vec![
        number("on", "on", 1.0),
        add_two_frame("a", [0.0; 3], [0.0, 0.0, 2.0]),
        match active {
            Some(active) => when(active, shift),
            None => shift,
        },
        middle_frame("m", "a_1", "a_2"),
    ])
}

/// A step that does not run makes nothing: the frame it would shift is as
/// the step before left it, and the step after it reads that. Running it
/// again makes what it makes and what reads that, nothing else.
#[test]
fn a_suppressed_step_leaves_the_part_as_the_steps_before_it() {
    let mut runner = runner();
    ok(runner.run(&shifted(None), &inputs(&[]), &no_state()));
    crate::test::assert_encloses(frame(&runner, "m"), [1.0, 0.0, 1.0]);
    take_built();

    ok(runner.run(&shifted(Some("0")), &inputs(&[]), &no_state()));
    assert_eq!(runner.part.suppressed(), ["s"]);
    assert_eq!(take_built(), ["m@m"]);
    crate::test::assert_encloses(frame(&runner, "m"), [0.0, 0.0, 1.0]);

    ok(runner.run(&shifted(None), &inputs(&[]), &no_state()));
    assert!(runner.part.suppressed().is_empty());
    assert_eq!(take_built(), ["a_1@s", "m@m"]);
    crate::test::assert_encloses(frame(&runner, "m"), [1.0, 0.0, 1.0]);
}

/// Whether a step runs is a formula of the parameters: it runs or not as
/// they change, and a parameter it does not read changes nothing.
#[test]
fn a_parameter_suppresses_a_step() {
    let mut runner = runner();
    let program = shifted(Some("on > 0"));
    ok(runner.run(&program, &inputs(&[]), &no_state()));
    assert!(runner.part.suppressed().is_empty());
    crate::test::assert_encloses(frame(&runner, "m"), [1.0, 0.0, 1.0]);
    take_built();

    ok(runner.run(&program, &inputs(&[("on", 0.0)]), &no_state()));
    assert_eq!(runner.part.suppressed(), ["s"]);
    crate::test::assert_encloses(frame(&runner, "m"), [0.0, 0.0, 1.0]);
    take_built();

    ok(runner.run(&program, &inputs(&[("on", 0.0), ("x", 3.0)]), &no_state()));
    assert_eq!(take_built(), Vec::<String>::new());
}

/// A step that reads what a suppressed step would have made fails, naming
/// that step.
#[test]
fn reading_what_a_suppressed_step_made_names_it() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        add_two_frame("a", [0.0; 3], [2.0, 0.0, 0.0]),
        when("0", middle_frame("m", "a_1", "a_2")),
        middle_frame("after", "m", "a_1"),
    ]);
    let output = runner.run(&program, &inputs(&[]), &no_state());
    let failed = errors(&output);
    assert_eq!(failed.len(), 1, "{failed:?}");
    assert_eq!(failed[0].0, "after");
    assert!(
        failed[0].1.contains("suppressed before here: `m`"),
        "{failed:?}"
    );
    let (_, error) = &output.step_errors[0];
    assert!(error.to_string().contains("built with `m` suppressed"));
}

/// A formula that does not come to a number fails the step, naming the
/// formula, and the step makes nothing.
#[test]
fn a_formula_that_comes_to_nothing_fails_the_step() {
    let mut runner = runner();
    let output = runner.run(&shifted(Some("nope + 1")), &inputs(&[]), &no_state());
    let failed = errors(&output);
    assert_eq!(failed.len(), 1, "{failed:?}");
    assert_eq!(failed[0].0, "s");
    assert!(failed[0].1.contains("nope"), "{failed:?}");
    let (_, error) = &output.step_errors[0];
    assert!(error.to_string().contains("whether the step runs"));
}

/// Whether a step runs is saved with it, and read back; a step that always
/// runs saves nothing of it.
#[test]
fn whether_a_step_runs_is_saved_with_it() {
    let program = shifted(Some("on"));
    let json = serde_json::to_value(&program).unwrap();
    assert_eq!(json["steps"][2]["active"], "on");
    assert!(json["steps"][1].get("active").is_none());
    assert_round_trips(Rc::new(registry()), None, &program);
}

/// Suppressing a step and running it again builds as building from
/// scratch does.
#[test]
fn suppressing_builds_as_from_scratch() {
    let programs = [
        shifted(None),
        shifted(Some("0")),
        shifted(Some("on")),
        shifted(None),
    ];
    assert_incremental(Rc::new(registry()), None, &programs);
}

/// What another process kept of a program with a step suppressed is read
/// back, and is not taken for the program with it running: the frame read
/// after it is made of what the steps before it made.
#[test]
fn what_is_kept_says_which_steps_ran() {
    let file = "part.geop";
    let files =
        |program: &Program| BTreeMap::from([(file.to_string(), program.to_json().unwrap())]);
    let workspace = |files: BTreeMap<String, String>| {
        move |cache| {
            Files::with_cache(files.clone(), Rc::new(registry()), cache) as Rc<dyn Workspace>
        }
    };
    let [first, _] = assert_reopens(
        Rc::new(registry()),
        workspace(files(&shifted(Some("0")))),
        file,
    );
    assert!(first.kept_makes > 0);

    // One shelf, two programs: the frame made with `s` suppressed is not
    // the one made with it running.
    let shelf = Shelf::of("1");
    let process = |program: &Program| {
        let workspace = Files::with_cache(files(program), Rc::new(registry()), shelf.cache());
        let location = Location {
            file: file.to_string(),
            within: Vec::new(),
        };
        let mut running = Running {
            runner: Runner::of(Rc::new(registry()), Some(workspace.clone()), location),
            part: Part::default(),
        };
        let program = workspace.program(file).unwrap();
        ok(running.run(&program, &inputs(&[]), &no_state()));
        workspace.cache().unwrap().flush();
        (running.runner.counts(), frame(&running, "m"))
    };
    let (_, running) = process(&shifted(None));
    let (suppressed, without) = process(&shifted(Some("0")));
    assert!(suppressed.kept_makes > 0, "{suppressed:?}");
    assert!(!running.could_be_equal(&without));
    let (again, with) = process(&shifted(None));
    assert_eq!(again.kept_makes, 0, "{again:?}");
    assert_eq!(with, running);
}
