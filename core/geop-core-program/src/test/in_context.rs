//! Parts placed reading the part placing them, in context
//! ([`crate::placed::OUTSIDE`]): a part reads a part placed beside it,
//! before it, and is built again when that one changes — and only then;
//! one placed before what it reads is refused, by name; and what it read
//! there is what a build of it is shared and kept by.

use std::{collections::BTreeMap, rc::Rc};

use crate::{
    part::Part,
    program::{Program, ProgramStep},
    runner::Runner,
    test::{
        Running, add_two_frame, assert_encloses, errors, inputs,
        instances::{load, place_file, write},
        middle_frame, no_state, ok, registry, runner, take_built,
    },
    testing::{Shelf, assert_incremental, assert_reopens},
    workspace::{Files, Location, Workspace},
};

/// The frame halfway between the two frames of the part placed beside it
/// as `bracket`, read in context.
fn between_the_brackets_frames(id: &str) -> ProgramStep {
    middle_frame(id, "/../bracket:a_1", "/../bracket:a_2")
}

/// An assembly placing a bracket of two frames at `x` and `x + 2`, then
/// the part `p` reading it.
fn assembly(x: f64) -> Program {
    let bracket = add_two_frame("a", [x, 0.0, 0.0], [x + 2.0, 0.0, 0.0]);
    let file = write(&format!("bracket_at_{x}"), vec![bracket]);
    Program::from_steps(vec![
        place_file("bracket", &file),
        load("p", vec![between_the_brackets_frames("m")]),
    ])
}

/// The frame `name` of the part placed as `instance`.
fn inside(runner: &Running, instance: &str, name: &str) -> geop_core_math::vector::Vector3 {
    let reader = runner.part.current().placed_part(instance).unwrap();
    let frame = reader
        .retrieve_element(&crate::test::whole_frame(name))
        .unwrap();
    frame.data
}

/// A part reads the part placed beside it before it, in context, and is
/// built again when that one changes: only what read it is made again.
#[test]
fn a_part_reads_a_part_placed_beside_it() {
    let mut runner = runner();
    ok(runner.run(&assembly(0.0), &inputs(&[]), &no_state()));
    assert_encloses(inside(&runner, "p", "m"), [1.0, 0.0, 0.0]);
    take_built();

    ok(runner.run(&assembly(0.0), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), Vec::<String>::new(), "nothing changed");

    ok(runner.run(&assembly(4.0), &inputs(&[]), &no_state()));
    assert_eq!(take_built(), ["a_1@a", "a_2@a", "m@m"]);
    assert_encloses(inside(&runner, "p", "m"), [5.0, 0.0, 0.0]);
}

/// A part placed before the part it reads, or reading what is not placed
/// beside it, is refused, naming both; a part built on its own, placed in
/// none, says it reads in context.
#[test]
fn a_part_reads_only_what_is_placed_before_it() {
    let mut runner = runner();
    let bracket = add_two_frame("a", [0.0; 3], [2.0, 0.0, 0.0]);
    let file = write("bracket_later", vec![bracket]);
    let swapped = Program::from_steps(vec![
        load("p", vec![between_the_brackets_frames("m")]),
        place_file("bracket", &file),
    ]);
    let output = runner.run(&swapped, &inputs(&[]), &no_state());
    let failed = errors(&output);
    assert_eq!(failed.len(), 1, "{failed:?}");
    let error = output.step_errors[0].1.to_string();
    assert!(
        error.contains("`p` reads `bracket` placed beside it in context")
            && error.contains("placed after `p`"),
        "{error}"
    );

    let mut alone = runner;
    let output = alone.run(
        &Program::from_steps(vec![between_the_brackets_frames("m")]),
        &inputs(&[]),
        &no_state(),
    );
    let error = output.step_errors[0].1.to_string();
    assert!(error.contains("built on its own"), "{error}");
}

/// Two parts of one file reading the same beside them are one build;
/// building one again from scratch builds the same, however the part read
/// changes.
#[test]
fn a_part_read_in_context_builds_as_from_scratch() {
    let mut runner = runner();
    let twice = |x: f64| {
        let mut program = assembly(x);
        program
            .steps
            .push(load("p", vec![between_the_brackets_frames("m")]));
        program.steps[2].id = "q".into();
        program
    };
    ok(runner.run(&twice(0.0), &inputs(&[]), &no_state()));
    assert_eq!(runner.runner.cached_builds(), 2, "the bracket, and p once");
    assert_encloses(inside(&runner, "q", "m"), [1.0, 0.0, 0.0]);

    let registry = Rc::new(registry());
    let workspace: Rc<dyn Workspace> = Rc::new(crate::test::instances::Files);
    assert_incremental(
        registry,
        Some((workspace, "")),
        &[assembly(0.0), assembly(4.0), twice(4.0), assembly(0.0)],
    );
}

/// The files of an assembly placing a bracket of frames at `x` and
/// `x + 2`, then the part `p` reading it, in a workspace of texts.
fn files(x: f64) -> BTreeMap<String, String> {
    let program = |steps: Vec<ProgramStep>| Program::from_steps(steps).to_json().unwrap();
    BTreeMap::from([
        (
            "asm.geop".to_string(),
            program(vec![
                place_file("bracket", "bracket.geop"),
                place_file("p", "p.geop"),
            ]),
        ),
        (
            "bracket.geop".to_string(),
            program(vec![add_two_frame("a", [x, 0.0, 0.0], [x + 2.0, 0.0, 0.0])]),
        ),
        (
            "p.geop".to_string(),
            program(vec![between_the_brackets_frames("m")]),
        ),
    ])
}

/// What an earlier process kept of a part read in context is read back
/// where what it read reads the same, and made again where it does not:
/// the bracket moved, the frame between its frames is not the one kept.
#[test]
fn what_is_kept_of_a_part_read_in_context_says_what_it_read() {
    let registry = || Rc::new(registry());
    let workspace = |files: BTreeMap<String, String>| {
        move |cache| Files::with_cache(files.clone(), registry(), cache) as Rc<dyn Workspace>
    };
    let [first, again] = assert_reopens(registry(), workspace(files(0.0)), "asm.geop");
    assert!(first.kept_makes > 0);
    assert_eq!(again.kept_makes, 0);

    let shelf = Shelf::of("1");
    let process = |x: f64| {
        let workspace = Files::with_cache(files(x), registry(), shelf.cache());
        let location = Location {
            file: "asm.geop".to_string(),
            within: Vec::new(),
        };
        let mut running = Running {
            runner: Runner::of(registry(), Some(workspace.clone()), location),
            part: Part::default(),
        };
        let program = workspace.program("asm.geop").unwrap();
        ok(running.run(&program, &inputs(&[]), &no_state()));
        workspace.cache().unwrap().flush();
        (running.runner.counts(), inside(&running, "p", "m"))
    };
    let (_, at_one) = process(0.0);
    let (moved, at_five) = process(4.0);
    assert!(moved.kept_makes > 0, "{moved:?}");
    assert_encloses(at_one, [1.0, 0.0, 0.0]);
    assert_encloses(at_five, [5.0, 0.0, 0.0]);
    let (back, again) = process(0.0);
    assert_eq!(back.kept_makes, 0, "{back:?}");
    assert_eq!(again, at_one);
}
