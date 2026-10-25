//! What tests of operations need of a program (feature `testing`): its
//! steps and their arguments as their types, a runner of it, and that it
//! builds the same incrementally and after a round trip through its file.
#![allow(
    clippy::panic,
    clippy::expect_used,
    clippy::indexing_slicing,
    reason = "a test fails by panicking"
)]

use std::rc::Rc;

use serde::{Serialize, de::DeserializeOwned};

use crate::{
    operation::StepArgs,
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    runner::{RunOutput, Runner},
    workspace::{Location, Workspace},
};

// ── Steps and their arguments ───────────────────────────────────────────────

/// The step `id`, `args` for the operation they are for.
pub fn as_step<A: StepArgs>(id: &str, args: A) -> ProgramStep {
    ProgramStep::of(id, &args).expect("a step's arguments are plain data")
}

/// The step `id` of `program`.
#[track_caller]
pub fn step<'a>(program: &'a mut Program, id: &str) -> &'a mut ProgramStep {
    let index = program.index_of(id).unwrap_or_else(|e| panic!("{e}"));
    &mut program.steps[index]
}

/// The arguments of `step`, as their type `A`.
#[track_caller]
pub fn args<A: DeserializeOwned>(step: &ProgramStep) -> A {
    serde_json::from_value(step.args.clone())
        .unwrap_or_else(|e| panic!("step {:?} holds no such arguments: {e}", step.id))
}

/// Changes the arguments of `step`, as their type `A`, by `edit`.
pub fn edit_args<A: Serialize + DeserializeOwned>(
    step: &mut ProgramStep,
    edit: impl FnOnce(&mut A),
) {
    let mut args: A = args(step);
    edit(&mut args);
    step.args = serde_json::to_value(args).expect("arguments are plain data");
}

// ── Running programs ────────────────────────────────────────────────────────

/// A runner of the operations of `registry`, running the file `file` of
/// `workspace`, if one is given.
pub fn runner(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
) -> Runner {
    let location = Location {
        file: (workspace.as_ref()).map_or_else(String::new, |(_, file)| file.to_string()),
        within: Vec::new(),
    };
    Runner::of(
        registry,
        workspace.map(|(workspace, _)| workspace),
        location,
    )
}

/// The failed steps of `output`, and why.
fn errors(output: RunOutput) -> Vec<(String, String)> {
    (output.step_errors.into_iter())
        .map(|(step, e)| (step, e.to_string()))
        .collect()
}

/// Runs `programs` one after another on one part, as an editor runs a
/// program edited again and again, and each on a fresh part by a fresh
/// runner, from scratch — its placed parts built anew, not taken from the
/// first runner's build cache; and asserts that after each the two parts
/// hold equal nodes of every target (see [`Part::differences`]), and that
/// their steps fail alike. A node reused for reading what did not change
/// would otherwise show here as a target that differs. The programs are
/// run in the operations of `registry`, as the file `file` of `workspace`,
/// if one is given.
#[track_caller]
pub fn assert_incremental(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
    programs: &[Program],
) {
    let file = workspace.as_ref().map(|(_, file)| file.to_string());
    let incremental = runner(registry.clone(), workspace.clone());
    let mut part = Part::default();
    for (k, program) in programs.iter().enumerate() {
        let ran = errors(incremental.run(&mut part, program, None));
        let mut scratch = Part::default();
        let built =
            errors(runner(registry.clone(), workspace.clone()).run(&mut scratch, program, None));
        assert_eq!(
            ran, built,
            "{file:?}, program {k}: its steps fail otherwise when run again"
        );
        let differences = part.differences(&scratch);
        assert!(
            differences.is_empty(),
            "{file:?}, program {k}: run again, it builds otherwise than from scratch: \
             {differences:?}"
        );
    }
}

/// Builds `program`, every step without an error, saves it, loads it back
/// — the same program, saved the same again — and builds what it loaded,
/// by a fresh runner: asserts that the two parts hold equal nodes of every
/// target (see [`Part::differences`]), and returns the first. The program
/// is run in the operations of `registry`, as the file `file` of
/// `workspace`, if one is given.
#[track_caller]
pub fn assert_round_trips(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
    program: &Program,
) -> Part {
    let file = workspace.as_ref().map(|(_, file)| file.to_string());
    let build = |program: &Program| {
        let mut part = Part::default();
        let failed =
            errors(runner(registry.clone(), workspace.clone()).run(&mut part, program, None));
        assert!(failed.is_empty(), "{file:?}: steps fail: {failed:?}");
        part
    };
    let built = build(program);
    let json = program
        .to_json()
        .unwrap_or_else(|e| panic!("{file:?}: not saved: {e}"));
    let loaded = Program::from_json(&json).unwrap_or_else(|e| panic!("{file:?}: not loaded: {e}"));
    assert_eq!(&loaded, program, "{file:?}: loaded, it is another program");
    let again = loaded
        .to_json()
        .unwrap_or_else(|e| panic!("{file:?}: not saved again: {e}"));
    assert_eq!(again, json, "{file:?}: saved again, it reads otherwise");
    let differences = built.differences(&build(&loaded));
    assert!(
        differences.is_empty(),
        "{file:?}: loaded, it builds otherwise: {differences:?}"
    );
    built
}

/// Edits of `original` to run [`assert_incremental`] through, as a user
/// makes them: as it is, its last step dropped, put back, its first two
/// steps swapped — a failure, where the second reads the first — and put
/// back.
pub fn edit_sequence(original: &Program) -> Vec<Program> {
    let mut programs = vec![original.clone()];
    if let Some(last) = original.steps.len().checked_sub(1) {
        let mut dropped = original.clone();
        dropped.steps.remove(last);
        programs.extend([dropped, original.clone()]);
    }
    if original.steps.len() >= 2 {
        let mut swapped = original.clone();
        swapped.steps.swap(0, 1);
        programs.extend([swapped, original.clone()]);
    }
    programs
}
