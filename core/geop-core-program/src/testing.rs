//! What tests of operations need of a program (feature `testing`).
#![allow(clippy::panic, reason = "a test fails by panicking")]

use std::rc::Rc;

use crate::{
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    runner::{RunOutput, Runner},
    workspace::{Location, Workspace},
};

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
