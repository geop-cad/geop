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
