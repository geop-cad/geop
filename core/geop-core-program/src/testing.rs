//! What tests of operations need of a program (feature `testing`).
#![allow(clippy::panic, reason = "a test fails by panicking")]

use std::rc::Rc;

use crate::{
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    program_runner::ProgramRunner,
    values::Settings,
    workspace::{Location, Workspace},
};

/// Runs `programs` one after another on one part, as an editor runs a
/// program edited again and again, and each on a fresh part, from scratch;
/// and asserts that after each the two parts hold equal versions of every
/// target (see [`Part::differences`]), and that their steps fail alike. A
/// version reused for reading what did not change would otherwise show
/// here as a target that differs. The programs are run in the operations
/// of `registry`, as the file `file` of `workspace`, if one is given.
#[track_caller]
pub fn assert_incremental(
    registry: Rc<OperationRegistry>,
    workspace: Option<(Rc<dyn Workspace>, &str)>,
    programs: &[Program],
) {
    let runner = ProgramRunner {
        registry,
        location: Location {
            file: workspace
                .as_ref()
                .map(|(_, file)| file.to_string())
                .unwrap_or_default(),
            within: Vec::new(),
        },
        workspace: workspace.map(|(workspace, _)| workspace),
    };
    let errors = |output: crate::program_runner::RunOutput| -> Vec<(String, String)> {
        (output.step_errors.into_iter())
            .map(|(step, e)| (step, e.to_string()))
            .collect()
    };
    let file = runner.location.file.clone();
    let mut incremental = Part::default();
    for (k, program) in programs.iter().enumerate() {
        let (inputs, state) = (Settings::new(), program.state.clone());
        let ran = errors(runner.run(&mut incremental, program, &inputs, &state));
        let mut scratch = Part::default();
        let built = errors(runner.run(&mut scratch, program, &inputs, &state));
        assert_eq!(
            ran, built,
            "program {k}: its steps fail otherwise when run again"
        );
        let differences = incremental.differences(&scratch);
        assert!(
            differences.is_empty(),
            "{file:?}, program {k}: run again, it builds otherwise than from scratch: \
             {differences:?}"
        );
    }
}
