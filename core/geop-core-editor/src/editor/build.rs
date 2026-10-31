//! [`Build`]: what an editor built of a program, and how its last run went.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::GeopError;
use geop_core_program::{
    operation_registry::OperationRegistry, part::Part, program::Program, reader::Reader,
    runner::Runner, workspace::Location,
};

/// What the editor built: the runner, holding every step's part, and how
/// its last run went.
pub(super) struct Build {
    pub(super) runner: Runner,
    /// The part, as the last run left it.
    pub(super) part: Part,
    /// How many steps the last run ran.
    pub(super) ran: usize,
    /// Why each step of the last run that failed did, by its id.
    pub(super) errors: BTreeMap<String, GeopError>,
    /// Of those failed because of one of their arguments, which, and what
    /// went wrong with it, by the step's id (see
    /// [`geop_core_math::geop_error::GeopError::concerning`]).
    pub(super) arguments: BTreeMap<String, (String, String)>,
}

impl Build {
    /// Nothing built yet, by a runner of `operations`.
    pub(super) fn new(operations: &Rc<OperationRegistry>) -> Self {
        Self {
            runner: Runner::of(operations.clone(), None, Location::default()),
            part: Part::default(),
            ran: 0,
            errors: BTreeMap::new(),
            arguments: BTreeMap::new(),
        }
    }

    /// Runs the first `end` of `steps` — all of them, if `None` — with
    /// their state, keeping how far it ran and why steps failed. The nodes
    /// of the steps after are kept, to be reused when they run again.
    pub(super) fn run(&mut self, steps: &Program, end: Option<usize>) {
        let end = end.unwrap_or(steps.steps.len()).min(steps.steps.len());
        let upto = steps.steps.get(end).map(|step| step.id.as_str());
        let output = (self.runner).run(&mut self.part, steps, upto);
        self.ran = end;
        self.arguments = (output.step_errors.iter())
            .filter_map(|(step, error)| {
                let key = error.argument()?.to_string();
                Some((step.clone(), (key, error.root_message().to_string())))
            })
            .collect();
        self.errors = output.step_errors.into_iter().collect();
    }

    /// The part before the step at `index` of `steps`, as the last run left
    /// it — or, at or past how far it ran, the part it built.
    pub(super) fn view_at(&self, steps: &Program, index: usize) -> Reader<'_> {
        let part = &self.part;
        match steps.steps.get(index).filter(|_| index < self.ran) {
            Some(step) => part.before(&step.id),
            None => part.current(),
        }
    }

    /// Why the step `id`, at `index`, does not build, if it does not.
    pub(super) fn step_error(&self, index: usize, id: &str) -> Option<GeopError> {
        if let Some(error) = self.errors.get(id) {
            return Some(error.clone());
        }
        if index < self.ran {
            return None;
        }
        Some(match self.errors.iter().next() {
            Some((step, error)) => {
                (error.clone()).with_context(format!("an earlier step, `{step}`, failed"))
            }
            None => GeopError::new("it did not run"),
        })
    }
}
