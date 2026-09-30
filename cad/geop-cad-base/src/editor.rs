//! [`Editor`]: the whole of editing a program, as one state machine an
//! editor drives with [`Command`]s and draws from the [`Update`]s it
//! answers.
//!
//! It holds the program, its undo and redo, how far it runs, and the step
//! being edited, and it builds whatever is shown — so an editor keeps no
//! state of its own beyond the camera, renders what it is sent, and sends
//! what the user did. What each step shows and does is the operation's; see
//! [`geop_ops::ui::StepEditor`].

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_ops::{
    EntityRef, OperationInfo, Operations, Step, StepResult,
    ui::{PartView, Presentation, StepEditEvent, StepEditor, Target},
};
use serde::{Deserialize, Serialize};

use crate::{PartOperation, Program, ProgramRunner, examples};

/// Something the user did.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Command<S: Scalar> {
    /// Nothing: only what to show, all of it — how an editor starts.
    Show,
    /// Start a new step of the operation `kind`, where the program runs to.
    New {
        kind: String,
    },
    /// Start editing the step `id`.
    Open {
        id: String,
    },
    /// Something the user did to the step being edited.
    Event {
        event: StepEditEvent<S>,
    },
    /// Put the step being edited into the program — refused, unless it
    /// builds.
    Commit,
    /// Stop editing the step, leaving the program as it was.
    Cancel,
    /// Whether the step being edited is shown as it builds, or the part
    /// before it.
    Preview {
        preview: bool,
    },
    Remove {
        id: String,
    },
    /// Move the step `id` to `index` among the other steps.
    Move {
        id: String,
        index: usize,
    },
    /// Run only the first `marker` steps — all of them, if `None` — and put
    /// new steps there.
    Seek {
        marker: Option<usize>,
    },
    /// Replace the program, e.g. with one read from a file.
    Load {
        program: Program,
    },
    LoadExample {
        name: String,
    },
    Undo,
    Redo,
}

/// A step of the program, as a list of steps shows it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepInfo {
    pub id: String,
    pub kind: &'static str,
    pub label: &'static str,
    /// Its fields in one line: `sketch=outline, distance=1.00`.
    pub summary: String,
    /// Why it failed, if it ran and did.
    pub error: Option<String>,
    /// Whether it runs: it is before where the program runs to.
    pub runs: bool,
    /// Whether it is the step being edited.
    pub editing: bool,
}

/// The program, as shown.
#[derive(Clone, Debug, Serialize)]
pub struct ProgramState {
    /// The program itself, to save.
    pub program: Program,
    pub steps: Vec<StepInfo>,
    /// How many steps run: new steps go there.
    pub marker: usize,
    pub can_undo: bool,
    pub can_redo: bool,
    /// Every operation a step can be.
    pub operations: Vec<OperationInfo>,
    /// The names of the example programs [`Command::LoadExample`] loads.
    pub examples: Vec<&'static str>,
}

/// What is drawn: a part, and which of its sketches and datums not to.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct SceneState<S: Scalar> {
    pub part: PartView<S>,
    /// The sketches and datums the steps shown have used: what was made
    /// from them shows them now. None of a kind while it is being picked.
    pub hidden: Vec<String>,
}

/// The step being edited, as shown.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct StepState<S: Scalar> {
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    /// Its id — none yet, for a new step.
    pub id: Option<String>,
    pub presentation: Presentation<S>,
    /// Why it does not build; only a step that builds can be committed.
    pub error: Option<String>,
    pub preview: bool,
}

/// What an editor shows after a command. What did not change since the
/// last update is left out.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Update<S: Scalar> {
    /// Why the command was refused, if it was: then it changed nothing.
    pub error: Option<String>,
    pub program: Option<ProgramState>,
    pub scene: Option<SceneState<S>>,
    /// The step being edited; `None` if there is none.
    pub step: Option<StepState<S>>,
}

/// The step being edited.
struct Open<S: Scalar> {
    /// Where it is, or goes.
    index: usize,
    /// The step it edits; `None` for a new one inserted at `index`.
    id: Option<String>,
    editor: StepEditor<PartOperation>,
    /// The part before it, as drawn: what picks test against.
    view: PartView<S>,
}

/// Which part is drawn, and without what: when it is the same as last
/// time, the scene is not sent again.
#[derive(Clone, PartialEq)]
struct Shown {
    run: u64,
    steps: usize,
    hidden: Vec<String>,
}

/// Editing a program: see the module docs.
pub struct Editor<S: Scalar> {
    program: Program,
    undo: Vec<Program>,
    redo: Vec<Program>,
    marker: Option<usize>,
    open: Option<Open<S>>,
    preview: bool,
    runner: ProgramRunner<S>,
    /// Counts runs, so a scene is resent only after one.
    run: u64,
    /// What each step that ran builds on (see [`geop_ops::ui::Dialog::picked`]).
    references: Vec<Vec<EntityRef>>,
    shown: Option<Shown>,
    /// The names of the example programs.
    examples: Vec<&'static str>,
}

impl<S: Scalar> Default for Editor<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Scalar> Editor<S> {
    pub fn new() -> Self {
        Self {
            program: Program::new(),
            undo: Vec::new(),
            redo: Vec::new(),
            marker: None,
            open: None,
            preview: true,
            runner: ProgramRunner::new(),
            run: 0,
            references: Vec::new(),
            shown: None,
            examples: examples::all().into_iter().map(|(name, _)| name).collect(),
        }
    }

    pub fn program(&self) -> &Program {
        &self.program
    }

    /// Applies `command`, and says what to show now.
    pub fn handle(&mut self, command: Command<S>) -> Update<S> {
        // What undo goes back to — not taken for what is sent as often as
        // the pointer moves, and cannot change the program.
        let before = (!matches!(command, Command::Event { .. } | Command::Preview { .. }))
            .then(|| self.program.clone());
        let result = self.apply(command);
        if matches!(result, Ok(Changed::Program))
            && let Some(before) = before
            && self.program != before
        {
            self.undo.push(before);
            self.redo.clear();
        }
        if !matches!(result, Ok(Changed::Nothing)) {
            self.rerun();
        }
        let step = self.step_state();
        let hidden = self.hidden(step.as_ref().map(|s| &s.presentation.pickable[..]));
        let shown = Shown {
            run: self.run,
            steps: self.shown_steps(step.as_ref()),
            hidden,
        };
        let scene = (self.shown.as_ref() != Some(&shown)).then(|| {
            self.shown = Some(shown.clone());
            SceneState {
                part: self.view_of(shown.steps),
                hidden: shown.hidden,
            }
        });
        Update {
            error: result.as_ref().err().map(|e| e.to_string()),
            program: matches!(result, Ok(Changed::Program | Changed::Run))
                .then(|| self.program_state()),
            scene,
            step,
        }
    }

    /// Applies `command`, saying what it changed.
    fn apply(&mut self, command: Command<S>) -> GeopResult<Changed> {
        use geop_core_math::geop_error::GeopError;
        let idle = |editor: &Self| {
            if editor.open.is_some() {
                Err(GeopError::new("finish editing the step first"))
            } else {
                Ok(())
            }
        };
        Ok(match command {
            Command::Show => {
                self.shown = None;
                Changed::Run
            }
            Command::New { kind } => {
                idle(self)?;
                let index = self.marker.unwrap_or(self.program.steps.len());
                self.runner.run(&self.program, Some(index));
                let before = self.runner.part_at(index);
                let step = PartOperation::new_step(&kind, before)?;
                self.open = Some(Open {
                    index,
                    id: None,
                    editor: StepEditor::new(step, before, true),
                    view: PartView::of(before)?,
                });
                Changed::Run
            }
            Command::Open { id } => {
                idle(self)?;
                let index = self.program.index_of(&id)?;
                self.runner.run(&self.program, Some(index));
                let before = self.runner.part_at(index);
                let step = self.program.steps[index].operation.clone();
                self.open = Some(Open {
                    index,
                    id: Some(id),
                    editor: StepEditor::new(step, before, false),
                    view: PartView::of(before)?,
                });
                Changed::Run
            }
            Command::Event { event } => {
                let Some(open) = &mut self.open else {
                    return Ok(Changed::Nothing);
                };
                let before = open.editor.step().clone();
                open.editor
                    .handle(self.runner.part_at(open.index), &open.view, &event);
                if *open.editor.step() == before {
                    Changed::Nothing
                } else {
                    Changed::Step
                }
            }
            Command::Commit => {
                let Some(open) = &self.open else {
                    return Err(GeopError::new("no step is being edited"));
                };
                if let Some(error) = self.step_error(open) {
                    return Err(GeopError::new(format!("the step does not build: {error}")));
                }
                self.program = self.with_open(open);
                if open.id.is_none()
                    && let Some(marker) = &mut self.marker
                {
                    *marker += 1;
                }
                self.open = None;
                Changed::Program
            }
            Command::Cancel => {
                self.open = None;
                Changed::Run
            }
            Command::Preview { preview } => {
                self.preview = preview;
                Changed::Nothing
            }
            Command::Remove { id } => {
                idle(self)?;
                let index = self.program.index_of(&id)?;
                self.program.steps.remove(index);
                if let Some(marker) = &mut self.marker
                    && index < *marker
                {
                    *marker -= 1;
                }
                Changed::Program
            }
            Command::Move { id, index } => {
                idle(self)?;
                let from = self.program.index_of(&id)?;
                if index >= self.program.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot move step {id:?} to {index}: the program has {} steps",
                        self.program.steps.len()
                    )));
                }
                let step = self.program.steps.remove(from);
                self.program.steps.insert(index, step);
                Changed::Program
            }
            Command::Seek { marker } => {
                idle(self)?;
                self.marker = marker.filter(|&m| m < self.program.steps.len());
                Changed::Run
            }
            Command::Load { program } => {
                idle(self)?;
                program.validate()?;
                self.program = program;
                self.marker = None;
                Changed::Program
            }
            Command::LoadExample { name } => {
                idle(self)?;
                let (_, program) = examples::all()
                    .into_iter()
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                self.program = program;
                self.marker = None;
                Changed::Program
            }
            Command::Undo | Command::Redo => {
                idle(self)?;
                let (from, to) = match command {
                    Command::Undo => (&mut self.undo, &mut self.redo),
                    _ => (&mut self.redo, &mut self.undo),
                };
                let Some(program) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push(std::mem::replace(&mut self.program, program));
                self.marker = None;
                // Undoing is not itself an edit to undo.
                Changed::Run
            }
        })
    }

    /// The program with the step being edited in it.
    fn with_open(&self, open: &Open<S>) -> Program {
        let mut program = self.program.clone();
        let operation = open.editor.step().clone();
        match &open.id {
            Some(id) => {
                program.steps[open.index] = Step {
                    id: id.clone(),
                    operation,
                }
            }
            None => {
                let id = program.fresh_id(&operation);
                program.steps.insert(open.index, Step { id, operation });
            }
        }
        program
    }

    /// Runs what is shown: the program up to and including the step being
    /// edited, or as far as it runs.
    fn rerun(&mut self) {
        match &self.open {
            Some(open) => {
                let program = self.with_open(open);
                self.runner.run(&program, Some(open.index + 1));
            }
            None => self.runner.run(&self.program, self.marker),
        }
        self.run += 1;
        let ran = self.runner.results().len();
        let steps = match &self.open {
            Some(open) => self.with_open(open).steps,
            None => self.program.steps.clone(),
        };
        self.references = steps[..ran]
            .iter()
            .enumerate()
            .map(|(i, step)| {
                let session = step.operation.new_session();
                let form = step.operation.form(self.runner.part_at(i), &*session);
                form.dialog.picked().cloned().collect()
            })
            .collect();
    }

    /// Why the step being edited does not build, if it does not.
    fn step_error(&self, open: &Open<S>) -> Option<String> {
        match self.runner.results().get(open.index) {
            Some(StepResult { error, .. }) => error.clone(),
            None => Some(
                self.runner
                    .results()
                    .iter()
                    .find_map(|r| r.error.as_ref())
                    .map_or("it did not run".into(), |e| {
                        format!("an earlier step failed: {e}")
                    }),
            ),
        }
    }

    fn step_state(&self) -> Option<StepState<S>> {
        let open = self.open.as_ref()?;
        let step = open.editor.step();
        let info = PartOperation::infos()
            .into_iter()
            .find(|i| i.kind == step.kind())?;
        Some(StepState {
            kind: info.kind,
            label: info.label,
            doc: info.doc,
            id: open.id.clone(),
            presentation: open.editor.presentation(self.runner.part_at(open.index)),
            error: self.step_error(open),
            preview: self.preview,
        })
    }

    /// How many steps the drawn part is built by: with the step being
    /// edited, if it is previewed, builds, and no plane is worked in —
    /// then what is drawn is drawn a second time, in the plane.
    fn shown_steps(&self, step: Option<&StepState<S>>) -> usize {
        match (&self.open, step) {
            (Some(open), Some(step)) => {
                let preview =
                    self.preview && step.error.is_none() && step.presentation.focus.is_none();
                open.index + usize::from(preview)
            }
            _ => self.runner.results().len(),
        }
    }

    /// What the shown steps built on — sketches, and datums as a whole —
    /// but none of a kind `picking` looks for.
    fn hidden(&self, picking: Option<&[Target]>) -> Vec<String> {
        let picking = picking.unwrap_or_default();
        let picking_sketch = picking.contains(&Target::Sketch);
        let picking_datum = picking.iter().any(|t| matches!(t, Target::Datum(_)));
        self.references
            .iter()
            .flatten()
            .filter_map(|r| match r {
                EntityRef::Sketch { name } if !picking_sketch => Some(name.clone()),
                EntityRef::Datum {
                    name,
                    component: None,
                } if !picking_datum => Some(name.clone()),
                _ => None,
            })
            .collect()
    }

    /// The part `steps` steps build, as drawn.
    fn view_of(&self, steps: usize) -> PartView<S> {
        let part = self.runner.part_at(steps);
        PartView::of(part).unwrap_or_else(|_| {
            PartView::of(&geop_ops::Part::new()).expect("an empty part can be drawn")
        })
    }

    fn program_state(&self) -> ProgramState {
        let ran = self.runner.results();
        let editing = self.open.as_ref().and_then(|o| o.id.as_deref());
        let runs_to = match &self.open {
            Some(open) => open.index,
            None => self.marker.unwrap_or(self.program.steps.len()),
        };
        let steps = self
            .program
            .steps
            .iter()
            .enumerate()
            .map(|(i, step)| {
                let session = step.operation.new_session();
                let form = step.operation.form(self.runner.part_at(i), &*session);
                StepInfo {
                    id: step.id.clone(),
                    kind: step.operation.kind(),
                    label: step.operation.label(),
                    summary: form.dialog.summary(),
                    error: ran
                        .get(i)
                        .filter(|_| i < runs_to)
                        .and_then(|r| r.error.clone()),
                    runs: i < runs_to,
                    editing: editing == Some(step.id.as_str()),
                }
            })
            .collect();
        ProgramState {
            program: self.program.clone(),
            steps,
            marker: self.marker.unwrap_or(self.program.steps.len()),
            can_undo: !self.undo.is_empty(),
            can_redo: !self.redo.is_empty(),
            operations: PartOperation::infos(),
            examples: self.examples.clone(),
        }
    }
}

/// What a command changed.
enum Changed {
    Nothing,
    /// The arguments of the step being edited.
    Step,
    /// What runs: which steps, or up to where.
    Run,
    /// The program.
    Program,
}
