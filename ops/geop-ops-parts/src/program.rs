//! [`Program`]: an ordered list of operations that builds a [`Part`]; the
//! edits it can undergo ([`ProgramEdit`]); and [`ProgramRunner`], which
//! builds it incrementally.
//!
//! Editing lives here, not in any editor, so that every editor — the
//! browser UI, a future desktop one, a script — changes programs the same
//! way and is only a more convenient way of writing them.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_part::{Part, validate_operation_id};
use serde::{Deserialize, Serialize};

use crate::operation::{Handle, PartOperation};

/// One step of a [`Program`]: an operation with its arguments, and the id
/// everything it creates is named after. Serializes as
/// `{"id": "box", "operation": "extrude", "args": {...}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Step {
    pub id: String,
    #[serde(flatten)]
    pub operation: PartOperation,
}

/// A recipe for building a [`Part`]: an ordered list of steps, each referring
/// to what earlier ones built only by name. Those names come from step ids
/// and sketch element ids, never from the internal ids a build happens to
/// assign (see `geop_core_part`), so a program means the same thing every
/// time it is run — including after a round trip through JSON.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Program {
    pub steps: Vec<Step>,
}

/// A change to a [`Program`]. Every edit of a program — whoever makes it —
/// is one of these, applied by [`Program::update`].
///
/// Steps are addressed by id, not position, so an edit means the same thing
/// however the steps around it have moved. Serializes as, e.g.,
/// `{"edit": "update", "id": "box", "operation": "extrude", "args": {...}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "edit", rename_all = "snake_case")]
pub enum ProgramEdit {
    /// Insert `operation` as a new step at position `index` (the end, if it
    /// is the number of steps), with the id `id` — or, if that is `None`, a
    /// fresh one derived from the operation (see [`Program::fresh_id`]).
    Insert {
        index: usize,
        #[serde(default)]
        id: Option<String>,
        #[serde(flatten)]
        operation: PartOperation,
    },
    /// Give step `id` a new operation or new arguments, in place.
    Update {
        id: String,
        #[serde(flatten)]
        operation: PartOperation,
    },
    /// Remove step `id`. Steps that referred to what it built fail from then
    /// on, until they are edited — the program is left as the user made it.
    Remove { id: String },
    /// Move step `id` to position `index` among the remaining steps.
    Move { id: String, index: usize },
    /// Replace the whole program, e.g. with one loaded from a file.
    Replace { program: Program },
}

impl ProgramEdit {
    /// What the edit does, in a few words — without the arguments, which
    /// can be a whole sketch.
    pub fn summary(&self) -> String {
        match self {
            ProgramEdit::Insert {
                index, operation, ..
            } => format!("insert a {} step at {index}", operation.kind()),
            ProgramEdit::Update { id, operation } => {
                format!("update step {id:?} to a {} step", operation.kind())
            }
            ProgramEdit::Remove { id } => format!("remove step {id:?}"),
            ProgramEdit::Move { id, index } => format!("move step {id:?} to {index}"),
            ProgramEdit::Replace { program } => {
                format!(
                    "replace the program by one of {} steps",
                    program.steps.len()
                )
            }
        }
    }
}

impl Program {
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends the step `id`: `operation` with its arguments.
    pub fn push(&mut self, id: impl Into<String>, operation: impl Into<PartOperation>) {
        self.steps.push(Step {
            id: id.into(),
            operation: operation.into(),
        });
    }

    /// The position of step `id`.
    pub fn index_of(&self, id: &str) -> GeopResult<usize> {
        self.steps
            .iter()
            .position(|s| s.id == id)
            .ok_or_else(|| GeopError::new(format!("program has no step {id:?}")))
    }

    /// An id no step has yet, for a new step running `operation`: its
    /// label, lowercased, and the lowest number that makes it unique —
    /// `sketch1`, `extrude2`.
    pub fn fresh_id(&self, operation: &PartOperation) -> String {
        let base = operation.label().to_lowercase().replace(' ', "_");
        (1..)
            .map(|n| format!("{base}{n}"))
            .find(|id| self.steps.iter().all(|s| &s.id != id))
            .expect("some number is free")
    }

    /// Checks that every step id is a valid operation id and unique: every
    /// name a step creates is built from its id.
    pub fn validate(&self) -> GeopResult<()> {
        let mut ids = HashSet::new();
        for step in &self.steps {
            validate_operation_id(&step.id)?;
            if !ids.insert(step.id.as_str()) {
                return Err(GeopError::new(format!(
                    "program has more than one step with id {:?}",
                    step.id
                )));
            }
        }
        Ok(())
    }

    /// Applies `edit`, returning the id of the step it inserted, changed or
    /// moved (`None` for a removal or a replacement). An edit that would
    /// leave the program invalid — an unknown step, a position past the end,
    /// a duplicate or malformed id — is rejected and changes nothing.
    ///
    /// This only changes the recipe; whether the steps still build is for
    /// running it to say (see [`ProgramRunner`]).
    pub fn update(&mut self, edit: ProgramEdit) -> GeopResult<Option<String>> {
        let summary = edit.summary();
        let ctx = with_context!("Program::update({summary})");
        let mut next = self.clone();
        let changed = match edit {
            ProgramEdit::Insert {
                index,
                id,
                operation,
            } => {
                if index > next.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot insert at {index}: the program has {} steps",
                        next.steps.len()
                    )))
                    .with_context(ctx);
                }
                let id = id.unwrap_or_else(|| next.fresh_id(&operation));
                next.steps.insert(
                    index,
                    Step {
                        id: id.clone(),
                        operation,
                    },
                );
                Some(id)
            }
            ProgramEdit::Update { id, operation } => {
                let index = next.index_of(&id).with_context(ctx)?;
                next.steps[index].operation = operation;
                Some(id)
            }
            ProgramEdit::Remove { id } => {
                let index = next.index_of(&id).with_context(ctx)?;
                next.steps.remove(index);
                None
            }
            ProgramEdit::Move { id, index } => {
                let from = next.index_of(&id).with_context(ctx)?;
                let step = next.steps.remove(from);
                if index > next.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot move to {index}: the program has {} other steps",
                        next.steps.len()
                    )))
                    .with_context(ctx);
                }
                next.steps.insert(index, step);
                Some(id)
            }
            ProgramEdit::Replace { program } => {
                next = program;
                None
            }
        };
        next.validate().with_context(ctx)?;
        *self = next;
        Ok(changed)
    }

    /// Runs every step in order, starting from `part` (typically
    /// [`Part::new`]), and returns the part the whole program builds — or
    /// the first error any step raises, at which point the steps after it
    /// never run.
    ///
    /// After each step, every entity of the part must have a name — an
    /// operation that leaves one unnamed has broken the one guarantee a
    /// program relies on.
    pub fn apply<S: Scalar>(&self, part: Part<S>) -> GeopResult<Part<S>> {
        self.validate()?;
        let mut part = part;
        for (index, step) in self.steps.iter().enumerate() {
            part = run_step(part, index, step)?;
        }
        Ok(part)
    }

    /// The program as pretty-printed JSON: one step per object, every sketch
    /// entity keyed by its id, so edits show up as small line diffs.
    pub fn to_json(&self) -> GeopResult<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| GeopError::new(format!("serializing program: {e}")))
    }

    pub fn from_json(json: &str) -> GeopResult<Self> {
        let program: Self = serde_json::from_str(json)
            .map_err(|e| GeopError::new(format!("reading program: {e}")))?;
        program.validate()?;
        Ok(program)
    }
}

/// Step `index` of a program applied to `part`, with every name checked.
fn run_step<S: Scalar>(part: Part<S>, index: usize, step: &Step) -> GeopResult<Part<S>> {
    let ctx = with_context!("program step {index} ({:?})", step.id);
    let part = step.operation.apply(part, &step.id).with_context(ctx)?;
    part.check_names().with_context(ctx)?;
    Ok(part)
}

/// A handle (see [`Handle`]) of the step `step`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepHandle {
    pub step: String,
    #[serde(flatten)]
    pub handle: Handle,
}

/// How one step of a run went.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepResult {
    pub id: String,
    /// Why the step failed; `None` if it succeeded.
    pub error: Option<String>,
}

/// Builds a program the way an editor needs it built: incrementally, and
/// only as far as asked.
///
/// It keeps the part after every step it has run. Running again after an
/// edit reuses the part after the longest unchanged prefix of steps, so
/// changing the last step replays one step, not the whole history. And a
/// run can stop early — while a step in the middle is being edited, only
/// the steps up to it need to run, however long the rest of the program is.
/// Parts past the stop are kept, not discarded, so moving the stop back
/// again costs nothing.
///
/// A run stops at the first step that fails: the steps after it would only
/// fail too, for want of what it should have built.
pub struct ProgramRunner<S: Scalar> {
    /// The steps the cache was built from.
    steps: Vec<Step>,
    /// `parts[i]`: the part after `steps[..i]`. A failed step leaves the
    /// part as it was, so this stays one longer than `steps`.
    parts: Vec<Part<S>>,
    results: Vec<StepResult>,
    /// How many steps the last run covers.
    ran: usize,
}

impl<S: Scalar> ProgramRunner<S> {
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
            parts: vec![Part::new()],
            results: Vec::new(),
            ran: 0,
        }
    }

    /// Runs the first `stop` steps of `program` — all of them if `None` —
    /// reusing whatever the previous runs built that still applies. See
    /// [`ProgramRunner::part`] and [`ProgramRunner::results`] for the
    /// outcome.
    pub fn run(&mut self, program: &Program, stop: Option<usize>) {
        let common = self
            .steps
            .iter()
            .zip(&program.steps)
            .take_while(|(a, b)| a == b)
            .count();
        self.steps.truncate(common);
        self.parts.truncate(common + 1);
        self.results.truncate(common);

        let target = stop.unwrap_or(program.steps.len()).min(program.steps.len());
        let failed = |results: &[StepResult]| results.iter().any(|r| r.error.is_some());
        while self.steps.len() < target && !failed(&self.results) {
            let index = self.steps.len();
            let step = &program.steps[index];
            let before = self.parts.last().expect("parts is never empty");
            let (part, error) = match run_step(before.clone(), index, step) {
                Ok(part) => (part, None),
                Err(e) => (before.clone(), Some(e.to_string())),
            };
            self.steps.push(step.clone());
            self.parts.push(part);
            self.results.push(StepResult {
                id: step.id.clone(),
                error,
            });
        }
        // Up to the stop, or up to and including the first failure.
        let first_failure = self.results.iter().position(|r| r.error.is_some());
        self.ran = match first_failure {
            Some(f) if f < target => f + 1,
            _ => target.min(self.steps.len()),
        };
    }

    /// The part the last run built.
    pub fn part(&self) -> &Part<S> {
        &self.parts[self.ran]
    }

    /// One result per step the last run covered.
    pub fn results(&self) -> &[StepResult] {
        &self.results[..self.ran]
    }

    /// Every handle of every step the last run built — all of them, of
    /// every group: which to offer is an editor's choice. Each is placed
    /// with the part as its step saw it.
    pub fn handles(&self) -> GeopResult<Vec<StepHandle>> {
        let mut handles = Vec::new();
        for (i, (step, result)) in self.steps.iter().zip(self.results()).enumerate() {
            if result.error.is_some() {
                continue;
            }
            let ctx = with_context!("handles of step {i} ({:?})", step.id);
            for handle in step.operation.handles(&self.parts[i]).with_context(ctx)? {
                handles.push(StepHandle {
                    step: step.id.clone(),
                    handle,
                });
            }
        }
        Ok(handles)
    }
}

impl<S: Scalar> Default for ProgramRunner<S> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::scalars::ScalInF64 as S;
    use geop_core_part::PartDescription;

    use super::*;
    use crate::{ExtrudeArgs, examples::box_with_drill_hole};

    fn extrude(sketch: &str, distance: f64) -> PartOperation {
        ExtrudeArgs {
            sketch: sketch.into(),
            distance,
            symmetric: false,
            combine: crate::Combine::NewBody,
        }
        .into()
    }

    /// Every kind of edit, addressed by id, and each rejected when it would
    /// leave the program invalid — without changing anything.
    #[test]
    fn edits_change_the_program_by_id() {
        let mut program = box_with_drill_hole();
        let len = program.steps.len();

        let id = program
            .update(ProgramEdit::Insert {
                index: len,
                id: None,
                operation: extrude("outline", 2.0),
            })
            .unwrap();
        assert_eq!(id.as_deref(), Some("extrude1"));
        assert_eq!(program.steps[len].id, "extrude1");

        program
            .update(ProgramEdit::Update {
                id: "extrude1".into(),
                operation: extrude("outline", 3.0),
            })
            .unwrap();
        assert_eq!(program.steps[len].operation, extrude("outline", 3.0));

        program
            .update(ProgramEdit::Move {
                id: "extrude1".into(),
                index: 0,
            })
            .unwrap();
        assert_eq!(program.steps[0].id, "extrude1");

        program
            .update(ProgramEdit::Remove {
                id: "extrude1".into(),
            })
            .unwrap();
        assert_eq!(program, box_with_drill_hole());

        let before = program.clone();
        for bad in [
            ProgramEdit::Insert {
                index: len + 1,
                id: None,
                operation: extrude("outline", 1.0),
            },
            ProgramEdit::Insert {
                index: 0,
                id: Some("box".into()),
                operation: extrude("outline", 1.0),
            },
            ProgramEdit::Insert {
                index: 0,
                id: Some("not an id".into()),
                operation: extrude("outline", 1.0),
            },
            ProgramEdit::Remove { id: "nope".into() },
            ProgramEdit::Move {
                id: "box".into(),
                index: len,
            },
        ] {
            assert!(program.update(bad.clone()).is_err(), "{bad:?}");
            assert_eq!(program, before, "a rejected {bad:?} changed the program");
        }
    }

    /// An edit serializes as one flat JSON object, as an editor sends it.
    #[test]
    fn edits_read_from_json() {
        let edit: ProgramEdit = serde_json::from_str(
            r#"{"edit": "insert", "index": 0, "operation": "extrude",
                "args": {"sketch": "outline", "distance": 2.0}}"#,
        )
        .unwrap();
        assert_eq!(
            edit,
            ProgramEdit::Insert {
                index: 0,
                id: None,
                operation: extrude("outline", 2.0),
            }
        );
    }

    /// A runner builds what [`Program::apply`] builds; stopping early builds
    /// just the steps before the stop; and a run after an edit starts from
    /// the last unchanged step.
    #[test]
    fn runner_stops_early_and_reuses_the_unchanged_prefix() {
        let program = box_with_drill_hole();
        let describe = |part: &Part<S>| PartDescription::of(part).unwrap();
        let mut runner = ProgramRunner::<S>::new();

        runner.run(&program, None);
        assert!(runner.results().iter().all(|r| r.error.is_none()));
        assert_eq!(
            describe(runner.part()),
            describe(&program.apply(Part::new()).unwrap())
        );

        // Back in time: only the box.
        runner.run(&program, Some(2));
        assert_eq!(runner.results().len(), 2);
        let description = describe(runner.part());
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(box)"]
        );

        // An edit to the hole keeps the box's part: the first two steps are
        // served from the cache, which has to hold exactly what they built.
        let mut edited = program.clone();
        edited
            .update(ProgramEdit::Update {
                id: "hole".into(),
                operation: extrude("hole_sketch", -0.25),
            })
            .unwrap();
        runner.run(&edited, None);
        assert!(runner.results().iter().all(|r| r.error.is_none()));
        assert_eq!(
            describe(runner.part()),
            describe(&edited.apply(Part::new()).unwrap())
        );
    }

    /// Every step's handles, placed where the part as that step saw it puts
    /// them: the box's distance at its top, the hole's at its bottom, and a
    /// handle for every sketch point.
    #[test]
    fn runner_provides_every_handle() {
        use crate::operation::{HandleGroup, HandleMotion};
        let mut runner = ProgramRunner::<S>::new();
        runner.run(&box_with_drill_hole(), None);
        let handles = runner.handles().unwrap();
        let feature: Vec<&StepHandle> = handles
            .iter()
            .filter(|h| h.handle.group == HandleGroup::Feature)
            .collect();
        assert_eq!(feature.len(), 2);
        let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
        let (boxed, hole) = (feature[0], feature[1]);
        assert_eq!(boxed.step, "box");
        assert!(close(boxed.handle.position, [1.0, 1.0, 1.0]), "{boxed:?}");
        assert_eq!(hole.step, "hole");
        assert!(close(hole.handle.position, [1.0, 1.0, 0.5]), "{hole:?}");
        let HandleMotion::Linear {
            direction,
            arg,
            value,
            scale,
        } = &hole.handle.motion
        else {
            panic!("{hole:?}")
        };
        assert!(close(*direction, [0.0, 0.0, 1.0]));
        assert_eq!(arg, &["distance"]);
        assert_eq!((*value, *scale), (-0.5, 1.0));
        // 4 corners of the outline, the circle's center.
        let sketch = handles.len() - feature.len();
        assert_eq!(sketch, 5);
        let json = serde_json::to_value(&handles[0]).unwrap();
        assert_eq!(json["step"], "outline");
        assert_eq!(json["motion"], "planar");
    }

    /// A step that fails ends the run there, reported by id; the part is
    /// what the steps before it built.
    #[test]
    fn runner_reports_the_failing_step() {
        let mut program = box_with_drill_hole();
        program
            .update(ProgramEdit::Update {
                id: "hole".into(),
                operation: ExtrudeArgs {
                    sketch: "hole_sketch".into(),
                    distance: -0.5,
                    symmetric: false,
                    combine: crate::Combine::Difference {
                        target: "extrude(nothing)".into(),
                    },
                }
                .into(),
            })
            .unwrap();
        let mut runner = ProgramRunner::<S>::new();
        runner.run(&program, None);
        let last = runner.results().last().unwrap();
        assert_eq!(last.id, "hole");
        assert!(last.error.as_deref().unwrap().contains("extrude(nothing)"));
        assert!(runner.part().solid_id("extrude(box)").is_ok());
    }
}
