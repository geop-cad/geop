//! What a panel, a tool or a settler is given of the [`Editor`] it extends
//! ([`EditorContext`], [`Rerun`]), and what it says a command changed
//! ([`Changed`]).

use geop_core_program::{
    element::PickedElement, part::Part, program::Program, reader::Reader, runner::Runner,
    values::StateValues,
};

use super::Editor;
use crate::{output::Inspection, view::PartView};
use geop_core_math::geop_error::{GeopError, GeopResult};

/// What a command changed: what the editor runs, settles and shows again
/// after it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Changed {
    Nothing,
    /// The arguments of the step being edited.
    Step,
    /// What runs: which steps, or up to where — or the program's state, as
    /// a drag moves it, which is no edit to undo by itself.
    Run,
    /// The program.
    Program,
    /// How the program is shown ([`geop_core_program::program::Display`]):
    /// an edit to undo, and to save with it, that runs nothing.
    Display,
}

/// A program, its runner and the part it last ran on: what a solve runs
/// again at every state it tries, the program's state set to it.
pub struct Rerun<'r> {
    pub runner: &'r Runner,
    pub part: &'r mut Part,
    pub program: &'r Program,
}

impl Rerun<'_> {
    /// The part as the program last ran.
    pub fn reader(&self) -> Reader<'_> {
        self.part.current()
    }
}

/// The [`Editor`], as a panel's command, a tool or a panel told of an edit
/// sees it: the program and what it built, and the few ways they change
/// that the editor keeps track of — so that undo, the marker and the step
/// being edited stay right whatever the extension does.
pub struct EditorContext<'a> {
    editor: &'a mut Editor,
}

impl<'a> EditorContext<'a> {
    pub(super) fn new(editor: &'a mut Editor) -> Self {
        Self { editor }
    }

    /// The program, as saved: the file edited.
    pub fn program(&self) -> &Program {
        &self.editor.file
    }

    /// The part the program builds, as far as it last ran.
    pub fn part(&self) -> Reader<'_> {
        self.editor.part()
    }

    /// The part as last drawn: what a tool picks in.
    pub fn drawn(&self) -> Option<&PartView> {
        self.editor.drawn.as_deref()
    }

    /// How many times the editor has run the program: what the part is
    /// built as changes only when this does.
    pub fn runs(&self) -> u64 {
        self.editor.run
    }

    /// Refuses while a step is edited.
    pub fn idle(&self) -> GeopResult<()> {
        self.editor.idle()
    }

    /// Runs the whole of the program run now: the step edited's, or the
    /// file's.
    pub fn run_all(&mut self) {
        self.editor.run_to(None);
    }

    /// The program run now — the step edited's, or the file's — with the
    /// part as it last ran, to run again.
    pub fn rerun(&mut self) -> Rerun<'_> {
        let editor = &mut *self.editor;
        Rerun {
            runner: &editor.build.runner,
            part: &mut editor.build.part,
            program: match &editor.open {
                Some(open) if open.within.is_none() => &open.program,
                _ => &editor.file,
            },
        }
    }

    /// Moves the file's state to `moved`, where it says.
    pub fn move_state(&mut self, moved: StateValues) {
        self.editor.file.state.extend(moved);
    }

    /// Edits the file — and the program of the step being edited, which
    /// then sees the edit too — by `edit`. The marker and the step edited
    /// stay at their steps, found again by id: an edit may move steps.
    pub fn edit_program(
        &mut self,
        edit: impl Fn(&mut Program) -> GeopResult<()>,
    ) -> GeopResult<()> {
        let editor = &mut *self.editor;
        let steps = &editor.file.steps;
        let marked = (editor.marker.and_then(|m| steps.get(m))).map(|s| s.id.clone());
        edit(&mut editor.file)?;
        if let Some(marked) = marked {
            editor.marker = Some(editor.file.index_of(&marked)?);
        }
        // A step of a placed part is of another program.
        if let Some(open) = &mut editor.open
            && open.within.is_none()
        {
            edit(&mut open.program)?;
            open.index = (open.program.steps.iter().position(|s| s.id == open.id))
                .ok_or_else(|| GeopError::new(format!("no step {:?}", open.id)))?;
        }
        Ok(())
    }

    /// Makes the file going back to `before` one edit to undo — what a
    /// drag of many events is — if it changed since.
    pub fn undoable(&mut self, before: Program) {
        if before != self.editor.file {
            self.editor.undo.push(before);
            self.editor.redo.clear();
        }
    }

    /// The answer of the inspector `kind` about the elements `picked` of
    /// the part as far as it runs.
    pub fn inspect(&mut self, kind: &str, picked: &[PickedElement]) -> GeopResult<Inspection> {
        let null = serde_json::Value::Null;
        self.editor.ask("", |outputs, output| {
            outputs.get_inspector(kind)?.inspect(output, picked, &null)
        })
    }
}
