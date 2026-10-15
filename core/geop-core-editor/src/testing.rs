//! What tests of editing share (feature `testing`): an [`Editor`] over a
//! [`Registry`] of the operations a test needs ([`editor_with`]), driven
//! as a front end drives it — pointers, clicks, dialog fields — and one
//! step edited that way ([`Stepping`]).

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::ray::Ray,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::program::Program;
use serde::de::DeserializeOwned;

use crate::{
    editing::{Button, Pointer, Presentation, Reach, StepEditEvent, Value},
    editor::{Command, Editor, StepState, Update},
    registry::Registry,
};

/// An editor of what `registry` offers, with no other files, on `program`,
/// which loads without an error.
pub fn editor_with(registry: Registry, program: Program) -> Editor {
    loaded(Editor::new(registry, BTreeMap::new()), program)
}

/// `editor` on `program`, which loads without an error.
pub fn loaded(mut editor: Editor, program: Program) -> Editor {
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
    editor
}

/// A pointer from `origin` along `dir`, reaching 0.009 units.
pub fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).expect("a test's pointer points somewhere"),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

/// Looking straight down at `(x, y)`.
pub fn from_above(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// What `editor` sends back on `command`, which it has to take without an
/// error.
#[track_caller]
pub fn handled(editor: &mut Editor, command: Command) -> Update {
    let update = editor.handle(command);
    assert!(update.error.is_none(), "{:?}", update.error);
    update
}

/// A click of the primary button with `pointer`.
pub fn click_at(pointer: Pointer<S>) -> Command {
    Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    }
}

/// The command a front end sends when the open step's dialog field `key`
/// is set to `value`.
pub fn dialog(key: &str, value: Value) -> Command {
    Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    }
}

/// One step edited as a front end edits it — every gesture a command to an
/// [`Editor`] — with what the editor last showed of it.
pub struct Stepping {
    pub editor: Editor,
    pub shown: StepState,
}

impl Stepping {
    /// The step being edited after `command`, the editor taking it.
    fn after(mut editor: Editor, command: Command) -> Self {
        let shown = handled(&mut editor, command)
            .step
            .expect("a step is edited");
        Self { editor, shown }
    }

    /// The step `id` of the program `editor` edits, opened again.
    pub fn open(editor: Editor, id: &str) -> Self {
        Self::after(editor, Command::Open { id: id.into() })
    }

    /// A new step of the operation `kind`, after the program `editor`
    /// edits.
    pub fn new(editor: Editor, kind: &str) -> Self {
        Self::after(editor, Command::New { kind: kind.into() })
    }

    /// Sends `event`, which the editor has to take without an error.
    pub fn send(&mut self, event: StepEditEvent<S>) {
        let update = handled(&mut self.editor, Command::Event { event });
        self.shown = update.step.expect("the step is still edited");
    }

    /// Sets the dialog field `key` to `value`.
    pub fn dialog(&mut self, key: &str, value: Value) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value,
        });
    }

    /// What the step shows now.
    pub fn presentation(&self) -> &Presentation<S> {
        &self.shown.presentation
    }

    /// The step's arguments, as its edits leave them.
    pub fn args<A: DeserializeOwned>(&self) -> A {
        let step = self.editor.editing().expect("a step is edited");
        serde_json::from_value(step.args.clone())
            .unwrap_or_else(|e| panic!("step {:?} holds no such arguments: {e}", step.id))
    }

    /// The operation's editing state, as an `E`.
    pub fn editing<E: 'static>(&self) -> &E {
        let editor = self.editor.step_editor().expect("a step is edited");
        (editor.editing().downcast_ref()).expect("the operation's editing state")
    }
}
