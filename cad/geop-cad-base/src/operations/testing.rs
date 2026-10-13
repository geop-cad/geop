//! What the tests of programs share beyond what every operation's tests
//! do (`geop_ops_model::testing`, `geop_ops_extrude_revolve::testing`):
//! building one beside the standard parts, and editing it as a front end
//! does.

use std::collections::BTreeMap;

use geop_core_editor::editing::{Button, Pointer, Reach, StepEditEvent, Value};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::ray::Ray,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops_extrude_revolve::testing::model_of;
use geop_ops_model::{Design, NamedModel};

pub(crate) use geop_ops_datums::{origin_axis, origin_plane};
pub(crate) use geop_ops_extrude_revolve::testing::{
    args, both_ways, disc, edit_args, error, extrude_sketch, extrusion, on_face, on_origin,
    one_way, outline, rectangle, sketch, step, two_ways,
};
pub(crate) use geop_ops_model::testing::{assert_valid, check_valid, solid_volume};

use super::{Program, Runner, build, workspace};
use crate::{Command, Editor, Update};

/// Building a program on its own, in an empty [`workspace`] (see [`build`]).
pub(crate) trait Built {
    /// The runner, holding the part the program builds.
    fn built(&self) -> GeopResult<Runner>;

    /// The model the program builds, or why a step fails.
    fn build_model(&self) -> GeopResult<NamedModel<Design>>;
}

impl Built for Program {
    fn built(&self) -> GeopResult<Runner> {
        build(self, "", workspace(BTreeMap::new()).scope(""))
    }

    fn build_model(&self) -> GeopResult<NamedModel<Design>> {
        Ok(model_of(&self.built()?))
    }
}

/// A pointer from `origin` along `dir`, reaching 0.009 units.
pub(crate) fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).expect("a test's pointer points somewhere"),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

/// Looking straight down at `(x, y)`.
pub(crate) fn from_above(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// What `editor` sends back on `command`, which it has to take without an
/// error.
pub(crate) fn handled(editor: &mut Editor, command: Command) -> Update {
    let update = editor.handle(command);
    assert!(update.error.is_none(), "{:?}", update.error);
    update
}

/// A click of the primary button with `pointer`.
pub(crate) fn click_at(pointer: Pointer<S>) -> Command {
    Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    }
}

/// An editor on `program`, which loads without an error.
pub(crate) fn loaded(program: Program) -> Editor {
    let mut editor = Editor::new();
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
    editor
}

/// An editor on the example `name`.
pub(crate) fn example_editor(name: &str) -> Editor {
    let mut editor = Editor::new();
    handled(&mut editor, Command::LoadExample { name: name.into() });
    editor
}

/// The command a front end sends when the open step's dialog field `key`
/// is set to `value`.
pub(crate) fn dialog(key: &str, value: Value) -> Command {
    Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    }
}

/// One step edited as a front end edits it — every gesture a command to an
/// [`Editor`] — with what the editor last showed of it.
pub(crate) struct Stepping {
    pub(crate) editor: Editor,
    pub(crate) shown: crate::editor::StepState,
}

impl Stepping {
    /// The step being edited after `command`, the editor taking it.
    fn after(mut editor: Editor, command: Command) -> Self {
        let shown = handled(&mut editor, command)
            .step
            .expect("a step is edited");
        Self { editor, shown }
    }

    /// The step `id` of `program`, opened again.
    pub(crate) fn open(program: Program, id: &str) -> Self {
        Self::after(loaded(program), Command::Open { id: id.into() })
    }

    /// A new step of the operation `kind`, after `program`.
    pub(crate) fn new(program: Program, kind: &str) -> Self {
        Self::after(loaded(program), Command::New { kind: kind.into() })
    }

    /// Sends `event`, which the editor has to take without an error.
    pub(crate) fn send(&mut self, event: StepEditEvent<S>) {
        let update = handled(&mut self.editor, Command::Event { event });
        self.shown = update.step.expect("the step is still edited");
    }

    /// Sets the dialog field `key` to `value`.
    pub(crate) fn dialog(&mut self, key: &str, value: Value) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value,
        });
    }

    /// What the step shows now.
    pub(crate) fn presentation(&self) -> &geop_core_editor::editing::Presentation<S> {
        &self.shown.presentation
    }

    /// The step's arguments, as its edits leave them.
    pub(crate) fn args<A: serde::de::DeserializeOwned>(&self) -> A {
        args(self.editor.editing().expect("a step is edited"))
    }

    /// The operation's editing state, as an `E`.
    pub(crate) fn editing<E: 'static>(&self) -> &E {
        let editor = self.editor.step_editor().expect("a step is edited");
        (editor.editing().downcast_ref()).expect("the operation's editing state")
    }
}
