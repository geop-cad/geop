//! What the tests of programs share beyond what every operation's tests
//! do (`geop_ops::testing`, `geop_ops_extrude_revolve::testing`): building
//! one beside the standard parts, and editing it as a front end does.

use std::collections::BTreeMap;

use geop_core_math::geop_error::GeopResult;
use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{
    Design, NamedModel,
    testing::model_of,
    ui::{Button, Pointer, StepEditEvent, Value},
};

pub(crate) use geop_ops::testing::{
    args, assert_valid, check_valid, edit_args, error, from_above, pointer, solid_volume, step,
};
pub(crate) use geop_ops::{origin_axis, origin_plane};
pub(crate) use geop_ops_extrude_revolve::testing::{
    both_ways, disc, extrude_sketch, extrusion, one_way, outline, rectangle, sketch, two_ways,
};

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
        build(self, &workspace(BTreeMap::new()).scope(""))
    }

    fn build_model(&self) -> GeopResult<NamedModel<Design>> {
        Ok(model_of(&self.built()?))
    }
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
