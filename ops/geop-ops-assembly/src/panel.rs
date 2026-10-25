//! The joints panel ([`JointsPanel`]): the values of an assembly a user
//! sets — a joint's coordinates — how free its placed parts are, and which
//! of its constraints conflict.

use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};

use geop_core_editor::{
    editor::{Changed, EditorContext},
    registry::Panel,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::{program::Program, reader::Reader};
use serde::{Deserialize, Serialize};

use geop_ops_model::placed::placed_parts;

use crate::{
    settle::Settled,
    solving::{self, Freedom, SettingsState, settings},
};

/// The joints panel: see the module docs.
pub struct JointsPanel {
    pub(crate) settled: Rc<Settled>,
    /// How free the placed parts are, as the last edit left them (see
    /// [`solving::freedom`]): asked over the whole part, so once an edit,
    /// never for every move of the pointer.
    freedom: RefCell<Option<Freedom>>,
}

impl JointsPanel {
    pub(crate) fn new(settled: Rc<Settled>) -> Self {
        Self {
            settled,
            freedom: RefCell::new(None),
        }
    }
}

/// What the joints panel shows.
#[derive(Serialize)]
pub struct JointsState {
    /// The values of the part shown a user sets, grouped as its steps
    /// declare them — a joint's coordinates — with where they are: what
    /// [`JointsCommand::Set`] sets (see [`solving::settings`]).
    pub joints: Vec<SettingsState>,
    /// How free each of its placed parts is — if it has placed parts — as
    /// the last edit left them (see [`solving::freedom`]).
    pub freedom: Option<Freedom>,
    /// A smallest set of its constraints that cannot all hold together, by
    /// name: none where they all hold.
    pub conflicting: Vec<String>,
}

/// What the joints panel is sent.
#[derive(Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum JointsCommand {
    /// Set the value `parameter` a step offers a user to set — a joint's
    /// coordinate: an angle in degrees, or a distance — to `value`: the
    /// parts move to it, everything else that is free giving way.
    Set { parameter: String, value: f64 },
}

impl Panel for JointsPanel {
    const KEY: &'static str = "joints";
    type State = JointsState;
    type Command = JointsCommand;

    fn state(&self, _: &Program, part: &Reader) -> JointsState {
        let placed = placed_parts(part).is_ok_and(|placed| !placed.is_empty());
        JointsState {
            joints: settings(part),
            freedom: self.freedom.borrow().clone().filter(|_| placed),
            conflicting: self.settled.conflicting.borrow().clone(),
        }
    }

    fn command(&self, ctx: &mut EditorContext, command: JointsCommand) -> GeopResult<Changed> {
        let JointsCommand::Set { parameter, value } = command;
        ctx.idle()?;
        ctx.run_all();
        let settable = (settings(&ctx.part()).into_iter())
            .any(|s| s.values.iter().any(|v| v.state == parameter));
        if !settable {
            return Err(GeopError::new(format!(
                "{parameter:?} is no value of this program a user sets"
            )));
        }
        let set = BTreeMap::from([(parameter, Scalar::from_f64(value))]);
        let mut rerun = ctx.rerun();
        let state = rerun.program.state.clone();
        let (moved, _) = solving::set(&mut rerun, &state, &set)?;
        ctx.move_state(moved);
        Ok(Changed::Program)
    }

    /// How free the placed parts of what is shown are, kept for the panel
    /// to show.
    fn after_edit(&self, ctx: &mut EditorContext) {
        let placed = placed_parts(&ctx.part()).is_ok_and(|placed| !placed.is_empty());
        let mut rerun = ctx.rerun();
        let state = rerun.program.state.clone();
        *self.freedom.borrow_mut() = placed
            .then(|| solving::freedom(&mut rerun, &state, &BTreeSet::new()).ok())
            .flatten();
    }
}
