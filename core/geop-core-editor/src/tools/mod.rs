//! [`Tool`]: a mode in hand while no step is edited, which the events of
//! the viewport go to — the drag tool of an assembly, the pick tool of an
//! inspector ([`pick`]).
//!
//! A tool is no operation: what it does is no step. It may move the
//! program's state, or ask about the part, and says what it changed as a
//! command does ([`Changed`]).

pub mod pick;

use std::any::Any;

use geop_core_math::geop_error::GeopResult;
use serde::Serialize;

use crate::{
    editing::{Presentation, StepEditEvent},
    editor::{Changed, EditorContext},
    output::Inspection,
};

/// What a tool is to an editor: the key it is taken in hand by
/// ([`crate::editor::Command::Tool`]), and how it is offered. A tool is no
/// operation, so it is offered where the inspectors are — beside what asks
/// about the part — not among the operations a step can be.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ToolInfo {
    pub key: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    /// What it is shown as: SVG on a 20 × 20 grid, as an operation's icon.
    pub icon: Option<&'static str>,
}

/// A mode in hand while no step is edited: see the module docs. Taken in
/// hand by its key, a fresh one each time (see
/// [`crate::registry::Registry::tool`]).
pub trait Tool: Any {
    /// Something the user did in the viewport.
    fn handle(&mut self, ctx: &mut EditorContext, event: &StepEditEvent) -> GeopResult<Changed>;

    /// What it shows: what it would take lit, and whether a press grabs.
    fn presentation(&self) -> Presentation;

    /// What it answers about the part, with every update while it is in
    /// hand: nothing, unless it says otherwise.
    fn inspection(&mut self, _ctx: &mut EditorContext) -> GeopResult<Option<Inspection>> {
        Ok(None)
    }
}
