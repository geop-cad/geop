//! The pick tool: an inspector that picks (see
//! [`crate::output::InspectorInfo::picks`]) in hand, which picks entities
//! in the viewport, and is asked about them again with every pick.
//!
//! It is no operation: inspecting changes no program, adds no step and is
//! nothing to undo. What the picks are asked is the inspector's.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::element::PickedElement;

use super::Tool;
use crate::{
    editing::{Dialog, Layout, Presentation, StepEditEvent},
    editor::{Changed, EditorContext},
    output::{Inspection, InspectorInfo, Picks},
};

/// An inspector that picks, in hand: what the pointer is over, what is
/// picked, and the inspector's answer about it.
pub struct PickTool {
    info: &'static InspectorInfo,
    picks: Picks,
    hover: Option<PickedElement>,
    picked: Vec<PickedElement>,
    /// The answer about the picks, once asked, with the run of the part it
    /// was asked of: none while it is to be asked again.
    answer: Option<(u64, Inspection)>,
}

impl PickTool {
    /// What the key of an inspector's pick tool starts with:
    /// `pick:measure`.
    pub const PREFIX: &'static str = "pick:";

    /// The tool of the inspector `info`. Fails for one that is asked of the
    /// whole part.
    pub fn new(info: &'static InspectorInfo) -> GeopResult<Self> {
        let picks = info.picks.ok_or_else(|| {
            GeopError::new(format!(
                "the inspector `{}` picks nothing: it is asked of the whole part",
                info.kind
            ))
        })?;
        Ok(Self {
            info,
            picks,
            hover: None,
            picked: Vec::new(),
            answer: None,
        })
    }

    /// Its inspector's kind.
    pub fn kind(&self) -> &'static str {
        self.info.kind
    }

    /// What is picked, the oldest first.
    pub fn picked(&self) -> &[PickedElement] {
        &self.picked
    }

    /// The kinds of what is drawn it picks from.
    fn accepts(&self) -> Vec<String> {
        self.picks
            .accepts
            .iter()
            .map(|kind| kind.to_string())
            .collect()
    }
}

impl Tool for PickTool {
    /// An event in the viewport, picking in what is drawn: a hover finds
    /// what a click would pick; a click picks it — joining the picks, or
    /// replacing the oldest once there are as many as the inspector takes;
    /// a click on one picked takes it out, a click on nothing clears them;
    /// Escape clears them too. A change of the picks asks again.
    fn handle(&mut self, ctx: &mut EditorContext, event: &StepEditEvent) -> GeopResult<Changed> {
        let Some(view) = ctx.drawn() else {
            return Ok(Changed::Nothing);
        };
        let accepts = self.accepts();
        let picked = |pointer| view.pick(pointer, &accepts).map(|hit| hit.element);
        let before = self.picked.clone();
        match event {
            StepEditEvent::Hover { pointer, .. } => self.hover = picked(pointer),
            StepEditEvent::Leave => self.hover = None,
            StepEditEvent::Click { pointer, .. } => match picked(pointer) {
                Some(entity) => match self.picked.iter().position(|p| *p == entity) {
                    Some(i) => {
                        self.picked.remove(i);
                    }
                    None => {
                        self.picked.push(entity);
                        if self.picked.len() > self.picks.most {
                            self.picked.remove(0);
                        }
                    }
                },
                None => self.picked.clear(),
            },
            StepEditEvent::Key { key } if key == "Escape" => self.picked.clear(),
            _ => {}
        }
        if self.picked != before {
            self.answer = None;
        }
        Ok(Changed::Nothing)
    }

    /// What it shows: the picks and what a click would pick lit, and what
    /// the answer draws.
    fn presentation(&self) -> Presentation {
        let mut highlights = self.picked.clone();
        highlights.extend(
            self.hover
                .iter()
                .filter(|h| !self.picked.contains(h))
                .cloned(),
        );
        Presentation {
            dialog: Dialog::new(),
            layout: Layout::default(),
            visuals: (self.answer.iter())
                .flat_map(|(_, answer)| answer.visuals.iter().cloned())
                .collect(),
            highlights,
            pickable: self.accepts(),
            focus: None,
            sheet: None,
            grab: false,
            prompt: None,
            gizmo: None,
            hidden: Vec::new(),
        }
    }

    /// The inspector's answer about the picks, asked again only if they or
    /// the part changed since.
    fn inspection(&mut self, ctx: &mut EditorContext) -> GeopResult<Option<Inspection>> {
        let runs = ctx.runs();
        if let Some((asked, answer)) = &self.answer
            && *asked == runs
        {
            return Ok(Some(answer.clone()));
        }
        let answer = ctx.inspect(self.info.kind, &self.picked)?;
        self.answer = Some((runs, answer.clone()));
        Ok(Some(answer))
    }
}
