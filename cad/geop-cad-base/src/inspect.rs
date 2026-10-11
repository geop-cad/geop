//! The pick tool of the [`crate::Editor`]: an inspector that picks (see
//! [`geop_ops::output::InspectorInfo::picks`]) in hand, which picks
//! entities in the viewport, and is asked about them again with every pick.
//!
//! It is no operation: inspecting changes no program, adds no step and is
//! nothing to undo. The editor keeps its picks as it keeps the drag tool's
//! grab (see [`crate::Command::PickTool`]); what the picks are asked is the
//! inspector's.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_ops::{
    Design, EntityRef,
    output::{Inspection, InspectorInfo, Picks},
    ui::{Dialog, PartView, Presentation, StepEditEvent},
};

/// An inspector that picks, in hand: what the pointer is over, what is
/// picked, and the inspector's answer about it.
pub struct PickTool {
    info: &'static InspectorInfo,
    picks: Picks,
    hover: Option<EntityRef>,
    picked: Vec<EntityRef>,
    /// The answer about the picks, once asked: none while it is to be
    /// asked again.
    pub(crate) answer: Option<Inspection>,
}

impl PickTool {
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
    pub fn picked(&self) -> &[EntityRef] {
        &self.picked
    }

    /// An event in the viewport, `view` being what is drawn: a hover finds
    /// what a click would pick; a click picks it — joining the picks, or
    /// replacing the oldest once there are as many as the inspector takes;
    /// a click on one picked takes it out, a click on nothing clears them;
    /// Escape clears them too. A change of the picks asks again.
    pub fn handle(&mut self, view: &PartView, event: &StepEditEvent<Design>) {
        let roles = self.picks.roles;
        let picked = |pointer| view.pick(pointer, roles, None).map(|hit| hit.entity);
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
    }

    /// What it shows: the picks and what a click would pick lit, and what
    /// the answer draws.
    pub fn presentation(&self) -> Presentation<Design> {
        let mut highlights = self.picked.clone();
        highlights.extend(
            self.hover
                .iter()
                .filter(|h| !self.picked.contains(h))
                .cloned(),
        );
        Presentation {
            dialog: Dialog::new(),
            visuals: (self.answer.iter())
                .flat_map(|answer| answer.visuals.iter().cloned())
                .collect(),
            highlights,
            pickable: self.picks.roles.to_vec(),
            focus: None,
            sheet: None,
            grab: false,
            prompt: None,
            gizmo: None,
        }
    }
}
