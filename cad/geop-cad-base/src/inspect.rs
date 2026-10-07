//! The inspect tools of the [`crate::Editor`]: the measure tool, which
//! picks entities and says what they measure, and the questions asked of
//! the part as drawn — its mass properties, which of its solids interfere,
//! its bill of materials.
//!
//! None of it is an operation: inspecting changes no program, adds no step
//! and is nothing to undo. The editor keeps the measure tool's picks as it
//! keeps the drag tool's grab, and answers a question once, when asked (see
//! [`crate::Command::MeasureTool`], [`crate::Command::Inspect`]).

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_ops::{
    EntityRef, Part,
    operation::Role,
    ui::{Dialog, PartView, Presentation, StepEditEvent},
};
use serde::{Deserialize, Serialize};

// While `geop_ops_inspect` and `geop_ops_bom` are out of the workspace (the
// core refactor), these stand in for what they answered: the commands and
// the updates keep their shape, and nothing is measured or listed.

/// How a bill of materials is laid out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Structure {
    #[default]
    Flat,
    Indented,
}

/// What the measure tool's picks measure: nothing, for now.
#[derive(Clone, Debug, Default, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Measurement<S: Scalar> {
    #[serde(skip)]
    scalar: std::marker::PhantomData<fn() -> S>,
}

/// The mass properties of a part's solids: not answered, for now.
#[derive(Clone, Debug, Serialize)]
pub struct MassReport;

/// Which solids of a part overlap: not answered, for now.
#[derive(Clone, Debug, Serialize)]
pub struct InterferenceReport;

/// A bill of materials: not answered, for now.
#[derive(Clone, Debug, Serialize)]
pub struct Bom;

/// A question asked of the part as drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Query {
    /// The mass properties of each of its solids, and of all of them.
    MassProperties,
    /// Which of its solids overlap, and which touch.
    Interference,
    /// Its bill of materials, laid out as `structure` says.
    Bom { structure: Structure },
}

/// What the inspect tools found, as an update carries it.
#[derive(Clone, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", bound = "S: Scalar")]
#[allow(clippy::large_enum_variant)]
pub enum Inspection<S: Scalar> {
    /// What the measure tool's picks measure — sent with every update while
    /// it is in hand.
    Measure(Measurement<S>),
    MassProperties(MassReport),
    Interference(InterferenceReport),
    Bom(Bom),
}

/// The answer to `query` about `part`, which the program `file` builds:
/// none, while the crates that answer are out of the workspace.
pub fn answer<S: Scalar>(
    _query: Query,
    _part: &Part<S>,
    _file: &str,
) -> GeopResult<Inspection<S>> {
    Err(geop_core_math::geop_error::GeopError::new(
        "mass properties, interference and the bill of materials are not available while their crates are out of the workspace",
    ))
}

/// What the measure tool picks: points, edges, faces — and datums, by their
/// points, lines and planes.
const PICKS: [Role; 5] = [
    Role::Point,
    Role::Curve,
    Role::Face,
    Role::Line,
    Role::Plane,
];

/// The measure tool, in hand: what the pointer is over, what is picked, and
/// what that measures.
pub struct MeasureTool<S: Scalar> {
    hover: Option<EntityRef>,
    picked: Vec<EntityRef>,
    measurement: Measurement<S>,
}

impl<S: Scalar> MeasureTool<S> {
    pub fn new(_part: &Part<S>) -> Self {
        Self {
            hover: None,
            picked: Vec::new(),
            measurement: Measurement::default(),
        }
    }

    /// What the picks measure.
    pub fn measurement(&self) -> &Measurement<S> {
        &self.measurement
    }

    /// Measures the picks again — in `part`, as it now is.
    pub fn remeasure(&mut self, _part: &Part<S>) {}

    /// An event in the viewport, `view` being what is drawn of `part`: a
    /// hover finds what a click would pick; a click picks it — a second
    /// pick joins the first, a third replaces the oldest, a click on one
    /// picked takes it out, a click on nothing clears them; Escape clears
    /// them too.
    pub fn handle(&mut self, view: &PartView<S>, part: &Part<S>, event: &StepEditEvent<S>) {
        let picked = |pointer| view.pick(pointer, &PICKS, None).map(|hit| hit.entity);
        match event {
            StepEditEvent::Hover { pointer, .. } => self.hover = picked(pointer),
            StepEditEvent::Leave => self.hover = None,
            StepEditEvent::Click { pointer, .. } => {
                match picked(pointer) {
                    Some(entity) => match self.picked.iter().position(|p| *p == entity) {
                        Some(i) => {
                            self.picked.remove(i);
                        }
                        None => {
                            self.picked.push(entity);
                            if self.picked.len() > 2 {
                                self.picked.remove(0);
                            }
                        }
                    },
                    None => self.picked.clear(),
                }
                self.remeasure(part);
            }
            StepEditEvent::Key { key } if key == "Escape" => {
                self.picked.clear();
                self.remeasure(part);
            }
            _ => {}
        }
    }

    /// What it shows: the picks and what a click would pick lit, and the
    /// least distance between two picks drawn as a line between the points
    /// attaining it, labelled with it.
    pub fn presentation(&self) -> Presentation<S> {
        let visuals = Vec::new();
        let mut highlights = self.picked.clone();
        highlights.extend(
            self.hover
                .iter()
                .filter(|h| !self.picked.contains(h))
                .cloned(),
        );
        Presentation {
            dialog: Dialog::new(),
            visuals,
            highlights,
            pickable: PICKS.to_vec(),
            focus: None,
            sheet: None,
            grab: false,
            prompt: None,
            gizmo: None,
        }
    }
}
