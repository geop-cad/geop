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
    ui::{Dialog, PartView, Presentation, Shape, StepEditEvent, Style, Visual},
};
use geop_ops_bom::{Bom, Structure, bom};
use geop_ops_inspect::{
    InterferenceReport, MassReport, Measurement, interference_report, mass_report, measure,
};
use serde::{Deserialize, Serialize};

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

/// The answer to `query` about `part`, which the program `file` builds.
pub fn answer<S: Scalar>(query: Query, part: &Part<S>, file: &str) -> GeopResult<Inspection<S>> {
    Ok(match query {
        Query::MassProperties => Inspection::MassProperties(mass_report(part)?),
        Query::Interference => Inspection::Interference(interference_report(part)?),
        Query::Bom { structure } => Inspection::Bom(bill_of_materials(part, file, structure)?),
    })
}

/// The bill of materials of `part`, which the program `file` builds, its
/// standard parts designated by their norms (see [`crate::stdlib`], which
/// writes them into their programs).
pub fn bill_of_materials<S: Scalar>(
    part: &Part<S>,
    file: &str,
    structure: Structure,
) -> GeopResult<Bom> {
    bom(part, file, structure)
}

/// The bill of materials of `part`, which the program `file` builds, as a
/// drawing lists it (see [`geop_ops_drawing::PartsListLine`]): flat, each
/// part and wire once with its count. Empty if `args` asks for none.
pub fn parts_list<S: Scalar>(
    part: &Part<S>,
    file: &str,
    args: &geop_ops_drawing::DrawingArgs,
) -> GeopResult<Vec<geop_ops_drawing::PartsListLine>> {
    if !args.bom {
        return Ok(Vec::new());
    }
    let bom = bill_of_materials(part, file, Structure::Flat)?;
    Ok(bom
        .lines
        .into_iter()
        .map(|line| geop_ops_drawing::PartsListLine {
            item: line.item,
            quantity: line.quantity,
            name: line.name,
            designation: line.designation.unwrap_or_default(),
            material: match line.kind {
                geop_ops_bom::LineKind::Part { material, .. } => material,
                geop_ops_bom::LineKind::Wire { colour, .. } => format!("wire, {colour}"),
            },
        })
        .collect())
}

/// What the measure tool picks: points, edges, faces — and datums, by their
/// points, lines and planes.
const PICKS: [Role; 5] = [Role::Point, Role::Edge, Role::Face, Role::Line, Role::Plane];

/// The measure tool, in hand: what the pointer is over, what is picked, and
/// what that measures.
pub struct MeasureTool<S: Scalar> {
    hover: Option<EntityRef>,
    picked: Vec<EntityRef>,
    measurement: Measurement<S>,
}

impl<S: Scalar> MeasureTool<S> {
    pub fn new(part: &Part<S>) -> Self {
        Self {
            hover: None,
            picked: Vec::new(),
            measurement: measure(part, &[]),
        }
    }

    /// What the picks measure.
    pub fn measurement(&self) -> &Measurement<S> {
        &self.measurement
    }

    /// Measures the picks again — in `part`, as it now is.
    pub fn remeasure(&mut self, part: &Part<S>) {
        self.measurement = measure(part, &self.picked);
    }

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
        let mut visuals = Vec::new();
        if let Some([a, b]) = self.measurement.witness {
            visuals.push(Visual::new(
                "distance",
                Shape::Polyline { points: vec![a, b] },
                Style::Selected,
            ));
            for (key, at) in [("from", a), ("to", b)] {
                visuals.push(Visual::new(key, Shape::Point { at }, Style::Selected));
            }
            if let Some(distance) = self
                .measurement
                .values
                .iter()
                .find(|m| m.label == "Distance")
            {
                let middle = a.add(&b).prod_scalar(S::from_f64(0.5));
                visuals.push(Visual::new(
                    "distance-label",
                    Shape::Label {
                        at: middle,
                        text: format!("{:.3} mm", distance.value.value),
                        offset: geop_core_math::vector::Vector3::zero(),
                    },
                    Style::Selected,
                ));
            }
        }
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
            grab: false,
            prompt: None,
        }
    }
}
