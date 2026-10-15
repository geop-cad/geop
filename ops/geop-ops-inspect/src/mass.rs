//! [`mass_report`]: the mass properties of every solid a part shows, and of
//! all of them together.

use geop_core_editor::{editing::Tone, selectable::SelectableElement};
use geop_core_math::{
    geop_error::GeopResult,
    scalars::{Ring, Scalar},
};
use geop_core_program::{
    element::{ElementReference, PickedElement},
    part::Reader,
    values::Design,
};
use geop_core_topology::mass::MassProperties;
use geop_ops_assembly::solids::{PlacedSolid, Shapes, placed_solids};
use geop_ops_model::{Solid, model};
use geop_ops_output::{Answer, Block, Bounded, Row};
use serde::Serialize;

/// Mass properties as shown, in millimetres and kilograms: the volume
/// (mm³), the area of the boundary (mm²), the mass (kg), the centre of mass
/// (mm), the inertia tensor about it along the world's axes (kg·mm²), and
/// the principal moments (kg·mm²) with their axes.
#[derive(Clone, Debug, Serialize)]
pub struct MassSummary {
    pub volume: Bounded,
    pub area: Bounded,
    pub mass: Bounded,
    pub center: [Bounded; 3],
    pub inertia: [[Bounded; 3]; 3],
    pub principal_moments: [Bounded; 3],
    /// The axis of each principal moment, a unit vector — where moments
    /// coincide, any axis of their plane is one, and these are a choice.
    pub principal_axes: [[f64; 3]; 3],
    /// Whether every integral came within its tolerance (see
    /// [`geop_core_math::quadrature`]): when not, the bounds are wider.
    pub converged: bool,
}

impl MassSummary {
    /// The summary of `properties` of a body whose boundary has the area
    /// `area`, with whether that area's quadrature converged.
    pub fn of<S: Scalar>(properties: &MassProperties<S>, area: (S, bool)) -> GeopResult<Self> {
        let principal = properties.principal()?;
        Ok(Self {
            volume: Bounded::of(properties.volume),
            area: Bounded::of(area.0),
            mass: Bounded::of(properties.mass),
            center: [0, 1, 2].map(|k| Bounded::of(properties.center[k])),
            inertia: properties.inertia.map(|row| row.map(Bounded::of)),
            principal_moments: principal.moments.map(Bounded::of),
            principal_axes: principal.axes.map(|axis| axis.to_f64_array()),
            converged: properties.converged && area.1,
        })
    }
}

impl MassSummary {
    /// As a table shows it: a row per property, in millimetres and
    /// kilograms.
    fn shown(&self) -> Block {
        let axes = ["x", "y", "z"];
        let center: Vec<String> = self.center.iter().map(|c| c.show(3)).collect();
        let inertia: Vec<String> = (axes.iter().zip(&self.inertia))
            .map(|(axis, row)| {
                let row: Vec<String> = row.iter().map(|c| format!("{:.5}", c.value)).collect();
                format!("{axis}: {}", row.join("  "))
            })
            .collect();
        let principal: Vec<String> = self.principal_moments.iter().map(|m| m.show(4)).collect();
        let about: Vec<String> = (self.principal_axes.iter())
            .map(|a| format!("({:.3}, {:.3}, {:.3})", a[0], a[1], a[2]))
            .collect();
        Block::Table {
            head: Vec::new(),
            rows: vec![
                Row::labelled("Mass", vec![self.mass.show_mass()]),
                Row::labelled("Volume", vec![format!("{} mm³", self.volume.show(3))]),
                Row::labelled("Area", vec![format!("{} mm²", self.area.show(3))]),
                Row::labelled("Centre", vec![format!("{} mm", center.join(", "))]),
                Row {
                    detail: Some("About the centre of mass, along the world's axes".into()),
                    ..Row::labelled("Inertia", vec![format!("{}\nkg·mm²", inertia.join("\n"))])
                },
                Row {
                    detail: Some(format!("about {}", about.join(", "))),
                    ..Row::labelled(
                        "Principal",
                        vec![format!("{}\nkg·mm²", principal.join("\n"))],
                    )
                },
            ],
            foot: Vec::new(),
        }
    }
}

/// One solid's mass properties, or why they could not be computed.
#[derive(Clone, Debug, Serialize)]
pub struct BodyMass {
    /// The solid, named as the part shown names it.
    pub name: String,
    /// What it is made of: its part's material — or, not given, water,
    /// and `assumed` says so.
    pub material: String,
    /// kg/m³.
    pub density: f64,
    pub assumed: bool,
    pub properties: Option<MassSummary>,
    pub error: Option<String>,
    /// What is drawn of it, to light.
    #[serde(skip)]
    pub drawn: Vec<PickedElement>,
}

/// The mass properties of every solid a part shows, and of all of them.
#[derive(Clone, Debug, Serialize)]
pub struct MassReport {
    pub bodies: Vec<BodyMass>,
    /// All of them together — if there are any, and every one could be
    /// computed: a total missing a body would mislead.
    pub total: Option<MassSummary>,
}

impl MassReport {
    /// As the inspect panel shows it: all solids together, then each, lit
    /// while pointed at, with its material — and whether it was assumed —
    /// and why it could not be weighed.
    pub fn shown(&self) -> Answer {
        let mut blocks = Vec::new();
        if self.bodies.is_empty() {
            blocks.push(Block::Text {
                text: "No solids.".into(),
                tone: Tone::Hint,
            });
        }
        if let Some(total) = self.total.as_ref().filter(|_| self.bodies.len() > 1) {
            blocks.push(Block::Group {
                title: format!("All {} solids", self.bodies.len()),
                detail: None,
                open: true,
                light: Vec::new(),
                blocks: vec![total.shown()],
            });
        }
        for body in &self.bodies {
            let mut shown = Vec::new();
            if body.assumed {
                shown.push(Block::Text {
                    text: "No material given: weighed as water. Set one under Parameters.".into(),
                    tone: Tone::Hint,
                });
            }
            shown.extend(body.error.iter().map(|error| Block::Text {
                text: error.clone(),
                tone: Tone::Error,
            }));
            shown.extend(body.properties.iter().map(MassSummary::shown));
            blocks.push(Block::Group {
                title: body.name.clone(),
                detail: Some(body.material.clone()),
                open: self.bodies.len() == 1,
                light: body.drawn.clone(),
                blocks: shown,
            });
        }
        Answer {
            blocks,
            plane: None,
        }
    }
}

/// The mass properties of every solid of the part `reader` reads — its own,
/// and those of the parts placed in it, where they are and of their own
/// parts' materials — and of all of them together. A part placed many
/// times is integrated once.
pub fn mass_report(reader: &Reader) -> GeopResult<MassReport> {
    let mut bodies = Vec::new();
    let mut all = Vec::new();
    let mut area = (Design::ZERO, true);
    let mut complete = true;
    let mut shapes = Shapes::default();
    for placed in placed_solids(reader)? {
        let material = placed.material();
        let computed = placed.mass_properties(&mut shapes).and_then(|p| {
            let a = placed.model.topology().solid_area(placed.solid)?;
            Ok((MassSummary::of(&p, a)?, p, a))
        });
        let (properties, error) = match computed {
            Ok((summary, p, a)) => {
                all.push(p);
                area = (area.0.add(a.0), area.1 && a.1);
                (Some(summary), None)
            }
            Err(e) => {
                complete = false;
                (None, Some(e.to_string()))
            }
        };
        bodies.push(BodyMass {
            drawn: drawn(&placed),
            name: placed.name,
            material: material.name,
            density: material.density,
            assumed: material.assumed,
            properties,
            error,
        });
    }
    let total = match complete {
        true => MassProperties::combine(&all)?
            .map(|p| MassSummary::of(&p, area))
            .transpose()?,
        false => None,
    };
    Ok(MassReport { bodies, total })
}

/// What is drawn of `solid` where its part is placed, to light: its faces,
/// named as the part read names them — behind the path of the part placed
/// that it is of.
pub(crate) fn drawn(solid: &PlacedSolid) -> Vec<PickedElement> {
    let Some(own) = solid.model.name_of(solid.solid) else {
        return Vec::new();
    };
    let path = solid.name.strip_suffix(own).unwrap_or_default();
    let reference = ElementReference::new(model(), own);
    (Solid::drawn(&solid.reader, &reference).into_iter())
        .map(|face| PickedElement {
            target: format!("{path}{}", face.target),
            ..face
        })
        .collect()
}
