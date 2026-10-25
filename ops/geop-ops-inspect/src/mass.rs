//! [`mass_report`]: the mass properties of every solid a part shows, and of
//! all of them together.

use geop_core_editor::output::{Answer, Block, Bounded, Row};
use geop_core_editor::{editing::Tone, selectable::SelectableElement};
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    element::{ElementReference, PickedElement},
    reader::Reader,
};
use geop_core_topology::mass::MassProperties;
use geop_ops_model::placed::solids::{PlacedSolid, Shapes, placed_solids};
use geop_ops_model::{Solid, model, parameters::Material};
use serde::Serialize;

/// Mass properties as shown, in millimetres and kilograms: the volume
/// (mm³), the area of the boundary (mm²) and the centroid (mm) — what the
/// shape decides — and, of a material, what it weighs ([`Inertial`]).
#[derive(Clone, Debug, Serialize)]
pub struct MassSummary {
    pub volume: Bounded,
    pub area: Bounded,
    /// The centre of volume, which of a uniform material is the centre of
    /// mass; of several solids, the mean weighted by mass where every one
    /// has a material and by volume where none does. None for solids of
    /// which only some have one: neither mean would be their centre.
    pub center: Option<[Bounded; 3]>,
    /// None where there is no material (or only some solids have one).
    pub inertial: Option<Inertial>,
    /// Whether every integral came within its tolerance (see
    /// [`geop_core_math::quadrature`]): when not, the bounds are wider.
    pub converged: bool,
}

/// What a body of a material weighs: its mass (kg), its inertia tensor
/// about the centre of mass along the world's axes (kg·mm²), and the
/// principal moments (kg·mm²) with their axes.
#[derive(Clone, Debug, Serialize)]
pub struct Inertial {
    pub mass: Bounded,
    pub inertia: [[Bounded; 3]; 3],
    pub principal_moments: [Bounded; 3],
    /// The axis of each principal moment, a unit vector — where moments
    /// coincide, any axis of their plane is one, and these are a choice.
    pub principal_axes: [[f64; 3]; 3],
}

/// How a body's — or bodies' — mass properties are known.
#[derive(Clone, Copy)]
pub(crate) enum Known {
    /// Of a material: mass and inertia.
    Weighed(MassProperties),
    /// Of no material: of density one (see
    /// [`geop_ops_model::placed::solids::PlacedSolid::shape_properties`]),
    /// of which only the volume and the centroid are reported.
    Shape(MassProperties),
}

impl MassSummary {
    /// The summary of what is `known` of a body — or bodies — whose
    /// boundary has the area `area`, with whether that area's quadrature
    /// converged.
    pub(crate) fn of(known: &Known, area: (Scalar, bool)) -> GeopResult<Self> {
        let (p, inertial) = match known {
            Known::Weighed(p) => {
                let principal = p.principal()?;
                let inertial = Inertial {
                    mass: Bounded::of(p.mass),
                    inertia: p.inertia.map(|row| row.map(Bounded::of)),
                    principal_moments: principal.moments.map(Bounded::of),
                    principal_axes: principal.axes.map(|axis| axis.to_f64_array()),
                };
                (p, Some(inertial))
            }
            Known::Shape(p) => (p, None),
        };
        Ok(Self {
            volume: Bounded::of(p.volume),
            area: Bounded::of(area.0),
            center: Some([0, 1, 2].map(|k| Bounded::of(p.center[k]))),
            inertial,
            converged: p.converged && area.1,
        })
    }

    /// As a table shows it: a row per property, in millimetres and
    /// kilograms.
    fn shown(&self) -> Block {
        let mut rows = Vec::new();
        if let Some(inertial) = &self.inertial {
            rows.push(Row::labelled("Mass", vec![inertial.mass.show_mass()]));
        }
        rows.push(Row::labelled(
            "Volume",
            vec![format!("{} mm³", self.volume.show(3))],
        ));
        rows.push(Row::labelled(
            "Area",
            vec![format!("{} mm²", self.area.show(3))],
        ));
        if let Some(center) = &self.center {
            let center: Vec<String> = center.iter().map(|c| c.show(3)).collect();
            let label = match self.inertial {
                Some(_) => "Centre",
                None => "Centroid",
            };
            rows.push(Row::labelled(
                label,
                vec![format!("{} mm", center.join(", "))],
            ));
        }
        if let Some(inertial) = &self.inertial {
            rows.extend(inertial.shown());
        }
        Block::Table {
            head: Vec::new(),
            rows,
            foot: Vec::new(),
        }
    }
}

impl Inertial {
    /// Its inertia rows, as a table shows them.
    fn shown(&self) -> [Row; 2] {
        let axes = ["x", "y", "z"];
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
        [
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
        ]
    }
}

/// One solid's mass properties, or why they could not be computed.
#[derive(Clone, Debug, Serialize)]
pub struct BodyMass {
    /// The solid, named as the part shown names it.
    pub name: String,
    /// What it is made of: its part's material, none where it is not
    /// given — then it has no mass, only a volume and a centroid.
    pub material: Option<Material>,
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
    /// computed: a total missing a body would mislead. What it says
    /// depends on their materials (see [`MassSummary::center`]).
    pub total: Option<MassSummary>,
}

impl MassReport {
    /// As the inspect panel shows it: all solids together, then each, lit
    /// while pointed at, with its material — and that it has none, which
    /// it needs to be weighed — and why it could not be computed.
    pub fn shown(&self) -> Answer {
        let mut blocks = Vec::new();
        if self.bodies.is_empty() {
            blocks.push(Block::Text {
                text: "No solids.".into(),
                tone: Tone::Hint,
            });
        }
        if let Some(total) = self.total.as_ref().filter(|_| self.bodies.len() > 1) {
            let mut shown = Vec::new();
            if total.center.is_none() {
                shown.push(Block::Text {
                    text: SOME_MATERIAL.into(),
                    tone: Tone::Hint,
                });
            }
            shown.push(total.shown());
            blocks.push(Block::Group {
                title: format!("All {} solids", self.bodies.len()),
                detail: None,
                open: true,
                light: Vec::new(),
                blocks: shown,
            });
        }
        for body in &self.bodies {
            let mut shown = Vec::new();
            if body.material.is_none() {
                shown.push(Block::Text {
                    text: NO_MATERIAL.into(),
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
                detail: Some(
                    body.material
                        .as_ref()
                        .map_or("None".into(), |m| m.name.clone()),
                ),
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

/// The hint on a body of no material.
const NO_MATERIAL: &str = "No material: set one under Properties to get mass and inertia.";

/// The hint on a total of solids only some of which have a material.
const SOME_MATERIAL: &str =
    "Only some solids have a material: of all of them, only their volume and area.";

/// The mass properties of every solid of the part `reader` reads — its own,
/// and those of the parts placed in it, where they are and of their own
/// parts' materials — and of all of them together. A part placed many
/// times is integrated once.
pub fn mass_report(reader: &Reader) -> GeopResult<MassReport> {
    let mut bodies = Vec::new();
    let (mut weighed, mut shapes_only) = (Vec::new(), Vec::new());
    let mut area = (Scalar::ZERO, true);
    let mut complete = true;
    let mut shapes = Shapes::default();
    for placed in placed_solids(reader)? {
        let material = placed.material();
        let computed = (placed.mass_properties(&mut shapes)).and_then(|p| {
            let known = match p {
                Some(p) => Known::Weighed(p),
                None => Known::Shape(placed.shape_properties(&mut shapes)?),
            };
            let a = placed.model.topology().solid_area(placed.solid)?;
            Ok((MassSummary::of(&known, a)?, known, a))
        });
        let (properties, error) = match computed {
            Ok((summary, known, a)) => {
                match known {
                    Known::Weighed(p) => weighed.push(p),
                    Known::Shape(p) => shapes_only.push(p),
                }
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
            material,
            properties,
            error,
        });
    }
    let total = match (weighed.is_empty(), shapes_only.is_empty()) {
        _ if !complete => None,
        (false, true) => (MassProperties::combine(&weighed)?)
            .map(|p| MassSummary::of(&Known::Weighed(p), area))
            .transpose()?,
        (true, _) => (MassProperties::combine(&shapes_only)?)
            .map(|p| MassSummary::of(&Known::Shape(p), area))
            .transpose()?,
        // Some of a material, some of none: neither mean is their centre.
        (false, false) => {
            let all = || weighed.iter().chain(&shapes_only);
            Some(MassSummary {
                volume: Bounded::of(all().fold(Scalar::ZERO, |v, p| v.add(p.volume))),
                area: Bounded::of(area.0),
                center: None,
                inertial: None,
                converged: all().all(|p| p.converged) && area.1,
            })
        }
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
    let reference = ElementReference::new(model(), own);
    (Solid::drawn(&solid.reader, &reference).into_iter())
        .map(|face| face.behind(&solid.instance))
        .collect()
}
