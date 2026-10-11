//! [`mass_report`]: the mass properties of every solid a part shows, and of
//! all of them together.

use geop_core_math::{
    geop_error::GeopResult,
    scalars::{Ring, Scalar},
};
use geop_core_topology::mass::MassProperties;
use geop_ops::{
    Design,
    operation::BuildView,
    output::Bounded,
    solids::{Shapes, placed_solids},
};
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
            principal_axes: principal
                .axes
                .map(|axis| [0, 1, 2].map(|k| axis[k].to_f64())),
            converged: properties.converged && area.1,
        })
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
}

/// The mass properties of every solid a part shows, and of all of them.
#[derive(Clone, Debug, Serialize)]
pub struct MassReport {
    pub bodies: Vec<BodyMass>,
    /// All of them together — if there are any, and every one could be
    /// computed: a total missing a body would mislead.
    pub total: Option<MassSummary>,
}

/// The mass properties of every solid of the part `view` shows — its own,
/// and those of the parts placed in it, where they are and of their own
/// parts' materials — and of all of them together. A part placed many
/// times is integrated once.
pub fn mass_report(view: BuildView) -> GeopResult<MassReport> {
    let mut bodies = Vec::new();
    let mut all = Vec::new();
    let mut area = (Design::ZERO, true);
    let mut complete = true;
    let mut shapes = Shapes::default();
    for placed in placed_solids(view)? {
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
