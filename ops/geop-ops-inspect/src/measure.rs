//! [`measure`]: what picked entities measure — one alone, its position,
//! length, area, radius or volume; two together, the least distance between
//! them and where it is attained, the angle between them, the distance
//! between their centres.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::{Field, Ring, Scalar},
    vector::Vector3,
};
use geop_core_topology::distance::{Feature, PlacedFace, closest_points};
use geop_ops::{
    Design, EntityRef,
    model::model_or_empty,
    operation::{Aspects, BuildView, Role, Seen},
    output::Bounded,
    placed::{about, owner},
    ui::{Shape, Style, Visual},
};
use serde::Serialize;

/// What is picked to be measured: points, edges, faces — and datums, by
/// their points, lines and planes.
pub const PICKS: &[Role] = &[
    Role::Point,
    Role::Curve,
    Role::Face,
    Role::Line,
    Role::Plane,
];

/// One number a measurement shows.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Measured {
    pub label: String,
    #[serde(flatten)]
    pub value: Bounded,
    /// `mm`, `mm²`, `mm³`, `kg` or `°`.
    pub unit: &'static str,
}

/// What picked entities measure.
#[derive(Clone, Debug, Serialize)]
pub struct Measurement {
    /// The entities measured, as picked.
    pub entities: Vec<EntityRef>,
    pub values: Vec<Measured>,
    /// Where the least distance between two entities is attained: a point
    /// on each.
    pub witness: Option<[Vector3<Design>; 2]>,
    /// The plane of one planar entity picked alone — a planar face, a datum
    /// plane — to cut a section view with.
    pub plane: Option<CoordinateSystem<Design>>,
    /// Why something could not be measured.
    pub error: Option<String>,
}

impl Measurement {
    /// What it draws in the viewport: the least distance between two
    /// picks, as a line between the points attaining it, labelled with it.
    pub fn visuals(&self) -> Vec<Visual<Design>> {
        let Some([a, b]) = self.witness else {
            return Vec::new();
        };
        let mut visuals = vec![Visual::new(
            "distance",
            Shape::Polyline { points: vec![a, b] },
            Style::Selected,
        )];
        for (key, at) in [("from", a), ("to", b)] {
            visuals.push(Visual::new(key, Shape::Point { at }, Style::Selected));
        }
        if let Some(distance) = self.values.iter().find(|m| m.label == "Distance") {
            visuals.push(Visual::new(
                "distance-label",
                Shape::Label {
                    at: a.add(&b).prod_scalar(Design::from_f64(0.5)),
                    text: format!("{:.3} mm", distance.value.value),
                    offset: Vector3::zero(),
                },
                Style::Selected,
            ));
        }
        visuals
    }
}

fn measured<S: Scalar>(label: &str, value: S, unit: &'static str) -> Measured {
    Measured {
        label: label.into(),
        value: Bounded::of(value),
        unit,
    }
}

/// What `entities`, named as the part `view` shows names them, measure:
/// nothing for none, one alone, or two together. More than two are
/// refused.
pub fn measure(view: BuildView, entities: &[EntityRef]) -> Measurement {
    let mut out = Measurement {
        entities: entities.to_vec(),
        values: Vec::new(),
        witness: None,
        plane: None,
        error: None,
    };
    let result = match entities {
        [] => Ok(()),
        [one] => alone(view, one, &mut out),
        [a, b] => together(view, a, b, &mut out),
        _ => Err(GeopError::new(format!(
            "{} entities are picked: a measurement takes one or two",
            entities.len()
        ))),
    };
    if let Err(e) = result {
        out.error = Some(e.to_string());
    }
    out
}

/// One entity alone: a point's coordinates, a curve's length — an edge's
/// or a sketch's, and its radius, if it is circular — a face's area — and
/// radius, if it is a cylinder — a solid's volume, area and mass.
fn alone(view: BuildView, entity: &EntityRef, out: &mut Measurement) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {entity}"));
    let aspects = Aspects::of(entity, &mut Seen::new(view)).with_context(&ctx)?;
    let (part, _, local) = owner(view, entity).with_context(&ctx)?;
    let owner = model_or_empty(&mut part.clone());
    out.plane = aspects.plane.clone();
    if let Some(curve) = &aspects.curve {
        out.values.push(measured(
            "Length",
            curve.length().with_context(&ctx)?.0,
            "mm",
        ));
    }
    match &local {
        EntityRef::Face { name } => {
            let face = owner.face_id(name)?;
            let (area, _) = owner.topology().face_area(face).with_context(&ctx)?;
            out.values.push(measured("Area", area, "mm²"));
            let surface = &owner.topology().get_face(face)?.surface;
            if let Some(radius) = surface.as_cylinder().with_context(&ctx)?.map(|c| c.radius) {
                out.values.push(measured("Radius", radius, "mm"));
                out.values
                    .push(measured("Diameter", radius.mul(Design::TWO), "mm"));
            }
        }
        EntityRef::Solid { name } => {
            let density = about(part).material().density;
            let per_mm3 = Design::from_f64(density).div(Design::from_f64(1e9))?;
            let solid = owner.solid_id(name)?;
            let mass = owner
                .topology()
                .mass_properties(solid)
                .with_context(&ctx)?
                .with_density(per_mm3);
            let (area, _) = owner.topology().solid_area(solid).with_context(&ctx)?;
            out.values.push(measured("Volume", mass.volume, "mm³"));
            out.values.push(measured("Area", area, "mm²"));
            out.values.push(measured("Mass", mass.mass, "kg"));
        }
        _ => {}
    }
    if let Some(arc) = &aspects.arc {
        out.values.push(measured("Radius", arc.circle.radius, "mm"));
        out.values.push(measured(
            "Diameter",
            arc.circle.radius.mul(Design::TWO),
            "mm",
        ));
    }
    if let Some(p) = aspects.point {
        for (k, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
            out.values.push(measured(axis, p[k], "mm"));
        }
    }
    Ok(())
}

/// Two entities together: the least distance between them, if both are
/// points, edges or faces; the angle between them, if both have a
/// direction; the distance between their centres, if both are circular.
fn together(
    view: BuildView,
    a: &EntityRef,
    b: &EntityRef,
    out: &mut Measurement,
) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {a} and {b}"));
    let (ma, mb) = (owned(view, a), owned(view, b));
    let (fa, fb) = (feature(view, a, ma.as_ref()), feature(view, b, mb.as_ref()));
    let mut seen = Seen::new(view);
    let (aa, ab) = (
        Aspects::of(a, &mut seen).with_context(&ctx)?,
        Aspects::of(b, &mut seen).with_context(&ctx)?,
    );
    match (fa, fb) {
        (Ok(fa), Ok(fb)) => {
            let closest = closest_points(&fa, &fb).with_context(&ctx)?;
            out.values
                .push(measured("Distance", closest.distance, "mm"));
            let delta = closest.b.sub(&closest.a);
            for (k, axis) in ["dX", "dY", "dZ"].into_iter().enumerate() {
                out.values.push(measured(axis, delta[k].abs(), "mm"));
            }
            out.witness = Some([closest.a, closest.b]);
        }
        (Err(e), _) | (_, Err(e)) if direction(&aa).is_none() || direction(&ab).is_none() => {
            return Err(ctx(e));
        }
        _ => {}
    }
    if let (Some(da), Some(db)) = (direction(&aa), direction(&ab)) {
        out.values.push(measured("Angle", angle(&da, &db)?, "°"));
    }
    if let (Some(ca), Some(cb)) = (&aa.arc, &ab.arc) {
        let between = cb.circle.center.sub(&ca.circle.center).norm();
        out.values.push(measured("Centre distance", between, "mm"));
    }
    Ok(())
}

/// The model of the part `entity` is of, where that part is, and the
/// entity as it names it (see [`owner`]).
type Owned<'a> = (
    std::borrow::Cow<'a, geop_ops::NamedModel<Design>>,
    Option<geop_core_math::primitives::Pose<Design>>,
    EntityRef,
);

/// [`owner`] of `entity`, with that part's model.
fn owned<'a>(view: BuildView<'a>, entity: &EntityRef) -> GeopResult<Owned<'a>> {
    let (part, pose, local) = owner(view, entity)?;
    Ok((model_or_empty(&mut part.clone()), pose, local))
}

/// What a distance can be measured to: a vertex or another point, an edge
/// or a sketch's curve, a face — each where it is placed; `owned` is
/// [`owned`] of it.
fn feature<'m>(
    view: BuildView,
    entity: &EntityRef,
    owned: Result<&'m Owned, &GeopError>,
) -> GeopResult<Feature<'m, Design>> {
    let ctx = |e: GeopError| e.with_context(format!("measuring a distance to {entity}"));
    let (model, pose, local) = owned.map_err(|e| ctx(GeopError::new(e.to_string())))?;
    if let EntityRef::Face { name } = local {
        return Ok(Feature::Face(
            PlacedFace::new(model.topology(), model.face_id(name)?, *pose).with_context(&ctx)?,
        ));
    }
    let aspects = Aspects::of(entity, &mut Seen::new(view)).with_context(&ctx)?;
    match (aspects.curve, aspects.point) {
        (Some(curve), _) => Ok(Feature::Curve(curve)),
        (None, Some(point)) => Ok(Feature::Point(point)),
        (None, None) => Err(GeopError::new(format!(
            "a distance is measured between vertices, points, curves and faces, and {entity} is none"
        ))),
    }
}

/// The direction an entity has: a line's, or a plane's normal — `true` for
/// a normal.
fn direction<S: Scalar>(aspects: &Aspects<S>) -> Option<(Vector3<S>, bool)> {
    match (&aspects.line, &aspects.plane) {
        (Some(line), _) => Some((line.direction, false)),
        (None, Some(plane)) => Some((*plane.w(), true)),
        _ => None,
    }
}

/// The acute angle between two lines, two planes or a line and a plane, in
/// degrees: between two directions, or between two normals, the angle
/// between them folded to at most 90°; between a line and a plane, 90°
/// less the angle between the line and the normal.
fn angle<S: Scalar>(
    (a, a_normal): &(Vector3<S>, bool),
    (b, b_normal): &(Vector3<S>, bool),
) -> GeopResult<S> {
    let (a, b) = (a.normalize()?, b.normalize()?);
    let cos = a.prod_dot(&b).abs();
    let sin = a.prod_cross(&b).norm();
    // In degrees: the interval atan2 encloses the angle, and the factor
    // 180 / pi is an enclosure too.
    let between = sin.atan2(cos).mul(S::from_f64(180.0).div(S::PI)?);
    Ok(if a_normal == b_normal {
        between
    } else {
        S::from_f64(90.0).sub(between)
    })
}
