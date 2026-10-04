//! [`measure`]: what picked entities measure — one alone, its position,
//! length, area, radius or volume; two together, the least distance between
//! them and where it is attained, the angle between them, the distance
//! between their centres.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::distance::{Feature, PlacedFace, closest_points};
use geop_ops::{EntityRef, Part, operation::Aspects};
use serde::Serialize;

use crate::{Bounded, bodies::resolve, mass::UNGIVEN_DENSITY};

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
#[serde(bound = "S: Scalar")]
pub struct Measurement<S: Scalar> {
    /// The entities measured, as picked.
    pub entities: Vec<EntityRef>,
    pub values: Vec<Measured>,
    /// Where the least distance between two entities is attained: a point
    /// on each.
    pub witness: Option<[Vector3<S>; 2]>,
    /// The plane of one planar entity picked alone — a planar face, a datum
    /// plane — to cut a section view with.
    pub plane: Option<CoordinateSystem<S>>,
    /// Why something could not be measured.
    pub error: Option<String>,
}

fn measured<S: Scalar>(label: &str, value: S, unit: &'static str) -> Measured {
    Measured {
        label: label.into(),
        value: Bounded::of(value),
        unit,
    }
}

/// What `entities`, named as `part` names them, measure: nothing for none,
/// one alone, or two together. More than two are refused.
pub fn measure<S: Scalar>(part: &Part<S>, entities: &[EntityRef]) -> Measurement<S> {
    let mut out = Measurement {
        entities: entities.to_vec(),
        values: Vec::new(),
        witness: None,
        plane: None,
        error: None,
    };
    let result = match entities {
        [] => Ok(()),
        [one] => alone(part, one, &mut out),
        [a, b] => together(part, a, b, &mut out),
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

/// One entity alone: a point's coordinates, an edge's length — and radius,
/// if it is circular — a face's area — and radius, if it is a cylinder —
/// a solid's volume, area and mass.
fn alone<S: Scalar>(part: &Part<S>, entity: &EntityRef, out: &mut Measurement<S>) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {entity}"));
    let aspects = Aspects::of(entity, part).with_context(&ctx)?;
    let (owner, local, _) = resolve(part, entity).with_context(&ctx)?;
    out.plane = aspects.plane.clone();
    match &local {
        EntityRef::Edge { name } => {
            let curve = &owner.topology().get_edge(owner.edge_id(name)?)?.curve;
            out.values
                .push(measured("Length", curve.length().with_context(&ctx)?.0, "mm"));
        }
        EntityRef::Face { name } => {
            let face = owner.face_id(name)?;
            let (area, _) = owner.topology().face_area(face).with_context(&ctx)?;
            out.values.push(measured("Area", area, "mm²"));
            let surface = &owner.topology().get_face(face)?.surface;
            if let Some((_, radius)) = surface.as_cylinder().with_context(&ctx)? {
                out.values.push(measured("Radius", radius, "mm"));
                out.values
                    .push(measured("Diameter", radius.mul(S::TWO), "mm"));
            }
        }
        EntityRef::Solid { name } => {
            let density = owner.material().map_or(UNGIVEN_DENSITY, |m| m.density);
            let per_mm3 = S::from_f64(density).div(S::from_f64(1e9))?;
            let mass = owner
                .topology()
                .mass_properties(owner.solid_id(name)?, per_mm3)
                .with_context(&ctx)?;
            out.values.push(measured("Volume", mass.volume, "mm³"));
            out.values.push(measured("Area", mass.area, "mm²"));
            out.values.push(measured("Mass", mass.mass, "kg"));
        }
        _ => {}
    }
    if let Some(arc) = &aspects.arc {
        out.values.push(measured("Radius", arc.circle.radius, "mm"));
        out.values
            .push(measured("Diameter", arc.circle.radius.mul(S::TWO), "mm"));
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
fn together<S: Scalar>(
    part: &Part<S>,
    a: &EntityRef,
    b: &EntityRef,
    out: &mut Measurement<S>,
) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {a} and {b}"));
    let (fa, fb) = (feature(part, a), feature(part, b));
    let (aa, ab) = (
        Aspects::of(a, part).with_context(&ctx)?,
        Aspects::of(b, part).with_context(&ctx)?,
    );
    match (fa, fb) {
        (Ok(fa), Ok(fb)) => {
            let closest = closest_points(&fa, &fb).with_context(&ctx)?;
            out.values.push(measured("Distance", closest.distance, "mm"));
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

/// What a distance can be measured to: a vertex or another point, an edge,
/// a face — each where it is placed.
fn feature<'p, S: Scalar>(part: &'p Part<S>, entity: &EntityRef) -> GeopResult<Feature<'p, S>> {
    let ctx = |e: GeopError| e.with_context(format!("measuring a distance to {entity}"));
    let (owner, local, pose) = resolve(part, entity).with_context(&ctx)?;
    match &local {
        EntityRef::Edge { name } => {
            let curve = &owner.topology().get_edge(owner.edge_id(name)?)?.curve;
            Ok(Feature::Curve(match &pose {
                Some(pose) => curve.place(pose),
                None => curve.clone(),
            }))
        }
        EntityRef::Face { name } => Ok(Feature::Face(
            PlacedFace::new(owner.topology(), owner.face_id(name)?, pose).with_context(&ctx)?,
        )),
        _ => match Aspects::of(entity, part).with_context(&ctx)?.point {
            Some(point) => Ok(Feature::Point(point)),
            None => Err(GeopError::new(format!(
                "a distance is measured between vertices, points, edges and faces, and {entity} is none"
            ))),
        },
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
    // atan2 of enclosures, from their ends — monotone over the quadrant
    // both lie in — each widened outward by two steps of an `f64`, against
    // the rounding of `atan2` and of the conversion to degrees.
    let atan = |y: S, x: S| {
        let deg = |y: f64, x: f64| y.atan2(x).to_degrees();
        let step = |v: f64, up: bool| {
            let bits = v.to_bits();
            let next = if v == 0.0 {
                f64::from_bits(1)
            } else if (v > 0.0) == up {
                f64::from_bits(bits + 1)
            } else {
                f64::from_bits(bits - 1)
            };
            if v == 0.0 && !up { -next } else { next }
        };
        let lo = deg(y.lower().to_f64(), x.upper().to_f64());
        let hi = deg(y.upper().to_f64(), x.lower().to_f64());
        S::from_f64(step(step(lo, false), false)).union(S::from_f64(step(step(hi, true), true)))
    };
    let between = atan(sin, cos);
    Ok(if a_normal == b_normal {
        between
    } else {
        S::from_f64(90.0).sub(between)
    })
}
