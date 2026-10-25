//! [`measure`]: what picked entities measure — one alone, its position,
//! length, area, radius or volume; two together, the least distance between
//! them and where it is attained, the angle between them, the distance
//! between their centres.

use geop_core_editor::output::{Answer, Block, Bounded, Row};
use geop_core_editor::view::drawn;
use geop_core_editor::{
    editing::{Shape as Drawn, Style, Tone, Visual},
    selectable::Selectable,
};
use geop_core_geometry::shape::{Arc, Axis, Circle};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Pose},
    scalars::{Ring, Scalar},
    vector::Vector3,
};
use geop_core_program::{element::PickedElement, reader::Reader, values::Design};
use geop_core_topology::distance::{Feature, PlacedFace, closest_points};
use geop_ops_model::parameters::located;
use geop_ops_model::selection::{Selected, Shape};
use serde::Serialize;

/// What is picked to be measured, by what is drawn of it: vertices,
/// edges, faces, datums, and a sketch's points and curves.
pub const PICKS: &[&str] = &[
    drawn::VERTEX,
    drawn::EDGE,
    drawn::FACE,
    drawn::DATUM,
    drawn::SKETCH_POINT,
    drawn::SKETCH_CURVE,
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
    pub entities: Vec<PickedElement>,
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
    /// As the inspect panel shows it: what is picked — or what to pick —
    /// why it could not be measured, and what it measures; and the plane
    /// of a planar entity picked alone, to cut a section view with.
    pub fn shown(&self) -> Answer {
        let mut blocks = Vec::new();
        if self.entities.is_empty() {
            blocks.push(Block::Text {
                text: "Click a vertex, edge, face or datum; a second one to measure between them."
                    .into(),
                tone: Tone::Hint,
            });
        } else {
            blocks.push(Block::List {
                items: (self.entities.iter())
                    .map(|e| Row {
                        light: vec![e.clone()],
                        ..Row::of(vec![label(e)])
                    })
                    .collect(),
            });
        }
        blocks.extend(self.error.iter().map(|error| Block::Text {
            text: error.clone(),
            tone: Tone::Error,
        }));
        blocks.push(Block::Table {
            head: Vec::new(),
            rows: (self.values.iter())
                .map(|v| Row::labelled(&v.label, vec![format!("{} {}", v.value.show(3), v.unit)]))
                .collect(),
            foot: Vec::new(),
        });
        Answer {
            blocks,
            plane: self.plane.clone(),
        }
    }

    /// What it draws in the viewport: the least distance between two
    /// picks, as a line between the points attaining it, labelled with it.
    pub fn visuals(&self) -> Vec<Visual<Design>> {
        let Some([a, b]) = self.witness else {
            return Vec::new();
        };
        let mut visuals = vec![Visual::new(
            "distance",
            Drawn::Polyline { points: vec![a, b] },
            Style::Selected,
        )];
        for (key, at) in [("from", a), ("to", b)] {
            visuals.push(Visual::new(key, Drawn::Point { at }, Style::Selected));
        }
        if let Some(distance) = self.values.iter().find(|m| m.label == "Distance") {
            visuals.push(Visual::new(
                "distance-label",
                Drawn::Label {
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

/// How the inspect panel names `picked`: `bolt: face extrude(head,end)`
/// for one of the part placed as `bolt`.
fn label(picked: &PickedElement) -> String {
    match picked.instance() {
        "" => picked.label(),
        path => format!("{path}: {}", picked.label()),
    }
}

/// An entity picked, in the part it is of.
struct Picked<'a> {
    /// The part it is of: the part read, or one placed in it.
    part: Reader<'a>,
    /// Where that part is placed; `None` for the part read itself.
    pose: Option<Pose<Design>>,
    element: Selected,
}

impl<'a> Picked<'a> {
    /// What `picked`, a pick of what is drawn of the part `reader` reads,
    /// is in the part it is of.
    fn of(reader: &Reader<'a>, picked: &PickedElement) -> GeopResult<Self> {
        let (part, pose) = match picked.instance() {
            "" => (reader.clone(), None),
            path => (reader.placed_part(path)?, Some(located(reader, path)?)),
        };
        let inner = picked.local();
        let element = (Selected::from_picked(&inner))
            .or_else(|| Selected::select(&part, &inner))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "{} cannot be measured: pick a vertex, an edge, a face, a datum, or a sketch's point or curve",
                    label(picked)
                ))
            })?;
        Ok(Self {
            part,
            pose,
            element,
        })
    }

    /// What it is, where its part is placed.
    fn shape(&self) -> GeopResult<Shape> {
        let shape = self.element.shape(&self.part)?;
        match &self.pose {
            Some(pose) => placed(shape, pose),
            None => Ok(shape),
        }
    }
}

/// `shape` once its part is moved by `pose`: a part placed in another is
/// measured as it is placed.
fn placed(shape: Shape, pose: &Pose<Design>) -> GeopResult<Shape> {
    let motion = pose.motion();
    let axis = |a: Axis<Design>| Axis {
        point: motion.apply(&a.point),
        direction: motion.rotate(&a.direction),
    };
    let frame = |f: Option<CoordinateSystem<Design>>| f.map(|f| motion.apply_frame(&f)).transpose();
    Ok(Shape {
        point: shape.point.map(|p| motion.apply(&p)),
        line: shape.line.map(axis),
        plane: frame(shape.plane)?,
        arc: shape.arc.map(|arc| Arc {
            circle: Circle {
                center: motion.apply(&arc.circle.center),
                normal: motion.rotate(&arc.circle.normal),
                radius: arc.circle.radius,
            },
            start: motion.apply(&arc.start),
            end: motion.apply(&arc.end),
        }),
        round: shape.round.map(axis),
        curve: shape.curve.map(|c| c.transform(&motion)),
        frame: frame(shape.frame)?,
    })
}

/// The direction `shape` has: a line's, or a plane's normal — `true` for a
/// normal.
fn direction(shape: &Shape) -> Option<(Vector3<Design>, bool)> {
    match (&shape.line, &shape.plane) {
        (Some(line), _) => Some((line.direction, false)),
        (None, Some(plane)) => Some((*plane.w(), true)),
        _ => None,
    }
}

/// What the entities `picked` of what is drawn of the part `reader` reads
/// measure: nothing for none, one alone, or two together. More than two
/// are refused.
pub fn measure(reader: &Reader, picked: &[PickedElement]) -> Measurement {
    let mut out = Measurement {
        entities: picked.to_vec(),
        values: Vec::new(),
        witness: None,
        plane: None,
        error: None,
    };
    let result = match picked {
        [] => Ok(()),
        [one] => alone(reader, one, &mut out),
        [a, b] => together(reader, a, b, &mut out),
        _ => Err(GeopError::new(format!(
            "{} entities are picked: a measurement takes one or two",
            picked.len()
        ))),
    };
    if let Err(e) = result {
        out.error = Some(e.to_string());
    }
    out
}

/// One entity alone: a point's coordinates, a curve's length — an edge's
/// or a sketch's, and its radius, if it is circular — a face's area — and
/// radius, if it is a cylinder.
fn alone(reader: &Reader, picked: &PickedElement, out: &mut Measurement) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {}", label(picked)));
    let entity = Picked::of(reader, picked).with_context(&ctx)?;
    let shape = entity.shape().with_context(&ctx)?;
    out.plane = shape.plane.clone();
    if let Some(curve) = &shape.curve {
        out.values.push(measured(
            "Length",
            curve.length().with_context(&ctx)?.0,
            "mm",
        ));
    }
    if let Selected::Face(face) = &entity.element {
        let face = entity.part.retrieve_element(face).with_context(&ctx)?;
        let topology = face.model.topology();
        let (area, _) = topology.face_area(face.id).with_context(&ctx)?;
        out.values.push(measured("Area", area, "mm²"));
        let surface = &topology.get_face(face.id)?.surface;
        if let Some(radius) = surface.as_cylinder().with_context(&ctx)?.map(|c| c.radius) {
            out.values.push(measured("Radius", radius, "mm"));
            out.values
                .push(measured("Diameter", radius.mul(Design::TWO), "mm"));
        }
    }
    if let Some(arc) = &shape.arc {
        out.values.push(measured("Radius", arc.circle.radius, "mm"));
        out.values.push(measured(
            "Diameter",
            arc.circle.radius.mul(Design::TWO),
            "mm",
        ));
    }
    if let Some(p) = shape.point {
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
    reader: &Reader,
    a: &PickedElement,
    b: &PickedElement,
    out: &mut Measurement,
) -> GeopResult<()> {
    let ctx = |e: GeopError| e.with_context(format!("measuring {} and {}", label(a), label(b)));
    let (pa, pb) = (
        Picked::of(reader, a).with_context(&ctx)?,
        Picked::of(reader, b).with_context(&ctx)?,
    );
    let (sa, sb) = (
        pa.shape().with_context(&ctx)?,
        pb.shape().with_context(&ctx)?,
    );
    let (fa, fb) = (feature(&pa, &sa), feature(&pb, &sb));
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
        (Err(e), _) | (_, Err(e)) if direction(&sa).is_none() || direction(&sb).is_none() => {
            return Err(ctx(e));
        }
        _ => {}
    }
    if let (Some(da), Some(db)) = (direction(&sa), direction(&sb)) {
        out.values.push(measured("Angle", angle(&da, &db)?, "°"));
    }
    if let (Some(ca), Some(cb)) = (&sa.arc, &sb.arc) {
        let between = cb.circle.center.sub(&ca.circle.center).norm();
        out.values.push(measured("Centre distance", between, "mm"));
    }
    Ok(())
}

/// What a distance can be measured to: a vertex or another point, an edge
/// or a sketch's curve, a face — each where it is placed; `shape` is what
/// `entity` is there.
fn feature<'m>(entity: &Picked<'m>, shape: &Shape) -> GeopResult<Feature<'m, Design>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "measuring a distance to {}",
            entity.element.picked().label()
        ))
    };
    if let Selected::Face(face) = &entity.element {
        let face = entity.part.retrieve_element(face).with_context(&ctx)?;
        return PlacedFace::new(face.model.topology(), face.id, entity.pose)
            .map(Feature::Face)
            .with_context(&ctx);
    }
    match (&shape.curve, shape.point) {
        (Some(curve), _) => Ok(Feature::Curve(curve.clone())),
        (None, Some(point)) => Ok(Feature::Point(point)),
        (None, None) => Err(ctx(GeopError::new(
            "a distance is measured between vertices, points, curves and faces",
        ))),
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
