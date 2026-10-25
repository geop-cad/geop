//! One element of a part, picked in the viewport, whatever it is
//! ([`Selected`]) — a vertex, an edge, a face, a datum, a sketch's point or
//! curve — and what it can be used as ([`Shape`], [`Role`]): a point, a
//! line, a plane, a curve, a circle, something round. What a datum is built
//! from, what a mate holds by, what is measured: a crate that lets the user
//! pick any element of a part reads the pick through `Selected` rather than
//! naming each kind of element itself.

use crate::{
    Circle, Curve, Edge, Face, Line, NamedModel, Point, Round, Vertex,
    datums::{AnyDatum, Datums},
    parameters::located,
    sketch::PlacedSketch,
};
use geop_core_editor::one_of;
use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, DatumKind, Motion},
    vector::Vector3,
};
use geop_core_program::{
    element::ElementReference, placed::InInstance, reader::Reader, values::Design,
};

one_of! {
    /// An element a datum is built from.
    pub enum Selected {
        Vertex(ElementReference<NamedModel<Design>, Vertex>),
        Edge(ElementReference<NamedModel<Design>, Edge>),
        Face(ElementReference<NamedModel<Design>, Face>),
        Datum(ElementReference<Datums, AnyDatum>),
        SketchPoint(ElementReference<PlacedSketch<Design>, Point>),
        SketchCurve(ElementReference<PlacedSketch<Design>, Curve>),
    }
}

/// What a construction takes one element as.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    Point,
    Line,
    Plane,
    /// Any curve: an edge, a sketch's curve.
    Curve,
    /// A circle or an arc of one.
    Circle,
    /// Something turning around an axis: a circular edge, a cylinder, a
    /// cone, a sketch's circle.
    Round,
}

impl Role {
    pub const ALL: [Role; 6] = [
        Role::Point,
        Role::Line,
        Role::Plane,
        Role::Curve,
        Role::Circle,
        Role::Round,
    ];

    /// Its name: `point`.
    pub fn name(self) -> &'static str {
        match self {
            Role::Point => "point",
            Role::Line => "line",
            Role::Plane => "plane",
            Role::Curve => "curve",
            Role::Circle => "circle",
            Role::Round => "round",
        }
    }

    /// It in words, with its article: `a point`.
    pub fn describe(self) -> &'static str {
        match self {
            Role::Point => "a point",
            Role::Line => "a line",
            Role::Plane => "a plane",
            Role::Curve => "an edge or a curve",
            Role::Circle => "a circle or an arc",
            Role::Round => "something round",
        }
    }
}

/// Everything an element can be used as. An element is usually several at
/// once: a straight edge is a line and a curve, a circular edge a circle, a
/// curve and something round, a datum point a point with a frame.
#[derive(Clone, Debug, Default)]
pub struct Shape {
    pub point: Option<Vector3<Design>>,
    /// The line it runs along: a straight edge, an axis, a sketch line.
    pub line: Option<Axis<Design>>,
    /// The plane it lies in, as a frame with `w` the normal.
    pub plane: Option<CoordinateSystem<Design>>,
    pub arc: Option<Arc<Design>>,
    /// The axis it turns around.
    pub round: Option<Axis<Design>>,
    /// The curve it runs along, whatever its shape.
    pub curve: Option<NurbCurve3D<Design>>,
    /// Its own axes, if it has any: a datum's frame.
    pub frame: Option<CoordinateSystem<Design>>,
}

impl Shape {
    /// Whether it can be used as `role`.
    pub fn fits(&self, role: Role) -> bool {
        match role {
            Role::Point => self.point.is_some(),
            Role::Line => self.line.is_some(),
            Role::Plane => self.plane.is_some(),
            Role::Curve => self.curve.is_some(),
            Role::Circle => self.arc.is_some(),
            Role::Round => self.round.is_some(),
        }
    }

    /// Every role it can fill.
    pub fn roles(&self) -> Vec<Role> {
        Role::ALL
            .into_iter()
            .filter(|&role| self.fits(role))
            .collect()
    }

    /// It moved by `motion`.
    pub fn transform(&self, motion: &Motion<Design>) -> GeopResult<Self> {
        let frame = |frame: &Option<CoordinateSystem<Design>>| {
            frame.as_ref().map(|f| motion.apply_frame(f)).transpose()
        };
        Ok(Shape {
            point: self.point.map(|p| motion.apply(&p)),
            line: self.line.as_ref().map(|l| l.transform(motion)),
            plane: frame(&self.plane)?,
            arc: self.arc.as_ref().map(|a| a.transform(motion)),
            round: self.round.as_ref().map(|r| r.transform(motion)),
            curve: self.curve.as_ref().map(|c| c.transform(motion)),
            frame: frame(&self.frame)?,
        })
    }

    /// What a curve is: a line, a circle, something round, as it is one.
    fn of_curve(curve: NurbCurve3D<Design>) -> GeopResult<Self> {
        let arc = curve.as_arc()?;
        Ok(Shape {
            line: curve.as_line()?,
            round: arc.as_ref().map(|arc| arc.circle.axis()),
            arc,
            curve: Some(curve),
            ..Shape::default()
        })
    }
}

impl Selected {
    /// What it is in the part `reader` reads: where the part, or the
    /// instance placed in it that it is of, has it.
    pub fn shape(&self, reader: &Reader) -> GeopResult<Shape> {
        let InInstance { element, instance } = self.shape_in_instance(reader)?;
        match instance.is_empty() {
            true => Ok(element),
            false => element.transform(&located(reader, &instance)?.motion()),
        }
    }

    /// What it is in its own part — the part `reader` reads, or the
    /// instance placed in it that it is of — and that instance's path:
    /// where that is, [`geop_ops_model::parameters::placement`] says.
    pub fn shape_in_instance(&self, reader: &Reader) -> GeopResult<InInstance<Shape>> {
        let instance = self.instance();
        let element = match instance.is_empty() {
            true => self.own_shape(reader)?,
            false => self.local().own_shape(&reader.placed_part(instance)?)?,
        };
        Ok(InInstance {
            element,
            instance: instance.to_string(),
        })
    }

    /// What it is, of the part `reader` reads itself.
    fn own_shape(&self, reader: &Reader) -> GeopResult<Shape> {
        Ok(match self {
            Selected::Vertex(vertex) => {
                let found = reader.retrieve_element(vertex)?;
                Shape {
                    point: Some(found.model.topology().get_vertex(found.id)?.point),
                    ..Shape::default()
                }
            }
            Selected::Edge(edge) => {
                let found = reader.retrieve_element(edge)?;
                Shape::of_curve(found.model.topology().get_edge(found.id)?.curve.clone())?
            }
            Selected::Face(face) => {
                let found = reader.retrieve_element(face)?;
                let surface = &found.model.topology().get_face(found.id)?.surface;
                Shape {
                    plane: found.model.frame_of_face(found.id).ok(),
                    round: surface.axis_of_revolution()?,
                    ..Shape::default()
                }
            }
            Selected::Datum(datum) => {
                let datum = reader.retrieve_element(datum)?;
                let frame = datum.frame;
                let mut shape = Shape::default();
                match datum.kind {
                    DatumKind::Point | DatumKind::Frame => shape.point = Some(*frame.origin()),
                    DatumKind::Axis => {
                        shape.line = Some(Axis::try_new(*frame.origin(), *frame.w())?)
                    }
                    DatumKind::Plane => shape.plane = Some(frame.clone()),
                }
                shape.frame = Some(frame);
                shape
            }
            Selected::SketchPoint(point) => Shape {
                point: Some(reader.retrieve_element(point)?),
                ..Shape::default()
            },
            Selected::SketchCurve(curve) => {
                let chain = reader.retrieve_element(curve)?;
                let [one] = <[NurbCurve3D<Design>; 1]>::try_from(chain.curves).map_err(|_| {
                    GeopError::new(format!("the curve {:?} is more than one", curve.name()))
                })?;
                // What the sketch says the curve is, where its curve alone
                // does not: a circle is round about its center.
                let mut shape = Shape::of_curve(one)?;
                shape.line = shape
                    .line
                    .or_else(|| reader.retrieve_element(&as_kind::<Line>(curve)).ok());
                shape.arc = shape
                    .arc
                    .or_else(|| reader.retrieve_element(&as_kind::<Circle>(curve)).ok());
                shape.round = (shape.round)
                    .or_else(|| reader.retrieve_element(&as_kind::<Round>(curve)).ok());
                shape
            }
        })
    }
}

/// The sketch's curve `curve` as what `K` it is.
fn as_kind<K>(
    curve: &ElementReference<PlacedSketch<Design>, Curve>,
) -> ElementReference<PlacedSketch<Design>, K> {
    ElementReference::new(curve.target().clone(), curve.name())
}
