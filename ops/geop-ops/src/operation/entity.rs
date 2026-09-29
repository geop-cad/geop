//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face, datum, solid or sketch of the part, by name — and [`Geometry`], what such an entity is: a point, a
//! line, a plane, an arc, something round, a curve — or several of these at
//! once. Which of them an entity is decides what can be built on it.

use crate::Part;
use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, DatumComponent, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use serde::{Deserialize, Serialize};

/// Something picked in the viewport, to build on or to use.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum EntityRef {
    Vertex {
        name: String,
    },
    Edge {
        name: String,
    },
    /// A face: planar, its plane — normal pointing out of its solid, sketch
    /// origin the world origin's projection onto it and sketch `x` the world
    /// axis most parallel to it, projected, so sketches on parallel faces
    /// line up. Round, its axis.
    Face {
        name: String,
    },
    /// A datum — or, of a frame, one of its axes or planes (see
    /// [`geop_core_math::primitives::Datum::component`]). Every part has
    /// the frame [`ORIGIN`](crate::ORIGIN).
    Datum {
        name: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        component: Option<DatumComponent>,
    },
    /// A solid, as a whole.
    Solid {
        name: String,
    },
    /// A sketch: its plane.
    Sketch {
        name: String,
    },
}

impl EntityRef {
    /// A datum, as a whole.
    pub fn datum(name: impl Into<String>) -> Self {
        EntityRef::Datum {
            name: name.into(),
            component: None,
        }
    }

    /// A component of the frame datum `name`.
    pub fn datum_component(name: impl Into<String>, component: DatumComponent) -> Self {
        EntityRef::Datum {
            name: name.into(),
            component: Some(component),
        }
    }

    /// How the entity is shown: its name, and which component of a frame.
    pub fn label(&self) -> String {
        match self {
            EntityRef::Datum {
                name,
                component: Some(component),
            } => format!("{name} {component}"),
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Datum { name, .. }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name } => name.clone(),
        }
    }
}

impl std::fmt::Display for EntityRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EntityRef::Vertex { name } => write!(f, "vertex {name:?}"),
            EntityRef::Edge { name } => write!(f, "edge {name:?}"),
            EntityRef::Face { name } => write!(f, "face {name:?}"),
            EntityRef::Datum {
                name,
                component: None,
            } => write!(f, "datum {name:?}"),
            EntityRef::Datum {
                name,
                component: Some(component),
            } => write!(f, "the {component} of datum {name:?}"),
            EntityRef::Solid { name } => write!(f, "solid {name:?}"),
            EntityRef::Sketch { name } => write!(f, "sketch {name:?}"),
        }
    }
}

/// Everything an entity can be used as. An entity is usually several at
/// once: a straight edge is a line and a curve, a circular edge an arc, a
/// curve and something round, a datum point a point and a frame, a datum
/// frame the same.
#[derive(Clone, Debug)]
pub struct Geometry<S: Scalar> {
    pub point: Option<Vector3<S>>,
    /// The line it runs along: a straight edge, an axis.
    pub line: Option<Axis<S>>,
    /// The plane it lies in, as a frame with `w` the normal and `u`/`v` a
    /// sketch's `x`/`y` on it.
    pub plane: Option<CoordinateSystem<S>>,
    pub arc: Option<Arc<S>>,
    /// The axis it turns around: a circular edge, a cylinder, a cone.
    pub round: Option<Axis<S>>,
    /// An edge's curve, whatever its shape.
    pub curve: Option<NurbCurve3D<S>>,
    /// Its own axes, if it has any: a datum's frame.
    pub frame: Option<CoordinateSystem<S>>,
}

impl<S: Scalar> Geometry<S> {
    fn none() -> Self {
        Self {
            point: None,
            line: None,
            plane: None,
            arc: None,
            round: None,
            curve: None,
            frame: None,
        }
    }

    /// Every role it can fill.
    pub fn roles(&self) -> Vec<Role> {
        Role::ALL
            .into_iter()
            .filter(|role| role.fits(self))
            .collect()
    }
}

/// What a construction needs an input to be.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    /// A vertex, a datum point, a frame's origin.
    Point,
    /// A straight edge, a datum axis, a frame's axis.
    Line,
    /// A planar face, a datum plane, a frame's plane.
    Plane,
    /// Any edge.
    Edge,
    /// A circular edge.
    Circle,
    /// Something that turns around an axis: a circular edge, a cylindrical,
    /// conical or spherical face.
    Round,
}

impl Role {
    const ALL: [Role; 6] = [
        Role::Point,
        Role::Line,
        Role::Plane,
        Role::Edge,
        Role::Circle,
        Role::Round,
    ];

    pub fn fits<S: Scalar>(self, geometry: &Geometry<S>) -> bool {
        match self {
            Role::Point => geometry.point.is_some(),
            Role::Line => geometry.line.is_some(),
            Role::Plane => geometry.plane.is_some(),
            Role::Edge => geometry.curve.is_some(),
            Role::Circle => geometry.arc.is_some(),
            Role::Round => geometry.round.is_some(),
        }
    }
}

/// A right-handed orthonormal frame at `origin` with `w` along `normal`,
/// and `u` the world axis most parallel to the plane normal to it,
/// projected into that plane — so frames with parallel normals line up.
pub fn frame_along<S: Scalar>(
    origin: Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<CoordinateSystem<S>> {
    let n = normal.normalize()?;
    let axis = (0..3)
        .min_by(|&a, &b| n[a].to_f64().abs().total_cmp(&n[b].to_f64().abs()))
        .expect("three axes");
    let mut a = Vector3::zero();
    a[axis] = S::ONE;
    let u = a.sub(&n.prod_scalar(n.prod_dot(&a))).normalize()?;
    let v = n.prod_cross(&u);
    CoordinateSystem::try_new(origin, u, v, n)
}

impl EntityRef {
    /// What the entity is in `part`. Fails if the part has no such entity.
    pub fn resolve<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Geometry<S>> {
        let ctx = with_context!("resolving {self}");
        let mut g = Geometry::none();
        match self {
            EntityRef::Vertex { name } => {
                let id = part.vertex_id(name).with_context(ctx)?;
                g.point = Some(part.topology().get_vertex(id).with_context(ctx)?.point);
            }
            EntityRef::Edge { name } => {
                let id = part.edge_id(name).with_context(ctx)?;
                let curve = part
                    .topology()
                    .get_edge(id)
                    .with_context(ctx)?
                    .curve
                    .clone();
                g.line = curve.as_line().with_context(ctx)?;
                g.arc = curve.as_arc().with_context(ctx)?;
                g.round = g.arc.as_ref().map(|arc| arc.circle.axis());
                g.curve = Some(curve);
            }
            EntityRef::Face { name } => {
                let id = part.face_id(name).with_context(ctx)?;
                let surface = &part.topology().get_face(id).with_context(ctx)?.surface;
                if let Some(plane) = surface.as_plane().with_context(ctx)? {
                    let origin = plane.project(&Vector3::zero());
                    g.plane = Some(frame_along(origin, &plane.normal)?);
                }
                g.round = surface.axis_of_revolution().with_context(ctx)?;
            }
            EntityRef::Datum { name, component } => {
                let id = part.datum_id(name).with_context(ctx)?;
                let mut datum = part.datum(id).with_context(ctx)?.clone();
                if let Some(component) = component {
                    datum = datum.component(*component).with_context(ctx)?;
                }
                let frame = datum.frame.clone();
                match datum.kind {
                    DatumKind::Point => g.point = Some(*frame.origin()),
                    DatumKind::Axis => g.line = Some(Axis::try_new(*frame.origin(), *frame.w())?),
                    DatumKind::Plane => g.plane = Some(frame.clone()),
                    // A coordinate system is used by its origin, and by its
                    // axes as the frame below.
                    DatumKind::Frame => g.point = Some(*frame.origin()),
                }
                g.frame = Some(frame);
            }
            // A solid as a whole is none of these.
            EntityRef::Solid { name } => {
                part.solid_id(name).with_context(ctx)?;
            }
            EntityRef::Sketch { name } => {
                let id = part.sketch_id(name).with_context(ctx)?;
                let plane = part.sketch(id).with_context(ctx)?.plane.clone();
                g.plane = Some(plane.clone());
                g.frame = Some(plane);
            }
        }
        Ok(g)
    }
}

/// The frame of the plane `plane` refers to in `part`: `u`/`v` a sketch's
/// `x`/`y` on it, `w = u x v` its normal. Fails if it is not a plane.
pub fn resolve_plane<S: Scalar>(
    part: &Part<S>,
    plane: &EntityRef,
) -> GeopResult<CoordinateSystem<S>> {
    plane
        .resolve(part)?
        .plane
        .ok_or_else(|| GeopError::new(format!("{plane} is not planar")))
}
