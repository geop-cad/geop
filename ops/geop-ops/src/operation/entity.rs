//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face or datum of the part by name, or the origin, a world axis or
//! a base plane — and [`Geometry`], what such an entity is: a point, a
//! line, a plane, an arc, something round, a curve — or several of these at
//! once. Which of them an entity is decides what can be built on it.

use crate::Part;
use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use serde::{Deserialize, Serialize};

/// One of the world's three axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WorldAxis {
    X,
    Y,
    Z,
}

impl WorldAxis {
    fn unit<S: Scalar>(self) -> Vector3<S> {
        match self {
            WorldAxis::X => v3(1., 0., 0.),
            WorldAxis::Y => v3(0., 1., 0.),
            WorldAxis::Z => v3(0., 0., 1.),
        }
    }
}

fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// Something picked to build on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum EntityRef {
    /// The world origin.
    Origin,
    /// A world axis, through the origin.
    Axis {
        axis: WorldAxis,
    },
    /// A base plane through the origin, named by its normal: the `Z` plane
    /// is normal to the `z` axis. A sketch's `x`/`y` on it run along world
    /// `y`/`z` (`X`), `x`/`-z` (`Y`) or `x`/`y` (`Z`).
    Plane {
        normal: WorldAxis,
    },
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
    Datum {
        name: String,
    },
}

impl std::fmt::Display for EntityRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EntityRef::Origin => write!(f, "the origin"),
            EntityRef::Axis { axis } => write!(f, "the {axis:?} axis"),
            EntityRef::Plane { normal } => write!(f, "the {normal:?} plane"),
            EntityRef::Vertex { name } => write!(f, "vertex {name:?}"),
            EntityRef::Edge { name } => write!(f, "edge {name:?}"),
            EntityRef::Face { name } => write!(f, "face {name:?}"),
            EntityRef::Datum { name } => write!(f, "datum {name:?}"),
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
    /// Its own axes, if it has any: a datum's frame, the world's for the
    /// origin, axes and base planes.
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
    /// A vertex, a datum point, the origin.
    Point,
    /// A straight edge, a datum axis, a world axis.
    Line,
    /// A planar face, a datum plane, a base plane.
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

/// The world's own axes, moved to `origin`.
pub fn world_frame<S: Scalar>(origin: Vector3<S>) -> GeopResult<CoordinateSystem<S>> {
    CoordinateSystem::try_new(origin, v3(1., 0., 0.), v3(0., 1., 0.), v3(0., 0., 1.))
}

/// The frame of the base plane normal to `normal`.
fn base_plane<S: Scalar>(normal: WorldAxis) -> GeopResult<CoordinateSystem<S>> {
    let origin = Vector3::zero();
    match normal {
        WorldAxis::X => {
            CoordinateSystem::try_new(origin, v3(0., 1., 0.), v3(0., 0., 1.), v3(1., 0., 0.))
        }
        WorldAxis::Y => {
            CoordinateSystem::try_new(origin, v3(1., 0., 0.), v3(0., 0., -1.), v3(0., 1., 0.))
        }
        WorldAxis::Z => world_frame(origin),
    }
}

impl EntityRef {
    /// What the entity is in `part`. Fails if the part has no such entity.
    pub fn resolve<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Geometry<S>> {
        let ctx = with_context!("resolving {self}");
        let mut g = Geometry::none();
        match self {
            EntityRef::Origin => {
                g.point = Some(Vector3::zero());
                g.frame = Some(world_frame(Vector3::zero())?);
            }
            EntityRef::Axis { axis } => {
                let direction = axis.unit();
                g.line = Some(Axis::try_new(Vector3::zero(), direction)?);
                g.frame = Some(frame_along(Vector3::zero(), &direction)?);
            }
            EntityRef::Plane { normal } => {
                let frame = base_plane(*normal)?;
                g.plane = Some(frame.clone());
                g.frame = Some(frame);
            }
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
            EntityRef::Datum { name } => {
                let id = part.datum_id(name).with_context(ctx)?;
                let datum = part.datum(id).with_context(ctx)?;
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
