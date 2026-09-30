//! [`Geometry`]: what an entity a datum is built from is — a point, a line,
//! a plane, an arc, something round, a curve, or several of these at once —
//! and the [`Role`]s that lets it fill. Which of them an entity is decides
//! what can be built on it.

use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::{CoordinateSystem, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{EntityRef, Part};
use serde::Serialize;

/// Everything an entity can be used as. An entity is usually several at
/// once: a straight edge is a line and a curve, a circular edge an arc, a
/// curve and something round, a datum point a point and a frame, a datum
/// frame the same.
#[derive(Clone, Debug, Default)]
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
    /// What `entity` is in `part`. Fails if the part has no such entity.
    pub fn of(entity: &EntityRef, part: &Part<S>) -> GeopResult<Self> {
        let ctx = with_context!("resolving {entity}");
        let mut g = Geometry::default();
        match entity {
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
                g.plane = entity.resolve_plane(part).ok();
                g.round = surface.axis_of_revolution().with_context(ctx)?;
            }
            EntityRef::Datum { .. } => {
                let datum = entity.resolve_datum(part)?;
                let frame = datum.frame;
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
            EntityRef::Sketch { .. } => {
                let plane = entity.resolve_plane(part)?;
                g.plane = Some(plane.clone());
                g.frame = Some(plane);
            }
        }
        Ok(g)
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

    /// As a construction's requirement reads: `a point`.
    pub fn describe(self) -> &'static str {
        match self {
            Role::Point => "a point",
            Role::Line => "a line",
            Role::Plane => "a plane",
            Role::Edge => "an edge",
            Role::Circle => "a circular edge",
            Role::Round => "a circular edge or a round face",
        }
    }
}
