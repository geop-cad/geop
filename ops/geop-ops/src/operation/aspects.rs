//! [`Aspects`]: what an entity a step builds on can be used as — a point, a
//! line, a plane, an arc, something round, a curve, a face, a solid, a
//! sheet, a sketch, or several of these at once — and the [`Role`]s that lets it fill. Which of them an
//! entity is decides what it can be picked for, and what can be built on
//! it.

use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis, Circle},
};
use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::{CoordinateSystem, DatumKind, Pose},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::CurveKind;
use serde::Serialize;

use super::EntityRef;
use crate::Part;

/// Everything an entity can be used as. An entity is usually several at
/// once: a straight edge is a line and a curve, a circular edge an arc, a
/// curve and something round, a datum point a point and a frame, a datum
/// frame the same.
#[derive(Clone, Debug, Default)]
pub struct Aspects<S: Scalar> {
    pub point: Option<Vector3<S>>,
    /// The line it runs along: a straight edge, an axis, a sketch line.
    pub line: Option<Axis<S>>,
    /// The plane it lies in, as a frame with `w` the normal and `u`/`v` a
    /// sketch's `x`/`y` on it.
    pub plane: Option<CoordinateSystem<S>>,
    pub arc: Option<Arc<S>>,
    /// The axis it turns around: a circular edge, a cylinder, a cone, a
    /// sketch circle.
    pub round: Option<Axis<S>>,
    /// An edge's curve, whatever its shape.
    pub curve: Option<NurbCurve3D<S>>,
    /// Its own axes, if it has any: a datum's frame.
    pub frame: Option<CoordinateSystem<S>>,
    /// A face, whatever its shape and whatever body it is part of.
    pub face: bool,
    /// A solid, as a whole.
    pub solid: bool,
    /// A face of a sheet, standing on its own: what a solid can be cut
    /// with.
    pub sheet: bool,
    /// A sketch, as a whole: its regions, to sweep.
    pub sketch: bool,
}

impl<S: Scalar> Aspects<S> {
    /// What `entity` is in `part`. Fails if the part has no such entity.
    pub fn of(entity: &EntityRef, part: &Part<S>) -> GeopResult<Self> {
        let ctx = with_context!("resolving {entity}");
        if let Some((name, inner)) = entity.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            return Aspects::of(&inner, instance.part())
                .with_context(ctx)?
                .placed(&instance.pose);
        }
        let mut g = Aspects::default();
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
                g.face = true;
                g.sheet = matches!(
                    part.topology().body_of_face(id).with_context(ctx)?,
                    geop_core_topology::Body::Sheet(_)
                );
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
            EntityRef::Solid { name } => {
                part.solid_id(name).with_context(ctx)?;
                g.solid = true;
            }
            EntityRef::Sketch { name } => {
                part.sketch_id(name).with_context(ctx)?;
                g.sketch = true;
            }
            EntityRef::SketchPoint { sketch, point } => {
                let placed = part.sketch(part.sketch_id(sketch).with_context(ctx)?)?;
                placed.sketch.point(*point).with_context(ctx)?;
                let geometry = placed.sketch.enclose::<S>().with_context(ctx)?;
                g.point = Some(placed.plane.uv_to_xyz(&geometry.points[point]));
            }
            EntityRef::SketchCurve { sketch, curve } => {
                let placed = part.sketch(part.sketch_id(sketch).with_context(ctx)?)?;
                let kind = &placed.sketch.curve(*curve).with_context(ctx)?.kind;
                let geometry = placed.sketch.enclose::<S>().with_context(ctx)?;
                let at = |p| placed.plane.uv_to_xyz(&geometry.points[&p]);
                match *kind {
                    CurveKind::Line { start, end } => {
                        let (a, b) = (at(start), at(end));
                        g.line = Some(Axis::try_new(a, b.sub(&a)).with_context(ctx)?);
                    }
                    CurveKind::Circle { center, .. } => {
                        g.round = Some(Axis::try_new(at(center), *placed.plane.w())?);
                    }
                    // Arcs and splines fill no role yet: nothing picks them.
                    CurveKind::Arc { .. } | CurveKind::Spline { .. } => {}
                }
            }
        }
        Ok(g)
    }

    /// What it is once its part is moved by `pose`: a part placed in
    /// another is used as it is placed.
    pub fn placed(self, placement: &Pose<S>) -> GeopResult<Self> {
        let motion = placement.motion();
        let pose = &motion;
        let axis = |a: Axis<S>| Axis {
            point: pose.apply(&a.point),
            direction: pose.rotate(&a.direction),
        };
        let frame = |f: Option<CoordinateSystem<S>>| f.map(|f| pose.apply_frame(&f)).transpose();
        Ok(Aspects {
            point: self.point.map(|p| pose.apply(&p)),
            line: self.line.map(axis),
            plane: frame(self.plane)?,
            arc: self.arc.map(|arc| Arc {
                circle: Circle {
                    center: pose.apply(&arc.circle.center),
                    normal: pose.rotate(&arc.circle.normal),
                    radius: arc.circle.radius,
                },
                start: pose.apply(&arc.start),
                end: pose.apply(&arc.end),
            }),
            round: self.round.map(axis),
            curve: self.curve.map(|c| c.transform(pose)),
            frame: frame(self.frame)?,
            face: self.face,
            solid: self.solid,
            sheet: self.sheet,
            sketch: self.sketch,
        })
    }

    /// Every role it can fill.
    pub fn roles(&self) -> Vec<Role> {
        Role::ALL
            .into_iter()
            .filter(|role| role.fits(self))
            .collect()
    }
}

/// What a step needs an entity it builds on to be: what a pick looks for,
/// and what a datum construction needs as an input.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    /// A vertex, a datum point, a frame's origin, a sketch point.
    Point,
    /// A straight edge, a datum axis, a frame's axis, a sketch line.
    Line,
    /// A planar face, a datum plane, a frame's plane.
    Plane,
    /// Any edge.
    Edge,
    /// A circular edge.
    Circle,
    /// Something that turns around an axis: a circular edge, a cylindrical,
    /// conical or spherical face, a sketch circle.
    Round,
    /// Any face, of a solid or standing on its own.
    Face,
    /// A solid, as a whole.
    Solid,
    /// A face standing on its own, part of no solid.
    Sheet,
    /// A sketch, as a whole.
    Sketch,
}

impl Role {
    pub const ALL: [Role; 10] = [
        Role::Point,
        Role::Line,
        Role::Plane,
        Role::Edge,
        Role::Circle,
        Role::Round,
        Role::Face,
        Role::Solid,
        Role::Sheet,
        Role::Sketch,
    ];

    pub fn fits<S: Scalar>(self, aspects: &Aspects<S>) -> bool {
        match self {
            Role::Point => aspects.point.is_some(),
            Role::Line => aspects.line.is_some(),
            Role::Plane => aspects.plane.is_some(),
            Role::Edge => aspects.curve.is_some(),
            Role::Circle => aspects.arc.is_some(),
            Role::Round => aspects.round.is_some(),
            Role::Face => aspects.face,
            Role::Solid => aspects.solid,
            Role::Sheet => aspects.sheet,
            Role::Sketch => aspects.sketch,
        }
    }

    /// As a requirement reads: `a point`.
    pub fn describe(self) -> &'static str {
        match self {
            Role::Point => "a point",
            Role::Line => "a line",
            Role::Plane => "a plane",
            Role::Edge => "an edge",
            Role::Circle => "a circular edge",
            Role::Round => "a circular edge or a round face",
            Role::Face => "a face",
            Role::Solid => "a solid",
            Role::Sheet => "a face on its own",
            Role::Sketch => "a sketch",
        }
    }

    /// Its name, as a selection lists what an entity can be used as.
    pub fn name(self) -> &'static str {
        match self {
            Role::Point => "point",
            Role::Line => "line",
            Role::Plane => "plane",
            Role::Edge => "edge",
            Role::Circle => "circle",
            Role::Round => "round",
            Role::Face => "face",
            Role::Solid => "solid",
            Role::Sheet => "sheet",
            Role::Sketch => "sketch",
        }
    }
}

/// `roles` in words, as a requirement reads: `a point or a plane`.
pub fn describe_roles(roles: &[Role], joiner: &str) -> String {
    roles
        .iter()
        .map(|r| r.describe())
        .collect::<Vec<_>>()
        .join(joiner)
}
