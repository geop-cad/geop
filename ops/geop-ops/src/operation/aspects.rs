//! [`Aspects`]: what an entity a step builds on can be used as — a point, a
//! line, a plane, an arc, something round, a curve, an edge of a face, a
//! face, a solid, a sheet, a sketch, or several of these at once — and the
//! [`Role`]s that lets it fill. Which of them an entity is decides what it
//! can be picked for, and what can be built on it.
//!
//! A curve is any: an edge of a solid, of a sheet or of a wire, or a curve
//! of a sketch. What runs along a curve takes any of them
//! ([`Role::Curve`]); what changes the faces at an edge — a fillet, a
//! flange — only an edge of a face ([`Role::Edge`]). A 3-D sketch is a
//! path as a whole; what it builds, edges and vertices of a wire, is
//! picked like any other.

use geop_core_geometry::{
    nurb_curve::NurbCurve3D,
    shape::{Arc, Axis, Circle},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, DatumKind, Pose},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::CurveKind;

use super::{EntityRef, Lookup, ResolveEntity};

pub use geop_core_program::editing::{Role, describe_roles};

/// Everything an entity can be used as. An entity is usually several at
/// once: a straight edge is a line, a curve and an edge, a circular edge an
/// arc, a curve, an edge and something round, a sketch's line a line and a
/// curve, a datum point a point and a frame, a datum frame the same.
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
    /// sketch's arc or circle.
    pub round: Option<Axis<S>>,
    /// The curve it runs along, whatever its shape: an edge's — of a face
    /// or of a wire — or a sketch's curve's, in space.
    pub curve: Option<NurbCurve3D<S>>,
    /// An edge of a face, of a solid or of a sheet: where faces meet, or
    /// where a sheet ends.
    pub edge: bool,
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
    /// Curves to run along: a sketch's, or an edge of a solid or a face.
    pub path: bool,
    /// A feature: what a step did by combining tools with a solid.
    pub feature: bool,
}

impl<S: Scalar> Aspects<S> {
    /// Whether it can be used as `role`.
    pub fn fits(&self, role: Role) -> bool {
        let aspects = self;
        match role {
            Role::Point => aspects.point.is_some(),
            Role::Line => aspects.line.is_some(),
            Role::Plane => aspects.plane.is_some(),
            Role::Edge => aspects.edge,
            Role::Curve => aspects.curve.is_some(),
            Role::Circle => aspects.arc.is_some(),
            Role::Round => aspects.round.is_some(),
            Role::Face => aspects.face,
            Role::Solid => aspects.solid,
            Role::Sheet => aspects.sheet,
            Role::Sketch => aspects.sketch,
            Role::Path => aspects.path,
            Role::Feature => aspects.feature,
            Role::Frame => {
                aspects.point.is_some()
                    || aspects.line.is_some()
                    || aspects.plane.is_some()
                    || aspects.round.is_some()
                    || aspects.frame.is_some()
            }
        }
    }

    /// What `entity` is in `part`. Fails if the part has no such entity.
    pub fn of(entity: &EntityRef, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Self> {
        let ctx = with_context!("resolving {entity}");
        // Of a placed part: what it is there, where that part is.
        if let Some(placed) = lookup.in_placed(entity) {
            let (mut inside, pose, local) = placed.with_context(ctx)?;
            return (Aspects::of(&local, &mut *inside).with_context(ctx)?).placed(&pose);
        }
        if entity.split_instance().is_some() {
            return Err(GeopError::new(format!(
                "{entity} is of no part placed here"
            )));
        }
        let mut g = Aspects::default();
        match entity {
            EntityRef::Vertex { name } => {
                let model = lookup.model();
                let id = model.vertex_id(name).with_context(ctx)?;
                g.point = Some(model.topology().get_vertex(id).with_context(ctx)?.point);
            }
            EntityRef::Edge { name } => {
                let model = lookup.model();
                let id = model.edge_id(name).with_context(ctx)?;
                let curve = model
                    .topology()
                    .get_edge(id)
                    .with_context(ctx)?
                    .curve
                    .clone();
                g.line = curve.as_line().with_context(ctx)?;
                g.arc = curve.as_arc().with_context(ctx)?;
                g.round = g.arc.as_ref().map(|arc| arc.circle.axis());
                g.curve = Some(curve);
                g.edge = model.topology().wire_of_edge(id).is_none();
                // An edge of a wire is no path on its own.
                g.path = g.edge;
            }
            EntityRef::Face { name } => {
                g.plane = entity.resolve_plane(lookup).ok();
                let model = lookup.model();
                let id = model.face_id(name).with_context(ctx)?;
                let surface = &model.topology().get_face(id).with_context(ctx)?.surface;
                g.round = surface.axis_of_revolution().with_context(ctx)?;
                g.face = true;
                g.sheet = matches!(
                    model.topology().body_of_face(id).with_context(ctx)?,
                    geop_core_topology::Body::Sheet(_)
                );
            }
            EntityRef::Datum { .. } => {
                let datum = entity.resolve_datum(lookup)?;
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
                lookup.model().solid_id(name).with_context(ctx)?;
                g.solid = true;
            }
            EntityRef::Sketch { name } => {
                lookup.sketch(name).with_context(ctx)?;
                g.sketch = true;
                g.path = true;
            }
            EntityRef::Sketch3d { name } => {
                lookup.sketch3d(name).with_context(ctx)?;
                g.path = true;
            }
            EntityRef::Feature { name } => {
                lookup.model().feature(name).with_context(ctx)?;
                g.feature = true;
            }
            EntityRef::Frame { on, at } => {
                let frame = Aspects::frame_on(on, at.as_deref(), lookup).with_context(ctx)?;
                g.point = Some(*frame.origin());
                g.frame = Some(frame);
            }
            EntityRef::SketchPoint { sketch, point } => {
                let placed = lookup.sketch(sketch).with_context(ctx)?;
                placed.sketch.point(*point).with_context(ctx)?;
                let geometry = placed.sketch.enclose::<S>().with_context(ctx)?;
                g.point = Some(placed.plane.uv_to_xyz(&geometry.points[point]));
            }
            EntityRef::SketchCurve { sketch, curve } => {
                let placed = lookup.sketch(sketch).with_context(ctx)?;
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
                    CurveKind::Arc { .. } | CurveKind::Spline { .. } => {}
                }
                let curve = placed.curve_in_space(*curve, &geometry).with_context(ctx)?;
                if matches!(kind, CurveKind::Arc { .. } | CurveKind::Circle { .. }) {
                    g.arc = curve.as_arc().with_context(ctx)?;
                    g.round = g.round.or(g.arc.as_ref().map(|arc| arc.circle.axis()));
                }
                g.curve = Some(curve);
            }
        }
        Ok(g)
    }

    /// The axis `entity` gives in `lookup`: a line's — a straight edge, a
    /// datum axis, a sketch line — or the one something round turns about:
    /// a circular edge, a cylindrical or conical face, a sketch circle.
    pub fn axis_of(
        entity: &EntityRef,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Axis<S>> {
        let aspects = Self::of(entity, lookup)?;
        aspects.line.or(aspects.round).ok_or_else(|| {
            GeopError::new(format!(
                "{entity} gives no axis: pick a straight edge, an axis, or a circular edge or round face to turn around its axis"
            ))
        })
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
            edge: self.edge,
            frame: frame(self.frame)?,
            face: self.face,
            solid: self.solid,
            sheet: self.sheet,
            sketch: self.sketch,
            path: self.path,
            feature: self.feature,
        })
    }

    /// Every role it can fill.
    pub fn roles(&self) -> Vec<Role> {
        Role::ALL
            .into_iter()
            .filter(|&role| self.fits(role))
            .collect()
    }
}
