//! Reference geometry: what a sketch is given rather than drawn — its own
//! origin and axes, and entities of the part projected into its plane — as
//! fixed points and curves (see [`geop_core_sketch::sketch`]), which
//! constraints can then be measured against.
//!
//! A projection refers to what it projects by name ([`EntityRef`]), never by
//! position: every time the sketch is built, each reference is brought up
//! to date with the part as it is then ([`Reference::update`]), so a sketch
//! dimensioned against a projected edge follows that edge when an earlier
//! step moves it — and the sketch's ids for what it projected stay the
//! same, keyed by the names of the vertices and edges they come from.
//!
//! Projecting reads what an edge *is* from its NURBS curve: a straight one
//! becomes a line, a circular one in a plane parallel to the sketch an arc
//! or a circle, so that it can be constrained like one — anything else a
//! spline, the exact NURBS the edge projects to.

use std::collections::BTreeMap;

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use geop_core_sketch::{CurveId, PointId, SplineShape};
use geop_core_topology::CoedgeGeometry;
use geop_ops::{Design, EntityRef, Part, RefId};
use serde::{Deserialize, Serialize};

use crate::{CurveKind, Sketch};

/// Where reference geometry comes from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Source {
    /// The sketch's own origin, and its `x` and `y` axes as construction
    /// lines through it.
    Frame,
    /// A vertex, an edge or a face of the part — its boundary — projected
    /// into the sketch's plane along the plane's normal.
    Projection { entity: EntityRef },
}

/// Reference geometry in a sketch: where it comes from, and the fixed
/// points and curves it is there, each by what of its source it is — the
/// name of a vertex or an edge, `origin`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Reference {
    #[serde(flatten)]
    pub source: Source,
    #[serde(default)]
    pub points: BTreeMap<String, PointId>,
    #[serde(default)]
    pub curves: BTreeMap<String, CurveId>,
}

/// A curve of reference geometry, its points by their keys.
#[derive(Clone, Debug, PartialEq)]
enum Shape {
    Line {
        start: String,
        end: String,
    },
    Arc {
        start: String,
        end: String,
        sweep: f64,
    },
    Circle {
        center: String,
        radius: f64,
    },
    Spline {
        control_points: Vec<String>,
        shape: SplineShape<Design>,
    },
}

/// What reference geometry is right now: its points where they are, and
/// its curves, all by key.
#[derive(Clone, Debug, Default, PartialEq)]
struct Geometry {
    points: BTreeMap<String, [f64; 2]>,
    curves: BTreeMap<String, Shape>,
}

/// The keys of the frame's origin and axes.
pub const ORIGIN: &str = "origin";
pub const X_AXIS: &str = "x";
pub const Y_AXIS: &str = "y";

impl Reference {
    /// A reference to `source`, not in any sketch yet: [`Reference::update`]
    /// puts it there.
    pub fn new(source: Source) -> Self {
        Self {
            source,
            points: BTreeMap::new(),
            curves: BTreeMap::new(),
        }
    }

    /// Whether `point` is one of its points.
    pub fn has_point(&self, point: PointId) -> bool {
        self.points.values().any(|&p| p == point)
    }

    /// Whether `curve` is one of its curves.
    pub fn has_curve(&self, curve: CurveId) -> bool {
        self.curves.values().any(|&c| c == curve)
    }

    /// The sketch's own origin and axes, added to `sketch`.
    pub fn frame(sketch: &mut Sketch) -> Self {
        let mut frame = Self::new(Source::Frame);
        frame.sync(sketch, &frame_geometry());
        frame
    }

    /// Brings its points and curves in `sketch` up to date with what it
    /// comes from in `part`, the sketch lying in `plane`: each moved to
    /// where its source is now, those new to the source added, those gone
    /// from it removed — with whatever constraints were on them.
    pub fn update<S: Scalar>(
        &mut self,
        sketch: &mut Sketch,
        part: &Part<S>,
        plane: &CoordinateSystem<S>,
    ) -> GeopResult<()> {
        let geometry = match &self.source {
            Source::Frame => frame_geometry(),
            Source::Projection { entity } => {
                let ctx = with_context!("projecting {entity} into the sketch");
                project(entity, part, plane).with_context(ctx)?
            }
        };
        self.sync(sketch, &geometry);
        Ok(())
    }

    /// Makes its points and curves in `sketch` `geometry`.
    fn sync(&mut self, sketch: &mut Sketch, geometry: &Geometry) {
        let construction = matches!(self.source, Source::Frame);
        for (key, &[x, y]) in &geometry.points {
            let at = (Design::from_f64(x), Design::from_f64(y));
            match self.points.get(key).and_then(|p| sketch.points.get_mut(p)) {
                Some(point) => {
                    (point.x, point.y) = at;
                    point.fixed = true;
                }
                None => {
                    let id = sketch.add_fixed_point(at.0, at.1);
                    self.points.insert(key.clone(), id);
                }
            }
        }
        let point = |key: &String| self.points[key];
        for (key, shape) in &geometry.curves {
            let kind = match shape {
                Shape::Line { start, end } => CurveKind::Line {
                    start: point(start),
                    end: point(end),
                },
                Shape::Arc { start, end, sweep } => CurveKind::Arc {
                    start: point(start),
                    end: point(end),
                    sweep: Design::from_f64(*sweep),
                },
                Shape::Circle { center, radius } => CurveKind::Circle {
                    center: point(center),
                    radius: Design::from_f64(*radius),
                },
                Shape::Spline {
                    control_points,
                    shape,
                } => CurveKind::Spline {
                    control_points: control_points.iter().map(point).collect(),
                    shape: Some(shape.clone()),
                },
            };
            match self.curves.get(key).and_then(|c| sketch.curves.get_mut(c)) {
                Some(curve) => {
                    curve.kind = kind;
                    curve.fixed = true;
                }
                None => {
                    let id = sketch.add_curve(kind);
                    let curve = sketch.curves.get_mut(&id).expect("just added");
                    curve.fixed = true;
                    curve.construction = construction;
                    self.curves.insert(key.clone(), id);
                }
            }
        }
        let gone_points: Vec<PointId> = self
            .points
            .iter()
            .filter(|(key, _)| !geometry.points.contains_key(*key))
            .map(|(_, &p)| p)
            .collect();
        let gone_curves: Vec<CurveId> = self
            .curves
            .iter()
            .filter(|(key, _)| !geometry.curves.contains_key(*key))
            .map(|(_, &c)| c)
            .collect();
        sketch.remove(&gone_points, &gone_curves, &[]);
        self.points
            .retain(|key, _| geometry.points.contains_key(key));
        self.curves
            .retain(|key, _| geometry.curves.contains_key(key));
    }

    /// Takes its points and curves out of `sketch`, with whatever
    /// constraints were on them.
    pub fn remove_from(&self, sketch: &mut Sketch) {
        let points: Vec<PointId> = self.points.values().copied().collect();
        let curves: Vec<CurveId> = self.curves.values().copied().collect();
        sketch.remove(&points, &curves, &[]);
    }
}

/// The sketch's own origin and axes.
fn frame_geometry() -> Geometry {
    let line = |end: &str| Shape::Line {
        start: ORIGIN.into(),
        end: end.into(),
    };
    Geometry {
        points: BTreeMap::from([
            (ORIGIN.into(), [0.0, 0.0]),
            (X_AXIS.into(), [1.0, 0.0]),
            (Y_AXIS.into(), [0.0, 1.0]),
        ]),
        curves: BTreeMap::from([(X_AXIS.into(), line(X_AXIS)), (Y_AXIS.into(), line(Y_AXIS))]),
    }
}

/// `entity` of `part` projected into `plane` along its normal, every key
/// the name of the vertex or edge it comes from — behind the instance's
/// name, for an entity of a part placed in this one.
fn project<S: Scalar>(
    entity: &EntityRef,
    part: &Part<S>,
    plane: &CoordinateSystem<S>,
) -> GeopResult<Geometry> {
    if let Some((name, inner)) = entity.split_instance() {
        // Projecting what is placed onto the plane is projecting it, where
        // its own part has it, onto the plane where that part has it.
        let instance = part.instance(part.instance_id(&name)?)?;
        let local = instance.pose.inverse().motion().apply_frame(plane)?;
        let inner = project(&inner, instance.part(), &local)?;
        let prefix = |key: String| inner_name(&name, &key);
        return Ok(Geometry {
            points: inner
                .points
                .into_iter()
                .map(|(k, p)| (prefix(k), p))
                .collect(),
            curves: inner
                .curves
                .into_iter()
                .map(|(k, shape)| (prefix(k), shape.renamed(&|key| inner_name(&name, key))))
                .collect(),
        });
    }
    let topology = part.topology();
    let name = |id: RefId| {
        part.name_of(id)
            .map(str::to_string)
            .ok_or_else(|| GeopError::new("an entity of the part has no name"))
    };
    let mut geometry = Geometry::default();
    let mut edges = Vec::new();
    match entity {
        EntityRef::Vertex { name: vertex } => {
            let point = topology.get_vertex(part.vertex_id(vertex)?)?.point;
            geometry
                .points
                .insert(vertex.clone(), plain(&in_plane(plane, &point)));
        }
        EntityRef::Edge { name } => edges.push(part.edge_id(name)?),
        EntityRef::Face { name: face } => {
            for coedge in topology.iterate_face_coedges(part.face_id(face)?) {
                match topology.get_coedge(coedge)?.geometry {
                    CoedgeGeometry::Edge(edge) => edges.push(edge),
                    CoedgeGeometry::Vertex(vertex) => {
                        let point = topology.get_vertex(vertex)?.point;
                        geometry
                            .points
                            .insert(name(vertex.into())?, plain(&in_plane(plane, &point)));
                    }
                }
            }
        }
        other => {
            return Err(GeopError::new(format!(
                "{other} cannot be projected: pick a vertex, an edge or a face"
            )));
        }
    }
    for edge_id in edges {
        let edge = topology.get_edge(edge_id)?;
        let key = name(edge_id.into())?;
        let (start, end) = (
            name(edge.start_vertex.into())?,
            name(edge.end_vertex.into())?,
        );
        for (vertex, at) in [(edge.start_vertex, &start), (edge.end_vertex, &end)] {
            let point = topology.get_vertex(vertex)?.point;
            geometry
                .points
                .insert(at.clone(), plain(&in_plane(plane, &point)));
        }
        let ctx = with_context!("edge {key:?}");
        if let Some(shape) =
            project_curve(&edge.curve, plane, &key, start, end, &mut geometry.points)
                .with_context(ctx)?
        {
            geometry.curves.insert(key, shape);
        }
    }
    Ok(geometry)
}

/// The name of `key` of a part placed as `instance`, in the part placing it.
fn inner_name(instance: &str, key: &str) -> String {
    format!("{instance}{}{key}", geop_ops::operation::INSTANCE_SEPARATOR)
}

impl Shape {
    /// The same curve, its point keys renamed by `rename`.
    fn renamed(self, rename: &dyn Fn(&str) -> String) -> Shape {
        match self {
            Shape::Line { start, end } => Shape::Line {
                start: rename(&start),
                end: rename(&end),
            },
            Shape::Arc { start, end, sweep } => Shape::Arc {
                start: rename(&start),
                end: rename(&end),
                sweep,
            },
            Shape::Circle { center, radius } => Shape::Circle {
                center: rename(&center),
                radius,
            },
            Shape::Spline {
                control_points,
                shape,
            } => Shape::Spline {
                control_points: control_points.iter().map(|k| rename(k)).collect(),
                shape,
            },
        }
    }
}

/// `p` in `plane`'s `u`/`v` coordinates: projected along its normal.
fn in_plane<S: Scalar>(plane: &CoordinateSystem<S>, p: &Vector3<S>) -> Vector2<S> {
    let q = plane.to_uvw(p);
    Vector2::from_array([q[0], q[1]])
}

/// A position as design data holds it: its midpoint.
fn plain<S: Scalar>(p: &Vector2<S>) -> [f64; 2] {
    [p[0].to_f64(), p[1].to_f64()]
}

/// The edge curve `curve` — from the vertex keyed `start` to the one keyed
/// `end` — projected into `plane`, reverse-engineered from its NURBS: a
/// straight curve a line, a circular one in a plane parallel to the
/// sketch's an arc or a circle, anything else the spline it projects to
/// exactly. `None` for a curve that projects to a point. Points it needs
/// beyond its end points — a circle's center, a spline's inner control
/// points — are added to `points`, keyed behind `key`.
fn project_curve<S: Scalar>(
    curve: &NurbCurve3D<S>,
    plane: &CoordinateSystem<S>,
    key: &str,
    start: String,
    end: String,
    points: &mut BTreeMap<String, [f64; 2]>,
) -> GeopResult<Option<Shape>> {
    let (t0, t1) = curve.domain();
    let (a, b) = (
        in_plane(plane, &curve.evaluate(t0)?),
        in_plane(plane, &curve.evaluate(t1)?),
    );
    if curve.as_line()?.is_some() {
        // Along the plane's normal, a line is only a point.
        return Ok((!a.could_be_equal(&b)).then_some(Shape::Line { start, end }));
    }
    if let Some(arc) = curve.as_arc()? {
        let normal = arc.circle.normal;
        let along = normal.prod_dot(plane.w());
        if normal
            .prod_cross(plane.w())
            .could_be_equal(&Vector3::zero())
        {
            let center = format!("{key}#center");
            points.insert(center.clone(), plain(&in_plane(plane, &arc.circle.center)));
            let radius = arc.circle.radius.to_f64();
            if arc.could_be_closed() {
                return Ok(Some(Shape::Circle { center, radius }));
            }
            // Seen from the side its normal points to, the arc turns
            // counter-clockwise; from the other side, clockwise.
            let sweep = if along.definitely_greater(S::ZERO) {
                arc.sweep()
            } else {
                -arc.sweep()
            };
            return Ok(Some(Shape::Arc { start, end, sweep }));
        }
    }
    // Any other curve, as the NURBS it projects to: projecting along the
    // normal is affine, so it moves the control points and keeps the
    // weights and knots — exactly.
    let n = curve.control_points.len();
    let degree = curve.degree;
    let knots = &curve.knot_vector;
    let clamped = |ends: &[S]| ends.iter().all(|k| k.could_be_equal(ends[0]));
    if !clamped(&knots[..=degree]) || !clamped(&knots[n..]) {
        return Err(GeopError::new(format!(
            "its curve is not clamped at its ends: knots {knots:?}"
        )));
    }
    let mut control_points = vec![start];
    for (i, cp) in curve.control_points.iter().enumerate().take(n - 1).skip(1) {
        let w = cp[3];
        let p = Vector3::from_array([cp[0].div(w)?, cp[1].div(w)?, cp[2].div(w)?]);
        let inner = format!("{key}#{i}");
        points.insert(inner.clone(), plain(&in_plane(plane, &p)));
        control_points.push(inner);
    }
    control_points.push(end);
    Ok(Some(Shape::Spline {
        control_points,
        shape: SplineShape {
            degree,
            knots: knots.iter().map(|k| Design::from_f64(k.to_f64())).collect(),
            weights: curve
                .control_points
                .iter()
                .map(|cp| Design::from_f64(cp[3].to_f64()))
                .collect(),
        },
    }))
}
