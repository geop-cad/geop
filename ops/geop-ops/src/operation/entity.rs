//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face, datum, solid or sketch of the part, by name — or of a part
//! placed in it, by its name there behind the instance's
//! ([`INSTANCE_SEPARATOR`]).

use crate::Part;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_sketch::{CurveId, PointId};
use geop_core_topology::Body;
use serde::{Deserialize, Serialize};

/// What separates an instance's name from the name of an entity of the part
/// it places: `bolt/extrude(head,end)` is the face `extrude(head,end)` of the
/// part placed as `bolt` — where it is placed — and `asm/bolt/...` reaches
/// into an instance of an instance. Instances are named by their step's id,
/// which cannot contain it (see [`crate::validate_operation_id`]), so the
/// first one always ends the instance's name.
pub const INSTANCE_SEPARATOR: char = '/';

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
    /// A 3-D sketch, as a whole: its curves.
    Sketch3d {
        name: String,
    },
    /// A curve of a sketch, by its id in the sketch.
    SketchCurve {
        sketch: String,
        curve: CurveId,
    },
    /// A point of a sketch, by its id in the sketch.
    SketchPoint {
        sketch: String,
        point: PointId,
    },
    /// What a step did by combining tools with a solid — a cut, a boss, a
    /// hole — by the step's id (see [`crate::part::Feature`]): picked by a
    /// face it made.
    Feature {
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
            EntityRef::SketchCurve { sketch, curve } => format!("{sketch} {curve}"),
            EntityRef::SketchPoint { sketch, point } => format!("{sketch} {point}"),
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Datum { name, .. }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name }
            | EntityRef::Sketch3d { name }
            | EntityRef::Feature { name } => name.clone(),
        }
    }
}

impl EntityRef {
    /// The name it is found by: its own, or its sketch's.
    fn name_mut(&mut self) -> &mut String {
        match self {
            EntityRef::Vertex { name }
            | EntityRef::Edge { name }
            | EntityRef::Face { name }
            | EntityRef::Datum { name, .. }
            | EntityRef::Solid { name }
            | EntityRef::Sketch { name }
            | EntityRef::Sketch3d { name }
            | EntityRef::Feature { name } => name,
            EntityRef::SketchCurve { sketch, .. } | EntityRef::SketchPoint { sketch, .. } => sketch,
        }
    }

    /// If it lies in a part placed in this one: the instance's name, and the
    /// entity as the placed part names it.
    pub fn split_instance(&self) -> Option<(String, EntityRef)> {
        let mut inner = self.clone();
        let name = inner.name_mut();
        let (instance, rest) = name.split_once(INSTANCE_SEPARATOR)?;
        let instance = instance.to_string();
        *name = rest.to_string();
        Some((instance, inner))
    }

    /// The entity, of the part placed as `instance`, as the part it is
    /// placed in names it.
    pub fn in_instance(&self, instance: &str) -> EntityRef {
        let mut outer = self.clone();
        let name = outer.name_mut();
        *name = format!("{instance}{INSTANCE_SEPARATOR}{name}");
        outer
    }

    /// Whether it is `scope` or part of it: a curve or a point of a sketch,
    /// or a face, an edge or a vertex of a solid — `solid`, the name of the
    /// solid it bounds, if any, which only the part knows.
    pub fn lies_in(&self, scope: &EntityRef, solid: Option<&str>) -> bool {
        match (self, scope) {
            (
                EntityRef::SketchCurve { sketch, .. } | EntityRef::SketchPoint { sketch, .. },
                EntityRef::Sketch { name },
            ) => sketch == name,
            (
                EntityRef::Face { .. } | EntityRef::Edge { .. } | EntityRef::Vertex { .. },
                EntityRef::Solid { name },
            ) => solid == Some(name.as_str()),
            _ => self == scope,
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
            EntityRef::Sketch3d { name } => write!(f, "3-D sketch {name:?}"),
            EntityRef::Feature { name } => write!(f, "feature {name:?}"),
            EntityRef::SketchCurve { sketch, curve } => {
                write!(f, "curve {curve} of sketch {sketch:?}")
            }
            EntityRef::SketchPoint { sketch, point } => {
                write!(f, "point {point} of sketch {sketch:?}")
            }
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
    /// The datum it refers to in `part` — for a component of a frame, that
    /// component as a datum of its own. Fails if it is no datum of the
    /// part.
    pub fn resolve_datum<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Datum<S>> {
        let ctx = with_context!("resolving {self}");
        if let Some((name, inner)) = self.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            let datum = inner.resolve_datum(instance.part()).with_context(ctx)?;
            return Ok(Datum {
                kind: datum.kind,
                frame: instance.pose.motion().apply_frame(&datum.frame)?,
            });
        }
        let EntityRef::Datum { name, component } = self else {
            return Err(GeopError::new(format!("{self} is not a datum")));
        };
        let datum = part.datum(part.datum_id(name).with_context(ctx)?)?;
        match component {
            Some(component) => datum.component(*component).with_context(ctx),
            None => Ok(datum.clone()),
        }
    }

    /// The plane it lies in, as a frame with `u`/`v` a sketch's `x`/`y` on
    /// it and `w = u x v` its normal: a planar face's — normal pointing out
    /// of its solid, origin the world origin's projection onto it and `u`
    /// the world axis most parallel to it, so sketches on parallel faces
    /// line up — a datum plane's, a frame's plane's, or a sketch's. Fails
    /// for anything else.
    pub fn resolve_plane<S: Scalar>(&self, part: &Part<S>) -> GeopResult<CoordinateSystem<S>> {
        let ctx = with_context!("resolving the plane of {self}");
        if let Some((name, inner)) = self.split_instance() {
            let instance = part.instance(part.instance_id(&name).with_context(ctx)?)?;
            let plane = inner.resolve_plane(instance.part()).with_context(ctx)?;
            return instance.pose.motion().apply_frame(&plane);
        }
        match self {
            EntityRef::Face { name } => {
                let id = part.face_id(name).with_context(ctx)?;
                let surface = &part.topology().get_face(id).with_context(ctx)?.surface;
                match surface.as_plane().with_context(ctx)? {
                    Some(plane) => frame_along(plane.project(&Vector3::zero()), &plane.normal),
                    None => Err(GeopError::new(format!("{self} is not planar"))),
                }
            }
            EntityRef::Datum { .. } => {
                let datum = self.resolve_datum(part)?;
                match datum.kind {
                    DatumKind::Plane => Ok(datum.frame),
                    _ => Err(GeopError::new(format!("{self} is not a plane"))),
                }
            }
            EntityRef::Sketch { name } => {
                let id = part.sketch_id(name).with_context(ctx)?;
                Ok(part.sketch(id).with_context(ctx)?.plane.clone())
            }
            _ => Err(GeopError::new(format!("{self} is not planar"))),
        }
    }

    /// The body it refers to in `part`: a solid by its name, a sheet —
    /// faces standing on their own, which have no name as a whole — by one
    /// of its faces. A face of a solid stands for the whole solid. Fails
    /// for anything else, and for a body of a placed part, which is that
    /// part's own program's to change.
    pub fn resolve_body<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Body> {
        if self.split_instance().is_some() {
            return Err(GeopError::new(format!(
                "{self} belongs to a placed part; change it in the part's own program"
            )));
        }
        match self {
            EntityRef::Solid { name } => Ok(Body::Solid(part.solid_id(name)?)),
            EntityRef::Face { name } => part.topology().body_of_face(part.face_id(name)?),
            other => Err(GeopError::new(format!(
                "{other} is no body: pick a solid or a face standing on its own"
            ))),
        }
    }
}
