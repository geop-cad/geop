//! [`EntityRef`]: how a step refers to geometry it builds on — a vertex,
//! edge, face, datum, solid or sketch of the part, by name.

use crate::Part;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind},
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
}
