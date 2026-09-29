//! [`Presentation`]: what an operation shows for a step — its dialog and
//! the [`Visual`]s it draws in the viewport.

use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar, vector::Vector3};
use serde::Serialize;

use super::{Dialog, Pointer, Target};
use crate::operation::EntityRef;

/// `v` in plain `f64`.
pub(crate) fn to_f64<S: Scalar>(v: &Vector3<S>) -> [f64; 3] {
    [v[0].to_f64(), v[1].to_f64(), v[2].to_f64()]
}

/// A plane with axes, in plain `f64`, as drawn: `(x, y)` in it lies at
/// `origin + x u + y v`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Frame {
    pub origin: [f64; 3],
    pub u: [f64; 3],
    pub v: [f64; 3],
    pub normal: [f64; 3],
}

impl Frame {
    pub fn of<S: Scalar>(frame: &CoordinateSystem<S>) -> Self {
        Frame {
            origin: to_f64(frame.origin()),
            u: to_f64(frame.u()),
            v: to_f64(frame.v()),
            normal: to_f64(frame.w()),
        }
    }

    /// The point `(x, y)` of the plane.
    pub fn to_world(&self, [x, y]: [f64; 2]) -> [f64; 3] {
        [0, 1, 2].map(|k| self.origin[k] + x * self.u[k] + y * self.v[k])
    }

    /// The area inside `outer` but outside `holes`, loops of the plane's
    /// points, as triangles — to draw a region filled. Empty for loops that
    /// do not triangulate.
    pub fn region(&self, outer: &[[f64; 2]], holes: &[Vec<[f64; 2]>]) -> Vec<[[f64; 3]; 3]> {
        type F = geop_core_math::scalars::ScalInF64;
        let v = |p: &[f64; 2]| geop_core_math::vector::Vector2::<F>::from_array(p.map(F::from_f64));
        let outer: Vec<_> = outer.iter().map(v).collect();
        let holes: Vec<Vec<_>> = holes.iter().map(|h| h.iter().map(v).collect()).collect();
        let back =
            |p: geop_core_math::vector::Vector2<F>| self.to_world([p[0].to_f64(), p[1].to_f64()]);
        geop_ops_rasterize::polygon_triangulate::triangulate_with_holes(&outer, &holes)
            .map(|tris| {
                tris.into_iter()
                    .map(|(a, b, c)| [back(a), back(b), back(c)])
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Where the pointer's ray meets the plane, in its coordinates, and how
    /// far along the ray that is. `None` if the ray runs along the plane or
    /// points away from it.
    pub fn at_pointer(&self, pointer: &Pointer) -> Option<([f64; 2], f64)> {
        let (t, point) = super::hit::ray_plane(pointer, self.origin, self.normal)?;
        let d = [0, 1, 2].map(|k| point[k] - self.origin[k]);
        let dot = |a: [f64; 3]| d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
        Some(([dot(self.u), dot(self.v)], t))
    }
}

/// What a visual is.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum Shape {
    Point {
        at: [f64; 3],
    },
    Polyline {
        points: Vec<[f64; 3]>,
    },
    /// A filled area.
    Triangles {
        triangles: Vec<[[f64; 3]; 3]>,
    },
    /// Text at `at`, moved on screen by `offset` pixels (right, up), so
    /// labels of one point do not stack.
    Label {
        at: [f64; 3],
        text: String,
        offset: [f64; 2],
    },
    /// Something to drag, at `at` — along `direction`, if it has one.
    Handle {
        at: [f64; 3],
        direction: Option<[f64; 3]>,
    },
}

/// What a visual means, which decides how it is drawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Style {
    /// Can still move: an under-constrained sketch entity.
    #[default]
    Free,
    /// Cannot move any more: a fully constrained one.
    Fixed,
    Selected,
    /// What a click would take.
    Hover,
    /// Part of what cannot be satisfied.
    Failed,
    /// Reference geometry that takes part in nothing but constraints.
    Construction,
    /// What a tool would draw next.
    Draft,
    /// A filled area: a closed region of a sketch.
    Region,
    /// A help line: a spline's control polygon.
    Guide,
    Handle,
}

/// Something an operation draws in the viewport, under a key the hit tests
/// report (see [`super::hit::hit_visuals`]).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Visual {
    pub key: String,
    #[serde(flatten)]
    pub shape: Shape,
    pub style: Style,
}

impl Visual {
    pub fn new(key: impl Into<String>, shape: Shape, style: Style) -> Self {
        Self {
            key: key.into(),
            shape,
            style,
        }
    }
}

/// What an operation shows for a step.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Presentation {
    pub dialog: Dialog,
    pub visuals: Vec<Visual>,
    /// Entities of the part to draw lit: what is picked, what a click would
    /// pick.
    pub highlights: Vec<EntityRef>,
    /// What a click in the viewport picks right now. The viewer shows
    /// reference geometry of these kinds — datums, the origin's axes and
    /// planes — and fades the rest.
    pub pickable: Vec<Target>,
    /// A plane to work in: the viewer faces it head on, stops orbiting, and
    /// draws a grid on it.
    pub focus: Option<Frame>,
    /// Whether a press where the pointer last hovered starts a drag (sent
    /// as [`super::Event::Drag`]) rather than moving the camera.
    pub grab: bool,
}
