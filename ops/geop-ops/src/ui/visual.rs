//! [`Presentation`]: what an operation shows for a step — its dialog and
//! the [`Visual`]s it draws in the viewport.

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use serde::Serialize;

use super::{Dialog, Target};
use crate::operation::EntityRef;

/// What a visual is.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "shape", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Shape<S: Scalar> {
    Point {
        at: Vector3<S>,
    },
    Polyline {
        points: Vec<Vector3<S>>,
    },
    /// A filled area.
    Triangles {
        triangles: Vec<[Vector3<S>; 3]>,
    },
    /// Text at `at`, moved by `offset` — measured in the pointer's reaches
    /// (see [`super::Reach`]), so it keeps its size on screen — so that
    /// labels of one point do not stack.
    Label {
        at: Vector3<S>,
        text: String,
        offset: Vector3<S>,
    },
    /// Something to drag, at `at` — along `direction`, if it has one.
    Handle {
        at: Vector3<S>,
        direction: Option<Vector3<S>>,
    },
}

impl<S: Scalar> Shape<S> {
    /// The area of `plane`'s `u`/`v` plane inside `outer` but outside
    /// `holes`, as triangles — to draw a region filled. No triangles for
    /// loops that do not triangulate.
    pub fn region(
        plane: &CoordinateSystem<S>,
        outer: &[Vector2<S>],
        holes: &[Vec<Vector2<S>>],
    ) -> Self {
        let triangles =
            geop_ops_rasterize::polygon_triangulate::triangulate_with_holes(outer, holes)
                .map(|tris| {
                    tris.into_iter()
                        .map(|(a, b, c)| [a, b, c].map(|p| plane.uv_to_xyz(&p)))
                        .collect()
                })
                .unwrap_or_default();
        Shape::Triangles { triangles }
    }
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
#[serde(bound = "S: Scalar")]
pub struct Visual<S: Scalar> {
    pub key: String,
    #[serde(flatten)]
    pub shape: Shape<S>,
    pub style: Style,
}

impl<S: Scalar> Visual<S> {
    pub fn new(key: impl Into<String>, shape: Shape<S>, style: Style) -> Self {
        Self {
            key: key.into(),
            shape,
            style,
        }
    }
}

/// What an operation shows for a step.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Presentation<S: Scalar> {
    pub dialog: Dialog,
    pub visuals: Vec<Visual<S>>,
    /// Entities of the part to draw lit: what is picked, what a click would
    /// pick.
    pub highlights: Vec<EntityRef>,
    /// What a click in the viewport picks right now. The viewer shows
    /// datums of these kinds and fades the rest.
    pub pickable: Vec<Target>,
    /// A plane to work in, its `u`/`v` plane: the viewer faces it head on,
    /// stops orbiting, and draws a grid on it.
    pub focus: Option<CoordinateSystem<S>>,
    /// Whether a press where the pointer last hovered starts a drag (sent
    /// as [`super::Event::Drag`]) rather than moving the camera.
    pub grab: bool,
}

impl<S: Scalar> Default for Presentation<S> {
    fn default() -> Self {
        Self {
            dialog: Dialog::default(),
            visuals: Vec::new(),
            highlights: Vec::new(),
            pickable: Vec::new(),
            focus: None,
            grab: false,
        }
    }
}
