//! [`Visual`]: what an operation draws in the viewport — and
//! [`Presentation`], what an editor shows of a step.

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use serde::Serialize;

use super::Dialog;
use crate::operation::{EntityRef, Role};

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
    /// The number field of the visual's key, as something to drag along
    /// `direction`: moving it by `direction` adds one to the field. Only the
    /// editor draws these, for a field with a handle (see
    /// [`super::Track`]).
    Handle {
        at: Vector3<S>,
        direction: Vector3<S>,
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
    /// Selected — drawn so by the editor, whatever its own style.
    Selected,
    /// What a click would take — drawn so by the editor.
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
/// report (see [`super::hit::hit_visuals`]) — and what the user can do with
/// it, which the editor handles alike for every operation (see
/// [`super::StepEditor`]).
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Visual<S: Scalar> {
    pub key: String,
    #[serde(flatten)]
    pub shape: Shape<S>,
    pub style: Style,
    /// A click on it selects it, or takes it out of the selection again.
    #[serde(skip)]
    pub selectable: bool,
    /// It can be dragged in the plane worked in: the operation is sent
    /// where to (see [`super::CanvasEvent::Move`]).
    #[serde(skip)]
    pub draggable: bool,
}

impl<S: Scalar> Visual<S> {
    pub fn new(key: impl Into<String>, shape: Shape<S>, style: Style) -> Self {
        Self {
            key: key.into(),
            shape,
            style,
            selectable: false,
            draggable: false,
        }
    }

    pub fn selectable(mut self) -> Self {
        self.selectable = true;
        self
    }

    pub fn draggable(mut self) -> Self {
        self.draggable = true;
        self
    }
}

/// What an editor shows for a step being edited: the operation's
/// [`Form`], and what the editor adds to it — what is picked, selected and
/// hovered, what a click would pick, and the handles of number fields.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Presentation<S: Scalar> {
    pub dialog: Dialog<S>,
    pub visuals: Vec<Visual<S>>,
    /// Entities of the part to draw lit: what is picked, what a click would
    /// pick.
    pub highlights: Vec<EntityRef>,
    /// What a click in the viewport picks right now: entities that can
    /// fill one of these roles. The viewer shows datums that can, and fades
    /// the rest.
    pub pickable: Vec<Role>,
    /// A plane to work in, its `u`/`v` plane: the viewer faces it head on,
    /// stops orbiting, and draws a grid on it.
    pub focus: Option<CoordinateSystem<S>>,
    /// Whether a press where the pointer last hovered starts a drag (sent
    /// as [`super::StepEditEvent::Drag`]) rather than moving the camera.
    pub grab: bool,
}
