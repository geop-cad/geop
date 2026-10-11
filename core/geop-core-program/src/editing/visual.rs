//! [`Visual`]: what an operation draws in the viewport — and
//! [`Presentation`], what an editor shows of a step.

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::{Scalar, as_f64},
    vector::Vector3,
};
use serde::Serialize;

use super::{Dialog, EntityRef, GizmoView, Role};

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
    /// A part placed in the part, by its instance's name, as the viewer
    /// draws it already (see [`crate::step_editor::Scene::hit_instance`]): lit when
    /// hovered or selected, hit where its faces are.
    Instance {
        name: String,
    },
    /// Three arrows from `at`, along `axes` — the world's `x`, `y` and `z`,
    /// or a frame's — drawn at a constant size on screen ([`TRIAD`] reaches
    /// long): what lines are drawn along, to see them by, or the frame a
    /// mate would put. Hit by nothing.
    Triad {
        at: Vector3<S>,
        axes: [Vector3<S>; 3],
    },
}

/// How long a [`Shape::Triad`]'s arrows are, in reaches.
pub const TRIAD: f64 = 6.0;

/// How far a sheet reaches: a square of `size` about `center`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Extent<S: Scalar> {
    pub center: Vector3<S>,
    #[serde(with = "as_f64")]
    pub size: S,
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
    /// Geometry that takes part in nothing but constraints.
    Construction,
    /// Geometry given from outside rather than drawn — a sketch's own axes,
    /// what it projects of the part: constrained against, never moved.
    Reference,
    /// What a tool would draw next.
    Draft,
    /// A filled area: a closed region of a sketch.
    Region,
    /// A help line: a spline's control polygon.
    Guide,
    Handle,
    /// Where what is drawn or dragged would snap to.
    Snap,
    /// What the tool in hand would remove.
    Removed,
    /// Text printed on a sheet of its own: a drawing's title block, its
    /// parts list, its views' captions.
    Paper,
}

/// A value asked for in place, at `at` in the viewport: a field of the
/// dialog's, by `key`, typed into as text — what is typed sent back as
/// [`super::Value::Text`] — starting from `value`.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Prompt<S: Scalar> {
    pub key: String,
    pub label: String,
    pub value: String,
    pub at: Vector3<S>,
}

/// Something an operation draws in the viewport, under a key the hit tests
/// report (see [`super::hit::hit_visuals`]) — and what the user can do with
/// it, which the editor handles alike for every operation (see
/// [`crate::step_editor::StepEditor`]).
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
    /// Each event of a drag of it reaches the step as it was when it was
    /// grabbed, as a gizmo's do: the operation applies the whole move, from
    /// where it was grabbed to where the pointer is, to that, and keeps no
    /// record of where the drag started. Without, it reaches the step as
    /// the drag's last event left it.
    #[serde(skip)]
    pub from_grab: bool,
}

impl<S: Scalar> Visual<S> {
    pub fn new(key: impl Into<String>, shape: Shape<S>, style: Style) -> Self {
        Self {
            key: key.into(),
            shape,
            style,
            selectable: false,
            draggable: false,
            from_grab: false,
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

    /// Draggable, each event of a drag applied to the step as it was when
    /// it was grabbed (see [`Visual::from_grab`]).
    pub fn draggable_from_grab(mut self) -> Self {
        self.draggable = true;
        self.from_grab = true;
        self
    }
}

/// What an editor shows for a step being edited: the operation's
/// [`super::Form`], and what the editor adds to it — what is picked, selected and
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
    /// A sheet of its own the step is edited on, in `focus`, and how far
    /// it reaches: the viewer shows the visuals alone, framed to it, and
    /// is sent no part to draw (see [`super::Form::sheet`]).
    pub sheet: Option<Extent<S>>,
    /// Whether a press where the pointer last hovered starts a drag (sent
    /// as [`super::StepEditEvent::Drag`]) rather than moving the camera.
    pub grab: bool,
    /// A value asked for in place.
    pub prompt: Option<Prompt<S>>,
    /// The gizmo to move, turn or scale by, as drawn.
    pub gizmo: Option<GizmoView<S>>,
}
