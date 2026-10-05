//! [`StepEditEvent`]: what the user did, as an operation receives it.

use geop_core_math::{
    primitives::Ray,
    scalars::{Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use super::GizmoDrag;
use crate::operation::EntityRef;

/// How far from its ray a [`Pointer`] reaches: what counts as under it, at
/// any distance along the ray. Everything a viewer draws at a constant size
/// on screen — a handle, a label, a frame datum — is laid out in reaches,
/// so it is hit where it is drawn however far away or zoomed in the view is.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Reach<S: Scalar> {
    /// Seen in perspective: a cone around the ray, from its origin — the
    /// eye — widening by `slope` per unit of distance.
    Cone {
        #[serde(with = "as_f64")]
        slope: S,
    },
    /// Seen orthographically: a tube of `radius` around the ray.
    Tube {
        #[serde(with = "as_f64")]
        radius: S,
    },
}

impl<S: Scalar> Reach<S> {
    /// How far from the ray it reaches, `t` along it.
    pub fn at(&self, t: S) -> S {
        match *self {
            Reach::Cone { slope } => slope.mul(t),
            Reach::Tube { radius } => radius,
        }
    }
}

/// Where the pointer is: the ray from the eye through it, and how far from
/// that ray it reaches.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Pointer<S: Scalar> {
    pub ray: Ray<S>,
    pub reach: Reach<S>,
}

impl<S: Scalar> Pointer<S> {
    /// `reaches` of the pointer's reach, `t` along its ray.
    pub fn reach_at(&self, reaches: f64, t: S) -> S {
        self.reach.at(t).mul(S::from_f64(reaches))
    }

    /// Whether something `dist` from the ray, `t` along it, could lie within
    /// `reaches` of it.
    pub fn within(&self, dist: S, t: S, reaches: f64) -> bool {
        !dist.definitely_greater(self.reach_at(reaches, t))
    }
}

/// Which mouse button.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Button {
    #[default]
    Primary,
    Secondary,
}

/// What a field was set to — or that it was pressed: in the dialog, by a
/// pick in the viewport, or by dragging a handle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", content = "value", rename_all = "snake_case")]
pub enum Value {
    /// A button, a list item or a reference field was pressed.
    Press,
    /// A list item's remove button was pressed.
    Remove,
    Bool(bool),
    Number(f64),
    /// Text typed: a value given as a formula, a colour.
    Text(String),
    /// An action, or a select's option, by its value.
    Choice(String),
    /// The entity at this index was taken out of a reference field.
    RemoveAt(usize),
    /// Everything was taken out of a reference field.
    Clear,
    /// What a reference field holds now: all an operation is ever sent for
    /// one (see [`super::Reference`]).
    Entities(Vec<EntityRef>),
}

/// One thing the user did while editing a step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum StepEditEvent<S: Scalar> {
    /// The dialog field `key` was used.
    Dialog { key: String, value: Value },
    /// The pointer moved over the viewport with no button held — `shift`
    /// held or not, which turns snapping off.
    Hover {
        pointer: Pointer<S>,
        #[serde(default)]
        shift: bool,
    },
    /// The pointer left the viewport.
    Leave,
    /// A click in the viewport: a press and release without moving.
    Click {
        pointer: Pointer<S>,
        #[serde(default)]
        button: Button,
        /// The second click of a double click.
        #[serde(default)]
        double: bool,
        #[serde(default)]
        shift: bool,
    },
    /// A drag with the primary button that started where the last
    /// presentation offered a grab (see [`super::Presentation::grab`]): the
    /// ray where it went down and the ray where the pointer is now. `done`
    /// on release.
    Drag {
        from: Pointer<S>,
        to: Pointer<S>,
        #[serde(default)]
        done: bool,
        #[serde(default)]
        shift: bool,
    },
    /// A key, as the browser names it: `l`, `Escape`, `Enter`, `Delete`.
    Key { key: String },
}

impl<S: Scalar> StepEditEvent<S> {
    /// The dialog field this event uses, and its value.
    pub fn dialog(&self) -> Option<(&str, &Value)> {
        match self {
            StepEditEvent::Dialog { key, value } => Some((key, value)),
            _ => None,
        }
    }

    /// Where the pointer is, for any event that has one — for a drag, where
    /// it is now.
    pub fn pointer(&self) -> Option<&Pointer<S>> {
        match self {
            StepEditEvent::Hover { pointer, .. } | StepEditEvent::Click { pointer, .. } => {
                Some(pointer)
            }
            StepEditEvent::Drag { to, .. } => Some(to),
            _ => None,
        }
    }
}

/// A pointer or key event the editor passes on to an operation, having
/// taken what is its own: picks, selecting, dragging handles (see
/// [`super::StepEditor`]).
#[derive(Clone, Debug, PartialEq)]
pub enum CanvasEvent<S: Scalar> {
    /// The pointer moved over the viewport with no button held, `shift`
    /// held or not.
    Hover { pointer: Pointer<S>, shift: bool },
    /// The pointer left the viewport.
    Leave,
    /// A click that selected nothing: a secondary one, a double one, one
    /// while the operation has a tool in hand, one on nothing selectable.
    Click {
        pointer: Pointer<S>,
        button: Button,
        double: bool,
        shift: bool,
    },
    /// The draggable visual `key` dragged in the plane worked in, from
    /// where it was grabbed to where the pointer is now — `pointer`, with
    /// `shift` held or not. `done` on release.
    Move {
        key: String,
        from: Vector3<S>,
        to: Vector3<S>,
        pointer: Pointer<S>,
        done: bool,
        shift: bool,
    },
    /// A drag of the tool in hand, where it strokes (see
    /// [`super::InHand::Strokes`]): the ray where it went down and the ray
    /// where the pointer is now. `done` on release.
    Stroke {
        from: Pointer<S>,
        to: Pointer<S>,
        done: bool,
        shift: bool,
    },
    /// A drag of the form's gizmo (see [`super::Gizmo`]): what it did
    /// since it started, to apply to the step's arguments as they were
    /// then — which they are again whenever this is sent. `done` on
    /// release.
    Gizmo { drag: GizmoDrag<S>, done: bool },
    /// A key, as the browser names it.
    Key { key: String },
}
