//! [`Event`]: what the user did, as an operation receives it.

use geop_core_math::{
    primitives::Ray,
    scalars::{Scalar, as_f64},
};
use serde::{Deserialize, Serialize};

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

/// What a dialog control was set to — or that it was pressed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", content = "value", rename_all = "snake_case")]
pub enum DialogValue {
    /// A button, or a list item, was pressed.
    Press,
    /// A list item's remove button was pressed.
    Remove,
    Bool(bool),
    Number(f64),
    /// A select's option, by its value.
    Choice(String),
}

/// One thing the user did while editing a step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Event<S: Scalar> {
    /// The dialog control `key` was used.
    Dialog { key: String, value: DialogValue },
    /// The pointer moved over the viewport with no button held.
    Hover { pointer: Pointer<S> },
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
    /// presentation offered a grab (see [`super::Presentation::grab`]): from
    /// where it went down to where the pointer is now. `done` on release.
    Drag {
        from: Pointer<S>,
        to: Pointer<S>,
        #[serde(default)]
        done: bool,
    },
    /// A key, as the browser names it: `l`, `Escape`, `Enter`, `Delete`.
    Key { key: String },
}

impl<S: Scalar> Event<S> {
    /// The dialog control this event uses, and its value.
    pub fn dialog(&self) -> Option<(&str, &DialogValue)> {
        match self {
            Event::Dialog { key, value } => Some((key, value)),
            _ => None,
        }
    }

    /// Where the pointer is, for any event that has one — for a drag, where
    /// it is now.
    pub fn pointer(&self) -> Option<&Pointer<S>> {
        match self {
            Event::Hover { pointer } | Event::Click { pointer, .. } => Some(pointer),
            Event::Drag { to, .. } => Some(to),
            _ => None,
        }
    }
}
