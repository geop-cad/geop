//! [`Event`]: what the user did, as an operation receives it.

use serde::{Deserialize, Serialize};

/// How big a screen pixel is along a [`Pointer`]'s ray: at distance `t`
/// from its origin, `at_origin + per_distance * t` world units. A
/// perspective view has `at_origin = 0`, an orthographic one
/// `per_distance = 0`.
///
/// Plain `f64`, like everything an editor sends: it is a UI tolerance
/// derived from screen pixels, not a geometric quantity the kernel reasons
/// about.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PixelScale {
    pub at_origin: f64,
    pub per_distance: f64,
}

impl PixelScale {
    /// World units per pixel at distance `t` along the ray.
    pub fn at(&self, t: f64) -> f64 {
        self.at_origin + self.per_distance * t.max(0.0)
    }
}

/// Where the pointer is: the ray from the eye through it, and enough about
/// the view to turn screen pixels into world units along it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pointer {
    pub origin: [f64; 3],
    /// Unit length, so a distance along the ray is a distance in the world.
    pub dir: [f64; 3],
    /// The screen's right and up, as unit directions in the world: what an
    /// offset on screen measures there.
    pub right: [f64; 3],
    pub up: [f64; 3],
    pub pixel: PixelScale,
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
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Event {
    /// The dialog control `key` was used.
    Dialog { key: String, value: DialogValue },
    /// The pointer moved over the viewport with no button held.
    Hover { pointer: Pointer },
    /// The pointer left the viewport.
    Leave,
    /// A click in the viewport: a press and release without moving.
    Click {
        pointer: Pointer,
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
        from: Pointer,
        to: Pointer,
        #[serde(default)]
        done: bool,
    },
    /// A key, as the browser names it: `l`, `Escape`, `Enter`, `Delete`.
    Key { key: String },
}

impl Event {
    /// The dialog control this event uses, and its value.
    pub fn dialog(&self) -> Option<(&str, &DialogValue)> {
        match self {
            Event::Dialog { key, value } => Some((key, value)),
            _ => None,
        }
    }

    /// Where the pointer is, for any event that has one — for a drag, where
    /// it is now.
    pub fn pointer(&self) -> Option<&Pointer> {
        match self {
            Event::Hover { pointer } | Event::Click { pointer, .. } => Some(pointer),
            Event::Drag { to, .. } => Some(to),
            _ => None,
        }
    }
}
