//! What an editor exchanges with an operation while a step is edited: the
//! [`Presentation`] it shows, a [`Dialog`] of fields and the [`Visual`]s it
//! draws, and the [`Event`]s of what the user did. Fields say what a value
//! means, never how it is edited, so a pick, a selection or a drag means
//! the same in every operation.

mod dialog;
mod entity;
mod event;
mod form;
pub mod gizmo;
pub mod hit;
mod visual;

pub use dialog::{
    Action, Choice, Control, Dialog, Field, ListItem, Number, Picked, Reference, Tone, Track, Unit,
    is_formula,
};
pub use entity::{EntityRef, INSTANCE_SEPARATOR, Role, describe_roles, instance_path};
pub use event::{Button, CanvasEvent, Pointer, Reach, StepEditEvent, Value};
pub use form::{Event, Form, InHand, Pull};
pub use gizmo::{Axis, Change, Gizmo, GizmoDrag, GizmoPart, GizmoView, Orientation};
pub use visual::{Extent, Presentation, Prompt, Shape, Style, TRIAD, Visual};
