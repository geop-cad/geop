//! What an operation exchanges with an editor while a step is being edited.
//!
//! An editor never edits a step's arguments itself. It sends what the user
//! did — an [`Event`]: a dialog control used, a click or a drag in the 3-D
//! viewport as a ray from the eye, a key — to [`crate::Operation::edit`], which answers with new
//! arguments, a new session (the temporary state of the edit: a tool in
//! hand, a half-drawn line, what is being picked) and a [`Presentation`]:
//! the [`Dialog`] to show and the [`Visual`]s to draw. The editor reruns the
//! program with the new arguments, draws both, and the cycle repeats.
//!
//! So an editor only ever renders generic primitives and forwards raw input,
//! and every decision — what a click means, what it snaps to, what a drag
//! changes — is made here, in Rust, where it can be tested. The helpers that
//! make those decisions consistent across operations live here too:
//! [`hit`] tests a pointer against visuals, [`PartView`] against the part,
//! and [`Picking`] / [`Dragging`] implement the two gestures most operations
//! share — picking an entity for an argument, dragging a handle.

mod dialog;
mod event;
pub mod hit;
mod interaction;
pub mod view;
mod visual;

pub use dialog::{ButtonItem, Choice, Control, Dialog, Field, ListItem, SelectStyle, Tone};
pub use event::{Button, DialogValue, Event, Pointer, Reach};
pub use interaction::{DRAG_SNAP, Dragging, Picked, Picking};
pub use view::{Extent, PartHit, PartView, Target};
pub use visual::{Presentation, Shape, Style, Visual};
