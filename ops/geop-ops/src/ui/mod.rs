//! What an editor exchanges with the operations while a step is edited.
//!
//! An editor never edits a step's arguments itself. It sends what the user
//! did — a [`StepEditEvent`]: a dialog field used, a click or a drag in the 3-D
//! viewport as a ray from the eye, a key — to a [`StepEditor`], which
//! turns it into what the operation understands: a field set to a value, by
//! the dialog, by a pick in the viewport or by dragging a handle; the
//! selection changed; a visual dragged in the plane worked in — or, for an
//! operation with a tool in hand, the click itself ([`CanvasEvent`]). What
//! comes back is a [`Presentation`]: the operation's [`Form`] — the
//! [`Dialog`] to show and the [`Visual`]s to draw — with what is picked,
//! selected and hovered lit. The editor reruns the program with the new
//! arguments, draws both, and the cycle repeats.
//!
//! Operations say what their fields and visuals *mean* — entities that can
//! fill a [`crate::operation::Role`], a length with a handle, a visual that
//! can be selected or dragged — and the editor decides how they are
//! edited, so a pick, a selection or a drag works the same in every
//! operation.
//!
//! So an editor only ever renders generic primitives and forwards raw input,
//! and every decision — what a click means, what it snaps to, what a drag
//! changes — is made here, in Rust, where it can be tested: [`hit`] tests a
//! pointer against visuals, [`PartView`] against the part.

mod dialog;
mod event;
mod form;
pub mod hit;
mod step;
pub mod view;
mod visual;

pub use dialog::{
    Action, Choice, Control, Dialog, Field, ListItem, Number, Picked, Reference, Tone, Track, Unit,
};
pub use event::{Button, CanvasEvent, Pointer, Reach, StepEditEvent, Value};
pub use form::{Edit, Form, InHand};
pub use step::{DRAG_SNAP, StepEditor};
pub use view::{Extent, PartHit, PartView, ViewThread};
pub use visual::{Presentation, Prompt, Shape, Style, Visual};
