//! What an editor exchanges with the operations while a step is edited.
//!
//! An editor never edits a step's arguments itself. It sends what the user
//! did — a [`StepEditEvent`]: a dialog field used, a click or a drag in the 3-D
//! viewport as a ray from the eye, a key — to a [`StepEditor`], which
//! turns it into what the operation understands: a field set to a value, by
//! the dialog, by a pick in the viewport or by dragging a handle — or, for
//! an operation drawing in a canvas of its own, the raw event. What comes
//! back is a [`Presentation`]: the operation's [`Form`] — the [`Dialog`] to
//! show and the [`Visual`]s to draw — with what is picked lit. The editor
//! reruns the program with the new arguments, draws both, and the cycle
//! repeats.
//!
//! So an editor only ever renders generic primitives and forwards raw input,
//! and every decision — what a click means, what it snaps to, what a drag
//! changes — is made here, in Rust, where it can be tested: [`hit`] tests a
//! pointer against visuals, [`PartView`] against the part.

mod dialog;
mod event;
pub mod hit;
mod step;
pub mod view;
mod visual;

pub use dialog::{ButtonItem, Choice, Control, Dialog, Field, ListItem, Tone};
pub use event::{Button, Pointer, Reach, StepEditEvent, Value};
pub use step::{DRAG_SNAP, StepEditor};
pub use view::{Extent, PartHit, PartView, Target};
pub use visual::{Form, Presentation, Shape, Style, Visual};
