//! What an editor exchanges with operations: the vocabulary of
//! [`geop_core_program::editing`], and building blocks for forms of this crate's
//! types ([`formula_number`], [`region`]), and what a step shows of the part
//! besides its model ([`annotations`], [`listing`]).

mod edit_form;
mod fields;
pub mod icons;
mod shown;
mod view;

pub(crate) use view::RESOLUTION;

pub use edit_form::{Edit, EditForm, FormState};
pub use fields::{choices, chosen, constraint_status, formula_number, region};
pub use geop_core_program::editing::{
    Action, Axis, Button, CanvasEvent, Change, Choice, Control, Dialog, Event, Extent, Field, Form,
    Gizmo, GizmoDrag, GizmoPart, GizmoView, InHand, ListItem, Number, Orientation, Picked, Pointer,
    Presentation, Prompt, Pull, Reach, Reference, Shape, StepEditEvent, Style, TRIAD, Tone, Track,
    Unit, Value, Visual, gizmo, hit,
};
pub use shown::{
    Annotations, Listed, Listing, StructureItem, annotations, annotations_of, listed, listing,
    structure,
};
pub use view::{
    EMPTY_EXTENT, FRAME, PartScene, PartView, ViewAnnotation, ViewCurve, ViewDatum, ViewEdge,
    ViewFace, ViewInstance, ViewSketch, ViewSketchPoint, ViewVertex, parts_drawn,
};
