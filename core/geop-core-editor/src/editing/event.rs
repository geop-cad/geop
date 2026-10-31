//! [`StepEditEvent`]: what the user did, as an operation receives it.

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{Motion, Ray},
    scalars::{Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use super::{Gizmo, GizmoDrag};
use geop_core_program::{element::PickedElement, numeric::NumericInput};

/// Dragged values snap to this: a drag is for rough shaping, a dialog for
/// exact values.
pub const DRAG_SNAP: f64 = 0.01;

/// How far from its ray a [`Pointer`] reaches: what counts as under it, at
/// any distance along the ray. Everything a viewer draws at a constant size
/// on screen — a handle, a label, a frame datum — is laid out in reaches,
/// so it is hit where it is drawn however far away or zoomed in the view is.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Reach {
    /// Seen in perspective: a cone around the ray, from its origin — the
    /// eye — widening by `slope` per unit of distance.
    Cone {
        #[serde(with = "as_f64")]
        slope: Scalar,
    },
    /// Seen orthographically: a tube of `radius` around the ray.
    Tube {
        #[serde(with = "as_f64")]
        radius: Scalar,
    },
}

impl Reach {
    /// How far from the ray it reaches, `t` along it.
    pub fn at(&self, t: Scalar) -> Scalar {
        match *self {
            Reach::Cone { slope } => slope.mul(t),
            Reach::Tube { radius } => radius,
        }
    }
}

/// Where the pointer is: the ray from the eye through it, how far from
/// that ray it reaches, and which of what lies under it it means.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pointer {
    pub ray: Ray,
    pub reach: Reach,
    /// Which of what lies under the pointer it means, counted front to
    /// back from the one a click picks first (see
    /// [`super::hit::Scene::under`]), wrapping round: a secondary click
    /// steps one further behind, and any move of the pointer starts again
    /// from the front. A pick, a hover and a click alike take the one
    /// this names, so what is lit is what a click holds.
    #[serde(default)]
    pub cycle: usize,
}

impl Pointer {
    /// The pointer moved by `motion`: into the frame of a part placed in
    /// the part drawn, for a step of it to be edited where it is placed. A
    /// rigid motion leaves its reach as it is.
    pub fn moved(&self, motion: &Motion) -> GeopResult<Self> {
        let ray = Ray::try_new(
            motion.apply(self.ray.origin()),
            motion.rotate(self.ray.dir()),
        )?;
        Ok(Pointer { ray, ..*self })
    }

    /// The same pointer, meaning the `cycle`-th of what lies under it (see
    /// [`Pointer::cycle`]).
    pub fn cycled(self, cycle: usize) -> Self {
        Pointer { cycle, ..self }
    }

    /// `reaches` of the pointer's reach, `t` along its ray.
    pub fn reach_at(&self, reaches: f64, t: Scalar) -> Scalar {
        self.reach.at(t).mul(Scalar::from_f64(reaches))
    }

    /// Whether something `dist` from the ray, `t` along it, could lie within
    /// `reaches` of it.
    pub fn within(&self, dist: Scalar, t: Scalar, reaches: f64) -> bool {
        !dist.definitely_greater(self.reach_at(reaches, t))
    }
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
    /// The element at this index was taken out of a reference field.
    RemoveAt(usize),
    /// Everything was taken out of a reference field.
    Clear,
    /// What a reference field holds now: all an operation is ever sent for
    /// one (see [`super::Reference`]).
    Elements(Vec<PickedElement>),
    /// Something done to a table field (see [`super::Table`]).
    Table(TableEdit),
}

/// What is done to a [`super::Table`], one thing at a time; rows and
/// columns are named by their index in the table as it was shown.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "edit", rename_all = "snake_case")]
pub enum TableEdit {
    /// The cell at `row`, `column` typed into.
    Cell {
        row: usize,
        column: usize,
        text: String,
    },
    /// A block of text pasted at `row`, `column` — from a spreadsheet, a
    /// row of cells per line — filling the cells from there on, and adding
    /// rows below the last for what runs over. An operation never sees
    /// this: its form takes it apart into the rows added and the cells set
    /// (see [`crate::edit_form::EditForm::table`]).
    Paste {
        row: usize,
        column: usize,
        cells: Vec<Vec<String>>,
    },
    /// A row added, after the last.
    AddRow,
    RemoveRow {
        row: usize,
    },
    /// The row `row` moved to be the row `to`.
    MoveRow {
        row: usize,
        to: usize,
    },
    /// The row chosen, of a table one row of which is.
    SelectRow {
        row: usize,
    },
    /// The header of the column `column` typed into.
    Column {
        column: usize,
        text: String,
    },
    /// A column added, after the last.
    AddColumn,
    RemoveColumn {
        column: usize,
    },
}

impl Value {
    /// The [`NumericInput`] a number field set to it gives: a number slid
    /// or dragged plain, text typed plain if it is a number and a formula
    /// otherwise. `None` for anything else, and for empty text. A formula
    /// that does not evaluate is kept, as typed: the step then fails naming
    /// it, and the field shows the text to correct.
    pub fn numeric_input(&self) -> Option<NumericInput> {
        match self {
            Value::Number(v) => Some(NumericInput::Plain(*v)),
            Value::Text(text) if !text.trim().is_empty() => Some(NumericInput::from(text.as_str())),
            _ => None,
        }
    }
}

/// One thing the user did while editing a step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StepEditEvent {
    /// The dialog field `key` was used.
    Dialog { key: String, value: Value },
    /// The pointer moved over the viewport with no button held — `shift`
    /// held or not, which turns snapping off.
    Hover {
        pointer: Pointer,
        #[serde(default)]
        shift: bool,
    },
    /// The pointer left the viewport.
    Leave,
    /// A click in the viewport: a press and release of the primary button
    /// without moving. (The secondary button's click is no event of its
    /// own: it steps the pointer's [`Pointer::cycle`].)
    Click {
        pointer: Pointer,
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
        from: Pointer,
        to: Pointer,
        #[serde(default)]
        done: bool,
        #[serde(default)]
        shift: bool,
    },
    /// A key, as the browser names it: `l`, `Escape`, `Enter`, `Delete`.
    Key { key: String },
    /// The view settled: `up` is the direction in the world that points up
    /// the screen. Sent when the camera comes to rest and when the pointer
    /// enters the viewport, so a step can name what it shows as it is seen
    /// (a sketch's horizontal is the screen's).
    View { up: Vector3 },
}

impl StepEditEvent {
    /// The event with its pointers moved by `motion` (see
    /// [`Pointer::moved`]); one without a pointer, as it is.
    pub fn moved(&self, motion: &Motion) -> GeopResult<Self> {
        Ok(match self {
            StepEditEvent::Hover { pointer, shift } => StepEditEvent::Hover {
                pointer: pointer.moved(motion)?,
                shift: *shift,
            },
            StepEditEvent::Click {
                pointer,
                double,
                shift,
            } => StepEditEvent::Click {
                pointer: pointer.moved(motion)?,
                double: *double,
                shift: *shift,
            },
            StepEditEvent::Drag {
                from,
                to,
                done,
                shift,
            } => StepEditEvent::Drag {
                from: from.moved(motion)?,
                to: to.moved(motion)?,
                done: *done,
                shift: *shift,
            },
            StepEditEvent::View { up } => StepEditEvent::View {
                up: motion.rotate(up),
            },
            other => other.clone(),
        })
    }

    /// The dialog field this event uses, and its value.
    pub fn dialog(&self) -> Option<(&str, &Value)> {
        match self {
            StepEditEvent::Dialog { key, value } => Some((key, value)),
            _ => None,
        }
    }

    /// Where the pointer is, for any event that has one — for a drag, where
    /// it is now.
    pub fn pointer(&self) -> Option<&Pointer> {
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
/// the step editor, [`crate::step_editor`]).
#[derive(Clone, Debug, PartialEq)]
pub enum CanvasEvent {
    /// The pointer moved over the viewport with no button held, `shift`
    /// held or not.
    Hover { pointer: Pointer, shift: bool },
    /// The pointer left the viewport.
    Leave,
    /// A primary click that selected nothing: a double one, one while the
    /// operation has a tool in hand, one on nothing selectable.
    Click {
        pointer: Pointer,
        double: bool,
        shift: bool,
    },
    /// What Escape does: end what is in progress — a chain being drawn,
    /// the points picked for a tool — and, with nothing in progress, put
    /// the tool in hand down. The selection is cleared already.
    Cancel,
    /// What Enter does: finish what is in progress, a
    /// chain being drawn, a constraint to apply; the tool stays in hand.
    Confirm,
    /// What Delete and Backspace do: delete the selection.
    Delete,
    /// The draggable visual `key` dragged in the plane worked in, from
    /// where it was grabbed to where the pointer is now — `pointer`, with
    /// `shift` held or not. `done` on release.
    Move {
        key: String,
        from: Vector3,
        to: Vector3,
        pointer: Pointer,
        done: bool,
        shift: bool,
    },
    /// A drag of the tool in hand, where it strokes (see
    /// [`super::InHand::Strokes`]): the ray where it went down and the ray
    /// where the pointer is now. `done` on release.
    Stroke {
        from: Pointer,
        to: Pointer,
        done: bool,
        shift: bool,
    },
    /// A drag of the form's gizmo (see [`super::Gizmo`]): what it did
    /// since it started, to apply to the step's arguments as they were
    /// then — which they are again whenever this is sent. `done` on
    /// release.
    /// `from` is the gizmo as the step asked for it when the drag started:
    /// where it stood, along what it stands on's own axes.
    Gizmo {
        drag: GizmoDrag,
        from: Gizmo,
        done: bool,
    },
    /// Any other key, as the browser names it, a letter lowercased: a
    /// tool's shortcut.
    Key { key: String },
    /// The view settled, `up` pointing up the screen (see
    /// [`StepEditEvent::View`]).
    View { up: Vector3 },
}
