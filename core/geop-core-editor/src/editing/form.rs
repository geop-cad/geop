//! [`Form`]: what an operation shows for a step while it is edited, and
//! [`Event`]: what reaches it of what the user did.

use geop_core_math::{
    primitives::{CoordinateSystem, Pose},
    scalars::Scalar,
    vector::Vector3,
};

use super::{CanvasEvent, Dialog, Extent, Gizmo, Layout, Prompt, Value, Visual};

/// What an operation has in hand while a step is edited (see
/// [`Form::tool`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum InHand {
    /// Nothing: a click selects, and a draggable visual is dragged.
    #[default]
    Nothing,
    /// A tool that clicks — drawing a line, say: clicks go to it rather
    /// than select, and draggable visuals are not dragged.
    Clicks,
    /// A tool that also strokes — trimming, say: besides its clicks, a
    /// press anywhere grabs, and the drag goes to it as a
    /// [`CanvasEvent::Stroke`].
    Strokes,
}

/// What a step asks of the state while it is edited, for the editor to
/// solve the program with: an operation never sets the state itself, it
/// says where it wants it pulled, and the constraints have the last word.
#[derive(Clone, Debug, PartialEq)]
pub enum Pull {
    /// The point `local` of a placed part — in its own frame — pulled
    /// towards the point `target`: a drag. The part is named by the state
    /// its pose is.
    Point {
        parameter: String,
        local: Vector3,
        target: Vector3,
    },
    /// A placed part pulled to the pose `target` as a whole: placed where
    /// it was typed in, turned by a gizmo.
    Pose { parameter: String, target: Pose },
    /// The number the state `parameter` is, set to `value` and held there:
    /// a joint's coordinate typed in, which the parts move to.
    Value { parameter: String, value: Scalar },
}

/// What an operation shows for a step: the fields of its dialog and what
/// it draws, and what the editor needs to know to edit it the same way as
/// every other step — what is selected, what the operation has in hand,
/// what it drags. Plain data: what a field does when it is set is the
/// operation's to say when it is sent the [`Event`].
#[derive(Clone, Debug, PartialEq)]
pub struct Form {
    pub dialog: Dialog,
    /// Where the dialog is shown: beside the viewport, or as a popup over
    /// it (see [`Layout`]).
    pub layout: Layout,
    pub visuals: Vec<Visual>,
    /// A plane to work in, its `u`/`v` plane: the viewer faces it head on,
    /// stops orbiting, and draws a grid on it. Draggable visuals are
    /// dragged in it.
    pub focus: Option<CoordinateSystem>,
    /// A sheet of its own the step is edited on — a drawing's paper, laid
    /// in `focus` — and how far it reaches: the viewer shows the step's
    /// visuals alone, without the part, framed to it.
    pub sheet: Option<Extent>,
    /// What the operation has in hand: clicks — and, for a tool that
    /// strokes, drags — go to it rather than select.
    pub tool: InHand,
    /// The keys of the visuals selected. A click on a selectable visual
    /// changes it, and the operation is sent the new selection
    /// ([`Event::Select`]) to keep.
    pub selection: Vec<String>,
    /// What the step pulls the state towards: the editor solves the
    /// program with them.
    pub pulls: Vec<Pull>,
    /// A value asked for in place, in the viewport.
    pub prompt: Option<Prompt>,
    /// A gizmo to move, turn or scale something by: its drags are sent as
    /// [`CanvasEvent::Gizmo`].
    pub gizmo: Option<Gizmo>,
    /// Solids and faces of the part, by name, not to draw while the step
    /// is edited: what the step builds, when its visuals show it better —
    /// a control cage shown alone.
    pub hidden: Vec<String>,
}

impl Default for Form {
    fn default() -> Self {
        Self {
            dialog: Dialog::new(),
            layout: Layout::default(),
            visuals: Vec::new(),
            focus: None,
            sheet: None,
            tool: InHand::Nothing,
            selection: Vec::new(),
            pulls: Vec::new(),
            prompt: None,
            gizmo: None,
            hidden: Vec::new(),
        }
    }
}

/// What reaches an operation of what the user did, once the editor has
/// done what it does alike for every operation: arming a reference field
/// and picking for it, dragging a number's handle, selecting.
#[derive(Clone, Debug, PartialEq)]
#[allow(
    clippy::large_enum_variant,
    reason = "an event is made once and read once; a stroke carries its points"
)]
pub enum Event {
    /// The field `key` of the dialog set to `value` — typed, chosen, picked
    /// for in the viewport or dragged by its handle.
    Field { key: String, value: Value },
    /// The visuals selected now, by their keys.
    Select(Vec<String>),
    /// What the pointer or a key did beyond that.
    Canvas(CanvasEvent),
}
