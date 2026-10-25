//! [`StepEditor`]: editing one step — the interaction every operation
//! shares, so a pick, a selection or a drag means the same whatever is
//! edited — against a [`Scene`]: the part as drawn, which picks are made in.

use std::{any::Any, rc::Rc};

use geop_core_math::{geop_error::GeopResult, vector::Vector3};

use crate::{
    editable::Editors,
    editing::{
        Action, Button, CanvasEvent, Control, DRAG_SNAP, Dialog, Event, Form, Gizmo, GizmoView,
        InHand, Orientation, Pointer, Presentation, Reference, Shape, StepEditEvent, Style, Value,
        Visual,
        gizmo::GizmoGrab,
        hit::{Scene, hit_visuals},
    },
};
use geop_core_program::{element::PickedElement, part::Part, program::Program, runner::Runner};

/// The key of the field the editor adds to a dialog whose gizmo has an
/// orientation of its own: the world's axes or its own, as
/// [`Value::Choice`] `world` or `local`.
pub const GIZMO_ORIENTATION: &str = "gizmo_orientation";

/// The step's arguments and the operation's editing state, as a drag
/// found them when it grabbed what it drags.
type Start = (serde_json::Value, Rc<dyn Any>);

/// What a drag holds, as it was grabbed.
enum Grab {
    /// The handle of the number field `key`, and the value that had.
    Handle { key: String, value: f64 },
    /// A draggable visual of the operation's, dragged in the plane worked
    /// in — or, with none, in the plane through where it was grabbed,
    /// facing the eye: `(point, normal)` — and, for one dragged from where
    /// it was grabbed (see [`Visual::from_grab`]), the step as it was then.
    Visual {
        key: String,
        plane: Option<(Vector3, Vector3)>,
        start: Option<Start>,
    },
    /// A stroke of the tool in hand, from where it went down.
    Stroke,
    /// A part of the form's gizmo, with the step's arguments and editing
    /// state as they were when it was pressed — what every event of the
    /// drag applies its change to — and what the drag has done so far, in
    /// words.
    Gizmo {
        grab: GizmoGrab,
        from: Box<Gizmo>,
        start: Start,
        readout: Option<String>,
    },
}

/// A step being edited: the operation's editing state, and the editor's
/// own: which reference field waits for a click and what a click would
/// pick for it, where the pointer is, what is being dragged. The step's
/// arguments are the program's: every edit writes them back to it.
///
/// It turns every event into what the operation understands, the same for
/// every operation:
///
/// - A reference field (see [`Reference`]) is armed by pressing it, and
///   then a click in the viewport picks for it — one entity, or, for a
///   field of several, one more or one less. Removing one of its entities
///   or clearing it is done here too. The operation is only ever sent what
///   the field holds now, as [`Value::Entities`].
/// - A number field with a handle is dragged by it, snapped to
///   [`DRAG_SNAP`], and set as a [`Value::Number`].
/// - A click on a selectable visual selects it, or takes it out of the
///   selection again; a click on nothing clears the selection, unless shift
///   is held, and Escape clears it. The operation keeps the selection: it
///   is sent the new one ([`Event::Select`]), and says what it is in its
///   form ([`Form::selection`]).
/// - A draggable visual is dragged in the plane worked in — or, where there
///   is none, in the plane through where it was grabbed, facing the eye —
///   as a [`CanvasEvent::Move`]; one dragged from where it was grabbed (see
///   [`Visual::from_grab`]) is sent to the step as it was then, as the
///   gizmo is. With a tool in hand that strokes, a drag from anywhere else
///   is the tool's, as a [`CanvasEvent::Stroke`].
/// - The form's gizmo, if it has one, is drawn over everything and grabbed
///   first: a drag of one of its parts is followed and snapped (see
///   [`crate::editing::gizmo`]) and sent as a [`CanvasEvent::Gizmo`], to
///   the step as it was when it was pressed, so each event applies the
///   whole drag once. A click on it does nothing, unless a field waits for
///   a pick: then it picks what is under it. Where it has an orientation
///   of its own, the dialog gets a [`GIZMO_ORIENTATION`] field to choose
///   the world's axes or its own.
/// - Escape, Enter, Delete and a secondary click mean the same in every
///   tool, and reach it as [`CanvasEvent::Cancel`], [`CanvasEvent::Confirm`]
///   and [`CanvasEvent::Delete`]; any other key, lowercased, is a tool's
///   shortcut.
///
/// Whatever else the pointer and the keys do goes to the operation as a
/// [`CanvasEvent`]: clicks while it has a tool in hand, and those on
/// nothing, hovers, keys.
///
/// An edit is made against the part as the last run left it: the editor
/// runs the program again after each, for the step to be shown as it is
/// now.
pub struct StepEditor {
    step: String,
    editing: Rc<dyn Any>,
    /// The reference field waiting for a click.
    armed: Option<String>,
    /// What a click would pick for it.
    hover: Option<PickedElement>,
    /// Where the pointer last was, if over the viewport.
    pointer: Option<Pointer>,
    /// Where the drag under way went down, and what it holds.
    grab: Option<(Pointer, Grab)>,
    /// Which way the gizmo is turned, where it has a way of its own.
    orientation: Orientation,
}

/// Takes `x` out of `xs`, or adds it at the end.
fn toggle<T: PartialEq>(xs: &mut Vec<T>, x: T) {
    match xs.iter().position(|y| *y == x) {
        Some(i) => {
            xs.remove(i);
        }
        None => xs.push(x),
    }
}

/// The handles of `dialog`'s number fields, as visuals under their keys.
fn handles(dialog: &Dialog) -> Vec<Visual> {
    (dialog.0.iter())
        .filter_map(|f| match &f.control {
            Control::Number(n) => n.handle.map(|track| {
                let shape = Shape::Handle {
                    at: track.at,
                    direction: track.direction,
                };
                Visual::new(f.key.clone(), shape, Style::Handle)
            }),
            _ => None,
        })
        .collect()
}

fn is_handle(visual: &Visual) -> bool {
    matches!(visual.shape, Shape::Handle { .. })
}

/// Whether a press on `visual` grabs it: a handle always, a draggable
/// visual unless a tool is in hand.
fn grabs(visual: &Visual, tool: InHand) -> bool {
    is_handle(visual) || (visual.draggable && tool == InHand::Nothing)
}

/// The keys of `form`'s reference fields that wait for a pick.
fn waiting(form: &Form) -> impl Iterator<Item = &str> {
    (form.dialog.0.iter()).filter_map(|f| match &f.control {
        Control::Reference(r) if r.waiting() => Some(f.key.as_str()),
        _ => None,
    })
}

/// The keys of `form`'s reference fields.
fn references(form: &Form) -> Vec<String> {
    (form.dialog.0.iter())
        .filter(|f| matches!(f.control, Control::Reference(_)))
        .map(|f| f.key.clone())
        .collect()
}

/// The program a step editor edits, the operations it is edited with, the
/// runner and the part it was last run on, and the scene picks are made in
/// — what every
/// event is handled against.
pub struct Editing<'a, Sc> {
    pub editors: &'a Editors,
    pub runner: &'a Runner,
    pub part: &'a Part,
    pub program: &'a mut Program,
    pub scene: &'a Sc,
}

impl StepEditor {
    /// Editing the step `step` of `program`. A new step whose first
    /// reference field is still empty starts by picking for it: what it is
    /// built on is what it needs first. One that already holds something —
    /// the newest sketch, say — needs no click, and leaves the pointer to
    /// the rest of the step: its handles, its drawing.
    pub fn new(
        editors: &Editors,
        runner: &Runner,
        part: &Part,
        program: &Program,
        step: &str,
        new: bool,
    ) -> GeopResult<Self> {
        let editing: Rc<dyn Any> = Rc::from(editors.initial_editing(program, step)?);
        let armed = match new {
            true => {
                let form = editors.presentation(runner, part, program, step, &*editing)?;
                let first =
                    (form.dialog.0.iter()).find(|f| matches!(f.control, Control::Reference(_)));
                first
                    .filter(|f| matches!(&f.control, Control::Reference(r) if r.waiting()))
                    .map(|f| f.key.clone())
            }
            false => None,
        };
        Ok(Self {
            step: step.to_string(),
            editing,
            armed,
            hover: None,
            pointer: None,
            grab: None,
            orientation: Orientation::default(),
        })
    }

    /// The step edited, by id.
    pub fn step(&self) -> &str {
        &self.step
    }

    /// The operation's editing state: its own, to look into.
    pub fn editing(&self) -> &dyn Any {
        &*self.editing
    }

    /// Starts the editing afresh — the operation's state as for a step just
    /// opened, nothing picked for or dragged — after the step's arguments
    /// were put back as an edit found them: undoing one. What the old
    /// editing state held may not be in the step any more.
    pub fn restart(&mut self, editors: &Editors, program: &Program) -> GeopResult<()> {
        self.editing = Rc::from(editors.initial_editing(program, &self.step)?);
        self.armed = None;
        self.hover = None;
        self.grab = None;
        Ok(())
    }

    /// The step's form.
    pub fn form(
        &self,
        editors: &Editors,
        runner: &Runner,
        part: &Part,
        program: &Program,
    ) -> GeopResult<Form> {
        editors.presentation(runner, part, program, &self.step, &*self.editing)
    }

    /// Sends `event` to the operation, and keeps what it made of the step:
    /// its arguments in the program, its editing state here. A reference
    /// field that is gone after it waits for no pick any more, and one that
    /// appeared empty waits for one — what the step needs picked next, as
    /// for a new step.
    fn send<Sc: Scene>(&mut self, e: &mut Editing<'_, Sc>, event: Event) -> GeopResult<()> {
        let before = references(&self.form(e.editors, e.runner, e.part, e.program)?);
        let scene: &dyn Scene = e.scene;
        let edited = (e.editors).process_user_event(
            e.runner,
            e.part,
            e.program,
            &self.step,
            &*self.editing,
            event,
            Some(scene),
        )?;
        e.program.step_mut(&self.step)?.args = edited.args;
        self.editing = Rc::from(edited.editing);
        let form = self.form(e.editors, e.runner, e.part, e.program)?;
        if let Some(armed) = &self.armed
            && !references(&form).contains(armed)
        {
            self.armed = None;
            self.hover = None;
        }
        if let Some(key) = waiting(&form).find(|key| !before.iter().any(|b| b == key)) {
            self.armed = Some(key.to_string());
            self.hover = None;
        }
        Ok(())
    }

    fn pass<Sc: Scene>(&mut self, e: &mut Editing<'_, Sc>, event: CanvasEvent) -> GeopResult<()> {
        self.send(e, Event::Canvas(event))
    }

    /// The selection is `selection` now.
    fn select<Sc: Scene>(
        &mut self,
        e: &mut Editing<'_, Sc>,
        form: &Form,
        selection: Vec<String>,
    ) -> GeopResult<()> {
        match selection == form.selection {
            true => Ok(()),
            false => self.send(e, Event::Select(selection)),
        }
    }

    /// The step as it is now, for a drag to start from.
    fn start<Sc: Scene>(&self, e: &Editing<'_, Sc>) -> Start {
        let args = (e.program.step(&self.step))
            .map(|s| s.args.clone())
            .unwrap_or_default();
        (args, self.editing.clone())
    }

    /// The step back as a drag found it, `start`.
    fn restore<Sc: Scene>(&mut self, e: &mut Editing<'_, Sc>, start: &Start) {
        if let Ok(step) = e.program.step_mut(&self.step) {
            step.args = start.0.clone();
        }
        self.editing = start.1.clone();
    }

    /// The form's gizmo, turned as chosen.
    fn gizmo(&self, form: &Form) -> Option<GizmoView> {
        (form.gizmo.as_ref()).map(|gizmo| GizmoView::of(gizmo, self.orientation))
    }

    /// Applies `event`, picking in `e.scene`: the part before the step as
    /// drawn, or, for an operation whose picks are made in what it builds,
    /// that.
    pub fn handle<Sc: Scene>(
        &mut self,
        mut e: Editing<'_, Sc>,
        event: &StepEditEvent,
    ) -> GeopResult<()> {
        let e = &mut e;
        let form = self.form(e.editors, e.runner, e.part, e.program)?;
        if let StepEditEvent::Dialog { key, value } = event {
            return self.dialog(e, &form, key, value.clone());
        }
        // A click on the gizmo is no click on what is under it — unless a
        // field waits for a pick, which a click is for, as for a handle.
        if let (StepEditEvent::Click { pointer, .. }, Some(gizmo), None) =
            (event, self.gizmo(&form), &self.armed)
            && gizmo.hit(pointer).is_some()
        {
            self.pointer = Some(*pointer);
            return Ok(());
        }
        match event {
            StepEditEvent::Hover { pointer, .. } | StepEditEvent::Click { pointer, .. } => {
                self.pointer = Some(*pointer)
            }
            StepEditEvent::Leave => self.pointer = None,
            _ => {}
        }
        if let Some(key) = self.armed.clone() {
            if let Some(Control::Reference(reference)) = form.dialog.get(&key) {
                // A drag is no pick: while the field waits, it can only be
                // of a handle (see `presentation`).
                return match event {
                    StepEditEvent::Drag {
                        from,
                        to,
                        done,
                        shift,
                    } => self.drag(e, &form, from, to, *done, *shift, true),
                    _ => self.pick(e, event, &key, reference),
                };
            }
            self.armed = None;
        }
        match event {
            StepEditEvent::Dialog { .. } => Ok(()),
            StepEditEvent::Hover { pointer, shift } => self.pass(
                e,
                CanvasEvent::Hover {
                    pointer: *pointer,
                    shift: *shift,
                },
            ),
            StepEditEvent::Leave => self.pass(e, CanvasEvent::Leave),
            StepEditEvent::Click {
                pointer,
                button,
                double,
                shift,
            } => {
                let in_hand = form.tool != InHand::Nothing;
                let hit = (*button == Button::Primary && !*double && !in_hand)
                    .then(|| {
                        let instance = |name: &str, p: &Pointer| e.scene.hit_instance(name, p);
                        hit_visuals(&form.visuals, pointer, instance, |v| v.selectable)
                            .map(|hit| hit.visual.key.clone())
                    })
                    .flatten();
                match (hit, button) {
                    (_, Button::Secondary) => self.pass(e, CanvasEvent::Confirm),
                    (Some(key), _) => {
                        let mut selection = form.selection.clone();
                        toggle(&mut selection, key);
                        self.select(e, &form, selection)
                    }
                    (None, _) => {
                        if !*shift && !in_hand {
                            self.select(e, &form, Vec::new())?;
                        }
                        self.pass(
                            e,
                            CanvasEvent::Click {
                                pointer: *pointer,
                                double: *double,
                                shift: *shift,
                            },
                        )
                    }
                }
            }
            StepEditEvent::Drag {
                from,
                to,
                done,
                shift,
            } => self.drag(e, &form, from, to, *done, *shift, false),
            StepEditEvent::Key { key } => match key.as_str() {
                "Escape" => {
                    self.select(e, &form, Vec::new())?;
                    self.pass(e, CanvasEvent::Cancel)
                }
                "Enter" => self.pass(e, CanvasEvent::Confirm),
                "Delete" | "Backspace" => self.pass(e, CanvasEvent::Delete),
                _ => {
                    let key = key.to_lowercase();
                    self.pass(e, CanvasEvent::Key { key })
                }
            },
        }
    }

    /// The dialog field `key` used: a reference field armed, or an entity
    /// taken out of it, or all of them; any other field set.
    fn dialog<Sc: Scene>(
        &mut self,
        e: &mut Editing<'_, Sc>,
        form: &Form,
        key: &str,
        value: Value,
    ) -> GeopResult<()> {
        if key == GIZMO_ORIENTATION {
            match value {
                Value::Choice(c) if c == "world" => self.orientation = Orientation::World,
                Value::Choice(c) if c == "local" => self.orientation = Orientation::Local,
                _ => {}
            }
            return Ok(());
        }
        let Some(Control::Reference(reference)) = form.dialog.get(key) else {
            let key = key.to_string();
            return self.send(e, Event::Field { key, value });
        };
        let mut elements: Vec<PickedElement> = reference.elements().cloned().collect();
        match value {
            Value::Press => {
                self.armed = (self.armed.as_deref() != Some(key)).then(|| key.to_string());
                self.hover = None;
                return Ok(());
            }
            Value::RemoveAt(i) if i < elements.len() => {
                elements.remove(i);
            }
            Value::Clear => elements.clear(),
            Value::Elements(set) => elements = set,
            _ => return Ok(()),
        }
        let (key, value) = (key.to_string(), Value::Elements(elements));
        self.send(e, Event::Field { key, value })
    }

    /// A pointer event while the reference field `key` waits for a click:
    /// a hover finds what a click would pick, a click picks it — and, for a
    /// field of one entity, is done. Escape stops picking.
    fn pick<Sc: Scene>(
        &mut self,
        e: &mut Editing<'_, Sc>,
        event: &StepEditEvent,
        key: &str,
        reference: &Reference,
    ) -> GeopResult<()> {
        let scene = e.scene;
        let pick = |pointer| {
            scene
                .pick(pointer, &reference.accepts)
                .map(|hit| hit.element)
        };
        match event {
            StepEditEvent::Hover { pointer, .. } => self.hover = pick(pointer),
            StepEditEvent::Leave => self.hover = None,
            StepEditEvent::Click {
                pointer,
                button: Button::Primary,
                ..
            } => {
                self.hover = None;
                if let Some(element) = pick(pointer) {
                    let mut elements: Vec<PickedElement> = reference.elements().cloned().collect();
                    if reference.multiple {
                        toggle(&mut elements, element);
                    } else {
                        elements = vec![element];
                        self.armed = None;
                    }
                    let (key, value) = (key.to_string(), Value::Elements(elements));
                    return self.send(e, Event::Field { key, value });
                }
            }
            StepEditEvent::Key { key } if key == "Escape" => {
                self.armed = None;
                self.hover = None;
            }
            _ => {}
        }
        Ok(())
    }

    /// A drag: of a number field's handle — its value when grabbed, moved
    /// by how far the pointer has moved along the handle's track, snapped
    /// to [`DRAG_SNAP`] — or of a draggable visual, in the plane worked in
    /// or, with none, in the plane through where it was grabbed, facing
    /// the eye.
    ///
    /// What is dragged is hit-tested where the drag started, once: while
    /// dragged, it moves away from there. After that it is found by its key.
    /// A drag that went down somewhere else is a new one, even if the last
    /// one's release never came. A drag that did not start on either is a
    /// stroke of the tool in hand,
    /// if it strokes — else nothing, as for a pointer that cannot be
    /// followed (looking straight along a handle's track, or along the
    /// plane). With `handles_only` — while a field waits for a pick — only a
    /// handle is grabbed.
    #[allow(clippy::too_many_arguments)]
    fn drag<Sc: Scene>(
        &mut self,
        e: &mut Editing<'_, Sc>,
        form: &Form,
        from: &Pointer,
        to: &Pointer,
        done: bool,
        shift: bool,
        handles_only: bool,
    ) -> GeopResult<()> {
        let handles = handles(&form.dialog);
        if self.grab.as_ref().is_some_and(|(down, _)| down != from) {
            self.grab = None;
        }
        if self.grab.is_none()
            && let (Some(gizmo), Some(asked)) = (self.gizmo(form), &form.gizmo)
            && let Some(part) = gizmo.hit(from)
        {
            let grab = Grab::Gizmo {
                grab: GizmoGrab::new(gizmo, part, from),
                from: Box::new(asked.clone()),
                start: self.start(e),
                readout: None,
            };
            self.grab = Some((*from, grab));
        }
        if self.grab.is_none() {
            let visuals: Vec<Visual> = form.visuals.iter().chain(&handles).cloned().collect();
            let instance = |name: &str, p: &Pointer| e.scene.hit_instance(name, p);
            let hit = hit_visuals(&visuals, from, instance, |v| match handles_only {
                true => is_handle(v),
                false => grabs(v, form.tool),
            });
            let grab = match hit {
                Some(hit) => {
                    let key = hit.visual.key.clone();
                    match form.dialog.get(&key) {
                        Some(Control::Number(n)) if is_handle(hit.visual) => Grab::Handle {
                            key,
                            value: n.value,
                        },
                        _ => Grab::Visual {
                            key,
                            plane: (form.focus.is_none())
                                .then(|| (from.ray.at(hit.t), *from.ray.dir())),
                            start: hit.visual.from_grab.then(|| self.start(e)),
                        },
                    }
                }
                None if form.tool == InHand::Strokes && !handles_only => Grab::Stroke,
                None => return Ok(()),
            };
            self.grab = Some((*from, grab));
        }
        let Some((down, mut grab)) = self.grab.take() else {
            return Ok(());
        };
        let moved = self.drag_by(e, form, &handles, &mut grab, from, to, done, shift);
        // Released: the next drag grabs afresh.
        if !done {
            self.grab = Some((down, grab));
        }
        moved
    }

    /// What dragging `grab` from `from` to `to` does.
    #[allow(clippy::too_many_arguments)]
    fn drag_by<Sc: Scene>(
        &mut self,
        e: &mut Editing<'_, Sc>,
        form: &Form,
        handles: &[Visual],
        grab: &mut Grab,
        from: &Pointer,
        to: &Pointer,
        done: bool,
        shift: bool,
    ) -> GeopResult<()> {
        match grab {
            Grab::Handle { key, value } => {
                let track =
                    (handles.iter())
                        .filter(|v| v.key == *key)
                        .find_map(|v| match v.shape {
                            Shape::Handle { at, direction } => Some((at, direction)),
                            _ => None,
                        });
                let moved = track.and_then(|(at, direction)| {
                    let along = |p: &Pointer| p.ray.line_parameter(&at, &direction);
                    Some(along(to)?.sub(along(from)?))
                });
                match moved {
                    Some(moved) => {
                        let value = *value + moved.to_f64();
                        let snapped = (value / DRAG_SNAP).round() * DRAG_SNAP;
                        let (key, value) = (key.clone(), Value::Number(snapped));
                        self.send(e, Event::Field { key, value })
                    }
                    None => Ok(()),
                }
            }
            Grab::Visual { key, plane, start } => {
                let in_plane = |pointer: &Pointer| -> Option<Vector3> {
                    let (origin, normal) = match (&form.focus, &plane) {
                        (Some(focus), _) => (focus.origin(), focus.w()),
                        (None, Some((point, normal))) => (point, normal),
                        (None, None) => return None,
                    };
                    pointer
                        .ray
                        .intersect_plane(origin, normal)
                        .map(|(_, at)| at)
                };
                match (in_plane(from), in_plane(to)) {
                    (Some(from), Some(at)) => {
                        if let Some(start) = start {
                            self.restore(e, start);
                        }
                        let event = CanvasEvent::Move {
                            key: key.clone(),
                            from,
                            to: at,
                            pointer: *to,
                            done,
                            shift,
                        };
                        self.pass(e, event)
                    }
                    _ => Ok(()),
                }
            }
            Grab::Gizmo {
                grab,
                from: asked,
                start,
                readout,
            } => {
                let Some((drag, text)) = grab.drag(from, to, shift) else {
                    return Ok(());
                };
                self.restore(e, start);
                *readout = Some(text);
                let from = Gizmo::clone(asked);
                self.pass(e, CanvasEvent::Gizmo { drag, from, done })
            }
            Grab::Stroke => {
                let event = CanvasEvent::Stroke {
                    from: *from,
                    to: *to,
                    done,
                    shift,
                };
                self.pass(e, event)
            }
        }
    }

    /// What to show: the step's form, with its reference fields saying what
    /// they hold in `scene` — what they are picked from — the handles of
    /// its number fields, what is selected and what the pointer is over
    /// drawn so, what the reference fields hold and what a click would pick
    /// lit, and what a click picks now. While a field waits for a pick,
    /// there is no plane to work in: what can be picked is shown in the
    /// part as it stands — and of the visuals, only the handles can be
    /// grabbed.
    pub fn presentation(
        &self,
        editors: &Editors,
        runner: &Runner,
        part: &Part,
        program: &Program,
        scene: &impl Scene,
    ) -> GeopResult<Presentation> {
        let mut form = self.form(editors, runner, part, program)?;
        let mut pickable = Vec::new();
        for (key, reference) in form.dialog.references_mut() {
            if self.armed.as_deref() == Some(key) {
                reference.armed = true;
                pickable = reference.accepts.clone();
            }
        }
        let mut gizmo = self.gizmo(&form);
        if form.gizmo.as_ref().is_some_and(|g| g.axes.is_some()) {
            let choice = |value: &str, label: &str, orientation: Orientation, title: &str| {
                Action::new(value, label)
                    .group("Gizmo")
                    .title(title)
                    .active(self.orientation == orientation)
            };
            let actions = vec![
                choice(
                    "world",
                    "World",
                    Orientation::World,
                    "Move, turn and scale along the world's axes",
                ),
                choice(
                    "local",
                    "Local",
                    Orientation::Local,
                    "Move, turn and scale along its own axes",
                ),
            ];
            form.dialog.actions(GIZMO_ORIENTATION, actions);
        }
        let mut visuals = form.visuals;
        visuals.extend(handles(&form.dialog));
        let tool = form.tool;
        // While a field waits for a pick, only a handle is hovered: no pick
        // takes one, so a press on it can only mean dragging it. Anything
        // else under the pointer is the field's to pick.
        let waiting = self.armed.is_some();
        if let Some(gizmo) = &mut gizmo {
            match &self.grab {
                Some((_, Grab::Gizmo { grab, readout, .. })) => {
                    gizmo.active = Some(grab.part);
                    gizmo.readout = readout.clone();
                }
                _ => gizmo.hover = self.pointer.and_then(|pointer| gizmo.hit(&pointer)),
            }
        }
        let on_gizmo = gizmo.as_ref().is_some_and(|g| g.hover.is_some());
        let hovered = self.pointer.filter(|_| !on_gizmo).and_then(|pointer| {
            let instance = |name: &str, p: &Pointer| scene.hit_instance(name, p);
            hit_visuals(&visuals, &pointer, instance, |v| match waiting {
                true => is_handle(v),
                false => v.selectable || grabs(v, tool),
            })
            .map(|hit| (hit.visual.key.clone(), grabs(hit.visual, tool)))
        });
        for visual in &mut visuals {
            if is_handle(visual) {
                continue;
            }
            if form.selection.contains(&visual.key) {
                visual.style = Style::Selected;
            } else if hovered.as_ref().is_some_and(|(key, _)| *key == visual.key) {
                visual.style = Style::Hover;
            }
        }
        let highlights = (form.dialog.picked().cloned())
            .chain(self.hover.clone())
            .collect();
        Ok(Presentation {
            dialog: form.dialog,
            visuals,
            highlights,
            pickable,
            focus: form.focus.filter(|_| !waiting),
            sheet: form.sheet.filter(|_| !waiting),
            // A tool that strokes takes a press anywhere.
            grab: self.grab.is_some()
                || on_gizmo
                || hovered.is_some_and(|(_, grab)| grab)
                || (tool == InHand::Strokes && !waiting),
            prompt: form.prompt,
            gizmo,
        })
    }
}

#[cfg(test)]
mod tests;
