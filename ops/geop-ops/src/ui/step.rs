//! [`StepEditor`]: editing one step — the interaction every operation
//! shares, so a pick, a selection or a drag means the same whatever is
//! edited.

use std::any::Any;

use geop_core_math::{scalars::Scalar, vector::Vector3};

use super::{
    Button, CanvasEvent, Control, Dialog, PartView, Pointer, Presentation, Reference, Shape,
    StepEditEvent, Style, Tone, Value, Visual, hit::hit_visuals,
};
use crate::{
    Part,
    operation::{Aspects, EntityRef, Operations, Role, describe_roles},
    ui::Form,
};

/// Dragged values snap to this: a drag is for rough shaping, a dialog for
/// exact values.
pub const DRAG_SNAP: f64 = 0.01;

/// What a drag holds, as it was grabbed.
enum Grab {
    /// The handle of the number field `key`, and the value that had.
    Handle { key: String, value: f64 },
    /// A draggable visual of the operation's.
    Visual { key: String },
}

/// A step being edited: the step with its arguments as they now are, and
/// the state of the edit — the operation's session, and the editor's own:
/// what is selected, which reference field waits for a click and what a
/// click would pick for it, where the pointer is, what is being dragged.
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
///   is held, and Escape clears it. The selection is the operation's to
///   read, and to change as its fields are used.
/// - A draggable visual is dragged in the plane worked in, as a
///   [`CanvasEvent::Move`].
///
/// Whatever else the pointer and the keys do goes to the operation as a
/// [`CanvasEvent`]: clicks while it has a tool in hand, and those on
/// nothing, hovers, keys.
pub struct StepEditor<O, S: Scalar> {
    step: O,
    session: Box<dyn Any>,
    /// The keys of the visuals selected, in the order they were.
    selection: Vec<String>,
    /// The reference field waiting for a click.
    armed: Option<String>,
    /// What a click would pick for it.
    hover: Option<EntityRef>,
    /// Where the pointer last was, if over the viewport.
    pointer: Option<Pointer<S>>,
    grab: Option<Grab>,
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
fn handles<S: Scalar>(dialog: &Dialog<S>) -> Vec<Visual<S>> {
    dialog
        .0
        .iter()
        .filter_map(|f| match &f.control {
            Control::Number(n) => n.handle.map(|track| {
                Visual::new(
                    f.key.clone(),
                    Shape::Handle {
                        at: track.at,
                        direction: track.direction,
                    },
                    Style::Handle,
                )
            }),
            _ => None,
        })
        .collect()
}

fn is_handle<S: Scalar>(visual: &Visual<S>) -> bool {
    matches!(visual.shape, Shape::Handle { .. })
}

/// Whether a press on `visual` grabs it: a handle always, a draggable
/// visual unless a tool is in hand.
fn grabs<S: Scalar>(visual: &Visual<S>, tool: bool) -> bool {
    is_handle(visual) || (visual.draggable && !tool)
}

/// Says what each entity `reference` holds is, in `part`: what it can be
/// used as, if the field takes several kinds — or why it cannot be used.
fn describe<S: Scalar>(part: &Part<S>, reference: &mut Reference) {
    for picked in &mut reference.value {
        let fits: Vec<Role> = match Aspects::of(&picked.entity, part) {
            Ok(aspects) => aspects
                .roles()
                .into_iter()
                .filter(|r| reference.roles.contains(r))
                .collect(),
            Err(e) => {
                picked.tone = Tone::Error;
                picked.detail = Some(e.root_message().to_string());
                continue;
            }
        };
        let in_scope = reference
            .scope
            .as_ref()
            .is_none_or(|scope| picked.entity.lies_in(scope));
        if fits.is_empty() || !in_scope {
            picked.tone = Tone::Error;
            let scope = match &reference.scope {
                Some(scope) => format!(" of {scope}"),
                None => String::new(),
            };
            picked.detail = Some(format!(
                "not {}{scope}",
                describe_roles(&reference.roles, " or ")
            ));
        } else if reference.roles.len() > 1 {
            let names: Vec<&str> = fits.iter().map(|r| r.name()).collect();
            picked.detail = Some(names.join(" · "));
        }
    }
}

impl<O: Operations, S: Scalar> StepEditor<O, S> {
    /// Editing `step` against `before`. A new step whose first reference
    /// field is still empty starts by picking for it: what it is built on
    /// is what it needs first. One that already holds something — the
    /// newest sketch, say — needs no click, and leaves the pointer to the
    /// rest of the step: its handles, its drawing.
    pub fn new(step: O, before: &Part<S>, new: bool) -> Self {
        let session = step.new_session();
        let armed = new
            .then(|| {
                step.form(before, &*session, &[])
                    .dialog
                    .0
                    .into_iter()
                    .find(|f| matches!(f.control, Control::Reference(_)))
                    .filter(|f| matches!(&f.control, Control::Reference(r) if r.value.is_empty()))
                    .map(|f| f.key)
            })
            .flatten();
        Self {
            step,
            session,
            selection: Vec::new(),
            armed,
            hover: None,
            pointer: None,
            grab: None,
        }
    }

    /// The step, with its arguments as they now are.
    pub fn step(&self) -> &O {
        &self.step
    }

    /// The operation's session: its own, to look into.
    pub fn session(&self) -> &dyn Any {
        &*self.session
    }

    /// The keys of the visuals selected.
    pub fn selection(&self) -> &[String] {
        &self.selection
    }

    fn form<'a>(&self, before: &'a Part<S>) -> Form<'a, S> {
        self.step.form(before, &*self.session, &self.selection)
    }

    fn set(&mut self, before: &Part<S>, key: &str, value: Value) {
        self.step
            .set(before, &mut *self.session, &mut self.selection, key, value);
    }

    fn pass(&mut self, before: &Part<S>, event: CanvasEvent<S>) {
        self.step
            .event(before, &mut *self.session, &mut self.selection, &event);
    }

    /// Applies `event` — `view` is `before` as drawn, what picks test
    /// against.
    pub fn handle(&mut self, before: &Part<S>, view: &PartView<S>, event: &StepEditEvent<S>) {
        let form = self.form(before);
        if let StepEditEvent::Dialog { key, value } = event {
            self.dialog(before, &form.dialog, key, value.clone());
            return;
        }
        match event {
            StepEditEvent::Hover { pointer } | StepEditEvent::Click { pointer, .. } => {
                self.pointer = Some(*pointer)
            }
            StepEditEvent::Leave => self.pointer = None,
            _ => {}
        }
        if let Some(key) = self.armed.clone() {
            if let Some(Control::Reference(reference)) = form.dialog.get(&key) {
                self.pick(before, view, event, &key, reference);
                return;
            }
            self.armed = None;
        }
        match event {
            StepEditEvent::Dialog { .. } => unreachable!("handled above"),
            StepEditEvent::Hover { pointer } => {
                self.pass(before, CanvasEvent::Hover { pointer: *pointer })
            }
            StepEditEvent::Leave => self.pass(before, CanvasEvent::Leave),
            StepEditEvent::Click {
                pointer,
                button,
                double,
                shift,
            } => {
                let selectable = |v: &Visual<S>| v.selectable;
                let hit = (*button == Button::Primary && !*double && !form.tool)
                    .then(|| hit_visuals(&form.visuals, pointer, selectable))
                    .flatten();
                match hit {
                    Some(hit) => toggle(&mut self.selection, hit.visual.key.clone()),
                    None => {
                        if *button == Button::Primary && !*shift && !form.tool {
                            self.selection.clear();
                        }
                        self.pass(
                            before,
                            CanvasEvent::Click {
                                pointer: *pointer,
                                button: *button,
                                double: *double,
                                shift: *shift,
                            },
                        );
                    }
                }
            }
            StepEditEvent::Drag { from, to, done } => self.drag(before, &form, from, to, *done),
            StepEditEvent::Key { key } => {
                if key == "Escape" {
                    self.selection.clear();
                }
                self.pass(before, CanvasEvent::Key { key: key.clone() });
            }
        }
    }

    /// The dialog field `key` used: a reference field armed, or an entity
    /// taken out of it, or all of them; any other field set.
    fn dialog(&mut self, before: &Part<S>, dialog: &Dialog<S>, key: &str, value: Value) {
        let Some(Control::Reference(reference)) = dialog.get(key) else {
            self.set(before, key, value);
            return;
        };
        let mut entities: Vec<EntityRef> = reference.entities().cloned().collect();
        match value {
            Value::Press => {
                self.armed = (self.armed.as_deref() != Some(key)).then(|| key.to_string());
                self.hover = None;
                return;
            }
            Value::RemoveAt(i) if i < entities.len() => {
                entities.remove(i);
            }
            Value::Clear => entities.clear(),
            Value::Entities(set) => entities = set,
            _ => return,
        }
        self.set(before, key, Value::Entities(entities));
    }

    /// A pointer event while the reference field `key` waits for a click:
    /// a hover finds what a click would pick, a click picks it — and, for a
    /// field of one entity, is done. Escape stops picking.
    fn pick(
        &mut self,
        before: &Part<S>,
        view: &PartView<S>,
        event: &StepEditEvent<S>,
        key: &str,
        reference: &Reference,
    ) {
        let pick = |pointer| view.pick(pointer, &reference.roles, reference.scope.as_ref());
        match event {
            StepEditEvent::Hover { pointer } => self.hover = pick(pointer).map(|h| h.entity),
            StepEditEvent::Leave => self.hover = None,
            StepEditEvent::Click {
                pointer,
                button: Button::Primary,
                ..
            } => {
                self.hover = None;
                if let Some(hit) = pick(pointer) {
                    let mut entities: Vec<EntityRef> = reference.entities().cloned().collect();
                    if reference.multiple {
                        toggle(&mut entities, hit.entity);
                    } else {
                        entities = vec![hit.entity];
                        self.armed = None;
                    }
                    self.set(before, key, Value::Entities(entities));
                }
            }
            StepEditEvent::Key { key } if key == "Escape" => {
                self.armed = None;
                self.hover = None;
            }
            _ => {}
        }
    }

    /// A drag: of a number field's handle — its value when grabbed, moved
    /// by how far the pointer has moved along the handle's track, snapped
    /// to [`DRAG_SNAP`] — or of a draggable visual, in the plane worked in.
    ///
    /// What is dragged is hit-tested where the drag started, once: while
    /// dragged, it moves away from there. After that it is found by its key.
    /// Nothing for a drag that did not start on either, and for a pointer
    /// that cannot be followed (looking straight along a handle's track, or
    /// along the plane).
    fn drag(
        &mut self,
        before: &Part<S>,
        form: &Form<S>,
        from: &Pointer<S>,
        to: &Pointer<S>,
        done: bool,
    ) {
        let handles = handles(&form.dialog);
        if self.grab.is_none() {
            let visuals: Vec<Visual<S>> = form.visuals.iter().chain(&handles).cloned().collect();
            let Some(hit) = hit_visuals(&visuals, from, |v| grabs(v, form.tool)) else {
                return;
            };
            let key = hit.visual.key.clone();
            self.grab = Some(match form.dialog.get(&key) {
                Some(Control::Number(n)) if is_handle(hit.visual) => Grab::Handle {
                    key,
                    value: n.value,
                },
                _ => Grab::Visual { key },
            });
        }
        let Some(grab) = &self.grab else {
            return;
        };
        match grab {
            Grab::Handle { key, value } => {
                let (key, value) = (key.clone(), *value);
                if done {
                    // Released: the next drag grabs afresh.
                    self.grab = None;
                }
                let moved = handles
                    .iter()
                    .filter(|v| v.key == key)
                    .find_map(|v| match v.shape {
                        Shape::Handle { at, direction } => Some((at, direction)),
                        _ => None,
                    })
                    .and_then(|(at, direction)| {
                        Some(
                            to.ray
                                .line_parameter(&at, &direction)?
                                .sub(from.ray.line_parameter(&at, &direction)?),
                        )
                    });
                if let Some(moved) = moved {
                    let value = value + moved.to_f64();
                    let snapped = (value / DRAG_SNAP).round() * DRAG_SNAP;
                    self.set(before, &key, Value::Number(snapped));
                }
            }
            Grab::Visual { key } => {
                let key = key.clone();
                if done {
                    self.grab = None;
                }
                let in_plane = |pointer: &Pointer<S>| -> Option<Vector3<S>> {
                    let (t, _) = pointer.ray.intersect_uv_plane(form.focus.as_ref()?)?;
                    Some(pointer.ray.at(t))
                };
                if let (Some(from), Some(to)) = (in_plane(from), in_plane(to)) {
                    self.pass(
                        before,
                        CanvasEvent::Move {
                            key,
                            from,
                            to,
                            done,
                        },
                    );
                }
            }
        }
    }

    /// What to show: the step's form, with its reference fields saying what
    /// they hold, the handles of its number fields, what is selected and
    /// what the pointer is over drawn so, what the reference fields hold
    /// and what a click would pick lit, and what a click picks now. While a
    /// field waits for a pick, there is no plane to work in: what can be
    /// picked is shown in the part as it stands.
    pub fn presentation(&self, before: &Part<S>) -> Presentation<S> {
        let mut form = self.form(before);
        let mut pickable = Vec::new();
        for (key, reference) in form.dialog.references_mut() {
            describe(before, reference);
            if self.armed.as_deref() == Some(key) {
                reference.armed = true;
                pickable = reference.roles.clone();
            }
        }
        let mut visuals = form.visuals;
        visuals.extend(handles(&form.dialog));
        let tool = form.tool;
        let hovered = self
            .pointer
            .filter(|_| self.armed.is_none())
            .and_then(|pointer| {
                hit_visuals(&visuals, &pointer, |v| v.selectable || grabs(v, tool))
                    .map(|hit| (hit.visual.key.clone(), grabs(hit.visual, tool)))
            });
        for visual in &mut visuals {
            if is_handle(visual) {
                continue;
            }
            if self.selection.contains(&visual.key) {
                visual.style = Style::Selected;
            } else if hovered.as_ref().is_some_and(|(key, _)| *key == visual.key) {
                visual.style = Style::Hover;
            }
        }
        let highlights = form
            .dialog
            .picked()
            .cloned()
            .chain(self.hover.clone())
            .collect();
        Presentation {
            dialog: form.dialog,
            visuals,
            highlights,
            pickable,
            focus: if self.armed.is_some() {
                None
            } else {
                form.focus
            },
            grab: self.grab.is_some() || hovered.is_some_and(|(_, grab)| grab),
        }
    }
}
