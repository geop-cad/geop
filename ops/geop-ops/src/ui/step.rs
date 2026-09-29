//! [`StepEditor`]: editing one step — the interaction every operation
//! shares, so a pick or a drag means the same whatever is edited.

use std::any::Any;

use geop_core_math::{scalars::Scalar, vector::Vector3};

use super::{
    Button, Control, Dialog, Event, PartView, Pointer, Presentation, Shape, Target, Value, Visual,
    hit::hit_visuals,
};
use crate::{
    Part,
    operation::{EntityRef, Operations},
};

/// Dragged values snap to this: a drag is for rough shaping, a dialog for
/// exact values.
pub const DRAG_SNAP: f64 = 0.01;

/// A handle as it was grabbed: its field, and the value that had.
struct Grab {
    key: String,
    value: f64,
}

/// A step being edited: the step with its arguments as they now are, and
/// the state of the edit — the operation's session, and the editor's own:
/// which pick field waits for a click, what a click would pick, which
/// handle is being dragged.
///
/// It turns every event into what the operation understands: a dialog
/// field's value, an entity picked for a pick field, a number dragged along
/// a handle go to [`Operations::set`]; whatever else the pointer and the
/// keys do goes to [`Operations::event`].
pub struct StepEditor<O> {
    step: O,
    session: Box<dyn Any>,
    /// The pick field waiting for a click.
    armed: Option<String>,
    /// What a click would pick for it.
    hover: Option<EntityRef>,
    grab: Option<Grab>,
    /// Whether the pointer is over a handle.
    over_handle: bool,
}

/// Where the handle `visual` is and what it slides along, if it is one.
fn track<S: Scalar>(visual: &Visual<S>) -> Option<(Vector3<S>, Vector3<S>)> {
    match visual.shape {
        Shape::Handle { at, direction } => Some((at, direction)),
        _ => None,
    }
}

impl<O: Operations> StepEditor<O> {
    /// Editing `step` against `before`. A new step starts by picking for
    /// its first pick field: what it is built on is what it needs first.
    pub fn new<S: Scalar>(step: O, before: &Part<S>, new: bool) -> Self {
        let session = step.new_session();
        let armed = new
            .then(|| {
                step.form(before, &*session)
                    .dialog
                    .0
                    .into_iter()
                    .find(|f| matches!(f.control, Control::Pick { .. }))
                    .map(|f| f.key)
            })
            .flatten();
        Self {
            step,
            session,
            armed,
            hover: None,
            grab: None,
            over_handle: false,
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

    /// Applies `event` — `view` is `before` as drawn, what picks test
    /// against.
    pub fn handle<S: Scalar>(&mut self, before: &Part<S>, view: &PartView<S>, event: &Event<S>) {
        let form = self.step.form(before, &*self.session);
        if let Event::Dialog { key, value } = event {
            if let Some(Control::Pick { .. }) = form.dialog.get(key) {
                self.armed = (self.armed.as_deref() != Some(key)).then(|| key.clone());
                self.hover = None;
            } else {
                self.step
                    .set(before, &mut *self.session, key, value.clone());
            }
            return;
        }
        if let Some(key) = self.armed.clone() {
            if let Some(Control::Pick {
                targets, multiple, ..
            }) = form.dialog.get(&key)
            {
                self.pick(before, view, event, &key, targets, *multiple);
                return;
            }
            self.armed = None;
        }
        if let Event::Drag { from, to, done } = event
            && let Some((key, value)) = self.drag(&form.dialog, &form.visuals, from, to, *done)
        {
            self.step
                .set(before, &mut *self.session, &key, Value::Number(value));
            return;
        }
        if let Event::Hover { pointer } = event {
            self.over_handle = over_handle(&form.visuals, pointer);
        }
        self.step.event(before, &mut *self.session, event);
    }

    /// A pointer event while the pick field `key` waits for a click: a
    /// hover finds what a click would pick, a click picks it — and, for a
    /// field of one entity, is done. Escape stops picking.
    fn pick<S: Scalar>(
        &mut self,
        before: &Part<S>,
        view: &PartView<S>,
        event: &Event<S>,
        key: &str,
        targets: &[Target],
        multiple: bool,
    ) {
        match event {
            Event::Hover { pointer } => {
                self.hover = view.pick(pointer, targets).map(|h| h.entity);
            }
            Event::Leave => self.hover = None,
            Event::Click {
                pointer,
                button: Button::Primary,
                ..
            } => {
                self.hover = None;
                if let Some(hit) = view.pick(pointer, targets) {
                    if !multiple {
                        self.armed = None;
                    }
                    self.step
                        .set(before, &mut *self.session, key, Value::Entity(hit.entity));
                }
            }
            Event::Key { key } if key == "Escape" => {
                self.armed = None;
                self.hover = None;
            }
            _ => {}
        }
    }

    /// On a drag of a handle: its field, and the value that gives it — the
    /// value it had when grabbed, moved by how far the pointer has moved
    /// along the handle's track, snapped to [`DRAG_SNAP`].
    ///
    /// The handle is hit-tested where the drag started, once: while
    /// dragged, it moves away from there. After that it is found by its key,
    /// and only its track is used, which a handle keeps while it slides
    /// along it. `None` for a drag that did not start on a handle, and for
    /// a pointer that cannot be followed along the track (looking straight
    /// along it).
    fn drag<S: Scalar>(
        &mut self,
        dialog: &Dialog,
        visuals: &[Visual<S>],
        from: &Pointer<S>,
        to: &Pointer<S>,
        done: bool,
    ) -> Option<(String, f64)> {
        if self.grab.is_none() {
            let hit = hit_visuals(visuals, from, |v| track(v).is_some())?;
            let Some(Control::Number { value, .. }) = dialog.get(&hit.visual.key) else {
                return None;
            };
            self.grab = Some(Grab {
                key: hit.visual.key.clone(),
                value: *value,
            });
        }
        let grab = self.grab.as_ref()?;
        let key = grab.key.clone();
        let moved = visuals
            .iter()
            .filter(|v| v.key == key)
            .find_map(track)
            .and_then(|(at, direction)| {
                Some(
                    to.ray
                        .line_parameter(&at, &direction)?
                        .sub(from.ray.line_parameter(&at, &direction)?),
                )
            });
        let value = grab.value;
        if done {
            // Released: the next drag grabs afresh.
            self.grab = None;
        }
        let value = value + moved?.to_f64();
        Some((key, (value / DRAG_SNAP).round() * DRAG_SNAP))
    }

    /// What to show: the step's form, what its pick fields hold and what a
    /// click would pick lit, and what a click picks now. While a field
    /// waits for a pick, there is no plane to work in: what can be picked
    /// is shown in the part as it stands.
    pub fn presentation<S: Scalar>(&self, before: &Part<S>) -> Presentation<S> {
        let mut form = self.step.form(before, &*self.session);
        let mut pickable = Vec::new();
        if let Some(key) = &self.armed
            && let Some(Control::Pick { armed, targets, .. }) = form.dialog.get_mut(key)
        {
            *armed = true;
            pickable = targets.clone();
        }
        let highlights = form
            .dialog
            .picked()
            .cloned()
            .chain(self.hover.clone())
            .collect();
        Presentation {
            dialog: form.dialog,
            visuals: form.visuals,
            highlights,
            pickable,
            focus: if self.armed.is_some() {
                None
            } else {
                form.focus
            },
            grab: form.grab || self.over_handle,
        }
    }
}

/// Whether a press at `pointer` would grab one of the handles among
/// `visuals`.
fn over_handle<S: Scalar>(visuals: &[Visual<S>], pointer: &Pointer<S>) -> bool {
    hit_visuals(visuals, pointer, |v| track(v).is_some()).is_some()
}
