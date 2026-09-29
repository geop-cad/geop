//! The gestures most operations share, as state an operation keeps in its
//! session: [`Picking`] an entity for an argument, and [`Dragging`] a
//! handle.

use geop_core_math::{scalars::Scalar, vector::Vector3};
use serde::{Deserialize, Serialize};

use super::{Button, Event, PartView, Pointer, Shape, Target, Visual, hit::hit_visuals};
use crate::operation::EntityRef;

/// Dragged values snap to this: a drag is for rough shaping, a dialog for
/// exact values.
pub const DRAG_SNAP: f64 = 0.01;

/// What a pointer event did to a [`Picking`].
#[derive(Clone, Debug, PartialEq)]
pub enum Picked<S: Scalar> {
    /// Nothing: no field is waiting for a pick, or it was no pointer event.
    Ignored,
    /// The pointer moved; what a click would pick may have changed.
    Hovered,
    /// A click picked `entity`, at `point`, for the field `field`.
    Picked {
        field: String,
        entity: EntityRef,
        point: Vector3<S>,
    },
    /// A click hit nothing that can be picked.
    Missed,
}

/// Picking an entity in the viewport for a dialog field: which field is
/// waiting for a pick, and what a click would pick now.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Picking {
    pub field: Option<String>,
    pub hover: Option<EntityRef>,
}

impl Picking {
    /// Whether `field` is waiting for a pick.
    pub fn is(&self, field: &str) -> bool {
        self.field.as_deref() == Some(field)
    }

    pub fn arm(&mut self, field: &str) {
        self.field = Some(field.to_string());
        self.hover = None;
    }

    pub fn disarm(&mut self) {
        self.field = None;
        self.hover = None;
    }

    /// Arms `field`, or disarms it if it is armed already: what its pick
    /// button does.
    pub fn toggle(&mut self, field: &str) {
        if self.is(field) {
            self.disarm();
        } else {
            self.arm(field);
        }
    }

    /// Picks with `event`, if a field is waiting for a pick: a hover finds
    /// what a click would pick, among `targets` in `view`; a click picks
    /// it. The field stays armed — a caller that wants one pick disarms it.
    pub fn handle<S: Scalar>(
        &mut self,
        view: &PartView<S>,
        event: &Event<S>,
        targets: &[Target],
    ) -> Picked<S> {
        let Some(field) = self.field.clone() else {
            return Picked::Ignored;
        };
        match event {
            Event::Hover { pointer } => {
                self.hover = view.pick(pointer, targets).map(|h| h.entity);
                Picked::Hovered
            }
            Event::Leave => {
                self.hover = None;
                Picked::Hovered
            }
            Event::Click {
                pointer,
                button: Button::Primary,
                ..
            } => {
                self.hover = None;
                match view.pick(pointer, targets) {
                    Some(hit) => Picked::Picked {
                        field,
                        entity: hit.entity,
                        point: hit.point,
                    },
                    None => Picked::Missed,
                }
            }
            _ => Picked::Ignored,
        }
    }
}

/// A handle as it was grabbed: which, and the value it had.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Grab {
    key: String,
    value: f64,
}

/// Dragging a handle along its direction (a [`Shape::Handle`] with one).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Dragging {
    grab: Option<Grab>,
}

/// The handle `visual` is, if it has a direction to be dragged along.
fn track<S: Scalar>(visual: &Visual<S>) -> Option<(Vector3<S>, Vector3<S>)> {
    match visual.shape {
        Shape::Handle {
            at,
            direction: Some(direction),
        } => Some((at, direction)),
        _ => None,
    }
}

impl Dragging {
    /// Whether a press at `pointer` would grab one of the handles among
    /// `visuals` — for [`super::Presentation::grab`].
    pub fn over_handle<S: Scalar>(visuals: &[Visual<S>], pointer: &Pointer<S>) -> bool {
        hit_visuals(visuals, pointer, |v| {
            matches!(v.shape, Shape::Handle { .. })
        })
        .is_some()
    }

    /// On a drag: the key of the handle it moves and the value that gives
    /// it — the value it had when grabbed, moved by how far the pointer has
    /// moved along the handle's track from where the drag started, snapped
    /// to [`DRAG_SNAP`].
    ///
    /// The handle is hit-tested, among `visuals`, where the drag started,
    /// once: while dragged, it moves away from there. After that it is
    /// found by its key, and only its track is used, which a handle keeps
    /// while it slides along it. `value_of` gives the value a handle has
    /// and how many world units it moves per unit of that value. `None` for
    /// any other event, for a drag that did not start on a handle, and for
    /// a pointer that cannot be followed along the handle's track (looking
    /// straight along it).
    pub fn linear<S: Scalar>(
        &mut self,
        visuals: &[Visual<S>],
        event: &Event<S>,
        value_of: impl Fn(&str) -> Option<(f64, f64)>,
    ) -> Option<(String, f64)> {
        let Event::Drag { from, to, done } = event else {
            return None;
        };
        if self.grab.is_none() {
            let hit = hit_visuals(visuals, from, |v| track(v).is_some())?;
            let (value, _) = value_of(&hit.visual.key)?;
            self.grab = Some(Grab {
                key: hit.visual.key.clone(),
                value,
            });
        }
        let grab = self.grab.clone()?;
        if *done {
            self.grab = None;
        }
        let (at, direction) = visuals
            .iter()
            .filter(|v| v.key == grab.key)
            .find_map(track)?;
        let (_, scale) = value_of(&grab.key)?;
        let moved = to
            .ray
            .line_parameter(&at, &direction)?
            .sub(from.ray.line_parameter(&at, &direction)?);
        let value = grab.value + moved.to_f64() / scale;
        Some((grab.key, (value / DRAG_SNAP).round() * DRAG_SNAP))
    }
}
