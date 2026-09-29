//! The gestures most operations share, as state an operation keeps in its
//! session: [`Picking`] an entity for an argument, and [`Dragging`] a
//! handle.

use serde::{Deserialize, Serialize};

use super::{
    Button, Event, PartView, Pointer, Shape, Target, Visual,
    hit::{hit_visuals, line_parameter},
};
use crate::operation::EntityRef;

/// Dragged values snap to this: a drag is for rough shaping, a dialog for
/// exact values.
pub const DRAG_SNAP: f64 = 0.01;

/// What a pointer event did to a [`Picking`].
#[derive(Clone, Debug, PartialEq)]
pub enum Picked {
    /// Nothing: no field is waiting for a pick, or it was no pointer event.
    Ignored,
    /// The pointer moved; what a click would pick may have changed.
    Hovered,
    /// A click picked `entity`, at `point`, for the field `field`.
    Picked {
        field: String,
        entity: EntityRef,
        point: [f64; 3],
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
    pub fn handle(&mut self, view: &PartView, event: &Event, targets: &[Target]) -> Picked {
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

/// A handle as it was grabbed: which, where its track runs, and the value
/// it had.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Grab {
    key: String,
    at: [f64; 3],
    direction: [f64; 3],
    /// Where along its track it was grabbed.
    start: f64,
    value: f64,
    /// World units the handle moves per unit of its value.
    scale: f64,
}

/// Dragging a handle along its direction (a [`Shape::Handle`] with one).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Dragging {
    grab: Option<Grab>,
}

impl Dragging {
    /// Whether a press at `pointer` would grab one of the handles among
    /// `visuals` — for [`super::Presentation::grab`].
    pub fn over_handle(visuals: &[Visual], pointer: &Pointer) -> bool {
        hit_visuals(visuals, pointer, |v| {
            matches!(v.shape, Shape::Handle { .. })
        })
        .is_some()
    }

    /// On a drag: the key of the handle it moves and the value that gives
    /// it — the value it had when grabbed, moved by how far the pointer has
    /// since moved along the handle's direction, snapped to [`DRAG_SNAP`].
    ///
    /// The handle is hit-tested, among `visuals`, where the drag started,
    /// once: while dragged, it moves away from there. `value_of` gives the
    /// value a handle has then, and how many world units it moves per unit
    /// of that value. `None` for any other event, for a drag that did not
    /// start on a handle, and for a pointer that cannot be followed along
    /// the handle's track (looking straight along it).
    pub fn linear(
        &mut self,
        visuals: &[Visual],
        event: &Event,
        value_of: impl Fn(&str) -> Option<(f64, f64)>,
    ) -> Option<(String, f64)> {
        let Event::Drag { from, to, done } = event else {
            return None;
        };
        if self.grab.is_none() {
            let hit = hit_visuals(visuals, from, |v| {
                matches!(
                    v.shape,
                    Shape::Handle {
                        direction: Some(_),
                        ..
                    }
                )
            })?;
            let Shape::Handle {
                at,
                direction: Some(direction),
            } = hit.visual.shape
            else {
                unreachable!("only handles with a direction are hit");
            };
            let (value, scale) = value_of(&hit.visual.key)?;
            self.grab = Some(Grab {
                key: hit.visual.key.clone(),
                at,
                direction,
                start: line_parameter(from, at, direction)?,
                value,
                scale,
            });
        }
        let grab = self.grab.clone()?;
        if *done {
            self.grab = None;
        }
        let s = line_parameter(to, grab.at, grab.direction)?;
        let value = grab.value + (s - grab.start) / grab.scale;
        Some((grab.key, (value / DRAG_SNAP).round() * DRAG_SNAP))
    }
}
