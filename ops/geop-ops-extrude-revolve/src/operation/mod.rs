//! Extrude and revolve as operations of a program (see
//! [`geop_ops::operation`]): the sketches of a part swept into solids, each
//! kept as a new body or combined with one the part has.

mod extrude;
mod revolve;

pub use extrude::{Extrude, ExtrudeArgs};
pub use revolve::{Revolve, RevolveArgs};

use geop_core_math::scalars::Scalar;
use geop_ops::{
    Part,
    operation::EntityRef,
    ui::{Dialog, Dragging, Event, PartView, Picked, Picking, Target, Tone},
};
use serde::{Deserialize, Serialize};

/// The temporary state of editing an extrude or a revolve.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SweepSession {
    pick: Picking,
    drag: Dragging,
    /// Whether the pointer is over a handle.
    over_handle: bool,
    /// Whether the user chose how to combine: until then, it follows the
    /// sign of the extrude's distance.
    combine_touched: bool,
}

/// The sketch field of a dialog: a button to pick it — or, before there is
/// any sketch, a hint to draw one.
fn sketch_field<S: Scalar>(d: &mut Dialog, before: &Part<S>, sketch: &str, pick: &Picking) {
    if before.sketches().next().is_none() {
        d.text("sketch", "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let shown = if sketch.is_empty() {
        "pick a sketch…"
    } else {
        sketch
    };
    d.pick_button("sketch", format!("sketch: {shown}"), pick.is("sketch"));
}

/// The sketch `event` picks, if the sketch field is waiting for a pick.
fn pick_sketch<S: Scalar>(
    view: &PartView<S>,
    event: &Event<S>,
    pick: &mut Picking,
) -> Option<String> {
    if !pick.is("sketch") {
        return None;
    }
    let Picked::Picked {
        entity: EntityRef::Sketch { name },
        ..
    } = pick.handle(view, event, &[Target::Sketch])
    else {
        return None;
    };
    pick.disarm();
    Some(name)
}

/// What a click picks now: a sketch, or the target to combine with.
fn pickable(pick: &Picking) -> Vec<Target> {
    if pick.is("sketch") {
        vec![Target::Sketch]
    } else {
        geop_ops_booleans::Combine::pickable(pick)
            .into_iter()
            .collect()
    }
}
