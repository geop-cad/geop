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
    ui::{Dialog, Target, Tone},
};

/// The sketch field of a dialog: a sketch to pick — or, before there is
/// any sketch, a hint to draw one.
fn sketch_field<S: Scalar>(d: &mut Dialog, before: &Part<S>, sketch: &str) {
    if before.sketches().next().is_none() {
        d.text("sketch", "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let value = if sketch.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Sketch {
            name: sketch.into(),
        }]
    };
    d.pick("sketch", "sketch", value, &[Target::Sketch], false);
}
