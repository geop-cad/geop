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
    operation::{EntityRef, Role},
    ui::{Form, Tone},
};

/// The sketch field of a form: a sketch to pick, `set` given its name (none
/// when the field is cleared) — or, before there is any sketch, a hint to
/// draw one.
fn sketch_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    before: &Part<S>,
    sketch: &str,
    set: impl Fn(&mut A, String) + 'a,
) {
    if before.sketches().next().is_none() {
        form.text("sketch", "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let value = if sketch.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Sketch {
            name: sketch.into(),
        }]
    };
    form.reference(
        "sketch",
        "sketch",
        value,
        &[Role::Sketch],
        None,
        false,
        move |edit, picked| {
            let name = match picked.as_slice() {
                [EntityRef::Sketch { name }] => name.clone(),
                _ => String::new(),
            };
            set(edit.args, name);
        },
    );
}
