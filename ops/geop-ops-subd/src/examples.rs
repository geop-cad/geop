//! The examples of subdivision surfaces an editor offers: a computer mouse.

use geop_core_editor::registry::ExampleSet;
use geop_core_program::program::Program;

use crate::{Cage, Mirror, SubdArgs};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("subd_mouse", subd_mouse)],
        workspaces: Vec::new(),
    }
}

/// A computer mouse, shaped as a subdivision surface: half a 1.2 x 2 x 0.8
/// box cage, mirrored in `x = 0`, its back raised into a hump and the rim
/// of its bottom creased, so that it stands on a flat sole with a sharp
/// edge while everything above rounds off. The body is `subd(mouse)`.
#[allow(clippy::expect_used, reason = "the cage is the example's own")]
pub fn subd_mouse() -> Program {
    let mut cage = Cage::cuboid([1.2, 2.0, 0.8]);
    cage.halve(0).expect("the box reaches across x = 0");
    let bottom = cage.face(8).expect("the box's bottom").vertices.clone();
    let on_plane = |v: u32| cage.vertex(v).is_ok_and(|x| x.at[0] == 0.0);
    let rim: Vec<[u32; 2]> = (0..bottom.len())
        .map(|k| [bottom[k], bottom[(k + 1) % bottom.len()]])
        .filter(|&[a, b]| !(on_plane(a) && on_plane(b)))
        .collect();
    cage.set_crease(&rim, true);
    let back = cage
        .vertices
        .iter()
        .filter(|v| v.at[1] > 0.0 && v.at[2] > 0.0)
        .map(|v| v.id)
        .collect();
    cage.transform(&back, Mirror::X, |p| [p[0], p[1], p[2] + 0.3]);
    let mut program = Program::new();
    program.push(
        "mouse",
        SubdArgs {
            cage,
            mirror: Mirror::X,
        },
    );
    program
}
