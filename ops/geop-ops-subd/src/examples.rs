//! The examples of subdivision surfaces an editor offers: a computer mouse,
//! and a mug with a handle.

use geop_core_editor::registry::ExampleSet;
use geop_core_program::program::Program;

use geop_core_math::plain::centroid;

use crate::{Cage, Mirror, SubdArgs};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("subd_mouse", subd_mouse), ("subd_mug", subd_mug)],
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
    cage.set_sharpness(&rim, 1.0);
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

/// The face of `cage` whose centre is nearest `point`, by id.
#[allow(clippy::expect_used, reason = "the cage is the example's own")]
fn face_near(cage: &Cage, point: [f64; 3]) -> u32 {
    let centre = |ids: &[u32]| {
        centroid(ids.iter().map(|&v| cage.vertex(v).expect("a corner").at)).expect("corners")
    };
    let distance = |c: [f64; 3]| (0..3).map(|k| (c[k] - point[k]).powi(2)).sum::<f64>();
    (cage.faces.iter())
        .min_by(|a, b| distance(centre(&a.vertices)).total_cmp(&distance(centre(&b.vertices))))
        .expect("a face")
        .id
}

/// A mug, shaped as a subdivision surface with the box modelling tools:
/// half an eight-sided cylinder mirrored in `y = 0`, cut into five rows,
/// its handle two side faces extruded out and bridged, its inside the top
/// inset and extruded down to a floor; the bottom's rim creased sharp to
/// stand on, the lip creased soft to round off. The body is `subd(mug)`.
#[allow(clippy::expect_used, reason = "the cage is the example's own")]
pub fn subd_mug() -> Program {
    let mirror = Mirror::Y;
    let mut cage = Cage::cylinder(1.0, 2.4, 8);
    cage.halve(1).expect("the cylinder reaches across y = 0");
    // Three rows into five: the vertical edges of the lower and the upper
    // row cut half way.
    cage.insert_loop(0, 8, 0.5).expect("a ring of quads");
    cage.insert_loop(8, 16, 0.5).expect("a ring of quads");
    // The handle: the lowest and the highest face on the `+x` side, pulled
    // out and joined.
    let lower = face_near(&cage, [0.95, 0.2, -0.9]);
    let upper = face_near(&cage, [0.95, 0.2, 0.9]);
    cage.extrude(&[lower].into(), 0.8, mirror).expect("a face");
    cage.extrude(&[upper].into(), 0.8, mirror).expect("a face");
    cage.bridge(lower, upper, mirror).expect("two quads");
    // The inside: the top inset for the wall, then sunk to the floor.
    let top = face_near(&cage, [0.0, 0.5, 1.2]);
    let lip: Vec<[u32; 2]> = cage
        .edges()
        .into_iter()
        .filter(|&[a, b]| {
            let (p, q) = (cage.vertex(a).expect("a").at, cage.vertex(b).expect("b").at);
            p[2] == 1.2 && q[2] == 1.2 && !(p[1] == 0.0 && q[1] == 0.0)
        })
        .collect();
    cage.inset(&[top].into(), 0.15, mirror).expect("a face");
    cage.extrude(&[top].into(), -2.0, mirror).expect("a face");
    cage.set_sharpness(&lip, 0.5);
    let bottom: Vec<[u32; 2]> = cage
        .edges()
        .into_iter()
        .filter(|&[a, b]| {
            let (p, q) = (cage.vertex(a).expect("a").at, cage.vertex(b).expect("b").at);
            p[2] == -1.2 && q[2] == -1.2 && !(p[1] == 0.0 && q[1] == 0.0)
        })
        .collect();
    cage.set_sharpness(&bottom, 1.0);
    let mut program = Program::new();
    program.push("mug", SubdArgs { cage, mirror });
    program
}
