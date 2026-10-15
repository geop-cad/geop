//! The parts the tests of holes and of patterns of features drill: a plate
//! with points sketched on its top, and one with a hole cut through it.
//! Only built for tests: with `testing`, or in this crate's own.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{
    Extents,
    testing::{both_ways, disc, extrude_sketch, extrusion, n, on_face, on_origin, outline, sketch},
};
use geop_ops_sketch::{Constraint, Sketch};

/// The plate's size: 60 x 40, 10 thick, its top at `z = 10`.
pub const PLATE: [f64; 3] = [60.0, 40.0, 10.0];

/// A sketch of fixed points at `points`: `p0`, `p1`, ...
pub fn points_sketch(points: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let ids: Vec<_> = points
        .iter()
        .map(|p| s.add_point(n(p[0]), n(p[1])))
        .collect();
    for (id, p) in ids.into_iter().zip(points) {
        s.constrain(Constraint::Fix {
            point: id,
            x: n(p[0]),
            y: n(p[1]),
        });
    }
    s.solve().unwrap();
    s
}

/// The plate `extrude(plate)`, and the sketch `centres` on its top with
/// points at `points` (in world `x`, `y`).
pub fn plate(points: &[[f64; 2]]) -> Program {
    let mut program = Program::new();
    let [w, d, t] = PLATE;
    program.push(
        "outline",
        sketch(
            on_origin(FrameAxis::Z),
            outline(&[[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]),
        ),
    );
    program.push(
        "plate",
        extrusion("outline", Extents::blind(t), Combine::NewBody),
    );
    program.push(
        "centres",
        sketch(on_face("extrude(plate,end)"), points_sketch(points)),
    );
    program
}

/// A plate 8 x 2 x 0.5, `extrude(plate)`, with a hole of radius 0.3 at
/// (1, 1) cut through it by the extrude `hole`.
pub fn plate_with_hole() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "plate",
        on_origin(FrameAxis::Z),
        outline(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
        Combine::NewBody,
    );
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([1.0, 1.0], 0.3),
        both_ways(2.0),
        Combine::cut("extrude(plate)"),
    );
    program
}
