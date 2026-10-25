//! The parts the tests of holes and of patterns of features drill: a plate
//! with points sketched on its top. Only built for this crate's tests.

use geop_core_program::program::Program;
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::sketch::{
    Constraint, Sketch,
    figures::{n, on_face},
};
use geop_ops_sketch::AddSketchArgs;

/// The plate's size: 60 x 40, 10 thick, its top at `z = 10`.
pub const PLATE: [f64; 3] = [60.0, 40.0, 10.0];

/// The plate's top, the face the holes are drilled into.
pub const PLATE_TOP: &str = "block(plate,end)";

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

/// The plate `block(plate)` on the origin, and the sketch `centres` on its
/// top with points at `points` (in world `x`, `y`).
pub fn plate(points: &[[f64; 2]]) -> Program {
    let mut program = Program::new();
    let [w, d, t] = PLATE;
    program.push("plate", PrimitiveArgs::block([0.0, 0.0, 0.0], [w, d, t]));
    program.push(
        "centres",
        AddSketchArgs::on(on_face(PLATE_TOP), points_sketch(points)),
    );
    program
}
