//! Programs from bug reports, rebuilt with the coordinates they were
//! reported with: a "clean" equivalent may not carry the same case at all.
//! Each is the smallest program found that still fails.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{
    NamedModel,
    testing::{Built, assert_valid, edges_with_middle_where},
};

use crate::{Fillet, FilletArgs};

/// The operations these programs are written in: given solids, blended.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Fillet)
}

/// Builds `program` and checks the part it builds is valid.
fn assert_builds_valid(program: &Program) {
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The edges of `part` whose middle is at `at`.
fn edges_at(part: &NamedModel, at: [f64; 3]) -> Vec<String> {
    edges_with_middle_where(part, |m| (0..3).all(|k| near(m[k], at[k])))
}

/// A prism over `corners`, 30 high, with its upright edge at `corner`
/// rounded by `radius`.
fn prism_with_rounded_upright(corners: &[[f64; 2]], corner: [f64; 2], radius: f64) -> Program {
    let mut program = Program::new();
    program.push("p", PrimitiveArgs::prism(corners, 0.0, 30.0));
    let before = program.build_model(registry).unwrap();
    let edge = edges_at(&before, [corner[0], corner[1], 15.0]);
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("round", FilletArgs::constant(edge, radius));
    program
}

/// A phone stand's side profile (Fusion 360 beginner tutorial, "Phone
/// stand": the profile sketched on a side plane, extruded, every side edge
/// filleted) has a back rest leaning 4 over 20 from a vertical back: the
/// two faces meet at 169°. Rounding that edge alone fails.
///
/// Rebuilt here as a prism with the same two faces at the corner `(0, 40)`:
/// the vertical side from `(0, 0)` and the side leaning to `(4, 60)`. The
/// fillet's boolean fails tracing the blend tool across the leaning face:
///
/// > the corrector did not bring the two surfaces onto a common point
/// > within 20 iterations: face_a lands at (u=0.0004, v=0.669) and face_b
/// > at (u=0.916, v=1e-16), which cannot be the same point
///
/// at `predictor_corrector_step` from the vertex at `(0, 39.85, 30)` on the
/// prism's top. Ruled out: the size (the same prism at a tenth fails the
/// same way), the radius (0.15 on the small one fails). A chamfer of the
/// same edge builds. Up to 155° the fillet builds; at 160° it fails
/// differently (see [`fillet_of_an_upright_edge_between_faces_at_160_degrees`]).
#[test]
#[ignore = "bug: the fillet of an edge whose faces meet at 169° fails in predictor_corrector_step (the corrector leaves the leaning face); 155° builds, a chamfer builds"]
fn fillet_of_an_upright_edge_between_faces_at_169_degrees() {
    assert_builds_valid(&prism_with_rounded_upright(
        &[
            [0.0, 0.0],
            [70.0, 0.0],
            [12.0, 60.0],
            [4.0, 60.0],
            [0.0, 40.0],
        ],
        [0.0, 40.0],
        1.5,
    ));
}
