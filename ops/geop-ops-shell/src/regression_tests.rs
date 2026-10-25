//! Programs from tutorials and bug reports, rebuilt in Rust: the shell of a
//! box whose edges were rounded first, as the beginners' lessons do it.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::{
    NamedModel, model, solid,
    testing::{Built, assert_valid, edges_with_middle_where},
};

use crate::{Shell, ShellArgs};

/// The operations these programs are written in: given solids, rounded
/// and shelled.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Fillet)?.with(Shell)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The planar face of `part` facing `+z` at height `z`.
fn top_face_at(part: &NamedModel, z: f64) -> String {
    let mut names: Vec<String> = (part.names().iter())
        .filter(|(id, _)| matches!(id, geop_ops_model::RefId::Face(_)))
        .map(|(_, name)| name.to_string())
        .filter(|name| {
            let face = part
                .topology()
                .get_face(part.face_id(name).unwrap())
                .unwrap();
            face.surface.as_plane().unwrap().is_some_and(|plane| {
                plane.normal[2].to_f64() > 0.999 && near(plane.point[2].to_f64(), z)
            })
        })
        .collect();
    names.sort();
    assert_eq!(names.len(), 1, "{names:?}");
    names.remove(0)
}

/// SolidWorks Lesson 1, in its order: a 100 x 60 x 50 block, its four
/// vertical edges rounded with radius 10, then shelled from the top with
/// walls 5 thick.
///
/// The shell is refused: "the edge is neither straight nor circular:
/// offsetting it is not supported", naming the edge where a blend meets
/// the bottom face. That edge is a quarter circle of radius 10 (a rolling
/// ball between two planes meets a third plane square to both in an arc),
/// but the fillet's boolean traced it as a fitted curve, so the shell does
/// not recognise it. Shelling first and rounding afterwards builds (see
/// `geop-ops-fillet`'s regression tests for what that leaves behind).
#[test]
#[ignore = "bug: shelling a box whose vertical edges were rounded is refused — the blends' edges on the caps are quarter circles the fillet left as traced curves, which the shell cannot offset"]
fn shell_of_a_box_with_rounded_vertical_edges() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [100.0, 60.0, 50.0]),
    );
    let part = program.build_model(registry).unwrap();
    let vertical = edges_with_middle_where(&part, |c| near(c[2], 25.0));
    assert_eq!(vertical.len(), 4, "{vertical:?}");
    program.push("round", FilletArgs::constant(vertical, 10.0));
    let part = program.build_model(registry).unwrap();
    let top = top_face_at(&part, 50.0);
    program.push(
        "shell",
        ShellArgs {
            solid: solid("fillet(round)"),
            faces: vec![ElementReference::new(model(), top)],
            thickness: 5.0.into(),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let id = part.solid_id("shell(shell)").unwrap();
    let volume = part.topology().mass_properties(id).unwrap().volume.to_f64();
    let corner = 1.0 - std::f64::consts::PI / 4.0;
    let outer = 100.0 * 60.0 * 50.0 - 4.0 * corner * 100.0 * 50.0;
    let inner = 90.0 * 50.0 * 45.0 - 4.0 * corner * 25.0 * 45.0;
    assert!(
        (volume - (outer - inner)).abs() < 1e-6,
        "{volume} vs {}",
        outer - inner
    );
}
