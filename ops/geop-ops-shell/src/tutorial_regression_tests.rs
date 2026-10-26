//! Programs from tutorials and bug reports, rebuilt in Rust: the shell of a
//! box whose edges were rounded first, as the beginners' lessons do it.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
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
/// The shell was refused: "the edge is neither straight nor circular:
/// offsetting it is not supported", naming the edge where a blend meets
/// the bottom face. That edge is a quarter circle of radius 10 (a rolling
/// ball between two planes meets a third plane square to both in an arc),
/// which the fillet's boolean traced as a fitted curve. The boolean now
/// splices the exact arc there (`geop_core_geometry::shape::circle_through`).
///
/// The volumes are integrated enclosures (the blend's spans are rational,
/// so the quadrature's error is estimated), and each has to hold the true
/// volume: the rounded block's, and the walls'.
#[test]
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
    let corner = 1.0 - std::f64::consts::PI / 4.0;
    let outer = 100.0 * 60.0 * 50.0 - 4.0 * corner * 100.0 * 50.0;
    let volume = |part: &NamedModel, name: &str| {
        let id = part.solid_id(name).unwrap();
        part.topology().mass_properties(id).unwrap().volume
    };
    // Integrated: an enclosure of the volume, which has to hold the true one.
    let encloses = |volume: Scalar, expected: f64| {
        assert!(
            volume.could_be_equal(Scalar::from_f64(expected)),
            "{volume:?} vs {expected}"
        )
    };
    encloses(volume(&part, "fillet(round)"), outer);
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
    let inner = 90.0 * 50.0 * 45.0 - 4.0 * corner * 25.0 * 45.0;
    encloses(volume(&part, "shell(shell)"), outer - inner);
}
