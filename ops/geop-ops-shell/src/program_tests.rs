//! Shelling solids built by programs: opened at faces picked by name, or
//! closed all round.

use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    testing::args,
};

use crate::{Shell, ShellArgs};
use geop_core_math::{
    geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar, vector::Vector3,
};
use geop_core_topology::Body;
use geop_lib_fixtures::{PrimitiveArgs, drilled_box};
use geop_ops_model::{
    NamedModel, model, solid,
    testing::{Built, assert_valid},
};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Shell)
}

/// A block of 2 x 2 x 1 on the ground, as the step `b`: the solid
/// `block(b)`, its top `block(b,end)`.
fn block(program: &mut Program) {
    program.push("b", PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]));
}

fn shell(name: &str, faces: &[&str], thickness: f64) -> ShellArgs {
    ShellArgs {
        solid: solid(name),
        faces: (faces.iter())
            .map(|f| ElementReference::new(model(), *f))
            .collect(),
        thickness: thickness.into(),
    }
}

fn face_count(part: &NamedModel, solid: &str) -> usize {
    let id = part.solid_id(solid).unwrap();
    part.topology().body_faces(Body::Solid(id)).unwrap().len()
}

/// The drilled box, opened at its top: the top's rim is a ring around the
/// walls, and another around the hole's, which the walls also line.
#[test]
fn drilled_box_opened_at_its_top() {
    let mut program = drilled_box();
    program.push("s", shell("cylinder(hole)", &["block(box,end)"], 0.1));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let rims: Vec<_> = part
        .names()
        .iter()
        .filter(|(_, name)| name.starts_with("shell(s,block(box,end)"))
        .map(|(_, name)| name.to_string())
        .collect();
    assert_eq!(rims.len(), 2, "{rims:?}");
    // The outside, its inner copy less the top, and the two rims.
    let before = drilled_box().build_model(registry).unwrap();
    assert_eq!(
        face_count(&part, "shell(s)"),
        2 * face_count(&before, "cylinder(hole)") - 1 + 1
    );
}

/// A round cylinder opened at its top: a cup.
#[test]
fn cup() {
    let mut program = Program::new();
    program.push("can", PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 1.0, 2.0));
    program.push("s", shell("cylinder(can)", &["cylinder(can,end)"], 0.1));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let rim = part.face_id("shell(s,cylinder(can,end))").unwrap();
    assert_eq!(part.topology().get_face(rim).unwrap().holes.len(), 1);
}

/// An L-shaped block opened at its top: the walls meet at an inward corner,
/// where their inner sides reach past the faces they are copies of.
#[test]
fn l_block_opened_at_its_top() {
    let mut program = Program::new();
    program.push(
        "l",
        PrimitiveArgs::prism(
            &[[0., 0.], [3., 0.], [3., 1.], [1., 1.], [1., 2.], [0., 2.]],
            0.0,
            1.0,
        ),
    );
    program.push("s", shell("prism(l)", &["prism(l,end)"], 0.2));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // 7 outer faces and their copies, and the rim.
    assert_eq!(face_count(&part, "shell(s)"), 7 + 7 + 1);
}

/// A plate with a round hole through it, closed all round: a void inside,
/// the hole lined.
#[test]
fn drilled_plate_closed_all_round() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 1.0, 0.5]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 0.5, 0.0], 0.25, 0.5).cut("block(plate)"),
    );
    program.push("s", shell("cylinder(hole)", &[], 0.05));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("shell(s)").unwrap();
    assert_eq!(part.topology().get_solid(solid).unwrap().shells.len(), 2);
}

/// A box opened at its top and two opposite sides: a channel, its top rim
/// in two strips.
#[test]
fn channel() {
    let mut program = Program::new();
    program.push("b", PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]));
    let part = program.build_model(registry).unwrap();
    // The two short sides: those whose faces are normal to x.
    let ends: Vec<String> = part
        .topology()
        .body_faces(Body::Solid(part.solid_id("block(b)").unwrap()))
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let plane = part
                .topology()
                .get_face(f)
                .unwrap()
                .surface
                .as_plane()
                .unwrap();
            plane.is_some_and(|p| p.normal[0].abs().could_be_equal(Scalar::ONE))
        })
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    assert_eq!(ends.len(), 2);
    program.push(
        "s",
        shell("block(b)", &["block(b,end)", &ends[0], &ends[1]], 0.1),
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    for k in 0..2 {
        part.face_id(&format!("shell(s,block(b,end),{k})")).unwrap();
    }
    // Bottom and long sides, inside and out; two strips on top; a U at
    // either end.
    assert_eq!(face_count(&part, "shell(s)"), 3 + 3 + 2 + 2);
}

/// The inner copy of a shelled box's floor is a face like any other, named
/// after the floor: a plane, lifted by the wall's thickness and facing up,
/// into the cup.
#[test]
fn inner_floor_of_a_shelled_box() {
    let mut program = Program::new();
    block(&mut program);
    program.push("s", shell("block(b)", &["block(b,end)"], 0.25));
    let part = program.build_model(registry).unwrap();
    let floor = part.face_id("shell(s,block(b,start))").unwrap();
    let plane = (part.topology().get_face(floor).unwrap().surface)
        .as_plane()
        .unwrap()
        .expect("the inner floor is a plane");
    assert!(
        plane.point[2].could_be_equal(Scalar::from_f64(0.25)),
        "{:?}",
        plane.point
    );
    let up = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]);
    assert!(plane.normal.could_be_equal(&up), "{:?}", plane.normal);
}

/// A shelled box is a solid like any other: a hole cut through two of its
/// walls at once, along the middle of it.
#[test]
fn hole_through_the_walls_of_a_shelled_box() {
    let mut program = Program::new();
    block(&mut program);
    program.push("s", shell("block(b)", &["block(b,end)"], 0.2));
    program.push(
        "hole",
        PrimitiveArgs::cylinder([0.5, -0.5, 0.5], FrameAxis::Y, 0.2, 3.0).cut("shell(s)"),
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The cup's 11 faces, and the hole's wall through each of the two.
    assert!(face_count(&part, "cylinder(hole)") > 11);
}

/// A shell step is written as the others are.
#[test]
fn shell_steps_round_trip_through_json() {
    let shell_args = shell("block(b)", &["block(b,end)"], 0.25);
    let mut program = Program::new();
    program.push("s", shell_args.clone());
    let json = serde_json::to_value(&program.steps[0]).unwrap();
    assert_eq!(json["operation"], "shell");
    assert_eq!(json["args"]["thickness"], 0.25);
    // The solid and the faces are saved by their names.
    assert_eq!(json["args"]["solid"], "block(b)");
    assert_eq!(json["args"]["faces"][0], "block(b,end)");
    assert_eq!(args::<ShellArgs>(&program.steps[0]), shell_args);
}

/// Walls need a thickness, and the faces opened must be the solid's.
#[test]
fn shells_refuse_what_they_cannot_build() {
    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
    program.push("b", PrimitiveArgs::block([3.0, 0.0, 0.0], [4.0, 1.0, 1.0]));
    let mut zero = program.clone();
    zero.push("s", shell("block(a)", &[], 0.0));
    assert!(zero.build_model(registry).is_err());
    let mut other = program.clone();
    other.push("s", shell("block(a)", &["block(b,end)"], 0.1));
    assert!(other.build_model(registry).is_err());
}
