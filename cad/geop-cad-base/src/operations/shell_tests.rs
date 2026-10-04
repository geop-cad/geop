//! Shelling solids built by programs: opened at faces picked by name, or
//! closed all round.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_shell::ShellArgs;
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::examples::{self, n};
use crate::{PartOperation, Program};

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        panic!(
            "{} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:?}");
    }
}

fn z_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z))
}

/// A sketch on the Z plane, and `name` its extrude `height` up, a new body.
fn extruded(program: &mut Program, name: &str, sketch: Sketch, height: f64) {
    let sketch_name = format!("{name}_sketch");
    program.push(
        &sketch_name,
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch,
            ..Default::default()
        },
    );
    program.push(
        name,
        ExtrudeArgs {
            sketch: sketch_name,
            extent: Extents::blind(height),
            face: false,
            combine: Combine::NewBody,
        },
    );
}

fn polygon(corners: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

fn shell(solid: &str, faces: &[&str], thickness: f64) -> ShellArgs {
    ShellArgs {
        solid: solid.into(),
        faces: faces.iter().map(|f| f.to_string()).collect(),
        thickness: thickness.into(),
    }
}

fn face_count(part: &Part<S>, solid: &str) -> usize {
    let id = part.solid_id(solid).unwrap();
    part.topology().body_faces(Body::Solid(id)).unwrap().len()
}

/// The drilled box, opened at its top: the top's rim is a ring around the
/// walls, and another around the hole's, which the walls also line.
#[test]
fn drilled_box_opened_at_its_top() {
    let mut program = examples::box_with_drill_hole();
    program.push("s", shell("extrude(hole)", &["extrude(box,end)"], 0.1));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let rims: Vec<_> = part
        .names()
        .iter()
        .filter(|(_, name)| name.starts_with("shell(s,extrude(box,end)"))
        .map(|(_, name)| name.to_string())
        .collect();
    assert_eq!(rims.len(), 2, "{rims:?}");
    // The outside, its inner copy less the top, and the two rims.
    let before = examples::box_with_drill_hole()
        .build::<S>(&NoFiles)
        .unwrap();
    assert_eq!(
        face_count(&part, "shell(s)"),
        2 * face_count(&before, "extrude(hole)") - 1 + 1
    );
}

/// A round cylinder, its circle extruded, opened at its top: a cup.
#[test]
fn cup() {
    let mut program = Program::new();
    let mut s = Sketch::new();
    let c = s.add_point(n(0.0), n(0.0));
    s.add_circle(c, n(1.0));
    extruded(&mut program, "can", s, 2.0);
    program.push("s", shell("extrude(can)", &["extrude(can,end)"], 0.1));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let rim = part.face_id("shell(s,extrude(can,end))").unwrap();
    assert_eq!(part.topology().get_face(rim).unwrap().holes.len(), 1);
}

/// An L-shaped block opened at its top: the walls meet at an inward corner,
/// where their inner sides reach past the faces they are copies of.
#[test]
fn l_block_opened_at_its_top() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "l",
        polygon(&[[0., 0.], [3., 0.], [3., 1.], [1., 1.], [1., 2.], [0., 2.]]),
        1.0,
    );
    program.push("s", shell("extrude(l)", &["extrude(l,end)"], 0.2));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    // 7 outer faces and their copies, and the rim.
    assert_eq!(face_count(&part, "shell(s)"), 7 + 7 + 1);
}

/// A plate with a slot-shaped outline — lines and half circles — and a
/// round hole, closed all round: a void inside, the hole lined.
#[test]
fn slot_plate_closed_all_round() {
    let mut s = Sketch::new();
    let p = [
        s.add_point(n(0.0), n(0.0)),
        s.add_point(n(2.0), n(0.0)),
        s.add_point(n(2.0), n(1.0)),
        s.add_point(n(0.0), n(1.0)),
    ];
    s.add_line(p[0], p[1]);
    s.add_arc(p[1], p[2], n(PI));
    s.add_line(p[2], p[3]);
    s.add_arc(p[3], p[0], n(PI));
    let c = s.add_point(n(1.0), n(0.5));
    s.add_circle(c, n(0.25));
    let mut program = Program::new();
    extruded(&mut program, "plate", s, 0.5);
    program.push("s", shell("extrude(plate)", &[], 0.05));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("shell(s)").unwrap();
    assert_eq!(part.topology().get_solid(solid).unwrap().shells.len(), 2);
}

/// A box opened at its top and two opposite sides: a channel, its top rim
/// in two strips.
#[test]
fn channel() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "b",
        polygon(&[[0., 0.], [2., 0.], [2., 1.], [0., 1.]]),
        1.0,
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    // The two short sides: those whose faces are normal to x.
    let ends: Vec<String> = part
        .topology()
        .body_faces(Body::Solid(part.solid_id("extrude(b)").unwrap()))
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
            plane.is_some_and(|p| p.normal[0].abs().could_be_equal(S::ONE))
        })
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    assert_eq!(ends.len(), 2);
    program.push(
        "s",
        shell("extrude(b)", &["extrude(b,end)", &ends[0], &ends[1]], 0.1),
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    for k in 0..2 {
        part.face_id(&format!("shell(s,extrude(b,end),{k})"))
            .unwrap();
    }
    // Bottom and long sides, inside and out; two strips on top; a U at
    // either end.
    assert_eq!(face_count(&part, "shell(s)"), 3 + 3 + 2 + 2);
}

/// A reference plane offset from a face of a shelled box: the inner floor,
/// lifted.
#[test]
fn offset_plane_from_a_shelled_face() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "b",
        polygon(&[[0., 0.], [2., 0.], [2., 2.], [0., 2.]]),
        1.0,
    );
    program.push("s", shell("extrude(b)", &["extrude(b,end)"], 0.25));
    program.push(
        "d",
        AddDatumArgs {
            selection: vec![EntityRef::Face {
                name: "shell(s,extrude(b,start))".into(),
            }],
            construction: Construction::Offset { distance: 0.5.into() },
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    let datum = part.datum(part.datum_id("d").unwrap()).unwrap();
    // The inner floor faces up, into the cup, at z = 0.25.
    let z = datum.frame.origin()[2].to_f64();
    assert!((z - 0.75).abs() < 1e-9, "{z}");
    assert!(datum.frame.w()[2].to_f64() > 0.99);
}

/// A shelled box is a solid like any other: a hole cut through two of its
/// walls at once, sketched on a reference plane offset into the middle of
/// it.
#[test]
fn hole_through_the_walls_of_a_shelled_box() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "b",
        polygon(&[[0., 0.], [2., 0.], [2., 2.], [0., 2.]]),
        1.0,
    );
    program.push("s", shell("extrude(b)", &["extrude(b,end)"], 0.2));
    program.push(
        "middle",
        AddDatumArgs {
            selection: vec![EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )],
            construction: Construction::Offset { distance: 1.0.into() },
        },
    );
    let mut s = Sketch::new();
    // On the plane normal to y, a sketch's x runs along x and its y along
    // -z: the circle is around x = 0.5, z = 0.5.
    let c = s.add_point(n(0.5), n(-0.5));
    s.add_circle(c, n(0.2));
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum("middle")),
            sketch: s,
            ..Default::default()
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents {
                symmetric: true,
                ..Extents::blind(3.0)
            },
            face: false,
            combine: Combine::Difference {
                target: "shell(s)".into(),
            },
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    // The cup's 11 faces, and the hole's wall through each of the two.
    assert!(face_count(&part, "extrude(hole)") > 11);
}

/// A shell step is written as the others are.
#[test]
fn shell_steps_round_trip_through_json() {
    let step = PartOperation::Shell(shell("extrude(b)", &["extrude(b,end)"], 0.25));
    let json = serde_json::to_value(&step).unwrap();
    assert_eq!(json["operation"], "shell");
    assert_eq!(json["args"]["thickness"], 0.25);
    let back: PartOperation = serde_json::from_value(json).unwrap();
    assert_eq!(back, step);
}

/// Walls need a thickness, and the faces opened must be the solid's.
#[test]
fn shells_refuse_what_they_cannot_build() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "a",
        polygon(&[[0., 0.], [1., 0.], [1., 1.], [0., 1.]]),
        1.0,
    );
    extruded(
        &mut program,
        "b",
        polygon(&[[3., 0.], [4., 0.], [4., 1.], [3., 1.]]),
        1.0,
    );
    let mut zero = program.clone();
    zero.push("s", shell("extrude(a)", &[], 0.0));
    assert!(zero.build::<S>(&NoFiles).is_err());
    let mut other = program.clone();
    other.push("s", shell("extrude(a)", &["extrude(b,end)"], 0.1));
    assert!(other.build::<S>(&NoFiles).is_err());
}
