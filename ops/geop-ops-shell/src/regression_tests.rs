//! Programs from bug reports, rebuilt as the smallest programs found that
//! still fail.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_core_topology::Body;
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::{
    NamedModel, model, solid,
    testing::{Built, assert_valid, edges_with_middle_where, face_towards},
};

use crate::{Shell, ShellArgs};

/// The operations these programs are written in: given solids, rounded,
/// then shelled.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Fillet)?.with(Shell)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The name of the planar face of `solid` facing `normal`.
fn face_name(part: &NamedModel, solid: &str, normal: [f64; 3]) -> String {
    let id = part.solid_id(solid).unwrap();
    let face = face_towards(part, id, normal);
    part.name_of(face).unwrap().to_string()
}

/// An enclosure (Fusion 360 "Electronics enclosure" tutorial: a box, its
/// vertical edges rounded, shelled open at the top): a 10 x 6 x 3 block
/// with one upright edge rounded by 1, then shelled 0.3 thick open at its
/// top. The shell refuses the edge where the blend meets the block's
/// bottom:
///
/// > the edge is neither straight nor circular: offsetting it is not
/// > supported — the inner copy of edge
/// > fillet(corner,block(box,p0),block(box,end),...)
///
/// That edge is where a cylinder (the blend, around a vertical axis) meets
/// a horizontal plane: a quarter circle. The fillet's boolean traced it,
/// and fitted a curve through its march instead of recognising the arc, so
/// the shell could not offset it. Ruled out: the open face (opening a side
/// instead of the top, with a top edge rounded, failed on the same kind of
/// edge), the number of fillets (all four uprights failed alike), the size
/// (the 100 x 60 x 30 enclosure with radius 5 and walls 2 failed alike).
/// The boolean now splices the exact arc where a plane crosses a surface
/// of revolution around its normal (`geop_core_geometry::shape::circle_through`).
///
/// Shelling a filleted body is the first thing every enclosure tutorial
/// does.
#[test]
fn shell_of_a_block_with_a_rounded_upright_edge() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 6.0, 3.0]),
    );
    let before = program.build_model(registry).unwrap();
    let edge = edges_with_middle_where(&before, |m| {
        near(m[0], 0.0) && near(m[1], 0.0) && near(m[2], 1.5)
    });
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("corner", FilletArgs::constant(edge, 1.0));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let top = face_name(&rounded, "fillet(corner)", [0.0, 0.0, 1.0]);
    program.push(
        "s",
        ShellArgs {
            solid: solid("fillet(corner)"),
            faces: vec![ElementReference::new(model(), top.clone())],
            thickness: 0.3.into(),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The outside's faces less the top, their inner copies, and the rim: a
    // ring around the walls, the blend's quarter among them.
    let id = part.solid_id("shell(s)").unwrap();
    let outside = rounded.solid_id("fillet(corner)").unwrap();
    let faces = |part: &NamedModel, id| part.topology().body_faces(Body::Solid(id)).unwrap().len();
    assert_eq!(faces(&part, id), 2 * (faces(&rounded, outside) - 1) + 1);
    let rim = part.face_id(&format!("shell(s,{top})")).unwrap();
    assert_eq!(part.topology().get_face(rim).unwrap().holes.len(), 1);
}

/// A 10 x 6 x 3 block with its upright edge at the origin rounded by 0.5,
/// shelled 1 thick: the blend is narrower than the walls are thick, so its
/// inner copy has no area — offset by 1, its cylinder of radius 0.5 would
/// pass its axis. The inner side has no copy of it: the two sides it
/// rounds meet in a sharp inner corner, a line from the bottom's inner
/// copy to the top's. Closed all round, the void is the block's inner
/// box, 8 x 4 x 1; opened at the top, the walls are the block less the box
/// from 1 up.
#[test]
fn shell_thicker_than_a_rounded_edge() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 6.0, 3.0]),
    );
    let before = program.build_model(registry).unwrap();
    let edge = edges_with_middle_where(&before, |m| {
        near(m[0], 0.0) && near(m[1], 0.0) && near(m[2], 1.5)
    });
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("corner", FilletArgs::constant(edge, 0.5));
    let rounded = program.build_model(registry).unwrap();
    let top = face_name(&rounded, "fillet(corner)", [0.0, 0.0, 1.0]);
    let outer = 180.0 - (1.0 - std::f64::consts::PI / 4.0) * 0.25 * 3.0;
    for (open, void) in [(vec![], 8.0 * 4.0 * 1.0), (vec![top], 8.0 * 4.0 * 2.0)] {
        let mut program = program.clone();
        program.push(
            "s",
            ShellArgs {
                solid: solid("fillet(corner)"),
                faces: open
                    .into_iter()
                    .map(|f| ElementReference::new(model(), f))
                    .collect(),
                thickness: 1.0.into(),
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let id = part.solid_id("shell(s)").unwrap();
        let volume = part.topology().mass_properties(id).unwrap().volume;
        assert!(
            volume.could_be_equal(Scalar::from_f64(outer - void)),
            "{volume:?} vs {}",
            outer - void
        );
    }
}

/// A 10 x 6 x 3 block with every edge rounded by 0.3, shelled 0.5 thick:
/// every blend is narrower than the walls, the corners' patches too. The
/// inner side is the sharp box 9 x 5 x 2: each blend's two edges along it
/// share one copy, where the faces it rounds meet, and each corner patch
/// leaves one vertex where the three walls' copies meet.
#[test]
fn shell_thicker_than_every_rounded_edge() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 6.0, 3.0]),
    );
    let before = program.build_model(registry).unwrap();
    let all = edges_with_middle_where(&before, |_| true);
    assert_eq!(all.len(), 12, "{all:?}");
    program.push("round", FilletArgs::constant(all, 0.3));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let outer = rounded
        .topology()
        .mass_properties(rounded.solid_id("fillet(round)").unwrap())
        .unwrap()
        .volume;
    program.push(
        "s",
        ShellArgs {
            solid: solid("fillet(round)"),
            faces: vec![],
            thickness: 0.5.into(),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let id = part.solid_id("shell(s)").unwrap();
    let volume = part.topology().mass_properties(id).unwrap().volume;
    let expected = outer.sub(Scalar::from_f64(9.0 * 5.0 * 2.0));
    assert!(
        volume.could_be_equal(expected),
        "{volume:?} vs {expected:?}"
    );
}
