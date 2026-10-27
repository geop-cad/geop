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
    testing::{Built, apply_to_model, assert_valid, edges_with_middle_where, face_towards},
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
/// The boolean once spliced the exact arc where a plane crosses a surface
/// of revolution around its normal; it no longer special-cases surface
/// kinds, so the shell has to offset the general (fitted) edge.
///
/// Shelling a filleted body is the first thing every enclosure tutorial
/// does. The shell now offsets every edge by the general rebuild, which
/// traces it where the offsets of its faces meet.
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
            other_faces: Vec::new(),
            other_thickness: 0.2.into(),
            outward: false,
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
                other_faces: Vec::new(),
                other_thickness: 0.2.into(),
                outward: false,
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
    // The rounded block is built once: its corners' blends take a while.
    let args = ShellArgs {
        solid: solid("fillet(round)"),
        faces: vec![],
        thickness: 0.5.into(),
        other_faces: Vec::new(),
        other_thickness: 0.2.into(),
        outward: false,
    };
    let part = apply_to_model::<Shell>(rounded, "s", &args).unwrap();
    assert_valid(&part);
    let id = part.solid_id("shell(s)").unwrap();
    let volume = part.topology().mass_properties(id).unwrap().volume;
    let expected = outer.sub(Scalar::from_f64(9.0 * 5.0 * 2.0));
    assert!(
        volume.could_be_equal(expected),
        "{volume:?} vs {expected:?}"
    );
}

/// An enclosure's lid side up: a 10 x 6 x 3 block with its four upright
/// edges and its four top edges rounded by 1, shelled 0.3 thick, open at
/// the top. The top is tangent to every blend around it, so the walls do
/// not stand on it: they end where it was, along its edges, each in an end
/// face from the edge down to its copy on the blend's inner copy — as a
/// sheet's free edge gets a wall. Nothing is left of the top.
#[test]
#[ignore = "slow: 18 s, most of it rounding the eight edges — run with `cargo test -- --ignored`"]
fn shell_opened_at_a_top_its_blends_are_tangent_to() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 6.0, 3.0]),
    );
    let before = program.build_model(registry).unwrap();
    let edges = edges_with_middle_where(&before, |m| near(m[2], 3.0) || near(m[2], 1.5));
    assert_eq!(edges.len(), 8, "{edges:?}");
    program.push("round", FilletArgs::constant(edges, 1.0));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let top = face_name(&rounded, "fillet(round)", [0.0, 0.0, 1.0]);
    let args = ShellArgs {
        solid: solid("fillet(round)"),
        faces: vec![ElementReference::new(model(), top.clone())],
        thickness: 0.3.into(),
        other_faces: Vec::new(),
        other_thickness: 0.2.into(),
        outward: false,
    };
    let part = apply_to_model::<Shell>(rounded, "s", &args).unwrap();
    assert_valid(&part);
    assert!(part.face_id(&format!("shell(s,{top})")).is_err());
}

/// A 10 x 6 x 3 block with its upright edge at the origin rounded by 1,
/// shelled exactly 1 thick, open at the top: the walls as thick as the
/// blend's radius (an enclosure tutorial with 2 mm walls and 2 mm
/// rounds). Thinner walls ([`shell_of_a_block_with_a_rounded_upright_edge`])
/// and thicker ones ([`shell_thicker_than_a_rounded_edge`]) both build;
/// at exactly the radius the blend's inner copy is a line, its two sides
/// meeting in a sharp inner corner as they do for thicker walls.
///
/// The shell failed offsetting the blend: "the offset by -1 could fold
/// over itself ... a factor 1 − d κ could be zero there". So it is, all
/// over: no box tells that factor from zero, and none is regular, so the
/// offset is no surface, as for thicker walls (`fitted_offset` now says
/// so, and refuses a fold only where some box is regular). The fast test
/// of that is `cylinders_change_radius` in `geop-core-geometry`; here the
/// rolled blend only approximates its cylinder, the factor wanders round
/// zero by its approximation, and every box is split to the last before
/// it is decided: 96 s.
#[test]
#[ignore = "slow: 96 s, every box of the blend's offset split to the last — run with `cargo test -- --ignored`"]
fn shell_as_thick_as_a_rounded_edge() {
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
    let top = face_name(&rounded, "fillet(corner)", [0.0, 0.0, 1.0]);
    let args = ShellArgs {
        solid: solid("fillet(corner)"),
        faces: vec![ElementReference::new(model(), top)],
        thickness: 1.0.into(),
        other_faces: Vec::new(),
        other_thickness: 0.2.into(),
        outward: false,
    };
    let part = apply_to_model::<Shell>(rounded, "s", &args).unwrap();
    assert_valid(&part);
}

/// A plate with a round boss, the boss's root rounded (the SolidWorks
/// "boss with fillet" tutorial), hollowed: a 60 x 40 x 5 plate with a boss
/// of radius 8 standing 20 high from its middle, the circle where they
/// meet rounded by 2, shelled 1 thick, closed all round. The plate and
/// boss alone shell; rounded, the shell fails for every thickness tried
/// (0.5, 1, 1.5, and open at the bottom or the boss's top alike):
///
/// > the bound on what the fit misses was not decided within 16384 boxes
///
/// at `fitted_offset(distance=-1)`, offsetting the blend
/// `fillet(r,#..,fillet,s0)`, a cubic surface of 73 x 13 control points
/// rolled round the boss.
///
/// Measured: the 16384 boxes are for all the surface's spans together,
/// and the blend has 70 x 10 of them, each needing several boxes before
/// its bound is under the drift. With no cap the bound is found, after
/// 17.5 minutes, and the shell then fails further on: "the seam, carried
/// on, does not pass through its moved end", between `block(plate,end)`
/// and the blend, pieces of one smooth surface, the moved end `5e-6`
/// wide. What is missing is a cheaper bound per box over a surface of
/// that many spans (each box costs milliseconds of polynomial products),
/// and then the seam's end between the plate and its tangent blend.
#[test]
#[ignore = "bug: shelling a plate with a boss whose root is rounded: the fit's bound over the rolled blend's 700 spans exceeds the 16384-box budget (17.5 min without it), and the seam between the plate's top and the blend then misses its moved end, 5e-6 wide"]
fn shell_of_a_plate_with_a_rounded_boss() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([-30.0, -20.0, 0.0], [30.0, 20.0, 5.0]),
    );
    program.push(
        "boss",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 8.0, 20.0).join("block(plate)"),
    );
    let before = program.build_model(registry).unwrap();
    let root = edges_with_middle_where(&before, |m| {
        near(m[2], 5.0) && (m[0] * m[0] + m[1] * m[1]).sqrt() < 9.0
    });
    assert!(!root.is_empty(), "no root edges");
    program.push("r", FilletArgs::constant(root, 2.0));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let args = ShellArgs {
        solid: solid("fillet(r)"),
        faces: Vec::new(),
        thickness: 1.0.into(),
        other_faces: Vec::new(),
        other_thickness: 0.2.into(),
        outward: false,
    };
    let part = apply_to_model::<Shell>(rounded, "s", &args).unwrap();
    assert_valid(&part);
}

/// The enclosure of [`shell_of_a_block_with_a_rounded_upright_edge`] in a
/// large assembly's coordinates: a 40 x 20 x 10 block with its corner at
/// `(100000, -100000, 0)`, the upright edge there rounded by 5, shelled 2
/// thick open at its top. The rounded block is valid there, and so is the
/// block shelled without the round, and both at the origin. Shelled, it
/// was invalid, 1180 errors, the vertices where the blend's inner copy
/// meets the side's `0.00033` wide. The blend's tangent line `ta` is a
/// seam of one smooth surface, rebuilt on the side's new surface along its
/// old pcurve carried on for room. That pcurve was fitted in twenty spans,
/// `7e-11` wide here (`1.3e-12` at the origin), and carried on by its
/// whole domain it extrapolated its end span twenty times its length. The
/// room is now only as much as keeps the seam's ends within the drift its
/// fit is held to.
#[test]
fn shell_of_a_rounded_block_far_from_the_origin() {
    let corner = [1e5, -1e5, 0.0];
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block(corner, [corner[0] + 40.0, corner[1] + 20.0, 10.0]),
    );
    let before = program.build_model(registry).unwrap();
    let edge = edges_with_middle_where(&before, |m| {
        near(m[0], corner[0]) && near(m[1], corner[1]) && near(m[2], 5.0)
    });
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("corner", FilletArgs::constant(edge, 5.0));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let top = face_name(&rounded, "fillet(corner)", [0.0, 0.0, 1.0]);
    let args = ShellArgs {
        solid: solid("fillet(corner)"),
        faces: vec![ElementReference::new(model(), top)],
        thickness: 2.0.into(),
        other_faces: Vec::new(),
        other_thickness: 0.2.into(),
        outward: false,
    };
    let part = apply_to_model::<Shell>(rounded, "s", &args).unwrap();
    assert_valid(&part);
}
