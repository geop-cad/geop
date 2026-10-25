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

/// The same prism as [`fillet_of_an_upright_edge_between_faces_at_169_degrees`]
/// with the back rest leaning 7 over 20 (the faces at 160.7°). The fillet
/// fails classifying a face of the blend tool:
///
/// > face_interior_point: no point strictly inside face FaceId(58) was
/// > found, stepping inward from the midpoint of each of its 3 outer
/// > coedges; that loop spans u=[-2e-9, 0.0012], v=[0.662, 0.671] (a loop
/// > spanning nothing encloses no area, so the face is degenerate)
///
/// The face is a piece of the prism's leaning side cut off by the tool: at
/// this angle the tool's contact lines on the two faces are far apart
/// along the faces but the piece between the tool and the top is a thin
/// triangle. Ruled out: the radius (0.5 fails the same way). At 150° and
/// 155° the fillet builds.
#[test]
#[ignore = "bug: the fillet of an edge whose faces meet at 160° fails classifying a thin triangular piece of the leaning face (face_interior_point finds no interior point); 155° builds"]
fn fillet_of_an_upright_edge_between_faces_at_160_degrees() {
    assert_builds_valid(&prism_with_rounded_upright(
        &[
            [0.0, 0.0],
            [70.0, 0.0],
            [40.0, 60.0],
            [7.0, 60.0],
            [0.0, 40.0],
        ],
        [0.0, 40.0],
        1.5,
    ));
}

/// A drawer handle (Fusion 360 tutorial "Drawer pull": two round posts, a
/// bar across their tops, the joints rounded): a post of radius 3 standing
/// under a bar 8 wide, the circle where they meet rounded by 1.5. The
/// fillet's contact on the bar's underside is a circle of radius 4.5, which
/// runs off the bar (half-width 4) on both sides.
///
/// The bracket case of the same shape is refused up front ("the blend is
/// too large for face ...: it would run past the face's boundary"); this
/// one is not, and the fillet builds an invalid solid:
///
/// > coedge 247's pcurve sample 1 (t=0.244) maps through its face's
/// > surface to a point that is not on edge 243's curve at all (point
/// > (-1.83, 4.11, 20.0))
///
/// at `y = 4.11`, outside the bar, on the edges between the blend faces
/// `fillet(round,cylinder(post,c0,end),fillet,q3)` and the bar's sides.
/// With a radius of 1 (contact exactly on the bar's edge) or 0.5 the
/// fillet builds valid. Either a refusal naming the bar's side, or a blend
/// that runs out through the bar's side, is expected; not an invalid model.
#[test]
#[ignore = "bug: a fillet whose contact circle runs past the face it lies on (post r=3 under a bar 8 wide, r=1.5) builds an invalid solid instead of refusing; r=1 and r=0.5 build valid"]
fn fillet_of_a_post_joint_whose_contact_runs_off_the_bar() {
    let mut program = Program::new();
    program.push(
        "post",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 3.0, 20.0),
    );
    program.push(
        "bar",
        PrimitiveArgs::block([-10.0, -4.0, 20.0], [10.0, 4.0, 28.0]).join("cylinder(post)"),
    );
    let before = program.build_model(registry).unwrap();
    let arcs = edges_with_middle_where(&before, |m| {
        near(m[2], 20.0) && (m[0].powi(2) + m[1].powi(2)).sqrt() < 3.5
    });
    assert_eq!(arcs.len(), 4, "{arcs:?}");
    program.push("round", FilletArgs::constant(arcs, 1.5));
    assert_builds_valid(&program);
}

/// A thin plate with a hole (Fusion 360 "Bottle opener" and sheet-like
/// tutorials: a large plate, a hole, its rim rounded by a small radius): a
/// plate 100 x 100, a hole of radius 5 through it, the rim rounded by 0.04.
/// The fillet fails classifying the piece of the top cut off by the tool:
///
/// > face_interior_point: no point strictly inside face FaceId(351) was
/// > found, stepping inward from the midpoint of each of its 4 outer
/// > coedges; that loop spans u=[0.4496, 0.5504], v=[0.4496, 0.5504]
///
/// The loop is the fillet's contact circle (radius 5.04) on the plate's
/// top, 0.0504 of the surface's domain across; what it bounds is the
/// annulus between it and the hole's rim, 0.0004 wide in `(u, v)`. Ruled
/// out: the plate's thickness (1 and 0.1 fail alike) and the absolute size
/// (the drilled box's rim fillet scaled by 1/20 and by 1000 builds). A
/// plate 20 x 20 with the same hole and fillet builds: the failure comes
/// with the fillet's width relative to the face it cuts.
#[test]
#[ignore = "bug: a hole-rim fillet 0.04 wide on a 100 x 100 face fails in face_interior_point on the 0.0004-wide (in uv) annulus the tool cuts off; the same fillet on a 20 x 20 face builds"]
fn fillet_of_a_hole_rim_tiny_against_its_face() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [100.0, 100.0, 1.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([50.0, 50.0, -1.0], 5.0, 3.0).cut("block(plate)"),
    );
    let before = program.build_model(registry).unwrap();
    let rim = edges_with_middle_where(&before, |m| {
        near(m[2], 1.0) && (m[0] - 50.0).abs() < 5.01 && (m[1] - 50.0).abs() < 5.01
    });
    assert_eq!(rim.len(), 4, "{rim:?}");
    program.push("round", FilletArgs::constant(rim, 0.04));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The rim moved out to 5.04 on the top.
    let contact = part.topology().vertices.values().any(|v| {
        let (dx, dy) = (
            v.point[0].sub(Scalar::from_f64(50.0)),
            v.point[1].sub(Scalar::from_f64(50.0)),
        );
        v.point[2].could_be_equal(Scalar::from_f64(1.0))
            && dx
                .mul(dx)
                .add(dy.mul(dy))
                .could_be_equal(Scalar::from_f64(5.04 * 5.04))
    });
    assert!(contact);
}
