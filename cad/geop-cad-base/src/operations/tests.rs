//! The operations run as programs: sketches extruded and revolved into
//! solids, and what they refuse.

use geop_core_program::{
    operation::StepArgs,
    program::{Program, ProgramStep},
};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output, RevolveArgs, RevolveAxis};
use std::f64::consts::PI;

use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_topology::validation::{ValidationParameters, validate};
use geop_ops_booleans::{BooleanArgs, Combine, SplitArgs, solid};
use geop_ops_model::{BooleanOp, NamedModel, element};
use geop_ops_sketch::{Constraint, Sketch, sketch_element, whole_sketch};

use crate::{
    examples::{n, polygon},
    operations::testing::{
        Built, assert_valid, both_ways, extrusion, on_face, on_origin, one_way, origin_axis,
        sketch, two_ways,
    },
};

fn extrude(sketch: &str, distance: f64, symmetric: bool) -> ExtrudeArgs {
    extrusion(
        sketch,
        Extents {
            side1: Extent::blind(distance),
            symmetric,
            side2: None,
            reversed: false,
        },
        Combine::NewBody,
    )
}

/// A symmetric extrude of a unit square on the Z plane by total thickness
/// 1 spans z in [-0.5, 0.5] — the sketch plane through the middle of the
/// solid, not at one face of it.
#[test]
fn symmetric_extrude_is_centered_on_the_sketch_plane() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", sketch(on_origin(FrameAxis::Z), s));
    program.push("slab", extrude("square", 1.0, true));
    let part = program.build_model().unwrap();
    assert_valid(&part);
    for v in part.topology().vertices.values() {
        assert!(
            v.point[2].could_be_equal(S::from_f64(0.5))
                || v.point[2].could_be_equal(S::from_f64(-0.5)),
            "vertex at unexpected height: {:?}",
            v.point
        );
    }
}

/// A plate with a round hole and a slot-shaped outline (lines + half
/// circles), extruded upwards from the XY plane.
#[test]
fn extrude_slot_plate_with_hole() {
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
    program.push("slot", sketch(on_origin(FrameAxis::Z), s));
    program.push("plate", extrude("slot", 0.5, false));
    let part = program.build_model().unwrap();
    assert_valid(&part);
    // 2 caps + 6 outer walls (the half circles are split in quarters) + 4
    // hole walls.
    assert_eq!(part.topology().faces.len(), 12);
}

/// Sketch a circle on a block's top face, extrude it upwards, and unite:
/// a boss on a block.
#[test]
fn boss_on_block_top_face() {
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    );
    let mut boss = Sketch::new();
    let c = boss.add_point(n(0.5), n(0.5));
    boss.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("base", sketch(on_origin(FrameAxis::Z), block));
    program.push("block", extrude("base", 1.0, false));
    program.push("boss_sketch", sketch(on_face("extrude(block,end)"), boss));
    program.push("boss", extrude("boss_sketch", 0.4, false));
    program.push(
        "join",
        BooleanArgs {
            a: solid("extrude(block)"),
            b: solid("extrude(boss)"),
            op: BooleanOp::Union,
        },
    );
    let part = program.build_model().unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1, "the boss joined the block");
    // The boss sits on top of the block, not below the sketch plane.
    assert!(
        part.topology()
            .vertices
            .values()
            .any(|v| v.point[2].could_be_equal(S::from_f64(1.4)))
    );
}

/// A half disc whose diameter is the axis itself: revolving gives a sphere,
/// from an arc profile.
#[test]
fn revolve_half_disc_is_sphere() {
    let mut s = Sketch::new();
    let top = s.add_point(n(0.0), n(1.0));
    let bottom = s.add_point(n(0.0), n(-1.0));
    let axis = s.add_line(bottom, top);
    s.add_arc(top, bottom, n(PI));
    let mut program = Program::new();
    program.push("half_disc", sketch(on_origin(FrameAxis::X), s));
    program.push(
        "ball",
        RevolveArgs {
            sketch: whole_sketch("half_disc"),
            axis: Some(RevolveAxis::Sketch(sketch_element("half_disc", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let part = program.build_model().unwrap();
    assert_valid(&part);
    // The half circle is split into two quarters.
    assert_eq!(part.topology().faces.len(), 8);
}

/// A profile straddling the axis is rejected rather than revolved into a
/// self-intersecting solid.
#[test]
fn revolve_across_axis_fails() {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    polygon(&mut s, &[[-0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [-0.5, 1.0]]);
    let mut program = Program::new();
    program.push("profile", sketch(on_origin(FrameAxis::Z), s));
    program.push(
        "bad",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Sketch(sketch_element("profile", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    assert!(program.build_model().is_err());
}

/// Revolves `s`, sketched on the origin's zx plane, around `axis`.
fn revolved_on_zx(
    s: Sketch,
    axis: RevolveAxis,
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let mut program = Program::new();
    program.push("profile", sketch(on_origin(FrameAxis::Y), s));
    program.push(
        "ring",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(axis),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.build_model()
}

/// A square clear of an axis from outside the sketch — the origin's z axis,
/// which lies in the sketch's plane — revolves into a ring with a square
/// cross section.
#[test]
fn revolve_square_around_an_outside_axis_is_a_ring() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let part = revolved_on_zx(s, z).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
}

/// A circle beside a construction line of its own sketch, not touching it,
/// revolves into a torus.
#[test]
fn revolve_circle_clear_of_its_axis_is_a_torus() {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(1.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let center = s.add_point(n(3.0), n(0.0));
    s.add_circle(center, n(1.0));
    let part = revolved_on_zx(s, RevolveAxis::Sketch(sketch_element("profile", axis))).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
}

/// A profile touching an axis from outside the sketch is refused: nothing
/// says which of its edges lie on that axis.
#[test]
fn revolve_touching_an_outside_axis_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let Err(error) = revolved_on_zx(s, z) else {
        panic!("revolved");
    };
    assert!(error.root_message().contains("touches"), "{error:?}");
}

/// An axis out of the sketch's plane is refused.
#[test]
fn revolve_around_an_axis_off_the_plane_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]);
    let y = RevolveAxis::Datum(origin_axis(FrameAxis::Y));
    let Err(error) = revolved_on_zx(s, y) else {
        panic!("revolved");
    };
    assert!(error.root_message().contains("plane"), "{error:?}");
}

/// Extrude refers to a sketch by name; naming a solid instead is an error.
#[test]
fn extrude_of_non_sketch_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", sketch(on_origin(FrameAxis::Z), s));
    program.push("slab", extrude("square", 1.0, false));
    program.push("again", extrude("extrude(slab)", 1.0, false));
    assert!(program.build_model().is_err());
}

/// The boss again, joined to the block by its own extrude: the result is
/// named after the extrude, and the block and the tool are gone.
#[test]
fn extrude_combines_with_its_target() {
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    );
    let mut boss = Sketch::new();
    let c = boss.add_point(n(0.5), n(0.5));
    boss.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("base", sketch(on_origin(FrameAxis::Z), block));
    program.push("block", extrude("base", 1.0, false));
    program.push("boss_sketch", sketch(on_face("extrude(block,end)"), boss));
    program.push(
        "boss",
        ExtrudeArgs {
            output: Output::Solid(Combine::join("extrude(block)")),
            ..extrude("boss_sketch", 0.4, false)
        },
    );
    let part = program.build_model().unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("extrude(boss)").is_ok());
    assert!(part.solid_id("extrude(block)").is_err());
    assert!(part.solid_id("extrude(boss,tool)").is_err());
    // The faces keep the names their extrudes gave them.
    assert!(part.face_id("extrude(boss,end)").is_ok());
    assert!(part.face_id("extrude(block,start)").is_ok());
}

/// A rectangle beside a construction axis, its inner edge constrained onto
/// the axis, revolved as far as `extent` says — or, as a `face`, its curves
/// swept into faces standing on their own.
fn revolved_rectangle(
    extent: Extents,
    face: bool,
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let (p, _) = polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    for point in [p[0], p[3]] {
        s.constrain(Constraint::PointOnCurve { point, curve: axis });
    }
    let mut program = Program::new();
    program.push("profile", sketch(on_origin(FrameAxis::Y), s));
    program.push(
        "turn",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Sketch(sketch_element("profile", axis))),
            extent,
            output: if face {
                Output::Faces { up_to: None }
            } else {
                Output::default()
            },
        },
    );
    program.build_model()
}

/// Any angle up to a full turn, either way, symmetric or each side its
/// own: a partial turn is closed by two caps sharing the edge on the axis,
/// three sides off the axis sweeping a face per span of at most a quarter
/// turn; sides adding up to a full turn close on themselves.
#[test]
fn revolve_turns_any_angle() {
    for (extent, faces) in [
        // A full turn: a cylinder, three profile edges off the axis, a
        // quadrant face each per 90°.
        (Extents::blind(360.0), 12),
        (Extents::blind(90.0), 3 + 2),
        (Extents::blind(-270.0), 9 + 2),
        (Extents::blind(45.0), 3 + 2),
        (both_ways(180.0), 6 + 2),
        (two_ways(30.0, 100.0), 6 + 2),
        (two_ways(200.0, 160.0), 12),
    ] {
        let part = revolved_rectangle(extent.clone(), false).unwrap();
        assert_valid(&part);
        assert_eq!(part.topology().faces.len(), faces, "{extent:?}");
        assert_eq!(part.face_id("revolve(turn,start)").is_ok(), faces != 12);
    }
    assert!(revolved_rectangle(two_ways(200.0, 200.0), false).is_err());
}

/// A square with a square hole, clear of the axis: a full turn sweeps the
/// hole into a void inside the ring, a quarter turn a tunnel through it.
#[test]
fn revolve_with_a_hole() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [4.0, 1.0], [4.0, 4.0], [1.0, 4.0]]);
    polygon(&mut s, &[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let full = revolved_on_zx(s.clone(), z.clone()).unwrap();
    assert_valid(&full);
    let ring = full.solid_id("revolve(ring)").unwrap();
    assert_eq!(full.topology().get_solid(ring).unwrap().shells.len(), 2);

    let mut program = Program::new();
    program.push("profile", sketch(on_origin(FrameAxis::Y), s));
    program.push(
        "ring",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(z),
            extent: Extents::blind(90.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let quarter = program.build_model().unwrap();
    assert_valid(&quarter);
    assert_eq!(quarter.topology().faces.len(), 4 + 4 + 2);
}

/// One sketch is one area: a sketch of two separate ones is refused.
#[test]
fn a_sketch_of_two_areas_is_refused() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    polygon(&mut s, &[[2.0, 0.0], [3.0, 0.0], [3.0, 1.0], [2.0, 1.0]]);
    let mut program = Program::new();
    program.push("squares", sketch(on_origin(FrameAxis::Z), s));
    program.push("slabs", extrude("squares", 1.0, false));
    let Err(error) = program.build_model() else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("separate areas"), "{error:?}");
}

/// A unit square on the Z plane, and `step` built on it.
fn on_unit_square(
    step: ExtrudeArgs,
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", sketch(on_origin(FrameAxis::Z), s));
    program.push("e", step);
    program.build_model()
}

/// Each side its own length: from one below the plane to the other above.
#[test]
fn extrude_each_side_its_own_length() {
    let part = on_unit_square(ExtrudeArgs {
        extent: two_ways(1.0, 0.5),
        ..extrude("square", 1.0, false)
    })
    .unwrap();
    assert_valid(&part);
    let height = |name: &str| {
        let v = part.vertex_id(name).unwrap();
        part.topology().get_vertex(v).unwrap().point[2]
    };
    assert!(height("extrude(e,square,p0,start)").could_be_equal(S::from_f64(-0.5)));
    assert!(height("extrude(e,square,p0,end)").could_be_equal(S::from_f64(1.0)));
}

/// As a face, the area's outline sweeps a tube of faces standing on their
/// own — no caps, no solid.
#[test]
fn extrude_an_area_as_faces() {
    let part = on_unit_square(ExtrudeArgs {
        output: Output::Faces { up_to: None },
        ..extrude("square", 1.0, false)
    })
    .unwrap();
    let model = part.topology();
    assert!(model.solids.is_empty());
    assert_eq!(part.sheet_face_names().len(), 4);
    if let Err(e) = validate(&ValidationParameters::default(), model) {
        panic!("{e:?}");
    }
}

/// As a face, a sketch of a line enclosing nothing sweeps a single face —
/// which a solid can then be split with; a revolved arc sweeps a surface of
/// revolution.
#[test]
fn sweep_a_line_as_a_face() {
    let mut s = Sketch::new();
    let a = s.add_point(n(1.0), n(0.0));
    let b = s.add_point(n(2.0), n(1.0));
    let line = s.add_line(a, b);
    let mut program = Program::new();
    program.push("slant", sketch(on_origin(FrameAxis::Y), s));
    program.push(
        "wall",
        ExtrudeArgs {
            output: Output::Faces { up_to: None },
            ..extrude("slant", 1.0, true)
        },
    );
    program.push(
        "cone",
        RevolveArgs {
            sketch: whole_sketch("slant"),
            axis: Some(RevolveAxis::Datum(origin_axis(FrameAxis::Z))),
            extent: Extents::blind(360.0),
            output: Output::Faces { up_to: None },
        },
    );
    let part = program.build_model().unwrap();
    assert!(part.topology().solids.is_empty());
    assert!(part.face_id(&format!("extrude(wall,slant,{line})")).is_ok());
    assert_eq!(part.sheet_face_names().len(), 1 + 4);
    if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
        panic!("{e:?}");
    }
}

/// The step `id`, `args` for the operation they are for.
fn as_step<A: StepArgs>(id: &str, args: A) -> ProgramStep {
    let mut program = Program::new();
    program.push(id, args);
    program.steps.remove(0)
}

/// A block, from `z0` to `z1` over `[0, 2]^2`, sketched on the Z plane
/// and extruded both ways from it, and `steps` after it.
fn on_block(
    z0: f64,
    z1: f64,
    steps: Vec<ProgramStep>,
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let mut base = Sketch::new();
    polygon(&mut base, &[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
    let mut post = Sketch::new();
    polygon(&mut post, &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]);
    let mut program = Program::new();
    program.push("base", sketch(on_origin(FrameAxis::Z), base));
    program.push(
        "block",
        ExtrudeArgs {
            extent: two_ways(z1, -z0),
            ..extrude("base", 1.0, false)
        },
    );
    program.push("post_sketch", sketch(on_origin(FrameAxis::Z), post));
    program.steps.extend(steps);
    program.build_model()
}

/// An extrude going up to the next face of a target: the post sketched
/// under the block grows up to the block's bottom and joins it there;
/// cut into the block, it cuts up to where it comes out of it.
#[test]
fn extrude_up_to_next() {
    let up_to_next = |combine: Combine| {
        as_step(
            "post",
            ExtrudeArgs {
                extent: one_way(Extent::UpToNext),
                output: Output::Solid(combine),
                ..extrude("post_sketch", 1.0, false)
            },
        )
    };
    let target = || "extrude(block)".to_string();

    let joined = on_block(2.0, 3.0, vec![up_to_next(Combine::join(target()))]).unwrap();
    assert_valid(&joined);
    assert_eq!(joined.solid_names(), ["extrude(post)"]);
    // The post's foot on the sketch, its head on the block's bottom: none of
    // the tool past that is left.
    assert_eq!(heights(&joined), [0, 2000, 3000]);
    // The block's six faces, the post's four walls and its foot — and the
    // block's top face in two: the tool, built long enough to reach past
    // the block, was imprinted on it where it came out again, and the
    // pieces it leaves are not merged back.
    assert_eq!(joined.topology().faces.len(), 6 + 4 + 1 + 1);

    let cut = on_block(0.0, 1.0, vec![up_to_next(Combine::cut(target()))]).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [0, 1000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    // Nothing ahead to stop at.
    let Err(error) = on_block(-2.0, -1.0, vec![up_to_next(Combine::join(target()))]) else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("nothing"), "{error:?}");

    // A new body stops at the next face of any solid, as a body of its
    // own, the block left as it was.
    let apart = on_block(2.0, 3.0, vec![up_to_next(Combine::NewBody)]).unwrap();
    assert_valid(&apart);
    assert_eq!(apart.solid_names(), ["extrude(block)", "extrude(post)"]);
    let block = apart.solid_id("extrude(block)").unwrap();
    assert_eq!(apart.topology().solid_faces(block).unwrap().len(), 6);
    let post = apart.solid_id("extrude(post)").unwrap();
    let mut post_heights: Vec<i64> = apart
        .topology()
        .iter_body_vertices(post)
        .unwrap()
        .map(|v| (apart.topology().vertices[&v].point[2].to_f64() * 1000.0).round() as i64)
        .collect();
    post_heights.sort();
    post_heights.dedup();
    assert_eq!(post_heights, [0, 2000]);
}

/// A square beside the z-axis turned up to the next face of the walls
/// standing at `walls` — each a rectangle on the Z plane from `z = -1` to
/// `2` — united into one solid.
fn turned_up_to_walls(
    walls: &[[[f64; 2]; 4]],
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let mut program = Program::new();
    let mut solids = Vec::new();
    for (k, corners) in walls.iter().enumerate() {
        let mut wall = Sketch::new();
        polygon(&mut wall, corners);
        let (sketch_id, id) = (format!("wall_sketch{k}"), format!("wall{k}"));
        program.push(&sketch_id, sketch(on_origin(FrameAxis::Z), wall));
        program.push(
            &id,
            ExtrudeArgs {
                extent: two_ways(2.0, 1.0),
                ..extrude(&sketch_id, 1.0, false)
            },
        );
        solids.push(format!("extrude({id})"));
    }
    let mut target = solids[0].clone();
    for (k, other) in solids.iter().enumerate().skip(1) {
        let id = format!("walls{k}");
        program.push(
            &id,
            BooleanArgs {
                a: solid(target),
                b: solid(other.clone()),
                op: BooleanOp::Union,
            },
        );
        target = format!("boolean({id})");
    }
    let mut profile = Sketch::new();
    // Sketch x along world x, sketch y along world -z.
    polygon(
        &mut profile,
        &[[2.0, -1.0], [3.0, -1.0], [3.0, 0.0], [2.0, 0.0]],
    );
    program.push("profile", sketch(on_origin(FrameAxis::Y), profile));
    program.push(
        "arm",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Datum(origin_axis(FrameAxis::Z))),
            extent: one_way(Extent::UpToNext),
            output: Output::Solid(Combine::join(target)),
        },
    );
    program.build_model()
}

/// A revolve going up to the next face: the square turns until it runs
/// into the wall standing in its way, and joins it there — but walls on
/// either side of the axis stand all around it, and no turn short of a full
/// one comes past them.
#[test]
fn revolve_up_to_next() {
    let near = [[-0.5, -1.5], [0.5, -1.5], [0.5, -3.5], [-0.5, -3.5]];
    let far = [[-0.5, 1.5], [0.5, 1.5], [0.5, 3.5], [-0.5, 3.5]];
    let part = turned_up_to_walls(&[near]).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["revolve(arm)"]);
    // The arm ends on a side of the wall: where its arcs, of radius 2 and
    // 3, meet `x = 0.5` or `x = -0.5`, whichever way it turned.
    let on_wall = |r: f64| {
        part.topology().vertices.values().any(|v| {
            let p = [0, 1].map(|k| v.point[k].to_f64());
            ((p[0].abs() - 0.5).abs() < 1e-6)
                && ((p[0] * p[0] + p[1] * p[1]).sqrt() - r).abs() < 1e-6
        })
    };
    assert!(on_wall(2.0) && on_wall(3.0));

    let Err(error) = turned_up_to_walls(&[near, far]) else {
        panic!("revolved");
    };
    assert!(error.root_message().contains("all around"), "{error:?}");
}

/// A face that ends inside the solid cuts nothing off.
#[test]
fn split_by_a_face_ending_inside_fails() {
    let mut cut = Sketch::new();
    // Sketch x along world x, sketch y along world -z: from z = 0.5 — in the
    // block, which spans z in [0, 1] — down past its bottom.
    let a = cut.add_point(n(1.0), n(-0.5));
    let b = cut.add_point(n(1.0), n(1.0));
    let line = cut.add_line(a, b);
    let program_steps = vec![
        as_step("cut_sketch", sketch(on_origin(FrameAxis::Y), cut)),
        as_step(
            "cut",
            ExtrudeArgs {
                output: Output::Faces { up_to: None },
                extent: both_ways(-5.0),
                ..extrude("cut_sketch", 1.0, false)
            },
        ),
        as_step(
            "halves",
            SplitArgs {
                solid: solid("extrude(block)"),
                face: element(format!("extrude(cut,cut_sketch,{line})")),
            },
        ),
    ];
    let Err(error) = on_block(0.0, 1.0, program_steps) else {
        panic!("split");
    };
    assert!(
        error.root_message().contains("inside the solid"),
        "{error:?}"
    );
}

/// Up to next from a face of the target itself: a "C" of two blocks joined
/// by a pillar, and a post sketched on the lower block's top, grown up to
/// the upper block's bottom — its foot lying on the face it was drawn on.
#[test]
fn extrude_up_to_next_from_a_face_of_the_target() {
    let between = |sketch: &str, z0: f64, z1: f64| -> ExtrudeArgs {
        ExtrudeArgs {
            extent: two_ways(z1, -z0),
            ..extrude(sketch, 1.0, false)
        }
    };
    let mut program = Program::new();
    for (id, corners) in [
        (
            "slab_sketch",
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        ),
        (
            "pillar_sketch",
            [[2.0, 0.0], [2.4, 0.0], [2.4, 2.0], [2.0, 2.0]],
        ),
    ] {
        let mut s = Sketch::new();
        polygon(&mut s, &corners);
        program.push(id, sketch(on_origin(FrameAxis::Z), s));
    }
    program.push("lower", between("slab_sketch", 0.0, 1.0));
    program.push("upper", between("slab_sketch", 2.0, 3.0));
    program.push("pillar", between("pillar_sketch", 0.0, 3.0));
    for (id, a, b) in [
        ("c1", "extrude(lower)", "extrude(pillar)"),
        ("c2", "boolean(c1)", "extrude(upper)"),
    ] {
        program.push(
            id,
            BooleanArgs {
                a: solid(a),
                b: solid(b),
                op: BooleanOp::Union,
            },
        );
    }
    let mut post = Sketch::new();
    let c = post.add_point(n(0.5), n(1.0));
    post.add_circle(c, n(0.3));
    program.push("post_sketch", sketch(on_face("extrude(lower,end)"), post));
    program.push(
        "post",
        ExtrudeArgs {
            extent: one_way(Extent::UpToNext),
            output: Output::Solid(Combine::join("boolean(c2)")),
            ..extrude("post_sketch", 1.0, false)
        },
    );
    let part = program.build_model().unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["extrude(post)"]);
    // The post spans the gap, from the lower block's top to the upper one's
    // bottom, and is not left sticking out of the top — where the tool came
    // out of the upper block again, only its imprint is left on that face.
    let heights = |of: &str| -> Vec<i64> {
        let mut z: Vec<i64> = part
            .topology()
            .vertices
            .iter()
            .filter(|(v, _)| part.name_of(**v).is_some_and(|n| n.contains(of)))
            .map(|(_, v)| (v.point[2].to_f64() * 1000.0).round() as i64)
            .collect();
        z.sort();
        z.dedup();
        z
    };
    assert_eq!(heights("post_sketch"), [1000, 2000, 3000]);
    assert_eq!(heights(""), [0, 1000, 2000, 3000]);
}

/// The post of [`on_block`] extruded up from the Z plane as far as `side1`
/// says, combined with the block as `combine` says — `reversed` turning it
/// the other way.
fn post_to_block(
    z0: f64,
    z1: f64,
    side1: Extent,
    reversed: bool,
    op: BooleanOp,
) -> geop_core_math::geop_error::GeopResult<NamedModel<geop_core_program::values::Design>> {
    let post = ExtrudeArgs {
        extent: Extents {
            side1,
            symmetric: false,
            side2: None,
            reversed,
        },
        output: Output::Solid(Combine::With {
            op,
            target: solid("extrude(block)"),
        }),
        ..extrude("post_sketch", 1.0, false)
    };
    on_block(z0, z1, vec![as_step("post", post)])
}

/// The heights of `part`'s vertices, in thousandths.
fn heights(part: &NamedModel<geop_core_program::values::Design>) -> Vec<i64> {
    let mut z: Vec<i64> = part
        .topology()
        .vertices
        .values()
        .map(|v| (v.point[2].to_f64() * 1000.0).round() as i64)
        .collect();
    z.sort();
    z.dedup();
    z
}

/// A cut or an intersection up to next from a sketch outside the target
/// goes up to the target, and from there as far as its next face: through
/// the block it meets.
#[test]
fn cut_and_intersect_up_to_next_from_outside() {
    let cut = post_to_block(2.0, 3.0, Extent::UpToNext, false, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(cut.solid_names(), ["extrude(post)"]);
    assert_eq!(heights(&cut), [2000, 3000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    let kept = post_to_block(2.0, 3.0, Extent::UpToNext, false, BooleanOp::Intersection).unwrap();
    assert_valid(&kept);
    assert_eq!(heights(&kept), [2000, 3000]);
    assert_eq!(kept.topology().faces.len(), 6);
}

/// Through all goes through the whole target, whichever way it is combined.
#[test]
fn through_all() {
    let cut = post_to_block(2.0, 3.0, Extent::ThroughAll, false, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [2000, 3000]);
    let kept = post_to_block(2.0, 3.0, Extent::ThroughAll, false, BooleanOp::Intersection).unwrap();
    assert_valid(&kept);
    assert_eq!(kept.topology().faces.len(), 6);
    // Nothing of the block ahead the other way.
    assert!(post_to_block(2.0, 3.0, Extent::ThroughAll, true, BooleanOp::Difference).is_err());
}

/// Turned the other way, a cut from a sketch above the block goes down into
/// it; going away from it, it meets nothing, and says so.
#[test]
fn reversed_cut_up_to_next() {
    let cut = post_to_block(-1.0, 0.0, Extent::UpToNext, true, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [-1000, 0]);
    let Err(error) = post_to_block(-1.0, 0.0, Extent::UpToNext, false, BooleanOp::Difference)
    else {
        panic!("cut");
    };
    assert!(error.root_message().contains("nothing"), "{error:?}");
}

/// A face going up to the next face of a solid stops where it meets it,
/// and leaves the solid as it is; through all, it goes past it.
#[test]
fn faces_up_to_next_and_through_all() {
    let face = |side1: Extent| {
        let post = ExtrudeArgs {
            extent: Extents {
                side1,
                symmetric: false,
                side2: None,
                reversed: false,
            },
            output: Output::Faces {
                up_to: Some(solid("extrude(block)")),
            },
            ..extrude("post_sketch", 1.0, false)
        };
        on_block(2.0, 3.0, vec![as_step("post", post)]).unwrap()
    };
    let up_to = face(Extent::UpToNext);
    let model = up_to.topology();
    if let Err(e) = validate(&ValidationParameters::default(), model) {
        panic!("{e:?}");
    }
    // The block untouched, the post's walls from the plane to its bottom.
    let block = up_to.solid_id("extrude(block)").unwrap();
    assert_eq!(model.solid_faces(block).unwrap().len(), 6);
    assert_eq!(up_to.sheet_face_names().len(), 4);
    assert_eq!(heights(&up_to), [0, 2000, 3000]);

    let through = face(Extent::ThroughAll);
    assert!(heights(&through).last().unwrap() > &3000);
}

/// A post joined up to the next face of a wedge hanging over half of it,
/// its edge pointing down: the half under the wedge meets it, the other
/// half never does, so nothing stops the post all round. It goes through
/// all — not flat up to the wedge's lowest edge, where its end would touch
/// the wedge along that existing edge alone.
#[test]
fn join_up_to_next_partly_under_a_wedge_goes_through_all() {
    let mut wedge = Sketch::new();
    // Sketch x along world x, sketch y along world -z: the edge at `z = 2`,
    // the top at `z = 4`.
    polygon(&mut wedge, &[[1.0, -2.0], [3.0, -4.0], [-1.0, -4.0]]);
    let y_plane = on_origin(FrameAxis::Y);
    let wedge_steps = vec![
        as_step("wedge_sketch", sketch(y_plane, wedge)),
        // From `y = 0` to `y = 1`: over half the post, `y` in `[0.5, 1.5]`.
        as_step("wedge", extrude("wedge_sketch", 1.0, false)),
        as_step(
            "post",
            ExtrudeArgs {
                extent: one_way(Extent::UpToNext),
                output: Output::Solid(Combine::join("extrude(wedge)")),
                ..extrude("post_sketch", 1.0, false)
            },
        ),
    ];
    let mut program = Program::new();
    let mut post = Sketch::new();
    polygon(&mut post, &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]);
    let z_plane = on_origin(FrameAxis::Z);
    program.push("post_sketch", sketch(z_plane, post));
    program.steps.extend(wedge_steps);
    let part = program.build_model().unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["extrude(post)"]);
    // Past the wedge's top.
    assert!(
        heights(&part).last().unwrap() > &4000,
        "{:?}",
        heights(&part)
    );
}
