//! The operations run as programs: sketches extruded and revolved into
//! solids, and what they refuse.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_sketch::{Constraint, PointId, Sketch};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::Part;
use geop_ops::{EntityRef, ORIGIN};
use geop_ops_booleans::{BooleanArgs, Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::{ExtrudeArgs, RevolveArgs};
use geop_ops_sketch::AddSketchArgs;

use crate::Program;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
}

/// A closed polygon through `corners`, returning its point ids.
fn polygon(s: &mut Sketch, corners: &[[f64; 2]]) -> Vec<PointId> {
    let p: Vec<PointId> = corners.iter().map(|c| s.add_point(c[0], c[1])).collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    p
}

fn sketch(plane: EntityRef, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs { plane, sketch }
}

fn extrude(sketch: &str, distance: f64, symmetric: bool) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: sketch.into(),
        distance,
        symmetric,
        combine: Combine::NewBody,
    }
}

/// A symmetric extrude of a unit square on the Z plane by total thickness
/// 1 spans z in [-0.5, 0.5] — the sketch plane through the middle of the
/// solid, not at one face of it.
#[test]
fn symmetric_extrude_is_centered_on_the_sketch_plane() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push(
        "square",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("slab", extrude("square", 1.0, true));
    let part = program.build::<S>().unwrap();
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
        s.add_point(0.0, 0.0),
        s.add_point(2.0, 0.0),
        s.add_point(2.0, 1.0),
        s.add_point(0.0, 1.0),
    ];
    s.add_line(p[0], p[1]);
    s.add_arc_with_sweep(p[1], p[2], PI);
    s.add_line(p[2], p[3]);
    s.add_arc_with_sweep(p[3], p[0], PI);
    let c = s.add_point(1.0, 0.5);
    s.add_circle(c, 0.25);
    let mut program = Program::new();
    program.push(
        "slot",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("plate", extrude("slot", 0.5, false));
    let part = program.build::<S>().unwrap();
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
    let c = boss.add_point(0.5, 0.5);
    boss.add_circle(c, 0.3);
    let mut program = Program::new();
    program.push(
        "base",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            block,
        ),
    );
    program.push("block", extrude("base", 1.0, false));
    program.push(
        "boss_sketch",
        sketch(
            EntityRef::Face {
                name: "extrude(block,end)".into(),
            },
            boss,
        ),
    );
    program.push("boss", extrude("boss_sketch", 0.4, false));
    program.push(
        "join",
        BooleanArgs {
            a: "extrude(block)".into(),
            b: "extrude(boss)".into(),
            op: BooleanOp::Union,
        },
    );
    let part = program.build::<S>().unwrap();
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

/// A rectangle beside a construction axis, with its inner edge constrained
/// onto the axis: revolving gives a cylinder.
#[test]
fn revolve_rectangle_about_construction_axis() {
    let mut s = Sketch::new();
    let a0 = s.add_point(0.0, -1.0);
    let a1 = s.add_point(0.0, 2.0);
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let p = polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    s.constrain(Constraint::PointOnCurve {
        point: p[0],
        curve: axis,
    });
    s.constrain(Constraint::PointOnCurve {
        point: p[3],
        curve: axis,
    });
    let mut program = Program::new();
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            s,
        ),
    );
    program.push(
        "tube",
        RevolveArgs {
            sketch: "profile".into(),
            axis,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>().unwrap();
    assert_valid(&part);
    // Three profile edges off the axis, a quadrant face each per 90°.
    assert_eq!(part.topology().faces.len(), 12);
}

/// A half disc whose diameter is the axis itself: revolving gives a sphere,
/// from an arc profile.
#[test]
fn revolve_half_disc_is_sphere() {
    let mut s = Sketch::new();
    let top = s.add_point(0.0, 1.0);
    let bottom = s.add_point(0.0, -1.0);
    let axis = s.add_line(bottom, top);
    s.add_arc_with_sweep(top, bottom, PI);
    let mut program = Program::new();
    program.push(
        "half_disc",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::X)),
            s,
        ),
    );
    program.push(
        "ball",
        RevolveArgs {
            sketch: "half_disc".into(),
            axis,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>().unwrap();
    assert_valid(&part);
    // The half circle is split into two quarters.
    assert_eq!(part.topology().faces.len(), 8);
}

/// A profile straddling the axis is rejected rather than revolved into a
/// self-intersecting solid.
#[test]
fn revolve_across_axis_fails() {
    let mut s = Sketch::new();
    let a0 = s.add_point(0.0, -1.0);
    let a1 = s.add_point(0.0, 2.0);
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    polygon(&mut s, &[[-0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [-0.5, 1.0]]);
    let mut program = Program::new();
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push(
        "bad",
        RevolveArgs {
            sketch: "profile".into(),
            axis,
            combine: Combine::NewBody,
        },
    );
    assert!(program.build::<S>().is_err());
}

/// Extrude refers to a sketch by name; naming a solid instead is an error.
#[test]
fn extrude_of_non_sketch_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push(
        "square",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("slab", extrude("square", 1.0, false));
    program.push("again", extrude("extrude(slab)", 1.0, false));
    assert!(program.build::<S>().is_err());
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
    let c = boss.add_point(0.5, 0.5);
    boss.add_circle(c, 0.3);
    let mut program = Program::new();
    program.push(
        "base",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            block,
        ),
    );
    program.push("block", extrude("base", 1.0, false));
    program.push(
        "boss_sketch",
        sketch(
            EntityRef::Face {
                name: "extrude(block,end)".into(),
            },
            boss,
        ),
    );
    program.push(
        "boss",
        ExtrudeArgs {
            combine: Combine::Union {
                target: "extrude(block)".into(),
            },
            ..extrude("boss_sketch", 0.4, false)
        },
    );
    let part = program.build::<S>().unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("extrude(boss)").is_ok());
    assert!(part.solid_id("extrude(block)").is_err());
    assert!(part.solid_id("extrude(boss,tool)").is_err());
    // The faces keep the names their extrudes gave them.
    assert!(part.face_id("extrude(boss,end)").is_ok());
    assert!(part.face_id("extrude(block,start)").is_ok());
}
