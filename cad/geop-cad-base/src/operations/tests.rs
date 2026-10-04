//! The operations run as programs: sketches extruded and revolved into
//! solids, and what they refuse.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_sketch::PointId;
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::Part;
use geop_ops::{EntityRef, NoFiles, ORIGIN};
use geop_ops_booleans::{BooleanArgs, Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, RevolveArgs};
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch::{Constraint, Sketch};

use crate::Program;
use crate::examples::n;

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
    let p: Vec<PointId> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    p
}

fn sketch(plane: EntityRef, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(plane),
        sketch,
        ..Default::default()
    }
}

fn extrude(sketch: &str, distance: f64, symmetric: bool) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: sketch.into(),
        extent: Extents {
            side1: Extent::Blind(distance),
            symmetric,
            side2: None,
            reversed: false,
        },
        face: false,
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
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push(
        "slot",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("plate", extrude("slot", 0.5, false));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    let part = program.build::<S>(&NoFiles).unwrap();
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
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
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
            axis: Some(EntityRef::SketchCurve {
                sketch: "profile".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    // Three profile edges off the axis, a quadrant face each per 90°.
    assert_eq!(part.topology().faces.len(), 12);
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
            axis: Some(EntityRef::SketchCurve {
                sketch: "half_disc".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
            axis: Some(EntityRef::SketchCurve {
                sketch: "profile".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    assert!(program.build::<S>(&NoFiles).is_err());
}

/// Revolves `s`, sketched on the origin's zx plane, around `axis`.
fn revolved_on_zx(s: Sketch, axis: EntityRef) -> geop_core_math::geop_error::GeopResult<Part<S>> {
    let mut program = Program::new();
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            s,
        ),
    );
    program.push(
        "ring",
        RevolveArgs {
            sketch: "profile".into(),
            axis: Some(axis),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.build::<S>(&NoFiles)
}

/// A square clear of an axis from outside the sketch — the origin's z axis,
/// which lies in the sketch's plane — revolves into a ring with a square
/// cross section.
#[test]
fn revolve_square_around_an_outside_axis_is_a_ring() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]);
    let z = EntityRef::datum_component(ORIGIN, DatumComponent::Axis(FrameAxis::Z));
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
    let part = revolved_on_zx(
        s,
        EntityRef::SketchCurve {
            sketch: "profile".into(),
            curve: axis,
        },
    )
    .unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
}

/// A profile touching an axis from outside the sketch is refused: nothing
/// says which of its edges lie on that axis.
#[test]
fn revolve_touching_an_outside_axis_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let z = EntityRef::datum_component(ORIGIN, DatumComponent::Axis(FrameAxis::Z));
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
    let y = EntityRef::datum_component(ORIGIN, DatumComponent::Axis(FrameAxis::Y));
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
    program.push(
        "square",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("slab", extrude("square", 1.0, false));
    program.push("again", extrude("extrude(slab)", 1.0, false));
    assert!(program.build::<S>(&NoFiles).is_err());
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
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("extrude(boss)").is_ok());
    assert!(part.solid_id("extrude(block)").is_err());
    assert!(part.solid_id("extrude(boss,tool)").is_err());
    // The faces keep the names their extrudes gave them.
    assert!(part.face_id("extrude(boss,end)").is_ok());
    assert!(part.face_id("extrude(block,start)").is_ok());
}

/// The rectangle of [`revolve_rectangle_about_construction_axis`], its
/// inner edge on the axis, revolved as far as `extent` says — or, as a
/// `face`, its curves swept into faces standing on their own.
fn revolved_rectangle(
    extent: Extents,
    face: bool,
) -> geop_core_math::geop_error::GeopResult<Part<S>> {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let p = polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    for point in [p[0], p[3]] {
        s.constrain(Constraint::PointOnCurve { point, curve: axis });
    }
    let mut program = Program::new();
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            s,
        ),
    );
    program.push(
        "turn",
        RevolveArgs {
            sketch: "profile".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "profile".into(),
                curve: axis,
            }),
            extent,
            face,
            combine: Combine::NewBody,
        },
    );
    program.build::<S>(&NoFiles)
}

/// Any angle up to a full turn, either way, symmetric or each side its
/// own: a partial turn is closed by two caps sharing the edge on the axis,
/// three sides off the axis sweeping a face per span of at most a quarter
/// turn; sides adding up to a full turn close on themselves.
#[test]
fn revolve_turns_any_angle() {
    let two = |a: f64, b: f64| Extents {
        side1: Extent::Blind(a),
        symmetric: false,
        side2: Some(Extent::Blind(b)),
        reversed: false,
    };
    let symmetric = |a: f64| Extents {
        side1: Extent::Blind(a),
        symmetric: true,
        side2: None,
        reversed: false,
    };
    for (extent, faces) in [
        (Extents::blind(90.0), 3 + 2),
        (Extents::blind(-270.0), 9 + 2),
        (Extents::blind(45.0), 3 + 2),
        (symmetric(180.0), 6 + 2),
        (two(30.0, 100.0), 6 + 2),
        (two(200.0, 160.0), 12),
    ] {
        let part = revolved_rectangle(extent.clone(), false).unwrap();
        assert_valid(&part);
        assert_eq!(part.topology().faces.len(), faces, "{extent:?}");
        assert_eq!(part.face_id("revolve(turn,start)").is_ok(), faces != 12);
    }
    assert!(revolved_rectangle(two(200.0, 200.0), false).is_err());
}

/// A square with a square hole, clear of the axis: a full turn sweeps the
/// hole into a void inside the ring, a quarter turn a tunnel through it.
#[test]
fn revolve_with_a_hole() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [4.0, 1.0], [4.0, 4.0], [1.0, 4.0]]);
    polygon(&mut s, &[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let z = EntityRef::datum_component(ORIGIN, DatumComponent::Axis(FrameAxis::Z));
    let full = revolved_on_zx(s.clone(), z.clone()).unwrap();
    assert_valid(&full);
    let ring = full.solid_id("revolve(ring)").unwrap();
    assert_eq!(full.topology().get_solid(ring).unwrap().shells.len(), 2);

    let mut program = Program::new();
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            s,
        ),
    );
    program.push(
        "ring",
        RevolveArgs {
            sketch: "profile".into(),
            axis: Some(z),
            extent: Extents::blind(90.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let quarter = program.build::<S>(&NoFiles).unwrap();
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
    program.push(
        "squares",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            s,
        ),
    );
    program.push("slabs", extrude("squares", 1.0, false));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("separate areas"), "{error:?}");
}

/// A unit square on the Z plane, and `step` built on it.
fn on_unit_square(step: ExtrudeArgs) -> geop_core_math::geop_error::GeopResult<Part<S>> {
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
    program.push("e", step);
    program.build::<S>(&NoFiles)
}

/// Each side its own length: from one below the plane to the other above.
#[test]
fn extrude_each_side_its_own_length() {
    let part = on_unit_square(ExtrudeArgs {
        extent: Extents {
            side1: Extent::Blind(1.0),
            symmetric: false,
            side2: Some(Extent::Blind(0.5)),
            reversed: false,
        },
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
        face: true,
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
    program.push(
        "slant",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            s,
        ),
    );
    program.push(
        "wall",
        ExtrudeArgs {
            face: true,
            ..extrude("slant", 1.0, true)
        },
    );
    program.push(
        "cone",
        RevolveArgs {
            sketch: "slant".into(),
            axis: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Axis(FrameAxis::Z),
            )),
            extent: Extents::blind(360.0),
            face: true,
            combine: Combine::NewBody,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert!(part.topology().solids.is_empty());
    assert!(part.face_id(&format!("extrude(wall,slant,{line})")).is_ok());
    assert_eq!(part.sheet_face_names().len(), 1 + 4);
    if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
        panic!("{e:?}");
    }
}

/// A block, from `z0` to `z1` over `[0, 2]^2`, sketched on the Z plane
/// and extruded both ways from it, and `steps` after it.
fn on_block(
    z0: f64,
    z1: f64,
    steps: Vec<(&str, crate::PartOperation)>,
) -> geop_core_math::geop_error::GeopResult<Part<S>> {
    let mut base = Sketch::new();
    polygon(&mut base, &[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
    let mut post = Sketch::new();
    polygon(&mut post, &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]);
    let z_plane = || EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z));
    let mut program = Program::new();
    program.push("base", sketch(z_plane(), base));
    program.push(
        "block",
        ExtrudeArgs {
            extent: Extents {
                side1: Extent::Blind(z1),
                symmetric: false,
                side2: Some(Extent::Blind(-z0)),
                reversed: false,
            },
            ..extrude("base", 1.0, false)
        },
    );
    program.push("post_sketch", sketch(z_plane(), post));
    for (id, step) in steps {
        program.push(id, step);
    }
    program.build::<S>(&NoFiles)
}

/// An extrude going up to the next face of a target: the post sketched
/// under the block grows up to the block's bottom and joins it there;
/// cut into the block, it cuts up to where it comes out of it.
#[test]
fn extrude_up_to_next() {
    let up_to_next = |combine: Combine| -> crate::PartOperation {
        ExtrudeArgs {
            extent: Extents {
                side1: Extent::UpToNext,
                symmetric: false,
                side2: None,
                reversed: false,
            },
            combine,
            ..extrude("post_sketch", 1.0, false)
        }
        .into()
    };
    let target = || "extrude(block)".to_string();
    let heights = |part: &Part<S>| {
        let mut z: Vec<i64> = part
            .topology()
            .vertices
            .values()
            .map(|v| (v.point[2].to_f64() * 1000.0).round() as i64)
            .collect();
        z.sort();
        z.dedup();
        z
    };

    let joined = on_block(
        2.0,
        3.0,
        vec![("post", up_to_next(Combine::Union { target: target() }))],
    )
    .unwrap();
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

    let cut = on_block(
        0.0,
        1.0,
        vec![("post", up_to_next(Combine::Difference { target: target() }))],
    )
    .unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [0, 1000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    // Nothing ahead to stop at.
    let Err(error) = on_block(
        -2.0,
        -1.0,
        vec![("post", up_to_next(Combine::Union { target: target() }))],
    ) else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("nothing"), "{error:?}");

    // A new body stops at the next face of any solid, as a body of its
    // own, the block left as it was.
    let apart = on_block(2.0, 3.0, vec![("post", up_to_next(Combine::NewBody))]).unwrap();
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
fn turned_up_to_walls(walls: &[[[f64; 2]; 4]]) -> geop_core_math::geop_error::GeopResult<Part<S>> {
    let z_plane = || EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z));
    let mut program = Program::new();
    let mut solids = Vec::new();
    for (k, corners) in walls.iter().enumerate() {
        let mut wall = Sketch::new();
        polygon(&mut wall, corners);
        let (sketch_id, id) = (format!("wall_sketch{k}"), format!("wall{k}"));
        program.push(&sketch_id, sketch(z_plane(), wall));
        program.push(
            &id,
            ExtrudeArgs {
                extent: Extents {
                    side1: Extent::Blind(2.0),
                    symmetric: false,
                    side2: Some(Extent::Blind(1.0)),
                    reversed: false,
                },
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
                a: target,
                b: other.clone(),
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
    program.push(
        "profile",
        sketch(
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            profile,
        ),
    );
    program.push(
        "arm",
        RevolveArgs {
            sketch: "profile".into(),
            axis: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Axis(FrameAxis::Z),
            )),
            extent: Extents {
                side1: Extent::UpToNext,
                symmetric: false,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::Union { target },
        },
    );
    program.build::<S>(&NoFiles)
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
        (
            "cut_sketch",
            sketch(
                EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
                cut,
            )
            .into(),
        ),
        (
            "cut",
            ExtrudeArgs {
                face: true,
                extent: Extents {
                    side1: Extent::Blind(-5.0),
                    symmetric: true,
                    side2: None,
                    reversed: false,
                },
                ..extrude("cut_sketch", 1.0, false)
            }
            .into(),
        ),
        (
            "halves",
            geop_ops_booleans::SplitArgs {
                solid: "extrude(block)".into(),
                face: format!("extrude(cut,cut_sketch,{line})"),
            }
            .into(),
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
    let z_plane = || EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z));
    let between = |sketch: &str, z0: f64, z1: f64| -> crate::PartOperation {
        ExtrudeArgs {
            extent: Extents {
                side1: Extent::Blind(z1),
                symmetric: false,
                side2: Some(Extent::Blind(-z0)),
                reversed: false,
            },
            ..extrude(sketch, 1.0, false)
        }
        .into()
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
        program.push(id, sketch(z_plane(), s));
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
                a: a.into(),
                b: b.into(),
                op: BooleanOp::Union,
            },
        );
    }
    let mut post = Sketch::new();
    let c = post.add_point(n(0.5), n(1.0));
    post.add_circle(c, n(0.3));
    program.push(
        "post_sketch",
        sketch(
            EntityRef::Face {
                name: "extrude(lower,end)".into(),
            },
            post,
        ),
    );
    program.push(
        "post",
        ExtrudeArgs {
            extent: Extents {
                side1: Extent::UpToNext,
                symmetric: false,
                side2: None,
                reversed: false,
            },
            combine: Combine::Union {
                target: "boolean(c2)".into(),
            },
            ..extrude("post_sketch", 1.0, false)
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    combine: fn(String) -> Combine,
) -> geop_core_math::geop_error::GeopResult<Part<S>> {
    let post = ExtrudeArgs {
        extent: Extents {
            side1,
            symmetric: false,
            side2: None,
            reversed,
        },
        combine: combine("extrude(block)".into()),
        ..extrude("post_sketch", 1.0, false)
    };
    on_block(z0, z1, vec![("post", post.into())])
}

/// The heights of `part`'s vertices, in thousandths.
fn heights(part: &Part<S>) -> Vec<i64> {
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
    let cut = post_to_block(2.0, 3.0, Extent::UpToNext, false, |target| {
        Combine::Difference { target }
    })
    .unwrap();
    assert_valid(&cut);
    assert_eq!(cut.solid_names(), ["extrude(post)"]);
    assert_eq!(heights(&cut), [2000, 3000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    let kept = post_to_block(2.0, 3.0, Extent::UpToNext, false, |target| {
        Combine::Intersection { target }
    })
    .unwrap();
    assert_valid(&kept);
    assert_eq!(heights(&kept), [2000, 3000]);
    assert_eq!(kept.topology().faces.len(), 6);
}

/// Through all goes through the whole target, whichever way it is combined.
#[test]
fn through_all() {
    let cut = post_to_block(2.0, 3.0, Extent::ThroughAll, false, |target| {
        Combine::Difference { target }
    })
    .unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [2000, 3000]);
    let kept = post_to_block(2.0, 3.0, Extent::ThroughAll, false, |target| {
        Combine::Intersection { target }
    })
    .unwrap();
    assert_valid(&kept);
    assert_eq!(kept.topology().faces.len(), 6);
    // Nothing of the block ahead the other way.
    assert!(
        post_to_block(2.0, 3.0, Extent::ThroughAll, true, |target| {
            Combine::Difference { target }
        })
        .is_err()
    );
}

/// Turned the other way, a cut from a sketch above the block goes down into
/// it; going away from it, it meets nothing, and says so.
#[test]
fn reversed_cut_up_to_next() {
    let cut = post_to_block(-1.0, 0.0, Extent::UpToNext, true, |target| {
        Combine::Difference { target }
    })
    .unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [-1000, 0]);
    let Err(error) = post_to_block(-1.0, 0.0, Extent::UpToNext, false, |target| {
        Combine::Difference { target }
    }) else {
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
            face: true,
            combine: Combine::Union {
                target: "extrude(block)".into(),
            },
            ..extrude("post_sketch", 1.0, false)
        };
        on_block(2.0, 3.0, vec![("post", post.into())]).unwrap()
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
