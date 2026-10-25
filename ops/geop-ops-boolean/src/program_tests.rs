//! The operations run as programs: uniting solids extruded and revolved
//! from sketches — and extruding and revolving up to the next face of
//! solids so united — and splitting a solid with a face standing on its
//! own, and what that refuses.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    testing::as_step,
};
use geop_ops_edit::{DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs};
use geop_ops_extrude_revolve::{
    Extent, ExtrudeArgs, Output, RevolveArgs, RevolveAxis,
    testing::{block_and_post, both_ways, extrude, extrusion, one_way, two_ways},
};
use geop_ops_model::datums::origin_axis;
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_face, on_origin, polygon},
    whole_sketch,
};
use geop_ops_model::{
    BooleanOp, Combine, NamedModel, element, model, solid,
    testing::{Built, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{Boolean, BooleanArgs, Split, SplitArgs};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Boolean)?
        .with(Split)?
        .with(DeleteBody)?
        .with(ExtractFace)
}

/// The element `name` of the part's model, as a `K`.
fn named<K>(name: impl Into<String>) -> ElementReference<NamedModel, K> {
    ElementReference::new(model(), name)
}

/// A block's side wall, extracted, cuts a smaller block standing through
/// it in two; deleting the first block and the extracted face leaves
/// only the halves.
#[test]
fn extract_a_wall_split_with_it_and_delete_the_rest() {
    let mut big = Sketch::new();
    let (_, sides) = polygon(&mut big, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut small = Sketch::new();
    polygon(
        &mut small,
        &[[0.5, 0.2], [1.5, 0.2], [1.5, 0.8], [0.5, 0.8]],
    );
    let wall = format!("extrude(a,big,{})", sides[1]);
    let extracted = format!("extract(x,{wall})");

    let on_xy = |drawn| AddSketchArgs::on(on_origin(FrameAxis::Z), drawn);
    let mut program = Program::new();
    program.push("big", on_xy(big));
    program.push("a", extrusion("big", two_ways(1.0, 1.0), Combine::NewBody));
    program.push("small", on_xy(small));
    program.push(
        "b",
        extrusion("small", two_ways(0.5, 0.5), Combine::NewBody),
    );
    program.push("x", ExtractFaceArgs { face: named(&wall) });
    program.push(
        "halves",
        SplitArgs {
            solid: solid("extrude(b)"),
            face: named(&extracted),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.sheet_face_names(), [extracted.as_str()]);
    assert_eq!(
        part.solid_names(),
        ["extrude(a)", "split(halves,0)", "split(halves,1)"]
    );
    assert!(part.face_id(&wall).is_ok(), "the wall itself stays");

    program.push(
        "clean",
        DeleteBodyArgs {
            bodies: vec![named("extrude(a)"), named(extracted)],
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.solid_names(), ["split(halves,0)", "split(halves,1)"]);
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
    program.push("base", AddSketchArgs::on(on_origin(FrameAxis::Z), block));
    program.push("block", extrude("base", 1.0, false));
    program.push(
        "boss_sketch",
        AddSketchArgs::on(on_face("extrude(block,end)"), boss),
    );
    program.push("boss", extrude("boss_sketch", 0.4, false));
    program.push(
        "join",
        BooleanArgs {
            a: solid("extrude(block)"),
            b: solid("extrude(boss)"),
            op: BooleanOp::Union,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1, "the boss joined the block");
    // The boss sits on top of the block, not below the sketch plane.
    assert!(
        part.topology()
            .vertices
            .values()
            .any(|v| v.point[2].could_be_equal(Scalar::from_f64(1.4)))
    );
}

/// A square beside the z-axis turned up to the next face of the walls
/// standing at `walls` — each a rectangle on the Z plane from `z = -1` to
/// `2` — united into one solid.
fn turned_up_to_walls(walls: &[[[f64; 2]; 4]]) -> GeopResult<NamedModel> {
    let mut program = Program::new();
    let mut solids = Vec::new();
    for (k, corners) in walls.iter().enumerate() {
        let mut wall = Sketch::new();
        polygon(&mut wall, corners);
        let (sketch_id, id) = (format!("wall_sketch{k}"), format!("wall{k}"));
        program.push(&sketch_id, AddSketchArgs::on(on_origin(FrameAxis::Z), wall));
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
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::Y), profile),
    );
    program.push(
        "arm",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Datum(origin_axis(FrameAxis::Z))),
            extent: one_way(Extent::UpToNext),
            output: Output::Solid(Combine::join(target)),
        },
    );
    program.build_model(registry)
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
        as_step(
            "cut_sketch",
            AddSketchArgs::on(on_origin(FrameAxis::Y), cut),
        ),
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
    let Err(error) = block_and_post(0.0, 1.0, program_steps).build_model(registry) else {
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
        program.push(id, AddSketchArgs::on(on_origin(FrameAxis::Z), s));
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
    program.push(
        "post_sketch",
        AddSketchArgs::on(on_face("extrude(lower,end)"), post),
    );
    program.push(
        "post",
        ExtrudeArgs {
            extent: one_way(Extent::UpToNext),
            output: Output::Solid(Combine::join("boolean(c2)")),
            ..extrude("post_sketch", 1.0, false)
        },
    );
    let part = program.build_model(registry).unwrap();
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
