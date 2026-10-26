//! The operations run as programs: uniting given solids — and extruding
//! and revolving up to the next face of solids so united — and splitting a
//! solid with a face standing on its own, and what that refuses.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{
    element::{ElementReference, HasElements},
    operation_registry::OperationRegistry,
    program::Program,
};
use geop_lib_fixtures::{GivenDatum, GivenDatumArgs, Primitive, PrimitiveArgs};
use geop_ops_edit::{DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs};
use geop_ops_extrude_revolve::{
    Extent, ExtrudeArgs, Output, RevolveArgs, RevolveAxis,
    testing::{extrude, one_way, two_ways},
};
use geop_ops_model::datums::{datum, origin_axis};
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_face, on_origin, polygon},
    whole_sketch,
};
use geop_ops_model::{
    BooleanOp, Combine, NamedModel, Path, model, solid,
    testing::{Built, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{
    Boolean, BooleanArgs, Imprint, ImprintArgs, Section, SectionArgs, SectionWith, Split, SplitArgs,
};

/// The operations these tests' programs are written in: given solids, and
/// the sketches and extrusions the tests of up to next are about.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Primitive)?
        .with(Boolean)?
        .with(Split)?
        .with(DeleteBody)?
        .with(ExtractFace)?
        .with(Imprint)?
        .with(Section)?
        .with(GivenDatum)
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
    // The big block's wall at `x = 1`, which the small block stands through.
    let wall = "block(a,c1)";
    let extracted = format!("extract(x,{wall})");

    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0, 0.0, -1.0], [1.0, 1.0, 1.0]));
    program.push("b", PrimitiveArgs::block([0.5, 0.2, -0.5], [1.5, 0.8, 0.5]));
    program.push("x", ExtractFaceArgs { face: named(wall) });
    program.push(
        "halves",
        SplitArgs {
            solid: solid("block(b)"),
            face: named(&extracted),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.sheet_face_names(), [extracted.as_str()]);
    assert_eq!(
        part.solid_names(),
        ["block(a)", "split(halves,0)", "split(halves,1)"]
    );
    assert!(part.face_id(wall).is_ok(), "the wall itself stays");

    program.push(
        "clean",
        DeleteBodyArgs {
            bodies: vec![named("block(a)"), named(extracted)],
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.solid_names(), ["split(halves,0)", "split(halves,1)"]);
}

/// A cylinder standing on a block's top face, united with it: a boss on a
/// block.
#[test]
fn boss_on_block_top_face() {
    let mut program = Program::new();
    program.push("block", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    program.push("boss", PrimitiveArgs::cylinder_z([0.5, 0.5, 1.0], 0.3, 0.4));
    program.push(
        "join",
        BooleanArgs {
            a: solid("block(block)"),
            b: solid("cylinder(boss)"),
            op: BooleanOp::Union,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1, "the boss joined the block");
    // The boss sits on top of the block, not below its top.
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

/// A face that ends inside the solid cuts nothing off: a wall of a second
/// block, extracted, standing at `x = 1` from below the first block up to
/// `z = 0.5` — in the block, which spans `z` in `[0, 1]`.
#[test]
fn split_by_a_face_ending_inside_fails() {
    let mut program = Program::new();
    program.push("block", PrimitiveArgs::block([0.0; 3], [2.0, 2.0, 1.0]));
    program.push(
        "cut",
        PrimitiveArgs::block([1.0, -5.0, -5.0], [6.0, 5.0, 0.5]),
    );
    program.push(
        "x",
        ExtractFaceArgs {
            face: named("block(cut,c3)"),
        },
    );
    program.push(
        "halves",
        SplitArgs {
            solid: solid("block(block)"),
            face: named("extract(x,block(cut,c3))"),
        },
    );
    let Err(error) = program.build_model(registry) else {
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
    // out of the upper block again, the pieces of that face its imprint cut
    // are merged back, and no vertex of the post is left there.
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
    assert_eq!(heights("post_sketch"), [1000, 2000]);
    assert_eq!(heights(""), [0, 1000, 2000, 3000]);
}

/// Two unit blocks overlapping by half, combined by a symmetric
/// difference: what is in one but not both, two blocks of half the size
/// touching along the overlap's edges, together of volume one. The step
/// round-trips and builds incrementally like any other.
#[test]
fn symmetric_difference_of_two_overlapping_blocks() {
    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
    program.push("b", PrimitiveArgs::block([0.5, 0.0, 0.0], [1.5, 1.0, 1.0]));
    program.push(
        "x",
        BooleanArgs {
            a: solid("block(a)"),
            b: solid("block(b)"),
            op: BooleanOp::SymmetricDifference,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let volume = part
        .topology()
        .mass_properties(part.solid_id("boolean(x)").unwrap())
        .unwrap()
        .volume;
    assert!(volume.could_be_equal(Scalar::ONE), "{volume:?}");
    geop_core_program::testing::assert_round_trips(
        std::rc::Rc::new(registry().unwrap()),
        None,
        &program,
    );
}

/// Two unit blocks, the second moved by half along each axis.
fn overlapping_blocks() -> Program {
    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    program.push("b", PrimitiveArgs::block([0.5; 3], [1.5; 3]));
    program
}

/// The name of the face of the solid `solid` of `part` that lies highest.
fn top_face(part: &NamedModel, solid: &str) -> String {
    let topology = part.topology();
    let faces = topology.solid_faces(part.solid_id(solid).unwrap()).unwrap();
    let top = (faces.into_iter())
        .max_by(|&f, &g| {
            let z = |f| topology.faces[&f].surface.aabb()[2].lower().to_f64();
            z(f).total_cmp(&z(g))
        })
        .unwrap();
    part.name(top).unwrap().to_string()
}

/// Two overlapping blocks imprinted on each other: both stay, three faces
/// of each divided where the other crosses it, and the volumes are as they
/// were. The step round-trips and builds incrementally.
#[test]
fn imprint_two_overlapping_blocks() {
    let mut program = overlapping_blocks();
    program.push(
        "i",
        ImprintArgs {
            body: named("block(a)"),
            tool: named("block(b)"),
            face: None,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["block(a)", "block(b)"]);
    for name in ["block(a)", "block(b)"] {
        let solid = part.solid_id(name).unwrap();
        let topology = part.topology();
        assert_eq!(topology.solid_faces(solid).unwrap().len(), 9, "{name}");
        let volume = topology.mass_properties(solid).unwrap().volume;
        assert!(volume.could_be_equal(Scalar::ONE), "{name}: {volume:?}");
    }
    let registry = std::rc::Rc::new(registry().unwrap());
    geop_core_program::testing::assert_round_trips(registry.clone(), None, &program);
    let before = overlapping_blocks();
    geop_core_program::testing::assert_incremental(registry, None, &[before, program]);
}

/// Imprinted on one face only, the first block's top, the second block
/// divides that face alone and stays as it was.
#[test]
fn imprint_onto_one_face() {
    let mut program = overlapping_blocks();
    // The first block's top, at `z = 1`.
    let blocks = program.build_model(registry).unwrap();
    let top = top_face(&blocks, "block(a)");
    program.push(
        "i",
        ImprintArgs {
            body: named("block(a)"),
            tool: named("block(b)"),
            face: Some(named(&top)),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let faces = |name: &str| {
        let solid = part.solid_id(name).unwrap();
        part.topology().solid_faces(solid).unwrap().len()
    };
    assert_eq!((faces("block(a)"), faces("block(b)")), (7, 6));

    // A face of the other block is refused, by name.
    let mut wrong = overlapping_blocks();
    wrong.push(
        "i",
        ImprintArgs {
            body: named("block(a)"),
            tool: named("block(b)"),
            face: Some(named(top_face(&blocks, "block(b)"))),
        },
    );
    let error = wrong.build_model(registry).unwrap_err();
    assert!(
        error.root_message().contains("is not of the body"),
        "{error:?}"
    );
}

/// The section of two overlapping blocks is one closed run of six curves,
/// both blocks left as they are: a path, picked by any of its edges.
#[test]
fn section_of_two_overlapping_blocks_is_a_closed_path() {
    let mut program = overlapping_blocks();
    program.push(
        "s",
        SectionArgs {
            body: named("block(a)"),
            with: SectionWith::Body(named("block(b)")),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let topology = part.topology();
    assert_eq!(topology.wires.len(), 1);
    assert_eq!(topology.faces.len(), 12, "the blocks as they were");
    let wire = topology.wires.values().next().unwrap();
    let edge = part.name(wire.edges[3]).unwrap();
    let path = <NamedModel as HasElements<Path>>::element(&part, edge).unwrap();
    assert_eq!(path.curves.len(), 6);
    assert_eq!(path.joint_names.len(), 6, "closed");
    geop_core_program::testing::assert_round_trips(
        std::rc::Rc::new(registry().unwrap()),
        None,
        &program,
    );
}

/// A cylinder cut by a datum plane across its axis: a circle, its four
/// quarters on the quarters of the side.
#[test]
fn section_of_a_cylinder_by_a_datum_plane() {
    let mut program = Program::new();
    program.push("c", PrimitiveArgs::cylinder_z([0.0; 3], 0.5, 2.0));
    program.push("p", GivenDatumArgs::plane([0.0, 0.0, 0.7], [0.0, 0.0, 1.0]));
    program.push(
        "s",
        SectionArgs {
            body: named("cylinder(c)"),
            with: SectionWith::Plane(datum("p")),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let topology = part.topology();
    let wire = topology.wires.values().next().unwrap();
    assert_eq!(wire.edges.len(), 4);
    for v in &wire.vertices {
        let z = topology.vertices[v].point[2];
        assert!(z.could_be_equal(Scalar::from_f64(0.7)), "{z:?}");
    }

    // A plane above the cylinder crosses nothing, and is refused.
    let mut above = Program::new();
    above.push("c", PrimitiveArgs::cylinder_z([0.0; 3], 0.5, 2.0));
    above.push("p", GivenDatumArgs::plane([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]));
    above.push(
        "s",
        SectionArgs {
            body: named("cylinder(c)"),
            with: SectionWith::Plane(datum("p")),
        },
    );
    let error = above.build_model(registry).unwrap_err();
    assert!(error.root_message().contains("do not meet"), "{error:?}");
}
