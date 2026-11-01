//! Split face run as a step: a sketch projected onto a block's top and onto
//! a cylinder, along the sketch's normal and the face's; the rest of the
//! configurations are in `regression_tests/split_face.rs`.

use std::rc::Rc;

use crate::{Along, SplitFace, SplitFaceArgs};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    program::Program,
    testing::{assert_incremental, assert_round_trips, edit_sequence},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_lib_sweep::shapes::{block_part, revolved_cylinder, v3};
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_origin},
    whole_sketch,
};
use geop_ops_model::{
    NamedModel, model,
    testing::{Built, assert_valid, error, face_towards, given_part, model_of, run_step},
};
use geop_ops_sketch::{AddSketch, AddSketchArgs};

/// The block the sketches are projected onto: `[-1, 1]^2 x [0, 1]`.
fn block() -> NamedModel {
    block_part("block", [-1.0, -1.0, 0.0], [1.0, 1.0, 1.0])
}

/// The name of the face of `block` facing `normal`.
fn face_of(block: &NamedModel, normal: [f64; 3]) -> String {
    let solid = block.solid_id("cube(block)").unwrap();
    let face = face_towards(block, solid, normal);
    block.name_of(face).unwrap().to_string()
}

/// The block with the sketch `mark` on the plane `z = 0` projected onto
/// its top.
fn split_top(mark: Sketch) -> GeopResult<NamedModel> {
    let block = block();
    let top = face_of(&block, [0.0, 0.0, 1.0]);
    let mut part = given_part(&block)?;
    let on_xy = AddSketchArgs::on(on_origin(FrameAxis::Z), mark);
    run_step::<AddSketch>(&mut part, "mark", &on_xy)?;
    let split = SplitFaceArgs {
        face: Some(ElementReference::new(model(), top)),
        sketch: whole_sketch("mark"),
        along: Along::SketchNormal,
    };
    run_step::<SplitFace>(&mut part, "p", &split)?;
    Ok(model_of(&part))
}

/// A circle projected up onto a block's top cuts a disc out of it: a face
/// of its own, inside a hole of the rest of the top.
#[test]
fn a_circle_cuts_a_disc_out_of_a_block_top() {
    let mut mark = Sketch::new();
    let c = mark.add_point(n(0.2), n(0.1));
    mark.add_circle(c, n(0.4));
    let part = split_top(mark).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.topology().faces.len(), 7);
    let top = part.face_id(&face_of(&block(), [0.0, 0.0, 1.0])).unwrap();
    assert_eq!(part.topology().get_face(top).unwrap().holes.len(), 1);
}

/// A line projected onto a block's top divides it in two, and leaves the
/// bottom — on the line's own plane — as it was.
#[test]
fn a_line_divides_a_block_top() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(2.0));
    mark.add_line(a, b);
    let part = split_top(mark).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 7);
    let bottom = part.face_id(&face_of(&block(), [0.0, 0.0, -1.0])).unwrap();
    assert_eq!(part.topology().get_face(bottom).unwrap().holes.len(), 0);
}

/// A line ending on the top divides nothing, and is refused.
#[test]
fn a_line_ending_on_the_face_fails() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(0.0));
    mark.add_line(a, b);
    let error = error(split_top(mark));
    assert!(error.root_message().contains("ends inside"), "{error:?}");
}

/// A circle in the plane `x = 0`, round `(0, 1, 1)` of radius `0.2`,
/// projected onto a cylinder of radius one round the line `x = -2, y = 0`
/// each point to its nearest: the circle the cylinder is cut along is the
/// sketch's circle seen from the axis — every point of it on the cylinder,
/// at the height of the point it came from and on the ray to it from the
/// axis — not along the sketch's normal, which misses the cylinder where
/// the circle is further than one from the axis.
#[test]
fn a_circle_is_projected_onto_a_cylinder_along_its_normal() {
    let mut cylinder = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut cylinder, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    // The quarter of the side facing the circle.
    let facing = v3(-2.0 + 0.894, 0.447, 1.0);
    let side = (cylinder.topology().faces.iter())
        .find(|(_, face)| {
            let (_, _, p) = face.surface.closest_point(&facing).unwrap();
            p.sub(&facing).norm().to_f64() < 1e-3
        })
        .map(|(&id, _)| cylinder.name_of(id).unwrap().to_string())
        .expect("a face facing the circle");
    let mut part = given_part(&cylinder).unwrap();
    let mut mark = Sketch::new();
    let c = mark.add_point(n(1.0), n(1.0));
    mark.add_circle(c, n(0.2));
    let on_yz = AddSketchArgs::on(on_origin(FrameAxis::X), mark);
    run_step::<AddSketch>(&mut part, "mark", &on_yz).unwrap();
    let split = SplitFaceArgs {
        face: Some(ElementReference::new(model(), side.clone())),
        sketch: whole_sketch("mark"),
        along: Along::FaceNormal,
    };
    run_step::<SplitFace>(&mut part, "p", &split).unwrap();
    let part = model_of(&part);
    assert_valid(&part);
    let model = part.topology();
    let side = part.face_id(&side).unwrap();
    let hole = model.get_face(side).unwrap().holes.len();
    assert_eq!(hole, 1);
    let cut = (model.edges.iter())
        .filter(|(id, _)| part.name_of(**id).unwrap().starts_with("split_face(p"))
        .map(|(_, e)| e)
        .next()
        .expect("an edge cut along");
    for i in 0..=16 {
        let (t0, t1) = cut.curve.domain();
        let t = Scalar::interpolate(t0, t1, Scalar::from_ratio(i, 16).unwrap());
        let p = cut.curve.evaluate(t).unwrap().to_f64_array();
        let (x, y) = (p[0] + 2.0, p[1]);
        assert!(
            ((x * x + y * y).sqrt() - 1.0).abs() < 1e-6,
            "{p:?} is off the cylinder"
        );
        // The sketch's point it came from: on the ray from the axis, in
        // the plane `x = 0`.
        let from = [2.0 * y / x, p[2]];
        let off = ((from[0] - 1.0).powi(2) + (from[1] - 1.0).powi(2)).sqrt() - 0.2;
        assert!(
            off.abs() < 1e-4,
            "{p:?} comes from {from:?}, {off:e} off the circle"
        );
    }
}

/// The circle of [`a_circle_is_projected_onto_a_cylinder_along_its_normal`]
/// projected each point to its nearest: the edges cut along enclose the
/// true projection — each point of the circle's nearest on the cylinder,
/// on the ray to it from the axis — not merely come near it.
#[test]
fn a_projection_along_the_face_normal_encloses_the_true_one() {
    use geop_core_geometry::{
        contains::curve::curve_could_contain,
        search::{MAX_NODES, min_subdivision_size},
    };
    let mut cylinder = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut cylinder, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    let facing = v3(-2.0 + 0.894, 0.447, 1.0);
    let side = (cylinder.topology().faces.iter())
        .find(|(_, face)| {
            let (_, _, p) = face.surface.closest_point(&facing).unwrap();
            p.sub(&facing).norm().to_f64() < 1e-3
        })
        .map(|(&id, _)| cylinder.name_of(id).unwrap().to_string())
        .expect("a face facing the circle");
    let mut part = given_part(&cylinder).unwrap();
    let mut mark = Sketch::new();
    let c = mark.add_point(n(1.0), n(1.0));
    mark.add_circle(c, n(0.2));
    let on_yz = AddSketchArgs::on(on_origin(FrameAxis::X), mark);
    run_step::<AddSketch>(&mut part, "mark", &on_yz).unwrap();
    let split = SplitFaceArgs {
        face: Some(ElementReference::new(model(), side)),
        sketch: whole_sketch("mark"),
        along: Along::FaceNormal,
    };
    run_step::<SplitFace>(&mut part, "p", &split).unwrap();
    let part = model_of(&part);
    let model = part.topology();
    let cuts: Vec<_> = (model.edges.iter())
        .filter(|(id, _)| part.name_of(**id).unwrap().starts_with("split_face(p"))
        .map(|(_, e)| &e.curve)
        .collect();
    assert!(!cuts.is_empty());
    for i in 0..48 {
        let a = std::f64::consts::TAU * (i as f64 + 0.5) / 48.0;
        let (y, z) = (1.0 + 0.2 * a.cos(), 1.0 + 0.2 * a.sin());
        // The circle's point `(0, y, z)`, seen from the axis.
        let r = (4.0 + y * y).sqrt();
        let foot = v3(-2.0 + 2.0 / r, y / r, z);
        let on = cuts.iter().any(|curve| {
            curve_could_contain(*curve, &foot, MAX_NODES, min_subdivision_size())
                .unwrap()
                .is_some()
        });
        assert!(
            on,
            "the true projection's point {foot:?} is on no edge cut along"
        );
    }
}

/// The sheet a circle is projected onto a cylinder along the face's normal
/// with: its rails enclose the ends of the line along the normal at the
/// foot of every point of the circle, not only at the feet they were fitted
/// through — what makes where the sheet crosses the face enclose the
/// projection.
#[test]
fn normal_sheets_enclose_the_lines_along_the_normal() {
    use geop_core_geometry::{
        contains::curve::curve_could_contain,
        nurb_curve::{NurbCurve3D, sqrt2_over_2},
        search::{MAX_NODES, min_subdivision_size},
    };
    use geop_ops_model::{Chain, Namer};
    let mut cylinder = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut cylinder, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    let facing = v3(-2.0 + 0.894, 0.447, 1.0);
    let side = (cylinder.topology().faces.iter())
        .find(|(_, face)| {
            let (_, _, p) = face.surface.closest_point(&facing).unwrap();
            p.sub(&facing).norm().to_f64() < 1e-3
        })
        .map(|(&id, _)| id)
        .expect("a face facing the circle");
    // The circle round `(0, 1, 1)` of radius `0.2` in the plane `x = 0`,
    // in four quarters.
    let centre = v3(0.0, 1.0, 1.0);
    let at = |k: usize| {
        let a = std::f64::consts::FRAC_PI_2 * k as f64;
        v3(0.0, 1.0 + 0.2 * a.cos(), 1.0 + 0.2 * a.sin())
    };
    let quarters: Vec<NurbCurve3D> = (0..4)
        .map(|k| {
            let (p, q) = (at(k), at(k + 1));
            NurbCurve3D::arc(p, p.add(&q).sub(&centre), q, sqrt2_over_2()).unwrap()
        })
        .collect();
    let chain = Chain {
        name: "circle".into(),
        curves: quarters.clone(),
        curve_names: (0..4).map(|k| format!("c{k}")).collect(),
        joint_names: (0..4).map(|k| format!("p{k}")).collect(),
    };
    let namer = Namer::new("split_face", "p").unwrap();
    crate::split_face::normal_sheets(&mut cylinder, &namer, side, &[chain]).unwrap();
    let model = cylinder.topology();
    let rail = |curve: usize, end: &str| {
        let name = format!("split_face(p,c{curve},{end})");
        let id = cylinder.edge_id(&name).unwrap();
        model.get_edge(id).unwrap().curve.clone()
    };
    let reach = {
        let hull = geop_ops_model::face_hull(model, &[side]).unwrap();
        let (lo, hi) =
            geop_core_math::plain::bounds(hull.iter().map(|p| p.to_f64_array())).unwrap();
        geop_core_math::plain::diagonal(lo, hi) / 16.0
    };
    for (k, quarter) in quarters.iter().enumerate() {
        let (near, far) = (rail(k, "near"), rail(k, "far"));
        for i in 0..=40 {
            let p = quarter
                .evaluate(Scalar::from_ratio(i, 40).unwrap())
                .unwrap();
            let [_, y, z] = p.to_f64_array();
            // Its nearest point on the cylinder, seen from the axis, and
            // the ends of the line along the normal there.
            let r = (4.0 + y * y).sqrt();
            let (ox, oy) = (2.0 / r, y / r);
            let end = |s: f64| v3(-2.0 + ox * (1.0 + s * reach), oy * (1.0 + s * reach), z);
            let on = |curve: &NurbCurve3D, q: &geop_core_math::vector::Vector3| {
                curve_could_contain(curve, q, MAX_NODES, min_subdivision_size())
                    .unwrap()
                    .is_some()
            };
            let (out, inward) = (end(1.0), end(-1.0));
            assert!(
                (on(&near, &out) && on(&far, &inward)) || (on(&near, &inward) && on(&far, &out)),
                "quarter {k} at {i}/40: the rails miss {out:?} and {inward:?}"
            );
        }
    }
}

/// A program of a given block, a sketch `mark` with a line at `x` on the
/// plane `z = 0`, and its top split along it.
fn split_block(x: f64) -> Program {
    let mut program = Program::new();
    program.push(
        "b",
        PrimitiveArgs::block([-1.0, -1.0, 0.0], [1.0, 1.0, 1.0]),
    );
    let mut mark = Sketch::new();
    let (a, b) = (mark.add_point(n(x), n(-2.0)), mark.add_point(n(x), n(2.0)));
    mark.add_line(a, b);
    program.push("mark", AddSketchArgs::on(on_origin(FrameAxis::Z), mark));
    program.push(
        "s",
        SplitFaceArgs {
            face: Some(ElementReference::new(model(), "block(b,end)")),
            sketch: whole_sketch("mark"),
            along: Along::SketchNormal,
        },
    );
    program
}

/// The operations of [`split_block`].
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?
        .with(AddSketch)?
        .with(SplitFace)
}

/// The step saves, loads and builds the very same part, its names and all.
#[test]
fn split_face_round_trips() {
    let built = assert_round_trips(Rc::new(registry().unwrap()), None, &split_block(0.3));
    let part = model_of(&built);
    assert_valid(&part);
    assert!(part.face_id("block(b,end)").is_ok());
    assert!(part.face_id("split_face(s,block(b,end),mark,c2)").is_ok());
}

/// A change of the sketch builds the split again, where the line now is, as
/// a build from scratch does.
#[test]
fn a_change_of_the_sketch_rebuilds_the_split() {
    let registry_of = Rc::new(registry().unwrap());
    assert_incremental(
        registry_of.clone(),
        None,
        &[split_block(0.3), split_block(-0.2), split_block(0.3)],
    );
    assert_incremental(registry_of, None, &edit_sequence(&split_block(0.3)));
    let at = |x| {
        let part = split_block(x).build_model(registry).unwrap();
        let edge = part.edge_id("split_face(s,mark,c2)").unwrap();
        let line = part.topology().get_edge(edge).unwrap().curve.clone();
        line.evaluate(line.mid_parameter().unwrap()).unwrap()[0]
    };
    assert!(at(0.3).could_be_equal(Scalar::from_f64(0.3)));
    assert!(at(-0.2).could_be_equal(Scalar::from_f64(-0.2)));
}
