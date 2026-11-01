//! Split face, in each configuration a user meets: valid, of the pieces it
//! should be, of the same volume and area, or refused by name.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::FrameAxis,
    scalars::Scalar,
};
use geop_core_program::element::ElementReference;
use geop_core_topology::Model;
use geop_lib_fixtures::{
    GivenDatum, GivenDatumArgs,
    bodies::{block_drilled, one_solid},
};
use geop_lib_sweep::shapes::{block_part, revolved_cylinder, sphere_solid, v3};
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::{
    NamedModel, model,
    sketch::{
        Sketch, SketchPlane,
        figures::{n, on_face, on_origin, polygon},
        whole_sketch,
    },
    testing::{
        assert_draws_its_trims, assert_valid, assert_valid_sheets, face_towards, given_part,
        model_of, run_step,
    },
};
use geop_ops_model::{datums::datum, origin_of};
use geop_ops_plastic::{Draft, DraftArgs};
use geop_ops_sketch::{AddSketch, AddSketchArgs};

use crate::{Along, SplitFace, SplitFaceArgs};

/// The block the faces are divided on: `[-1, 1]^2 x [0, 1]`.
fn block() -> NamedModel {
    block_part("b", [-1.0, -1.0, 0.0], [1.0, 1.0, 1.0])
}

fn top_of(block: &NamedModel) -> String {
    let solid = block.solid_id("cube(b)").unwrap();
    let top = face_towards(block, solid, [0.0, 0.0, 1.0]);
    block.name_of(top).unwrap().to_string()
}

/// A cylinder of radius 1 and height 2 standing on the origin.
fn cylinder() -> NamedModel {
    let mut part = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut part, "c", v3(0.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    part
}

/// The face of `part` that `at`, which is on no other, lies on, by name.
fn face_at(part: &NamedModel, at: [f64; 3]) -> String {
    let p = v3(at[0], at[1], at[2]);
    let topology = part.topology();
    let found = topology.faces.iter().find(|(_, face)| {
        let (_, _, foot) = face.surface.closest_point(&p).unwrap();
        foot.sub(&p).norm().to_f64() < 1e-3
    });
    let (&id, _) = found.expect("a face there");
    part.name_of(id).unwrap().to_string()
}

/// `part` with `mark` sketched on `plane`, and each of `faces` divided along
/// it, one step each.
fn split_all(
    part: &NamedModel,
    faces: &[String],
    plane: SketchPlane,
    mark: Sketch,
) -> GeopResult<NamedModel> {
    let mut built = given_part(part)?;
    run_step::<AddSketch>(&mut built, "mark", &AddSketchArgs::on(plane, mark))?;
    for (k, face) in faces.iter().enumerate() {
        let args = SplitFaceArgs {
            face: Some(ElementReference::new(model(), face.clone())),
            sketch: whole_sketch("mark"),
            along: Along::SketchNormal,
        };
        run_step::<SplitFace>(&mut built, &format!("s{k}"), &args)?;
    }
    Ok(model_of(&built))
}

/// `after` is `before` with `more` faces: valid, drawn where its trims say,
/// of the same volume and area.
#[track_caller]
fn assert_only_divided(before: &NamedModel, after: &NamedModel, more: usize) {
    assert_valid(after);
    assert_draws_its_trims(after);
    let (was, is) = (before.topology(), after.topology());
    assert_eq!(is.faces.len(), was.faces.len() + more, "pieces");
    assert_eq!(after.solid_names(), before.solid_names());
    assert!(after.sheet_face_names().is_empty());
    let name = before.solid_names().remove(0);
    let (a, b) = (
        before.solid_id(&name).unwrap(),
        after.solid_id(&name).unwrap(),
    );
    let volume = |m: &Model, s| m.mass_properties(s).unwrap().volume;
    assert!(
        volume(was, a).could_be_equal(volume(is, b)),
        "volume {:?} became {:?}",
        volume(was, a),
        volume(is, b)
    );
    let (x, _) = was.solid_area(a).unwrap();
    let (y, _) = is.solid_area(b).unwrap();
    assert!(x.could_be_equal(y), "area {x:?} became {y:?}");
}

/// `mark` on the plane `z = 0`, dividing the block's top: `more` faces more
/// than the block had.
#[track_caller]
fn top_divided(mark: Sketch, more: usize) -> NamedModel {
    let block = block();
    let top = top_of(&block);
    let part = split_all(&block, &[top], on_origin(FrameAxis::Z), mark).unwrap();
    assert_only_divided(&block, &part, more);
    part
}

/// The refusal of `mark` on the plane `z = 0` for the block's top.
#[track_caller]
fn top_refused(mark: Sketch) -> GeopError {
    let block = block();
    let top = top_of(&block);
    match split_all(&block, &[top], on_origin(FrameAxis::Z), mark) {
        Ok(_) => panic!("expected a refusal"),
        Err(e) => e,
    }
}

fn line(mark: &mut Sketch, a: [f64; 2], b: [f64; 2]) {
    let (a, b) = (
        mark.add_point(n(a[0]), n(a[1])),
        mark.add_point(n(b[0]), n(b[1])),
    );
    mark.add_line(a, b);
}

fn circle(mark: &mut Sketch, at: [f64; 2], radius: f64) {
    let c = mark.add_point(n(at[0]), n(at[1]));
    mark.add_circle(c, n(radius));
}

/// Lines joining `points` in turn, leaving the last unjoined to the first.
fn open_polyline(mark: &mut Sketch, points: &[[f64; 2]]) {
    let ids: Vec<_> = (points.iter())
        .map(|p| mark.add_point(n(p[0]), n(p[1])))
        .collect();
    for pair in ids.windows(2) {
        mark.add_line(pair[0], pair[1]);
    }
}

/// A line reaching past the top on both sides is trimmed to it.
#[test]
fn a_line_reaching_past_the_face() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    top_divided(mark, 1);
}

/// A line ending exactly on the top's boundary, on two of its edges.
#[test]
fn a_line_from_edge_to_edge() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -1.0], [0.3, 1.0]);
    top_divided(mark, 1);
}

/// A line from corner to corner of the top.
#[test]
fn a_line_from_corner_to_corner() {
    let mut mark = Sketch::new();
    line(&mut mark, [-1.0, -1.0], [1.0, 1.0]);
    top_divided(mark, 1);
}

/// A sketch on the face itself: what is left round the circle keeps the
/// name, with a hole in it, and the disc is a new face.
#[test]
fn a_sketch_on_the_face_itself() {
    let block = block();
    let top = top_of(&block);
    let mut mark = Sketch::new();
    circle(&mut mark, [0.2, 0.1], 0.4);
    let part = split_all(&block, std::slice::from_ref(&top), on_face(&top), mark).unwrap();
    assert_only_divided(&block, &part, 1);
    let top = part.face_id(&top).unwrap();
    assert_eq!(part.topology().get_face(top).unwrap().holes.len(), 1);
}

/// Two circles apart: an island each.
#[test]
fn circles_apart() {
    let mut mark = Sketch::new();
    circle(&mut mark, [-0.5, 0.0], 0.3);
    circle(&mut mark, [0.5, 0.0], 0.3);
    top_divided(mark, 2);
}

/// One circle inside another: a disc inside a ring inside what is left.
#[test]
fn circles_nested() {
    let mut mark = Sketch::new();
    circle(&mut mark, [0.0, 0.0], 0.3);
    circle(&mut mark, [0.0, 0.0], 0.6);
    top_divided(mark, 2);
}

/// Two lines apart divide the top in three.
#[test]
fn two_lines_apart() {
    let mut mark = Sketch::new();
    line(&mut mark, [-0.3, -2.0], [-0.3, 2.0]);
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    top_divided(mark, 2);
}

/// Two lines crossing, sharing no point of the sketch, divide it in four.
#[test]
fn two_lines_crossing() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.2, -2.0], [0.2, 2.0]);
    line(&mut mark, [-2.0, 0.1], [2.0, 0.1]);
    top_divided(mark, 3);
}

/// Three lines meeting at a point of the sketch divide it in three.
#[test]
fn three_lines_meeting() {
    let mut mark = Sketch::new();
    let p = mark.add_point(n(0.1), n(0.2));
    for at in [[0.3, -2.0], [-2.0, 0.5], [2.0, 0.6]] {
        let a = mark.add_point(n(at[0]), n(at[1]));
        mark.add_line(p, a);
    }
    top_divided(mark, 2);
}

/// A zigzag leaving the top over its side `y = 1` and coming back: a corner
/// cut off at either end, and a wedge in the middle, each a stretch from edge
/// to edge, make four pieces.
#[test]
fn a_zigzag_crossing_several_times() {
    let mut mark = Sketch::new();
    open_polyline(
        &mut mark,
        &[[-1.5, 0.0], [-0.6, 1.5], [0.0, 0.0], [0.6, 1.5], [1.5, 0.0]],
    );
    top_divided(mark, 3);
}

/// A circle round the top's corner: only its quarter on the top counts.
#[test]
fn a_circle_round_a_corner() {
    let mut mark = Sketch::new();
    circle(&mut mark, [1.0, 1.0], 0.8);
    top_divided(mark, 1);
}

/// A circle across all four sides: a piece at each corner, and the middle.
#[test]
fn a_circle_across_all_sides() {
    let mut mark = Sketch::new();
    circle(&mut mark, [0.0, 0.0], 1.2);
    top_divided(mark, 4);
}

/// A circle and a line across it, of one sketch.
#[test]
fn a_line_across_a_circle() {
    let mut mark = Sketch::new();
    circle(&mut mark, [0.0, 0.0], 0.5);
    line(&mut mark, [0.1, -2.0], [0.1, 2.0]);
    top_divided(mark, 3);
}

/// An arc from side to side of the top.
#[test]
fn an_arc_across_the_face() {
    let mut mark = Sketch::new();
    let (a, b) = (
        mark.add_point(n(-0.5), n(-1.5)),
        mark.add_point(n(-0.5), n(1.5)),
    );
    mark.add_arc(a, b, n(1.0));
    top_divided(mark, 1);
}

/// A spline from side to side of the top, and a closed one inside the part
/// of it left of that.
#[test]
fn splines() {
    let mut mark = Sketch::new();
    let points = [[0.4, -2.0], [0.7, -0.5], [0.3, 0.5], [0.6, 2.0]];
    let open: Vec<_> = (points.iter())
        .map(|p| mark.add_point(n(p[0]), n(p[1])))
        .collect();
    mark.add_spline(open);
    let corners = [[-0.7, -0.3], [-0.2, -0.3], [-0.2, 0.3], [-0.7, 0.3]];
    let mut closed: Vec<_> = (corners.iter())
        .map(|p| mark.add_point(n(p[0]), n(p[1])))
        .collect();
    closed.push(closed[0]);
    mark.add_spline(closed);
    top_divided(mark, 2);
}

/// A square inside the top, of lines joined at their corners.
#[test]
fn a_closed_polygon() {
    let mut mark = Sketch::new();
    polygon(
        &mut mark,
        &[[-0.2, -0.3], [0.5, -0.3], [0.5, 0.4], [-0.2, 0.4]],
    );
    let part = top_divided(mark, 1);
    let top = part.face_id(&top_of(&block())).unwrap();
    assert_eq!(part.topology().get_face(top).unwrap().holes.len(), 1);
}

/// Construction curves do not divide.
#[test]
fn a_construction_line_beside_a_line() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    let (a, b) = (
        mark.add_point(n(-2.0), n(0.0)),
        mark.add_point(n(2.0), n(0.0)),
    );
    let helper = mark.add_line(a, b);
    mark.set_construction(helper, true);
    top_divided(mark, 1);
}

/// A letter's outlines each close inside the top: the `O`'s two make a ring
/// and its counter.
#[test]
fn a_text() {
    let mut mark = Sketch::new();
    let at = mark.add_point(n(-0.5), n(-0.3));
    mark.add_text(at, "O", n(0.8), n(0.0));
    top_divided(mark, 2);
}

/// A circle touching the top's sides from inside: the disc, and a corner
/// each.
#[test]
fn a_circle_touching_the_boundary() {
    let mut mark = Sketch::new();
    circle(&mut mark, [0.0, 0.0], 1.0);
    top_divided(mark, 4);
}

/// A line up a cylinder's wall, seen from the side: a straight generator.
#[test]
fn a_line_up_a_quarter_of_a_cylinder() {
    let part = cylinder();
    let face = face_at(&part, [0.8, 0.6, 1.0]);
    let mut mark = Sketch::new();
    line(&mut mark, [0.7, -3.0], [0.7, 3.0]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::Y), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// A level line across a quarter of a cylinder's wall, seen from the side:
/// an arc.
#[test]
fn a_level_line_across_a_quarter_of_a_cylinder() {
    let part = cylinder();
    let face = face_at(&part, [0.8, 0.6, 1.0]);
    let mut mark = Sketch::new();
    line(&mut mark, [-3.0, 1.0], [3.0, 1.0]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::X), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// A level line all the way round a cylinder: each quarter of its wall
/// divided, one step each, each next meeting the last on the edge they share.
#[test]
fn a_whole_loop_round_a_cylinder() {
    let part = cylinder();
    let faces = [
        [0.8, 0.6, 1.0],
        [-0.8, 0.6, 1.0],
        [-0.8, -0.6, 1.0],
        [0.8, -0.6, 1.0],
    ]
    .map(|at| face_at(&part, at));
    let mut mark = Sketch::new();
    line(&mut mark, [-3.0, 1.0], [3.0, 1.0]);
    let after = split_all(&part, &faces, on_origin(FrameAxis::X), mark).unwrap();
    assert_only_divided(&part, &after, 4);
}

/// A square projected onto a quarter of a cylinder's wall: a patch bounded
/// by two arcs and two generators.
#[test]
fn a_square_on_a_quarter_of_a_cylinder() {
    let part = cylinder();
    let face = face_at(&part, [0.8, 0.6, 1.0]);
    let mut mark = Sketch::new();
    polygon(&mut mark, &[[0.2, 0.7], [0.5, 0.7], [0.5, 1.3], [0.2, 1.3]]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::X), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// A line across the wedge of a cylinder's cap, ending on its arc.
#[test]
fn a_line_across_a_wedge_of_a_cap() {
    let part = cylinder();
    let face = face_at(&part, [0.3, 0.3, 2.0]);
    let mut mark = Sketch::new();
    line(&mut mark, [0.5, -3.0], [0.5, 3.0]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::Z), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// A circle on a patch of a sphere.
#[test]
fn a_circle_on_a_patch_of_a_sphere() {
    let mut part = NamedModel::new();
    sphere_solid(&mut part, "s", v3(0.0, 0.0, 0.0), Scalar::from_f64(1.0)).unwrap();
    let on = 1.0 / 3f64.sqrt();
    let face = face_at(&part, [on, on, on]);
    let mut mark = Sketch::new();
    circle(&mut mark, [0.5, 0.5], 0.1);
    let after = split_all(&part, &[face], on_origin(FrameAxis::Z), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// The top of a block with a hole round `(1, 1)`, and what is sketched on the
/// plane `z = 0` divides it.
fn drilled_top(mark: Sketch) -> GeopResult<(NamedModel, NamedModel)> {
    let (block, solid) = one_solid(&block_drilled());
    let id = block.solid_id(&solid).unwrap();
    let top = face_towards(&block, id, [0.0, 0.0, 1.0]);
    let top = block.name_of(top).unwrap().to_string();
    let after = split_all(&block, &[top], on_origin(FrameAxis::Z), mark)?;
    Ok((block, after))
}

/// A line through a hole: two stretches, from the outer boundary to the
/// hole, and the top divided in two, the hole in neither.
#[test]
fn a_line_through_a_hole() {
    let mut mark = Sketch::new();
    line(&mut mark, [1.1, -1.0], [1.1, 3.0]);
    let (block, after) = drilled_top(mark).unwrap();
    assert_only_divided(&block, &after, 1);
}

/// A circle round a hole: a ring and what is left, two holes in one.
#[test]
fn a_circle_round_a_hole() {
    let mut mark = Sketch::new();
    circle(&mut mark, [1.0, 1.0], 0.7);
    let (block, after) = drilled_top(mark).unwrap();
    assert_only_divided(&block, &after, 1);
}

/// A bend out of a hole and back into it: the wedge between it and the hole
/// is a piece, and what is left has the rest of the hole for its hole.
#[test]
fn a_bend_out_of_a_hole_and_back() {
    let mut mark = Sketch::new();
    open_polyline(&mut mark, &[[1.0, 1.1], [0.3, 1.0], [1.0, 0.9]]);
    let (block, after) = drilled_top(mark).unwrap();
    assert_only_divided(&block, &after, 1);
}

/// A line from the outer boundary to a hole only joins the hole to the
/// boundary: no piece is cut off.
#[test]
fn a_line_from_the_boundary_to_a_hole_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [1.0, -1.0], [1.0, 1.0]);
    let e = drilled_top(mark).expect_err("refused");
    assert!(e.root_message().contains("only joins"), "{e:?}");
    assert!(e.root_message().contains("mark,c"), "{e:?}");
}

/// A line beside the top misses it.
#[test]
fn a_sketch_missing_the_face_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [3.0, -2.0], [3.0, 2.0]);
    let e = top_refused(mark);
    assert!(e.root_message().contains("nothing crosses"), "{e:?}");
}

/// A line ending inside the top divides nothing.
#[test]
fn a_line_ending_inside_the_face_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 0.0]);
    let e = top_refused(mark);
    assert!(e.root_message().contains("ends inside"), "{e:?}");
    assert!(e.root_message().contains("mark,c"), "{e:?}");
}

/// A sketch of construction curves alone has nothing to divide along.
#[test]
fn only_construction_curves_are_refused() {
    let mut mark = Sketch::new();
    let (a, b) = (
        mark.add_point(n(-2.0), n(0.0)),
        mark.add_point(n(2.0), n(0.0)),
    );
    let helper = mark.add_line(a, b);
    mark.set_construction(helper, true);
    let e = top_refused(mark);
    assert!(e.root_message().contains("construction"), "{e:?}");
}

/// A line exactly along an edge of the top: that is a boundary already.
#[test]
fn a_line_along_an_edge_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [1.0, -1.0], [1.0, 1.0]);
    let e = top_refused(mark);
    assert!(e.root_message().contains("along its edge"), "{e:?}");
    assert!(e.root_message().contains("cube(b,c1,start)"), "{e:?}");
}

/// A line along a part of an edge of the top.
#[test]
fn a_line_along_a_part_of_an_edge_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [1.0, -0.5], [1.0, 0.5]);
    let e = top_refused(mark);
    assert!(e.root_message().contains("along its edge"), "{e:?}");
    assert!(e.root_message().contains("cube(b,c1,start)"), "{e:?}");
}

/// A square with a side on an edge of the top.
#[test]
fn a_square_with_a_side_along_an_edge_is_refused() {
    let mut mark = Sketch::new();
    polygon(
        &mut mark,
        &[[0.5, -0.5], [1.0, -0.5], [1.0, 0.5], [0.5, 0.5]],
    );
    let e = top_refused(mark);
    assert!(e.root_message().contains("along its edge"), "{e:?}");
    assert!(e.root_message().contains("cube(b,c1,start)"), "{e:?}");
}

/// A line along an edge of the top and reaching past both its ends.
#[test]
fn a_line_reaching_past_along_an_edge_is_refused() {
    let mut mark = Sketch::new();
    line(&mut mark, [1.0, -3.0], [1.0, 3.0]);
    let e = top_refused(mark);
    assert!(e.root_message().contains("along its edge"), "{e:?}");
    assert!(e.root_message().contains("cube(b,c1,start)"), "{e:?}");
}

/// A sketch whose plane stands square to the top, a line in it level with the
/// top: its sweep lies in the top's plane, and has no one curve on the top.
#[test]
fn a_sweep_in_the_plane_of_the_face_is_refused() {
    let block = block();
    let top = top_of(&block);
    let mut mark = Sketch::new();
    // On the plane `y = 0` the sketch's `y` is the world's `-z`.
    line(&mut mark, [-3.0, -1.0], [3.0, -1.0]);
    let e = split_all(&block, &[top], on_origin(FrameAxis::Y), mark).unwrap_err();
    assert!(e.root_message().contains("along its edge"), "{e:?}");
}

/// A short line level with the top on such a plane: a strip of the sweep lies
/// on the top, reaching neither of its sides it runs along.
#[test]
fn a_strip_of_a_sweep_in_the_plane_of_the_face_is_refused() {
    let block = block();
    let top = top_of(&block);
    let mut mark = Sketch::new();
    line(&mut mark, [-0.5, -1.0], [0.5, -1.0]);
    let e = split_all(&block, &[top], on_origin(FrameAxis::Y), mark).unwrap_err();
    assert!(e.root_message().contains("along its edge"), "{e:?}");
}

/// A line on a sphere's octant, clear of its pole: part of a small circle
/// from edge to edge.
#[test]
fn a_line_across_a_patch_of_a_sphere() {
    let mut part = NamedModel::new();
    sphere_solid(&mut part, "s", v3(0.0, 0.0, 0.0), Scalar::from_f64(1.0)).unwrap();
    let on = 1.0 / 3f64.sqrt();
    let face = face_at(&part, [on, on, on]);
    let mut mark = Sketch::new();
    line(&mut mark, [0.5, -3.0], [0.5, 3.0]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::Z), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// A radial line on a sphere's octant, from the axis out: a meridian ending
/// on the pole, where the octant's loop passes along its collapsed row.
///
/// The sheet's boundary crossing on the octant `sphere(s,north,q0)` comes back
/// from `splice_edge_into_face` as a "degenerate split": the traced edge's
/// pcurve is `0.15` wide all along, reaching outside the domain in `v`, where
/// the 3-D curve is `5e-6` wide. So the width is in fitting the pcurve from
/// the pole, whose `u` is free, not in anything split face chose: the sweep is
/// a plane through the axis, and the trace from the pole runs along a meridian
/// (16 steps). Not traced further: known to fail in `remesh`, on the copy of
/// the face, before anything of this operation.
#[test]
#[ignore = "fails in remesh: a curve traced from a sphere's pole fits a pcurve 0.15 wide; split face refuses nothing here"]
fn a_meridian_from_a_pole_of_a_sphere() {
    let mut part = NamedModel::new();
    sphere_solid(&mut part, "s", v3(0.0, 0.0, 0.0), Scalar::from_f64(1.0)).unwrap();
    let on = 1.0 / 3f64.sqrt();
    let face = face_at(&part, [on, on, on]);
    let mut mark = Sketch::new();
    line(&mut mark, [0.0, 0.0], [2.0, 1.0]);
    let after = split_all(&part, &[face], on_origin(FrameAxis::Z), mark).unwrap();
    assert_only_divided(&part, &after, 1);
}

/// What the top is divided into by a line, by name: the piece split off along
/// the line is named after the step, the face and the line; the rest keeps the
/// face's name, and the line's edge is named after the step and the line.
#[test]
fn a_piece_and_its_edge_are_named_after_the_curve() {
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    let part = top_divided(mark, 1);
    assert!(part.face_id("cube(b,start)").is_ok());
    assert!(part.face_id("split_face(s0,cube(b,start),mark,c2)").is_ok());
    assert!(part.edge_id("split_face(s0,mark,c2)").is_ok());
    // The piece is traced back to the face, as a later step reads it.
    assert_eq!(
        origin_of("split_face(s0,cube(b,start),mark,c2)"),
        Some("cube(b,start)")
    );
}

/// A curve crossing the face in several stretches: an edge and a piece per
/// stretch, each named by which it is.
#[test]
fn stretches_of_one_curve_are_counted() {
    let mut mark = Sketch::new();
    circle(&mut mark, [0.0, 0.0], 1.2);
    let part = top_divided(mark, 4);
    // Each of the circle's quarters crosses the top in one stretch, ending on
    // two of its sides round one corner.
    for q in ["c1", "c1#1", "c1#2", "c1#3"] {
        let piece = format!("split_face(s0,cube(b,start),mark,{q})");
        assert!(part.face_id(&piece).is_ok(), "{piece}");
    }

    // One line, through the hole of a drilled block's top round `(1, 1)`.
    let mut through = Sketch::new();
    line(&mut through, [1.1, -1.0], [1.1, 3.0]);
    let (_, part) = drilled_top(through).unwrap();
    let edges = |k: usize| format!("split_face(s0,mark,c2,{k})");
    assert!(part.edge_id(&edges(0)).is_ok(), "{:?}", edges(0));
    assert!(part.edge_id(&edges(1)).is_ok(), "{:?}", edges(1));
}

/// A second step on a piece the first made, and on what kept the name: the
/// line of the second ends on the edge of the first.
#[test]
fn a_piece_divided_again() {
    let block = block();
    let mut built = given_part(&block).unwrap();
    let z = on_origin(FrameAxis::Z);
    let sketches = [
        ("across", [[0.3, -2.0], [0.3, 2.0]]),
        ("back", [[-2.0, 0.5], [0.3, 0.5]]),
    ];
    for (name, [a, b]) in sketches {
        let mut mark = Sketch::new();
        line(&mut mark, a, b);
        run_step::<AddSketch>(&mut built, name, &AddSketchArgs::on(z.clone(), mark)).unwrap();
    }
    let one = SplitFaceArgs {
        face: Some(ElementReference::new(model(), "cube(b,start)")),
        sketch: whole_sketch("across"),
        along: Along::SketchNormal,
    };
    run_step::<SplitFace>(&mut built, "one", &one).unwrap();
    // The line ending at `x = 0.3`, on the first line: in whichever piece
    // lies left of it.
    let left = {
        let part = model_of(&built);
        let piece = "split_face(one,cube(b,start),across,c2)";
        let on_left = |name: &str| {
            let face = part.face_id(name).unwrap();
            let topology = part.topology();
            topology.face_edges(face).any(|e| {
                let start = topology.get_edge(e).unwrap().start_vertex;
                topology.get_vertex(start).unwrap().point[0].to_f64() < 0.0
            })
        };
        if on_left(piece) {
            piece
        } else {
            "cube(b,start)"
        }
    };
    let two = SplitFaceArgs {
        face: Some(ElementReference::new(model(), left)),
        sketch: whole_sketch("back"),
        along: Along::SketchNormal,
    };
    run_step::<SplitFace>(&mut built, "two", &two).unwrap();
    assert_only_divided(&block, &model_of(&built), 2);
}

/// A face standing on its own is divided the same.
#[test]
fn a_face_standing_on_its_own() {
    let block = block();
    let mut built = given_part(&block).unwrap();
    let wall = ElementReference::new(model(), "cube(b,start)");
    run_step::<geop_ops_edit::ExtractFace>(
        &mut built,
        "x",
        &geop_ops_edit::ExtractFaceArgs { face: wall },
    )
    .unwrap();
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    let z = AddSketchArgs::on(on_origin(FrameAxis::Z), mark);
    run_step::<AddSketch>(&mut built, "mark", &z).unwrap();
    let args = SplitFaceArgs {
        face: Some(ElementReference::new(model(), "extract(x,cube(b,start))")),
        sketch: whole_sketch("mark"),
        along: Along::SketchNormal,
    };
    run_step::<SplitFace>(&mut built, "s", &args).unwrap();
    let part = model_of(&built);
    assert_valid_sheets(&part);
    assert_eq!(part.topology().faces.len(), 8);
}

/// A line of a sketch beside a cylinder's quarter, reaching past the patch
/// and projected along the face's own normal: its points past the patch have
/// no foot point on it, and it is refused saying so, naming the curve.
#[test]
fn a_line_past_the_surface_along_the_face_normal_is_refused() {
    let mut part = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut part, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    let face = face_at(&part, [-2.0 + 0.894, 0.447, 1.0]);
    let mut built = given_part(&part).unwrap();
    let mut mark = Sketch::new();
    line(&mut mark, [-3.0, 0.5], [3.0, 1.5]);
    let on_x = AddSketchArgs::on(on_origin(FrameAxis::X), mark);
    run_step::<AddSketch>(&mut built, "mark", &on_x).unwrap();
    let args = SplitFaceArgs {
        face: Some(ElementReference::new(model(), face)),
        sketch: whole_sketch("mark"),
        along: Along::FaceNormal,
    };
    let e = run_step::<SplitFace>(&mut built, "s", &args).unwrap_err();
    assert!(e.root_message().contains("no foot point"), "{e:?}");
    assert!(e.to_string().contains("mark,c2"), "{e:?}");
}

/// A square over a patch of a cylinder, projected along the face's own normal.
#[test]
fn a_square_along_the_face_normal() {
    let mut part = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut part, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    let face = face_at(&part, [-2.0 + 0.894, 0.447, 1.0]);
    let mut built = given_part(&part).unwrap();
    let mut mark = Sketch::new();
    polygon(
        &mut mark,
        &[[0.9, 0.8], [1.1, 0.85], [1.05, 1.2], [0.85, 1.15]],
    );
    let on_x = AddSketchArgs::on(on_origin(FrameAxis::X), mark);
    run_step::<AddSketch>(&mut built, "mark", &on_x).unwrap();
    let args = SplitFaceArgs {
        face: Some(ElementReference::new(model(), face)),
        sketch: whole_sketch("mark"),
        along: Along::FaceNormal,
    };
    run_step::<SplitFace>(&mut built, "s", &args).unwrap();
    assert_only_divided(&part, &model_of(&built), 1);
}

/// A square of level and upright sides over that patch, along the face's own
/// normal. Its slanted cousin above builds; here the proof of the foot points
/// over the first stretch of a level side, `rail_miss` through
/// `NurbSurface3D::foot_moving`, finds none (`over` is none, the middle's
/// foot is proven, the seed is right): the target's height is exactly
/// constant. Not traced further into `krawczyk_foot`; this is the projection
/// along the face's normal alone, not the sketch's.
#[test]
#[ignore = "the foot point proof fails over a level side of a sketch: no enclosure from krawczyk_foot"]
fn a_level_square_along_the_face_normal() {
    let mut part = NamedModel::new();
    let f = Scalar::from_f64;
    revolved_cylinder(&mut part, "c", v3(-2.0, 0.0, 0.0), f(1.0), f(2.0)).unwrap();
    let face = face_at(&part, [-2.0 + 0.894, 0.447, 1.0]);
    let mut built = given_part(&part).unwrap();
    let mut mark = Sketch::new();
    polygon(&mut mark, &[[0.9, 0.8], [1.1, 0.8], [1.1, 1.2], [0.9, 1.2]]);
    let on_x = AddSketchArgs::on(on_origin(FrameAxis::X), mark);
    run_step::<AddSketch>(&mut built, "mark", &on_x).unwrap();
    let args = SplitFaceArgs {
        face: Some(ElementReference::new(model(), face)),
        sketch: whole_sketch("mark"),
        along: Along::FaceNormal,
    };
    run_step::<SplitFace>(&mut built, "s", &args).unwrap();
    assert_only_divided(&part, &model_of(&built), 1);
}

/// The pieces are real faces: a fillet runs along the piece of an edge one
/// borders, up to where the edge was split.
#[test]
fn a_fillet_along_a_piece_of_an_edge() {
    let block = block();
    let mut mark = Sketch::new();
    line(&mut mark, [0.3, -2.0], [0.3, 2.0]);
    let part = top_divided(mark, 1);
    let mut built = given_part(&part).unwrap();
    let piece = (part.topology().edges.keys())
        .map(|&e| part.name_of(e).unwrap().to_string())
        .find(|name| name.starts_with("split_face(s0,cube(b,c0,start)"))
        .expect("a piece of the top's side");
    let fillet = FilletArgs::constant(vec![piece], 0.1);
    run_step::<Fillet>(&mut built, "f", &fillet).unwrap();
    let after = model_of(&built);
    assert_valid(&after);
    assert!(after.topology().faces.len() > block.topology().faces.len() + 1);
}

/// The pieces are real faces: a wall divided level drafted above its parting
/// line alone, which stays where it is. The rebuild takes the other piece for
/// a smooth blend, tangent to the moved one along the line, which it refuses
/// (`geop_lib_shell::rebuild::tangent_to_kept`); the tilt about that line
/// would meet it there.
#[test]
#[ignore = "refused by the face rebuild: a piece coplanar with the one moved reads as a smooth blend"]
fn a_draft_of_a_piece_of_a_wall_about_the_parting_line() {
    let block = block();
    let wall = {
        let solid = block.solid_id("cube(b)").unwrap();
        let wall = face_towards(&block, solid, [1.0, 0.0, 0.0]);
        block.name_of(wall).unwrap().to_string()
    };
    let mut mark = Sketch::new();
    line(&mut mark, [-3.0, 0.5], [3.0, 0.5]);
    let part = split_all(
        &block,
        std::slice::from_ref(&wall),
        on_origin(FrameAxis::X),
        mark,
    )
    .unwrap();
    assert_only_divided(&block, &part, 1);
    let upper = [wall.clone(), format!("split_face(s0,{wall},mark,c2)")]
        .into_iter()
        .find(|name| {
            let topology = part.topology();
            let face = part.face_id(name).unwrap();
            topology.face_edges(face).any(|e| {
                let edge = topology.get_edge(e).unwrap();
                topology.get_vertex(edge.start_vertex).unwrap().point[2].to_f64() > 0.75
            })
        })
        .expect("the piece above the line");
    let mut built = given_part(&part).unwrap();
    let plane = GivenDatumArgs::plane([0.0, 0.0, 0.5], [0.0, 0.0, 1.0]);
    run_step::<GivenDatum>(&mut built, "parting", &plane).unwrap();
    let draft = DraftArgs {
        faces: vec![ElementReference::new(model(), upper)],
        neutral: Some(SketchPlane::Datum(datum("parting"))),
        angle: 5.0,
        reversed: false,
    };
    run_step::<Draft>(&mut built, "d", &draft).unwrap();
    let after = model_of(&built);
    assert_valid(&after);
    assert_eq!(
        after.topology().faces.len(),
        block.topology().faces.len() + 1
    );
}
