//! The operations on boxes built directly: validity, where the copies are,
//! how much they hold, what they are named, and what is refused.

use geop_core_math::{
    geop_error::GeopResult, plain, primitives::FrameAxis, scalars::ScalInF64 as S,
};
use geop_core_program::operation::Operation;
use geop_ops_booleans::{Combine, solid};
use geop_ops_datums::{origin_axis, origin_plane};
use geop_ops_extrude_revolve::shapes::block_part;
use geop_ops_model::{
    BooleanOp, NamedModel, read_model,
    testing::{assert_valid, assert_valid_sheets, given_part, volume},
};
use geop_ops_sketch::SketchPlane;

use crate::{
    BodyRef, CircularPattern, CircularPatternArgs, Direction, LinearPattern, LinearPatternArgs,
    Mirror, MirrorArgs, MoveBody, MoveBodyArgs, PatternDirection, Seeds, Spacing, TurnAxis, body,
};

/// What `O`'s step `step` with `args` makes of `model`, given with the
/// datums, as a program of two steps would build it.
fn apply_to_model<O: Operation>(
    model: NamedModel<S>,
    step: &str,
    args: &O::Args,
) -> GeopResult<NamedModel<S>> {
    let mut part = given_part(&model)?;
    part.begin_step();
    let runner = geop_core_program::runner::Runner::new(Default::default());
    O::run(step, &runner, &mut part, args)?;
    part.end_run(0);
    Ok(read_model(&part.current())?.into_owned())
}

/// The lowest corner of the solid named `solid`, by midpoints.
fn min_corner(part: &NamedModel<S>, solid: &str) -> [f64; 3] {
    let id = part.solid_id(solid).unwrap();
    let model = part.topology();
    let corners = model.iter_body_vertices(id).unwrap();
    plain::bounds(corners.map(|vertex| model.get_vertex(vertex).unwrap().point.to_f64_array()))
        .unwrap()
        .0
}

fn close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9)
}

/// A unit cube at `min`, named `cube(c)`.
fn cube_at(min: [f64; 3]) -> NamedModel<S> {
    block_part("c", min, min.map(|x| x + 1.0))
}

fn cube() -> Vec<BodyRef> {
    vec![body("cube(c)")]
}

fn axis(axis: FrameAxis) -> Option<TurnAxis> {
    Some(TurnAxis::Datum(origin_axis(axis)))
}

fn along(a: FrameAxis, count: usize, spacing: Spacing) -> Direction {
    Direction {
        along: Some(PatternDirection::Datum(origin_axis(a))),
        reversed: false,
        count: (count as f64).into(),
        spacing,
    }
}

/// Three cubes in a row, two apart: two copies, each a solid of its own
/// named after the copy, two and four along `x`.
#[test]
fn row_of_new_bodies() {
    let args = LinearPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        first: along(FrameAxis::X, 3, Spacing::step(2.0)),
        second: None,
    };
    let part = apply_to_model::<LinearPattern>(cube_at([0.0; 3]), "p", &args).unwrap();
    assert_valid(&part);
    assert_eq!(
        part.solid_names(),
        [
            "cube(c)",
            "linear_pattern(p,1,cube(c))",
            "linear_pattern(p,2,cube(c))"
        ]
    );
    assert!(close(
        min_corner(&part, "linear_pattern(p,2,cube(c))"),
        [4.0, 0.0, 0.0]
    ));
    assert!(part.id_of("linear_pattern(p,1,cube(c,end))").is_some());
    assert!(
        (volume(
            &part,
            part.solid_id("linear_pattern(p,1,cube(c))").unwrap(),
            16
        ) - 1.0)
            .abs()
            < 1e-6
    );
}

/// Spread over a total, reversed: four over six against `y`, two apart.
#[test]
fn row_over_a_total_reversed() {
    let mut first = along(FrameAxis::Y, 4, Spacing::extent(6.0));
    first.reversed = true;
    let args = LinearPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        first,
        second: None,
    };
    let part = apply_to_model::<LinearPattern>(cube_at([0.0; 3]), "p", &args).unwrap();
    assert_valid(&part);
    assert!(close(
        min_corner(&part, "linear_pattern(p,3,cube(c))"),
        [0.0, -6.0, 0.0]
    ));
}

/// Copies half a cube apart overlap the cube and each other: joined to the
/// cube, they make one bar two long, named after the step.
#[test]
fn overlapping_copies_joined_to_the_seed() {
    let args = LinearPatternArgs {
        seeds: Seeds::bodies(
            cube(),
            Combine::With {
                op: BooleanOp::Union,
                target: solid("cube(c)"),
            },
        ),
        first: along(FrameAxis::X, 3, Spacing::step(0.5)),
        second: None,
    };
    let part = apply_to_model::<LinearPattern>(cube_at([0.0; 3]), "p", &args).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(p)"]);
    assert!((volume(&part, part.solid_id("linear_pattern(p)").unwrap(), 16) - 2.0).abs() < 1e-6);
}

/// A grid of two by three, named `i.j`.
#[test]
fn grid_of_new_bodies() {
    let args = LinearPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        first: along(FrameAxis::X, 2, Spacing::step(2.0)),
        second: Some(along(FrameAxis::Y, 3, Spacing::step(1.5))),
    };
    let part = apply_to_model::<LinearPattern>(cube_at([0.0; 3]), "p", &args).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 6);
    assert!(close(
        min_corner(&part, "linear_pattern(p,1.2,cube(c))"),
        [2.0, 3.0, 0.0]
    ));
    assert!(close(
        min_corner(&part, "linear_pattern(p,0.1,cube(c))"),
        [0.0, 1.5, 0.0]
    ));
}

/// Two parallel directions would stack the grid's rows: refused.
#[test]
fn parallel_grid_is_refused() {
    let args = LinearPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        first: along(FrameAxis::X, 2, Spacing::step(2.0)),
        second: Some(along(FrameAxis::X, 2, Spacing::step(3.0))),
    };
    let error =
        apply_to_model::<LinearPattern>(cube_at([0.0; 3]), "p", &args).expect_err("refused");
    assert!(error.root_message().contains("parallel"), "{error:?}");
}

/// Six cubes around the `z` axis, a full turn: each a valid solid, the
/// copy turned a half turn opposite the cube.
#[test]
fn full_turn_around_an_axis() {
    let args = CircularPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        axis: axis(FrameAxis::Z),
        reversed: false,
        count: 6.0.into(),
        angle: Spacing::extent(360.0),
    };
    let part = apply_to_model::<CircularPattern>(cube_at([2.0, -0.5, 0.0]), "r", &args).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 6);
    assert!(close(
        min_corner(&part, "circular_pattern(r,3,cube(c))"),
        [-3.0, -0.5, 0.0]
    ));
    let turned = volume(
        &part,
        part.solid_id("circular_pattern(r,1,cube(c))").unwrap(),
        16,
    );
    assert!((turned - 1.0).abs() < 1e-6, "{turned}");
}

/// Copies that would come round onto the cube are refused.
#[test]
fn coming_round_again_is_refused() {
    let args = CircularPatternArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        axis: axis(FrameAxis::Z),
        reversed: false,
        count: 7.0.into(),
        angle: Spacing::step(60.0),
    };
    let error = apply_to_model::<CircularPattern>(cube_at([2.0, 0.0, 0.0]), "r", &args)
        .expect_err("refused");
    assert!(error.root_message().contains("come round"), "{error:?}");
}

/// Mirrored in the origin's `yz` plane, the cube's image is a valid solid
/// on the other side, facing out: its volume positive.
#[test]
fn mirrored_cube_faces_out() {
    let args = MirrorArgs {
        seeds: Seeds::bodies(cube(), Combine::NewBody),
        plane: Some(SketchPlane::Datum(origin_plane(FrameAxis::X))),
    };
    let part = apply_to_model::<Mirror>(cube_at([1.0, 0.0, 0.0]), "m", &args).unwrap();
    assert_valid(&part);
    assert!(close(
        min_corner(&part, "mirror(m,cube(c))"),
        [-2.0, 0.0, 0.0]
    ));
    let image = volume(&part, part.solid_id("mirror(m,cube(c))").unwrap(), 16);
    assert!((image - 1.0).abs() < 1e-6, "{image}");
}

/// Moved, the cube keeps every name; turned a quarter about `z` and
/// shifted, it lands where the motion says.
#[test]
fn moved_body_keeps_its_names() {
    let before = cube_at([1.0, 0.0, 0.0]);
    let mut names: Vec<String> = before.names().iter().map(|(_, n)| n.into()).collect();
    names.sort();
    let args = MoveBodyArgs {
        bodies: cube(),
        translation: [0.0, 0.0, 5.0],
        axis: axis(FrameAxis::Z),
        angle: 90.0,
        copy: false,
        combine: Combine::NewBody,
    };
    let part = apply_to_model::<MoveBody>(before, "mv", &args).unwrap();
    assert_valid(&part);
    let mut after: Vec<String> = part.names().iter().map(|(_, n)| n.into()).collect();
    after.sort();
    assert_eq!(names, after);
    assert!(close(min_corner(&part, "cube(c)"), [-1.0, 1.0, 5.0]));
}

/// Copied, the cube stays and the copy is named after the step; a copy
/// that is not moved is refused.
#[test]
fn moved_copy() {
    let mut args = MoveBodyArgs {
        bodies: cube(),
        translation: [3.0, 0.0, 0.0],
        axis: None,
        angle: 0.0,
        copy: true,
        combine: Combine::NewBody,
    };
    let part = apply_to_model::<MoveBody>(cube_at([0.0; 3]), "mv", &args).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["cube(c)", "move(mv,cube(c))"]);
    assert!(close(
        min_corner(&part, "move(mv,cube(c))"),
        [3.0, 0.0, 0.0]
    ));
    args.translation = [0.0; 3];
    assert!(apply_to_model::<MoveBody>(cube_at([0.0; 3]), "mv", &args).is_err());
}

/// A face standing on its own can be copied, but not joined or cut.
#[test]
fn sheets_are_copied_not_combined() {
    let mut part = cube_at([0.0; 3]);
    let face = part.face_id("cube(c,end)").unwrap();
    part.copy_faces(&[face], None, |n| format!("x({n})"))
        .unwrap();
    let sheet = vec![body("x(cube(c,end))")];
    let mut args = LinearPatternArgs {
        seeds: Seeds::bodies(sheet, Combine::NewBody),
        first: along(FrameAxis::Z, 2, Spacing::step(2.0)),
        second: None,
    };
    let copied = apply_to_model::<LinearPattern>(part.clone(), "p", &args).unwrap();
    assert_valid_sheets(&copied);
    assert!(copied.id_of("linear_pattern(p,1,x(cube(c,end)))").is_some());
    args.seeds.combine = Combine::With {
        op: BooleanOp::Union,
        target: solid("cube(c)"),
    };
    let error = apply_to_model::<LinearPattern>(part, "p", &args).expect_err("refused");
    assert!(error.root_message().contains("no solid"), "{error:?}");
}
