//! The operations on boxes built directly: validity, where the copies are,
//! how much they hold, what they are named, and what is refused.

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate},
};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Operation, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::shapes::cube_solid;
use geop_ops_rasterize::{rasterize, stl::stl_triangles};

use crate::{
    CircularPattern, CircularPatternArgs, Direction, LinearPattern, LinearPatternArgs, Mirror,
    MirrorArgs, MoveBody, MoveBodyArgs, Spacing,
};

fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

pub(crate) fn assert_valid(part: &Part<S>) {
    if let Err(errors) = validate(&ValidationParameters::default(), part.topology()) {
        let messages: Vec<String> = errors.iter().map(|e| format!("{e:?}")).collect();
        panic!("{} errors:\n{}", messages.len(), messages.join("\n"));
    }
    part.check_names().unwrap();
}

/// The volume the solid named `solid` encloses, from its mesh as drawn:
/// negative if its faces point in.
pub(crate) fn volume(part: &Part<S>, solid: &str) -> f64 {
    let id = part.solid_id(solid).unwrap();
    let faces = part.topology().body_faces(Body::Solid(id)).unwrap();
    let raster = rasterize(part.topology(), 16).unwrap();
    stl_triangles(&raster, &faces)
        .iter()
        .map(|t| {
            let [a, b, c] = t.corners.map(|p| p.map(f64::from));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

/// The lowest corner of the solid named `solid`, by midpoints.
fn min_corner(part: &Part<S>, solid: &str) -> [f64; 3] {
    let id = part.solid_id(solid).unwrap();
    let model = part.topology();
    let mut min = [f64::INFINITY; 3];
    for vertex in model.iter_body_vertices(id).unwrap() {
        let p = model.get_vertex(vertex).unwrap().point;
        for k in 0..3 {
            min[k] = min[k].min(p[k].to_f64());
        }
    }
    min
}

fn close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9)
}

/// A unit cube at `min`, named `cube(c)`.
fn cube_at(min: [f64; 3]) -> Part<S> {
    let mut part = Part::new();
    cube_solid(
        &mut part,
        "c",
        v3(min[0], min[1], min[2]),
        v3(min[0] + 1.0, min[1] + 1.0, min[2] + 1.0),
    )
    .unwrap();
    part
}

fn cube() -> Vec<EntityRef> {
    vec![EntityRef::Solid {
        name: "cube(c)".into(),
    }]
}

fn axis(axis: FrameAxis) -> Option<EntityRef> {
    Some(EntityRef::datum_component(
        ORIGIN,
        DatumComponent::Axis(axis),
    ))
}

fn along(a: FrameAxis, count: usize, spacing: Spacing) -> Direction {
    Direction {
        along: axis(a),
        reversed: false,
        count,
        spacing,
    }
}

/// Three cubes in a row, two apart: two copies, each a solid of its own
/// named after the copy, two and four along `x`.
#[test]
fn row_of_new_bodies() {
    let args = LinearPatternArgs {
        bodies: cube(),
        first: along(FrameAxis::X, 3, Spacing::Step(2.0)),
        second: None,
        combine: Combine::NewBody,
    };
    let part = LinearPattern
        .apply(cube_at([0.0; 3]), "p", &args, &NoFiles)
        .unwrap();
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
    assert!((volume(&part, "linear_pattern(p,1,cube(c))") - 1.0).abs() < 1e-6);
}

/// Spread over a total, reversed: four over six against `y`, two apart.
#[test]
fn row_over_a_total_reversed() {
    let mut first = along(FrameAxis::Y, 4, Spacing::Extent(6.0));
    first.reversed = true;
    let args = LinearPatternArgs {
        bodies: cube(),
        first,
        second: None,
        combine: Combine::NewBody,
    };
    let part = LinearPattern
        .apply(cube_at([0.0; 3]), "p", &args, &NoFiles)
        .unwrap();
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
        bodies: cube(),
        first: along(FrameAxis::X, 3, Spacing::Step(0.5)),
        second: None,
        combine: Combine::Union {
            target: "cube(c)".into(),
        },
    };
    let part = LinearPattern
        .apply(cube_at([0.0; 3]), "p", &args, &NoFiles)
        .unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(p)"]);
    assert!((volume(&part, "linear_pattern(p)") - 2.0).abs() < 1e-6);
}

/// A grid of two by three, named `i.j`.
#[test]
fn grid_of_new_bodies() {
    let args = LinearPatternArgs {
        bodies: cube(),
        first: along(FrameAxis::X, 2, Spacing::Step(2.0)),
        second: Some(along(FrameAxis::Y, 3, Spacing::Step(1.5))),
        combine: Combine::NewBody,
    };
    let part = LinearPattern
        .apply(cube_at([0.0; 3]), "p", &args, &NoFiles)
        .unwrap();
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
        bodies: cube(),
        first: along(FrameAxis::X, 2, Spacing::Step(2.0)),
        second: Some(along(FrameAxis::X, 2, Spacing::Step(3.0))),
        combine: Combine::NewBody,
    };
    let error = LinearPattern
        .apply(cube_at([0.0; 3]), "p", &args, &NoFiles)
        .err()
        .expect("refused");
    assert!(error.root_message().contains("parallel"), "{error:?}");
}

/// Six cubes around the `z` axis, a full turn: each a valid solid, the
/// copy turned a half turn opposite the cube.
#[test]
fn full_turn_around_an_axis() {
    let args = CircularPatternArgs {
        bodies: cube(),
        axis: axis(FrameAxis::Z),
        reversed: false,
        count: 6,
        angle: Spacing::Extent(360.0),
        combine: Combine::NewBody,
    };
    let part = CircularPattern
        .apply(cube_at([2.0, -0.5, 0.0]), "r", &args, &NoFiles)
        .unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 6);
    assert!(close(
        min_corner(&part, "circular_pattern(r,3,cube(c))"),
        [-3.0, -0.5, 0.0]
    ));
    let turned = volume(&part, "circular_pattern(r,1,cube(c))");
    assert!((turned - 1.0).abs() < 1e-6, "{turned}");
}

/// Copies that would come round onto the cube are refused.
#[test]
fn coming_round_again_is_refused() {
    let args = CircularPatternArgs {
        bodies: cube(),
        axis: axis(FrameAxis::Z),
        reversed: false,
        count: 7,
        angle: Spacing::Step(60.0),
        combine: Combine::NewBody,
    };
    let error = CircularPattern
        .apply(cube_at([2.0, 0.0, 0.0]), "r", &args, &NoFiles)
        .err()
        .expect("refused");
    assert!(error.root_message().contains("come round"), "{error:?}");
}

/// Mirrored in the origin's `yz` plane, the cube's image is a valid solid
/// on the other side, facing out: its volume positive.
#[test]
fn mirrored_cube_faces_out() {
    let args = MirrorArgs {
        bodies: cube(),
        plane: Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Plane(FrameAxis::X),
        )),
        combine: Combine::NewBody,
    };
    let part = Mirror
        .apply(cube_at([1.0, 0.0, 0.0]), "m", &args, &NoFiles)
        .unwrap();
    assert_valid(&part);
    assert!(close(
        min_corner(&part, "mirror(m,cube(c))"),
        [-2.0, 0.0, 0.0]
    ));
    let image = volume(&part, "mirror(m,cube(c))");
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
    let part = MoveBody.apply(before, "mv", &args, &NoFiles).unwrap();
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
    let part = MoveBody
        .apply(cube_at([0.0; 3]), "mv", &args, &NoFiles)
        .unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["cube(c)", "move(mv,cube(c))"]);
    assert!(close(
        min_corner(&part, "move(mv,cube(c))"),
        [3.0, 0.0, 0.0]
    ));
    args.translation = [0.0; 3];
    assert!(
        MoveBody
            .apply(cube_at([0.0; 3]), "mv", &args, &NoFiles)
            .is_err()
    );
}

/// A face standing on its own can be copied, but not joined or cut.
#[test]
fn sheets_are_copied_not_combined() {
    let mut part = cube_at([0.0; 3]);
    let face = part.face_id("cube(c,end)").unwrap();
    part.copy_faces(&[face], None, |n| format!("x({n})"))
        .unwrap();
    let sheet = vec![EntityRef::Face {
        name: "x(cube(c,end))".into(),
    }];
    let mut args = LinearPatternArgs {
        bodies: sheet,
        first: along(FrameAxis::Z, 2, Spacing::Step(2.0)),
        second: None,
        combine: Combine::NewBody,
    };
    let copied = LinearPattern
        .apply(part.clone(), "p", &args, &NoFiles)
        .unwrap();
    assert_valid(&copied);
    assert!(copied.id_of("linear_pattern(p,1,x(cube(c,end)))").is_some());
    args.combine = Combine::Union {
        target: "cube(c)".into(),
    };
    let error = LinearPattern
        .apply(part, "p", &args, &NoFiles)
        .err()
        .expect("refused");
    assert!(error.root_message().contains("no solid"), "{error:?}");
}
