//! Patterns of features: a hole cut by an extrude, a tapped hole with its
//! thread, a boss joined to a plate, a hole going up to the next face —
//! each done again at every instance of a linear or circular pattern, as
//! the step that made it did it.

use std::f64::consts::PI;

use geop_core_math::primitives::FrameAxis;
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_math::vector::Vector3;
use geop_ops::{EntityRef, NamedModel};
use geop_ops_extrude_revolve::{Extent, Extents};
use geop_ops_hole::{HoleArgs, HoleKind, iso::Fit};
use geop_ops_pattern::{
    CircularPatternArgs, Direction, LinearPatternArgs, MirrorArgs, Seeds, Spacing,
};
use geop_ops_sketch::AddSketchArgs;

use super::hole_tests::{Made, hole, iso, made, points_sketch};
use super::pattern_tests::{axis, extruded};
use super::testing::{
    assert_valid, assert_volume, both_ways, cut, disc, extrude_sketch, extrusion, join,
    origin_plane, outline,
};
use crate::operations::testing::Built;
use crate::{Program, Push};

/// `program`, read back from its JSON as saved, built.
fn build(program: &Program) -> Made {
    let json = serde_json::to_string(program).unwrap();
    let read: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(&read, program);
    made(&read).unwrap()
}

fn feature(step: &str) -> Vec<EntityRef> {
    vec![EntityRef::Feature { name: step.into() }]
}

/// `count` instances along `x`, `step` apart.
fn along_x(count: f64, step: f64) -> Direction {
    Direction {
        along: axis(FrameAxis::X),
        reversed: false,
        count: count.into(),
        spacing: Spacing::step(step),
    }
}

/// A linear pattern of the features `features`, `count` along `x`.
fn linear(features: &[&str], count: f64, step: f64) -> LinearPatternArgs {
    LinearPatternArgs {
        seeds: Seeds::features(
            features
                .iter()
                .map(|f| EntityRef::Feature {
                    name: f.to_string(),
                })
                .collect(),
        ),
        first: along_x(count, step),
        second: None,
    }
}

/// A plate 8 x 2 x 0.5, `extrude(plate)`, with a hole of radius 0.3 at
/// (1, 1) cut through it by the extrude `hole`.
pub(super) fn plate_with_hole() -> Program {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    extrude_sketch(
        &mut program,
        "hole",
        origin_plane(FrameAxis::Z),
        disc([1.0, 1.0], 0.3),
        both_ways(2.0),
        cut("extrude(plate)"),
    );
    program
}

/// The faces of `part` the feature of `step` made, as a pick tells.
fn faces_of(part: &NamedModel<S>, step: &str) -> Vec<String> {
    let made = part.feature_faces();
    let mut faces: Vec<String> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|name| made.of(name) == Some(step))
        .map(str::to_string)
        .collect();
    faces.sort();
    faces
}

/// A plate 8 x 2 x 0.5 with a hole of radius 0.3 at (1, 1), cut by an
/// extrude — the feature `hole` — done again four times two apart: a row
/// of four holes, the plate one solid, named after the pattern. The copy's
/// wall is the pattern's; the original's still the hole's.
#[test]
fn extruded_hole_patterned_in_a_row() {
    let mut program = plate_with_hole();
    program.push("holes", linear(&["hole"], 4.0, 2.0));
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
    let expected = 8.0 * 2.0 * 0.5 - 4.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "linear_pattern(holes)", expected, 1e-3);

    let hole = faces_of(&part, "hole");
    assert!(!hole.is_empty(), "the hole's wall is the hole's");
    let copies = faces_of(&part, "holes");
    assert!(
        copies
            .iter()
            .any(|f| f.contains("linear_pattern(holes,3,extrude(hole,hole_sketch,c")),
        "{copies:?}"
    );
    // The plate's own faces are no feature's.
    assert!(faces_of(&part, "plate").is_empty());
}

/// The same hole in a square plate, done again six times around the
/// plate's middle: a bolt circle.
#[test]
fn extruded_hole_patterned_around_an_axis() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]]),
        Extents::blind(0.5),
    );
    extrude_sketch(
        &mut program,
        "hole",
        origin_plane(FrameAxis::Z),
        disc([2.0, 0.0], 0.3),
        both_ways(2.0),
        cut("extrude(plate)"),
    );
    program.push(
        "bolts",
        CircularPatternArgs {
            seeds: Seeds::features(feature("hole")),
            axis: axis(FrameAxis::Z),
            reversed: false,
            count: 6.0.into(),
            angle: Spacing::extent(360.0),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["circular_pattern(bolts)"]);
    let expected = 36.0 * 0.5 - 6.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "circular_pattern(bolts)", expected, 1e-3);
}

/// The row of holes mirrored in the `yz` plane, as a feature of its own:
/// the plate, reaching to `x = -8`, drilled four times on either side.
/// The pattern is a feature like any other: its copies are its tools.
#[test]
fn pattern_of_holes_mirrored() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[-8.0, 0.0], [8.0, 0.0], [8.0, 2.0], [-8.0, 2.0]]),
        Extents::blind(0.5),
    );
    extrude_sketch(
        &mut program,
        "hole",
        origin_plane(FrameAxis::Z),
        disc([1.0, 1.0], 0.3),
        both_ways(2.0),
        cut("extrude(plate)"),
    );
    program.push("holes", linear(&["hole"], 4.0, 2.0));
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::features(
                feature("hole")
                    .into_iter()
                    .chain(feature("holes"))
                    .collect(),
            ),
            plane: Some(origin_plane(FrameAxis::X)),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["mirror(m)"]);
    let expected = 16.0 * 2.0 * 0.5 - 8.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "mirror(m)", expected, 1e-3);
}

/// A boss of radius 0.3 joined onto the plate's top, done again three
/// times: three bosses on one solid.
#[test]
fn boss_patterned_in_a_row() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "extrude(plate,end)".into(),
            }),
            sketch: disc([1.0, 1.0], 0.3),
            ..Default::default()
        },
    );
    program.push(
        "boss",
        extrusion("boss_sketch", Extents::blind(0.5), join("extrude(plate)")),
    );
    program.push("bosses", linear(&["boss"], 3.0, 2.5));
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(bosses)"]);
    let expected = 8.0 * 2.0 * 0.5 + 3.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "linear_pattern(bosses)", expected, 1e-3);
}

fn simple_m6(face: &str, end: Extent) -> HoleArgs {
    HoleArgs {
        face: face.into(),
        ..hole(HoleKind::Simple, iso("M6", Fit::Normal), end)
    }
}

/// A tapped M6 hole, 8 deep, in the hole tests' plate, done again three
/// times 15 apart: each copy drilled to the tap drill and carrying a copy
/// of the thread, named after the pattern, on its axis.
#[test]
fn tapped_hole_patterned_with_its_thread() {
    let mut program = super::hole_tests::plate(&[[10.0, 20.0]]);
    let mut tapped = simple_m6("extrude(plate,end)", Extent::blind(8.0));
    tapped.kind = HoleKind::Tapped;
    program.push("h", tapped);
    program.push("p", linear(&["h"], 3.0, 15.0));
    let part = build(&program);
    assert_valid(&part);
    let removed = 3.0 * PI * 2.5 * 2.5 * 8.0;
    let expected = 60.0 * 40.0 * 10.0 - removed;
    assert_volume(&part, "linear_pattern(p)", expected, 5e-3 * removed);
    let v = |c: [f64; 3]| Vector3::from_array(c.map(S::from_f64));
    let original = part.thread("hole(h,centres,p0,thread)").unwrap();
    for (i, x) in [(1, 25.0), (2, 40.0)] {
        let name = format!("linear_pattern(p,{i},hole(h,centres,p0,thread))");
        let thread = part.thread(&name).unwrap();
        assert_eq!(thread.designation, "M6x1");
        assert_eq!(thread.length, original.length);
        assert!(
            thread.axis.point.could_be_equal(&v([x, 20.0, 10.0])),
            "{name}: {:?}",
            thread.axis.point
        );
        assert!(thread.axis.direction.could_be_equal(&v([0.0, 0.0, -1.0])));
        assert!(
            thread
                .face
                .starts_with(&format!("linear_pattern(p,{i},hole(h,centres,p0,wall")),
            "{}",
            thread.face
        );
    }
    assert_eq!(part.threads.iter().count(), 3);
}

/// The copies of a patterned thread are drawn as the pattern's own
/// annotations, as the hole draws its thread: every thread of the part is
/// shown, once.
#[test]
fn a_patterned_thread_is_drawn() {
    let mut program = super::hole_tests::plate(&[[10.0, 20.0]]);
    let mut tapped = simple_m6("extrude(plate,end)", Extent::blind(8.0));
    tapped.kind = HoleKind::Tapped;
    program.push("h", tapped);
    program.push("p", linear(&["h"], 3.0, 15.0));
    let runner = program.built().unwrap();
    let drawn = geop_ops::ui::annotations_of(runner.part.current());
    let mut names: Vec<&str> = drawn.iter().map(|a| a.name.as_str()).collect();
    names.sort();
    assert_eq!(
        names,
        [
            "hole(h,centres,p0,thread)",
            "linear_pattern(p,1,hole(h,centres,p0,thread))",
            "linear_pattern(p,2,hole(h,centres,p0,thread))",
        ]
    );
    assert!(
        drawn
            .iter()
            .all(|a| a.label == "M6x1" && a.polyline.len() > 2)
    );
}

/// A hole drilled up from the bottom of a stepped block, up to the next
/// face — the step 10 high where it is drilled — done again where the
/// block is 20 high: the copy goes up to the next face where it is, all
/// 20, as a hole drilled there would.
#[test]
fn hole_up_to_next_patterned_stops_at_the_next_face_where_it_is() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "low",
        outline(&[[0.0, 0.0], [30.0, 0.0], [30.0, 40.0], [0.0, 40.0]]),
        Extents::blind(10.0),
    );
    extrude_sketch(
        &mut program,
        "high",
        origin_plane(FrameAxis::Z),
        outline(&[[30.0, 0.0], [60.0, 0.0], [60.0, 40.0], [30.0, 40.0]]),
        Extents::blind(20.0),
        join("extrude(low)"),
    );
    program.push(
        "centres",
        AddSketchArgs {
            plane: Some(origin_plane(FrameAxis::Z)),
            sketch: points_sketch(&[[15.0, 20.0]]),
            ..Default::default()
        },
    );
    program.push("h", simple_m6("extrude(low,start)", Extent::UpToNext));
    program.push("p", linear(&["h"], 2.0, 30.0));
    let part = build(&program);
    assert_valid(&part);
    let area = PI * 3.3 * 3.3;
    let removed = area * 10.0 + area * 20.0;
    let expected = 30.0 * 40.0 * 10.0 + 30.0 * 40.0 * 20.0 - removed;
    assert_volume(&part, "linear_pattern(p)", expected, 5e-3 * removed);
}

/// Bodies and features at once, a body picked as a feature, and a step
/// that made no feature are refused, saying so.
#[test]
fn what_is_no_feature_is_refused() {
    let program = plate_with_hole();
    let refused = |args: LinearPatternArgs| {
        let mut program = program.clone();
        program.push("p", args);
        program.build_model().err().unwrap().to_string()
    };
    let mut both = linear(&["hole"], 2.0, 2.0);
    both.seeds.bodies = vec![EntityRef::Solid {
        name: "extrude(hole)".into(),
    }];
    assert!(refused(both).contains("one or the other"));
    let error = refused(linear(&["plate"], 2.0, 2.0));
    assert!(error.contains("is no feature"), "{error}");
    let mut solid = linear(&[], 2.0, 2.0);
    solid.seeds.features = vec![EntityRef::Solid {
        name: "extrude(hole)".into(),
    }];
    let error = refused(solid);
    assert!(error.contains("is no feature"), "{error}");
}
