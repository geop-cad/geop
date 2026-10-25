//! Patterns of features: a hole cut by a cylinder, a tapped hole with its
//! thread, a boss joined to a plate, a hole going up to the next face —
//! each done again at every instance of a linear or circular pattern, as
//! the step that made it did it.

use geop_core_program::{
    element::{ElementReference, PickedElement},
    program::Program,
};
use std::f64::consts::PI;

use crate::{
    HoleArgs, HoleKind,
    iso::Fit,
    program_tests::{Made, hole, iso, made, named, registry},
    testing::{PLATE_TOP, plate, points_sketch},
};
use geop_core_editor::selectable::{Selectable, selected};
use geop_core_math::{primitives::FrameAxis, scalars::Scalar, vector::Vector3};
use geop_lib_fixtures::{PrimitiveArgs, drilled_plate};
use geop_ops_model::datums::{origin_axis, origin_plane};
use geop_ops_model::sketch::{SketchPlane, figures::on_origin};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Extent, Face, NamedModel, model,
    testing::{Built, assert_valid, assert_volume},
};
use geop_ops_pattern::{
    CircularPatternArgs, Direction, FeatureRef, LinearPatternArgs, MirrorArgs, PatternDirection,
    Seeds, TurnAxis, body, feature,
};
use geop_ops_sketch::AddSketchArgs;

/// A plate `block(plate)` 0.5 thick on the Z plane over `[x0, x1] x [y0,
/// y1]`.
fn plate_over(program: &mut Program, [x0, y0]: [f64; 2], [x1, y1]: [f64; 2]) {
    program.push("plate", PrimitiveArgs::block([x0, y0, 0.0], [x1, y1, 0.5]));
}

/// A cylinder of radius 0.3 around `(x, y)` through the plate, cut from
/// it: the feature `hole`.
fn hole_through_plate(program: &mut Program, [x, y]: [f64; 2]) {
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([x, y, -1.0], 0.3, 2.5).cut("block(plate)"),
    );
}

/// `program`, read back from its JSON as saved, built.
fn build(program: &Program) -> Made {
    let json = serde_json::to_string(program).unwrap();
    let read: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(&read, program);
    made(&read).unwrap()
}

fn features(steps: &[&str]) -> Vec<FeatureRef> {
    steps.iter().map(|&step| feature(step)).collect()
}

/// `count` instances along `x`, `step` apart.
fn along_x(count: f64, step: f64) -> Direction {
    Direction {
        along: Some(PatternDirection::Datum(origin_axis(FrameAxis::X))),
        reversed: false,
        count: count.into(),
        spacing: Spacing::step(step),
    }
}

/// A linear pattern of the features `features`, `count` along `x`.
fn linear(features: &[&str], count: f64, step: f64) -> LinearPatternArgs {
    LinearPatternArgs {
        seeds: Seeds::features(self::features(features)),
        first: along_x(count, step),
        second: None,
    }
}

/// The faces of `part` the feature of `step` made, as a pick tells.
fn faces_of(part: &NamedModel, step: &str) -> Vec<String> {
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

/// A plate 8 x 2 x 0.5 with a hole of radius 0.3 at (1, 1), cut by a
/// cylinder — the feature `hole` — done again four times two apart: a row
/// of four holes, the plate one solid, named after the pattern. The copy's
/// wall is the pattern's; the original's still the hole's.
#[test]
fn cut_hole_patterned_in_a_row() {
    let mut program = drilled_plate();
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
            .any(|f| f.contains("linear_pattern(holes,3,cylinder(hole,c")),
        "{copies:?}"
    );
    // The plate's own faces are no feature's.
    assert!(faces_of(&part, "plate").is_empty());
}

/// The same hole in a square plate, done again six times around the
/// plate's middle: a bolt circle.
#[test]
fn cut_hole_patterned_around_an_axis() {
    let mut program = Program::new();
    plate_over(&mut program, [-3.0, -3.0], [3.0, 3.0]);
    hole_through_plate(&mut program, [2.0, 0.0]);
    program.push(
        "bolts",
        CircularPatternArgs {
            seeds: Seeds::features(features(&["hole"])),
            axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
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
    plate_over(&mut program, [-8.0, 0.0], [8.0, 2.0]);
    hole_through_plate(&mut program, [1.0, 1.0]);
    program.push("holes", linear(&["hole"], 4.0, 2.0));
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::features(features(&["hole", "holes"])),
            plane: Some(SketchPlane::Datum(origin_plane(FrameAxis::X))),
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
    plate_over(&mut program, [0.0, 0.0], [8.0, 2.0]);
    program.push(
        "boss",
        PrimitiveArgs::cylinder_z([1.0, 1.0, 0.5], 0.3, 0.5).join("block(plate)"),
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
        face: Some(named(face)),
        ..hole(HoleKind::Simple, iso("M6", Fit::Normal), end)
    }
}

/// A tapped M6 hole, 8 deep, in the hole tests' plate, done again three
/// times 15 apart: each copy drilled to the tap drill and carrying a copy
/// of the thread, named after the pattern, on its axis.
#[test]
fn tapped_hole_patterned_with_its_thread() {
    let mut program = plate(&[[10.0, 20.0]]);
    let mut tapped = simple_m6(PLATE_TOP, Extent::blind(8.0));
    tapped.kind = HoleKind::Tapped;
    program.push("h", tapped);
    program.push("p", linear(&["h"], 3.0, 15.0));
    let part = build(&program);
    assert_valid(&part);
    let removed = 3.0 * PI * 2.5 * 2.5 * 8.0;
    let expected = 60.0 * 40.0 * 10.0 - removed;
    assert_volume(&part, "linear_pattern(p)", expected, 5e-3 * removed);
    let v = |c: [f64; 3]| Vector3::from_array(c.map(Scalar::from_f64));
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
    let mut program = plate(&[[10.0, 20.0]]);
    let mut tapped = simple_m6(PLATE_TOP, Extent::blind(8.0));
    tapped.kind = HoleKind::Tapped;
    program.push("h", tapped);
    program.push("p", linear(&["h"], 3.0, 15.0));
    let part = program.built(registry).unwrap();
    let drawn = geop_core_editor::view::annotations_of(&part.current());
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
    program.push(
        "low",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [30.0, 40.0, 10.0]),
    );
    program.push(
        "high",
        PrimitiveArgs::block([30.0, 0.0, 0.0], [60.0, 40.0, 20.0]).join("block(low)"),
    );
    program.push(
        "centres",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::Z)),
            sketch: points_sketch(&[[15.0, 20.0]]),
            ..Default::default()
        },
    );
    program.push("h", simple_m6("block(low,start)", Extent::UpToNext));
    program.push("p", linear(&["h"], 2.0, 30.0));
    let part = build(&program);
    assert_valid(&part);
    let area = PI * 3.3 * 3.3;
    let removed = area * 10.0 + area * 20.0;
    let expected = 30.0 * 40.0 * 10.0 + 30.0 * 40.0 * 20.0 - removed;
    assert_volume(&part, "linear_pattern(p)", expected, 5e-3 * removed);
}

/// Bodies and features at once, and a step that made no feature, are
/// refused, saying so.
#[test]
fn what_is_no_feature_is_refused() {
    let program = drilled_plate();
    let refused = |args: LinearPatternArgs| {
        let mut program = program.clone();
        program.push("p", args);
        program.build_model(registry).err().unwrap().to_string()
    };
    let mut both = linear(&["hole"], 2.0, 2.0);
    both.seeds.bodies = vec![body("cylinder(hole)")];
    assert!(refused(both).contains("one or the other"));
    let error = refused(linear(&["plate"], 2.0, 2.0));
    assert!(error.contains("is no feature"), "{error}");
}

/// A feature is picked by clicking a face it made — the hole by its wall —
/// and lit as every face of it; a face no feature made picks none.
#[test]
fn a_feature_is_picked_by_a_face_it_made() {
    let part = drilled_plate().built(registry).unwrap();
    let reader = part.current();
    let part = made(&drilled_plate()).unwrap();
    let face_named = |name: &str| ElementReference::<NamedModel, Face>::new(model(), name);
    let wall = faces_of(&part, "hole");
    let hit: PickedElement = face_named(&wall[0]).picked();
    let picked: Vec<FeatureRef> = selected(&reader, &[hit]);
    assert_eq!(picked, [feature("hole")]);
    let mut lit: Vec<String> = (picked[0].drawn(&reader).into_iter())
        .map(|face| face.name)
        .collect();
    lit.sort();
    assert_eq!(lit, wall);
    let top = face_named("block(plate,end)").picked();
    assert!(selected::<FeatureRef>(&reader, &[top]).is_empty());
}
