//! Patterns of features: a hole cut by an extrude, a tapped hole with its
//! thread, a boss joined to a plate, a hole going up to the next face —
//! each done again at every instance of a linear or circular pattern, as
//! the step that made it did it.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_math::vector::Vector3;
use geop_ops::{EntityRef, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs};
use geop_ops_hole::{HoleArgs, HoleKind, Standard, iso::Fit};
use geop_ops_pattern::{CircularPatternArgs, Direction, LinearPatternArgs, MirrorArgs, Spacing};
use geop_ops_sketch::{AddSketchArgs, Constraint, Sketch};

use super::pattern_tests::{assert_valid, axis, build, circle, extruded, polygon, volume, z_plane};
use crate::examples::n;
use crate::{PartOperation, Program};

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
        bodies: Vec::new(),
        features: features
            .iter()
            .map(|f| EntityRef::Feature {
                name: f.to_string(),
            })
            .collect(),
        first: along_x(count, step),
        second: None,
        combine: Combine::NewBody,
    }
}

/// A sketch on the `z` plane named `{name}_sketch`, and the extrude `name`
/// of it — `extent` — combined with the plate as `combine` says.
fn extrude_into(
    program: &mut Program,
    name: &str,
    sketch: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(
        &sketch_name,
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch,
            ..Default::default()
        },
    );
    program.push(
        name,
        ExtrudeArgs {
            sketch: sketch_name,
            extent,
            face: false,
            combine,
        },
    );
}

/// Through the plate, both ways from the `z` plane.
fn through() -> Extents {
    let mut extents = Extents::blind(2.0);
    extents.symmetric = true;
    extents
}

/// A plate 8 x 2 x 0.5, `extrude(plate)`, with a hole of radius 0.3 at
/// (1, 1) cut through it by the extrude `hole`.
pub(super) fn plate_with_hole() -> Program {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        polygon(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    extrude_into(
        &mut program,
        "hole",
        circle([1.0, 1.0], 0.3),
        through(),
        Combine::Difference {
            target: "extrude(plate)".into(),
        },
    );
    program
}

/// The faces of `part` the feature of `step` made, as a pick tells.
fn faces_of(part: &Part<S>, step: &str) -> Vec<String> {
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
    let got = volume(&part, "linear_pattern(holes)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");

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
        polygon(&[[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]]),
        Extents::blind(0.5),
    );
    extrude_into(
        &mut program,
        "hole",
        circle([2.0, 0.0], 0.3),
        through(),
        Combine::Difference {
            target: "extrude(plate)".into(),
        },
    );
    program.push(
        "bolts",
        CircularPatternArgs {
            bodies: Vec::new(),
            features: feature("hole"),
            axis: axis(FrameAxis::Z),
            reversed: false,
            count: 6.0.into(),
            angle: Spacing::extent(360.0),
            combine: Combine::NewBody,
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["circular_pattern(bolts)"]);
    let expected = 36.0 * 0.5 - 6.0 * PI * 0.3 * 0.3 * 0.5;
    let got = volume(&part, "circular_pattern(bolts)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
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
        polygon(&[[-8.0, 0.0], [8.0, 0.0], [8.0, 2.0], [-8.0, 2.0]]),
        Extents::blind(0.5),
    );
    extrude_into(
        &mut program,
        "hole",
        circle([1.0, 1.0], 0.3),
        through(),
        Combine::Difference {
            target: "extrude(plate)".into(),
        },
    );
    program.push("holes", linear(&["hole"], 4.0, 2.0));
    program.push(
        "m",
        MirrorArgs {
            bodies: Vec::new(),
            features: feature("hole")
                .into_iter()
                .chain(feature("holes"))
                .collect(),
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            combine: Combine::NewBody,
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["mirror(m)"]);
    let expected = 16.0 * 2.0 * 0.5 - 8.0 * PI * 0.3 * 0.3 * 0.5;
    let got = volume(&part, "mirror(m)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
}

/// A boss of radius 0.3 joined onto the plate's top, done again three
/// times: three bosses on one solid.
#[test]
fn boss_patterned_in_a_row() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        polygon(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "extrude(plate,end)".into(),
            }),
            sketch: circle([1.0, 1.0], 0.3),
            ..Default::default()
        },
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: "boss_sketch".into(),
            extent: Extents::blind(0.5),
            face: false,
            combine: Combine::Union {
                target: "extrude(plate)".into(),
            },
        },
    );
    program.push("bosses", linear(&["boss"], 3.0, 2.5));
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(bosses)"]);
    let expected = 8.0 * 2.0 * 0.5 + 3.0 * PI * 0.3 * 0.3 * 0.5;
    let got = volume(&part, "linear_pattern(bosses)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
}

/// A sketch of fixed points at `points`: `p0`, `p1`, ...
fn points_sketch(points: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    for p in points {
        let id = s.add_point(n(p[0]), n(p[1]));
        s.constrain(Constraint::Fix {
            point: id,
            x: n(p[0]),
            y: n(p[1]),
        });
    }
    s.solve().unwrap();
    s
}

fn simple_m6(face: &str, end: Extent) -> HoleArgs {
    HoleArgs {
        face: face.into(),
        points: vec![EntityRef::Sketch {
            name: "centres".into(),
        }],
        kind: HoleKind::Simple,
        standard: Standard::Iso {
            size: "M6".into(),
            fit: Fit::Normal,
        },
        end,
        drill_point: false,
        thread_length: None,
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
    program.push("h", PartOperation::Hole(tapped));
    program.push("p", linear(&["h"], 3.0, 15.0));
    let part = build(&program);
    assert_valid(&part);
    let removed = 3.0 * PI * 2.5 * 2.5 * 8.0;
    let got = volume(&part, "linear_pattern(p)");
    let expected = 60.0 * 40.0 * 10.0 - removed;
    assert!(
        (got - expected).abs() <= 5e-3 * removed,
        "{got} vs {expected}"
    );
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
    assert_eq!(part.threads().count(), 3);
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
        polygon(&[[0.0, 0.0], [30.0, 0.0], [30.0, 40.0], [0.0, 40.0]]),
        Extents::blind(10.0),
    );
    extrude_into(
        &mut program,
        "high",
        polygon(&[[30.0, 0.0], [60.0, 0.0], [60.0, 40.0], [30.0, 40.0]]),
        Extents::blind(20.0),
        Combine::Union {
            target: "extrude(low)".into(),
        },
    );
    program.push(
        "centres",
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: points_sketch(&[[15.0, 20.0]]),
            ..Default::default()
        },
    );
    program.push(
        "h",
        PartOperation::Hole(simple_m6("extrude(low,start)", Extent::UpToNext)),
    );
    program.push("p", linear(&["h"], 2.0, 30.0));
    let part = build(&program);
    assert_valid(&part);
    let area = PI * 3.3 * 3.3;
    let removed = area * 10.0 + area * 20.0;
    let expected = 30.0 * 40.0 * 10.0 + 30.0 * 40.0 * 20.0 - removed;
    let got = volume(&part, "linear_pattern(p)");
    assert!(
        (got - expected).abs() <= 5e-3 * removed,
        "{got} vs {expected}"
    );
}

/// Bodies and features at once, a body picked as a feature, and a step
/// that made no feature are refused, saying so.
#[test]
fn what_is_no_feature_is_refused() {
    let program = plate_with_hole();
    let refused = |args: LinearPatternArgs| {
        let mut program = program.clone();
        program.push("p", args);
        program
            .build::<S>(&geop_ops::NoFiles)
            .err()
            .unwrap()
            .to_string()
    };
    let mut both = linear(&["hole"], 2.0, 2.0);
    both.bodies = vec![EntityRef::Solid {
        name: "extrude(hole)".into(),
    }];
    assert!(refused(both).contains("one or the other"));
    let error = refused(linear(&["plate"], 2.0, 2.0));
    assert!(error.contains("is no feature"), "{error}");
    let mut solid = linear(&[], 2.0, 2.0);
    solid.features = vec![EntityRef::Solid {
        name: "extrude(hole)".into(),
    }];
    let error = refused(solid);
    assert!(error.contains("is no feature"), "{error}");
}
