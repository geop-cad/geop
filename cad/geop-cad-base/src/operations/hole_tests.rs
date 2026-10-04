//! Holes and threads drilled by programs into plates and shafts: each kind
//! of hole, every end condition, a pattern of points, a hole coming out
//! through a curved face, tapped holes carrying their cosmetic threads.
//!
//! What a hole removed is checked against its dimensions by volume: the
//! part's volume is measured on its triangulation, which is accurate to the
//! chordal error of the mesh — so the tolerance below is the mesh's, not a
//! geometric epsilon.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::Body;
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs};
use geop_ops_hole::{
    HoleArgs, HoleKind, Standard, ThreadArgs,
    iso::{Fit, SIZES, metric},
};
use geop_ops_sketch::{AddSketchArgs, Constraint, Sketch};

use super::regression_tests::check_valid;
use crate::examples::n;
use crate::{PartOperation, Program};

/// The plate's size: 60 x 40, 10 thick, its top at `z = 10`.
const PLATE: [f64; 3] = [60.0, 40.0, 10.0];

fn assert_valid(part: &Part<S>) {
    if let Err(report) = check_valid(part) {
        panic!("{report}");
    }
}

/// The volume of the solid `solid`, measured on its triangulation.
fn volume(part: &Part<S>, solid: &str) -> f64 {
    let id = part.solid_id(solid).unwrap();
    let faces = part.topology().body_faces(Body::Solid(id)).unwrap();
    let raster = geop_ops_rasterize::rasterize(part.topology(), 48).unwrap();
    geop_ops_rasterize::stl::stl_triangles(&raster, &faces)
        .iter()
        .map(|t| {
            let [a, b, c] = t.corners.map(|p| p.map(f64::from));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

/// Whether `measured` is `expected` to within the mesh's accuracy: half a
/// percent of `scale`, the volume the curved faces enclose.
fn assert_volume(measured: f64, expected: f64, scale: f64) {
    assert!(
        (measured - expected).abs() <= 5e-3 * scale,
        "volume {measured}, expected {expected} (± {})",
        5e-3 * scale
    );
}

/// A sketch of fixed points at `points`: `p0`, `p1`, ...
fn points_sketch(points: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let ids: Vec<_> = points
        .iter()
        .map(|p| s.add_point(n(p[0]), n(p[1])))
        .collect();
    for (id, p) in ids.into_iter().zip(points) {
        s.constrain(Constraint::Fix {
            point: id,
            x: n(p[0]),
            y: n(p[1]),
        });
    }
    s.solve().unwrap();
    s
}

/// A closed polygon through `corners`.
fn polygon(corners: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

/// A sketch named `name` on `plane`.
fn sketch_on(program: &mut Program, name: &str, plane: EntityRef, sketch: Sketch) {
    program.push(
        name,
        AddSketchArgs {
            plane: Some(plane),
            sketch,
            ..Default::default()
        },
    );
}

fn z_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z))
}

/// The plate `extrude(plate)`, and the sketch `centres` on its top with
/// points at `points` (in world `x`, `y`).
fn plate(points: &[[f64; 2]]) -> Program {
    let mut program = Program::new();
    let [w, d, t] = PLATE;
    sketch_on(
        &mut program,
        "outline",
        z_plane(),
        polygon(&[[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]),
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(t),
            face: false,
            combine: Combine::NewBody,
        },
    );
    sketch_on(
        &mut program,
        "centres",
        EntityRef::Face {
            name: "extrude(plate,end)".into(),
        },
        points_sketch(points),
    );
    program
}

/// A hole of `kind` and `standard` drilled into the plate's top at every
/// point of the sketch `centres`, as far as `end`.
fn hole(kind: HoleKind, standard: Standard, end: Extent) -> HoleArgs {
    HoleArgs {
        face: "extrude(plate,end)".into(),
        points: vec![EntityRef::Sketch {
            name: "centres".into(),
        }],
        kind,
        standard,
        end,
        drill_point: false,
        thread_length: None,
    }
}

fn iso(size: &str, fit: Fit) -> Standard {
    Standard::Iso {
        size: size.into(),
        fit,
    }
}

/// `program` with `args` as the step `h`, built and checked valid.
fn drilled(mut program: Program, args: HoleArgs) -> Part<S> {
    program.push("h", PartOperation::Hole(args));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    part
}

fn plate_volume() -> f64 {
    PLATE.iter().product()
}

/// A cylinder's volume.
fn cylinder(d: f64, h: f64) -> f64 {
    PI * d * d / 4.0 * h
}

/// A blind M6 clearance hole, 6 deep: its wall named after its point, the
/// plate short of exactly that cylinder.
#[test]
fn blind_clearance_hole() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(HoleKind::Simple, iso("M6", Fit::Normal), Extent::Blind(6.0)),
    );
    for q in 0..4 {
        part.face_id(&format!("hole(h,centres,p0,wall,q{q})"))
            .unwrap();
        part.face_id(&format!("hole(h,centres,p0,bottom,q{q})"))
            .unwrap();
    }
    let removed = cylinder(6.6, 6.0);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A blind hole ending in a drill's point: a 118° cone below the depth.
#[test]
fn blind_hole_with_drill_point() {
    let mut args = hole(HoleKind::Simple, iso("M8", Fit::Close), Extent::Blind(5.0));
    args.drill_point = true;
    let part = drilled(plate(&[[20.0, 20.0]]), args);
    part.face_id("hole(h,centres,p0,point,q0)").unwrap();
    let r = 8.4 / 2.0;
    let tip = r / 59f64.to_radians().tan();
    let removed = cylinder(8.4, 5.0) + PI * r * r * tip / 3.0;
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A counterbored M5 hole through all: the counterbore Ø10, as deep as the
/// screw's head is high, and the clearance hole below it.
#[test]
fn counterbored_hole_through_all() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Counterbore,
            iso("M5", Fit::Normal),
            Extent::ThroughAll,
        ),
    );
    part.face_id("hole(h,centres,p0,shoulder,q0)").unwrap();
    let removed = cylinder(10.0, 5.0) + cylinder(5.5, 5.0);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A countersunk M6 hole through all: a 90° cone Ø12.6 on the face.
#[test]
fn countersunk_hole_through_all() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Countersink,
            iso("M6", Fit::Normal),
            Extent::ThroughAll,
        ),
    );
    part.face_id("hole(h,centres,p0,countersink,q0)").unwrap();
    let (rs, r) = (12.6 / 2.0, 6.6 / 2.0);
    let cone = rs - r;
    let removed = PI * cone / 3.0 * (rs * rs + rs * r + r * r) + cylinder(6.6, PLATE[2] - cone);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A custom hole: the diameter given by hand.
#[test]
fn custom_hole() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Counterbore,
            Standard::Custom {
                diameter: 4.0,
                head_diameter: 9.0,
                head_depth: 3.0,
            },
            Extent::Blind(7.0),
        ),
    );
    let removed = cylinder(9.0, 3.0) + cylinder(4.0, 4.0);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A tapped M6 hole, 8 deep: drilled to the tap drill, Ø5, and its wall
/// carrying the cosmetic thread M6x1, 8 long, from the face into the plate.
#[test]
fn tapped_hole_carries_a_cosmetic_thread() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::Blind(8.0)),
    );
    let removed = cylinder(5.0, 8.0);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
    let thread = part.thread("hole(h,centres,p0,thread)").unwrap();
    assert_eq!(thread.designation, "M6x1");
    assert!(thread.internal);
    assert_eq!(thread.length, 8.0);
    assert_eq!(thread.pitch, 1.0);
    assert!(
        thread.face.starts_with("hole(h,centres,p0,wall"),
        "{}",
        thread.face
    );
    let v = |c: [f64; 3]| geop_core_math::vector::Vector3::from_array(c.map(S::from_f64));
    assert!(
        thread.axis.point.could_be_equal(&v([20.0, 20.0, 10.0])),
        "{:?}",
        thread.axis.point
    );
    assert!(thread.axis.direction.could_be_equal(&v([0.0, 0.0, -1.0])));
    // Drawn, and described.
    let view = geop_ops::ui::PartView::of(&part).unwrap();
    assert_eq!(view.threads.len(), 1);
    assert!(view.threads[0].polyline.len() > 8 * 4);
    let description = geop_ops::PartDescription::of(&part).unwrap();
    assert_eq!(description.threads.len(), 1);

    // Through all, the thread runs through the whole plate.
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::ThroughAll),
    );
    let thread = part.thread("hole(h,centres,p0,thread)").unwrap();
    // Measured on the model: the plate's thickness, to rounding.
    assert!((thread.length - PLATE[2]).abs() < 1e-9, "{}", thread.length);

    // A thread longer than its hole is refused.
    let mut args = hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::Blind(8.0));
    args.thread_length = Some(9.0);
    let mut program = plate(&[[20.0, 20.0]]);
    program.push("h", PartOperation::Hole(args));
    let error = program.build::<S>(&NoFiles).err().expect("refused");
    assert!(error.root_message().contains("only 8"), "{error}");
}

/// Four points of one sketch: four holes, each named after its point.
#[test]
fn a_pattern_of_points_in_one_sketch() {
    let points = [[10.0, 10.0], [50.0, 10.0], [50.0, 30.0], [10.0, 30.0]];
    let part = drilled(
        plate(&points),
        hole(HoleKind::Simple, iso("M4", Fit::Normal), Extent::ThroughAll),
    );
    for k in 0..4 {
        part.face_id(&format!("hole(h,centres,p{k},wall,q0)"))
            .unwrap();
    }
    let removed = 4.0 * cylinder(4.5, PLATE[2]);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// Points picked one by one, a sketch point and a datum point 10 in from
/// the plate's corner: two holes.
#[test]
fn points_picked_one_by_one() {
    let mut program = plate(&[[20.0, 20.0], [40.0, 20.0]]);
    program.push(
        "corner",
        geop_ops_datums::AddDatumArgs {
            selection: vec![EntityRef::Vertex {
                name: "extrude(plate,outline,p2,end)".into(),
            }],
            construction: geop_ops_datums::Construction::Point {
                x: -10.0,
                y: -10.0,
                z: 0.0,
            },
        },
    );
    let mut args = hole(HoleKind::Simple, iso("M3", Fit::Close), Extent::Blind(3.0));
    args.points = vec![
        EntityRef::SketchPoint {
            sketch: "centres".into(),
            point: geop_core_sketch::PointId(1),
        },
        EntityRef::datum("corner"),
    ];
    let part = drilled(program, args);
    part.face_id("hole(h,centres,p1,wall,q0)").unwrap();
    part.face_id("hole(h,corner,wall,q0)").unwrap();
    let removed = 2.0 * cylinder(3.2, 3.0);
    assert_volume(volume(&part, "hole(h)"), plate_volume() - removed, removed);
}

/// A "C" lying on its back, two arms 10 thick with a gap of 10 between:
/// up to next goes through the top arm and stops in the gap, through all
/// goes through both.
#[test]
fn up_to_next_stops_in_the_gap() {
    let program = || {
        let mut program = Program::new();
        sketch_on(
            &mut program,
            "c",
            z_plane(),
            polygon(&[
                [0.0, 0.0],
                [40.0, 0.0],
                [40.0, 10.0],
                [10.0, 10.0],
                [10.0, 20.0],
                [40.0, 20.0],
                [40.0, 30.0],
                [0.0, 30.0],
            ]),
        );
        program.push(
            "block",
            ExtrudeArgs {
                sketch: "c".into(),
                extent: Extents::blind(10.0),
                face: false,
                combine: Combine::NewBody,
            },
        );
        // On the top arm's top, `y = 30`: sketch `x` is world `x`, sketch
        // `y` world `-z`.
        sketch_on(
            &mut program,
            "centres",
            EntityRef::Face {
                name: "extrude(block,c,c14)".into(),
            },
            points_sketch(&[[25.0, -5.0]]),
        );
        program
    };
    let args = |end| HoleArgs {
        face: "extrude(block,c,c14)".into(),
        ..hole(HoleKind::Simple, iso("M6", Fit::Normal), end)
    };
    let block = 40.0 * 30.0 * 10.0 - 30.0 * 10.0 * 10.0;
    let wall = cylinder(6.6, 10.0);
    let part = drilled(program(), args(Extent::UpToNext));
    assert_volume(volume(&part, "hole(h)"), block - wall, wall);
    let part = drilled(program(), args(Extent::ThroughAll));
    assert_volume(volume(&part, "hole(h)"), block - 2.0 * wall, wall);
}

/// A hole drilled down from the flat top of a "D" lying along `x` comes out
/// through its round bottom.
#[test]
fn hole_coming_out_through_a_curved_face() {
    let mut program = Program::new();
    // On the X plane: sketch `x` is world `y`, sketch `y` world `z`. A flat
    // top at `z = 10`, the bottom a half circle of radius 10 around `z =
    // 10`.
    let mut d = Sketch::new();
    let a = d.add_point(n(10.0), n(10.0));
    let b = d.add_point(n(-10.0), n(10.0));
    d.add_line(a, b);
    d.add_arc(b, a, n(PI));
    sketch_on(
        &mut program,
        "d",
        EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::X)),
        d,
    );
    program.push(
        "bar",
        ExtrudeArgs {
            sketch: "d".into(),
            extent: Extents::blind(30.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let top = "extrude(bar,d,c2)";
    // Measured on the same mesh, so its error on the round bottom cancels.
    let bar = volume(&program.build::<S>(&NoFiles).unwrap(), "extrude(bar)");
    // On the top, normal `+z`: sketch `x`, `y` are world `x`, `y`.
    sketch_on(
        &mut program,
        "centres",
        EntityRef::Face { name: top.into() },
        points_sketch(&[[15.0, 3.0]]),
    );
    let part = drilled(
        program,
        HoleArgs {
            face: top.into(),
            ..hole(HoleKind::Simple, iso("M5", Fit::Normal), Extent::ThroughAll)
        },
    );
    // Through the half disc, `sqrt(100 - y^2)` deep at `y` off its middle,
    // for every `y` across the hole — its own disc of radius 2.75.
    let r: f64 = 2.75;
    let steps = 2000;
    let mut removed = 0.0;
    for i in 0..steps {
        let y0 = -r + 2.0 * r * (i as f64 + 0.5) / steps as f64;
        for j in 0..steps {
            let x = -r + 2.0 * r * (j as f64 + 0.5) / steps as f64;
            if x * x + y0 * y0 <= r * r {
                let y = 3.0 + y0;
                removed += (100.0 - y * y).sqrt() * (2.0 * r / steps as f64).powi(2);
            }
        }
    }
    assert_volume(volume(&part, "hole(h)"), bar - removed, removed);
}

/// What is not supported is refused, saying why: a point off the face, a
/// face that is not flat, a countersink ISO 15065 has no size for, a tapped
/// hole of a custom size.
#[test]
fn unsupported_holes_are_refused() {
    let refused = |program: Program, args: HoleArgs, says: &str| {
        let mut program = program;
        program.push("h", PartOperation::Hole(args));
        let error = program.build::<S>(&NoFiles).err().expect("refused");
        assert!(error.root_message().contains(says), "{error}");
    };
    let mut off = plate(&[[20.0, 20.0]]);
    sketch_on(&mut off, "below", z_plane(), points_sketch(&[[5.0, 5.0]]));
    refused(
        off,
        HoleArgs {
            points: vec![EntityRef::Sketch {
                name: "below".into(),
            }],
            ..hole(HoleKind::Simple, iso("M6", Fit::Normal), Extent::Blind(3.0))
        },
        "does not lie on face",
    );
    refused(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Countersink,
            iso("M24", Fit::Normal),
            Extent::ThroughAll,
        ),
        "stops at M20",
    );
    refused(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Tapped,
            Standard::Custom {
                diameter: 5.0,
                head_diameter: 0.0,
                head_depth: 0.0,
            },
            Extent::Blind(5.0),
        ),
        "pick an ISO size",
    );
    refused(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Counterbore,
            iso("M10", Fit::Normal),
            Extent::Blind(9.0),
        ),
        "head alone takes 10",
    );
}

/// A bolt's blank, its axis `z`: a head Ø10 and 5 high, and on it a shank
/// Ø6 and 20 long, `extrude(shank)`.
fn shaft() -> Program {
    let mut program = Program::new();
    let circle = |r: f64| {
        let mut s = Sketch::new();
        let c = s.add_point(n(0.0), n(0.0));
        s.add_circle(c, n(r));
        s
    };
    sketch_on(&mut program, "head_circle", z_plane(), circle(5.0));
    program.push(
        "head",
        ExtrudeArgs {
            sketch: "head_circle".into(),
            extent: Extents::blind(5.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    sketch_on(
        &mut program,
        "shank_circle",
        EntityRef::Face {
            name: "extrude(head,end)".into(),
        },
        circle(3.0),
    );
    program.push(
        "shank",
        ExtrudeArgs {
            sketch: "shank_circle".into(),
            extent: Extents::blind(20.0),
            face: false,
            combine: Combine::Union {
                target: "extrude(head)".into(),
            },
        },
    );
    program
}

/// The name of a face of the shank's round side.
fn shaft_side(part: &Part<S>) -> String {
    part.names()
        .iter()
        .map(|(_, name)| name.to_string())
        .find(|name| {
            name.starts_with("extrude(shank,shank_circle,c")
                && !name.contains(",start")
                && !name.contains(",end")
                && part.face_id(name).is_ok()
        })
        .expect("a side face")
}

/// A cosmetic M6 thread on the shank: recorded on the whole round side,
/// external, from its free end down to the head; 12 long up from the head
/// when reversed.
#[test]
fn cosmetic_thread_on_a_shaft() {
    let program = shaft();
    let side = shaft_side(&program.build::<S>(&NoFiles).unwrap());
    let mut threaded = program.clone();
    threaded.push(
        "t",
        PartOperation::Thread(ThreadArgs {
            face: side.clone(),
            size: "M6".into(),
            length: None,
            reversed: false,
            modelled: false,
        }),
    );
    let part = threaded.build::<S>(&NoFiles).unwrap();
    let thread = part.thread("thread(t)").unwrap();
    assert!(!thread.internal);
    // Measured on the model: the shaft's length, to rounding.
    assert!((thread.length - 20.0).abs() < 1e-9, "{}", thread.length);
    assert!(thread.axis.point[2].could_be_equal(S::from_f64(25.0)));

    let mut reversed = program.clone();
    reversed.push(
        "t",
        PartOperation::Thread(ThreadArgs {
            face: side.clone(),
            size: "M6".into(),
            length: Some(12.0),
            reversed: true,
            modelled: false,
        }),
    );
    let part = reversed.build::<S>(&NoFiles).unwrap();
    let thread = part.thread("thread(t)").unwrap();
    assert_eq!(thread.length, 12.0);
    assert!(thread.axis.point[2].could_be_equal(S::from_f64(5.0)));

    // An M8 does not fit a Ø6 shaft.
    let mut wrong = program.clone();
    wrong.push(
        "t",
        PartOperation::Thread(ThreadArgs {
            face: side,
            size: "M8".into(),
            length: None,
            reversed: false,
            modelled: false,
        }),
    );
    let error = wrong.build::<S>(&NoFiles).err().expect("refused");
    assert!(error.root_message().contains("needs a shaft of"), "{error}");
}

/// The volume a modelled ISO thread of `pitch` cuts per turn out of a
/// shaft of `radius` whose minor diameter is `minor`: the groove's section
/// inside the shaft — a trapezoid `P / 4` wide at the minor diameter,
/// widening at 30° per flank — swept once round (Pappus).
fn groove_per_turn(radius: f64, minor: f64, pitch: f64) -> f64 {
    let depth = radius - minor / 2.0;
    let (inner, outer) = (pitch / 4.0, pitch / 4.0 + 2.0 * depth / 3f64.sqrt());
    let area = (inner + outer) / 2.0 * depth;
    let centroid = minor / 2.0 + depth * (inner + 2.0 * outer) / (3.0 * (inner + outer));
    area * 2.0 * PI * centroid
}

/// A modelled M6 thread cut into the shank, 5 turns down from its free
/// end: valid, and short of what the groove takes — but for the first
/// half turn, where the groove starts above the shank's end.
#[test]
#[ignore = "slow: a modelled thread's booleans take half a minute — run with `cargo test -- --ignored`"]
fn modelled_thread_on_a_shaft() {
    let program = shaft();
    let blank = program.build::<S>(&NoFiles).unwrap();
    let side = shaft_side(&blank);
    let mut threaded = program;
    threaded.push(
        "t",
        PartOperation::Thread(ThreadArgs {
            face: side,
            size: "M6".into(),
            length: Some(5.0),
            reversed: false,
            modelled: true,
        }),
    );
    let part = threaded.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(
        part.names()
            .iter()
            .any(|(_, name)| name.starts_with("thread(t,c")),
        "no face of the thread is left"
    );
    let size = metric("M6").unwrap();
    let removed = volume(&blank, "extrude(shank)") - volume(&part, "thread(t)");
    let groove = 5.0 * groove_per_turn(3.0, size.minor_diameter(), size.pitch);
    assert!(
        removed > 0.85 * groove && removed < 1.02 * groove,
        "removed {removed}, the groove {groove}"
    );
}

/// A modelled M6 thread cut into the wall of a tapped hole through the
/// plate, from the face it was drilled into, all the way.
#[test]
#[ignore = "slow: a modelled thread's booleans take half a minute or more — run with `cargo test -- --ignored`"]
fn modelled_thread_in_a_tapped_hole() {
    let mut program = plate(&[[20.0, 20.0]]);
    program.push(
        "h",
        PartOperation::Hole(hole(
            HoleKind::Tapped,
            iso("M6", Fit::Normal),
            Extent::ThroughAll,
        )),
    );
    program.push(
        "t",
        PartOperation::Thread(ThreadArgs {
            face: "hole(h,centres,p0,wall,q0)".into(),
            size: "M6".into(),
            length: Some(6.0),
            reversed: false,
            modelled: true,
        }),
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let before = plate_volume() - cylinder(5.0, PLATE[2]);
    let removed = before - volume(&part, "thread(t)");
    assert!(removed > 0.0, "removed {removed}");
}

/// Every ISO size, every kind of hole, blind and through all, in a plate
/// thick enough for the largest: each valid, each removing what its
/// dimensions say.
#[test]
#[ignore = "slow: every ISO size × hole kind × end — run with `cargo test -- --ignored`"]
fn every_iso_size_and_kind() {
    let mut failures = Vec::new();
    for size in &SIZES {
        for kind in [
            HoleKind::Simple,
            HoleKind::Counterbore,
            HoleKind::Countersink,
            HoleKind::Tapped,
        ] {
            if kind == HoleKind::Countersink && size.countersink.is_none() {
                continue;
            }
            for end in [Extent::Blind(30.0), Extent::ThroughAll] {
                let mut program = Program::new();
                let (w, t) = (100.0, 40.0);
                sketch_on(
                    &mut program,
                    "outline",
                    z_plane(),
                    polygon(&[[0.0, 0.0], [w, 0.0], [w, w], [0.0, w]]),
                );
                program.push(
                    "plate",
                    ExtrudeArgs {
                        sketch: "outline".into(),
                        extent: Extents::blind(t),
                        face: false,
                        combine: Combine::NewBody,
                    },
                );
                sketch_on(
                    &mut program,
                    "centres",
                    EntityRef::Face {
                        name: "extrude(plate,end)".into(),
                    },
                    points_sketch(&[[50.0, 50.0]]),
                );
                program.push(
                    "h",
                    PartOperation::Hole(hole(kind, iso(size.name, Fit::Normal), end)),
                );
                let label = format!("{} {kind:?} {end:?}", size.name);
                match program.build::<S>(&NoFiles) {
                    Err(e) => failures.push(format!("{label}: {e}")),
                    Ok(part) => {
                        if let Err(report) = check_valid(&part) {
                            failures.push(format!("{label}: invalid: {report}"));
                        }
                        let _ = metric(size.name).unwrap();
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
