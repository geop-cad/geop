//! Holes and threads drilled by programs into plates and shafts: each kind
//! of hole, every end condition, a pattern of points, a hole coming out
//! through a curved face, tapped holes carrying their cosmetic threads.
//!
//! What a hole removed is checked against its dimensions by volume: the
//! part's volume is measured on its triangulation, which is accurate to the
//! chordal error of the mesh — so the tolerance below is the mesh's, not a
//! geometric epsilon.

use std::f64::consts::PI;

use crate::{
    HoleArgs, HoleCentre, HoleKind, Standard, ThreadArgs,
    iso::{Fit, SIZES, metric},
};
use geop_core_math::geop_error::GeopResult;
use geop_core_math::primitives::FrameAxis;
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_program::element::ElementReference;
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extent, Extents};
use geop_ops_model::{CosmeticThread, Design, NamedModel, Threads, model, read_model, read_threads};
use geop_ops_sketch::{Sketch, sketch_element, whole_sketch};

use geop_ops_add_datum::{AddDatum, AddDatumArgs, Construction, Selected};
use geop_ops_datums::datums;
use geop_ops_extrude_revolve::testing::{
    Built, extrusion, n, on_face, on_origin, outline, sketch,
};
use geop_ops_model::testing::{assert_valid, assert_volume, check_valid, solid_volume};
use geop_ops_parameters::{ProgramFile, Registry};
use geop_ops_pattern::{CircularPattern, LinearPattern, Mirror};

use crate::testing::{PLATE, plate, points_sketch};
use crate::{Hole, Thread};

/// The operations these tests' programs are written in, and those of
/// [`super::pattern_tests`].
pub(super) fn registry() -> GeopResult<Registry> {
    (geop_ops_extrude_revolve::testing::registry()?.with(AddDatum)?)
        .with(Hole)?
        .with(Thread)?
        .with(LinearPattern)?
        .with(CircularPattern)?
        .with(Mirror)
}

/// A hole of `kind` and `standard` drilled into the plate's top at every
/// point of the sketch `centres`, as far as `end`.
pub(super) fn hole(kind: HoleKind, standard: Standard, end: Extent) -> HoleArgs {
    HoleArgs {
        face: Some(named("extrude(plate,end)")),
        points: vec![HoleCentre::Sketch(whole_sketch("centres"))],
        kind,
        standard,
        end,
        drill_point: false,
        thread_length: None,
    }
}

/// The entity of the model named `name`, as a `K`.
pub(super) fn named<K>(name: &str) -> ElementReference<NamedModel<Design>, K> {
    ElementReference::new(model(), name)
}

pub(super) fn iso(size: &str, fit: Fit) -> Standard {
    Standard::Iso {
        size: size.into(),
        fit,
    }
}

/// A program as built: its model, and the cosmetic threads its steps
/// recorded.
pub(super) struct Made {
    pub(super) model: NamedModel<S>,
    pub(super) threads: Threads<Design>,
}

impl std::ops::Deref for Made {
    type Target = NamedModel<S>;

    fn deref(&self) -> &NamedModel<S> {
        &self.model
    }
}

impl Made {
    /// The cosmetic thread `name`.
    pub(super) fn thread(&self, name: &str) -> GeopResult<&CosmeticThread<Design>> {
        self.threads.get(name)
    }
}

/// `program`, built: its model and threads.
pub(super) fn made(program: &ProgramFile) -> GeopResult<Made> {
    let runner = program.built(registry)?;
    let view = runner.part.current();
    Ok(Made {
        model: read_model(&view)?.into_owned(),
        threads: read_threads(&view)?.into_owned(),
    })
}

/// `program` with `args` as the step `h`, built and checked valid.
fn drilled(mut program: ProgramFile, args: HoleArgs) -> Made {
    program.push("h", args);
    let part = made(&program).unwrap();
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
        hole(HoleKind::Simple, iso("M6", Fit::Normal), Extent::blind(6.0)),
    );
    for q in 0..4 {
        part.face_id(&format!("hole(h,centres,p0,wall,q{q})"))
            .unwrap();
        part.face_id(&format!("hole(h,centres,p0,bottom,q{q})"))
            .unwrap();
    }
    let removed = cylinder(6.6, 6.0);
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
}

/// A blind hole ending in a drill's point: a 118° cone below the depth.
#[test]
fn blind_hole_with_drill_point() {
    let mut args = hole(HoleKind::Simple, iso("M8", Fit::Close), Extent::blind(5.0));
    args.drill_point = true;
    let part = drilled(plate(&[[20.0, 20.0]]), args);
    part.face_id("hole(h,centres,p0,point,q0)").unwrap();
    let r = 8.4 / 2.0;
    let tip = r / 59f64.to_radians().tan();
    let removed = cylinder(8.4, 5.0) + PI * r * r * tip / 3.0;
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
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
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
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
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
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
            Extent::blind(7.0),
        ),
    );
    let removed = cylinder(9.0, 3.0) + cylinder(4.0, 4.0);
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
}

/// A tapped M6 hole, 8 deep: drilled to the tap drill, Ø5, and its wall
/// carrying the cosmetic thread M6x1, 8 long, from the face into the plate.
#[test]
fn tapped_hole_carries_a_cosmetic_thread() {
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::blind(8.0)),
    );
    let removed = cylinder(5.0, 8.0);
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
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
    // Drawn.
    let annotations = part.threads.drawn("h").unwrap();
    assert_eq!(annotations.len(), 1);
    assert!(annotations[0].polyline.len() > 8 * 4);

    // Through all, the thread runs through the whole plate.
    let part = drilled(
        plate(&[[20.0, 20.0]]),
        hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::ThroughAll),
    );
    let thread = part.thread("hole(h,centres,p0,thread)").unwrap();
    // Measured on the model: the plate's thickness, to rounding.
    assert!((thread.length - PLATE[2]).abs() < 1e-9, "{}", thread.length);

    // A thread longer than its hole is refused.
    let mut args = hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::blind(8.0));
    args.thread_length = Some(9.0);
    let mut program = plate(&[[20.0, 20.0]]);
    program.push("h", args);
    let error = program.build_model(registry).expect_err("refused");
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
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
}

/// Points picked one by one, a sketch point and a datum point 10 in from
/// the plate's corner: two holes.
#[test]
fn points_picked_one_by_one() {
    let mut program = plate(&[[20.0, 20.0], [40.0, 20.0]]);
    program.push(
        "corner",
        AddDatumArgs {
            selection: vec![Selected::Vertex(named("extrude(plate,outline,p2,end)"))],
            construction: Construction::Point {
                x: (-10.0).into(),
                y: (-10.0).into(),
                z: 0.0.into(),
            },
        },
    );
    let mut args = hole(HoleKind::Simple, iso("M3", Fit::Close), Extent::blind(3.0));
    args.points = vec![
        HoleCentre::SketchPoint(sketch_element("centres", geop_core_sketch::PointId(1))),
        HoleCentre::Datum(ElementReference::new(datums(), "corner")),
    ];
    let part = drilled(program, args);
    part.face_id("hole(h,centres,p1,wall,q0)").unwrap();
    part.face_id("hole(h,corner,wall,q0)").unwrap();
    let removed = 2.0 * cylinder(3.2, 3.0);
    assert_volume(&part, "hole(h)", plate_volume() - removed, 5e-3 * removed);
}

/// A "C" lying on its back, two arms 10 thick with a gap of 10 between:
/// up to next goes through the top arm and stops in the gap, through all
/// goes through both.
#[test]
fn up_to_next_stops_in_the_gap() {
    let program = || {
        let mut program = ProgramFile::new();
        program.push(
            "c",
            sketch(
                on_origin(FrameAxis::Z),
                outline(&[
                    [0.0, 0.0],
                    [40.0, 0.0],
                    [40.0, 10.0],
                    [10.0, 10.0],
                    [10.0, 20.0],
                    [40.0, 20.0],
                    [40.0, 30.0],
                    [0.0, 30.0],
                ]),
            ),
        );
        program.push(
            "block",
            extrusion("c", Extents::blind(10.0), Combine::NewBody),
        );
        // On the top arm's top, `y = 30`: sketch `x` is world `x`, sketch
        // `y` world `-z`.
        program.push(
            "centres",
            sketch(
                on_face("extrude(block,c,c14)"),
                points_sketch(&[[25.0, -5.0]]),
            ),
        );
        program
    };
    let args = |end| HoleArgs {
        face: Some(named("extrude(block,c,c14)")),
        ..hole(HoleKind::Simple, iso("M6", Fit::Normal), end)
    };
    let block = 40.0 * 30.0 * 10.0 - 30.0 * 10.0 * 10.0;
    let wall = cylinder(6.6, 10.0);
    let part = drilled(program(), args(Extent::UpToNext));
    assert_volume(&part, "hole(h)", block - wall, 5e-3 * wall);
    let part = drilled(program(), args(Extent::ThroughAll));
    assert_volume(&part, "hole(h)", block - 2.0 * wall, 5e-3 * wall);
}

/// A hole drilled down from the flat top of a "D" lying along `x` comes out
/// through its round bottom.
#[test]
fn hole_coming_out_through_a_curved_face() {
    let mut program = ProgramFile::new();
    // On the X plane: sketch `x` is world `y`, sketch `y` world `z`. A flat
    // top at `z = 10`, the bottom a half circle of radius 10 around `z =
    // 10`.
    let mut d = Sketch::new();
    let a = d.add_point(n(10.0), n(10.0));
    let b = d.add_point(n(-10.0), n(10.0));
    d.add_line(a, b);
    d.add_arc(b, a, n(PI));
    program.push("d", sketch(on_origin(FrameAxis::X), d));
    program.push(
        "bar",
        extrusion("d", Extents::blind(30.0), Combine::NewBody),
    );
    let top = "extrude(bar,d,c2)";
    // Measured on the same mesh, so its error on the round bottom cancels.
    let bar = solid_volume(&program.build_model(registry).unwrap(), "extrude(bar)");
    // On the top, normal `+z`: sketch `x`, `y` are world `x`, `y`.
    program.push(
        "centres",
        sketch(on_face(top), points_sketch(&[[15.0, 3.0]])),
    );
    let part = drilled(
        program,
        HoleArgs {
            face: Some(named(top)),
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
    assert_volume(&part, "hole(h)", bar - removed, 5e-3 * removed);
}

/// What is not supported is refused, saying why: a point off the face, a
/// face that is not flat, a countersink ISO 15065 has no size for, a tapped
/// hole of a custom size.
#[test]
fn unsupported_holes_are_refused() {
    let refused = |program: ProgramFile, args: HoleArgs, says: &str| {
        let mut program = program;
        program.push("h", args);
        let error = program.build_model(registry).expect_err("refused");
        assert!(error.root_message().contains(says), "{error}");
    };
    let mut off = plate(&[[20.0, 20.0]]);
    off.push(
        "below",
        sketch(on_origin(FrameAxis::Z), points_sketch(&[[5.0, 5.0]])),
    );
    refused(
        off,
        HoleArgs {
            points: vec![HoleCentre::Sketch(whole_sketch("below"))],
            ..hole(HoleKind::Simple, iso("M6", Fit::Normal), Extent::blind(3.0))
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
            Extent::blind(5.0),
        ),
        "pick an ISO size",
    );
    refused(
        plate(&[[20.0, 20.0]]),
        hole(
            HoleKind::Counterbore,
            iso("M10", Fit::Normal),
            Extent::blind(9.0),
        ),
        "head alone takes 10",
    );
}

/// A bolt's blank, its axis `z`: a head Ø10 and 5 high, and on it a shank
/// Ø6 and 20 long, `extrude(shank)`.
fn shaft() -> ProgramFile {
    let mut program = ProgramFile::new();
    let circle = |r: f64| {
        let mut s = Sketch::new();
        let c = s.add_point(n(0.0), n(0.0));
        s.add_circle(c, n(r));
        s
    };
    program.push(
        "head_circle",
        sketch(on_origin(FrameAxis::Z), circle(5.0)),
    );
    program.push(
        "head",
        extrusion("head_circle", Extents::blind(5.0), Combine::NewBody),
    );
    program.push(
        "shank_circle",
        sketch(on_face("extrude(head,end)"), circle(3.0)),
    );
    program.push(
        "shank",
        extrusion(
            "shank_circle",
            Extents::blind(20.0),
            Combine::join("extrude(head)"),
        ),
    );
    program
}

/// The name of a face of the shank's round side.
fn shaft_side(part: &NamedModel<S>) -> String {
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
    let side = shaft_side(&program.build_model(registry).unwrap());
    let mut threaded = program.clone();
    threaded.push(
        "t",
        ThreadArgs {
            face: Some(named(&side)),
            size: "M6".into(),
            length: None,
            reversed: false,
            modelled: false,
        },
    );
    let part = made(&threaded).unwrap();
    let thread = part.thread("thread(t)").unwrap();
    assert!(!thread.internal);
    // Measured on the model: the shaft's length, to rounding.
    assert!((thread.length - 20.0).abs() < 1e-9, "{}", thread.length);
    assert!(thread.axis.point[2].could_be_equal(S::from_f64(25.0)));

    let mut reversed = program.clone();
    reversed.push(
        "t",
        ThreadArgs {
            face: Some(named(&side)),
            size: "M6".into(),
            length: Some(12.0),
            reversed: true,
            modelled: false,
        },
    );
    let part = made(&reversed).unwrap();
    let thread = part.thread("thread(t)").unwrap();
    assert_eq!(thread.length, 12.0);
    assert!(thread.axis.point[2].could_be_equal(S::from_f64(5.0)));

    // An M8 does not fit a Ø6 shaft.
    let mut wrong = program.clone();
    wrong.push(
        "t",
        ThreadArgs {
            face: Some(named(&side)),
            size: "M8".into(),
            length: None,
            reversed: false,
            modelled: false,
        },
    );
    let error = wrong.build_model(registry).expect_err("refused");
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
    let blank = program.build_model(registry).unwrap();
    let side = shaft_side(&blank);
    let mut threaded = program;
    threaded.push(
        "t",
        ThreadArgs {
            face: Some(named(&side)),
            size: "M6".into(),
            length: Some(5.0),
            reversed: false,
            modelled: true,
        },
    );
    let part = made(&threaded).unwrap();
    assert_valid(&part);
    assert!(
        part.names()
            .iter()
            .any(|(_, name)| name.starts_with("thread(t,c")),
        "no face of the thread is left"
    );
    let size = metric("M6").unwrap();
    let removed = solid_volume(&blank, "extrude(shank)") - solid_volume(&part, "thread(t)");
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
        hole(HoleKind::Tapped, iso("M6", Fit::Normal), Extent::ThroughAll),
    );
    program.push(
        "t",
        ThreadArgs {
            face: Some(named("hole(h,centres,p0,wall,q0)")),
            size: "M6".into(),
            length: Some(6.0),
            reversed: false,
            modelled: true,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let before = plate_volume() - cylinder(5.0, PLATE[2]);
    let removed = before - solid_volume(&part, "thread(t)");
    assert!(removed > 0.0, "removed {removed}");
}

/// Every ISO size, every kind of hole, blind and through all, in a plate
/// thick enough for the largest: each valid, each removing what its
/// dimensions say.
///
/// M24 through all (Ø26) once exceeded the boolean's trace budget: each
/// quarter of the rim on the plate's bottom, 20.4 long, took ~204 steps of
/// a stride fixed at 0.1. The stride now scales with the surfaces and
/// their curvature, so a quarter turn takes 16 steps whatever its radius.
#[test]
#[ignore = "slow: every ISO size × hole kind × end (3 min) — run with `cargo test -- --ignored`"]
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
            for end in [Extent::blind(30.0), Extent::ThroughAll] {
                let mut program = ProgramFile::new();
                let (w, t) = (100.0, 40.0);
                program.push(
                    "outline",
                    sketch(
                        on_origin(FrameAxis::Z),
                        outline(&[[0.0, 0.0], [w, 0.0], [w, w], [0.0, w]]),
                    ),
                );
                program.push(
                    "plate",
                    extrusion("outline", Extents::blind(t), Combine::NewBody),
                );
                program.push(
                    "centres",
                    sketch(
                        on_face("extrude(plate,end)"),
                        points_sketch(&[[50.0, 50.0]]),
                    ),
                );
                program.push("h", hole(kind, iso(size.name, Fit::Normal), end.clone()));
                let label = format!("{} {kind:?} {end:?}", size.name);
                match program.build_model(registry) {
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
