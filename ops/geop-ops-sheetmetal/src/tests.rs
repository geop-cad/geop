//! Sheet-metal bodies built, flanged and unfolded: valid solids, flat
//! patterns as long as their developed lengths, and the refusals.

use std::f64::consts::PI;

use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::{Field, Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_sketch::Sketch;
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{Design, NoFiles, Operation, Part, PlacedSketch};
use geop_ops_rasterize::rasterize;

use crate::{
    BaseFlange, BaseFlangeArgs, Corner, EdgeFlange, EdgeFlangeArgs, FlangePosition, FlatPattern,
    FlatPatternArgs, FlatPatternData, Hem, HemArgs, HemKind, LengthReference, Relief, Sheet,
    SheetCut, SheetCutArgs, SheetMetalRules, flat_pattern_dxf,
};

fn d(x: f64) -> Design {
    Design::from_f64(x)
}

/// A part with the sketch `k` on the world's XY plane.
fn sketched(sketch: Sketch<Design>) -> Part<S> {
    let mut part = Part::<S>::new();
    let plane = CoordinateSystem::world_at(Vector3::zero());
    part.add_sketch(PlacedSketch { plane, sketch }, "k")
        .unwrap();
    part
}

/// The open chain of lines through `points`.
fn chain(points: &[[f64; 2]]) -> Sketch<Design> {
    let mut s = Sketch::new();
    let p: Vec<_> = points
        .iter()
        .map(|c| s.add_point(d(c[0]), d(c[1])))
        .collect();
    for w in p.windows(2) {
        s.add_line(w[0], w[1]);
    }
    s
}

/// The closed polygon through `points`.
fn polygon(points: &[[f64; 2]]) -> Sketch<Design> {
    let mut s = Sketch::new();
    let p: Vec<_> = points
        .iter()
        .map(|c| s.add_point(d(c[0]), d(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

fn rules(thickness: f64, radius: f64, k: f64) -> SheetMetalRules {
    SheetMetalRules {
        thickness,
        bend_radius: radius,
        k_factor: k,
        ..SheetMetalRules::default()
    }
}

fn base(part: Part<S>, rules: SheetMetalRules, depth: f64) -> Part<S> {
    let args = BaseFlangeArgs {
        sketch: "k".into(),
        rules,
        depth: depth.into(),
        flip: false,
    };
    BaseFlange.apply(part, "b", &args, &NoFiles).unwrap()
}

fn flange(edge: &str) -> EdgeFlangeArgs {
    EdgeFlangeArgs {
        edge: edge.into(),
        angle: 90.0.into(),
        length: 0.5.into(),
        reference: LengthReference::OuterSharp,
        position: FlangePosition::MaterialInside,
        radius: None,
        offset_start: 0.0,
        offset_end: 0.0,
        corner: Corner::Open,
    }
}

fn unfold(part: Part<S>, solid: &str) -> Part<S> {
    let args = FlatPatternArgs {
        solid: solid.into(),
        keep: false,
    };
    FlatPattern.apply(part, "fp", &args, &NoFiles).unwrap()
}

/// Why `result` was refused; panics if it was built.
fn refused<T>(result: GeopResult<T>) -> String {
    match result {
        Ok(_) => panic!("built, but should have been refused"),
        Err(e) => e.to_string(),
    }
}

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        panic!(
            "{} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:?}");
    }
    part.check_names().unwrap();
}

/// The box around the solid's vertices: its lower and upper corner, each
/// coordinate the hull of the vertices' enclosures.
fn extent(part: &Part<S>) -> [S; 3] {
    let points: Vec<Vector3<S>> = part.topology().vertices.values().map(|v| v.point).collect();
    [0, 1, 2].map(|k| {
        let lo = points.iter().map(|p| p[k]).reduce(S::min).unwrap();
        let hi = points.iter().map(|p| p[k]).reduce(S::max).unwrap();
        hi.sub(lo)
    })
}

/// The area of the face named `name`, from its triangles.
fn area(part: &Part<S>, name: &str) -> f64 {
    let raster = rasterize(part.topology(), 64).unwrap();
    let face = part.face_id(name).unwrap();
    raster.faces[&face]
        .iter()
        .map(|t| {
            let f = |v: &Vector3<S>| v.to_array().map(|c| c.to_f64());
            let (a, b, c) = (f(&t.a), f(&t.b), f(&t.c));
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() / 2.0
        })
        .sum()
}

/// An L-bracket from a chain of two lines: valid, its bend where the
/// corner was, and its flat pattern as long as the developed length — the
/// two legs short of the bend, and the bend's arc on the neutral surface.
#[test]
fn l_bracket_from_a_chain_unfolds_to_its_developed_length() {
    let (t, r, k) = (0.1, 0.2, 0.4);
    let part = base(
        sketched(chain(&[[0.0, 1.0], [0.0, 0.0], [1.5, 0.0]])),
        rules(t, r, k),
        0.8,
    );
    assert_valid(&part);
    let sheet = part.body_data::<Sheet<S>>("base_flange(b)").unwrap();
    assert_eq!((sheet.flats.len(), sheet.bends.len()), (2, 1));
    // The material lies right of the chain, outside the corner: the bend
    // turns away from the B side, so the A side is its inside.
    assert!(!sheet.bends[0].toward_b);
    part.face_id("base_flange(b,k,p1,a)").unwrap();
    part.face_id("base_flange(b,k,c3,b)").unwrap();
    // Folded: a leg along each line, as thick as the sheet on its outside.
    let [x, y, z] = extent(&part);
    assert!(x.could_be_equal(S::from_f64(1.5 + t)), "{x:?}");
    assert!(y.could_be_equal(S::from_f64(1.0 + t)), "{y:?}");
    assert!(z.could_be_equal(S::from_f64(0.8)), "{z:?}");

    let flat = unfold(part, "base_flange(b)");
    assert_valid(&flat);
    assert!(flat.solid_id("base_flange(b)").is_err());
    let s = S::from_f64;
    let developed = s(1.0 - r)
        .add(s(1.5 - r))
        .add(S::PI.div(S::TWO).unwrap().mul(s(r).add(s(k).mul(s(t)))));
    let [x, y, z] = extent(&flat);
    // Laid out along the first line, -y; the thickness along x.
    assert!(y.could_be_equal(developed), "{y:?} against {developed:?}");
    assert!(x.could_be_equal(s(t)), "{x:?}");
    assert!(z.could_be_equal(s(0.8)), "{z:?}");
    let data = flat
        .body_data::<FlatPatternData>("flat_pattern(fp)")
        .unwrap();
    assert_eq!(data.bends.len(), 1);
    assert_eq!(data.bends[0].bend, "base_flange(b,k,p1)");
    flat.sketch_id("flat_pattern(fp,bend_lines)").unwrap();
    flat.face_id("flat_pattern(fp,base_flange(b,k,p1,a))")
        .unwrap();
}

/// A chain with an arc between its lines bends along the arc, at its
/// radius: a U channel of two bends.
#[test]
fn u_channel_with_an_arc_bend() {
    let mut s = Sketch::new();
    let p = [[0.0, 1.0], [0.0, 0.3], [0.3, 0.0], [1.0, 0.0], [1.0, 1.0]]
        .map(|c| s.add_point(d(c[0]), d(c[1])));
    s.add_line(p[0], p[1]);
    // Counter-clockwise seen from +z, a quarter turn of radius 0.3 about (0.3, 0.3).
    s.add_arc(p[1], p[2], d(PI / 2.0));
    s.add_line(p[2], p[3]);
    s.add_line(p[3], p[4]);
    let part = base(sketched(s), rules(0.05, 0.1, 0.5), 0.5);
    assert_valid(&part);
    let sheet = part.body_data::<Sheet<S>>("base_flange(b)").unwrap();
    assert_eq!((sheet.flats.len(), sheet.bends.len()), (3, 2));
    assert!(sheet.bends[0].radius.could_be_equal(S::from_f64(0.3)));
    let flat = unfold(part, "base_flange(b)");
    assert_valid(&flat);
}

/// A plate with two flanges: one narrower than its edge, with reliefs
/// beside it, and one along the whole next edge, its corner set back. Both
/// bent bodies and the flat pattern are valid, and the flat pattern's
/// faces are as large as the bent ones — the bends as large as their
/// neutral surfaces.
#[test]
fn plate_with_two_flanges_and_reliefs_unfolds_area_for_area() {
    let (t, r, k) = (0.1, 0.1, 0.5);
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]])),
        rules(t, r, k),
        1.0,
    );
    assert_valid(&part);
    let mut first = flange("base_flange(b,k,c4,b)");
    first.offset_start = 0.4;
    first.offset_end = 0.5;
    let part = EdgeFlange.apply(part, "f1", &first, &NoFiles).unwrap();
    assert_valid(&part);
    part.face_id("edge_flange(f1,relief0,in)").unwrap();
    part.face_id("edge_flange(f1,relief1,across)").unwrap();
    let part = EdgeFlange
        .apply(part, "f2", &flange("base_flange(b,k,c5,b)"), &NoFiles)
        .unwrap();
    assert_valid(&part);
    let sheet = part
        .body_data::<Sheet<S>>("edge_flange(f2)")
        .unwrap()
        .clone();
    assert_eq!((sheet.flats.len(), sheet.bends.len()), (3, 2));
    // The flanges stand up along +z, the B side's way, half a unit high.
    let [_, _, z] = extent(&part);
    assert!(z.could_be_equal(S::from_f64(0.5)), "{z:?}");

    let folded = part.clone();
    let flat = unfold(part, "edge_flange(f2)");
    assert_valid(&flat);
    let [_, _, z] = extent(&flat);
    assert!(z.could_be_equal(S::from_f64(t)), "{z:?}");
    let relative = |a: f64, b: f64| ((a - b) / b).abs();
    for face in [
        "base_flange(b,plate,a)",
        "edge_flange(f1,flange,b)",
        "edge_flange(f2,flange,a)",
    ] {
        let (bent, laid) = (
            area(&folded, face),
            area(&flat, &format!("flat_pattern(fp,{face})")),
        );
        assert!(relative(laid, bent) < 1e-9, "{face}: {laid} against {bent}");
    }
    // Bent up, towards the B side: that is their inside.
    for bend in ["f1", "f2"] {
        let face = |side: &str| format!("edge_flange({bend},bend,{side})");
        let inner = area(&folded, &face("b"));
        let outer = area(&folded, &face("a"));
        let neutral = (1.0 - k) * inner + k * outer;
        let laid = area(&flat, &format!("flat_pattern(fp,{})", face("a")));
        assert!(
            relative(laid, neutral) < 1e-3,
            "{bend}: {laid} against {neutral}"
        );
    }
}

/// Two flanges along whole edges that meet at a corner: the second keeps
/// the corner gap from the first's bend, and the body stays valid.
#[test]
fn flanges_meeting_at_a_corner_keep_a_gap() {
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
        rules(0.1, 0.1, 0.44),
        1.0,
    );
    let part = EdgeFlange
        .apply(part, "f1", &flange("base_flange(b,k,c4,b)"), &NoFiles)
        .unwrap();
    assert_valid(&part);
    let part = EdgeFlange
        .apply(part, "f2", &flange("base_flange(b,k,c5,b)"), &NoFiles)
        .unwrap();
    assert_valid(&part);
    // The rest of the second edge beside the gap, and the step back to it.
    part.face_id("edge_flange(f2,before)").unwrap();
    let flat = unfold(part, "edge_flange(f2)");
    assert_valid(&flat);
}

/// A flange bent from a flange: a Z, turning the other way.
#[test]
fn flange_on_a_flange() {
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
        rules(0.1, 0.1, 0.44),
        1.0,
    );
    let part = EdgeFlange
        .apply(part, "f1", &flange("base_flange(b,k,c4,b)"), &NoFiles)
        .unwrap();
    let mut second = flange("edge_flange(f1,flange,end,a)");
    second.position = FlangePosition::BendOutside;
    second.angle = 60.0.into();
    let part = EdgeFlange.apply(part, "f2", &second, &NoFiles).unwrap();
    assert_valid(&part);
    let flat = unfold(part, "edge_flange(f2)");
    assert_valid(&flat);
}

/// What is not sheet metal is refused, by name: a flat pattern of a solid
/// without a record, a flange on a bent edge, a flange on a curved edge,
/// and a flat pattern of a sheet-metal body another step has changed.
#[test]
fn what_is_not_sheet_metal_is_refused() {
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
        rules(0.1, 0.1, 0.44),
        1.0,
    );
    let part = EdgeFlange
        .apply(part, "f1", &flange("base_flange(b,k,c4,b)"), &NoFiles)
        .unwrap();
    // The bend's own edge is no flat edge to bend.
    let err = refused(EdgeFlange.apply(
        part.clone(),
        "f2",
        &flange("edge_flange(f1,line,b)"),
        &NoFiles,
    ));
    assert!(err.contains("already bent"), "{err}");
    let err = refused(EdgeFlange.apply(
        part.clone(),
        "f2",
        &flange("edge_flange(f1,bend,s0,a)"),
        &NoFiles,
    ));
    assert!(err.contains("no edge of a sheet-metal body"), "{err}");

    // A plate with a round hole: its rim is no straight edge to bend.
    let mut s = polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let c = s.add_point(d(0.5), d(0.5));
    s.add_circle(c, d(0.2));
    let holed = base(sketched(s), rules(0.1, 0.1, 0.44), 1.0);
    assert_valid(&holed);
    let rim = holed.body_data::<Sheet<S>>("base_flange(b)").unwrap().flats[0].holes[0][0]
        .name
        .name(&["b"]);
    let err = refused(EdgeFlange.apply(holed, "f", &flange(&rim), &NoFiles));
    assert!(err.contains("no edge of a sheet-metal body"), "{err}");

    // A solid with no record of bends.
    let mut plain = part.clone();
    plain
        .rename(plain.solid_id("edge_flange(f1)").unwrap(), "other")
        .unwrap();
    let err = refused(FlatPattern.apply(
        plain,
        "fp",
        &FlatPatternArgs {
            solid: "other".into(),
            keep: false,
        },
        &NoFiles,
    ));
    assert!(err.contains("not a sheet-metal body"), "{err}");

    // Relief that does not fit beside a flange set in less than its width.
    let mut narrow = flange("base_flange(b,k,c6,b)");
    narrow.offset_start = 0.02;
    let err = refused(EdgeFlange.apply(part, "f3", &narrow, &NoFiles));
    assert!(err.contains("does not fit"), "{err}");
}

/// A tear cuts no notch, which only a bend outside the sheet leaves room
/// for; a set-back flange narrower than its edge is refused. Bend outside
/// sets nothing back, and an edge picked on the A side bends down.
#[test]
fn tear_beside_a_bend_outside() {
    let mut rules = rules(0.1, 0.1, 0.44);
    rules.relief = Relief::Tear;
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
        rules,
        1.0,
    );
    let mut args = flange("base_flange(b,k,c4,a)");
    args.offset_start = 0.2;
    args.offset_end = 0.2;
    let err = refused(EdgeFlange.apply(part.clone(), "f1", &args, &NoFiles));
    assert!(err.contains("needs a relief"), "{err}");
    args.position = FlangePosition::BendOutside;
    let part = EdgeFlange.apply(part, "f1", &args, &NoFiles).unwrap();
    assert_valid(&part);
    let sheet = part.body_data::<Sheet<S>>("edge_flange(f1)").unwrap();
    assert!(!sheet.bends[0].toward_b);
    let mut args = flange("base_flange(b,k,c6,b)");
    args.position = FlangePosition::BendOutside;
    args.reference = LengthReference::Tangent;
    let part = EdgeFlange.apply(part, "f2", &args, &NoFiles).unwrap();
    assert_valid(&part);
    assert_valid(&unfold(part, "edge_flange(f2)"));
}

/// Every combination of angle, radius and K-factor either builds a valid
/// flanged plate and L bracket, each unfolding to its developed length, or
/// is refused by name — a set-back too deep for the plate.
#[test]
#[ignore = "slow: sheet-metal sweep over angles, radii and K-factors — run with `cargo test -- --ignored`"]
fn sweep_angles_radii_and_k_factors() {
    let s = S::from_f64;
    let t = 0.1;
    let tan_half = |theta: S| {
        let half = theta.div(S::TWO).unwrap();
        half.sin().div(half.cos()).unwrap()
    };
    let mut built = 0;
    for angle in [15.0, 45.0, 90.0, 120.0, 170.0] {
        let theta = s(angle).mul(S::PI).div(s(180.0)).unwrap();
        for radius in [0.02, 0.1, 0.5] {
            let r = s(radius);
            for k in [0.0, 0.33, 0.5, 1.0] {
                let ctx = format!("angle {angle}, radius {radius}, k {k}");
                let neutral = theta.mul(r.add(s(k).mul(s(t))));

                // A flange set in from the start of a plate's edge.
                let part = base(
                    sketched(polygon(&[[0.0, 0.0], [3.0, 0.0], [3.0, 2.0], [0.0, 2.0]])),
                    rules(t, radius, k),
                    1.0,
                );
                let mut args = flange("base_flange(b,k,c4,b)");
                args.angle = angle.into();
                args.length = 1.5.into();
                args.offset_start = 0.5;
                match EdgeFlange.apply(part, "f1", &args, &NoFiles) {
                    Err(e) => {
                        let e = e.to_string();
                        assert!(
                            e.contains("cuts") || e.contains("leaves nothing flat"),
                            "{ctx}: {e}"
                        );
                    }
                    Ok(part) => {
                        built += 1;
                        assert_valid(&part);
                        let flat = unfold(part, "edge_flange(f1)");
                        assert_valid(&flat);
                        // Along -y, from the plate's far edge: the
                        // flange reaches out of the set-back line by the
                        // bend and its flat length — or not even past the
                        // plate's own edge beside it.
                        let setback = r.add(s(t)).mul(tan_half(theta));
                        let lowest = setback.sub(neutral).sub(s(1.5).sub(setback));
                        let developed = s(2.0).sub(lowest.min(S::ZERO));
                        let [_, y, _] = extent(&flat);
                        assert!(
                            y.could_be_equal(developed),
                            "{ctx}: {y:?} against {developed:?}"
                        );
                    }
                }

                // An L bracket drawn as a chain at the angle, turning left:
                // away from the material on its right.
                let (c, sn) = (angle.to_radians().cos(), angle.to_radians().sin());
                let part = sketched(chain(&[[-2.0, 0.0], [0.0, 0.0], [2.0 * c, 2.0 * sn]]));
                let args = BaseFlangeArgs {
                    sketch: "k".into(),
                    rules: rules(t, radius, k),
                    depth: 0.5.into(),
                    flip: false,
                };
                let part = match BaseFlange.apply(part, "b", &args, &NoFiles) {
                    Ok(part) => part,
                    Err(e) => {
                        // A sharp bend of a large radius sets back further
                        // than its legs are long.
                        assert!(
                            e.to_string().contains("too short for its bends"),
                            "{ctx}: {e}"
                        );
                        continue;
                    }
                };
                assert_valid(&part);
                let sheet = part.body_data::<Sheet<S>>("base_flange(b)").unwrap();
                let bend = &sheet.bends[0];
                assert!(!bend.toward_b, "{ctx}");
                assert!(bend.angle.could_be_equal(theta), "{ctx}: {:?}", bend.angle);
                let developed = s(4.0)
                    .sub(S::TWO.mul(r.mul(tan_half(bend.angle))))
                    .add(sheet.developed_length(bend));
                let flat = unfold(part, "base_flange(b)");
                assert_valid(&flat);
                let [x, _, _] = extent(&flat);
                assert!(
                    x.could_be_equal(developed),
                    "{ctx}: {x:?} against {developed:?}"
                );
            }
        }
    }
    // Only the sharpest bends at the largest radius are too deep to set
    // back into the plate.
    assert!(built >= 50, "{built} of 60 flanges built");
}

/// A bracket — a plate with two holes, a flange set in from both ends of
/// its front edge and one along its whole back edge — has mass properties
/// the quadrature resolves on every face, its bends' among them.
#[test]
fn flanged_bracket_mass_properties_converge() {
    let mut s = polygon(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.2], [0.0, 1.2]]);
    for x in [0.5, 1.5] {
        let c = s.add_point(d(x), d(0.6));
        s.add_circle(c, d(0.15));
    }
    let part = base(sketched(s), rules(0.08, 0.08, 0.44), 1.0);
    let mut front = flange("base_flange(b,k,c4,b)");
    front.length = 0.6.into();
    front.offset_start = 0.3;
    front.offset_end = 0.3;
    let part = EdgeFlange.apply(part, "front", &front, &NoFiles).unwrap();
    let mut back = flange("base_flange(b,k,c6,b)");
    back.length = 0.4.into();
    let part = EdgeFlange.apply(part, "back", &back, &NoFiles).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("edge_flange(back)").unwrap();
    let mass = part.topology().mass_properties(solid).unwrap();
    assert!(mass.converged, "{mass:?}");
}

/// `part` with the sketch `name` placed on `plane`.
fn with_sketch(
    mut part: Part<S>,
    name: &str,
    plane: CoordinateSystem<S>,
    sketch: Sketch<Design>,
) -> Part<S> {
    part.add_sketch(PlacedSketch { plane, sketch }, name)
        .unwrap();
    part
}

/// The plane at `origin` spanned by `u` and `v`.
fn plane(origin: [f64; 3], u: [f64; 3], v: [f64; 3]) -> CoordinateSystem<S> {
    let p = |c: [f64; 3]| Vector3::from_array(c.map(S::from_f64));
    let (u, v) = (p(u), p(v));
    CoordinateSystem::try_new(p(origin), u, v, u.prod_cross(&v)).unwrap()
}

/// Circles of radius `r` about `centers`.
fn circles(centers: &[[f64; 2]], r: f64) -> Sketch<Design> {
    let mut s = Sketch::new();
    for c in centers {
        let p = s.add_point(d(c[0]), d(c[1]));
        s.add_circle(p, d(r));
    }
    s
}

fn cut(part: Part<S>, id: &str, sketch: &str, face: &str) -> GeopResult<Part<S>> {
    let args = SheetCutArgs {
        sketch: sketch.into(),
        face: face.into(),
    };
    SheetCut.apply(part, id, &args, &NoFiles)
}

/// The centres of the circles whose arcs make the A-side edges named
/// `prefix...,a))` — of a flat pattern — each once.
fn arc_centers(part: &Part<S>, prefix: &str) -> Vec<[f64; 2]> {
    let mut centers: Vec<[f64; 2]> = Vec::new();
    for (id, name) in part.names().iter() {
        let geop_ops::RefId::Edge(e) = id else {
            continue;
        };
        if !(name.starts_with(prefix) && name.ends_with(",a))")) {
            continue;
        }
        let curve = &part.topology().get_edge(e).unwrap().curve;
        let Some(arc) = curve.as_arc().unwrap() else {
            continue;
        };
        let c = [arc.circle.center[0].to_f64(), arc.circle.center[1].to_f64()];
        if !centers
            .iter()
            .any(|o| (o[0] - c[0]).abs() + (o[1] - c[1]).abs() < 1e-9)
        {
            centers.push(c);
        }
    }
    centers.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    centers
}

/// The bracket of the examples, flanged first: a plate 2 by 1.2, a flange
/// set in from both ends of its front edge and one along its whole back
/// edge, 0.4 high.
fn flanged_bracket() -> Part<S> {
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.2], [0.0, 1.2]])),
        rules(0.08, 0.08, 0.44),
        1.0,
    );
    let mut front = flange("base_flange(b,k,c4,b)");
    front.length = 0.6.into();
    front.offset_start = 0.3;
    front.offset_end = 0.3;
    let part = EdgeFlange.apply(part, "front", &front, &NoFiles).unwrap();
    let mut back = flange("base_flange(b,k,c6,b)");
    back.length = 0.4.into();
    EdgeFlange.apply(part, "back", &back, &NoFiles).unwrap()
}

/// Mounting holes cut after flanging — through the plate and through the
/// back flange, each sketched on the face it goes through — leave a valid
/// body that still unfolds, and the flat pattern has each hole where it
/// belongs: the plate's where it was drawn, the flange's as far from the
/// bend as it is, along the flange, from where the bend ends.
#[test]
fn holes_cut_after_flanging_unfold_into_place() {
    let (t, r, k) = (0.08, 0.08, 0.44);
    let part = flanged_bracket();
    // On the plate's top, its B side.
    let part = with_sketch(
        part,
        "h",
        plane([0.0, 0.0, t], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        circles(&[[0.5, 0.6]], 0.15),
    );
    let part = cut(part, "c1", "h", "").unwrap();
    // On the back flange's outside, at y = 1.2, seen from behind: 0.3 up.
    let part = with_sketch(
        part,
        "f",
        plane([0.0, 1.2, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        circles(&[[-1.0, 0.3]], 0.05),
    );
    let part = cut(part, "c2", "f", "edge_flange(back,flange,a)").unwrap();
    assert_valid(&part);
    let sheet = part.body_data::<Sheet<S>>("sheet_cut(c2)").unwrap();
    assert_eq!(sheet.cuts.len(), 2);
    part.face_id("sheet_cut(c2,f,c1)").unwrap();

    let flat = unfold(part, "sheet_cut(c2)");
    assert_valid(&flat);
    let plate = arc_centers(&flat, "flat_pattern(fp,sheet_cut(c1,");
    assert_eq!(plate.len(), 1, "{plate:?}");
    assert!(
        (plate[0][0] - 0.5).abs() + (plate[0][1] - 0.6).abs() < 1e-12,
        "{plate:?}"
    );
    // The back bend starts where the plate is set back by `r + t`, is as
    // wide as its developed length, and the flange's flat starts at the
    // height `t + r` the bend ends at.
    let developed = PI / 2.0 * (r + k * t);
    let want = [1.0, 1.2 - (r + t) + developed + (0.3 - (t + r))];
    let flange = arc_centers(&flat, "flat_pattern(fp,sheet_cut(c2,");
    assert_eq!(flange.len(), 1, "{flange:?}");
    assert!(
        (flange[0][0] - want[0]).abs() + (flange[0][1] - want[1]).abs() < 1e-12,
        "{flange:?} against {want:?}"
    );
}

/// A slot cut from the plate across the back bend into its flange, and a
/// hole on the line the bend starts at, are cut through the bend unrolled:
/// the body is valid, and laid flat both are exactly as drawn.
#[test]
fn cuts_across_a_bend_are_unrolled() {
    let t = 0.08;
    let part = flanged_bracket();
    let mut s = polygon(&[[0.8, 0.9], [1.2, 0.9], [1.2, 1.3], [0.8, 1.3]]);
    let c = s.add_point(d(0.4), d(1.1));
    s.add_circle(c, d(0.1));
    let part = with_sketch(
        part,
        "x",
        plane([0.0, 0.0, t], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        s,
    );
    let part = cut(part, "c", "x", "base_flange(b,plate,b)").unwrap();
    assert_valid(&part);
    let flat = unfold(part, "sheet_cut(c)");
    assert_valid(&flat);
    let hole = arc_centers(&flat, "flat_pattern(fp,sheet_cut(c,");
    assert_eq!(hole.len(), 1, "{hole:?}");
    assert!(
        (hole[0][0] - 0.4).abs() + (hole[0][1] - 1.1).abs() < 1e-12,
        "{hole:?}"
    );
    // The slot's corners, where they were drawn.
    for corner in [[0.8, 0.9], [1.2, 0.9], [1.2, 1.3], [0.8, 1.3]] {
        let found = flat.topology().vertices.values().any(|v| {
            (v.point[0].to_f64() - corner[0]).abs() + (v.point[1].to_f64() - corner[1]).abs()
                < 1e-12
                && v.point[2].to_f64().abs() < 1e-12
        });
        assert!(found, "no corner at {corner:?}");
    }
}

/// A notch over the plate's free left edge splits that edge in two, and a
/// flange is then refused on it; the cuts that cannot be made are refused
/// by name.
#[test]
fn notches_and_refused_cuts() {
    let t = 0.1;
    let square = || {
        base(
            sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
            rules(t, 0.1, 0.44),
            1.0,
        )
    };
    let top = || plane([0.0, 0.0, t], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let notch = polygon(&[[-0.1, 0.4], [0.2, 0.4], [0.2, 0.6], [-0.1, 0.6]]);
    let part = cut(with_sketch(square(), "n", top(), notch), "c", "n", "").unwrap();
    assert_valid(&part);
    part.face_id("base_flange(b,k,c7,0)").unwrap();
    part.face_id("base_flange(b,k,c7,2)").unwrap();
    assert!(part.face_id("base_flange(b,k,c7,1)").is_err());
    part.face_id("sheet_cut(c,n,c5)").unwrap();
    let err = refused(EdgeFlange.apply(
        part.clone(),
        "f",
        &flange("base_flange(b,k,c7,b)"),
        &NoFiles,
    ));
    assert!(err.contains("has been cut"), "{err}");
    assert_valid(&unfold(part, "sheet_cut(c)"));

    // A corner on the outline.
    let corner = polygon(&[[0.0, 0.4], [0.2, 0.4], [0.2, 0.6], [0.0, 0.6]]);
    let err = refused(cut(with_sketch(square(), "n", top(), corner), "c", "n", ""));
    assert!(err.contains("at an end"), "{err}");
    // Right across: two pieces.
    let across = polygon(&[[0.4, -0.1], [0.6, -0.1], [0.6, 1.1], [0.4, 1.1]]);
    let err = refused(cut(with_sketch(square(), "n", top(), across), "c", "n", ""));
    assert!(err.contains("splits face"), "{err}");
    // Off the sheet.
    let off = circles(&[[3.0, 3.0]], 0.2);
    let err = refused(cut(with_sketch(square(), "n", top(), off), "c", "n", ""));
    assert!(err.contains("off the sheet"), "{err}");
    // Not parallel.
    let side = plane([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    let err = refused(cut(
        with_sketch(square(), "n", side, circles(&[[0.5, 0.5]], 0.1)),
        "c",
        "n",
        "base_flange(b,plate,b)",
    ));
    assert!(err.contains("not parallel"), "{err}");
}

/// A closed hem and an open one fold a plate's edges right back over it:
/// valid bodies, as high as the fold — two thicknesses and the gap — and
/// laid flat as long as the plate short of the fold, the half turn on the
/// neutral surface, and the hem's flat.
#[test]
fn hems_fold_back_and_unfold_to_their_developed_length() {
    let (t, r, k) = (0.1, 0.1, 0.44);
    let square = || {
        base(
            sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
            rules(t, r, k),
            1.0,
        )
    };
    let hem = |kind, gap| HemArgs {
        edge: "base_flange(b,k,c4,b)".into(),
        kind,
        length: 0.3,
        gap,
        offset_start: 0.0,
        offset_end: 0.0,
    };
    for (kind, gap, inner) in [(HemKind::Closed, 0.0, r), (HemKind::Open, 0.06, 0.03)] {
        let part = Hem.apply(square(), "h", &hem(kind, gap), &NoFiles).unwrap();
        assert_valid(&part);
        let sheet = part.body_data::<Sheet<S>>("hem(h)").unwrap();
        assert!(sheet.bends[0].angle.could_be_equal(S::PI));
        // Between vertices: the fold's outside, flush with the old edge,
        // is no vertex.
        let [x, y, z] = extent(&part);
        assert!(x.could_be_equal(S::from_f64(1.0)), "{x:?}");
        assert!(y.could_be_equal(S::from_f64(1.0 - (t + inner))), "{y:?}");
        assert!(
            z.could_be_equal(S::from_f64(2.0 * (t + inner))),
            "{kind:?}: {z:?}"
        );
        let flat = unfold(part, "hem(h)");
        assert_valid(&flat);
        let s = S::from_f64;
        let fold = s(inner + t);
        let developed = s(1.0)
            .sub(fold)
            .add(S::PI.mul(s(inner).add(s(k * t))))
            .add(s(0.3).sub(fold));
        let [_, y, _] = extent(&flat);
        assert!(
            y.could_be_equal(developed),
            "{kind:?}: {y:?} against {developed:?}"
        );
    }
    let err = refused(Hem.apply(square(), "h", &hem(HemKind::Open, 0.0), &NoFiles));
    assert!(err.contains("gap must be positive"), "{err}");
    let mut short = hem(HemKind::Closed, 0.0);
    short.length = 0.2;
    let err = refused(Hem.apply(square(), "h", &short, &NoFiles));
    assert!(err.contains("leaves nothing flat"), "{err}");
}

/// A second flange closing its corner with the first: its bend keeps the
/// corner gap from the first's, its flat reaches on to the gap from the
/// first flange's inside. Valid folded and laid flat; a closed corner of
/// two radii is refused.
#[test]
fn a_closed_corner_reaches_to_the_flange_beside_it() {
    let (t, r) = (0.1, 0.1);
    let part = base(
        sketched(polygon(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])),
        rules(t, r, 0.44),
        1.0,
    );
    let part = EdgeFlange
        .apply(part, "f1", &flange("base_flange(b,k,c4,b)"), &NoFiles)
        .unwrap();
    let mut second = flange("base_flange(b,k,c5,b)");
    second.corner = Corner::Closed;
    let mut other_radius = second.clone();
    other_radius.radius = Some(0.05);
    let err = refused(EdgeFlange.apply(part.clone(), "f2", &other_radius, &NoFiles));
    assert!(err.contains("radii differ"), "{err}");
    let part = EdgeFlange.apply(part, "f2", &second, &NoFiles).unwrap();
    assert_valid(&part);
    part.face_id("edge_flange(f2,flange,corner0)").unwrap();
    // The first flange's inside is a thickness in from the plate's front
    // edge; the second's flat stops the corner gap short of it.
    let gap = SheetMetalRules::default().corner_gap;
    let reach = part.topology().vertices.values().any(|v| {
        v.point[0].could_be_equal(S::from_f64(1.0))
            && v.point[1].could_be_equal(S::from_f64(t + gap))
            && v.point[2].could_be_equal(S::from_f64(t + r))
    });
    assert!(reach, "the second flange's flat does not reach the first");
    assert_valid(&unfold(part, "edge_flange(f2)"));
}

/// A DXF file read back: every entity of its `ENTITIES` section as its
/// type, layer and text, and the layers its table declares.
fn read_dxf(dxf: &str) -> (Vec<(String, String, String)>, Vec<String>) {
    let lines: Vec<&str> = dxf.lines().collect();
    assert!(
        lines.len().is_multiple_of(2),
        "a DXF file is pairs of lines"
    );
    let pairs: Vec<(i32, &str)> = lines
        .chunks(2)
        .map(|p| (p[0].trim().parse().unwrap(), p[1]))
        .collect();
    assert_eq!(pairs.last().unwrap(), &(0, "EOF"));
    let mut layers = Vec::new();
    let mut in_layer = false;
    let mut entities: Vec<(String, String, String)> = Vec::new();
    let mut in_entities = false;
    for &(code, value) in &pairs {
        match (code, value) {
            (2, "ENTITIES") => in_entities = true,
            (0, "ENDSEC") => in_entities = false,
            (0, "LAYER") => in_layer = true,
            (2, name) if in_layer => {
                layers.push(name.to_string());
                in_layer = false;
            }
            (0, kind) if in_entities => {
                entities.push((kind.to_string(), String::new(), String::new()))
            }
            (8, layer) if in_entities => entities.last_mut().unwrap().1 = layer.to_string(),
            (1, text) if in_entities => entities.last_mut().unwrap().2 = text.to_string(),
            _ => {}
        }
    }
    (entities, layers)
}

/// The bracket with a hole cut through its plate and one through its back
/// flange, laid out for laser cutting as DXF: its outline and both holes on
/// the CUT layer — lines and circles, nothing approximated — its two bend
/// lines on the BEND layer, each with how it is bent; and only those two
/// layers declared. The flat pattern step records the same file.
#[test]
fn flat_pattern_dxf_reads_back_by_layer() {
    let t = 0.08;
    let part = with_sketch(
        flanged_bracket(),
        "h",
        plane([0.0, 0.0, t], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        circles(&[[0.5, 0.6]], 0.15),
    );
    let part = cut(part, "c1", "h", "").unwrap();
    let part = with_sketch(
        part,
        "f",
        plane([0.0, 1.2, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        circles(&[[-1.0, 0.3]], 0.05),
    );
    let part = cut(part, "c2", "f", "edge_flange(back,flange,a)").unwrap();
    let (body, dxf) = flat_pattern_dxf(&part, None).unwrap();
    assert_eq!(body, "sheet_cut(c2)");
    let (entities, layers) = read_dxf(&dxf);
    assert_eq!(layers, ["CUT", "BEND"]);
    let count = |kind: &str, layer: &str| {
        entities
            .iter()
            .filter(|(k, l, _)| k == kind && l == layer)
            .count()
    };
    assert_eq!(count("CIRCLE", "CUT"), 2, "{entities:?}");
    // The outline, each side one line where plate, bend and flange run on
    // in one: left, right and the back flange's end; along the front, the
    // plate either side, the reliefs' three sides each — their outer sides
    // running on into the front flange's — and the front flange's end.
    assert_eq!(count("LINE", "CUT"), 12, "{entities:?}");
    assert_eq!(count("LINE", "BEND"), 2, "{entities:?}");
    let notes: Vec<&str> = entities
        .iter()
        .filter(|(k, l, _)| k == "TEXT" && l == "BEND")
        .map(|(_, _, t)| t.as_str())
        .collect();
    assert_eq!(notes, ["UP 90%%d R0.08", "UP 90%%d R0.08"]);
    assert_eq!(entities.len(), 18, "{entities:?}");

    let flat = unfold(part, "sheet_cut(c2)");
    let (_, recorded) = flat_pattern_dxf(&flat, Some("flat_pattern(fp)")).unwrap();
    assert_eq!(recorded, dxf);
}
