//! Fast tests: STEP texts written here, read into valid bodies; parts
//! written and read back the same.

use geop_core_math::{
    scalars::Ring,
    scalars::{Scalar, scal_in_f64::ScalInF64},
    vector::Vector3,
};
use geop_core_topology::{
    Model,
    validation::{ValidationParameters, validate},
};
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::shapes::{
    cube::cube_solid, cylinder::revolved_cylinder, sphere::sphere_solid,
};

use crate::{add_bodies, read_step, write_step};

type S = ScalInF64;

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// The part the STEP text `text` imports into, checked valid.
fn import(text: &str) -> Part<S> {
    let bodies = read_step::<S>(text).unwrap_or_else(|e| panic!("reading failed: {e}"));
    let mut part = Part::new();
    add_bodies(&mut part, &Namer::new("import", "i").unwrap(), bodies)
        .unwrap_or_else(|e| panic!("adding failed: {e}"));
    assert_valid(part.topology());
    part
}

fn assert_valid(model: &Model<S>) {
    if let Err(errors) = validate(&ValidationParameters::default(), model) {
        let shown: Vec<String> = errors.iter().map(|e| e.to_string()).collect();
        panic!("invalid model:\n{}", shown.join("\n"));
    }
}

/// `(faces, edges, vertices)` of a model.
fn counts(model: &Model<S>) -> (usize, usize, usize) {
    (model.faces.len(), model.edges.len(), model.vertices.len())
}

/// The box around a model's edges, sampled.
fn bounds(model: &Model<S>) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for edge in model.edges.values() {
        let (t0, t1) = edge.curve.domain();
        for i in 0..=256 {
            let t = t0.to_f64() + (t1.to_f64() - t0.to_f64()) * i as f64 / 256.0;
            let p = edge.curve.evaluate(S::from_f64(t)).unwrap();
            for c in 0..3 {
                lo[c] = lo[c].min(p[c].to_f64());
                hi[c] = hi[c].max(p[c].to_f64());
            }
        }
    }
    (lo, hi)
}

fn assert_bounds(model: &Model<S>, lo: [f64; 3], hi: [f64; 3]) {
    let (a, b) = bounds(model);
    for c in 0..3 {
        // Sampled: an arc's extreme may fall between samples.
        let tol = 1e-4 * (hi[c] - lo[c]).abs().max(1.0);
        assert!(
            (a[c] - lo[c]).abs() < tol && (b[c] - hi[c]).abs() < tol,
            "bounds {a:?} {b:?}, expected {lo:?} {hi:?}"
        );
    }
}

/// `part` written as STEP and read back: the same counts and bounds.
fn round_trip(part: &Part<S>) -> Part<S> {
    let text = write_step(part, "part").unwrap();
    let back = import(&text);
    assert_eq!(
        counts(back.topology()),
        counts(part.topology()),
        "counts changed"
    );
    let (lo, hi) = bounds(part.topology());
    assert_bounds(back.topology(), lo, hi);
    back
}

#[test]
fn a_cube_round_trips() {
    let mut part = Part::new();
    cube_solid(&mut part, "c", v(0.0, 0.0, 0.0), v(2.0, 1.0, 3.0)).unwrap();
    let back = round_trip(&part);
    assert_eq!(back.solid_names(), vec!["import(i,s0)".to_string()]);
}

#[test]
fn a_cylinder_round_trips() {
    let mut part = Part::new();
    revolved_cylinder(
        &mut part,
        "c",
        v(1.0, 2.0, 0.0),
        S::from_f64(0.5),
        S::from_f64(2.0),
    )
    .unwrap();
    round_trip(&part);
}

#[test]
fn a_sphere_round_trips() {
    let mut part = Part::new();
    sphere_solid(&mut part, "s", v(0.0, 0.0, 1.0), S::from_f64(1.5)).unwrap();
    round_trip(&part);
}

/// A STEP file around the body entities `body` — instances from `#100`
/// on, the solid `#999` — in millimetres, or in `unit` (an SI unit's
/// prefix and name, as `.MILLI.,.METRE.`).
fn file(unit: &str, body: &str) -> String {
    format!(
        "ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('test','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));
ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT({unit}));
#2=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#3=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','');
#4=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#3))GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2))REPRESENTATION_CONTEXT('',''));
#5=CARTESIAN_POINT('',(0.,0.,0.));
#6=DIRECTION('',(0.,0.,1.));
#7=DIRECTION('',(1.,0.,0.));
#8=AXIS2_PLACEMENT_3D('',#5,#6,#7);
#9=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#8,#999),#4);
{body}
ENDSEC;
END-ISO-10303-21;
"
    )
}

/// A cylinder of radius 1 and height 2 as a CAD system writes it: an
/// analytic surface going all the way round along a seam, its rims each one
/// closed circle.
const CYLINDER: &str = "
#100=CARTESIAN_POINT('',(0.,0.,2.));
#101=AXIS2_PLACEMENT_3D('',#100,#6,#7);
#110=CYLINDRICAL_SURFACE('',#8,1.);
#111=PLANE('',#8);
#112=PLANE('',#101);
#120=CARTESIAN_POINT('',(1.,0.,0.));
#121=VERTEX_POINT('',#120);
#122=CARTESIAN_POINT('',(1.,0.,2.));
#123=VERTEX_POINT('',#122);
#130=CIRCLE('',#8,1.);
#131=CIRCLE('',#101,1.);
#132=VECTOR('',#6,1.);
#133=LINE('',#120,#132);
#140=EDGE_CURVE('',#121,#121,#130,.T.);
#141=EDGE_CURVE('',#123,#123,#131,.T.);
#142=EDGE_CURVE('',#121,#123,#133,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#142,.T.);
#152=ORIENTED_EDGE('',*,*,#141,.F.);
#153=ORIENTED_EDGE('',*,*,#142,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_BOUND('',#154,.T.);
#156=ADVANCED_FACE('side',(#155),#110,.T.);
#160=ORIENTED_EDGE('',*,*,#140,.F.);
#161=EDGE_LOOP('',(#160));
#162=FACE_OUTER_BOUND('',#161,.T.);
#163=ADVANCED_FACE('bottom',(#162),#111,.F.);
#170=ORIENTED_EDGE('',*,*,#141,.T.);
#171=EDGE_LOOP('',(#170));
#172=FACE_OUTER_BOUND('',#171,.T.);
#173=ADVANCED_FACE('top',(#172),#112,.T.);
#180=CLOSED_SHELL('',(#156,#163,#173));
#999=MANIFOLD_SOLID_BREP('cylinder',#180);";

#[test]
fn a_cylinder_along_a_seam_is_cut_into_sectors() {
    let part = import(&file(".MILLI.,.METRE.", CYLINDER));
    let model = part.topology();
    // The side in two sectors along two meridians; each rim in three: cut
    // twice, and at the seam's vertex.
    assert_eq!(counts(model), (4, 8, 6));
    assert_bounds(model, [-1.0, -1.0, 0.0], [1.0, 1.0, 2.0]);
    // The sectors' rims, in six pieces, are parallels and their sides,
    // four, meridians: straight pcurves, written down rather than fitted.
    // (The discs' arcs are arcs in the plane.)
    let straight = model
        .coedges
        .values()
        .filter(|c| c.pcurve.degree == 1 && c.pcurve.control_points.len() == 2)
        .count();
    assert_eq!(straight, 10);
    assert!(part.face_id("import(i,s0,f0,q0)").is_ok());
    assert!(part.face_id("import(i,s0,f2)").is_ok());
}

/// A quarter of a pipe bend: a torus' tube of radius 2 round an axis 10
/// away, from the plane `y = 0` round to `x = 0`, closed by a disc at
/// either end — the tube face going all the way round its tube along a
/// seam on its outer equator.
const PIPE_BEND: &str = "
#100=TOROIDAL_SURFACE('',#8,10.,2.);
#101=CARTESIAN_POINT('',(10.,0.,0.));
#102=DIRECTION('',(0.,1.,0.));
#103=AXIS2_PLACEMENT_3D('',#101,#102,#7);
#104=CARTESIAN_POINT('',(0.,10.,0.));
#105=DIRECTION('',(0.,1.,0.));
#106=AXIS2_PLACEMENT_3D('',#104,#7,#105);
#107=DIRECTION('',(0.,-1.,0.));
#108=AXIS2_PLACEMENT_3D('',#101,#107,#7);
#109=DIRECTION('',(-1.,0.,0.));
#110=AXIS2_PLACEMENT_3D('',#104,#109,#105);
#111=PLANE('',#108);
#112=PLANE('',#110);
#120=CARTESIAN_POINT('',(12.,0.,0.));
#121=VERTEX_POINT('',#120);
#122=CARTESIAN_POINT('',(0.,12.,0.));
#123=VERTEX_POINT('',#122);
#130=CIRCLE('',#8,12.);
#131=CIRCLE('',#103,2.);
#132=CIRCLE('',#106,2.);
#140=EDGE_CURVE('',#121,#123,#130,.T.);
#141=EDGE_CURVE('',#121,#121,#131,.T.);
#142=EDGE_CURVE('',#123,#123,#132,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#142,.T.);
#152=ORIENTED_EDGE('',*,*,#140,.F.);
#153=ORIENTED_EDGE('',*,*,#141,.T.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_BOUND('',#154,.T.);
#156=ADVANCED_FACE('tube',(#155),#100,.T.);
#160=ORIENTED_EDGE('',*,*,#141,.F.);
#161=EDGE_LOOP('',(#160));
#162=FACE_OUTER_BOUND('',#161,.T.);
#163=ADVANCED_FACE('start',(#162),#111,.T.);
#170=ORIENTED_EDGE('',*,*,#142,.F.);
#171=EDGE_LOOP('',(#170));
#172=FACE_OUTER_BOUND('',#171,.T.);
#173=ADVANCED_FACE('end',(#172),#112,.T.);
#180=CLOSED_SHELL('',(#156,#163,#173));
#999=MANIFOLD_SOLID_BREP('bend',#180);";

#[test]
fn a_pipe_bend_is_cut_along_parallels() {
    let part = import(&file(".MILLI.,.METRE.", PIPE_BEND));
    let model = part.topology();
    assert_eq!(model.solids.len(), 1);
    // The tube in pieces between parallels, named after it; the discs
    // whole.
    assert!(part.face_id("import(i,s0,f0,q0)").is_ok());
    assert!(part.face_id("import(i,s0,f0,q1)").is_ok());
    assert!(part.face_id("import(i,s0,f0)").is_err());
    assert!(part.edge_id("import(i,s0,f0,m0)").is_ok());
    assert!(part.face_id("import(i,s0,f1)").is_ok());
    assert_bounds(model, [0.0, 0.0, -2.0], [12.0, 12.0, 2.0]);
}

/// A whole torus — an O-ring's — as one face along two seams, a circle round
/// the tube and the outer equator, meeting at one vertex.
const TORUS: &str = "
#100=TOROIDAL_SURFACE('',#8,10.,2.);
#101=CARTESIAN_POINT('',(10.,0.,0.));
#102=DIRECTION('',(0.,1.,0.));
#103=AXIS2_PLACEMENT_3D('',#101,#102,#7);
#120=CARTESIAN_POINT('',(12.,0.,0.));
#121=VERTEX_POINT('',#120);
#130=CIRCLE('',#8,12.);
#131=CIRCLE('',#103,2.);
#140=EDGE_CURVE('',#121,#121,#130,.T.);
#141=EDGE_CURVE('',#121,#121,#131,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#141,.T.);
#152=ORIENTED_EDGE('',*,*,#140,.F.);
#153=ORIENTED_EDGE('',*,*,#141,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_OUTER_BOUND('',#154,.T.);
#156=ADVANCED_FACE('ring',(#155),#100,.T.);
#180=CLOSED_SHELL('',(#156));
#999=MANIFOLD_SOLID_BREP('o-ring',#180);";

#[test]
fn a_whole_torus_is_cut_into_bands_and_sectors() {
    let part = import(&file(".MILLI.,.METRE.", TORUS));
    let model = part.topology();
    assert_eq!(model.solids.len(), 1);
    // Bands between parallels, each in sectors between meridians.
    assert!(part.face_id("import(i,s0,f0,b0,q0)").is_ok());
    assert!(part.face_id("import(i,s0,f0,b1,q0)").is_ok());
    assert!(part.vertex_id("import(i,s0,f0,m0,v)").is_ok());
    assert_bounds(model, [-12.0, -12.0, -2.0], [12.0, 12.0, 2.0]);
}

/// An apple: a torus whose tube, of radius 2, crosses its axis 1 away — the
/// outside of the circle turned, from where it meets the axis below to where
/// it meets it above, as one face along a seam.
const APPLE: &str = "
#100=TOROIDAL_SURFACE('',#8,1.,2.);
#101=CARTESIAN_POINT('',(1.,0.,0.));
#102=DIRECTION('',(0.,1.,0.));
#103=AXIS2_PLACEMENT_3D('',#101,#102,#7);
#120=CARTESIAN_POINT('',(0.,0.,-1.7320508075688772));
#121=VERTEX_POINT('',#120);
#122=CARTESIAN_POINT('',(0.,0.,1.7320508075688772));
#123=VERTEX_POINT('',#122);
#130=CIRCLE('',#103,2.);
#140=EDGE_CURVE('',#121,#123,#130,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#140,.F.);
#154=EDGE_LOOP('',(#150,#151));
#155=FACE_OUTER_BOUND('',#154,.T.);
#156=ADVANCED_FACE('apple',(#155),#100,.T.);
#180=CLOSED_SHELL('',(#156));
#999=MANIFOLD_SOLID_BREP('apple',#180);";

#[test]
fn a_torus_crossing_its_axis_is_read_between_its_poles() {
    let part = import(&file(".MILLI.,.METRE.", APPLE));
    let model = part.topology();
    assert_eq!(model.solids.len(), 1);
    assert!(part.face_id("import(i,s0,f0,q1)").is_ok());
    // Its edges: the meridians it is cut along, at angles 0 and a half turn,
    // through the circle's highest and lowest points.
    assert_bounds(model, [-3.0, 0.0, -2.0], [3.0, 0.0, 2.0]);
}

/// A quarter of a ball of radius 1, as the faces of a SolidWorks part had
/// one: the half above `z = 0`, cut by a plane through the centre turned 12°
/// from `x = 0`. The sphere's axis is `y`, and its pole `(0, 1, 0)` lies
/// inside the rim the ball has on `z = 0` — not at a vertex.
const QUARTER_BALL: &str = "
#100=DIRECTION('',(0.,1.,0.));
#101=AXIS2_PLACEMENT_3D('',#5,#100,#7);
#102=SPHERICAL_SURFACE('',#101,1.);
#103=DIRECTION('',(0.9781476007338057,-0.20791169081775934,0.));
#104=DIRECTION('',(0.20791169081775934,0.9781476007338057,0.));
#105=AXIS2_PLACEMENT_3D('',#5,#103,#104);
#106=PLANE('',#105);
#107=PLANE('',#8);
#110=CARTESIAN_POINT('',(0.20791169081775934,0.9781476007338057,0.));
#111=VERTEX_POINT('',#110);
#112=CARTESIAN_POINT('',(-0.20791169081775934,-0.9781476007338057,0.));
#113=VERTEX_POINT('',#112);
#120=VECTOR('',#104,1.);
#121=LINE('',#112,#120);
#122=CIRCLE('',#8,1.);
#123=CIRCLE('',#105,1.);
#130=EDGE_CURVE('',#113,#111,#121,.T.);
#131=EDGE_CURVE('',#111,#113,#122,.T.);
#132=EDGE_CURVE('',#111,#113,#123,.T.);
#140=ORIENTED_EDGE('',*,*,#130,.F.);
#141=ORIENTED_EDGE('',*,*,#131,.F.);
#142=EDGE_LOOP('',(#140,#141));
#143=FACE_OUTER_BOUND('',#142,.T.);
#144=ADVANCED_FACE('bottom',(#143),#107,.F.);
#150=ORIENTED_EDGE('',*,*,#132,.T.);
#151=ORIENTED_EDGE('',*,*,#130,.T.);
#152=EDGE_LOOP('',(#150,#151));
#153=FACE_OUTER_BOUND('',#152,.T.);
#154=ADVANCED_FACE('cut',(#153),#106,.T.);
#160=ORIENTED_EDGE('',*,*,#131,.T.);
#161=ORIENTED_EDGE('',*,*,#132,.F.);
#162=EDGE_LOOP('',(#160,#161));
#163=FACE_OUTER_BOUND('',#162,.T.);
#164=ADVANCED_FACE('ball',(#163),#102,.T.);
#170=CLOSED_SHELL('',(#144,#154,#164));
#999=MANIFOLD_SOLID_BREP('quarter ball',#170);";

/// A sphere's axis is a free choice: where the file's pole lies on an edge
/// of a face, away from its ends, the face is built about another axis.
#[test]
fn a_ball_whose_rim_runs_through_its_pole_imports() {
    let part = import(&file(".MILLI.,.METRE.", QUARTER_BALL));
    let model = part.topology();
    assert_eq!(model.solids.len(), 1);
    let (s, c) = (0.20791169081775934, 0.9781476007338057);
    assert_bounds(model, [-1.0, -c, 0.0], [s, 1.0, 1.0]);
}

/// A helix of radius 1 and pitch 1 turning 1.5 times, from `(1, 0, z0)`, as
/// STEP entities from `#first` on: the kernel's own, exactly on its
/// cylinder. The id of the curve is `#first`.
fn helix_entities(first: u64, z0: f64) -> String {
    use geop_core_geometry::nurb_curve::{Handedness, NurbCurve3D};
    use geop_core_math::primitives::CoordinateSystem;
    let helix = NurbCurve3D::<S>::helix(
        &CoordinateSystem::world_at(v(0.0, 0.0, z0)),
        S::ONE,
        S::ONE,
        1.5,
        Handedness::Right,
    )
    .unwrap();
    let mut text = String::new();
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for (k, cp) in helix.control_points.iter().enumerate() {
        let w = cp[3].to_f64();
        let id = first + 1 + k as u64;
        text += &format!(
            "#{id}=CARTESIAN_POINT('',({:?},{:?},{:?}));\n",
            cp[0].to_f64() / w,
            cp[1].to_f64() / w,
            cp[2].to_f64() / w
        );
        points.push(format!("#{id}"));
        weights.push(format!("{w:?}"));
    }
    let mut knots: Vec<(f64, usize)> = Vec::new();
    for k in &helix.knot_vector {
        match knots.last_mut() {
            Some((last, n)) if *last == k.to_f64() => *n += 1,
            _ => knots.push((k.to_f64(), 1)),
        }
    }
    let multiplicities: Vec<String> = knots.iter().map(|(_, n)| n.to_string()).collect();
    let values: Vec<String> = knots.iter().map(|(k, _)| format!("{k:?}")).collect();
    text += &format!(
        "#{first}=(BOUNDED_CURVE()B_SPLINE_CURVE(2,({}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({}))REPRESENTATION_ITEM(''));\n",
        points.join(","),
        multiplicities.join(","),
        values.join(","),
        weights.join(",")
    );
    text
}

/// A thread's flank, as a strip of a cylinder of radius 1 between two
/// helices a quarter of their pitch apart, turning one and a half times,
/// their ends joined along the cylinder: cut along meridians into pieces
/// that each turn less than once. At a pitch of 1: a pcurve fitted as one
/// cubic across the helices' joints, where they are only C1, drifted from
/// them by about 1e-4, more than the kernel's accuracy, until the fit broke
/// at the joints (see `AGENTS.md` on approximated data that is only C1).
#[test]
fn a_strip_turning_more_than_once_is_cut_along_meridians() {
    let body = format!(
        "
#100=CYLINDRICAL_SURFACE('',#8,1.);
{}{}
#120=CARTESIAN_POINT('',(1.,0.,0.));
#121=VERTEX_POINT('',#120);
#122=CARTESIAN_POINT('',(-1.,0.,1.5));
#123=VERTEX_POINT('',#122);
#124=CARTESIAN_POINT('',(-1.,0.,1.75));
#125=VERTEX_POINT('',#124);
#126=CARTESIAN_POINT('',(1.,0.,0.25));
#127=VERTEX_POINT('',#126);
#130=VECTOR('',#6,1.);
#131=LINE('',#122,#130);
#132=LINE('',#120,#130);
#140=EDGE_CURVE('',#121,#123,#200,.T.);
#141=EDGE_CURVE('',#123,#125,#131,.T.);
#142=EDGE_CURVE('',#127,#125,#300,.T.);
#143=EDGE_CURVE('',#121,#127,#132,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#141,.T.);
#152=ORIENTED_EDGE('',*,*,#142,.F.);
#153=ORIENTED_EDGE('',*,*,#143,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_OUTER_BOUND('',#154,.T.);
#156=ADVANCED_FACE('flank',(#155),#100,.T.);
#180=OPEN_SHELL('',(#156));
#999=SHELL_BASED_SURFACE_MODEL('',(#180));",
        helix_entities(200, 0.0),
        helix_entities(300, 0.25)
    );
    let part = import(&file(".MILLI.,.METRE.", &body));
    let model = part.topology();
    // One and a half turns, cut every half turn: three pieces at least.
    assert!(model.faces.len() >= 3, "{} faces", model.faces.len());
    assert!(part.face_id("import(i,s0,f0,q0)").is_ok());
    assert!(part.edge_id("import(i,s0,f0,m0)").is_ok());
    assert_bounds(model, [-1.0, -1.0, 0.0], [1.0, 1.0, 1.75]);
}

#[test]
fn lengths_are_read_in_millimetres() {
    let part = import(&file(".CENTI.,.METRE.", CYLINDER));
    assert_bounds(part.topology(), [-10.0, -10.0, 0.0], [10.0, 10.0, 20.0]);
}

/// The body entities of a polyhedron of plane faces: `faces` lists each
/// face's corners counter-clockwise seen from outside.
fn polyhedron(points: &[[f64; 3]], faces: &[&[usize]]) -> String {
    let mut out = String::new();
    let mut id = 100;
    let mut next = || {
        id += 1;
        id
    };
    let mut vertex = Vec::new();
    for p in points {
        let c = next();
        let vp = next();
        out += &format!(
            "#{c}=CARTESIAN_POINT('',({:?},{:?},{:?}));\n#{vp}=VERTEX_POINT('',#{c});\n",
            p[0], p[1], p[2]
        );
        vertex.push((c, vp));
    }
    let mut edges: Vec<((usize, usize), usize)> = Vec::new();
    let mut face_ids = Vec::new();
    for face in faces {
        let mut oriented = Vec::new();
        for k in 0..face.len() {
            let (a, b) = (face[k], face[(k + 1) % face.len()]);
            let (key, forward) = if a < b {
                ((a, b), true)
            } else {
                ((b, a), false)
            };
            let edge = match edges.iter().find(|(k, _)| *k == key) {
                Some(&(_, e)) => e,
                None => {
                    let (pa, pb) = (points[key.0], points[key.1]);
                    let d = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
                    let (dir, vec, line, e) = (next(), next(), next(), next());
                    out += &format!(
                        "#{dir}=DIRECTION('',({:?},{:?},{:?}));\n#{vec}=VECTOR('',#{dir},1.);\n#{line}=LINE('',#{},#{vec});\n#{e}=EDGE_CURVE('',#{},#{},#{line},.T.);\n",
                        d[0], d[1], d[2], vertex[key.0].0, vertex[key.0].1, vertex[key.1].1
                    );
                    edges.push((key, e));
                    e
                }
            };
            let o = next();
            out += &format!(
                "#{o}=ORIENTED_EDGE('',*,*,#{edge},.{}.);\n",
                if forward { "T" } else { "F" }
            );
            oriented.push(format!("#{o}"));
        }
        // The plane through the first corner, its normal out of the solid.
        let (p0, p1, p2) = (points[face[0]], points[face[1]], points[face[2]]);
        let a = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let b = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        let n = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let (nd, xd, ax, plane, lp, bound, f) =
            (next(), next(), next(), next(), next(), next(), next());
        out += &format!(
            "#{nd}=DIRECTION('',({:?},{:?},{:?}));\n#{xd}=DIRECTION('',({:?},{:?},{:?}));\n#{ax}=AXIS2_PLACEMENT_3D('',#{},#{nd},#{xd});\n#{plane}=PLANE('',#{ax});\n#{lp}=EDGE_LOOP('',({}));\n#{bound}=FACE_OUTER_BOUND('',#{lp},.T.);\n#{f}=ADVANCED_FACE('',(#{bound}),#{plane},.T.);\n",
            n[0],
            n[1],
            n[2],
            a[0],
            a[1],
            a[2],
            vertex[face[0]].0,
            oriented.join(",")
        );
        face_ids.push(format!("#{f}"));
    }
    let shell = next();
    out += &format!(
        "#{shell}=CLOSED_SHELL('',({}));\n#999=MANIFOLD_SOLID_BREP('box',#{shell});\n",
        face_ids.join(",")
    );
    out
}

#[test]
fn a_box_of_planes_and_lines_imports() {
    let p = [
        [0.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [3.0, 2.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 1.0],
        [3.0, 0.0, 1.0],
        [3.0, 2.0, 1.0],
        [0.0, 2.0, 1.0],
    ];
    let faces: [&[usize]; 6] = [
        &[0, 3, 2, 1],
        &[4, 5, 6, 7],
        &[0, 1, 5, 4],
        &[1, 2, 6, 5],
        &[2, 3, 7, 6],
        &[3, 0, 4, 7],
    ];
    let part = import(&file(".MILLI.,.METRE.", &polyhedron(&p, &faces)));
    assert_eq!(counts(part.topology()), (6, 12, 8));
    assert_bounds(part.topology(), [0.0; 3], [3.0, 2.0, 1.0]);
}

/// The box of [`a_box_of_planes_and_lines_imports`], its corners counted
/// the same way.
const BOX: [[f64; 3]; 8] = [
    [0.0, 0.0, 0.0],
    [3.0, 0.0, 0.0],
    [3.0, 2.0, 0.0],
    [0.0, 2.0, 0.0],
    [0.0, 0.0, 1.0],
    [3.0, 0.0, 1.0],
    [3.0, 2.0, 1.0],
    [0.0, 2.0, 1.0],
];

/// A vertex the file puts 3e-4 off the corner its three planes meet at —
/// further than the kernel can carry as one point — with the lines of its
/// edges running to it: the vertex is put where the planes meet, and its
/// edges rebuilt there.
#[test]
fn a_vertex_off_its_faces_is_put_where_they_meet() {
    let mut p = BOX;
    p[6] = [3.0003, 2.0, 1.0];
    // Each face's plane through corners of its own other than that one.
    let faces: [&[usize]; 6] = [
        &[0, 3, 2, 1],
        &[7, 4, 5, 6],
        &[0, 1, 5, 4],
        &[5, 1, 2, 6],
        &[2, 3, 7, 6],
        &[3, 0, 4, 7],
    ];
    let part = import(&file(".MILLI.,.METRE.", &polyhedron(&p, &faces)));
    let model = part.topology();
    assert_eq!(counts(model), (6, 12, 8));
    let corner = model
        .vertices
        .values()
        .map(|v| v.point)
        .find(|q| q[0].to_f64() > 2.0 && q[1].to_f64() > 1.0 && q[2].to_f64() > 0.5)
        .expect("the corner");
    assert!(corner.could_be_equal(&v(3.0, 2.0, 1.0)), "{corner:?}");
}

/// Four planes at a vertex that do not meet in a point: the box's top cut
/// along a diagonal into two triangles, one of them tilted so that it
/// passes 3e-4 above the corner the other three planes meet at. No point
/// lies on all four, so the vertex is refused, named, with how near they
/// come.
#[test]
fn faces_that_do_not_meet_at_a_vertex_are_refused_by_name() {
    let faces: [&[usize]; 7] = [
        &[0, 3, 2, 1],
        &[4, 5, 6],
        &[4, 6, 7],
        &[0, 1, 5, 4],
        &[1, 2, 6, 5],
        &[2, 3, 7, 6],
        &[3, 0, 4, 7],
    ];
    let text = polyhedron(&BOX, &faces);
    // The second triangle's plane, through the corner 4, turned about the
    // line from 4 to 7.
    let level = "DIRECTION('',(0.0,0.0,6.0))";
    let second = text
        .match_indices(level)
        .nth(1)
        .expect("two top triangles")
        .0;
    let text = format!(
        "{}DIRECTION('',(-0.0006,0.0,6.0)){}",
        &text[..second],
        &text[second + level.len()..]
    );
    let error = read_step::<S>(&file(".MILLI.,.METRE.", &text))
        .err()
        .expect("refused")
        .to_string();
    // The triangles have three edges: neither can be rebuilt instead.
    assert!(
        error.contains("do not meet near [3.0, 2.0, 1.0]")
            && error.contains("no choice of the faces"),
        "{error}"
    );
}

/// The box's top in two faces, the smaller of which the file puts on a
/// plane 3e-4 above the rest of the box — a step no edge can lie on both
/// sides of. Where the two planes of its edge do not meet, the smaller
/// face is rebuilt from its edges, which the file has where the box's
/// other faces are: level with the rest of the top.
#[test]
fn a_face_whose_surface_contradicts_its_neighbours_is_rebuilt_from_its_edges() {
    let mut p = BOX.to_vec();
    p.extend([[2.0, 0.0, 1.0], [2.0, 2.0, 1.0]]);
    let faces: [&[usize]; 7] = [
        &[0, 3, 2, 1],
        &[4, 8, 9, 7],
        &[8, 5, 6, 9],
        &[0, 1, 5, 8, 4],
        &[1, 2, 6, 5],
        &[2, 3, 7, 9, 6],
        &[3, 0, 4, 7],
    ];
    // The second top face's plane, through its first corner (vertex 8,
    // the point `#117`), raised by 3e-4.
    let text = polyhedron(&p, &faces);
    let placement = "AXIS2_PLACEMENT_3D('',#117,";
    assert_eq!(text.matches(placement).count(), 1, "{text}");
    let text = format!(
        "#998=CARTESIAN_POINT('',(2.0,0.0,1.0003));\n{}",
        text.replace(placement, "AXIS2_PLACEMENT_3D('',#998,")
    );
    let text = file(".MILLI.,.METRE.", &text);
    let bodies = read_step::<S>(&text).unwrap();
    assert_eq!(bodies[0].healed.faces.len(), 1, "{:?}", bodies[0].healed);
    let part = import(&text);
    let model = part.topology();
    assert_eq!(counts(model), (7, 15, 10));
    assert_bounds(model, [0.0; 3], [3.0, 2.0, 1.0]);
}

/// A quarter of a cylinder as a rational B-spline surface, standing on its
/// own: its rims rational B-spline curves.
const QUARTER: &str = "
#100=CARTESIAN_POINT('',(1.,0.,0.));
#101=CARTESIAN_POINT('',(1.,1.,0.));
#102=CARTESIAN_POINT('',(0.,1.,0.));
#103=CARTESIAN_POINT('',(1.,0.,1.));
#104=CARTESIAN_POINT('',(1.,1.,1.));
#105=CARTESIAN_POINT('',(0.,1.,1.));
#110=(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,1,((#100,#103),(#101,#104),(#102,#105)),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(((1.,1.),(0.7071067811865476,0.7071067811865476),(1.,1.)))REPRESENTATION_ITEM('')SURFACE());
#120=VERTEX_POINT('',#100);
#121=VERTEX_POINT('',#102);
#122=VERTEX_POINT('',#103);
#123=VERTEX_POINT('',#105);
#130=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#100,#101,#102),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,0.7071067811865476,1.))REPRESENTATION_ITEM(''));
#131=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#103,#104,#105),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,0.7071067811865476,1.))REPRESENTATION_ITEM(''));
#132=VECTOR('',#6,1.);
#133=LINE('',#100,#132);
#134=LINE('',#102,#132);
#140=EDGE_CURVE('',#120,#121,#130,.T.);
#141=EDGE_CURVE('',#122,#123,#131,.T.);
#142=EDGE_CURVE('',#120,#122,#133,.T.);
#143=EDGE_CURVE('',#121,#123,#134,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#143,.T.);
#152=ORIENTED_EDGE('',*,*,#141,.F.);
#153=ORIENTED_EDGE('',*,*,#142,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_OUTER_BOUND('',#154,.T.);
#156=ADVANCED_FACE('',(#155),#110,.T.);
#157=OPEN_SHELL('',(#156));
#999=SHELL_BASED_SURFACE_MODEL('',(#157));";

#[test]
fn a_rational_b_spline_sheet_imports() {
    let part = import(&file(".MILLI.,.METRE.", QUARTER));
    let model = part.topology();
    assert_eq!(counts(model), (1, 4, 4));
    assert!(model.solids.is_empty());
    assert_bounds(model, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    // On the cylinder of radius 1.
    let face = model.faces.values().next().unwrap();
    let p = face
        .surface
        .evaluate(S::from_f64(0.3), S::from_f64(0.6))
        .unwrap();
    assert!(p[0].mul(p[0]).add(p[1].mul(p[1])).could_be_equal(S::ONE));
}

#[test]
fn an_unsupported_entity_is_refused_by_name() {
    let text = file(
        ".MILLI.,.METRE.",
        &CYLINDER.replace(
            "#110=CYLINDRICAL_SURFACE('',#8,1.);",
            "#110=OFFSET_SURFACE('',#111,1.,.F.);",
        ),
    );
    let error = read_step::<S>(&text).err().expect("refused").to_string();
    assert!(
        error.contains("#110 OFFSET_SURFACE is not supported"),
        "{error}"
    );
}

/// An assembly placing the cylinder twice, the second turned to lie along
/// `y`, as products, occurrences and placements: flattened into two
/// solids where they are placed.
#[test]
fn an_assembly_is_flattened_with_its_placements() {
    let assembly = format!(
        "{CYLINDER}
#20=SHAPE_REPRESENTATION('assembly',(#8,#21,#25),#4);
#22=CARTESIAN_POINT('',(10.,0.,0.));
#21=AXIS2_PLACEMENT_3D('',#22,#6,#7);
#26=CARTESIAN_POINT('',(0.,10.,0.));
#27=DIRECTION('',(0.,1.,0.));
#25=AXIS2_PLACEMENT_3D('',#26,#27,#7);
#30=APPLICATION_CONTEXT('');
#31=PRODUCT_CONTEXT('',#30,'mechanical');
#32=PRODUCT_DEFINITION_CONTEXT('part definition',#30,'design');
#40=PRODUCT('cylinder','cylinder','',(#31));
#41=PRODUCT_DEFINITION_FORMATION('','',#40);
#42=PRODUCT_DEFINITION('design','',#41,#32);
#43=PRODUCT_DEFINITION_SHAPE('','',#42);
#44=SHAPE_DEFINITION_REPRESENTATION(#43,#9);
#50=PRODUCT('assembly','assembly','',(#31));
#51=PRODUCT_DEFINITION_FORMATION('','',#50);
#52=PRODUCT_DEFINITION('design','',#51,#32);
#53=PRODUCT_DEFINITION_SHAPE('','',#52);
#54=SHAPE_DEFINITION_REPRESENTATION(#53,#20);
#60=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#52,#42,$);
#61=NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#52,#42,$);
#62=PRODUCT_DEFINITION_SHAPE('','',#60);
#63=PRODUCT_DEFINITION_SHAPE('','',#61);
#64=ITEM_DEFINED_TRANSFORMATION('','',#8,#21);
#65=ITEM_DEFINED_TRANSFORMATION('','',#8,#25);
#66=(REPRESENTATION_RELATIONSHIP('','',#9,#20)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#64)SHAPE_REPRESENTATION_RELATIONSHIP());
#67=(REPRESENTATION_RELATIONSHIP('','',#9,#20)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#65)SHAPE_REPRESENTATION_RELATIONSHIP());
#68=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#66,#62);
#69=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#67,#63);"
    );
    let part = import(&file(".MILLI.,.METRE.", &assembly));
    let model = part.topology();
    assert_eq!(model.solids.len(), 2);
    assert_eq!(
        part.solid_names(),
        vec!["import(i,s0)".to_string(), "import(i,s1)".to_string()]
    );
    // One round z at x = 10, one along y — `z` turned onto `y` — at y = 10.
    assert_bounds(model, [-1.0, -1.0, -1.0], [11.0, 12.0, 2.0]);
}

/// A face standing on its own is written as a surface model, and read
/// back as one.
#[test]
fn a_sheet_round_trips() {
    let part = import(&file(".MILLI.,.METRE.", QUARTER));
    let text = write_step(&part, "sheet").unwrap();
    assert!(text.contains("SHELL_BASED_SURFACE_MODEL"));
    let back = round_trip(&part);
    assert!(back.topology().solids.is_empty());
}

/// `instance` placed at `pose`.
fn placed(
    instance: &geop_ops::Instance<S>,
    pose: geop_core_math::primitives::Pose<S>,
) -> geop_ops::Instance<S> {
    geop_ops::Instance {
        pose,
        ..instance.clone()
    }
}

/// An assembly — a base plate placing a peg twice, once turned to lie along
/// `x`, and a subassembly placing the same peg — is written as products:
/// one per distinct component, each placement an occurrence. Read back, it
/// is flattened into its solids where they are placed.
#[test]
fn an_assembly_is_written_as_products_and_occurrences() {
    use geop_core_math::primitives::{Pose, Quaternion};
    use std::collections::BTreeSet;
    let component = |file: &str, part: Part<S>| {
        geop_ops::Instance::of(file.into(), part, BTreeSet::from([file.to_string()]))
    };
    let mut peg = Part::new();
    revolved_cylinder(
        &mut peg,
        "c",
        v(0.0, 0.0, 0.0),
        S::from_f64(0.5),
        S::from_f64(2.0),
    )
    .unwrap();
    let peg = component("parts/peg.geop", peg);
    let at = |x: f64, y: f64, z: f64| Pose::new(v(x, y, z), Quaternion::identity()).unwrap();
    let mut sub = Part::new();
    sub.add_instance(placed(&peg, at(0.0, 0.0, 1.0)), "p")
        .unwrap();
    let sub = component("sub.geop", sub);

    let mut part = Part::new();
    cube_solid(&mut part, "base", v(0.0, 0.0, -1.0), v(10.0, 10.0, 0.0)).unwrap();
    part.add_instance(placed(&peg, at(2.0, 2.0, 0.0)), "a")
        .unwrap();
    // About `y` by a quarter turn: the peg's `z` onto `x`.
    let half = std::f64::consts::FRAC_PI_4;
    let turned = Pose::new(
        v(9.0, 8.0, 0.5),
        Quaternion::new(
            S::from_f64(half.cos()),
            S::ZERO,
            S::from_f64(half.sin()),
            S::ZERO,
        ),
    )
    .unwrap();
    part.add_instance(placed(&peg, turned), "b").unwrap();
    part.add_instance(placed(&sub, at(8.0, 2.0, 0.0)), "s")
        .unwrap();

    let text = write_step(&part, "robot").unwrap();
    // To check it in another CAD system: written where STEP_EXPORT_DIR says.
    if let Some(dir) = std::env::var_os("STEP_EXPORT_DIR") {
        std::fs::write(std::path::Path::new(&dir).join("robot.step"), &text).unwrap();
    }
    let count = |what: &str| text.matches(what).count();
    assert_eq!(
        count("=PRODUCT('"),
        3,
        "the assembly, the subassembly, the peg"
    );
    assert_eq!(count("NEXT_ASSEMBLY_USAGE_OCCURRENCE"), 4);
    assert_eq!(count("ITEM_DEFINED_TRANSFORMATION"), 4);
    assert_eq!(
        count("MANIFOLD_SOLID_BREP"),
        2,
        "the base and the peg, once each"
    );
    assert!(text.contains("PRODUCT('peg','peg'"));

    let back = import(&text);
    let model = back.topology();
    assert_eq!(model.solids.len(), 4, "the base and three pegs");
    // The pegs reach z = 2 (a), 3 (in the subassembly) and x = 11 (b).
    assert_bounds(model, [0.0, 0.0, -1.0], [11.0, 10.0, 3.0]);
}

/// The bodies an import keeps read back as they were kept: a cylinder
/// (rational arcs, a seam cut into sectors) and a sphere, kept and read
/// back, build the same model, every scalar the same enclosure; and the
/// key changes with the file's text.
#[test]
fn kept_bodies_read_back_as_they_were() {
    let mut cylinder = Part::new();
    revolved_cylinder(
        &mut cylinder,
        "c",
        v(1.0, 2.0, 0.0),
        S::from_f64(0.5),
        S::from_f64(2.0),
    )
    .unwrap();
    let mut sphere = Part::new();
    sphere_solid(&mut sphere, "s", v(0.0, 0.0, 1.0), S::from_f64(1.5)).unwrap();
    for (name, text) in [
        ("cylinder", write_step(&cylinder, "cylinder").unwrap()),
        ("sphere", write_step(&sphere, "sphere").unwrap()),
    ] {
        let bodies = read_step::<S>(&text).unwrap();
        let kept = crate::cache::encode(&bodies).unwrap();
        let back = crate::cache::decode::<S>(&kept).unwrap();
        assert_eq!(back.len(), bodies.len(), "{name}");
        for (a, b) in bodies.iter().zip(&back) {
            assert_eq!(a.label, b.label);
            assert_eq!(a.face_names, b.face_names);
            assert_eq!(a.spec.vertices, b.spec.vertices, "{name}");
            for (fa, fb) in a.spec.faces.iter().zip(&b.spec.faces) {
                assert_eq!(
                    fa.surface.control_points, fb.surface.control_points,
                    "{name}"
                );
                assert_eq!(fa.surface.knot_vector_u, fb.surface.knot_vector_u, "{name}");
            }
            for (ea, eb) in a.spec.edges.iter().zip(&b.spec.edges) {
                assert_eq!(ea.curve.control_points, eb.curve.control_points, "{name}");
            }
        }
        let mut part = Part::<S>::new();
        add_bodies(&mut part, &Namer::new("import", "i").unwrap(), back).unwrap();
        assert_valid(part.topology());
        assert_ne!(
            crate::cache::key::<S>(&text),
            crate::cache::key::<S>(&text.replace("cylinder", "kreis").replace("sphere", "kugel"))
        );
    }
}
