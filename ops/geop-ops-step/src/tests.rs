//! Fast tests: STEP texts written here, read into valid bodies; parts
//! written and read back the same — example parts among them; an assembly
//! written as products and occurrences; and the editor's STEP export
//! downloaded and imported back by a step, as the front end does it.

use geop_core_editor::{
    editing::{Control, Value},
    editor::{Command, Editor},
    registry::{Example, Registry},
    testing::{dialog, handled, loaded},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain,
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    cache::{Cache, CacheStore},
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    reader::Reader,
    runner::Runner,
    values::{StateValue, pose_state},
    workspace::{Files, Location, Workspace},
};
use geop_core_topology::{
    Model,
    validation::{ValidationParameters, validate},
};
use std::{
    cell::{Cell, RefCell},
    collections::BTreeMap,
    rc::Rc,
};

use geop_lib_fixtures::{PrimitiveArgs, drilled_box};
use geop_lib_stdlib::WithStandardParts;
use geop_lib_sweep::shapes::{cube_solid, revolved_cylinder, sphere_solid, v3};
use geop_ops_assembly::{AddPart, AddPartArgs, examples::pin};
use geop_ops_extrude_revolve::{
    Extrude, Revolve,
    examples::{box_with_drill_hole, bracket, cross_drilled_shaft, revolved_cone_on_box},
};
use geop_ops_model::{
    NamedModel, Namer, read_model,
    testing::{assert_model_valid_sheets, model_of},
};
use geop_ops_sketch::AddSketch;

use crate::{ImportStep, ImportStepArgs, Imported, add_bodies, read_step, write_part, write_step};

/// `model` as the text of a STEP file whose product is `name`.
fn written(model: &NamedModel, name: &str) -> String {
    write_step(model, name).unwrap()
}

/// The part the STEP text `text` imports into, checked valid.
fn import(text: &str) -> NamedModel {
    let bodies = read_step(text).unwrap_or_else(|e| panic!("reading failed: {e}"));
    let mut part = NamedModel::new();
    add_bodies(&mut part, &Namer::new("import", "i").unwrap(), &bodies)
        .unwrap_or_else(|e| panic!("adding failed: {e}"));
    assert_model_valid_sheets(part.topology());
    part
}

/// `(faces, edges, vertices)` of a model.
fn counts(model: &Model) -> (usize, usize, usize) {
    (model.faces.len(), model.edges.len(), model.vertices.len())
}

/// The box around a model's edges, sampled.
fn bounds(model: &Model) -> ([f64; 3], [f64; 3]) {
    plain::bounds(model.edges.values().flat_map(|edge| {
        let (t0, t1) = edge.curve.domain();
        (0..=256).map(move |i| {
            let t = t0.to_f64() + (t1.to_f64() - t0.to_f64()) * i as f64 / 256.0;
            edge.curve
                .evaluate(Scalar::from_f64(t))
                .unwrap()
                .to_f64_array()
        })
    }))
    .unwrap()
}

fn assert_bounds(model: &Model, lo: [f64; 3], hi: [f64; 3]) {
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
fn round_trip(part: &NamedModel) -> NamedModel {
    let text = written(part, "part");
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
    let mut part = NamedModel::new();
    cube_solid(&mut part, "c", v3(0.0, 0.0, 0.0), v3(2.0, 1.0, 3.0)).unwrap();
    let back = round_trip(&part);
    assert_eq!(back.solid_names(), vec!["import(i,s0)".to_string()]);
}

#[test]
fn a_cylinder_round_trips() {
    let mut part = NamedModel::new();
    revolved_cylinder(
        &mut part,
        "c",
        v3(1.0, 2.0, 0.0),
        Scalar::from_f64(0.5),
        Scalar::from_f64(2.0),
    )
    .unwrap();
    round_trip(&part);
}

#[test]
fn a_sphere_round_trips() {
    let mut part = NamedModel::new();
    sphere_solid(&mut part, "s", v3(0.0, 0.0, 1.0), Scalar::from_f64(1.5)).unwrap();
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
    use geop_core_geometry::nurb_curve::{Handedness, Helix, NurbCurve3D};
    use geop_core_math::primitives::CoordinateSystem;
    let helix = NurbCurve3D::helix(
        &CoordinateSystem::world_at(v3(0.0, 0.0, z0)),
        Scalar::ONE,
        &Helix::new(Scalar::ONE, 1.5, Handedness::Right),
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
    assert!(corner.could_be_equal(&v3(3.0, 2.0, 1.0)), "{corner:?}");
}

/// The box's corner `6` given twice, the second `1e-9` above the first —
/// within the file's uncertainty of `1e-7` — and an edge between the two
/// on the top and the back: an edge of no length the file can tell. It is
/// taken out, the two corners one, put where the three planes meet.
#[test]
fn an_edge_shorter_than_the_files_uncertainty_is_taken_out() {
    let mut p = BOX.to_vec();
    p.push([3.0, 2.0, 1.0 + 1e-9]);
    let faces: [&[usize]; 6] = [
        &[0, 3, 2, 1],
        &[4, 5, 6, 8, 7],
        &[0, 1, 5, 4],
        &[5, 1, 2, 6],
        &[2, 3, 7, 8, 6],
        &[3, 0, 4, 7],
    ];
    let part = import(&file(".MILLI.,.METRE.", &polyhedron(&p, &faces)));
    let model = part.topology();
    assert_eq!(counts(model), (6, 12, 8));
    assert!(
        (model.vertices.values()).any(|v| v.point.could_be_equal(&v3(3.0, 2.0, 1.0))),
        "no corner at (3, 2, 1)"
    );
}

/// The box's corner `6` given twice, `1e-9` apart: the edge from it to `7`
/// starts at the second, every other edge at the first. Within the file's
/// uncertainty, where one coedge of a loop ends and the next starts, they
/// are one vertex.
#[test]
fn a_corner_given_twice_where_a_loop_passes_it_is_one() {
    let mut p = BOX.to_vec();
    p.push([3.0, 2.0, 1.0 + 1e-9]);
    let faces: [&[usize]; 6] = [
        &[0, 3, 2, 1],
        &[4, 5, 6, 7],
        &[0, 1, 5, 4],
        &[5, 1, 2, 6],
        &[2, 3, 7, 6],
        &[3, 0, 4, 7],
    ];
    // Point k is written as #(101 + 2k), its vertex as #(102 + 2k).
    let text = polyhedron(&p, &faces);
    let from_six = "EDGE_CURVE('',#114,#116,";
    assert!(text.contains(from_six), "{text}");
    let text = text.replace(from_six, "EDGE_CURVE('',#118,#116,");
    let part = import(&file(".MILLI.,.METRE.", &text));
    assert_eq!(counts(part.topology()), (6, 12, 8));
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
    let error = read_step(&file(".MILLI.,.METRE.", &text))
        .expect_err("refused")
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
    let bodies = read_step(&text).unwrap();
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
        .evaluate(Scalar::from_f64(0.3), Scalar::from_f64(0.6))
        .unwrap();
    assert!(
        p[0].mul(p[0])
            .add(p[1].mul(p[1]))
            .could_be_equal(Scalar::ONE)
    );
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
    let error = read_step(&text).expect_err("refused").to_string();
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
    let text = written(&part, "sheet");
    assert!(text.contains("SHELL_BASED_SURFACE_MODEL"));
    let back = round_trip(&part);
    assert!(back.topology().solids.is_empty());
}

/// The STEP files of a program's folder, by name, counting in `reads` how
/// often one is read. Each read is a text of its own, equal to the last.
struct Folder {
    files: BTreeMap<String, String>,
    reads: Rc<Cell<usize>>,
    /// The revision of every file in it.
    revision: u64,
}

impl Workspace for Folder {
    fn program(&self, path: &str) -> GeopResult<Rc<Program>> {
        Err(GeopError::new(format!("{path:?} is no program")))
    }

    fn read(&self, path: &str) -> GeopResult<Rc<str>> {
        self.reads.set(self.reads.get() + 1);
        let text =
            (self.files.get(path)).ok_or_else(|| GeopError::new(format!("no file {path:?}")))?;
        Ok(Rc::from(text.as_str()))
    }

    fn list(&self) -> Vec<String> {
        self.files.keys().cloned().collect()
    }

    fn resolve(&self, _: &str, name: &str) -> String {
        name.to_string()
    }

    fn relative(&self, _: &str, path: &str) -> String {
        path.to_string()
    }

    fn revision(&self, _: &str) -> u64 {
        self.revision
    }
}

/// An import step reads its file through the workspace only when the
/// file's revision changed, and builds from it only once: run again, it
/// reads nothing; given a new revision of an equal text, it reads it, and
/// nothing — neither the bodies nor the model — is built anew.
#[test]
fn an_import_builds_an_equal_file_once() {
    let mut cylinder = NamedModel::new();
    revolved_cylinder(
        &mut cylinder,
        "c",
        v3(1.0, 2.0, 0.0),
        Scalar::from_f64(0.5),
        Scalar::from_f64(2.0),
    )
    .unwrap();
    let text = written(&cylinder, "cylinder");
    let reads = Rc::new(Cell::new(0));
    // The runner given the folder anew: equal files, in a revision.
    let given = |runner: &mut Runner, revision: u64| {
        runner.workspace = Some(Rc::new(Folder {
            files: BTreeMap::from([("cylinder.step".to_string(), text.clone())]),
            reads: reads.clone(),
            revision,
        }));
    };
    let mut runner = Runner::new(OperationRegistry::default().with(ImportStep).unwrap());
    given(&mut runner, 0);
    let mut program = Program::new();
    let args = ImportStepArgs {
        file: "cylinder.step".into(),
    };
    program.push("i", args);
    let mut part = Part::default();
    let run = |runner: &Runner, part: &mut Part| {
        program.run(runner, part).unwrap();
        let model = read_model(&part.current()).unwrap();
        // Where the model is kept: a model built anew is kept elsewhere.
        std::ptr::from_ref(model.as_ref())
    };
    let first = run(&runner, &mut part);
    assert_eq!(reads.get(), 1);
    assert_eq!(run(&runner, &mut part), first, "nothing built anew");
    assert_eq!(reads.get(), 1, "the file did not change: nothing read");
    // The same files, in a new revision: an equal text, to the same bodies.
    given(&mut runner, 1);
    assert_eq!(run(&runner, &mut part), first, "the model not built anew");
    assert_eq!(reads.get(), 2, "the new revision read");
    let model = read_model(&part.current()).unwrap();
    assert_eq!(model.solid_names(), vec!["import(i,s0)".to_string()]);
    assert_model_valid_sheets(model.topology());
}

/// A step placing `file`, fixed where the program's state puts it.
fn placing(file: &str) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        ..Default::default()
    }
    .fixed()
}

fn at(x: f64, y: f64, z: f64) -> Pose {
    let v = Vector3::from_array([x, y, z].map(Scalar::from_f64));
    Pose::new(v, Quaternion::identity()).unwrap()
}

/// `program` placing each of `placed` as its name, where its pose puts it.
fn with_placed(mut program: Program, placed: &[(&str, &str, Pose)]) -> Program {
    for (name, file, pose) in placed {
        program.push(*name, placing(file));
        (program.state).insert(pose_state(name), StateValue::Pose(*pose));
    }
    program
}

/// The part `program`, the file `file` among `files`, builds — written in
/// given solids, or in the sketches, extrusions and revolves of the
/// example parts, placing parts and importing STEP files.
fn build(program: &Program, file: &str, files: BTreeMap<String, String>) -> Part {
    let registry = (geop_lib_fixtures::registry().and_then(|r| r.with(AddSketch)))
        .and_then(|r| r.with(Extrude))
        .and_then(|r| r.with(Revolve))
        .and_then(|r| r.with(AddPart))
        .and_then(|r| r.with(ImportStep))
        .unwrap();
    let mut runner = Runner::new(registry.clone());
    runner.workspace = Some(Files::new(files, Rc::new(registry)));
    runner.location = Location {
        file: file.to_string(),
        within: Vec::new(),
    };
    let mut part = Part::default();
    program.run(&runner, &mut part).unwrap();
    part
}

/// An assembly — a base plate placing a peg twice, once turned to lie along
/// `x`, and a subassembly placing the same peg — is written as products:
/// one per distinct component, each placement an occurrence. Read back, it
/// is flattened into its solids where they are placed.
#[test]
fn an_assembly_is_written_as_products_and_occurrences() {
    // A peg of radius 0.5 standing 2 high on the origin.
    let mut peg = Program::new();
    peg.push("c", PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 0.5, 2.0));
    let sub = with_placed(
        Program::new(),
        &[("p", "parts/peg.geop", at(0.0, 0.0, 1.0))],
    );

    // A 10 x 10 plate, 1 thick, under the origin's plane.
    let mut base = Program::new();
    base.push(
        "base",
        PrimitiveArgs::block([0.0, 0.0, -1.0], [10.0, 10.0, 0.0]),
    );
    // About `y` by a quarter turn: the peg's `z` onto `x`.
    let half = std::f64::consts::FRAC_PI_4;
    let turn = Quaternion::new(
        Scalar::from_f64(half.cos()),
        Scalar::ZERO,
        Scalar::from_f64(half.sin()),
        Scalar::ZERO,
    );
    let turned = Pose::new(
        Vector3::from_array([9.0, 8.0, 0.5].map(Scalar::from_f64)),
        turn,
    )
    .unwrap();
    let robot = with_placed(
        base,
        &[
            ("a", "parts/peg.geop", at(2.0, 2.0, 0.0)),
            ("b", "parts/peg.geop", turned),
            ("s", "sub.geop", at(8.0, 2.0, 0.0)),
        ],
    );

    let files = BTreeMap::from([
        ("parts/peg.geop".to_string(), peg.to_json().unwrap()),
        ("sub.geop".to_string(), sub.to_json().unwrap()),
    ]);
    let part = build(&robot, "robot.geop", files);
    let text = write_part(&part.current(), "robot").unwrap();
    // To check it in another CAD system: written where STEP_EXPORT_DIR says.
    if let Some(dir) = std::env::var_os("STEP_EXPORT_DIR") {
        std::fs::write(std::path::Path::new(&dir).join("robot.step"), &text).unwrap();
    }
    let count = |what: &str| text.matches(what).count();
    // TODO: the peg written once, however often it is placed, once the
    // parts of no state are shared between instances (see
    // `geop_core_program::part::Part`): 3 products and 2 solids.
    assert_eq!(
        count("=PRODUCT('"),
        5,
        "the assembly, the subassembly, and each of the three pegs"
    );
    assert_eq!(count("NEXT_ASSEMBLY_USAGE_OCCURRENCE"), 4);
    assert_eq!(count("ITEM_DEFINED_TRANSFORMATION"), 4);
    assert_eq!(count("MANIFOLD_SOLID_BREP"), 4, "the base and each peg");
    assert!(text.contains("PRODUCT('peg','peg'"));

    // Read back by a program importing it.
    let mut program = Program::new();
    let file = "robot.step".to_string();
    program.push("imp", ImportStepArgs { file: file.clone() });
    let files = BTreeMap::from([(file, text)]);
    let back = model_of(&build(&program, "main.geop", files));
    let model: &Model = back.topology();
    assert_eq!(model.solids.len(), 4, "the base and three pegs");
    // The pegs reach z = 2 (a), 3 (in the subassembly) and x = 11 (b).
    let (lo, hi) = plain::bounds(model.vertices.values().map(|v| v.point.to_f64_array())).unwrap();
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);
    assert!(close(lo, [0.0, 0.0, -1.0]), "{lo:?}");
    assert!(close(hi, [11.0, 10.0, 3.0]), "{hi:?}");
}

/// The box around a model's vertices.
fn vertex_bounds(model: &Model) -> ([f64; 3], [f64; 3]) {
    plain::bounds(model.vertices.values().map(|v| v.point.to_f64_array())).unwrap()
}

/// The model a program of one import step builds, reading `file` — named
/// `part.step` next to the program — from `text`.
fn imported(text: String) -> NamedModel {
    let mut program = Program::new();
    program.push(
        "imp",
        ImportStepArgs {
            file: "part.step".into(),
        },
    );
    let files = BTreeMap::from([("part.step".to_string(), text)]);
    model_of(&build(&program, "main.geop", files))
}

/// Example parts — extruded, revolved, drilled, filleted, booleaned — come
/// back from STEP with as many faces, edges and vertices, in the same
/// place, and valid.
#[test]
fn examples_round_trip_through_step() {
    let chosen: [Example<Program>; 5] = [
        ("box_with_drill_hole", box_with_drill_hole),
        ("bracket", bracket),
        ("cross_drilled_shaft", cross_drilled_shaft),
        ("revolved_cone_on_box", revolved_cone_on_box),
        ("pin", pin),
    ];
    for (name, program) in chosen {
        let part = model_of(&build(&program(), "part.geop", BTreeMap::new()));
        let text = write_step(&part, name).unwrap();
        let back = imported(text);
        assert_eq!(
            counts(back.topology()),
            counts(part.topology()),
            "{name}: counts changed"
        );
        let (a, b) = (
            vertex_bounds(part.topology()),
            vertex_bounds(back.topology()),
        );
        for c in 0..3 {
            assert!(
                (a.0[c] - b.0[c]).abs() < 1e-9 && (a.1[c] - b.1[c]).abs() < 1e-9,
                "{name}: bounds {a:?} became {b:?}"
            );
        }
        if let Err(errors) = validate(&ValidationParameters::default(), back.topology()) {
            panic!("{name}: invalid after import: {:?}", errors);
        }
        assert_eq!(back.solid_names().len(), part.solid_names().len(), "{name}");
    }
}

/// What an editor of STEP files offers: the parts and assemblies written
/// and read — given solids, and the example assemblies' parts and the
/// standard parts as they are written: sketched on datums, extruded,
/// patterned, placed, with their parameters — and STEP itself.
fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_reference_geometry::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_pattern::register(r)?;
        let r = geop_ops_assembly::register(r)?;
        let r = geop_lib_fixtures::register(r)?;
        let r = crate::register(r)?;
        Ok(r)
    })();
    registry.expect("the operations each have a name of their own")
}

/// An editor of [`registry`], the standard parts beside the files it is
/// sent.
fn editor() -> Editor {
    Editor::new(registry(), WithStandardParts(BTreeMap::new()))
}

/// The part shown, downloaded as STEP the way the front end asks for it.
fn step_file(editor: &mut Editor) -> String {
    let update = editor.handle(Command::Export {
        exporter: "step".into(),
        format: None,
        options: serde_json::Value::Null,
        date: String::new(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let export = update.export.expect("a file is written");
    assert!(export.name.ends_with(".step"), "{}", export.name);
    let text = export.text().expect("STEP is text").to_string();
    assert!(text.contains("MANIFOLD_SOLID_BREP"), "no solid in {text}");
    text
}

/// The part shown written as STEP, stored in a folder below a new
/// program's, is offered by a new "Import STEP" step's file explorer by
/// its path relative to the program — beside the other files, which it
/// does not take — and, chosen, brings its solid back, named after the
/// step.
#[test]
fn a_part_exported_as_step_is_imported_back() {
    let mut editor = loaded(editor(), drilled_box());
    let files = std::collections::BTreeMap::from([
        (
            "parts/scans/box.step".to_string(),
            Some(step_file(&mut editor)),
        ),
        ("notes.txt".to_string(), Some("notes".to_string())),
    ]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("parts/imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Import STEP");
    let Some(Control::File { files, upload, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is a file field");
    };
    let listed: Vec<(&str, bool)> = (files.iter())
        .filter(|f| !f.path.starts_with("std:"))
        .map(|f| (f.path.as_str(), f.accepted))
        .collect();
    assert_eq!(listed, [("../notes.txt", false), ("scans/box.step", true)]);
    assert_eq!(upload, &["step", "stp"]);
    let update = editor.handle(dialog("file", Value::Choice("scans/box.step".into())));
    let step = update.step.unwrap();
    assert!(step.error.is_none(), "{:?}", step.error);
    let Some(Control::File { value, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is a file field");
    };
    assert_eq!(value, "scans/box.step");
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let id = editor.program().steps.last().unwrap().id.clone();
    let solids: Vec<String> = editor
        .handle(Command::Show)
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == "solid")
        .map(|item| item.name)
        .collect();
    assert_eq!(solids, vec![format!("import({id},s0)")]);
}

/// Cache files kept in memory as a folder on disk keeps them, counting how
/// many were written: one for every time a file was made into something.
#[derive(Clone, Default)]
struct Shelf {
    files: Rc<RefCell<BTreeMap<String, String>>>,
    writes: Rc<Cell<usize>>,
}

impl CacheStore for Shelf {
    fn build(&self) -> &str {
        "this build"
    }

    fn read(&self, path: &str) -> Option<String> {
        self.files.borrow().get(path).cloned()
    }

    fn write(&self, path: &str, text: &str) {
        self.writes.set(self.writes.get() + 1);
        (self.files.borrow_mut()).insert(path.to_string(), text.to_string());
    }
}

/// A STEP file is read once for every build an editor makes of it — the
/// import's preview, the program it is committed to, and a part placed
/// from that program in another file, which shares its bodies — and once
/// for every kernel of one build: another editor, given what the first
/// kept, reads the same bodies from there, to the bit.
#[test]
fn a_step_file_is_read_once_for_every_build_importing_it() {
    let text = step_file(&mut loaded(editor(), drilled_box()));
    let shelf = Shelf::default();
    let open = |program: Program, path: &str| {
        let cache = Cache::stored(Box::new(shelf.clone()));
        let mut editor = Editor::with_cache(registry(), WithStandardParts(BTreeMap::new()), cache);
        let files = BTreeMap::from([("box.step".to_string(), Some(text.clone()))]);
        assert!(editor.handle(Command::Files { files }).error.is_none());
        let path = Some(path.to_string());
        let update = editor.handle(Command::Load { program, path });
        assert!(update.error.is_none(), "{:?}", update.error);
        editor
    };
    let bodies = |part: Reader, id: &str| -> Imported {
        let read = ElementReference::whole(crate::imported(id));
        part.retrieve_element(&read).cloned().unwrap()
    };

    let mut editor = open(Program::new(), "part.geop");
    handled(
        &mut editor,
        Command::New {
            kind: "import_step".into(),
        },
    );
    handled(
        &mut editor,
        dialog("file", Value::Choice("box.step".into())),
    );
    assert_eq!(shelf.writes.get(), 1, "read for the preview");
    handled(&mut editor, Command::Commit);
    assert_eq!(shelf.writes.get(), 1, "the preview's bodies committed");
    let part = editor.program().clone();
    let id = part.steps[0].id.clone();
    let read = bodies(editor.part(), &id);
    assert!(shelf.files.borrow().contains_key("box.step.geopc"));

    let assembly = with_placed(Program::new(), &[("p", "part.geop", at(0.0, 0.0, 0.0))]);
    let update = editor.handle(Command::Load {
        program: assembly,
        path: Some("asm.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let placed = bodies(editor.part().placed_part("p").unwrap(), &id);
    assert!(
        Rc::ptr_eq(&placed.0, &read.0),
        "the placed part's bodies shared"
    );
    assert_eq!(shelf.writes.get(), 1);

    let other = open(part, "part.geop");
    assert_eq!(
        shelf.writes.get(),
        1,
        "the other kernel's read from the cache"
    );
    assert_eq!(bodies(other.part(), &id), read);
}

/// A STEP file whose corner lies 3e-4 above where its three faces meet —
/// further than the kernel can carry as one point — imports with the corner
/// put back where they meet, and the dialog says what was rebuilt, and how
/// far it moved.
#[test]
fn an_import_that_heals_the_file_says_what_it_rebuilt() {
    let mut editor = loaded(editor(), drilled_box());
    let text = step_file(&mut editor);
    // The first vertex's point, raised.
    let at = text.find("=VERTEX_POINT(").expect("a vertex");
    let id = &text[at..];
    let id = &id[id.find("',#").unwrap() + 3..id.find(");").unwrap()];
    let line = format!("#{id}=CARTESIAN_POINT('',(");
    let start = text.find(&line).expect("its point") + line.len();
    let end = start + text[start..].find("))").unwrap();
    let mut xyz: Vec<f64> = text[start..end]
        .split(',')
        .map(|c| c.parse().unwrap())
        .collect();
    xyz[2] += 3e-4;
    let raised = format!(
        "{}{:?},{:?},{:?}{}",
        &text[..start],
        xyz[0],
        xyz[1],
        xyz[2],
        &text[end..]
    );

    let files = std::collections::BTreeMap::from([("box.step".to_string(), Some(raised))]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let update = editor.handle(dialog("file", Value::Choice("box.step".into())));
    let step = update.step.expect("a step is edited");
    assert!(step.error.is_none(), "{:?}", step.error);
    let Some(Control::Text { text, .. }) = step.presentation.dialog.get("healed_0") else {
        panic!(
            "the dialog does not say what was rebuilt: {:?}",
            step.presentation.dialog
        );
    };
    assert!(
        text.contains("by up to 3.0e-4 mm") && text.contains("1 vertices and"),
        "{text}"
    );
    assert!(editor.handle(Command::Commit).error.is_none());
}

/// The arm, exported as STEP the way the front end asks: one product for
/// the arm and one for each of the three links it places, each placement an
/// occurrence named after its step — and read back, the three links where
/// the arm has them.
#[test]
fn an_assembly_is_exported_as_step_products() {
    let mut editor = editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let text = step_file(&mut editor);
    // TODO: one product of the link for all three, once the parts of no
    // state are shared between instances (see
    // `geop_core_program::part::Part`).
    assert_eq!(
        text.matches("=PRODUCT('").count(),
        4,
        "the arm and each link"
    );
    assert!(text.contains("PRODUCT('link','link'"));
    for occurrence in ["upper", "fore", "hand"] {
        assert!(
            text.contains(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{occurrence}'")),
            "{occurrence} is placed"
        );
    }
    let bodies = read_step(&text).unwrap();
    assert_eq!(bodies.len(), 3, "the link, once where each step places it");
}

/// The bolted plate written as STEP: the standard screw and nut are named
/// by the designations their own programs carry, one product per size.
#[test]
fn an_assembly_step_file_names_its_parts() {
    let mut editor = editor();
    handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "bolted_plate".into(),
            folder: None,
        },
    );
    let text = step_file(&mut editor);
    assert!(
        text.contains("PRODUCT('ISO 4762 M4x12'"),
        "the screw's product"
    );
    assert!(text.contains("PRODUCT('plate'"), "the plate's product");
}

/// The drilled box written as STEP, as `geop compile box.geop -o box.step`
/// writes it: a Part 21 file, which a program beside it imports back to as
/// many solids as the box has, and which is meshed again as STL.
#[test]
fn a_part_written_as_step_imports_back_to_as_many_solids() {
    let program = drilled_box();
    let mut editor = loaded(editor(), program.clone());
    let text = step_file(&mut editor);
    assert!(text.starts_with("ISO-10303-21;"), "{}", &text[..40]);

    let mut import = Program::new();
    import.push(
        "imp",
        ImportStepArgs {
            file: "box.step".into(),
        },
    );
    let solids = |part: &Part| read_model(&part.current()).unwrap().solid_names().len();
    let files = BTreeMap::from([("box.step".to_string(), text.clone())]);
    assert_eq!(
        solids(&build(&import, "import.geop", files)),
        solids(&build(&program, "box.geop", BTreeMap::new())),
    );

    let files = BTreeMap::from([("box.step".to_string(), Some(text))]);
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: import,
            path: Some("import.geop".into()),
        },
    );
    let update = handled(
        &mut editor,
        Command::Export {
            exporter: "stl".into(),
            format: None,
            options: serde_json::Value::Null,
            date: String::new(),
        },
    );
    let bytes = update.export.expect("a file is written").bytes().to_vec();
    let triangles = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert_eq!(bytes.len(), 84 + 50 * triangles);
    assert!(triangles > 0);
}

/// One plane face of two squares side by side, both its bounds running
/// counter-clockwise about its normal: two outer loops, as some systems
/// write a face their kernel splits later.
const TWO_SQUARES: &str = "
#100=CARTESIAN_POINT('',(0.,0.,0.));
#101=CARTESIAN_POINT('',(1.,0.,0.));
#102=CARTESIAN_POINT('',(1.,1.,0.));
#103=CARTESIAN_POINT('',(0.,1.,0.));
#104=CARTESIAN_POINT('',(2.,0.,0.));
#105=CARTESIAN_POINT('',(3.,0.,0.));
#106=CARTESIAN_POINT('',(3.,1.,0.));
#107=CARTESIAN_POINT('',(2.,1.,0.));
#110=VERTEX_POINT('',#100);
#111=VERTEX_POINT('',#101);
#112=VERTEX_POINT('',#102);
#113=VERTEX_POINT('',#103);
#114=VERTEX_POINT('',#104);
#115=VERTEX_POINT('',#105);
#116=VERTEX_POINT('',#106);
#117=VERTEX_POINT('',#107);
#120=DIRECTION('',(1.,0.,0.));
#121=DIRECTION('',(0.,1.,0.));
#122=VECTOR('',#120,1.);
#123=VECTOR('',#121,1.);
#130=LINE('',#100,#122);
#131=LINE('',#101,#123);
#132=LINE('',#103,#122);
#133=LINE('',#100,#123);
#134=LINE('',#104,#122);
#135=LINE('',#105,#123);
#136=LINE('',#107,#122);
#137=LINE('',#104,#123);
#140=EDGE_CURVE('',#110,#111,#130,.T.);
#141=EDGE_CURVE('',#111,#112,#131,.T.);
#142=EDGE_CURVE('',#113,#112,#132,.T.);
#143=EDGE_CURVE('',#110,#113,#133,.T.);
#144=EDGE_CURVE('',#114,#115,#134,.T.);
#145=EDGE_CURVE('',#115,#116,#135,.T.);
#146=EDGE_CURVE('',#117,#116,#136,.T.);
#147=EDGE_CURVE('',#114,#117,#137,.T.);
#150=ORIENTED_EDGE('',*,*,#140,.T.);
#151=ORIENTED_EDGE('',*,*,#141,.T.);
#152=ORIENTED_EDGE('',*,*,#142,.F.);
#153=ORIENTED_EDGE('',*,*,#143,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));
#155=FACE_BOUND('',#154,.T.);
#160=ORIENTED_EDGE('',*,*,#144,.T.);
#161=ORIENTED_EDGE('',*,*,#145,.T.);
#162=ORIENTED_EDGE('',*,*,#146,.F.);
#163=ORIENTED_EDGE('',*,*,#147,.F.);
#164=EDGE_LOOP('',(#160,#161,#162,#163));
#165=FACE_BOUND('',#164,.T.);
#170=PLANE('',#8);
#171=ADVANCED_FACE('pair',(#155,#165),#170,.T.);
#172=OPEN_SHELL('',(#171));
#999=SHELL_BASED_SURFACE_MODEL('',(#172));";

/// A face with two outer loops is two faces: each loop round its own.
#[test]
fn a_face_of_two_outer_loops_is_two_faces() {
    let part = import(&file(".MILLI.,.METRE.", TWO_SQUARES));
    assert_eq!(counts(part.topology()), (2, 8, 8));
}

/// A cone written as its base disc and its side, the side bounded by the
/// base circle alone: its apex is no vertex of the file, only where the
/// side's surface comes to a point.
const CONE_WITHOUT_APEX: &str = "
#100=CARTESIAN_POINT('',(1.,0.,0.));
#101=VERTEX_POINT('',#100);
#102=CIRCLE('',#8,1.);
#103=EDGE_CURVE('',#101,#101,#102,.T.);
#104=ORIENTED_EDGE('',*,*,#103,.T.);
#105=EDGE_LOOP('',(#104));
#106=FACE_OUTER_BOUND('',#105,.T.);
#107=PLANE('',#8);
#108=ADVANCED_FACE('base',(#106),#107,.T.);
#110=ORIENTED_EDGE('',*,*,#103,.F.);
#111=EDGE_LOOP('',(#110));
#112=FACE_OUTER_BOUND('',#111,.T.);
#113=CONICAL_SURFACE('',#8,1.,0.7853981633974483);
#114=ADVANCED_FACE('side',(#112),#113,.T.);
#120=CLOSED_SHELL('',(#108,#114));
#999=MANIFOLD_SOLID_BREP('cone',#120);";

/// The cone's apex, where no loop of the file goes, is a vertex of the
/// imported solid all the same, where the side's surface comes to a point:
/// a valid cone of height 1 below its base.
#[test]
fn a_cone_without_its_apex_imports() {
    let part = import(&file(".MILLI.,.METRE.", CONE_WITHOUT_APEX));
    let (lo, hi) = bounds(part.topology());
    assert!(
        (lo[2] + 1.0).abs() < 1e-9 && hi[2].abs() < 1e-9,
        "{lo:?} {hi:?}"
    );
}
