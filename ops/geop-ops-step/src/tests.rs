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
        assert!((a[c] - lo[c]).abs() < tol && (b[c] - hi[c]).abs() < tol, "bounds {a:?} {b:?}, expected {lo:?} {hi:?}");
    }
}

/// `part` written as STEP and read back: the same counts and bounds.
fn round_trip(part: &Part<S>) -> Part<S> {
    let text = write_step(part, "part").unwrap();
    let back = import(&text);
    assert_eq!(counts(back.topology()), counts(part.topology()), "counts changed");
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
    revolved_cylinder(&mut part, "c", v(1.0, 2.0, 0.0), S::from_f64(0.5), S::from_f64(2.0)).unwrap();
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
    assert!(part.face_id("import(i,s0,f0,q0)").is_ok());
    assert!(part.face_id("import(i,s0,f2)").is_ok());
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
        out += &format!("#{c}=CARTESIAN_POINT('',({:?},{:?},{:?}));\n#{vp}=VERTEX_POINT('',#{c});\n", p[0], p[1], p[2]);
        vertex.push((c, vp));
    }
    let mut edges: Vec<((usize, usize), usize)> = Vec::new();
    let mut face_ids = Vec::new();
    for face in faces {
        let mut oriented = Vec::new();
        for k in 0..face.len() {
            let (a, b) = (face[k], face[(k + 1) % face.len()]);
            let (key, forward) = if a < b { ((a, b), true) } else { ((b, a), false) };
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
            out += &format!("#{o}=ORIENTED_EDGE('',*,*,#{edge},.{}.);\n", if forward { "T" } else { "F" });
            oriented.push(format!("#{o}"));
        }
        // The plane through the first corner, its normal out of the solid.
        let (p0, p1, p2) = (points[face[0]], points[face[1]], points[face[2]]);
        let a = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let b = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        let n = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let (nd, xd, ax, plane, lp, bound, f) = (next(), next(), next(), next(), next(), next(), next());
        out += &format!(
            "#{nd}=DIRECTION('',({:?},{:?},{:?}));\n#{xd}=DIRECTION('',({:?},{:?},{:?}));\n#{ax}=AXIS2_PLACEMENT_3D('',#{},#{nd},#{xd});\n#{plane}=PLANE('',#{ax});\n#{lp}=EDGE_LOOP('',({}));\n#{bound}=FACE_OUTER_BOUND('',#{lp},.T.);\n#{f}=ADVANCED_FACE('',(#{bound}),#{plane},.T.);\n",
            n[0], n[1], n[2], a[0], a[1], a[2], vertex[face[0]].0, oriented.join(",")
        );
        face_ids.push(format!("#{f}"));
    }
    let shell = next();
    out += &format!("#{shell}=CLOSED_SHELL('',({}));\n#999=MANIFOLD_SOLID_BREP('box',#{shell});\n", face_ids.join(","));
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
    let p = face.surface.evaluate(S::from_f64(0.3), S::from_f64(0.6)).unwrap();
    assert!(p[0].mul(p[0]).add(p[1].mul(p[1])).could_be_equal(S::ONE));
}

#[test]
fn an_unsupported_entity_is_refused_by_name() {
    let text = file(".MILLI.,.METRE.", &CYLINDER.replace("#110=CYLINDRICAL_SURFACE('',#8,1.);", "#110=OFFSET_SURFACE('',#111,1.,.F.);"));
    let error = read_step::<S>(&text).err().expect("refused").to_string();
    assert!(error.contains("#110 OFFSET_SURFACE is not supported"), "{error}");
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
    assert_eq!(part.solid_names(), vec!["import(i,s0)".to_string(), "import(i,s1)".to_string()]);
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
