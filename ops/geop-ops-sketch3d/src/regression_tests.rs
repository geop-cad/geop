//! 3-D sketches drawn the way the surfacing tutorials draw them, built and
//! checked as bodies.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_core_sketch::space::Constraint3d;
use geop_ops_model::NamedModel;
use geop_ops_model::sketch3d::Sketch3d;
use geop_ops_model::testing::{Built, assert_valid_sheets};

use crate::{AddSketch3d, AddSketch3dArgs};

fn registry() -> GeopResult<OperationRegistry> {
    OperationRegistry::default().with(AddSketch3d)
}

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// The part of the one step `net`, drawing `s`.
fn built(s: Sketch3d) -> GeopResult<NamedModel> {
    let mut program = Program::new();
    program.push(
        "net",
        AddSketch3dArgs {
            sketch: s,
            references: Vec::new(),
        },
    );
    program.build_model(registry)
}

/// The names of the vertices and the edges of `part`, sorted.
fn names(part: &NamedModel) -> (Vec<String>, Vec<String>) {
    let mut vertices: Vec<String> = (part.topology().vertices.keys())
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    let mut edges: Vec<String> = (part.topology().edges.keys())
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    vertices.sort();
    edges.sort();
    (vertices, edges)
}

/// The network of curves a UV surface is spanned through, drawn as one 3-D
/// sketch, as the tutorials draw it: a spline along `x` and a spline along
/// `y` crossing at `(1, 1, 0)`, a point both are drawn through, and a third
/// curve starting where the first starts, at `(0, 1, 0)` — the editor
/// snaps a click onto an existing point, so both are one point of the
/// sketch.
///
/// The wire built from it used to be invalid: a vertex per point a curve
/// ends at, and an edge per curve, so the two splines crossed at a point
/// that was no vertex. A point two curves pass through is one vertex, and
/// both curves are cut there: four edges of the two splines, named after
/// the curve and the point each starts at.
#[test]
fn curves_crossing_at_a_drawn_point_build_a_valid_wire() {
    let mut s = Sketch3d::new();
    let along_x: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&x| s.add_point(v([x, 1.0, 0.0])))
        .collect();
    let cx = s.add_spline(along_x.clone());
    let crossing = along_x[1];
    let below = s.add_point(v([1.0, 0.0, 0.0]));
    let above = s.add_point(v([1.0, 2.0, 0.0]));
    let cy = s.add_spline(vec![below, crossing, above]);
    let b = s.add_point(v([0.0, 2.0, 0.0]));
    let line = s.add_line(along_x[0], b);
    let part = built(s).unwrap();
    assert_valid_sheets(&part);
    let (vertices, edges) = names(&part);
    let vertex = |p| format!("sketch3d(net,{p})");
    assert_eq!(
        vertices,
        [along_x[0], crossing, along_x[2], below, above, b].map(vertex),
        "one vertex per point a curve ends at or two curves pass through"
    );
    let mut expected = vec![
        format!("sketch3d(net,{cx},{})", along_x[0]),
        format!("sketch3d(net,{cx},{crossing})"),
        format!("sketch3d(net,{cy},{below})"),
        format!("sketch3d(net,{cy},{crossing})"),
        format!("sketch3d(net,{line})"),
    ];
    expected.sort();
    assert_eq!(edges, expected, "both splines cut at the crossing");
    let at = part.topology().vertices[&part.vertex_id(&vertex(crossing)).unwrap()].point;
    assert!(at.could_be_equal(&v([1.0, 1.0, 0.0])), "{at:?}");
}

/// The same network, each curve drawn through points of its own and the
/// points made one by coincidences: one vertex at the crossing, both
/// splines cut there, and one vertex where the line starts at the first
/// spline's start.
#[test]
fn coincident_points_are_one_vertex() {
    let mut s = Sketch3d::new();
    let along_x: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&x| s.add_point(v([x, 1.0, 0.0])))
        .collect();
    s.add_spline(along_x.clone());
    let along_y: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&y| s.add_point(v([1.0, y, 0.0])))
        .collect();
    s.add_spline(along_y.clone());
    s.constrain(Constraint3d::Coincident {
        a: along_x[1],
        b: along_y[1],
    });
    let a = s.add_point(v([0.0, 1.0, 0.0]));
    let b = s.add_point(v([0.0, 2.0, 0.0]));
    s.add_line(a, b);
    s.constrain(Constraint3d::Coincident {
        a: along_x[0],
        b: a,
    });
    let part = built(s).unwrap();
    assert_valid_sheets(&part);
    let (vertices, edges) = names(&part);
    assert_eq!(vertices.len(), 6, "{vertices:?}");
    assert_eq!(edges.len(), 5, "{edges:?}");
    assert!(
        vertices.contains(&format!("sketch3d(net,{})", along_x[1])),
        "named after the lowest id of the class: {vertices:?}"
    );
}

/// An arc's middle point is a point of it: a line starting there cuts the
/// arc in two, each half an exact arc ending at the shared vertex.
#[test]
fn a_line_from_an_arcs_middle_point_cuts_the_arc() {
    let mut s = Sketch3d::new();
    let start = s.add_point(v([-1.0, 0.0, 0.0]));
    let middle = s.add_point(v([0.0, 1.0, 0.0]));
    let end = s.add_point(v([1.0, 0.0, 0.0]));
    let arc = s.add_arc(start, middle, end);
    let up = s.add_point(v([0.0, 1.0, 2.0]));
    let line = s.add_line(middle, up);
    let part = built(s).unwrap();
    assert_valid_sheets(&part);
    let (vertices, edges) = names(&part);
    assert_eq!(vertices.len(), 4, "{vertices:?}");
    let mut expected = vec![
        format!("sketch3d(net,{arc},{start})"),
        format!("sketch3d(net,{arc},{middle})"),
        format!("sketch3d(net,{line})"),
    ];
    expected.sort();
    assert_eq!(edges, expected);
    // Each half of the arc stays on the unit circle: its middle is there.
    for name in &expected[..2] {
        let curve = &part.topology().edges[&part.edge_id(name).unwrap()].curve;
        let mid = curve.evaluate(Scalar::from_f64(0.5)).unwrap();
        let r = mid.norm();
        assert!(r.could_be_equal(Scalar::ONE), "{name}: {mid:?}");
    }
}

/// Two curves crossing where neither has a point are refused, by name: the
/// user cannot see a crossing that makes no vertex, and cutting there
/// would move the user's intent.
#[test]
fn curves_crossing_at_no_point_are_refused() {
    let mut s = Sketch3d::new();
    let a = s.add_point(v([0.0, 0.0, 0.0]));
    let b = s.add_point(v([2.0, 2.0, 0.0]));
    let c = s.add_point(v([0.0, 2.0, 0.0]));
    let d = s.add_point(v([2.0, 0.0, 0.0]));
    let rising = s.add_line(a, b);
    let falling = s.add_line(c, d);
    let err = built(s).expect_err("refused");
    let message = format!("{err:?}");
    assert!(
        message.contains(&format!("curves {rising} and {falling} cross at"))
            && message.contains("where neither has a point"),
        "{message}"
    );
}

/// Two points at one place that are not one point — the network drawn with
/// each spline through points of its own, as the original report had it —
/// are refused, by name: the kernel would see two vertices at one place, or
/// two edges crossing at no vertex.
#[test]
fn two_points_at_one_place_are_refused() {
    let mut s = Sketch3d::new();
    let along_x: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&x| s.add_point(v([x, 1.0, 0.0])))
        .collect();
    s.add_spline(along_x.clone());
    let along_y: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&y| s.add_point(v([1.0, y, 0.0])))
        .collect();
    s.add_spline(along_y.clone());
    let err = built(s).expect_err("refused");
    let message = format!("{err:?}");
    assert!(
        message.contains(&format!(
            "points {} and {} are both at",
            along_x[1], along_y[1]
        )),
        "{message}"
    );
}
