//! Solving 3-D sketches and building their curves.

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve3D};
use geop_core_math::{
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::*;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_array([x, y, z].map(Scalar::from_f64))
}

fn close(a: &Vector3, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// Whether `t` points along `axis`, the positive way.
fn along(t: &Vector3, axis: usize) -> bool {
    let n = t.norm().to_f64();
    (0..3).all(|k| {
        let want = if k == axis { n } else { 0.0 };
        (t[k].to_f64() - want).abs() < 1e-9 * n.max(1.0)
    })
}

/// Fixes every coordinate of `point` at `at`.
fn fix(s: &mut Sketch3d, point: PointId, at: [f64; 3]) {
    for axis in Coordinate::ALL {
        s.constrain(Constraint3d::Coordinate {
            point,
            axis,
            value: Scalar::from_f64(at[axis.index()]),
        });
    }
}

/// A sloppily drawn corner in space becomes exact: fixed at the origin, its
/// first leg along x and 2 long, its second leg along z and 1 long.
#[test]
fn polyline_is_solved_and_fully_constrained() {
    let mut s = Sketch3d::new();
    let p = [
        s.add_point(v(0.1, -0.1, 0.05)),
        s.add_point(v(2.2, 0.1, -0.1)),
        s.add_point(v(1.9, 0.2, 1.1)),
    ];
    let a = s.add_line(p[0], p[1]);
    let b = s.add_line(p[1], p[2]);
    fix(&mut s, p[0], [0.0; 3]);
    s.constrain(Constraint3d::ParallelTo {
        line: a,
        direction: v(1.0, 0.0, 0.0),
    });
    s.constrain(Constraint3d::ParallelTo {
        line: b,
        direction: v(0.0, 0.0, 1.0),
    });
    s.constrain(Constraint3d::Length {
        line: a,
        value: Scalar::from_f64(2.0),
    });
    s.constrain(Constraint3d::Distance {
        a: p[1],
        b: p[2],
        value: Scalar::ONE,
    });
    let report = s.solve().unwrap();
    assert!(report.converged, "{report:?}");
    assert_eq!(report.dof, 0, "{report:?}");
    assert!(close(&s.points[&p[0]].at, [0.0, 0.0, 0.0]));
    assert!(close(&s.points[&p[1]].at, [2.0, 0.0, 0.0]));
    assert!(close(&s.points[&p[2]].at, [2.0, 0.0, 1.0]));
    // Enclosed, each point's box holds the exact one.
    let geometry = s.enclose().unwrap();
    assert!(geometry.points[&p[2]].could_be_equal(&v(2.0, 0.0, 1.0)));
}

/// A line tangent to an arc of radius 1 at their joint: the arc turns off
/// the line's direction, wherever its far end is.
#[test]
fn line_and_arc_are_made_tangent() {
    let mut s = Sketch3d::new();
    let p = [
        s.add_point(v(0.0, 0.0, 0.0)),
        s.add_point(v(2.0, 0.0, 0.0)),
        s.add_point(v(2.8, 0.1, 0.3)),
        s.add_point(v(3.0, 0.0, 1.0)),
    ];
    let line = s.add_line(p[0], p[1]);
    let arc = s.add_arc(p[1], p[2], p[3]);
    fix(&mut s, p[0], [0.0, 0.0, 0.0]);
    fix(&mut s, p[1], [2.0, 0.0, 0.0]);
    s.constrain(Constraint3d::Tangent { a: line, b: arc });
    s.constrain(Constraint3d::Radius {
        arc,
        value: Scalar::ONE,
    });
    let report = s.solve().unwrap();
    assert!(report.converged, "{report:?}");
    let geometry = s.enclose().unwrap();
    let pieces = s.curve_nurbs(arc, &geometry).unwrap();
    let t = pieces[0].tangent(Scalar::ZERO).unwrap();
    assert!(along(&t, 0), "{t:?}");
    let radius = geometry.points[&p[3]].sub(&geometry.points[&p[1]]);
    assert!(radius.norm().to_f64() <= 2.0 + 1e-9);
}

/// An arc through three points on the unit circle, three quarters round:
/// four quarter pieces, every point of them on the circle, chained from the
/// start to the end.
#[test]
fn arc_pieces_lie_on_its_circle() {
    let mut s = Sketch3d::new();
    let a = std::f64::consts::FRAC_1_SQRT_2;
    let p = [
        s.add_point(v(1.0, 0.0, 0.0)),
        s.add_point(v(-a, 0.0, a)),
        s.add_point(v(0.0, 0.0, -1.0)),
    ];
    let arc = s.add_arc(p[0], p[1], p[2]);
    let geometry = Enclosure3d::as_drawn(&s);
    let pieces = s.curve_nurbs(arc, &geometry).unwrap();
    assert_eq!(pieces.len(), 4);
    for piece in &pieces {
        for i in 0..=8 {
            let q = piece.evaluate(Scalar::from_ratio(i, 8).unwrap()).unwrap();
            assert!(q[1].could_be_equal(Scalar::ZERO), "{q:?}");
            assert!(q.norm_sq().could_be_equal(Scalar::ONE), "{q:?}");
        }
    }
    let start = pieces[0].evaluate(Scalar::ZERO).unwrap();
    let end = pieces[3].evaluate(Scalar::ONE).unwrap();
    assert!(start.could_be_equal(&v(1.0, 0.0, 0.0)));
    assert!(end.could_be_equal(&v(0.0, 0.0, -1.0)));
    for w in pieces.windows(2) {
        let (a, b) = (
            w[0].evaluate(Scalar::ONE).unwrap(),
            w[1].evaluate(Scalar::ZERO).unwrap(),
        );
        assert!(a.could_be_equal(&b), "{a:?} vs {b:?}");
    }
}

/// A spline after a line, tangent to it, leaves along the line; its end
/// arrives along the direction it is given, the way it runs there.
#[test]
fn spline_takes_its_end_directions() {
    let mut s = Sketch3d::new();
    let p = [
        s.add_point(v(0.0, 0.0, 0.0)),
        s.add_point(v(1.0, 0.0, 0.0)),
        s.add_point(v(2.0, 1.0, 0.5)),
        s.add_point(v(3.0, 1.0, 2.0)),
    ];
    let line = s.add_line(p[0], p[1]);
    let spline = s.add_spline(vec![p[1], p[2], p[3]]);
    s.constrain(Constraint3d::Tangent { a: line, b: spline });
    s.constrain(Constraint3d::TangentTo {
        curve: spline,
        end: End::End,
        direction: v(0.0, 0.0, -1.0),
    });
    let geometry = s.enclose().unwrap();
    let curve = &s.curve_nurbs(spline, &geometry).unwrap()[0];
    let t0 = curve.tangent(Scalar::ZERO).unwrap();
    assert!(along(&t0, 0), "{t0:?}");
    let t1 = curve.tangent(Scalar::ONE).unwrap();
    assert!(along(&t1, 2), "{t1:?}");
}

/// A spline end given its direction twice is refused.
#[test]
fn spline_end_directed_twice_is_refused() {
    let mut s = Sketch3d::new();
    let p = [
        s.add_point(v(0.0, 0.0, 0.0)),
        s.add_point(v(1.0, 0.0, 0.0)),
        s.add_point(v(2.0, 1.0, 0.5)),
    ];
    let line = s.add_line(p[0], p[1]);
    let spline = s.add_spline(vec![p[1], p[2]]);
    s.constrain(Constraint3d::Tangent { a: line, b: spline });
    s.constrain(Constraint3d::TangentTo {
        curve: spline,
        end: End::Start,
        direction: v(0.0, 1.0, 0.0),
    });
    assert!(s.validate().unwrap_err().root_message().contains("twice"));
}

/// A point on an edge given from outside slides along it: dragged off it,
/// it stays on it, as near the pointer as it can.
#[test]
fn point_on_a_reference_curve_stays_on_it() {
    let mut s = Sketch3d::new();
    let f = Scalar::from_f64;
    let edge = NurbCurve::try_new(
        1,
        vec![
            Vector4::from_array([f(0.0), f(0.0), f(1.0), f(1.0)]),
            Vector4::from_array([f(4.0), f(0.0), f(1.0), f(1.0)]),
        ],
        vec![f(0.0), f(0.0), f(1.0), f(1.0)],
    )
    .unwrap();
    let reference = s.add_reference(edge);
    let p = s.add_point(v(1.0, 0.2, 0.9));
    s.constrain(Constraint3d::OnCurve {
        point: p,
        curve: reference,
    });
    let report = s.solve_with_drag(&[(p, v(3.0, 1.0, 1.0))]).unwrap();
    assert!(report.converged, "{report:?}");
    assert!(
        close(&s.points[&p].at, [3.0, 0.0, 1.0]),
        "{:?}",
        s.points[&p]
    );
}

/// Curves joined end to end, drawn in any order and direction, make one
/// chain that runs the way the oldest curve does; a branch is refused.
#[test]
fn curves_chain_up() {
    let mut s = Sketch3d::new();
    let p: Vec<PointId> = (0..4)
        .map(|i| s.add_point(v(i as f64, 0.0, (i * i) as f64)))
        .collect();
    let middle = s.add_line(p[1], p[2]);
    let first = s.add_line(p[1], p[0]);
    let last = s.add_line(p[2], p[3]);
    let chains = s.chains().unwrap();
    assert_eq!(chains.len(), 1);
    assert!(!chains[0].closed);
    let edges: Vec<(CurveId, bool)> = chains[0]
        .edges
        .iter()
        .map(|e| (e.curve, e.reversed))
        .collect();
    assert_eq!(edges, [(first, true), (middle, false), (last, false)]);
    let pieces = chains[0].to_nurbs(&s, &Enclosure3d::as_drawn(&s)).unwrap();
    assert_eq!(pieces[0].start, crate::ProfileJoint::Point(p[0]));
    assert_eq!(pieces[2].end, crate::ProfileJoint::Point(p[3]));

    let q = s.add_point(v(1.0, 5.0, 0.0));
    s.add_line(p[1], q);
    assert!(s.chains().unwrap_err().root_message().contains("branch"));
}

/// A line from a fixed point, given a direction it does not run along as
/// drawn, is found not to hold and is solved onto it: its free end moves,
/// its fixed one stays.
#[test]
fn a_line_turned_to_a_new_direction_is_solved_onto_it() {
    let mut s = Sketch3d::new();
    let p = s.add_fixed_point(v(1.0, 1.0, 1.0));
    let q = s.add_point(v(1.2, 1.1, 2.0));
    let line = s.add_line(p, q);
    s.constrain(Constraint3d::ParallelTo {
        line,
        direction: v(0.0, 0.0, 1.0),
    });
    assert!(!s.check().unwrap().converged);
    let report = s.solve().unwrap();
    assert!(report.converged, "{report:?}");
    assert!(close(&s.points[&p].at, [1.0, 1.0, 1.0]));
    let at = s.points[&q].at;
    assert!(
        (at[0].to_f64() - 1.0).abs() < 1e-9 && (at[1].to_f64() - 1.0).abs() < 1e-9,
        "{at:?}"
    );
}

/// Two splines drawn through one point, and a line from the first's start:
/// the wire cuts both splines at the shared point, into edges that end
/// exactly there, and leaves the line whole. A spline is split at the knot
/// of the shared point; an arc with a line from its middle point is rebuilt
/// as two arcs on its circle.
#[test]
fn wire_edges_are_cut_where_curves_share_a_point() {
    let mut s = Sketch3d::new();
    let x: Vec<_> = [0.0, 1.0, 2.0, 3.0]
        .map(|t| s.add_point(v(t, 1.0, 0.0)))
        .to_vec();
    let cx = s.add_spline(x.clone());
    let below = s.add_point(v(1.0, 0.0, 0.0));
    let above = s.add_point(v(1.0, 2.0, 0.0));
    let cy = s.add_spline(vec![below, x[1], above]);
    let up = s.add_point(v(0.0, 1.0, 1.0));
    let line = s.add_line(x[0], up);
    let edges = s.wire_edges().unwrap();
    let edge = |curve, start, end, cut| WireEdge {
        curve,
        start,
        end,
        cut,
    };
    assert_eq!(
        edges,
        vec![
            edge(cx, x[0], x[1], true),
            edge(cx, x[1], x[3], true),
            edge(cy, below, x[1], true),
            edge(cy, x[1], above, true),
            edge(line, x[0], up, false),
        ]
    );
    let geometry = s.enclose().unwrap();
    s.check_wire(&geometry).unwrap();
    for curve in [cx, cy] {
        let pieces = s.wire_edge_nurbs(curve, &geometry).unwrap();
        assert_eq!(pieces.len(), 2);
        let shared = v(1.0, 1.0, 0.0);
        let at = |piece: &NurbCurve3D, t: Scalar| piece.evaluate(t).unwrap();
        assert!(at(&pieces[0], Scalar::ONE).could_be_equal(&shared));
        assert!(at(&pieces[1], Scalar::ZERO).could_be_equal(&shared));
        // The pieces are the spline: they begin and end where it does, and
        // the spline's point where it is split is the shared point.
        let whole = &s.curve_nurbs(curve, &geometry).unwrap()[0];
        assert!(at(&pieces[0], Scalar::ZERO).could_be_equal(&at(whole, Scalar::ZERO)));
        assert!(at(&pieces[1], Scalar::ONE).could_be_equal(&at(whole, Scalar::ONE)));
        assert!(at(whole, whole.knot_vector[4]).could_be_equal(&shared));
    }
    assert_eq!(s.wire_edge_nurbs(line, &geometry).unwrap().len(), 1);

    let mut s = Sketch3d::new();
    let start = s.add_point(v(-1.0, 0.0, 0.0));
    let middle = s.add_point(v(0.0, 1.0, 0.0));
    let end = s.add_point(v(1.0, 0.0, 0.0));
    let arc = s.add_arc(start, middle, end);
    let up = s.add_point(v(0.0, 1.0, 2.0));
    s.add_line(middle, up);
    let geometry = s.enclose().unwrap();
    let halves = s.wire_edge_nurbs(arc, &geometry).unwrap();
    assert_eq!(halves.len(), 2);
    assert!(
        halves[0]
            .evaluate(Scalar::ONE)
            .unwrap()
            .could_be_equal(&v(0.0, 1.0, 0.0))
    );
    assert!(
        halves[1]
            .evaluate(Scalar::ZERO)
            .unwrap()
            .could_be_equal(&v(0.0, 1.0, 0.0))
    );
    for half in &halves {
        let mid = half.evaluate(Scalar::from_f64(0.5)).unwrap();
        assert!(mid.norm().could_be_equal(Scalar::ONE), "{mid:?}");
        assert!(mid[1].definitely_greater(Scalar::ZERO), "{mid:?}");
    }
}

/// Curves meeting where neither has a point, two points at one place that
/// are not one, and curves running along each other are refused by name.
#[test]
fn check_wire_refuses_undrawn_meetings() {
    let mut s = Sketch3d::new();
    let a = s.add_point(v(0.0, 0.0, 0.0));
    let b = s.add_point(v(2.0, 2.0, 0.0));
    let c = s.add_point(v(0.0, 2.0, 0.0));
    let d = s.add_point(v(2.0, 0.0, 0.0));
    let rising = s.add_line(a, b);
    let falling = s.add_line(c, d);
    let geometry = s.enclose().unwrap();
    let message = s
        .check_wire(&geometry)
        .unwrap_err()
        .root_message()
        .to_string();
    assert!(
        message.contains(&format!("curves {rising} and {falling} cross at"))
            && message.contains("neither has a point"),
        "{message}"
    );

    let mut s = Sketch3d::new();
    let a = s.add_point(v(0.0, 0.0, 0.0));
    let b = s.add_point(v(2.0, 0.0, 0.0));
    s.add_line(a, b);
    let c = s.add_point(v(1.0, 0.0, 0.0));
    let d = s.add_point(v(1.0, 2.0, 0.0));
    s.add_line(c, d);
    let e = s.add_point(v(1.0, 0.0, 0.0));
    s.add_line(c, e);
    let geometry = s.enclose().unwrap();
    let message = s
        .check_wire(&geometry)
        .unwrap_err()
        .root_message()
        .to_string();
    assert!(
        message.contains(&format!("points {c} and {e} are both at")),
        "{message}"
    );

    let mut s = Sketch3d::new();
    let a = s.add_point(v(0.0, 0.0, 0.0));
    let b = s.add_point(v(2.0, 0.0, 0.0));
    let whole = s.add_line(a, b);
    let c = s.add_point(v(1.0, 0.0, 0.0));
    let d = s.add_point(v(3.0, 0.0, 0.0));
    let along = s.add_line(c, d);
    let geometry = s.enclose().unwrap();
    let message = s
        .check_wire(&geometry)
        .unwrap_err()
        .root_message()
        .to_string();
    assert!(
        message.contains(&format!("curves {whole} and {along} run along each other")),
        "{message}"
    );
}
