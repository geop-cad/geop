//! Solving 3-D sketches and building their curves.

use geop_core_geometry::nurb_curve::NurbCurve;
use geop_core_math::{
    for_all_scalars,
    scalars::{ScalInF64, Scalar},
    vector::{Vector3, Vector4},
};

use super::*;

type T = ScalInF64;

fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

fn close(a: &Vector3<T>, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// Whether `t` points along `axis`, the positive way.
fn along(t: &Vector3<T>, axis: usize) -> bool {
    let n = t.norm().to_f64();
    (0..3).all(|k| {
        let want = if k == axis { n } else { 0.0 };
        (t[k].to_f64() - want).abs() < 1e-9 * n.max(1.0)
    })
}

/// Fixes every coordinate of `point` at `at`.
fn fix(s: &mut Sketch3d<T>, point: PointId, at: [f64; 3]) {
    for axis in Coordinate::ALL {
        s.constrain(Constraint3d::Coordinate {
            point,
            axis,
            value: T::from_f64(at[axis.index()]),
        });
    }
}

/// A sloppily drawn corner in space becomes exact: fixed at the origin, its
/// first leg along x and 2 long, its second leg along z and 1 long.
#[test]
fn polyline_is_solved_and_fully_constrained() {
    let mut s = Sketch3d::<T>::new();
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
        value: T::from_f64(2.0),
    });
    s.constrain(Constraint3d::Distance {
        a: p[1],
        b: p[2],
        value: T::ONE,
    });
    let report = s.solve().unwrap();
    assert!(report.converged, "{report:?}");
    assert_eq!(report.dof, 0, "{report:?}");
    assert!(close(&s.points[&p[0]].at, [0.0, 0.0, 0.0]));
    assert!(close(&s.points[&p[1]].at, [2.0, 0.0, 0.0]));
    assert!(close(&s.points[&p[2]].at, [2.0, 0.0, 1.0]));
    // Enclosed, each point's box holds the exact one.
    let geometry = s.enclose::<T>().unwrap();
    assert!(geometry.points[&p[2]].could_be_equal(&v(2.0, 0.0, 1.0)));
}

/// A line tangent to an arc of radius 1 at their joint: the arc turns off
/// the line's direction, wherever its far end is.
#[test]
fn line_and_arc_are_made_tangent() {
    let mut s = Sketch3d::<T>::new();
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
    s.constrain(Constraint3d::Radius { arc, value: T::ONE });
    let report = s.solve().unwrap();
    assert!(report.converged, "{report:?}");
    let geometry = s.enclose::<T>().unwrap();
    let pieces = s.curve_nurbs(arc, &geometry).unwrap();
    let t = pieces[0].tangent(T::ZERO).unwrap();
    assert!(along(&t, 0), "{t:?}");
    let radius = geometry.points[&p[3]].sub(&geometry.points[&p[1]]);
    assert!(radius.norm().to_f64() <= 2.0 + 1e-9);
}

/// An arc through three points on the unit circle, three quarters round:
/// four quarter pieces, every point of them on the circle, chained from the
/// start to the end.
fn check_arc_pieces_lie_on_its_circle<S: Scalar>() {
    let mut s = Sketch3d::<T>::new();
    let a = std::f64::consts::FRAC_1_SQRT_2;
    let p = [
        s.add_point(v(1.0, 0.0, 0.0)),
        s.add_point(v(-a, 0.0, a)),
        s.add_point(v(0.0, 0.0, -1.0)),
    ];
    let arc = s.add_arc(p[0], p[1], p[2]);
    let geometry = Enclosure3d::<S>::as_drawn(&s);
    let pieces = s.curve_nurbs(arc, &geometry).unwrap();
    assert_eq!(pieces.len(), 4);
    for piece in &pieces {
        for i in 0..=8 {
            let q = piece.evaluate(S::from_ratio(i, 8).unwrap()).unwrap();
            assert!(q[1].could_be_equal(S::ZERO), "{q:?}");
            assert!(q.norm_sq().could_be_equal(S::ONE), "{q:?}");
        }
    }
    let start = pieces[0].evaluate(S::ZERO).unwrap();
    let end = pieces[3].evaluate(S::ONE).unwrap();
    assert!(start.could_be_equal(&v(1.0, 0.0, 0.0)));
    assert!(end.could_be_equal(&v(0.0, 0.0, -1.0)));
    for w in pieces.windows(2) {
        let (a, b) = (
            w[0].evaluate(S::ONE).unwrap(),
            w[1].evaluate(S::ZERO).unwrap(),
        );
        assert!(a.could_be_equal(&b), "{a:?} vs {b:?}");
    }
}
#[test]
fn arc_pieces_lie_on_its_circle() {
    for_all_scalars!(check_arc_pieces_lie_on_its_circle);
}

/// A spline after a line, tangent to it, leaves along the line; its end
/// arrives along the direction it is given, the way it runs there.
#[test]
fn spline_takes_its_end_directions() {
    let mut s = Sketch3d::<T>::new();
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
    let geometry = s.enclose::<T>().unwrap();
    let curve = &s.curve_nurbs(spline, &geometry).unwrap()[0];
    let t0 = curve.tangent(T::ZERO).unwrap();
    assert!(along(&t0, 0), "{t0:?}");
    let t1 = curve.tangent(T::ONE).unwrap();
    assert!(along(&t1, 2), "{t1:?}");
}

/// A spline end given its direction twice is refused.
#[test]
fn spline_end_directed_twice_is_refused() {
    let mut s = Sketch3d::<T>::new();
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
    let mut s = Sketch3d::<T>::new();
    let f = T::from_f64;
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
    let mut s = Sketch3d::<T>::new();
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
    let pieces = chains[0]
        .to_nurbs(&s, &Enclosure3d::<T>::as_drawn(&s))
        .unwrap();
    assert_eq!(pieces[0].start, crate::ProfileJoint::Point(p[0]));
    assert_eq!(pieces[2].end, crate::ProfileJoint::Point(p[3]));

    let q = s.add_point(v(1.0, 5.0, 0.0));
    s.add_line(p[1], q);
    assert!(s.chains().unwrap_err().root_message().contains("branch"));
}
