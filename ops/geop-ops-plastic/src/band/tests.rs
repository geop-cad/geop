//! Offsetting chains of lines and arcs, and the bands between offsets.

use super::*;
use geop_core_math::scalars::ScalInF64 as S;
use geop_ops_extrude_revolve::common::arc2;

fn p(x: f64, y: f64) -> Vector2<S> {
    Vector2::from_array([S::from_f64(x), S::from_f64(y)])
}

/// The closed chain of lines through `corners`.
fn polygon(corners: &[[f64; 2]]) -> Chain<S> {
    let n = corners.len();
    Chain {
        pieces: (0..n)
            .map(|i| {
                let (a, b) = (corners[i], corners[(i + 1) % n]);
                Piece {
                    curve: line2(p(a[0], a[1]), p(b[0], b[1])).unwrap(),
                    center: None,
                    name: format!("c{i}"),
                }
            })
            .collect(),
        joints: (0..n).map(|i| format!("p{i}")).collect(),
        closed: true,
    }
}

/// Where each curve starts.
fn corners(curves: &[NurbCurve2D<S>]) -> Vec<[f64; 2]> {
    curves
        .iter()
        .map(|c| {
            let q = point(&c.control_points[0]).unwrap();
            [q[0].to_f64(), q[1].to_f64()]
        })
        .collect()
}

fn close(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(x, y)| (x[0] - y[0]).abs() < 1e-12 && (x[1] - y[1]).abs() < 1e-12)
}

/// A counter-clockwise square moved to its left shrinks, mitred; a
/// clockwise one grows, and moved to its right shrinks.
#[test]
fn squares_offset_to_their_left() {
    let ccw = polygon(&[[0., 0.], [1., 0.], [1., 1.], [0., 1.]]);
    let inner = corners(&ccw.offset(S::from_f64(0.1)).unwrap());
    assert!(
        close(&inner, &[[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9]]),
        "{inner:?}"
    );
    let cw = polygon(&[[0., 0.], [0., 1.], [1., 1.], [1., 0.]]);
    let outer = corners(&cw.offset(S::from_f64(0.1)).unwrap());
    assert!(
        close(&outer, &[[-0.1, -0.1], [-0.1, 1.1], [1.1, 1.1], [1.1, -0.1]]),
        "{outer:?}"
    );
    let inner = corners(&cw.offset(S::from_f64(-0.1)).unwrap());
    assert!(
        close(&inner, &[[0.1, 0.1], [0.1, 0.9], [0.9, 0.9], [0.9, 0.1]]),
        "{inner:?}"
    );
}

/// The band around a clockwise square: the ring outside it, outer loop
/// first and counter-clockwise.
#[test]
fn band_around_a_hole() {
    let cw = polygon(&[[0., 0.], [0., 1.], [1., 1.], [1., 0.]]);
    let band = cw.band(S::from_f64(-0.1), S::from_f64(0.2)).unwrap();
    assert_eq!(band.len(), 2);
    let outer = corners(&band[0].curves);
    assert!(
        close(&outer, &[[-0.2, -0.2], [1.2, -0.2], [1.2, 1.2], [-0.2, 1.2]]),
        "{outer:?}"
    );
    let hole = corners(&band[1].curves);
    assert!(
        close(&hole, &[[0.1, 0.1], [0.1, 0.9], [0.9, 0.9], [0.9, 0.1]]),
        "{hole:?}"
    );
}

/// A line running on into a quarter arc turning left: the arc's offset to
/// the left shrinks around its centre, and meets the line's.
#[test]
fn line_into_arc() {
    let r = std::f64::consts::FRAC_1_SQRT_2;
    let chain = Chain {
        pieces: vec![
            Piece {
                curve: line2(p(0., 0.), p(1., 0.)).unwrap(),
                center: None,
                name: "line".into(),
            },
            Piece {
                curve: arc2(p(1., 0.), p(2., 0.), p(2., 1.), S::from_f64(r)).unwrap(),
                center: Some(p(1., 1.)),
                name: "arc".into(),
            },
        ],
        joints: vec!["a".into(), "b".into(), "c".into()],
        closed: false,
    };
    let moved = chain.offset(S::from_f64(0.25)).unwrap();
    let at = |c: &NurbCurve2D<S>, t: f64| {
        let q = c.evaluate(S::from_f64(t)).unwrap();
        [q[0].to_f64(), q[1].to_f64()]
    };
    assert!(close(&[at(&moved[0], 1.0)], &[[1.0, 0.25]]));
    assert!(close(&[at(&moved[1], 0.0)], &[[1.0, 0.25]]));
    assert!(close(&[at(&moved[1], 1.0)], &[[1.75, 1.0]]));
    let mid = at(&moved[1], 0.5);
    let radius = ((mid[0] - 1.0).powi(2) + (mid[1] - 1.0).powi(2)).sqrt();
    assert!((radius - 0.75).abs() < 1e-12, "{radius}");
    assert!(chain.offset(S::from_f64(1.5)).is_err());
}
