//! Offsetting chains of lines and arcs, and the bands between offsets.

use super::*;
use geop_core_math::scalars::Scalar;

fn p(x: f64, y: f64) -> Vector2 {
    Vector2::from_array([Scalar::from_f64(x), Scalar::from_f64(y)])
}

/// The closed chain of lines through `corners`.
fn polygon(corners: &[[f64; 2]]) -> Chain {
    let n = corners.len();
    Chain {
        pieces: (0..n)
            .map(|i| {
                let (a, b) = (corners[i], corners[(i + 1) % n]);
                Piece {
                    curve: NurbCurve2D::line(p(a[0], a[1]), p(b[0], b[1])).unwrap(),
                    center: None,
                    name: format!("c{i}"),
                }
            })
            .collect(),
        joints: (0..n).map(|i| format!("p{i}")).collect(),
        smooth: vec![false; n],
        closed: true,
    }
}

/// Where each curve starts.
fn corners(curves: &[NurbCurve2D]) -> Vec<[f64; 2]> {
    curves
        .iter()
        .map(|c| {
            let q = c.start().unwrap();
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
    let inner = corners(&ccw.offset(Scalar::from_f64(0.1)).unwrap());
    assert!(
        close(&inner, &[[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9]]),
        "{inner:?}"
    );
    let cw = polygon(&[[0., 0.], [0., 1.], [1., 1.], [1., 0.]]);
    let outer = corners(&cw.offset(Scalar::from_f64(0.1)).unwrap());
    assert!(
        close(
            &outer,
            &[[-0.1, -0.1], [-0.1, 1.1], [1.1, 1.1], [1.1, -0.1]]
        ),
        "{outer:?}"
    );
    let inner = corners(&cw.offset(Scalar::from_f64(-0.1)).unwrap());
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
    let band = cw
        .band(Scalar::from_f64(-0.1), Scalar::from_f64(0.2))
        .unwrap();
    assert_eq!(band.len(), 2);
    let outer = corners(&band[0].curves);
    assert!(
        close(
            &outer,
            &[[-0.2, -0.2], [1.2, -0.2], [1.2, 1.2], [-0.2, 1.2]]
        ),
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
                curve: NurbCurve2D::line(p(0., 0.), p(1., 0.)).unwrap(),
                center: None,
                name: "line".into(),
            },
            Piece {
                curve: NurbCurve2D::arc(p(1., 0.), p(2., 0.), p(2., 1.), Scalar::from_f64(r))
                    .unwrap(),
                center: Some(p(1., 1.)),
                name: "arc".into(),
            },
        ],
        joints: vec!["a".into(), "b".into(), "c".into()],
        smooth: vec![false; 2],
        closed: false,
    };
    let moved = chain.offset(Scalar::from_f64(0.25)).unwrap();
    let at = |c: &NurbCurve2D, t: f64| {
        let q = c.evaluate(Scalar::from_f64(t)).unwrap();
        [q[0].to_f64(), q[1].to_f64()]
    };
    assert!(close(&[at(&moved[0], 1.0)], &[[1.0, 0.25]]));
    assert!(close(&[at(&moved[1], 0.0)], &[[1.0, 0.25]]));
    assert!(close(&[at(&moved[1], 1.0)], &[[1.75, 1.0]]));
    let mid = at(&moved[1], 0.5);
    let radius = ((mid[0] - 1.0).powi(2) + (mid[1] - 1.0).powi(2)).sqrt();
    assert!((radius - 0.75).abs() < 1e-12, "{radius}");
    assert!(chain.offset(Scalar::from_f64(1.5)).is_err());
}

/// A curve that is neither line nor arc — a cubic, as the shell traces an
/// edge — and a line it runs on into smoothly, offset: the cubic's offset
/// is fitted, proven within the fit's tolerance of the true offset, and
/// meets the line's at the joint.
#[test]
fn line_into_a_cubic() {
    let h = |x: f64, y: f64| {
        geop_core_math::vector::Vector3::from_array([x, y, 1.0].map(Scalar::from_f64))
    };
    let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
        .map(Scalar::from_f64)
        .to_vec();
    let cubic =
        NurbCurve2D::try_new(3, vec![h(1., 0.), h(2., 0.), h(3., 1.), h(3., 2.)], knots).unwrap();
    let chain = Chain {
        pieces: vec![
            Piece {
                curve: NurbCurve2D::line(p(0., 0.), p(1., 0.)).unwrap(),
                center: None,
                name: "line".into(),
            },
            Piece {
                curve: cubic.clone(),
                center: None,
                name: "cubic".into(),
            },
        ],
        joints: vec!["a".into(), "b".into(), "c".into()],
        smooth: vec![false, true],
        closed: false,
    };
    let d = Scalar::from_f64(0.25);
    let moved = chain.offset(d).unwrap();
    let joint = moved[0].end().unwrap();
    assert!(
        joint.could_be_equal(&moved[1].start().unwrap()),
        "{moved:?}"
    );
    assert!(joint.could_be_equal(&p(1.0, 0.25)), "{joint:?}");
    let tolerance = Scalar::from_f64(0.25e-6);
    let miss = cubic
        .offset_miss(d, cubic.domain(), &moved[1], tolerance)
        .unwrap();
    assert!(!miss.definitely_greater(tolerance), "{miss:?}");
}
