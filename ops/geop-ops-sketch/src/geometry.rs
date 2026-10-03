//! The sketch editor's own plane geometry, in plain numbers: where the
//! pointer is, the arc a tool draws through it, where a curve's label goes.
//! This is the editor's side of the browser boundary — pointer positions,
//! previews and UI tolerances, never results the kernel reasons about.
//! What the sketch *is* is its design data, in [`Design`] scalars, read here
//! with [`xy`] and written back as sharp values; what it means comes from
//! its solver.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};
use geop_core_sketch::{ProfileLoop, Sketch, profile::curve_polyline};
use geop_ops::Design;

/// `[x, y]`, as the pointer and the screen have it.
pub type P2 = [f64; 2];

pub fn add(a: P2, b: P2) -> P2 {
    [a[0] + b[0], a[1] + b[1]]
}

pub fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub fn scale(a: P2, s: f64) -> P2 {
    [a[0] * s, a[1] * s]
}

pub fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub fn dist(a: P2, b: P2) -> f64 {
    let d = sub(a, b);
    d[0].hypot(d[1])
}

/// A number from the pointer or a dialog, as design data: sharp.
pub fn design(x: f64) -> Design {
    Design::from_f64(x)
}

/// Where the sketch point `p` is drawn.
pub fn xy(sketch: &Sketch<Design>, p: geop_core_sketch::PointId) -> P2 {
    let q = sketch.points[&p].xy();
    [q[0].to_f64(), q[1].to_f64()]
}

/// Points as drawn.
fn plain(points: GeopResult<Vec<Vector2<Design>>>) -> Vec<P2> {
    points
        .map(|points| {
            points
                .iter()
                .map(|q| [q[0].to_f64(), q[1].to_f64()])
                .collect()
        })
        .unwrap_or_default()
}

/// The polyline a curve is drawn as (empty where it cannot be drawn).
pub fn polyline(sketch: &Sketch<Design>, curve: geop_core_sketch::CurveId) -> Vec<P2> {
    plain(curve_polyline(sketch, curve))
}

/// The polyline a loop is drawn as (empty where it cannot be drawn).
pub fn loop_polyline(sketch: &Sketch<Design>, lp: &ProfileLoop) -> Vec<P2> {
    plain(lp.polyline(sketch))
}

/// The signed sweep of the arc from `s` to `e` through `p`, by the
/// inscribed angle theorem: counter-clockwise (positive) when `p` lies right
/// of the chord. Not finite when `p` is on the chord's line.
pub fn sweep_through(s: P2, e: P2, p: P2) -> f64 {
    let (a, b) = (sub(s, p), sub(e, p));
    let phi = cross(a, b).atan2(dot(a, b)).abs();
    let sign = if cross(sub(e, s), sub(p, s)) < 0.0 {
        1.0
    } else {
        -1.0
    };
    sign * (std::f64::consts::TAU - 2.0 * phi)
}

/// The point halfway along a polyline, by length: where a curve's label
/// goes.
pub fn polyline_mid(poly: &[P2]) -> P2 {
    let total: f64 = poly.windows(2).map(|w| dist(w[0], w[1])).sum();
    let mut along = 0.0;
    for w in poly.windows(2) {
        let d = dist(w[0], w[1]);
        if along + d >= total / 2.0 && d > 0.0 {
            return add(w[0], scale(sub(w[1], w[0]), (total / 2.0 - along) / d));
        }
        along += d;
    }
    poly.first().copied().unwrap_or([0.0, 0.0])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A half circle through the point above the chord's middle turns
    /// clockwise, one below it counter-clockwise.
    #[test]
    fn sweeps_through_a_point() {
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, 1.0]);
        assert!((s + std::f64::consts::PI).abs() < 1e-12, "{s}");
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, -1.0]);
        assert!((s - std::f64::consts::PI).abs() < 1e-12, "{s}");
    }

    #[test]
    fn polyline_middles() {
        assert_eq!(
            polyline_mid(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
            [2.0, 0.0]
        );
    }
}
