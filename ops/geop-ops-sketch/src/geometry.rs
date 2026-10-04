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

pub use geop_core_sketch::plain::{
    P2, Plain, add, angle_between, cross, dist, dot, perp, rotate, scale, segments_cross, sub,
    sweep_through, unit, wrap, xy,
};

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

    #[test]
    fn polyline_middles() {
        assert_eq!(
            polyline_mid(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
            [2.0, 0.0]
        );
    }
}
