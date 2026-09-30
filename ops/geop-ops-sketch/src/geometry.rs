//! What editing a sketch needs of its plane's geometry beyond
//! [`geop_core_sketch::point`]: the arc a tool draws through the pointer,
//! and where a curve's label goes. Distances here are UI tolerances and
//! previews, not results the kernel reasons about; what the sketch *is*
//! comes from its solver.

use geop_core_sketch::point::{P2, add, cross, dist, dot, scale, sub};

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
