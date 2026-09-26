//! Generic 2-D polygon area — no topology or rendering dependency, so it
//! lives here rather than in `geop-ops-rasterize`: `geop-core-topology`'s
//! own edit code (`splice_edge_into_face`'s loop-orientation check) needs
//! it too, and topology sits below rasterize in the dependency order.

use crate::{scalars::Scalar, vector::Vector2};

/// Twice the signed area of `poly` (shoelace formula): positive for CCW,
/// negative for CW.
fn signed_area2<S: Scalar>(poly: &[Vector2<S>]) -> S {
    let n = poly.len();
    let mut sum = S::ZERO;
    for i in 0..n {
        let j = (i + 1) % n;
        sum = sum.add(poly[i][0].mul(poly[j][1]).sub(poly[j][0].mul(poly[i][1])));
    }
    sum
}

/// Signed area of `poly` (positive for CCW, negative for CW).
pub fn polygon_signed_area<S: Scalar>(poly: &[Vector2<S>]) -> S {
    signed_area2(poly).div(S::TWO).unwrap_or(S::ZERO)
}
