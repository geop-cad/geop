//! Generic 2-D polygon area and containment — no topology or rendering
//! dependency, so it
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

/// Whether `p` lies inside the closed polylines `loops`, outer boundaries
/// and holes alike: an odd number of crossings of a ray from `p` along `+x`.
/// Only crossings that definitely happen count, so a `p` whose ray could
/// graze a corner is decided as if it did not.
pub fn loops_contain<S: Scalar>(loops: &[Vec<Vector2<S>>], p: &Vector2<S>) -> bool {
    let mut inside = false;
    for poly in loops {
        for (i, a) in poly.iter().enumerate() {
            let b = poly[(i + 1) % poly.len()];
            if a[1].definitely_greater(p[1]) != b[1].definitely_greater(p[1]) {
                let Ok(along) = p[1].sub(a[1]).div(b[1].sub(a[1])) else {
                    continue;
                };
                if a[0].add(along.mul(b[0].sub(a[0]))).definitely_greater(p[0]) {
                    inside = !inside;
                }
            }
        }
    }
    inside
}
