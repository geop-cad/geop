//! Generic 2-D polygon area, containment and triangulation — no topology
//! or rendering dependency, so it
//! lives here rather than in `geop-ops-rasterize`: `geop-core-topology`'s
//! own edit code (`splice_edge_into_face`'s loop-orientation check) needs
//! it too, and topology sits below rasterize in the dependency order.

use crate::{
    plain::{self, cross2, sub},
    scalars::Scalar,
    vector::Vector2,
};

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

/// Ear-clip a (possibly non-convex) simple polygon into triangles, as
/// index triples into `poly`: `n - 2` of them for `n >= 3` corners, none
/// for fewer. Plain numbers, for rendering: a corner that rounding leaves
/// no ear at is cut off all the same.
pub fn ear_clip(poly: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    let orient_positive = plain::signed_area2(poly) >= 0.0;
    let point_in_triangle = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> bool {
        let d1 = cross2(sub(b, a), sub(p, a));
        let d2 = cross2(sub(c, b), sub(p, b));
        let d3 = cross2(sub(a, c), sub(p, c));
        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
        !(has_neg && has_pos)
    };

    let mut indices: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::with_capacity(n.saturating_sub(2));
    // Each round cuts off one corner — an ear, or, where rounding leaves
    // none, the first corner — until a triangle is left.
    for _ in 3..n {
        let m = indices.len();
        let mut clipped = false;
        for k in 0..m {
            let ip = indices[(k + m - 1) % m];
            let ic = indices[k];
            let inext = indices[(k + 1) % m];
            let turn = cross2(sub(poly[ic], poly[ip]), sub(poly[inext], poly[ic]));
            let is_convex = if orient_positive {
                turn > 0.0
            } else {
                turn < 0.0
            };
            if !is_convex {
                continue;
            }
            // A vertex coinciding with one of the ear's own corners (e.g.
            // a zero-width slit bridge's duplicated endpoint) lies on the
            // triangle's boundary by construction, not its interior —
            // don't let that reject an otherwise-valid ear.
            let contains_other = indices.iter().any(|&iq| {
                iq != ip
                    && iq != ic
                    && iq != inext
                    && poly[iq] != poly[ip]
                    && poly[iq] != poly[ic]
                    && poly[iq] != poly[inext]
                    && point_in_triangle(poly[iq], poly[ip], poly[ic], poly[inext])
            });
            if contains_other {
                continue;
            }
            triangles.push([ip, ic, inext]);
            indices.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            let m = indices.len();
            triangles.push([indices[m - 1], indices[0], indices[1]]);
            indices.remove(0);
        }
    }
    triangles.push([indices[0], indices[1], indices[2]]);
    triangles
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ear_clip_triangulates_convex_polygon_fully() {
        let poly = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let tris = ear_clip(&poly);
        let total: f64 = tris
            .iter()
            .map(|t| cross2(sub(poly[t[1]], poly[t[0]]), sub(poly[t[2]], poly[t[0]])).abs() / 2.0)
            .sum();
        assert!((total - 1.0).abs() < 1e-9);
    }
}
