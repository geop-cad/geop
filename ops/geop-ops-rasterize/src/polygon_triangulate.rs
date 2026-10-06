//! 2-D ear-clipping triangulation of a simple polygon with holes.
//!
//! Used to rasterize trimmed faces (faces with more than one edge_loop): such faces are
//! planar/affine in this codebase, so triangulating the `(u, v)`
//! outer-loop-minus-holes polygon and mapping each triangle through
//! `surface.eval_at` gives an exact 3-D triangulation.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

/// A triangle of a `(u, v)` triangulation, as its three corners.
pub type Triangle2<S> = (Vector2<S>, Vector2<S>, Vector2<S>);

pub use geop_core_math::polygon::polygon_signed_area;

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

/// 2-D cross product (perp-dot) of `a` and `b`.
fn cross2<S: Scalar>(a: Vector2<S>, b: Vector2<S>) -> S {
    a[0].mul(b[1]).sub(a[1].mul(b[0]))
}

fn to_f64_2<S: Scalar>(p: Vector2<S>) -> (f64, f64) {
    (p[0].to_f64(), p[1].to_f64())
}

/// True if `p` lies inside (or on the edge_loop of) triangle `(a, b, c)`,
/// using same-sign barycentric tests. Ambiguous (interval-straddling) sign
/// comparisons are treated as "inside": this is conservative for ear-clipping
/// (an ambiguous result rejects the candidate ear rather than risking an
/// invalid triangulation).
fn point_in_triangle<S: Scalar>(
    p: Vector2<S>,
    a: Vector2<S>,
    b: Vector2<S>,
    c: Vector2<S>,
) -> bool {
    let d1 = cross2(b.sub(&a), p.sub(&a));
    let d2 = cross2(c.sub(&b), p.sub(&b));
    let d3 = cross2(a.sub(&c), p.sub(&c));

    let all_nonneg = !d1.definitely_less(S::ZERO)
        && !d2.definitely_less(S::ZERO)
        && !d3.definitely_less(S::ZERO);
    let all_nonpos = !d1.definitely_greater(S::ZERO)
        && !d2.definitely_greater(S::ZERO)
        && !d3.definitely_greater(S::ZERO);
    all_nonneg || all_nonpos
}

/// Ear-clipping triangulation of a simple polygon (no holes), returned as
/// index triples into `poly`. Robust to interval scalars: convexity and
/// point-in-triangle tests use three-valued comparisons, treating ambiguous
/// results conservatively (reject the candidate ear); a fallback guarantees
/// termination on degenerate input.
fn ear_clip<S: Scalar>(poly: &[Vector2<S>]) -> Vec<(usize, usize, usize)> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }

    let total = signed_area2(poly);
    // Orientation used to decide which turn-sign counts as "convex". Default
    // to CCW (positive) if the total area is ambiguous (near-zero).
    let orient_positive = !total.definitely_less(S::ZERO);

    let mut indices: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::with_capacity(n.saturating_sub(2));

    let max_iters = n * n + 8;
    let mut iters = 0;
    while indices.len() > 3 && iters < max_iters {
        iters += 1;
        let m = indices.len();
        let mut clipped = false;
        for k in 0..m {
            let ip = indices[(k + m - 1) % m];
            let ic = indices[k];
            let inext = indices[(k + 1) % m];

            let turn = cross2(poly[ic].sub(&poly[ip]), poly[inext].sub(&poly[ic]));
            let is_convex = if orient_positive {
                turn.definitely_greater(S::ZERO)
            } else {
                turn.definitely_less(S::ZERO)
            };
            if !is_convex {
                continue;
            }

            // A vertex that coincides with one of the triangle's own corners
            // (e.g. the duplicated bridge endpoints introduced by
            // `merge_hole_into`'s zero-width slit) lies on the triangle's
            // edge_loop by construction, not in its interior — don't let that
            // reject an otherwise-valid ear.
            let contains_other = indices.iter().any(|&iq| {
                iq != ip
                    && iq != ic
                    && iq != inext
                    && !points_equal(poly[iq], poly[ip])
                    && !points_equal(poly[iq], poly[ic])
                    && !points_equal(poly[iq], poly[inext])
                    && point_in_triangle(poly[iq], poly[ip], poly[ic], poly[inext])
            });
            if contains_other {
                continue;
            }

            triangles.push((ip, ic, inext));
            indices.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            // Degenerate/ambiguous configuration: clip the first vertex
            // regardless, to guarantee termination. `TriangleFace::try_new`
            // (in the caller) will discard genuinely-degenerate triangles.
            let m = indices.len();
            let ip = indices[m - 1];
            let ic = indices[0];
            let inext = indices[1];
            triangles.push((ip, ic, inext));
            indices.remove(0);
        }
    }
    if indices.len() == 3 {
        triangles.push((indices[0], indices[1], indices[2]));
    }
    triangles
}

/// Squared Euclidean distance between `a` and `b`, via `to_f64()`. Used only
/// to heuristically order bridge candidates — does not affect correctness.
fn dist2_f64<S: Scalar>(a: Vector2<S>, b: Vector2<S>) -> f64 {
    let (ax, ay) = to_f64_2(a);
    let (bx, by) = to_f64_2(b);
    let dx = ax - bx;
    let dy = ay - by;
    dx * dx + dy * dy
}

fn points_equal<S: Scalar>(a: Vector2<S>, b: Vector2<S>) -> bool {
    a.could_be_equal(&b)
}

/// True if segments `(p1,p2)` and `(p3,p4)` properly cross (each segment's
/// endpoints lie strictly on opposite sides of the other segment's line).
/// Ambiguous (collinear/touching) configurations are treated as
/// non-intersecting — shared endpoints are handled separately by the caller.
fn segments_properly_intersect<S: Scalar>(
    p1: Vector2<S>,
    p2: Vector2<S>,
    p3: Vector2<S>,
    p4: Vector2<S>,
) -> bool {
    let d1 = cross2(p2.sub(&p1), p3.sub(&p1));
    let d2 = cross2(p2.sub(&p1), p4.sub(&p1));
    let d3 = cross2(p4.sub(&p3), p1.sub(&p3));
    let d4 = cross2(p4.sub(&p3), p2.sub(&p3));

    let opposite = |x: S, y: S| {
        (x.definitely_greater(S::ZERO) && y.definitely_less(S::ZERO))
            || (x.definitely_less(S::ZERO) && y.definitely_greater(S::ZERO))
    };

    opposite(d1, d2) && opposite(d3, d4)
}

/// Even-odd point-in-polygon test via `to_f64()` ray casting. Used only to
/// pick a good bridge target when merging a hole into the outer polygon — not
/// correctness-critical (the final ear-clipping result is validated with
/// exact comparisons).
fn point_in_polygon<S: Scalar>(p: Vector2<S>, poly: &[Vector2<S>]) -> bool {
    let n = poly.len();
    let (px, py) = to_f64_2(p);
    let mut inside = false;
    for i in 0..n {
        let (ax, ay) = to_f64_2(poly[i]);
        let (bx, by) = to_f64_2(poly[(i + 1) % n]);
        if (ay > py) != (by > py) {
            let x_intersect = ax + (py - ay) / (by - ay) * (bx - ax);
            if px < x_intersect {
                inside = !inside;
            }
        }
    }
    inside
}

/// True if the bridge segment `h -> m` (a candidate connection from a hole
/// vertex to a vertex of `poly`, the polygon the holes merged so far are
/// part of) crosses no edge — of `poly`, nor of the holes still to merge,
/// the one `h` is on among them — and its midpoint lies inside `poly` and
/// outside those holes.
///
/// The holes still to merge are obstacles too: a bridge from a hole's
/// rightmost point to a vertex left of it runs straight across the hole,
/// and the slit it leaves makes the ear clipping fill the hole and overlap
/// its own triangles.
fn is_bridge_visible<S: Scalar>(
    poly: &[Vector2<S>],
    holes: &[&[Vector2<S>]],
    h: Vector2<S>,
    m: Vector2<S>,
) -> bool {
    if points_equal(h, m) {
        return false;
    }
    let crosses = |ring: &[Vector2<S>]| {
        let n = ring.len();
        (0..n).any(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            let touches = [a, b]
                .iter()
                .any(|&p| points_equal(p, h) || points_equal(p, m));
            !touches && segments_properly_intersect(h, m, a, b)
        })
    };
    if crosses(poly) || holes.iter().any(|ring| crosses(ring)) {
        return false;
    }
    let two = S::TWO;
    let mid = Vector2::from_array([
        h[0].add(m[0]).div(two).unwrap_or(h[0]),
        h[1].add(m[1]).div(two).unwrap_or(h[1]),
    ]);
    point_in_polygon(mid, poly) && !holes.iter().any(|ring| point_in_polygon(mid, ring))
}

/// Whether the direction from `poly[i]` to `h` points into `poly` — a CCW
/// polygon — at its vertex `i`: lies within the interior angle between the
/// edge leaving it and the edge arriving at it. A vertex a slit runs to
/// appears twice, once on either side of the slit, and only the copy whose
/// angle holds the bridge can take it: bridging to the other one runs the
/// new slit through the old one.
fn opens_towards<S: Scalar>(poly: &[Vector2<S>], i: usize, h: Vector2<S>) -> bool {
    let n = poly.len();
    let m = poly[i];
    let (out, back, d) = (
        poly[(i + 1) % n].sub(&m),
        poly[(i + n - 1) % n].sub(&m),
        h.sub(&m),
    );
    let inside = |a: Vector2<S>, b: Vector2<S>| cross2(a, b).definitely_greater(S::ZERO);
    if cross2(out, back).definitely_less(S::ZERO) {
        // Reflex: anywhere but between the edges on the outside.
        inside(out, d) || inside(d, back)
    } else {
        // Convex: strictly between the two edges.
        inside(out, d) && inside(d, back)
    }
}

/// Merge `hole` (already oriented CW) into `merged` (already oriented CCW) by
/// finding a visible bridge — one crossing neither `merged`, nor `hole`, nor
/// the holes `pending` to merge after it — and splicing the hole's vertices
/// in, producing a single simple polygon with a zero-width slit.
fn merge_hole_into<S: Scalar>(
    merged: &mut Vec<Vector2<S>>,
    hole: &[Vector2<S>],
    pending: &[&[Vector2<S>]],
) {
    // Bridge start: the hole vertex with max x (tie-break max y). This choice
    // only affects which bridge is found, not correctness.
    let hi = (1..hole.len()).fold(0usize, |best, i| {
        let (bx, by) = to_f64_2(hole[best]);
        let (ix, iy) = to_f64_2(hole[i]);
        if ix > bx || (ix == bx && iy > by) {
            i
        } else {
            best
        }
    });
    let h = hole[hi];
    let obstacles: Vec<&[Vector2<S>]> = std::iter::once(hole)
        .chain(pending.iter().copied())
        .collect();

    let mut candidates: Vec<usize> = (0..merged.len()).collect();
    candidates.sort_by(|&a, &b| {
        dist2_f64(h, merged[a])
            .partial_cmp(&dist2_f64(h, merged[b]))
            .unwrap()
    });

    let mi = candidates
        .into_iter()
        .find(|&i| {
            opens_towards(merged, i, h) && is_bridge_visible(merged, &obstacles, h, merged[i])
        })
        .unwrap_or(0);
    let m = merged[mi];

    let mut new_merged = Vec::with_capacity(merged.len() + hole.len() + 2);
    new_merged.extend_from_slice(&merged[0..=mi]);
    for k in 0..hole.len() {
        new_merged.push(hole[(hi + k) % hole.len()]);
    }
    new_merged.push(h);
    new_merged.push(m);
    new_merged.extend_from_slice(&merged[mi + 1..]);

    *merged = new_merged;
}

/// Merge `outer` (a simple polygon) and `holes` (simple polygons strictly
/// inside `outer` and disjoint from each other) into one simple polygon —
/// each hole spliced in via a zero-width slit bridge (see
/// [`merge_hole_into`]) — oriented CCW with every hole CW. The result
/// traces exactly the trimmed region's boundary, so any triangulator (or,
/// in `adaptive`, a constrained-Delaunay triangulator) that respects it as
/// a closed ring automatically excludes the holes without further
/// bookkeeping.
pub(crate) fn merge_outer_and_holes<S: Scalar>(
    outer: &[Vector2<S>],
    holes: &[Vec<Vector2<S>>],
) -> Vec<Vector2<S>> {
    let mut merged: Vec<Vector2<S>> = outer.to_vec();
    if signed_area2(&merged).definitely_less(S::ZERO) {
        merged.reverse();
    }

    let holes: Vec<Vec<Vector2<S>>> = holes
        .iter()
        .filter(|hole| hole.len() >= 3)
        .map(|hole| {
            let mut h = hole.clone();
            if !signed_area2(&h).definitely_less(S::ZERO) {
                h.reverse();
            }
            h
        })
        .collect();
    for (i, hole) in holes.iter().enumerate() {
        let pending: Vec<&[Vector2<S>]> = holes[i + 1..].iter().map(Vec::as_slice).collect();
        merge_hole_into(&mut merged, hole, &pending);
    }
    merged
}

/// Triangulate `outer` (a simple polygon) minus `holes` (simple polygons
/// strictly inside `outer` and disjoint from each other), returning
/// triangles as `(a, b, c)` vertex triples.
pub fn triangulate_with_holes<S: Scalar>(
    outer: &[Vector2<S>],
    holes: &[Vec<Vector2<S>>],
) -> GeopResult<Vec<Triangle2<S>>> {
    if outer.len() < 3 {
        return Err(geop_core_math::geop_error::GeopError::new(
            "triangulate_with_holes: outer polygon needs >= 3 vertices",
        ));
    }

    let merged = merge_outer_and_holes(outer, holes);
    let triples = ear_clip(&merged);
    Ok(triples
        .into_iter()
        .map(|(a, b, c)| (merged[a], merged[b], merged[c]))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;

    fn check_square_with_hole<S: Scalar>() {
        let f = S::from_f64;
        let c = |x: f64, y: f64| Vector2::from_array([f(x), f(y)]);
        let outer = vec![c(0.0, 0.0), c(1.0, 0.0), c(1.0, 1.0), c(0.0, 1.0)];
        let hole = vec![c(0.25, 0.25), c(0.75, 0.25), c(0.75, 0.75), c(0.25, 0.75)];

        let tris = triangulate_with_holes(&outer, &[hole]).unwrap();
        assert!(!tris.is_empty());

        let mut total = S::ZERO;
        for (a, b, c2) in &tris {
            total = total.add(cross2(b.sub(a), c2.sub(a)).abs());
        }
        // `total` = sum of |2 * signed triangle area| = 2 * (outer area - hole area).
        let expected = f(2.0 * (1.0 - 0.25));
        assert!(
            total.could_be_equal(expected),
            "total={total}, expected={expected}"
        );
    }
    #[test]
    fn square_with_hole() {
        for_all_scalars!(check_square_with_hole);
    }

    /// `n` points round the circle of `radius` about `(x, y)`, clockwise or
    /// not.
    fn circle<S: Scalar>(x: f64, y: f64, radius: f64, n: usize) -> Vec<Vector2<S>> {
        (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                Vector2::from_array([
                    S::from_f64(x + radius * a.cos()),
                    S::from_f64(y + radius * a.sin()),
                ])
            })
            .collect()
    }

    /// The area `tris` cover, counting overlaps twice.
    fn covered<S: Scalar>(tris: &[(Vector2<S>, Vector2<S>, Vector2<S>)]) -> f64 {
        tris.iter()
            .map(|(a, b, c)| cross2(b.sub(a), c.sub(a)).abs().to_f64() / 2.0)
            .sum()
    }

    /// A rectangle with two round holes, as a sketch's region is: the
    /// triangles cover it exactly once.
    fn check_rectangle_with_two_round_holes<S: Scalar>() {
        let c = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
        let outer = vec![c(-2.9, -1.4), c(2.9, -1.4), c(2.9, 1.4), c(-2.9, 1.4)];
        let holes = [
            circle::<S>(-1.75, 0.0, 0.63, 64),
            circle::<S>(1.75, 0.0, 0.5, 64),
        ];
        let area = |h: &[Vec<Vector2<S>>]| {
            polygon_signed_area(&outer).to_f64().abs()
                - h.iter()
                    .map(|h| polygon_signed_area(h).to_f64().abs())
                    .sum::<f64>()
        };
        for holes in [&holes[..1], &holes[1..], &holes[..]] {
            let tris = triangulate_with_holes(&outer, holes).unwrap();
            let expected = area(holes);
            assert!(
                (covered(&tris) - expected).abs() < 1e-6,
                "{} vs {expected} with {} holes",
                covered(&tris),
                holes.len()
            );
        }
    }
    #[test]
    fn rectangle_with_two_round_holes() {
        for_all_scalars!(check_rectangle_with_two_round_holes);
    }

    fn check_polygon_signed_area<S: Scalar>() {
        let f = S::from_f64;
        let c = |x: f64, y: f64| Vector2::from_array([f(x), f(y)]);
        let ccw = vec![c(0.0, 0.0), c(1.0, 0.0), c(1.0, 1.0), c(0.0, 1.0)];
        assert!(polygon_signed_area(&ccw).could_be_equal(f(1.0)));

        let cw: Vec<_> = ccw.into_iter().rev().collect();
        assert!(polygon_signed_area(&cw).could_be_equal(f(-1.0)));
    }
    #[test]
    fn polygon_signed_area_sign() {
        for_all_scalars!(check_polygon_signed_area);
    }
}
