//! Two exact 2-D polygon operations, both built on Sutherland–Hodgman
//! half-plane clipping and both used by [`super::grid`] once per uv grid
//! cell:
//!
//! - [`clip_to_rect`]: clip an arbitrary (possibly non-convex) polygon to
//!   an axis-aligned rectangle. Sutherland–Hodgman only requires the
//!   *clip* region be convex — which a rectangle trivially is — regardless
//!   of how complex the subject polygon is; that's what makes it simpler
//!   and more robust here than the reverse (clipping a simple rectangle by
//!   an arbitrary polygon, which — because the *clip* side would then be
//!   the complex one — needs general polygon-polygon clipping, with all
//!   its well-known degenerate-touching-edge fragility).
//! - [`subtract_convex`]: remove a convex region from a polygon, via the
//!   same half-plane primitive applied per edge (see its own doc comment)
//!   — used for holes, since a face's trim can have several.
//!
//! Both computed directly from the boundary's own exact edges, never
//! reconstructed, so a mesh edge can never end up cutting across the
//! interior instead of following the trim.
//!
//! Plain `f64` — see [`super::grid`]'s doc comment for why that's the
//! right call for a rendering-only algorithm.

pub type Point = [f64; 2];

/// One Sutherland–Hodgman pass: keep the part of `poly` where `side(p) >=
/// 0`, inserting an interpolated vertex at every edge that crosses `side
/// == 0`. Correct for `poly` entirely inside the half-plane (returned
/// unchanged), entirely outside (empty), or straddling.
fn clip_half_plane(poly: &[Point], side: impl Fn(Point) -> f64) -> Vec<Point> {
    if poly.is_empty() {
        return Vec::new();
    }
    let n = poly.len();
    let mut out = Vec::with_capacity(n + 2);
    for i in 0..n {
        let cur = poly[i];
        let prev = poly[(i + n - 1) % n];
        let (s_cur, s_prev) = (side(cur), side(prev));
        let (cur_in, prev_in) = (s_cur >= 0.0, s_prev >= 0.0);
        if cur_in != prev_in {
            // s_prev + t*(s_cur - s_prev) == 0. `s_prev != s_cur` is
            // guaranteed here since they disagree in sign (or one is
            // exactly 0, the boundary case, which can't hit this branch
            // since `>= 0` would make both sides agree as "in").
            let t = s_prev / (s_prev - s_cur);
            out.push([
                prev[0] + t * (cur[0] - prev[0]),
                prev[1] + t * (cur[1] - prev[1]),
            ]);
        }
        if cur_in {
            out.push(cur);
        }
    }
    // A vertex lying exactly on the clip line is emitted twice — once as
    // the interpolated crossing, once as itself — which leaves a
    // zero-length edge in the result. That edge carries no direction, so
    // anything downstream that reads edge directions (`subtract_convex`'s
    // half-planes, `ear_clip`'s turns) gets a meaningless answer from it.
    // Drop them here, at the one place they are created.
    out.dedup_by(|a, b| points_equal(*a, *b));
    while out.len() > 1 && points_equal(out[0], out[out.len() - 1]) {
        out.pop();
    }
    out
}

/// Clip `subject` (any simple polygon) to `[u0, u1] x [v0, v1]`.
pub fn clip_to_rect(subject: &[Point], u0: f64, u1: f64, v0: f64, v1: f64) -> Vec<Point> {
    let mut poly = subject.to_vec();
    poly = clip_half_plane(&poly, |p| p[0] - u0);
    poly = clip_half_plane(&poly, |p| u1 - p[0]);
    poly = clip_half_plane(&poly, |p| p[1] - v0);
    poly = clip_half_plane(&poly, |p| v1 - p[1]);
    poly
}

fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
fn points_equal(a: Point, b: Point) -> bool {
    a == b
}
fn signed_area2(poly: &[Point]) -> f64 {
    (0..poly.len())
        .map(|i| cross(poly[i], poly[(i + 1) % poly.len()]))
        .sum()
}

/// `true` if `poly` turns the same way at every vertex (a degenerate
/// near-zero turn doesn't count against it either way).
pub fn is_convex(poly: &[Point]) -> bool {
    if poly.len() < 3 {
        return true;
    }
    let n = poly.len();
    let (mut pos, mut neg) = (false, false);
    for i in 0..n {
        let turn = cross(
            sub(poly[(i + 1) % n], poly[i]),
            sub(poly[(i + 2) % n], poly[(i + 1) % n]),
        );
        if turn > 1e-12 {
            pos = true;
        } else if turn < -1e-12 {
            neg = true;
        }
    }
    !(pos && neg)
}

/// `subject` (any simple polygon) with the *convex* region `hole` removed,
/// as zero or more simple polygons (a hole entirely inside `subject`
/// splits it into one piece; touching an edge or corner can still leave
/// one; fully consuming `subject` leaves none).
///
/// Exact and free of the bridging/visibility machinery a general "merge a
/// hole into one ring via a slit" approach needs (that technique is what
/// `grid` used to call this with — see its module doc comment on the bug
/// that caused: an unlucky bridge choice on an already-tiny, already-clipped
/// per-cell fragment could pick a visually-valid-looking but wrong bridge,
/// producing a sliver that read as a triangle escaping the trim). Instead:
/// a convex region is exactly the intersection of its edges' inside
/// half-planes, so its complement is the *union* of their outside
/// half-planes — split `subject` by each edge of `hole` in turn, peeling
/// off the outside part (this edge's contribution to the result) and
/// carrying only the inside part forward to the next edge. Every point of
/// `subject` ends up in exactly one peeled-off piece (outside at least one
/// edge) or the final leftover (inside every edge, i.e. inside `hole`,
/// correctly discarded) — never double-counted, unlike a naive per-edge
/// clip-and-collect would.
pub fn subtract_convex(subject: &[Point], hole: &[Point]) -> Vec<Vec<Point>> {
    if hole.len() < 3 {
        return vec![subject.to_vec()];
    }
    let hole_ccw = signed_area2(hole) >= 0.0;
    let n = hole.len();
    let mut remaining = subject.to_vec();
    let mut pieces = Vec::new();
    for i in 0..n {
        if remaining.len() < 3 {
            break;
        }
        let a = hole[i];
        let b = hole[(i + 1) % n];
        let edge = sub(b, a);
        if edge == [0.0, 0.0] {
            // A zero-length edge bounds no half-plane: its `side` is 0
            // everywhere, which would peel off *all* of `remaining` as
            // "outside the hole" — the whole cell, hole included.
            continue;
        }
        // Positive = inside `hole` relative to this edge, for either
        // winding.
        let side = |p: Point| {
            let c = cross(edge, sub(p, a));
            if hole_ccw { c } else { -c }
        };
        let outside = clip_half_plane(&remaining, |p| -side(p));
        let inside = clip_half_plane(&remaining, side);
        if outside.len() >= 3 {
            pieces.push(outside);
        }
        remaining = inside;
    }
    pieces
}

/// Ear-clip a (possibly non-convex) simple polygon into triangles, as
/// index triples into `poly`. A local plain-`f64` twin of
/// `polygon_triangulate`'s interval-scalar version — that one operates on
/// `Vector2<S>` and lives one layer below where this module's `f64` points
/// are meaningful.
pub fn ear_clip(poly: &[Point]) -> Vec<[usize; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    let orient_positive = signed_area2(poly) >= 0.0;
    let point_in_triangle = |p: Point, a: Point, b: Point, c: Point| -> bool {
        let d1 = cross(sub(b, a), sub(p, a));
        let d2 = cross(sub(c, b), sub(p, b));
        let d3 = cross(sub(a, c), sub(p, c));
        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
        !(has_neg && has_pos)
    };

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
            let turn = cross(sub(poly[ic], poly[ip]), sub(poly[inext], poly[ic]));
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
                    && !points_equal(poly[iq], poly[ip])
                    && !points_equal(poly[iq], poly[ic])
                    && !points_equal(poly[iq], poly[inext])
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
    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }
    triangles
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area(poly: &[Point]) -> f64 {
        (0..poly.len())
            .map(|i| cross(poly[i], poly[(i + 1) % poly.len()]))
            .sum::<f64>()
            .abs()
            / 2.0
    }

    #[test]
    fn rect_entirely_inside_subject_yields_the_rect() {
        let subject = vec![[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]];
        let clipped = clip_to_rect(&subject, 0.0, 1.0, 0.0, 1.0);
        assert!(
            (area(&clipped) - 1.0).abs() < 1e-9,
            "area={}",
            area(&clipped)
        );
    }

    #[test]
    fn subject_entirely_inside_rect_is_unchanged() {
        let subject = vec![[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]];
        let clipped = clip_to_rect(&subject, 0.0, 1.0, 0.0, 1.0);
        assert!(
            (area(&clipped) - 0.25).abs() < 1e-9,
            "area={}",
            area(&clipped)
        );
    }

    #[test]
    fn disjoint_gives_empty() {
        let subject = vec![[5.0, 5.0], [6.0, 5.0], [6.0, 6.0], [5.0, 6.0]];
        let clipped = clip_to_rect(&subject, 0.0, 1.0, 0.0, 1.0);
        assert!(clipped.is_empty());
    }

    #[test]
    fn l_shape_subject_clipped_by_rect_matches_expected_area() {
        // L-shape (unit square minus its top-right quadrant) as the
        // *subject*, clipped by a rect covering the whole unit square —
        // must recover the L-shape's own area exactly (0.75), including
        // through edges that exactly coincide with the clip rectangle's.
        let l_shape = vec![
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 0.5],
            [0.5, 0.5],
            [0.5, 1.0],
            [0.0, 1.0],
        ];
        let clipped = clip_to_rect(&l_shape, 0.0, 1.0, 0.0, 1.0);
        assert!(
            (area(&clipped) - 0.75).abs() < 1e-9,
            "area={}",
            area(&clipped)
        );
    }

    #[test]
    fn rect_cuts_the_notch_out_of_an_l_shape() {
        // Same L-shape, but clip only its bottom-left quadrant — entirely
        // solid material, no notch in play — must recover exactly 0.25.
        let l_shape = vec![
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 0.5],
            [0.5, 0.5],
            [0.5, 1.0],
            [0.0, 1.0],
        ];
        let clipped = clip_to_rect(&l_shape, 0.0, 0.5, 0.0, 0.5);
        assert!(
            (area(&clipped) - 0.25).abs() < 1e-9,
            "area={}",
            area(&clipped)
        );
    }

    #[test]
    fn subtract_convex_hole_inside_square() {
        let square = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let hole = vec![[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]];
        let pieces = subtract_convex(&square, &hole);
        let total: f64 = pieces.iter().map(|p| area(p)).sum();
        assert!((total - 0.75).abs() < 1e-9, "total={total}");
        // A vertex exactly on the hole's boundary (e.g. one of its own
        // corners) legitimately appears in the output; only strictly
        // *inside* the open hole region would be a bug.
        for piece in &pieces {
            for &p in piece {
                let strictly_inside_hole = p[0] > 0.25 + 1e-9
                    && p[0] < 0.75 - 1e-9
                    && p[1] > 0.25 + 1e-9
                    && p[1] < 0.75 - 1e-9;
                assert!(
                    !strictly_inside_hole,
                    "vertex {p:?} strayed inside the hole"
                );
            }
        }
    }

    #[test]
    fn subtract_convex_hole_covering_whole_subject_leaves_nothing() {
        let square = vec![[0.25, 0.25], [0.75, 0.25], [0.75, 0.75], [0.25, 0.75]];
        let hole = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        assert!(subtract_convex(&square, &hole).is_empty());
    }

    #[test]
    fn subtract_convex_hole_touching_subject_edge_exactly() {
        // The hole's right edge exactly coincides with the subject's own
        // right edge — the exact-coincidence case that broke the earlier
        // bridge-based approach.
        let square = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let hole = vec![[0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [0.5, 1.0]];
        let pieces = subtract_convex(&square, &hole);
        let total: f64 = pieces.iter().map(|p| area(p)).sum();
        assert!((total - 0.5).abs() < 1e-9, "total={total}");
    }

    #[test]
    fn ear_clip_triangulates_convex_polygon_fully() {
        let poly = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let tris = ear_clip(&poly);
        let total: f64 = tris
            .iter()
            .map(|t| cross(sub(poly[t[1]], poly[t[0]]), sub(poly[t[2]], poly[t[0]])).abs() / 2.0)
            .sum();
        assert!((total - 1.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod hole_tests {
    use super::*;

    /// A 32-gon approximating the circle of radius `r` around `c`, clockwise
    /// — the winding a face's hole boundary has.
    fn circle_cw(c: Point, r: f64) -> Vec<Point> {
        (0..32)
            .rev()
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / 32.0;
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            })
            .collect()
    }

    /// Subtracting a hole from a grid cell must leave nothing inside the
    /// hole, for a cell straddling the hole's boundary anywhere around it.
    #[test]
    fn subtracting_a_hole_leaves_nothing_inside_it() {
        let center = [0.5, 0.5];
        let hole = circle_cw(center, 0.25);
        let step = 0.125;
        for i in 0..8 {
            for j in 0..8 {
                let (u0, v0) = (i as f64 * step, j as f64 * step);
                let (u1, v1) = (u0 + step, v0 + step);
                let cell = vec![[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
                let hole_piece = clip_to_rect(&hole, u0, u1, v0, v1);
                if hole_piece.len() < 3 {
                    // Nothing of the hole in this cell — then the cell must
                    // not be inside the hole either.
                    let mid = [(u0 + u1) / 2.0, (v0 + v1) / 2.0];
                    let d = ((mid[0] - center[0]).powi(2) + (mid[1] - center[1]).powi(2)).sqrt();
                    assert!(
                        d > 0.24,
                        "cell [{u0},{u1}]x[{v0},{v1}] lies in the hole but clipping found none of it"
                    );
                    continue;
                }
                assert!(
                    is_convex(&hole_piece),
                    "a convex hole clipped to a cell stays convex: {hole_piece:?}"
                );
                for piece in subtract_convex(&cell, &hole_piece) {
                    // Degenerate slivers render as nothing; only a piece
                    // with real area can show through a hole.
                    if signed_area2(&piece).abs() / 2.0 < 1e-12 {
                        continue;
                    }
                    let centroid = [
                        piece.iter().map(|p| p[0]).sum::<f64>() / piece.len() as f64,
                        piece.iter().map(|p| p[1]).sum::<f64>() / piece.len() as f64,
                    ];
                    let d = ((centroid[0] - center[0]).powi(2) + (centroid[1] - center[1]).powi(2))
                        .sqrt();
                    assert!(
                        d > 0.24,
                        "piece {piece:?} sits inside the hole\n  cell [{u0},{u1}]x[{v0},{v1}]\n  hole fragment (area2 {}) {hole_piece:?}",
                        signed_area2(&hole_piece)
                    );
                }
            }
        }
    }
}
