//! Face rasterization: an axis-aligned `(u, v)` grid, sized to the face's
//! own curvature, with every cell clipped exactly against the trim
//! boundary.
//!
//! Deliberately simple, after a constrained-Delaunay approach (now removed) turned out both too slow (the same triangle set could be
//! reached by directly rasterizing a grid, without ever needing an
//! incremental triangulation to converge) and occasionally wrong (a
//! *reconstructed* Delaunay edge near the trim can end up connecting two
//! vertices without actually following the boundary between them, cutting
//! across the interior instead of tracing it). A grid cell's edges are
//! either untouched (fully inside the trim) or literally computed by
//! [`super::clip`] as the exact intersection with the boundary — there is
//! no reconstruction step that could get an edge wrong.
//!
//! Pipeline:
//! 1. Sample the outer boundary and every hole (as before), and simplify
//!    away redundant collinear points.
//! 2. Pick a uniform grid resolution: start from `n`, and keep doubling it
//!    until every cell's flat approximation is within a curvature-derived
//!    tolerance of the true surface (so a flat face stays coarse and a
//!    curved one gets whatever resolution its curvature actually needs —
//!    still simple, just not spatially varying the way a quadtree would).
//! 3. Split the outer boundary and every hole into convex parts — itself,
//!    if it is convex, else its triangles — and clip each part to the grid
//!    cells it overlaps ([`super::clip::clip_to_rect`], exact for a convex
//!    part: one convex piece, or none). Then in each cell subtract the
//!    holes' pieces from the outer boundary's
//!    ([`super::clip::subtract_convex`]), and ear-clip whatever's left
//!    ([`geop_core_math::polygon::ear_clip`]).
//!
//!    Three earlier versions of this step turned out wrong. The first
//!    globally merged outer+holes into one ring (via a zero-width "slit"
//!    bridge, since removed) and
//!    clipped *that* per cell: a slit is one specific pair of edges,
//!    traversed in each direction once, and if a grid cell boundary
//!    happened to cross it the two traversals could clip to slightly
//!    different results, leaving a sliver of the slit exposed as a real —
//!    visibly wrong — mesh edge. The second instead re-derived that same
//!    slit-bridge technique *per cell*, on each cell's own already-clipped
//!    outer/hole fragments — better (no more cross-cell inconsistency),
//!    but the bridge-visibility heuristic could still pick a technically-
//!    valid-looking bridge across the wrong part of a small fragment,
//!    producing the same kind of escaping sliver. [`super::clip::subtract_convex`]
//!    has no bridge/visibility step to get wrong: it computes the exact
//!    complement of a convex hole directly, via the same half-plane
//!    primitive `clip_to_rect` itself is built from.
//!
//!    A third clipped the (possibly concave) outer boundary to each cell
//!    directly, subtracting holes the same way: Sutherland–Hodgman returns a
//!    boundary that enters a cell twice as one polygon, its two pieces
//!    joined by a zero-width bridge along the cell's edge, and ear-clipping
//!    that covered the gap between them — a face whose trim went around a
//!    hole was drawn across it.
//!
//! Plain `f64` throughout — see [`super::clip`]'s doc comment for why
//! that's the right call for a rendering-only algorithm even in an
//! otherwise interval-exact kernel.

use std::collections::HashMap;

use geop_core_math::{
    geop_error::GeopResult,
    plain::{bounds, distance, dot, sub},
    polygon::ear_clip,
    scalars::Scalar,
    solvers::refine::{Refine, refine},
    vector::{Vector2, Vector3},
};
use geop_core_topology::{Face, Model, boundary::BoundaryType, uv_loop::sample_loop_to_polygon};

use crate::clip::{Point, clip_to_rect, is_convex, subtract_convex};

/// Where the grid starts, before curvature refines it: a flat face needs
/// nothing finer, whatever `n` the caller asked for, since its cells
/// approximate it exactly and the trim is clipped against them exactly.
const MIN_GRID_N: usize = 2;
/// Grid resolution never grows past this (in either direction), how ever
/// far short of tolerance a face's curvature would otherwise ask for: a
/// point budget.
const MAX_GRID_N: usize = 128;
/// How many times resolution may double while searching for one that
/// meets tolerance.
const MAX_DOUBLINGS: usize = 7;

/// Perpendicular distance from `p` to segment `a..b` — see `grid`'s
/// module doc comment on why curvature is measured this way (immune to a
/// merely non-uniformly parametrized but still flat surface).
pub(crate) fn dist_point_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub(b, a);
    let len2 = dot(ab, ab);
    let t = if len2 > 1e-18 {
        (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    distance(p, closest)
}

fn dist_point_to_segment2(p: Point, a: Point, b: Point) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let t = if len2 > 1e-18 {
        ((ap[0] * ab[0] + ap[1] * ab[1]) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    ((p[0] - closest[0]).powi(2) + (p[1] - closest[1]).powi(2)).sqrt()
}

/// A face's `(u, v)` domain, in plain `f64`, and a clamp into it — see
/// [`Samples`]: plain-`f64` grid/midpoint arithmetic on a
/// value already at a domain edge can legitimately land a ULP or two
/// outside it, which a real `Scalar` interval never does (its width
/// already encloses the true value) but a degenerate zero-width interval
/// reconstructed via `Scalar::from_f64` has no such margin, so the kernel's
/// (correctly strict) domain check can reject it.
#[derive(Clone, Copy)]
struct Domain {
    u: (f64, f64),
    v: (f64, f64),
}

impl Domain {
    fn of(face: &Face) -> Self {
        let (u0, u1) = face.surface.domain_u();
        let (v0, v1) = face.surface.domain_v();
        Self {
            u: (u0.to_f64(), u1.to_f64()),
            v: (v0.to_f64(), v1.to_f64()),
        }
    }
    fn clamp(&self, p: Point) -> Point {
        [
            p[0].clamp(self.u.0, self.u.1),
            p[1].clamp(self.v.0, self.v.1),
        ]
    }
}

/// A face's surface at the `(u, v)` its mesh samples, each evaluated once:
/// a grid corner is shared by up to six triangles, and by every finer grid
/// the refinement measures, whose cell edges' midpoints are the next grid's
/// corners.
pub(crate) struct Samples<'a> {
    face: &'a Face,
    domain: Domain,
    points: HashMap<[u64; 2], Vector3>,
    normals: HashMap<[u64; 2], Option<Vector3>>,
}

impl<'a> Samples<'a> {
    pub(crate) fn new(face: &'a Face) -> Self {
        Self {
            face,
            domain: Domain::of(face),
            points: HashMap::new(),
            normals: HashMap::new(),
        }
    }

    /// `uv` in the domain, and the key it is kept under.
    fn at(&self, uv: Point) -> (Point, [u64; 2]) {
        let uv = self.domain.clamp(uv);
        (uv, [uv[0].to_bits(), uv[1].to_bits()])
    }

    /// `uv` in the domain, as the surface's parameters.
    pub(crate) fn uv(&self, uv: Point) -> Vector2 {
        let (uv, _) = self.at(uv);
        Vector2::from_array([Scalar::from_f64(uv[0]), Scalar::from_f64(uv[1])])
    }

    /// The surface's point at `uv`.
    pub(crate) fn point(&mut self, uv: Point) -> GeopResult<Vector3> {
        let (uv, key) = self.at(uv);
        if let Some(p) = self.points.get(&key) {
            return Ok(*p);
        }
        let p = self
            .face
            .surface
            .evaluate(Scalar::from_f64(uv[0]), Scalar::from_f64(uv[1]))?;
        self.points.insert(key, p);
        Ok(p)
    }

    fn point_f64(&mut self, uv: Point) -> GeopResult<[f64; 3]> {
        let p = self.point(uv)?;
        Ok(p.to_f64_array())
    }

    /// The surface's normal at `uv`: none where it has none (a pole, where
    /// the two derivatives are parallel).
    pub(crate) fn normal(&mut self, uv: Point) -> Option<Vector3> {
        let (uv, key) = self.at(uv);
        let surface = &self.face.surface;
        *self.normals.entry(key).or_insert_with(|| {
            surface
                .normal(Scalar::from_f64(uv[0]), Scalar::from_f64(uv[1]))
                .ok()
        })
    }
}

/// Drop every polygon vertex that's collinear (within a tolerance relative
/// to the local segment length) with its immediate neighbors — see
/// `grid`'s use: `sample_loop_to_polygon` samples every loop at a fixed
/// density so genuinely curved boundaries come through faithfully, but
/// that leaves a dead-straight edge (most edges in this crate's test
/// shapes) represented by dozens of exactly collinear points, which would
/// otherwise bloat the boundary ring `clip` has to test every grid cell
/// against for no benefit.
fn simplify_polygon(poly: &[Vector2]) -> Vec<Vector2> {
    let n = poly.len();
    if n <= 3 {
        return poly.to_vec();
    }
    let f64_pt = |p: Vector2| [p[0].to_f64(), p[1].to_f64()];
    let keep: Vec<bool> = (0..n)
        .map(|i| {
            let prev = f64_pt(poly[(i + n - 1) % n]);
            let cur = f64_pt(poly[i]);
            let next = f64_pt(poly[(i + 1) % n]);
            let seg_len = ((next[0] - prev[0]).powi(2) + (next[1] - prev[1]).powi(2)).sqrt();
            seg_len < 1e-12 || dist_point_to_segment2(cur, prev, next) > seg_len * 1e-9
        })
        .collect();
    let result: Vec<Vector2> = (0..n).filter(|&i| keep[i]).map(|i| poly[i]).collect();
    if result.len() >= 3 {
        result
    } else {
        poly.to_vec()
    }
}

/// The worst deviation, over a `nu x nv` grid spanning `[min, max]`,
/// between the surface's actual point at a cell edge's midpoint and the
/// straight chord between its corners — reported per direction, since a
/// face can curve in one and be straight in the other (a cylinder wall,
/// an extruded profile), and then only that direction needs refining.
fn worst_grid_deviation(
    samples: &mut Samples,
    min: Point,
    max: Point,
    nu: usize,
    nv: usize,
) -> GeopResult<[f64; 2]> {
    // The point `i / nu` and `j / nv` of the way across, as
    // `triangulate_region` places its cells: the midpoints of a cell's edges
    // are then exactly the corners of the grid twice as fine.
    let at = |i: usize, nu: usize, j: usize, nv: usize| -> Point {
        [
            min[0] + (max[0] - min[0]) * i as f64 / nu as f64,
            min[1] + (max[1] - min[1]) * j as f64 / nv as f64,
        ]
    };
    let mut worst = [0.0f64; 2];
    for j in 0..nv {
        for i in 0..nu {
            let p00 = samples.point_f64(at(i, nu, j, nv))?;
            let p10 = samples.point_f64(at(i + 1, nu, j, nv))?;
            let p01 = samples.point_f64(at(i, nu, j + 1, nv))?;
            let mid_u = samples.point_f64(at(2 * i + 1, 2 * nu, j, nv))?;
            let mid_v = samples.point_f64(at(i, nu, 2 * j + 1, 2 * nv))?;
            worst[0] = worst[0].max(dist_point_to_segment(mid_u, p00, p10));
            worst[1] = worst[1].max(dist_point_to_segment(mid_v, p00, p01));
        }
    }
    Ok(worst)
}

/// Triangulate `face`'s trimmed `(u, v)` region: a uniform grid (its
/// resolution chosen from the face's own curvature — see the module doc
/// comment), each cell clipped exactly against the trim boundary. The
/// triangles' corners are in `(u, v)`, as `samples` takes them.
pub(crate) fn triangulate_face(
    model: &Model,
    face: &Face,
    n: usize,
    samples: &mut Samples,
) -> GeopResult<Vec<[Point; 3]>> {
    let BoundaryType::Loop(outer_anchor) = face.outer else {
        return Ok(Vec::new());
    };
    let outer = simplify_polygon(&sample_loop_to_polygon(model, outer_anchor, n.max(2))?);
    let holes = face
        .holes
        .iter()
        .filter_map(|b| match b {
            BoundaryType::Loop(anchor) => Some(*anchor),
            BoundaryType::Vertex(_) => None,
        })
        .map(|anchor| {
            Ok(simplify_polygon(&sample_loop_to_polygon(
                model,
                anchor,
                n.max(2),
            )?))
        })
        .collect::<GeopResult<Vec<_>>>()?;
    if outer.len() < 3 {
        return Ok(Vec::new());
    }

    let outer_f64: Vec<Point> = outer
        .iter()
        .map(|p| [p[0].to_f64(), p[1].to_f64()])
        .collect();
    let holes_f64: Vec<Vec<Point>> = holes
        .iter()
        .map(|h| h.iter().map(|p| [p[0].to_f64(), p[1].to_f64()]).collect())
        .collect();

    let Some((min, max)) = bounds(outer_f64.iter().copied()) else {
        return Ok(Vec::new());
    };
    if max[0] <= min[0] || max[1] <= min[1] {
        return Ok(Vec::new());
    }

    // Curvature tolerance derived from the face's own evaluated size, so
    // `n` means a quality rather than one absolute size that would over- or
    // under-tessellate depending on scale. It shrinks with `n²` because the
    // deviation of a grid cell from the surface does too (halving a cell
    // quarters its sagitta): that makes `n` mean "about this many cells
    // across a face that curves through a quarter turn", and it is what
    // decides how round a cylinder's silhouette looks.
    let evaluated = outer_f64
        .iter()
        .map(|&p| samples.point_f64(p))
        .collect::<GeopResult<Vec<_>>>()?;
    let (bbox_min, bbox_max) = bounds(evaluated).expect("the outer loop has points");
    let face_size = distance(bbox_min, bbox_max).max(1e-9);
    let quality = n.max(2) as f64;
    let max_sagitta = (face_size / (4.0 * quality * quality)).max(1e-9);

    // `u` and `v` are refined independently: a cylinder wall curves in one
    // direction only, and a square grid would spend as many triangles along
    // its straight direction as along its round one.
    let cells = refine([MIN_GRID_N; 2], MAX_DOUBLINGS, |cells| -> GeopResult<_> {
        let worst = worst_grid_deviation(samples, min, max, cells[0], cells[1])?;
        let mut finer = cells;
        for k in 0..2 {
            if worst[k] > max_sagitta && cells[k] < MAX_GRID_N {
                finer[k] = (cells[k] * 2).min(MAX_GRID_N);
            }
        }
        Ok(if finer == cells {
            Refine::Done(cells)
        } else {
            Refine::Finer(finer)
        })
    })?
    .either();
    let (nu, nv) = (cells[0], cells[1]);

    Ok(triangulate_region(&outer_f64, &holes_f64, min, max, nu, nv))
}

/// The region inside the simple polygon `outer` and outside every one of
/// `holes`, as triangles: the grid's own clipping on one cell, the
/// polygon's bounding box — no bridge joins a hole to the outline (see the
/// module doc). None for a polygon that encloses nothing.
pub fn triangulate_polygon(outer: &[Point], holes: &[Vec<Point>]) -> Vec<[Point; 3]> {
    let Some(&first) = outer.first() else {
        return Vec::new();
    };
    let (min, max) = outer.iter().fold((first, first), |(lo, hi), p| {
        (
            [lo[0].min(p[0]), lo[1].min(p[1])],
            [hi[0].max(p[0]), hi[1].max(p[1])],
        )
    });
    if outer.len() < 3 || max[0] <= min[0] || max[1] <= min[1] {
        return Vec::new();
    }
    triangulate_region(outer, holes, min, max, 1, 1)
}

/// The region inside `outer` and outside every one of `holes`, cut along a
/// `nu` x `nv` grid over `[min, max]` into triangles that each lie in one
/// cell.
fn triangulate_region(
    outer: &[Point],
    holes: &[Vec<Point>],
    min: Point,
    max: Point,
    nu: usize,
    nv: usize,
) -> Vec<[Point; 3]> {
    let cell_u = |i: usize| min[0] + (max[0] - min[0]) * i as f64 / nu as f64;
    let cell_v = |j: usize| min[1] + (max[1] - min[1]) * j as f64 / nv as f64;
    // The cells `[lo, hi]` spans along one axis, one either side to spare:
    // a point on a cell line may round into either cell, and clipping
    // exactly discards a cell it only touches.
    let span = |lo: f64, hi: f64, start: f64, end: f64, n: usize| {
        let at = |x: f64| ((x - start) / (end - start) * n as f64).floor() as isize;
        let first = (at(lo) - 1).clamp(0, n as isize - 1) as usize;
        let last = (at(hi) + 1).clamp(0, n as isize - 1) as usize;
        first..=last
    };

    // Every boundary is split once into convex parts, and each part is
    // clipped to the cells it overlaps. Sutherland–Hodgman clips a convex
    // polygon to a cell exactly, into one convex piece. A concave one it
    // returns as one polygon however often the boundary enters the cell,
    // joining its separate pieces by zero-width bridges along the cell's
    // edge, and ear-clipping that — no longer a simple polygon — may cover
    // the gap between them. So a concave boundary is split into triangles,
    // once, as the simple polygon it is, and a convex one stays whole: its
    // pieces then have no seams inside them, where rounding would leave
    // hairline slivers.
    let into_cells = |polygon: &[Point], cells: &mut [Vec<Vec<Point>>]| {
        let parts: Vec<Vec<Point>> = if is_convex(polygon) {
            vec![polygon.to_vec()]
        } else {
            ear_clip(polygon)
                .into_iter()
                .map(|t| vec![polygon[t[0]], polygon[t[1]], polygon[t[2]]])
                .collect()
        };
        for part in parts {
            let (lo, hi) = part.iter().fold(
                ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
                |(lo, hi), p| {
                    (
                        [lo[0].min(p[0]), lo[1].min(p[1])],
                        [hi[0].max(p[0]), hi[1].max(p[1])],
                    )
                },
            );
            for i in span(lo[0], hi[0], min[0], max[0], nu) {
                for j in span(lo[1], hi[1], min[1], max[1], nv) {
                    let piece =
                        clip_to_rect(&part, cell_u(i), cell_u(i + 1), cell_v(j), cell_v(j + 1));
                    if piece.len() >= 3 {
                        cells[i * nv + j].push(piece);
                    }
                }
            }
        }
    };
    let mut inside: Vec<Vec<Vec<Point>>> = vec![Vec::new(); nu * nv];
    into_cells(outer, &mut inside);
    let mut cut: Vec<Vec<Vec<Point>>> = vec![Vec::new(); nu * nv];
    for hole in holes {
        into_cells(hole, &mut cut);
    }

    // In each cell, every hole's convex pieces are subtracted from the outer
    // boundary's: what is left of a convex piece is convex pieces again,
    // which ear-clip exactly.
    let mut triangles = Vec::new();
    for (pieces, holes) in inside.into_iter().zip(cut) {
        let pieces = holes.iter().fold(pieces, |pieces, hole| {
            pieces
                .iter()
                .flat_map(|piece| subtract_convex(piece, hole))
                .collect()
        });
        for piece in &pieces {
            for tri in ear_clip(piece) {
                triangles.push([piece[tri[0]], piece[tri[1]], piece[tri[2]]]);
            }
        }
    }
    triangles
}

#[cfg(test)]
mod tests {
    /// The area the triangles of `triangulate_region` cover.
    fn covered(triangles: &[[Point; 3]]) -> f64 {
        triangles
            .iter()
            .map(|[a, b, c]| {
                ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs() / 2.0
            })
            .sum()
    }

    /// A trim that enters a cell twice leaves the cell two separate
    /// pieces, with what lies between them outside the trim: a U whose two
    /// arms cross the upper cell. Covering the gap between the arms is
    /// what drew a face across a hole it went around (see
    /// `cylinder_joined_over_a_hole` in `geop-cad-base`).
    #[test]
    fn a_trim_entering_a_cell_twice_leaves_the_gap_uncovered() {
        let u = [
            [0.0, 0.0],
            [3.0, 0.0],
            [3.0, 3.0],
            [2.0, 3.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 3.0],
            [0.0, 3.0],
        ];
        for (nu, nv) in [(1, 2), (1, 3), (2, 2), (3, 4)] {
            let triangles = triangulate_region(&u, &[], [0.0, 0.0], [3.0, 3.0], nu, nv);
            let area = covered(&triangles);
            assert!(
                (area - 7.0).abs() < 1e-9,
                "{nu} x {nv} cells cover {area}, not 7"
            );
        }
    }

    /// A concave hole of many corners in a single cell — a crescent, as a
    /// solid's face is left with where a torus was joined over it — is
    /// split into many triangles, each taken out of what is left of the
    /// cell in turn. That must stay cheap: a piece a triangle does not
    /// overlap is left whole, not cut along the triangle's sides.
    #[test]
    fn a_concave_hole_of_many_corners_stays_cheap() {
        let square = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        let n = 60;
        let arc = |r: f64, k: usize| {
            let t = std::f64::consts::PI * (0.15 + 1.7 * k as f64 / n as f64);
            [2.0 + r * t.cos(), 2.0 + r * t.sin()]
        };
        // Clockwise, as a hole: along the outer arc, back along the inner.
        let mut crescent: Vec<Point> = (0..=n).rev().map(|k| arc(1.5, k)).collect();
        crescent.extend((0..=n).map(|k| arc(1.0, k)));
        let started = std::time::Instant::now();
        let triangles =
            triangulate_region(&square, &[crescent.clone()], [0.0, 0.0], [4.0, 4.0], 1, 1);
        let hole = covered(
            &ear_clip(&crescent)
                .into_iter()
                .map(|t| [crescent[t[0]], crescent[t[1]], crescent[t[2]]])
                .collect::<Vec<_>>(),
        );
        assert!((covered(&triangles) - (16.0 - hole)).abs() < 1e-9);
        assert!(
            triangles.len() < 20 * crescent.len(),
            "{} triangles",
            triangles.len()
        );
        assert!(
            started.elapsed().as_secs() < 2,
            "took {:?}",
            started.elapsed()
        );
    }

    /// A round hole takes out itself, and leaves nothing inside it, on a
    /// grid of any fineness.
    #[test]
    fn a_round_hole_leaves_nothing_inside_it() {
        let square = [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]];
        // A 32-gon of radius 1 around (2, 2), clockwise.
        let hole: Vec<Point> = (0..32)
            .rev()
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / 32.0;
                [2.0 + a.cos(), 2.0 + a.sin()]
            })
            .collect();
        let hole_area = 16.0 * (std::f64::consts::TAU / 32.0).sin();
        for n in [1, 2, 3, 4, 8, 13] {
            let triangles = triangulate_region(
                &square,
                std::slice::from_ref(&hole),
                [0.0, 0.0],
                [4.0, 4.0],
                n,
                n,
            );
            let inside: Vec<_> = triangles
                .iter()
                .filter(|t| {
                    let c = [0, 1].map(|k| (t[0][k] + t[1][k] + t[2][k]) / 3.0);
                    (c[0] - 2.0).hypot(c[1] - 2.0) < 0.95 && covered(&[**t]) > 1e-12
                })
                .collect();
            assert!(
                inside.is_empty(),
                "{n} x {n} cells: {} triangles inside the hole, e.g. {:?}",
                inside.len(),
                inside.first()
            );
            let area = covered(&triangles);
            assert!(
                (area - (16.0 - hole_area)).abs() < 1e-9,
                "{n} x {n} cells cover {area}, not 16 - {hole_area}"
            );
        }
    }

    /// A round hole whose quarter points lie on the cell lines — as an
    /// extruded cap's is sampled, where the grid halves the face right
    /// through them: a rounding error off them in plain `f64`, and exactly
    /// on them in fixed point.
    #[test]
    fn a_hole_touching_cell_lines_leaves_nothing_inside_it() {
        let square = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let samplings: [Vec<Point>; 2] = [
            vec![
                [0.7499999999999996, 0.49999999999999983],
                [0.744503716533199, 0.44786620480482087],
                [0.7268208044051963, 0.39486997246752176],
                [0.6963909975788529, 0.3453048931931495],
                [0.6546951068068501, 0.3036090024211465],
                [0.6051300275324778, 0.2731791955948034],
                [0.5521337951951788, 0.25549628346680053],
                [0.49999999999999983, 0.24999999999999994],
                [0.44786620480482087, 0.2554962834668005],
                [0.39486997246752176, 0.2731791955948034],
                [0.3453048931931495, 0.3036090024211465],
                [0.3036090024211465, 0.34530489319314944],
                [0.2731791955948034, 0.39486997246752187],
                [0.25549628346680053, 0.44786620480482087],
                [0.24999999999999994, 0.4999999999999999],
                [0.2554962834668005, 0.5521337951951787],
                [0.2731791955948034, 0.6051300275324778],
                [0.3036090024211465, 0.6546951068068501],
                [0.34530489319314944, 0.6963909975788529],
                [0.39486997246752187, 0.7268208044051963],
                [0.44786620480482087, 0.7445037165331991],
                [0.4999999999999999, 0.7499999999999996],
                [0.5521337951951787, 0.744503716533199],
                [0.6051300275324778, 0.7268208044051963],
                [0.6546951068068501, 0.6963909975788529],
                [0.6963909975788529, 0.6546951068068501],
                [0.7268208044051963, 0.6051300275324778],
                [0.7445037165331991, 0.5521337951951788],
            ],
            vec![
                [0.75, 0.5],
                [0.744503716705367, 0.4478662048932165],
                [0.7268208044115454, 0.39486997248604894],
                [0.6963909976184368, 0.3453048930969089],
                [0.6546951066702604, 0.30360900214873254],
                [0.6051300275139511, 0.2731791955884546],
                [0.5521337951067835, 0.25549628329463303],
                [0.5, 0.25],
                [0.4478662048932165, 0.2554962835274637],
                [0.39486997248604894, 0.27317919535562396],
                [0.3453048930969089, 0.3036090023815632],
                [0.30360900214873254, 0.34530489332973957],
                [0.2731791955884546, 0.39486997248604894],
                [0.25549628329463303, 0.44786620466038585],
                [0.25, 0.5],
                [0.2554962835274637, 0.5521337951067835],
                [0.27317919535562396, 0.6051300275139511],
                [0.3036090023815632, 0.6546951066702604],
                [0.34530489332973957, 0.6963909973856062],
                [0.39486997248604894, 0.7268208044115454],
                [0.44786620466038585, 0.7445037164725363],
                [0.5, 0.75],
                [0.5521337951067835, 0.744503716705367],
                [0.6051300275139511, 0.7268208044115454],
                [0.6546951066702604, 0.6963909976184368],
                [0.6963909973856062, 0.6546951066702604],
                [0.7268208044115454, 0.6051300275139511],
                [0.7445037164725363, 0.5521337951067835],
            ],
        ];
        for hole in samplings {
            let hole_area: f64 = (0..hole.len())
                .map(|i| {
                    let (a, b) = (hole[i], hole[(i + 1) % hole.len()]);
                    a[0] * b[1] - b[0] * a[1]
                })
                .sum::<f64>()
                .abs()
                / 2.0;
            for n in [1, 2, 3, 4, 8] {
                let triangles = triangulate_region(
                    &square,
                    std::slice::from_ref(&hole),
                    [0.0, 0.0],
                    [1.0, 1.0],
                    n,
                    n,
                );
                let area = covered(&triangles);
                assert!(
                    (area - (1.0 - hole_area)).abs() < 1e-9,
                    "{n} x {n} cells cover {area}, not 1 - {hole_area}"
                );
            }
        }
    }

    /// The same for a hole: one that enters a cell twice takes out only
    /// itself, not what lies between its arms.
    #[test]
    fn a_hole_entering_a_cell_twice_takes_out_only_itself() {
        let square = [[-1.0, -1.0], [4.0, -1.0], [4.0, 4.0], [-1.0, 4.0]];
        // The U, clockwise, as a hole's boundary runs.
        let hole: Vec<Point> = [
            [0.0, 0.0],
            [3.0, 0.0],
            [3.0, 3.0],
            [2.0, 3.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 3.0],
            [0.0, 3.0],
        ]
        .into_iter()
        .rev()
        .collect();
        for (nu, nv) in [(1, 1), (1, 2), (2, 3), (5, 5)] {
            let triangles = triangulate_region(
                &square,
                std::slice::from_ref(&hole),
                [-1.0, -1.0],
                [4.0, 4.0],
                nu,
                nv,
            );
            let area = covered(&triangles);
            assert!(
                (area - 18.0).abs() < 1e-9,
                "{nu} x {nv} cells cover {area}, not 25 - 7"
            );
        }
    }
    use super::*;
    use geop_core_topology::test_fixtures::test_cube_model;

    fn triangle_area(a: Vector2, b: Vector2, c: Vector2) -> f64 {
        let (ax, ay) = (a[0].to_f64(), a[1].to_f64());
        let (bx, by) = (b[0].to_f64(), b[1].to_f64());
        let (cx, cy) = (c[0].to_f64(), c[1].to_f64());
        ((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)).abs() / 2.0
    }

    #[test]
    fn flat_face_stays_coarse() {
        let topology = &test_cube_model();
        let face = topology.faces.values().next().unwrap();
        let tris = crate::face_triangles_uv(topology, face, 32).unwrap();
        // Zero curvature never triggers a resolution doubling, so the grid
        // stays at the minimum whatever quality was asked for: a flat face
        // is approximated exactly by any grid, and its trim is clipped
        // against the cells exactly either way.
        assert_eq!(
            tris.len(),
            2 * MIN_GRID_N * MIN_GRID_N,
            "flat face should stay at the coarsest grid, got {}",
            tris.len()
        );
    }

    #[test]
    fn triangulation_covers_exact_area() {
        let topology = &test_cube_model();
        let face = topology.faces.values().next().unwrap();
        let tris = crate::face_triangles_uv(topology, face, 8).unwrap();
        let total: f64 = tris.iter().map(|(a, b, c)| triangle_area(*a, *b, *c)).sum();
        assert!(
            (total - 1.0).abs() < 1e-6,
            "total uv area={total}, expected 1.0"
        );
    }
}
