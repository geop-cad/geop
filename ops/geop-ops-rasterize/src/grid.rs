//! Face rasterization: an axis-aligned `(u, v)` grid, sized to the face's
//! own curvature, with every cell clipped exactly against the trim
//! boundary.
//!
//! Deliberately simple, after a constrained-Delaunay approach ([`super::delaunay`],
//! now removed) turned out both too slow (the same triangle set could be
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
//! 3. Clip the outer boundary to each grid cell
//!    ([`super::clip::clip_to_rect`]: `[]` if it misses the cell entirely,
//!    the cell unchanged if it's fully inside, an exact clipped polygon
//!    otherwise), then subtract each hole
//!    ([`super::clip::subtract_convex`]) the same way, and ear-clip
//!    whatever's left.
//!
//!    Two earlier versions of this step turned out wrong. The first
//!    globally merged outer+holes into one ring (via a zero-width "slit"
//!    bridge — see `polygon_triangulate::merge_outer_and_holes`) and
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
//! Plain `f64` throughout — see [`super::clip`]'s doc comment for why
//! that's the right call for a rendering-only algorithm even in an
//! otherwise interval-exact kernel.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};
use geop_core_topology::{
    Face, Model, boundary::BoundaryType, loop_sampling::sample_loop_to_polygon,
};

use crate::clip::{Point, clip_to_rect, ear_clip, is_convex, subtract_convex};

/// Where the grid starts, before curvature refines it: a flat face needs
/// nothing finer, whatever `n` the caller asked for, since its cells
/// approximate it exactly and the trim is clipped against them exactly.
const MIN_GRID_N: usize = 2;
/// Grid resolution never grows past this (in either direction), how ever
/// far short of tolerance a face's curvature would otherwise ask for — a
/// point budget, same role as `adaptive`'s old `MAX_STEINER_POINTS`.
const MAX_GRID_N: usize = 128;
/// How many times resolution may double while searching for one that
/// meets tolerance.
const MAX_DOUBLINGS: u32 = 7;

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot3(sub3(a, b), sub3(a, b)).sqrt()
}

/// Perpendicular distance from `p` to segment `a..b` — see `grid`'s
/// module doc comment on why curvature is measured this way (immune to a
/// merely non-uniformly parametrized but still flat surface).
fn dist_point_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub3(b, a);
    let len2 = dot3(ab, ab);
    let t = if len2 > 1e-18 {
        (dot3(sub3(p, a), ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    dist3(p, closest)
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
/// `evaluate_f64`'s call sites: plain-`f64` grid/midpoint arithmetic on a
/// value already at a domain edge can legitimately land a ULP or two
/// outside it, which a real `Scalar` interval never does (its width
/// already encloses the true value) but a degenerate zero-width interval
/// reconstructed via `S::from_f64` has no such margin, so the kernel's
/// (correctly strict) domain check can reject it.
#[derive(Clone, Copy)]
struct Domain {
    u: (f64, f64),
    v: (f64, f64),
}

impl Domain {
    fn of<S: Scalar>(face: &Face<S>) -> Self {
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

fn evaluate_f64<S: Scalar>(face: &Face<S>, domain: Domain, uv: Point) -> GeopResult<[f64; 3]> {
    let uv = domain.clamp(uv);
    let p = face
        .surface
        .evaluate(S::from_f64(uv[0]), S::from_f64(uv[1]))?;
    Ok([p[0].to_f64(), p[1].to_f64(), p[2].to_f64()])
}

/// Drop every polygon vertex that's collinear (within a tolerance relative
/// to the local segment length) with its immediate neighbors — see
/// `grid`'s use: `sample_loop_to_polygon` samples every loop at a fixed
/// density so genuinely curved boundaries come through faithfully, but
/// that leaves a dead-straight edge (most edges in this crate's test
/// shapes) represented by dozens of exactly collinear points, which would
/// otherwise bloat the boundary ring `clip` has to test every grid cell
/// against for no benefit.
fn simplify_polygon<S: Scalar>(poly: &[Vector2<S>]) -> Vec<Vector2<S>> {
    let n = poly.len();
    if n <= 3 {
        return poly.to_vec();
    }
    let f64_pt = |p: Vector2<S>| [p[0].to_f64(), p[1].to_f64()];
    let keep: Vec<bool> = (0..n)
        .map(|i| {
            let prev = f64_pt(poly[(i + n - 1) % n]);
            let cur = f64_pt(poly[i]);
            let next = f64_pt(poly[(i + 1) % n]);
            let seg_len = ((next[0] - prev[0]).powi(2) + (next[1] - prev[1]).powi(2)).sqrt();
            seg_len < 1e-12 || dist_point_to_segment2(cur, prev, next) > seg_len * 1e-9
        })
        .collect();
    let result: Vec<Vector2<S>> = (0..n).filter(|&i| keep[i]).map(|i| poly[i]).collect();
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
fn worst_grid_deviation<S: Scalar>(
    face: &Face<S>,
    domain: Domain,
    min: Point,
    max: Point,
    nu: usize,
    nv: usize,
) -> GeopResult<[f64; 2]> {
    let corner = |i: usize, j: usize| -> Point {
        [
            min[0] + (max[0] - min[0]) * i as f64 / nu as f64,
            min[1] + (max[1] - min[1]) * j as f64 / nv as f64,
        ]
    };
    // Cache one row of evaluated corners at a time to avoid re-evaluating
    // shared corners (each interior corner is shared by up to 4 cells).
    let mut prev_row: Vec<[f64; 3]> = (0..=nu)
        .map(|i| evaluate_f64(face, domain, corner(i, 0)))
        .collect::<GeopResult<_>>()?;
    let mut worst = [0.0f64; 2];
    for j in 0..nv {
        let cur_row: Vec<[f64; 3]> = (0..=nu)
            .map(|i| evaluate_f64(face, domain, corner(i, j + 1)))
            .collect::<GeopResult<_>>()?;
        for i in 0..nu {
            let (p00, p10, p01) = (prev_row[i], prev_row[i + 1], cur_row[i]);
            let mid_u = evaluate_f64(
                face,
                domain,
                [
                    (corner(i, j)[0] + corner(i + 1, j)[0]) / 2.0,
                    corner(i, j)[1],
                ],
            )?;
            let mid_v = evaluate_f64(
                face,
                domain,
                [
                    corner(i, j)[0],
                    (corner(i, j)[1] + corner(i, j + 1)[1]) / 2.0,
                ],
            )?;
            worst[0] = worst[0].max(dist_point_to_segment(mid_u, p00, p10));
            worst[1] = worst[1].max(dist_point_to_segment(mid_v, p00, p01));
        }
        prev_row = cur_row;
    }
    Ok(worst)
}

/// Triangulate `face`'s trimmed `(u, v)` region: a uniform grid (its
/// resolution chosen from the face's own curvature — see the module doc
/// comment), each cell clipped exactly against the trim boundary.
pub(crate) fn triangulate_face<S: Scalar>(
    model: &Model<S>,
    face: &Face<S>,
    n: usize,
) -> GeopResult<Vec<(Vector2<S>, Vector2<S>, Vector2<S>)>> {
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

    let mut min = outer_f64[0];
    let mut max = outer_f64[0];
    for &p in &outer_f64 {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    }
    if max[0] <= min[0] || max[1] <= min[1] {
        return Ok(Vec::new());
    }

    let domain = Domain::of(face);

    // Curvature tolerance derived from the face's own evaluated size, so
    // `n` means a quality rather than one absolute size that would over- or
    // under-tessellate depending on scale. It shrinks with `n²` because the
    // deviation of a grid cell from the surface does too (halving a cell
    // quarters its sagitta): that makes `n` mean "about this many cells
    // across a face that curves through a quarter turn", and it is what
    // decides how round a cylinder's silhouette looks.
    let mut bbox_min = [f64::INFINITY; 3];
    let mut bbox_max = [f64::NEG_INFINITY; 3];
    for &p in &outer_f64 {
        let e = evaluate_f64(face, domain, p)?;
        for i in 0..3 {
            bbox_min[i] = bbox_min[i].min(e[i]);
            bbox_max[i] = bbox_max[i].max(e[i]);
        }
    }
    let face_size = dist3(bbox_min, bbox_max).max(1e-9);
    let quality = n.max(2) as f64;
    let max_sagitta = (face_size / (4.0 * quality * quality)).max(1e-9);

    // `u` and `v` are refined independently: a cylinder wall curves in one
    // direction only, and a square grid would spend as many triangles along
    // its straight direction as along its round one.
    let mut cells = [MIN_GRID_N; 2];
    for _ in 0..MAX_DOUBLINGS {
        let worst = worst_grid_deviation(face, domain, min, max, cells[0], cells[1])?;
        let mut refined = false;
        for k in 0..2 {
            if worst[k] > max_sagitta && cells[k] < MAX_GRID_N {
                cells[k] = (cells[k] * 2).min(MAX_GRID_N);
                refined = true;
            }
        }
        if !refined {
            break;
        }
    }
    let (nu, nv) = (cells[0], cells[1]);

    let mut triangles = Vec::new();
    for i in 0..nu {
        let u0 = min[0] + (max[0] - min[0]) * i as f64 / nu as f64;
        let u1 = min[0] + (max[0] - min[0]) * (i + 1) as f64 / nu as f64;
        for j in 0..nv {
            let v0 = min[1] + (max[1] - min[1]) * j as f64 / nv as f64;
            let v1 = min[1] + (max[1] - min[1]) * (j + 1) as f64 / nv as f64;

            // Clip the outer boundary to this cell's rectangle — not the
            // other way around — so an arbitrarily concave trim clips
            // correctly against the always-convex cell (see `clip`'s
            // module doc comment) — then subtract each hole the same way.
            let outer_piece = clip_to_rect(&outer_f64, u0, u1, v0, v1);
            if outer_piece.is_empty() {
                continue;
            }
            let mut pieces = vec![outer_piece];
            for hole in &holes_f64 {
                let hole_piece = clip_to_rect(hole, u0, u1, v0, v1);
                if hole_piece.len() < 3 {
                    continue;
                }
                pieces = pieces
                    .iter()
                    .flat_map(|piece| {
                        if is_convex(&hole_piece) {
                            subtract_convex(piece, &hole_piece)
                        } else {
                            // A hole boundary concave within this one cell
                            // is rare (it needs a grid coarser than the
                            // hole's own local curvature), but it still has
                            // to come out exactly: cut the fragment into
                            // triangles and subtract those, since a triangle
                            // is convex and their union is the fragment.
                            // The alternative — classifying the whole piece
                            // by its centroid — keeps or drops a whole cell
                            // at a time, which is what a flap of surface
                            // hanging across a hole looks like.
                            ear_clip(&hole_piece)
                                .iter()
                                .map(|t| [hole_piece[t[0]], hole_piece[t[1]], hole_piece[t[2]]])
                                .fold(vec![piece.clone()], |acc, triangle| {
                                    acc.iter()
                                        .flat_map(|p| subtract_convex(p, &triangle))
                                        .collect()
                                })
                        }
                    })
                    .collect();
            }

            for piece in &pieces {
                for tri in ear_clip(piece) {
                    let to_uv = |p: Point| {
                        let p = domain.clamp(p);
                        Vector2::from_array([S::from_f64(p[0]), S::from_f64(p[1])])
                    };
                    triangles.push((
                        to_uv(piece[tri[0]]),
                        to_uv(piece[tri[1]]),
                        to_uv(piece[tri[2]]),
                    ));
                }
            }
        }
    }

    Ok(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{for_all_scalars, vector::Vector3};
    use geop_ops::Part;
    use geop_ops_extrude_revolve::{cube_solid, sphere::sphere_solid};

    fn triangle_area<S: Scalar>(a: Vector2<S>, b: Vector2<S>, c: Vector2<S>) -> f64 {
        let (ax, ay) = (a[0].to_f64(), a[1].to_f64());
        let (bx, by) = (b[0].to_f64(), b[1].to_f64());
        let (cx, cy) = (c[0].to_f64(), c[1].to_f64());
        ((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)).abs() / 2.0
    }

    fn check_flat_face_stays_coarse<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t1",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        let face = model.faces.values().next().unwrap();
        let tris = triangulate_face(&model, face, 32).unwrap();
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
    fn flat_face_stays_coarse() {
        for_all_scalars!(check_flat_face_stays_coarse);
    }

    fn check_curved_face_is_refined<S: Scalar>() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t3", Vector3::zero(), S::ONE).unwrap();
        let model = part.topology();
        let face = model.faces.values().next().unwrap();
        let tris = triangulate_face(&model, face, 8).unwrap();
        assert!(
            tris.len() > 8,
            "a sphere quadrant should be refined well past a trivial fan, got {}",
            tris.len()
        );
    }

    /// The point of the refinement: every triangle of a curved face stays
    /// within the curvature tolerance of the true surface, so the mesh
    /// follows the shape instead of cutting corners off it. Checked on a
    /// unit sphere, where the distance to the surface is exact.
    fn check_curved_face_follows_the_surface<S: Scalar>() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t4", Vector3::zero(), S::ONE).unwrap();
        let model = part.topology();
        let face = model.faces.values().next().unwrap();
        let tris = triangulate_face(&model, face, 24).unwrap();
        let domain = Domain::of(face);
        let mut worst: f64 = 0.0;
        for (a, b, c) in &tris {
            let mid = [
                (a[0].to_f64() + b[0].to_f64() + c[0].to_f64()) / 3.0,
                (a[1].to_f64() + b[1].to_f64() + c[1].to_f64()) / 3.0,
            ];
            // The triangle's own centroid in 3-D, against the sphere it
            // should be lying on.
            let corners: Vec<[f64; 3]> = [a, b, c]
                .iter()
                .map(|uv| evaluate_f64(face, domain, [uv[0].to_f64(), uv[1].to_f64()]).unwrap())
                .collect();
            let centroid = [0, 1, 2].map(|k| corners.iter().map(|p| p[k]).sum::<f64>() / 3.0);
            let _ = mid;
            worst = worst.max((1.0 - dist3(centroid, [0.0; 3])).abs());
        }
        // `n = 24` asks for a sagitta under `face_size / (4 n²)`, which for
        // this patch (size ≈ 1.7) is about 7e-4.
        assert!(worst < 1e-3, "worst deviation from the sphere: {worst}");
    }
    #[test]
    fn curved_face_follows_the_surface() {
        for_all_scalars!(check_curved_face_follows_the_surface);
    }
    #[test]
    fn curved_face_is_refined() {
        for_all_scalars!(check_curved_face_is_refined);
    }

    fn check_triangulation_covers_exact_area<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t2",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        let face = model.faces.values().next().unwrap();
        let tris = triangulate_face(&model, face, 8).unwrap();
        let total: f64 = tris.iter().map(|(a, b, c)| triangle_area(*a, *b, *c)).sum();
        assert!(
            (total - 1.0).abs() < 1e-6,
            "total uv area={total}, expected 1.0"
        );
    }
    /// A face with a round hole must not get a single triangle inside that
    /// hole: the hole's area is missing from the triangulation, and no
    /// triangle's centroid falls in it.
    fn check_hole_is_not_triangulated<S: Scalar>() {
        use geop_core_math::primitives::CoordinateSystem;
        use geop_ops_extrude_revolve::common::{arc2, polygon, sqrt2_over_2};

        let mut part = Part::<S>::new();
        let v2 = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
        let cs = CoordinateSystem::try_new(
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(-1.0)]),
        )
        .unwrap();
        let outer = polygon(&[v2(0.0, 0.0), v2(4.0, 0.0), v2(4.0, 4.0), v2(0.0, 4.0)]).unwrap();
        // A circle of radius 1 around (2, 2), clockwise (a hole).
        let q = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let ccw: Vec<_> = (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                arc2(
                    v2(2.0 + a.0, 2.0 + a.1),
                    v2(2.0 + a.0 + b.0, 2.0 + a.1 + b.1),
                    v2(2.0 + b.0, 2.0 + b.1),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect();
        let hole: Vec<_> = ccw.iter().rev().map(|c| c.reverse()).collect();
        let namer = geop_ops::Namer::new("extrude", "e").unwrap();
        geop_ops_extrude_revolve::extrude::extrude(
            &mut part,
            &geop_ops_extrude_revolve::extrude::ExtrudeNames::single(&namer),
            &cs,
            &geop_ops_extrude_revolve::common::Profile::closed(outer),
            &[geop_ops_extrude_revolve::common::Profile::closed(hole).with_prefix("h")],
        )
        .unwrap();
        let model = part.topology();

        // The two cap faces are the ones with a hole.
        let capped: Vec<_> = model
            .faces
            .values()
            .filter(|f| !f.holes.is_empty())
            .collect();
        assert_eq!(capped.len(), 2);
        for face in capped {
            let tris = triangulate_face(&model, face, 8).unwrap();
            let area: f64 = tris.iter().map(|(a, b, c)| triangle_area(*a, *b, *c)).sum();
            // The cap's uv square spans the 4x4 footprint, so the hole is
            // π / 16 of it.
            let expected = 1.0 - std::f64::consts::PI / 16.0;
            assert!(
                (area - expected).abs() < 0.02,
                "uv area {area}, expected about {expected}"
            );
            for (a, b, c) in &tris {
                if triangle_area(*a, *b, *c) < 1e-12 {
                    continue;
                }
                let centroid = [
                    (a[0].to_f64() + b[0].to_f64() + c[0].to_f64()) / 3.0,
                    (a[1].to_f64() + b[1].to_f64() + c[1].to_f64()) / 3.0,
                ];
                // uv (0.5, 0.5) is the hole's center; radius 1 of 4 units.
                let r = ((centroid[0] - 0.5).powi(2) + (centroid[1] - 0.5).powi(2)).sqrt();
                assert!(r > 0.24, "triangle inside the hole at uv {centroid:?}");
            }
        }
    }
    #[test]
    fn hole_is_not_triangulated() {
        for_all_scalars!(check_hole_is_not_triangulated);
    }

    #[test]
    fn triangulation_covers_exact_area() {
        for_all_scalars!(check_triangulation_covers_exact_area);
    }

    /// A genuinely non-rectangular, holed footprint (unlike a cube/sphere
    /// face) must still triangulate cleanly: every triangle finite, and
    /// some non-trivial mesh actually produced — a crash or an empty
    /// result here is exactly the failure mode clipping is meant to rule
    /// out for trims a fixed grid-without-clipping can't represent.
    fn check_holed_footprint_triangulates_cleanly<S: Scalar>() {
        let mut part = Part::<S>::new();
        geop_ops_extrude_revolve::figure8_profile::figure8_profile(&mut part, "f").unwrap();
        let model = part.topology();
        let mut total_triangles = 0;
        for face in model.faces.values() {
            let tris = triangulate_face(&model, face, 8).unwrap();
            for (a, b, c) in &tris {
                for v in [a, b, c] {
                    assert!(
                        v[0].to_f64().is_finite() && v[1].to_f64().is_finite(),
                        "non-finite triangle vertex"
                    );
                }
            }
            total_triangles += tris.len();
        }
        assert!(
            total_triangles > 0,
            "figure8_profile must produce some triangles"
        );
    }
    #[test]
    fn holed_footprint_triangulates_cleanly() {
        for_all_scalars!(check_holed_footprint_triangulates_cleanly);
    }
}
