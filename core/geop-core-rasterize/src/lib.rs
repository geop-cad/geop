//! Turn a [`Model`] into sampled geometry — [`rasterize`]: a point per
//! vertex, a polyline per edge, triangles per face, each tagged with the
//! entity it came from — to draw it, pick in it, write it as STL ([`stl`]),
//! or render it for debugging ([`debug`]).
//!
//! Every face — trimmed or not — has its outer boundary (and any holes)
//! sampled from their pcurves into `(u, v)` polygons, then triangulated by
//! [`grid::triangulate_face`]: a curvature-sized `(u, v)` grid with every
//! cell clipped exactly against the trim (see that module's doc comment),
//! so a flat patch renders as a couple of triangles and a curved one gets
//! whatever resolution its own curvature actually needs — mapped through
//! `surface.evaluate`, the rendered mesh always both respects the face's
//! real trim and follows its true curvature.

mod clip;
pub mod debug;
mod grid;
pub mod stl;

pub use grid::triangulate_polygon;

/// A triangle in a plane, by its corners.
pub type Triangle2 = (Vector2, Vector2, Vector2);

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain::distance,
    primitives::TriangleFace,
    scalars::Scalar,
    solvers::refine::{Refine, Refined, refine},
    vector::{Vector2, Vector3},
};
use geop_core_topology::{EdgeId, Face, FaceId, Model, VertexId};
use grid::dist_point_to_segment;

/// `face`'s trimmed region triangulated in `(u, v)` space, by a grid sized
/// by the face's curvature and clipped to its trim (see
/// [`grid::triangulate_face`]), each triangle by its corners.
pub fn face_triangles_uv(model: &Model, face: &Face, n: usize) -> GeopResult<Vec<Triangle2>> {
    let mut samples = grid::Samples::new(face);
    Ok(grid::triangulate_face(model, face, n, &mut samples)?
        .into_iter()
        .map(|[a, b, c]| (samples.uv(a), samples.uv(b), samples.uv(c)))
        .collect())
}

/// A [`Model`] rasterized once, with every sampled point/polyline/triangle
/// kept alongside the id of the entity it came from.
///
/// This is the single place the crate turns topology into sampled geometry
/// — drawing and ray picking (`geop_core_editor::view::PartView`), STL, and debug
/// rendering ([`debug`]) all build on top of it, rather than each walking
/// `Model` and sampling curves/surfaces on their own. That matters beyond
/// not repeating code: it guarantees a pick can never disagree with what
/// the viewer actually drew, because both read the same triangles.
pub struct RasterizedModel {
    pub vertices: BTreeMap<VertexId, Vector3>,
    /// Each edge's curve sampled into an `n`-point polyline.
    pub edges: BTreeMap<EdgeId, Vec<Vector3>>,
    /// Each face's trimmed region, triangulated (see [`face_triangles_uv`])
    /// and mapped through `surface.evaluate` — one face generally maps to
    /// several triangles.
    pub faces: BTreeMap<FaceId, Vec<TriangleFace>>,
}

/// A curve is first sampled this finely, and refined from there.
const MIN_EDGE_SEGMENTS: usize = 1;
/// How many times an edge's sampling may double.
const MAX_EDGE_DOUBLINGS: usize = 10;

/// `curve` as a polyline that stays within `quality`'s curvature tolerance
/// of it: a straight edge is two points, a tight arc gets as many as it
/// needs. Same rule as [`grid`]'s cells — the tolerance shrinks with
/// `quality²` because a segment's deviation from the curve does too — so a
/// model's edges and faces come out equally smooth.
fn sample_curve(
    curve: &geop_core_geometry::nurb_curve::NurbCurve3D,
    quality: usize,
) -> GeopResult<Vec<Vector3>> {
    let (t0, t1) = curve.domain();
    let at = |frac: (usize, usize)| -> GeopResult<Vector3> {
        let f = Scalar::from_ratio(frac.0 as i64, frac.1 as i64)?;
        curve.evaluate(t0.add(t1.sub(t0).mul(f)))
    };
    let points_at = |segments: usize| -> GeopResult<Vec<Vector3>> {
        (0..=segments).map(|i| at((i, segments))).collect()
    };

    let first = points_at(MIN_EDGE_SEGMENTS)?;
    let size = first
        .iter()
        .flat_map(|p| {
            first
                .iter()
                .map(move |q| distance(p.to_f64_array(), q.to_f64_array()))
        })
        .fold(0.0f64, f64::max)
        .max(1e-9);
    let tolerance = size / (4.0 * (quality * quality) as f64);

    // Doubled while the true point halfway along some segment strays from
    // its chord by more than the tolerance; at the cap, as fine as it got.
    let refined = refine(
        (MIN_EDGE_SEGMENTS, first),
        MAX_EDGE_DOUBLINGS,
        |(segments, points)| -> GeopResult<_> {
            let mut worst = 0.0f64;
            for i in 0..segments {
                let mid = at((2 * i + 1, 2 * segments))?;
                worst = worst.max(dist_point_to_segment(
                    mid.to_f64_array(),
                    points[i].to_f64_array(),
                    points[i + 1].to_f64_array(),
                ));
            }
            if worst <= tolerance {
                return Ok(Refine::Done(points));
            }
            Ok(Refine::Finer((2 * segments, points_at(2 * segments)?)))
        },
    )?;
    Ok(match refined {
        Refined::Done(points) | Refined::Exhausted((_, points)) => points,
    })
}

/// Sample every vertex/edge/face of `model` into a [`RasterizedModel`].
/// `n` is a quality: how finely a curve or surface that actually curves is
/// approximated (see [`sample_curve`] and [`grid::triangulate_face`]), not
/// a fixed sample count — a straight edge or flat face stays cheap.
pub fn rasterize(model: &Model, n: usize) -> GeopResult<RasterizedModel> {
    if n < 2 {
        return Err(GeopError::new("rasterize: n must be >= 2"));
    }

    let vertices = model
        .vertices
        .iter()
        .map(|(&id, vertex)| (id, vertex.point))
        .collect();

    let mut edges = BTreeMap::new();
    for (&id, edge) in model.edges.iter() {
        edges.insert(id, sample_curve(&edge.curve, n)?);
    }

    let mut faces = BTreeMap::new();
    for (&id, face) in model.faces.iter() {
        let mut tris = Vec::new();
        let mut samples = grid::Samples::new(face);
        for [a, b, c] in grid::triangulate_face(model, face, n, &mut samples)? {
            let Ok(t) =
                TriangleFace::try_new(samples.point(a)?, samples.point(b)?, samples.point(c)?)
            else {
                continue;
            };
            // The surface's own normal at each corner, for smooth shading.
            // A corner where it does not exist leaves the whole triangle
            // flat.
            tris.push(
                match (samples.normal(a), samples.normal(b), samples.normal(c)) {
                    (Some(na), Some(nb), Some(nc)) => t.with_vertex_normals([na, nb, nc]),
                    _ => t,
                },
            );
        }
        faces.insert(id, tris);
    }

    Ok(RasterizedModel {
        vertices,
        edges,
        faces,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::Color10;
    use geop_core_topology::test_fixtures::test_cube_model;

    /// Rasterize `model`, sanity-check it's a non-empty mesh, and save it to
    /// `outputs/<name>.html` for visual inspection.
    fn rasterize_and_save(model: &Model, name: &str) {
        let scene = rasterize(model, 8).unwrap().scene(|_| Color10::Blue);
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles.is_empty());
        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file(&format!("outputs/{name}.html")).unwrap();
    }

    #[test]
    fn rasterize_cube() {
        rasterize_and_save(&test_cube_model(), "cube");
    }
}
