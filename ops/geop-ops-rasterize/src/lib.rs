//! Turn a [`Model`] into a renderable [`PrimitiveScene`]: one triangle mesh
//! (faces), line list (edges), and point list (vertices).
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
mod grid;
pub mod polygon_triangulate;
pub mod stl;
mod topology_debug;

use std::collections::HashMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Color10, PrimitiveScene, TriangleFace},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{EdgeId, Face, FaceId, Model, VertexId};

pub use topology_debug::rasterize_topology;

/// Triangulate `face`'s trimmed region in `(u, v)` space — see
/// [`grid::triangulate_face`] for the actual (curvature-sized grid, clipped
/// to the trim) algorithm; this just forwards to it. Kept as its own
/// name/doc entry point since it's `pub` API several other crates
/// (`geop-cad-base::pick`, this module's own `rasterize_model`) call by
/// name.
pub fn face_triangles_uv<S: Scalar>(
    model: &Model<S>,
    face: &Face<S>,
    n: usize,
) -> GeopResult<Vec<(Vector2<S>, Vector2<S>, Vector2<S>)>> {
    grid::triangulate_face(model, face, n)
}

/// A [`Model`] rasterized once, with every sampled point/polyline/triangle
/// kept alongside the id of the entity it came from.
///
/// This is the single place the crate turns topology into sampled geometry
/// — [`PrimitiveScene`] rendering (`rasterize_model`) and ray picking
/// (`geop_ops::ui::PartView`) both build on top of it, rather than each walking
/// `Model` and sampling curves/surfaces on their own. That matters beyond
/// not repeating code: it guarantees a pick can never disagree with what
/// the viewer actually drew, because both read the same triangles.
pub struct RasterizedModel<S: Scalar> {
    pub vertices: HashMap<VertexId, Vector3<S>>,
    /// Each edge's curve sampled into an `n`-point polyline.
    pub edges: HashMap<EdgeId, Vec<Vector3<S>>>,
    /// Each face's trimmed region, triangulated (see [`face_triangles_uv`])
    /// and mapped through `surface.evaluate` — one face generally maps to
    /// several triangles.
    pub faces: HashMap<FaceId, Vec<TriangleFace<S>>>,
}

/// A curve is first sampled this finely, and refined from there.
const MIN_EDGE_SEGMENTS: usize = 1;
/// How many times an edge's sampling may double.
const MAX_EDGE_DOUBLINGS: u32 = 10;

/// `curve` as a polyline that stays within `quality`'s curvature tolerance
/// of it: a straight edge is two points, a tight arc gets as many as it
/// needs. Same rule as [`grid`]'s cells — the tolerance shrinks with
/// `quality²` because a segment's deviation from the curve does too — so a
/// model's edges and faces come out equally smooth.
fn sample_curve<S: Scalar>(
    curve: &geop_core_geometry::nurb_curve::NurbCurve3D<S>,
    quality: usize,
) -> GeopResult<Vec<Vector3<S>>> {
    let (t0, t1) = curve.domain();
    let at = |frac: (usize, usize)| -> GeopResult<Vector3<S>> {
        let f = S::from_ratio(frac.0 as i64, frac.1 as i64)?;
        curve.evaluate(t0.add(t1.sub(t0).mul(f)))
    };
    let points_at = |segments: usize| -> GeopResult<Vec<Vector3<S>>> {
        (0..=segments).map(|i| at((i, segments))).collect()
    };

    let mut segments = MIN_EDGE_SEGMENTS;
    let mut points = points_at(segments)?;
    let f64_pt = |p: &Vector3<S>| [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()];
    let size = points
        .iter()
        .flat_map(|p| points.iter().map(move |q| dist3(f64_pt(p), f64_pt(q))))
        .fold(0.0f64, f64::max)
        .max(1e-9);
    let tolerance = size / (4.0 * (quality * quality) as f64);

    for _ in 0..MAX_EDGE_DOUBLINGS {
        // The true point halfway along each segment, against the chord.
        let mut worst = 0.0f64;
        for i in 0..segments {
            let mid = at((2 * i + 1, 2 * segments))?;
            worst = worst.max(dist_point_to_segment(
                f64_pt(&mid),
                f64_pt(&points[i]),
                f64_pt(&points[i + 1]),
            ));
        }
        if worst <= tolerance {
            break;
        }
        segments *= 2;
        points = points_at(segments)?;
    }
    Ok(points)
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Perpendicular distance from `p` to the segment `a..b`.
fn dist_point_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if len2 > 1e-18 {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / len2)
            .clamp(0.0, 1.0)
    } else {
        0.0
    };
    dist3(p, [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t])
}

/// Sample every vertex/edge/face of `model` into a [`RasterizedModel`].
/// `n` is a quality: how finely a curve or surface that actually curves is
/// approximated (see [`sample_curve`] and [`grid::triangulate_face`]), not
/// a fixed sample count — a straight edge or flat face stays cheap.
pub fn rasterize_model_tagged<S: Scalar>(
    model: &Model<S>,
    n: usize,
) -> GeopResult<RasterizedModel<S>> {
    if n < 2 {
        return Err(GeopError::new("rasterize_model_tagged: n must be >= 2"));
    }

    let vertices = model
        .vertices
        .iter()
        .map(|(&id, vertex)| (id, vertex.point))
        .collect();

    let mut edges = HashMap::with_capacity(model.edges.len());
    for (&id, edge) in model.edges.iter() {
        edges.insert(id, sample_curve(&edge.curve, n)?);
    }

    let mut faces = HashMap::with_capacity(model.faces.len());
    for (&id, face) in model.faces.iter() {
        let mut tris = Vec::new();
        // Grid corners are shared by up to six triangles, and a normal costs
        // two derivative evaluations, so each `(u, v)` is evaluated once.
        let mut normals: HashMap<[u64; 2], Option<Vector3<S>>> = HashMap::new();
        for (uv_a, uv_b, uv_c) in face_triangles_uv(model, face, n)? {
            let a = face.surface.evaluate(uv_a[0], uv_a[1])?;
            let b = face.surface.evaluate(uv_b[0], uv_b[1])?;
            let c = face.surface.evaluate(uv_c[0], uv_c[1])?;
            let Ok(t) = TriangleFace::try_new(a, b, c) else {
                continue;
            };
            // The surface's own normal at each corner, for smooth shading.
            // A corner where it does not exist (a pole, where the two
            // derivatives are parallel) leaves the whole triangle flat.
            let corner = |uv: Vector2<S>, cache: &mut HashMap<[u64; 2], Option<Vector3<S>>>| {
                let key = [uv[0].to_f64().to_bits(), uv[1].to_f64().to_bits()];
                *cache
                    .entry(key)
                    .or_insert_with(|| face.surface.normal(uv[0], uv[1]).ok())
            };
            let (na, nb, nc) = (
                corner(uv_a, &mut normals),
                corner(uv_b, &mut normals),
                corner(uv_c, &mut normals),
            );
            tris.push(match (na, nb, nc) {
                (Some(na), Some(nb), Some(nc)) => t.with_vertex_normals([na, nb, nc]),
                _ => t,
            });
        }
        faces.insert(id, tris);
    }

    Ok(RasterizedModel {
        vertices,
        edges,
        faces,
    })
}

/// Rasterize `model` into a [`PrimitiveScene`]: `points` (one per vertex),
/// `lines` (one per edge), and `triangles` (one mesh per face, `n` samples
/// per parametric direction / edge_loop segment). Untrimmed (no-hole) faces
/// are drawn in blue, holed faces in olive.
pub fn rasterize_model<S: Scalar>(model: &Model<S>, n: usize) -> GeopResult<PrimitiveScene<S>> {
    rasterize_model_impl(model, n, false, &default_face_color)
}

/// Like `rasterize_model`, but draws each face as a wireframe of its
/// triangulation instead of filled/shaded triangles — useful when overlaying
/// traced intersection curves on top of the faces, since a solid mesh can
/// occlude or visually blend with the curves.
pub fn rasterize_model_wireframe<S: Scalar>(
    model: &Model<S>,
    n: usize,
) -> GeopResult<PrimitiveScene<S>> {
    rasterize_model_impl(model, n, true, &default_face_color)
}

/// Like `rasterize_model`, but `face_color` picks each face's color
/// directly (by `FaceId`) instead of the default blue/olive
/// untrimmed/holed convention — e.g. coloring by which solid a face
/// belongs to, regardless of whether that face happens to have a hole.
pub fn rasterize_model_with_face_color<S: Scalar>(
    model: &Model<S>,
    n: usize,
    face_color: impl Fn(FaceId) -> Color10,
) -> GeopResult<PrimitiveScene<S>> {
    rasterize_model_impl(model, n, false, &move |id, _face| face_color(id))
}

fn default_face_color<S: Scalar>(_id: FaceId, _face: &Face<S>) -> Color10 {
    Color10::Blue
}

fn rasterize_model_impl<S: Scalar>(
    model: &Model<S>,
    n: usize,
    wireframe: bool,
    face_color: &dyn Fn(FaceId, &Face<S>) -> Color10,
) -> GeopResult<PrimitiveScene<S>> {
    let rasterized = rasterize_model_tagged(model, n)?;
    let mut scene = PrimitiveScene::new();

    for point in rasterized.vertices.into_values() {
        scene.add_point(point, Color10::DarkGray);
    }

    for polyline in rasterized.edges.into_values() {
        scene.add_polyline(&polyline, Color10::Gray);
    }

    for (face_id, tris) in rasterized.faces {
        let face = model.get_face(face_id)?;
        let color = face_color(face_id, face);
        for t in tris {
            if wireframe {
                for (p, q) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
                    if let Ok(l) = geop_core_math::primitives::Line::try_new(p, q) {
                        scene.add_line(l, color);
                    }
                }
            } else {
                scene.add_triangle(t, color);
            }
        }
    }

    Ok(scene)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;
    use geop_core_math::primitives::TriangleFace;
    use geop_ops::Part;
    use geop_ops_extrude_revolve::{cube_solid, sphere::sphere_solid};

    /// Rasterize `model`, sanity-check it's a non-empty mesh, and save it to
    /// `outputs/<name>.html` for visual inspection.
    fn rasterize_and_save<S: Scalar>(model: &Model<S>, name: &str) {
        let scene = rasterize_model(model, 8).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles.is_empty());
        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file(&format!("outputs/{name}.html")).unwrap();
    }

    fn check_rasterize_cube<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t1",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        rasterize_and_save(&model, "cube");
    }
    #[test]
    fn rasterize_cube() {
        for_all_scalars!(check_rasterize_cube);
    }

    // `tetrahedron_solid`/`figure8_profile`/`revolve` are currently
    // unavailable (disabled/removed in `basic_shapes` during the ongoing
    // euler-op rewrite) — re-add their rasterize tests once they're back.

    /// On a unit sphere the surface normal at a point *is* that point, so a
    /// mesh carrying the kernel's normals can be checked exactly — and a
    /// renderer shading with them gets the sphere, not its facets.
    fn check_sphere_triangles_carry_surface_normals<S: Scalar>() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t3", Vector3::zero(), S::ONE).unwrap();
        let model = part.topology();
        let scene = rasterize_model(&model, 24).unwrap();
        assert!(!scene.triangles.is_empty());
        let mut with_normals = 0;
        for (t, _) in &scene.triangles {
            let Some(normals) = t.vertex_normals else {
                continue;
            };
            with_normals += 1;
            for (n, p) in normals.iter().zip([t.a, t.b, t.c]) {
                // Sign follows the winding, so compare the directions.
                let dot = n.prod_dot(&p).to_f64().abs();
                assert!(
                    (dot - 1.0).abs() < 1e-6,
                    "normal {n:?} is not the sphere's own at {p:?} (|n·p| = {dot})"
                );
            }
        }
        assert!(
            with_normals * 20 > scene.triangles.len(),
            "only {with_normals} of {} triangles carry surface normals",
            scene.triangles.len()
        );
    }
    #[test]
    fn sphere_triangles_carry_surface_normals() {
        for_all_scalars!(check_sphere_triangles_carry_surface_normals);
    }

    /// A face that curves in one direction only gets triangles only where
    /// it needs them: a cylinder wall is refined around its circumference
    /// and left at the coarsest grid along its (straight) axis. Without
    /// that, a long cylinder spends as many rows up its side as it does
    /// segments around it, for nothing.
    fn check_cylinder_wall_is_refined_only_around<S: Scalar>() {
        let mut part = Part::<S>::new();
        geop_ops_extrude_revolve::cylinder::revolved_cylinder(
            &mut part,
            "t5",
            Vector3::zero(),
            S::ONE,
            S::from_f64(4.0),
        )
        .unwrap();
        let model = part.topology();
        let rasterized = rasterize_model_tagged(&model, 24).unwrap();
        // The wall quadrants are the faces whose triangles are all off-axis.
        let walls: Vec<&Vec<TriangleFace<S>>> = rasterized
            .faces
            .values()
            .filter(|tris| {
                !tris.is_empty()
                    && tris.iter().all(|t| {
                        [t.a, t.b, t.c]
                            .iter()
                            .all(|p| p[0].mul(p[0]).add(p[1].mul(p[1])).could_be_equal(S::ONE))
                    })
            })
            .collect();
        assert_eq!(walls.len(), 4, "a revolved cylinder has 4 wall quadrants");
        for tris in walls {
            // 2 rows up the axis x 32 segments around x 2 triangles is the
            // budget; a square grid would be 16 times that.
            assert!(
                tris.len() <= 2 * 2 * 32,
                "a cylinder wall quadrant came out as {} triangles",
                tris.len()
            );
            assert!(tris.len() >= 2 * 2 * 8, "and it still has to look round");
        }
    }
    #[test]
    fn cylinder_wall_is_refined_only_around() {
        for_all_scalars!(check_cylinder_wall_is_refined_only_around);
    }

    /// A straight edge needs two points; a circular one needs enough to look
    /// round. Neither is a fixed count any more.
    fn check_edge_sampling_follows_curvature<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t2",
            Vector3::zero(),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let cube_edges = rasterize_model_tagged(part.topology(), 24).unwrap().edges;
        for polyline in cube_edges.values() {
            assert_eq!(polyline.len(), 2, "a straight edge is a single segment");
        }

        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t4", Vector3::zero(), S::ONE).unwrap();
        let model = part.topology();
        let sphere_edges = rasterize_model_tagged(&model, 24).unwrap().edges;
        for polyline in sphere_edges.values() {
            // A quarter circle of radius 1 within `1 / (4 * 24²)` of the arc
            // needs 16 segments (its sagitta falls off with the square).
            assert!(
                polyline.len() >= 17,
                "a quarter circle came out as {} points",
                polyline.len()
            );
            for p in polyline {
                let r = p[0].mul(p[0]).add(p[1].mul(p[1])).add(p[2].mul(p[2]));
                assert!(r.could_be_equal(S::ONE), "sample off the sphere: {p:?}");
            }
        }
    }
    #[test]
    fn edge_sampling_follows_curvature() {
        for_all_scalars!(check_edge_sampling_follows_curvature);
    }
}
