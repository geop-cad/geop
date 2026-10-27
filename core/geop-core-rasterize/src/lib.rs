//! Turn a [`Model`] into sampled geometry — [`rasterize`]: a point per
//! vertex, a polyline per edge, triangles per face, each tagged with the
//! entity it came from — to draw it, pick in it, write it as STL ([`stl`]),
//! or render it for debugging ([`debug`]).
//!
//! Every edge is sampled once ([`sample_curve`]), and every face — trimmed
//! or not — has its outer boundary (and any holes) made of those samples,
//! placed in its `(u, v)` (see [`mesh::boundary`]), then triangulated by
//! [`grid::triangulate_face`]: a curvature-sized `(u, v)` grid with every
//! cell clipped exactly against the trim (see that module's doc comment),
//! so a flat patch renders as a couple of triangles and a curved one gets
//! whatever resolution its own curvature actually needs — mapped through
//! `surface.evaluate`, the rendered mesh always both respects the face's
//! real trim and follows its true curvature. The faces' meshes are then
//! joined along their edges (see [`mesh`]): the mesh of a solid is closed.

mod clip;
pub mod debug;
mod grid;
mod mesh;
pub mod stl;

pub use grid::triangulate_polygon;

/// A triangle in a plane, by its corners.
pub type Triangle2 = (Vector2, Vector2, Vector2);

use std::collections::BTreeMap;

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain::distance,
    primitives::TriangleFace,
    scalars::Scalar,
    solvers::refine::{Refine, Refined, refine},
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    CoedgeGeometry, CoedgeId, Curve2, Edge, EdgeId, Face, FaceId, Model, Sense, VertexId,
    boundary::BoundaryType,
};
use grid::dist_point_to_segment;

/// `face`'s trimmed region triangulated in `(u, v)` space, by a grid sized
/// by the face's curvature and clipped to its trim (see
/// [`grid::triangulate_face`]), each triangle by its corners — and the
/// trim it covers: its loops as the polygons of its edges' samples, the
/// outer first.
pub fn face_triangles_uv(
    model: &Model,
    face: &Face,
    n: usize,
) -> GeopResult<(Vec<Triangle2>, Vec<Vec<Vector2>>)> {
    let mut edges = BTreeMap::new();
    for anchor in loops(face) {
        for c in model.iterate_loop_coedges(anchor) {
            if let CoedgeGeometry::Edge(e) = model.get_coedge(c)?.geometry {
                edges.insert(e, sample_edge(model, e, n)?);
            }
        }
    }
    let mut samples = grid::Samples::new(face);
    let (triangles, boundaries) = face_mesh(model, face, n, &edges, &mut samples)?;
    let at = |p: &clip::Point| Vector2::from_array(p.map(Scalar::from_f64));
    Ok((
        triangles
            .into_iter()
            .map(|[a, b, c]| (samples.uv(a), samples.uv(b), samples.uv(c)))
            .collect(),
        boundaries
            .iter()
            .map(|b| b.uv.iter().map(at).collect())
            .collect(),
    ))
}

/// The anchors of `face`'s trim loops, the outer first.
fn loops(face: &Face) -> Vec<geop_core_topology::CoedgeId> {
    std::iter::once(&face.outer)
        .chain(&face.holes)
        .filter_map(|b| match b {
            BoundaryType::Loop(anchor) => Some(*anchor),
            BoundaryType::Vertex(_) => None,
        })
        .collect()
}

/// `face` triangulated in `(u, v)`, trimmed by the polylines `edges` its
/// edges were sampled into: the triangles, and its trim loops.
fn face_mesh(
    model: &Model,
    face: &Face,
    n: usize,
    edges: &BTreeMap<EdgeId, Vec<Vector3>>,
    samples: &mut grid::Samples,
) -> GeopResult<(Vec<[clip::Point; 3]>, Vec<mesh::Boundary>)> {
    let BoundaryType::Loop(_) = face.outer else {
        return Ok((Vec::new(), Vec::new()));
    };
    let boundaries = loops(face)
        .into_iter()
        .map(|anchor| mesh::boundary(model, face, anchor, n, edges))
        .collect::<GeopResult<Vec<_>>>()?;
    let holes: Vec<Vec<clip::Point>> = boundaries[1..].iter().map(|b| b.uv.clone()).collect();
    let triangles = grid::triangulate_face(&boundaries[0].uv, &holes, n, samples)?;
    Ok((triangles, boundaries))
}

/// Edge `e` of `model` sampled (see [`sample_curve`]), its ends exactly
/// its vertices' points: the polyline every face it bounds meets along.
fn sample_edge(model: &Model, e: EdgeId, n: usize) -> GeopResult<Vec<Vector3>> {
    let edge = model.get_edge(e)?;
    let mut points = sample_curve(&edge.curve, n)?;
    let last = points.len() - 1;
    points[0] = model.get_vertex(edge.start_vertex)?.point;
    points[last] = model.get_vertex(edge.end_vertex)?.point;
    Ok(points)
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
    Rasterizer::default().rasterize(model, n)
}

/// How many models a [`Rasterizer`] keeps the faces of: those of the part
/// before the step an editor shows, the part with it, and those of the
/// last steps opened.
const KEPT_MODELS: u64 = 4;

/// What a face is meshed from on its own ([`face_mesh`]), at a quality:
/// its surface, and each of its loops, the outer first, as the coedges it
/// runs through — each one's pcurve and sense, and its edge, with the
/// edge's ends. Two faces of equal sources are meshed alike, bit for bit,
/// whatever their shells or their coedges' ids.
#[derive(PartialEq)]
struct FaceSource {
    n: usize,
    surface: NurbSurface3D,
    loops: Vec<Vec<CoedgeSource>>,
}

/// A coedge of a [`FaceSource`]: what it runs along, which way, its pcurve,
/// and its edge's curve and ends, if it has an edge.
type CoedgeSource = (CoedgeGeometry, Sense, Curve2, Option<(Edge, [Vector3; 2])>);

impl FaceSource {
    fn of(model: &Model, face: &Face, n: usize) -> GeopResult<Self> {
        let coedge = |c: CoedgeId| -> GeopResult<CoedgeSource> {
            let coedge = model.get_coedge(c)?;
            let edge = match coedge.geometry {
                CoedgeGeometry::Edge(e) => {
                    let edge = model.get_edge(e)?;
                    let start = model.get_vertex(edge.start_vertex)?.point;
                    let end = model.get_vertex(edge.end_vertex)?.point;
                    Some((edge.clone(), [start, end]))
                }
                CoedgeGeometry::Vertex(_) => None,
            };
            Ok((coedge.geometry, coedge.sense, coedge.pcurve.clone(), edge))
        };
        let loops = (loops(face).into_iter())
            .map(|anchor| model.iterate_loop_coedges(anchor).map(coedge).collect())
            .collect::<GeopResult<_>>()?;
        Ok(Self {
            n,
            surface: face.surface.clone(),
            loops,
        })
    }
}

/// A face meshed on its own, before it is joined to its neighbours (see
/// [`mesh`]): its triangles in `(u, v)`, its trim loops, and its surface
/// where they sampled it.
struct Meshed {
    triangles: Vec<[clip::Point; 3]>,
    boundaries: Vec<mesh::Boundary>,
    samples: grid::Samples,
}

/// Rasterizes models one after another, meshing a face on its own only
/// once while it stays the same: the part before a step and the part with
/// it share the faces the step left as they were, and those are not
/// meshed again. The faces are joined along their edges anew each time,
/// so what it makes of a model is what [`rasterize`] makes of it.
#[derive(Default)]
pub struct Rasterizer {
    /// How many models it rasterized.
    models: u64,
    /// The faces it meshed, by id: from what, and the model that last
    /// used each — kept while one of the last [`KEPT_MODELS`] did.
    faces: BTreeMap<FaceId, Vec<(FaceSource, Meshed, u64)>>,
}

thread_local! {
    static FACES_MESHED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many faces this thread has meshed on its own ever: what a
/// [`Rasterizer`] does not do again for a face that stayed the same — so a
/// test bounds it by counting these, not by timing it.
pub fn faces_meshed() -> usize {
    FACES_MESHED.get()
}

impl Rasterizer {
    /// `model` rasterized as [`rasterize`] does, of the faces meshed
    /// before what stayed the same.
    pub fn rasterize(&mut self, model: &Model, n: usize) -> GeopResult<RasterizedModel> {
        if n < 2 {
            return Err(GeopError::new("rasterize: n must be >= 2"));
        }
        self.models += 1;
        let now = self.models;

        let vertices = model
            .vertices
            .iter()
            .map(|(&id, vertex)| (id, vertex.point))
            .collect();

        let mut edges = BTreeMap::new();
        for &id in model.edges.keys() {
            edges.insert(id, sample_edge(model, id, n)?);
        }
        let last: BTreeMap<EdgeId, usize> = edges.iter().map(|(&e, p)| (e, p.len() - 1)).collect();

        // Every face meshed on its own, or as it was, then all joined along
        // their edges.
        let mut ids = Vec::new();
        let mut sources = Vec::new();
        let mut all_samples = Vec::new();
        let mut all_boundaries = Vec::new();
        let mut triangles = Vec::new();
        for (&id, face) in model.faces.iter() {
            let source = FaceSource::of(model, face, n)?;
            let kept = self.faces.entry(id).or_default();
            let meshed = match kept.iter().position(|(meshed, ..)| *meshed == source) {
                Some(k) => kept.swap_remove(k).1,
                None => {
                    FACES_MESHED.set(FACES_MESHED.get() + 1);
                    let mut samples = grid::Samples::new(face);
                    let (triangles, boundaries) = face_mesh(model, face, n, &edges, &mut samples)?;
                    Meshed {
                        triangles,
                        boundaries,
                        samples,
                    }
                }
            };
            ids.push(id);
            sources.push(source);
            all_samples.push(meshed.samples);
            all_boundaries.push(meshed.boundaries);
            triangles.push(meshed.triangles);
        }
        let trims: Vec<mesh::Trim> = all_boundaries
            .iter()
            .map(|boundaries| mesh::Trim {
                model,
                boundaries,
                last: &last,
            })
            .collect();
        let mut meshes: Vec<mesh::Mesh> = triangles
            .iter()
            .zip(&trims)
            .map(|(tris, trim)| mesh::Mesh::new(tris, trim))
            .collect::<GeopResult<_>>()?;
        mesh::zip(&mut meshes, &trims);

        let mut faces = BTreeMap::new();
        for ((&id, m), samples) in ids.iter().zip(&meshes).zip(&mut all_samples) {
            let points = (0..m.uv.len())
                .map(|i| match mesh::placed(model, m.on[i], &edges)? {
                    Some(p) => Ok(p),
                    None => samples.point(m.uv[i]),
                })
                .collect::<GeopResult<Vec<_>>>()?;
            let mut tris = Vec::new();
            for &[a, b, c] in &m.triangles {
                // A triangle without area — three points along an edge —
                // has no normal: it covers nothing, and the triangles
                // beside it meet each other.
                let Ok(t) = TriangleFace::try_new(points[a], points[b], points[c]) else {
                    continue;
                };
                // The surface's own normal at each corner, for smooth
                // shading. A corner where it does not exist leaves the
                // whole triangle flat.
                let mut normal = |i: usize| samples.normal(m.uv[i]);
                tris.push(match (normal(a), normal(b), normal(c)) {
                    (Some(na), Some(nb), Some(nc)) => t.with_vertex_normals([na, nb, nc]),
                    _ => t,
                });
            }
            faces.insert(id, tris);
        }

        // Each face kept as it was meshed on its own, with what its surface
        // was sampled at since, for the next model.
        let meshed = (triangles.into_iter().zip(all_boundaries).zip(all_samples)).map(
            |((triangles, boundaries), samples)| Meshed {
                triangles,
                boundaries,
                samples,
            },
        );
        for ((id, source), meshed) in ids.into_iter().zip(sources).zip(meshed) {
            self.faces
                .entry(id)
                .or_default()
                .push((source, meshed, now));
        }
        self.faces.retain(|_, kept| {
            kept.retain(|(.., used)| used + KEPT_MODELS > now);
            !kept.is_empty()
        });

        Ok(RasterizedModel {
            vertices,
            edges,
            faces,
        })
    }
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
