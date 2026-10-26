//! The rasterization of curved and swept solids (see
//! [`geop_core_rasterize`]), tested here because these are the shapes this
//! crate builds: spheres, cylinders, extrusions with holes.

use geop_core_math::{
    plain::distance,
    primitives::TriangleFace,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_rasterize::{debug::Color10, face_triangles_uv, rasterize};
use geop_ops_model::NamedModel;

use crate::shapes::{cube_solid, sphere_solid};

fn triangle_area(a: Vector2, b: Vector2, c: Vector2) -> f64 {
    let (ax, ay) = (a[0].to_f64(), a[1].to_f64());
    let (bx, by) = (b[0].to_f64(), b[1].to_f64());
    let (cx, cy) = (c[0].to_f64(), c[1].to_f64());
    ((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)).abs() / 2.0
}

/// On a unit sphere the surface normal at a point *is* that point, so a
/// mesh carrying the kernel's normals can be checked exactly — and a
/// renderer shading with them gets the sphere, not its facets.
#[test]
fn sphere_triangles_carry_surface_normals() {
    let mut model = NamedModel::new();
    sphere_solid(&mut model, "t3", Vector3::zero(), Scalar::ONE).unwrap();
    let topology = model.topology();
    let scene = rasterize(topology, 24).unwrap().scene(|_| Color10::Blue);
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

/// A face that curves in one direction only gets triangles only where
/// it needs them: a cylinder wall is refined around its circumference
/// and left at the coarsest grid along its (straight) axis. Without
/// that, a long cylinder spends as many rows up its side as it does
/// segments around it, for nothing.
#[test]
fn cylinder_wall_is_refined_only_around() {
    let mut model = NamedModel::new();
    crate::shapes::revolved_cylinder(
        &mut model,
        "t5",
        Vector3::zero(),
        Scalar::ONE,
        Scalar::from_f64(4.0),
    )
    .unwrap();
    let topology = model.topology();
    let rasterized = rasterize(topology, 24).unwrap();
    // The wall quadrants are the faces whose triangles are all off-axis:
    // on the cylinder, or on a chord of its rim, where a point of the mesh
    // lies on the rim's polyline (the faces meet along it).
    let walls: Vec<&Vec<TriangleFace>> = rasterized
        .faces
        .values()
        .filter(|tris| {
            !tris.is_empty()
                && tris.iter().all(|t| {
                    [t.a, t.b, t.c]
                        .iter()
                        .all(|p| p[0].to_f64().hypot(p[1].to_f64()) > 0.99)
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

/// A straight edge needs two points; a circular one needs enough to look
/// round. Neither is a fixed count any more.
#[test]
fn edge_sampling_follows_curvature() {
    let mut model = NamedModel::new();
    cube_solid(
        &mut model,
        "t2",
        Vector3::zero(),
        Vector3::from_array([Scalar::ONE; 3]),
    )
    .unwrap();
    let cube_edges = rasterize(model.topology(), 24).unwrap().edges;
    for polyline in cube_edges.values() {
        assert_eq!(polyline.len(), 2, "a straight edge is a single segment");
    }

    let mut model = NamedModel::new();
    sphere_solid(&mut model, "t4", Vector3::zero(), Scalar::ONE).unwrap();
    let topology = model.topology();
    let sphere_edges = rasterize(topology, 24).unwrap().edges;
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
            assert!(
                r.could_be_equal(Scalar::ONE),
                "sample off the sphere: {p:?}"
            );
        }
    }
}

#[test]
fn curved_face_is_refined() {
    let mut model = NamedModel::new();
    sphere_solid(&mut model, "t3", Vector3::zero(), Scalar::ONE).unwrap();
    let topology = model.topology();
    let face = topology.faces.values().next().unwrap();
    let tris = face_triangles_uv(topology, face, 8).unwrap().0;
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
#[test]
fn curved_face_follows_the_surface() {
    let mut model = NamedModel::new();
    sphere_solid(&mut model, "t4", Vector3::zero(), Scalar::ONE).unwrap();
    let topology = model.topology();
    let face = topology.faces.values().next().unwrap();
    let tris = face_triangles_uv(topology, face, 24).unwrap().0;
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
            .map(|uv| face.surface.evaluate(uv[0], uv[1]).unwrap().to_f64_array())
            .collect();
        let centroid = [0, 1, 2].map(|k| corners.iter().map(|p| p[k]).sum::<f64>() / 3.0);
        let _ = mid;
        worst = worst.max((1.0 - distance(centroid, [0.0; 3])).abs());
    }
    // `n = 24` asks for a sagitta under `face_size / (4 n²)`, which for
    // this patch (size ≈ 1.7) is about 7e-4.
    assert!(worst < 1e-3, "worst deviation from the sphere: {worst}");
}

/// A face with a round hole must not get a single triangle inside that
/// hole: the hole's area is missing from the triangulation, and no
/// triangle's centroid falls in it.
#[test]
fn hole_is_not_triangulated() {
    use crate::common::polygon;
    use geop_core_geometry::nurb_curve::{NurbCurve2D, sqrt2_over_2};
    use geop_core_math::primitives::CoordinateSystem;

    let mut model = NamedModel::new();
    let v2 = |x: f64, y: f64| Vector2::from_array([Scalar::from_f64(x), Scalar::from_f64(y)]);
    let cs = CoordinateSystem::try_new(
        Vector3::from_array([Scalar::ZERO; 3]),
        Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]),
        Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]),
        Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::from_f64(-1.0)]),
    )
    .unwrap();
    let outer = polygon(&[v2(0.0, 0.0), v2(4.0, 0.0), v2(4.0, 4.0), v2(0.0, 4.0)]).unwrap();
    // A circle of radius 1 around (2, 2), clockwise (a hole).
    let q = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
    let ccw: Vec<_> = (0..4)
        .map(|i| {
            let (a, b) = (q[i], q[(i + 1) % 4]);
            NurbCurve2D::arc(
                v2(2.0 + a.0, 2.0 + a.1),
                v2(2.0 + a.0 + b.0, 2.0 + a.1 + b.1),
                v2(2.0 + b.0, 2.0 + b.1),
                sqrt2_over_2(),
            )
            .unwrap()
        })
        .collect();
    let hole: Vec<_> = ccw.iter().rev().map(|c| c.reverse()).collect();
    let namer = geop_ops_model::Namer::new("extrude", "e").unwrap();
    use crate::sweep::SweepLoop;
    use geop_ops_model::Profile;
    crate::extrude::extrude(
        &mut model,
        &namer,
        Some(&namer.root()),
        &cs,
        &[Scalar::ZERO, Scalar::ONE],
        &[
            SweepLoop::plain(Profile::closed(outer)),
            SweepLoop::plain(Profile::closed(hole).with_prefix("h")),
        ],
    )
    .unwrap();
    let topology = model.topology();

    // The two cap faces are the ones with a hole.
    let capped: Vec<_> = topology
        .faces
        .values()
        .filter(|f| !f.holes.is_empty())
        .collect();
    assert_eq!(capped.len(), 2);
    for face in capped {
        let tris = face_triangles_uv(topology, face, 8).unwrap().0;
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

/// A genuinely non-rectangular, holed footprint (unlike a cube/sphere
/// face) must still triangulate cleanly: every triangle finite, and
/// some non-trivial mesh actually produced — a crash or an empty
/// result here is exactly the failure mode clipping is meant to rule
/// out for trims a fixed grid-without-clipping can't represent.
#[test]
fn holed_footprint_triangulates_cleanly() {
    let mut model = NamedModel::new();
    crate::shapes::figure8_profile(&mut model, "f").unwrap();
    let topology = model.topology();
    let mut total_triangles = 0;
    for face in topology.faces.values() {
        let tris = face_triangles_uv(topology, face, 8).unwrap().0;
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

/// STL facets of curved solids, and of a box.
mod stl {
    use geop_core_math::{scalars::Scalar, vector::Vector3};
    use geop_core_rasterize::{
        rasterize,
        stl::{StlFormat, StlTriangle, stl_triangles, write_stl},
    };
    use geop_core_topology::FaceId;
    use geop_ops_model::NamedModel;

    use crate::shapes::{cube_solid, sphere_solid};

    fn triangles_of(model: &NamedModel) -> Vec<StlTriangle> {
        let raster = rasterize(model.topology(), 8).unwrap();
        let mut faces: Vec<FaceId> = raster.faces.keys().copied().collect();
        faces.sort_by_key(|f| f.0);
        stl_triangles(&raster, &faces)
    }

    fn unit_cube() -> Vec<StlTriangle> {
        let mut model = NamedModel::new();
        cube_solid(
            &mut model,
            "c",
            Vector3::from_array([Scalar::ZERO; 3]),
            Vector3::from_array([Scalar::ONE; 3]),
        )
        .unwrap();
        triangles_of(&model)
    }

    /// Every facet of a closed convex solid around `centre` faces away from
    /// it, by its winding and by its normal.
    fn assert_outward(triangles: &[StlTriangle], centre: [f32; 3]) {
        assert!(!triangles.is_empty());
        let sub = |p: [f32; 3], q: [f32; 3]| [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
        let dot = |p: [f32; 3], q: [f32; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
        for t in triangles {
            let [a, b, c] = t.corners;
            let (u, v) = (sub(b, a), sub(c, a));
            let cross = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let out = sub(a, centre);
            assert!(dot(cross, out) > 0.0, "{t:?} winds inward");
            assert!(dot(t.normal, out) > 0.0, "{t:?} has an inward normal");
        }
    }

    #[test]
    fn cube_facets_face_outward() {
        assert_outward(&unit_cube(), [0.5; 3]);
    }

    /// Curved faces: the winding has to come from the surface normals.
    #[test]
    fn sphere_facets_face_outward() {
        let mut model = NamedModel::new();
        sphere_solid(
            &mut model,
            "s",
            Vector3::from_array([Scalar::ZERO; 3]),
            Scalar::ONE,
        )
        .unwrap();
        assert_outward(&triangles_of(&model), [0.0; 3]);
    }

    #[test]
    fn binary_layout() {
        let triangles = unit_cube();
        let mut bytes = Vec::new();
        write_stl(&triangles, "cube", StlFormat::Binary, &mut bytes).unwrap();
        assert_eq!(bytes.len(), 84 + 50 * triangles.len());
        assert!(!bytes.starts_with(b"solid"));
        assert_eq!(
            u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize,
            triangles.len()
        );
    }

    #[test]
    fn ascii_layout() {
        let triangles = unit_cube();
        let mut bytes = Vec::new();
        write_stl(&triangles, "my cube", StlFormat::Ascii, &mut bytes).unwrap();
        let text = String::from_utf8(bytes).unwrap();
        assert!(text.starts_with("solid my_cube\n"));
        assert!(text.trim_end().ends_with("endsolid my_cube"));
        assert_eq!(text.matches("facet normal").count(), triangles.len());
        assert_eq!(text.matches("vertex").count(), 3 * triangles.len());
    }
}

/// Checks the mesh of every face of `model`, wound outward as an STL file
/// has it, is closed: every side of a triangle is the side of exactly one
/// other, the other way round — no crack between two faces, no T where a
/// point of one lies on a side of the other.
#[track_caller]
fn assert_closed(model: &geop_core_topology::Model, n: usize) {
    use geop_core_rasterize::stl::stl_triangles;
    use std::collections::HashMap;
    let raster = rasterize(model, n).unwrap();
    let key = |p: [f32; 3]| p.map(f32::to_bits);
    // Per directed side, the faces whose triangles run it.
    let mut sides: HashMap<([u32; 3], [u32; 3]), Vec<geop_core_topology::FaceId>> = HashMap::new();
    for &face in raster.faces.keys() {
        for t in stl_triangles(&raster, &[face]) {
            for k in 0..3 {
                let (a, b) = (key(t.corners[k]), key(t.corners[(k + 1) % 3]));
                sides.entry((a, b)).or_default().push(face);
            }
        }
    }
    assert!(!sides.is_empty());
    let open: Vec<_> = sides
        .iter()
        .filter(|&(&(a, b), faces)| faces.len() != 1 || sides.get(&(b, a)).map(Vec::len) != Some(1))
        .map(|(&(a, b), faces)| {
            (
                a.map(f32::from_bits),
                b.map(f32::from_bits),
                faces.clone(),
                sides.get(&(b, a)).cloned().unwrap_or_default(),
            )
        })
        .collect();
    assert!(
        open.is_empty(),
        "{} of {} sides are not matched once the other way round, e.g. {:?}",
        open.len(),
        sides.len(),
        &open[..open.len().min(6)]
    );
}

/// A sphere, a cylinder, and a block with a round hole through it: each
/// face meets its neighbours along the very samples of their shared edges,
/// so the mesh is closed — at the poles, round the caps, through the hole.
#[test]
fn meshes_of_solids_are_closed() {
    let mut sphere = NamedModel::new();
    sphere_solid(&mut sphere, "s", Vector3::zero(), Scalar::ONE).unwrap();
    assert_closed(sphere.topology(), 12);

    let mut cylinder = NamedModel::new();
    crate::shapes::revolved_cylinder(
        &mut cylinder,
        "c",
        Vector3::zero(),
        Scalar::ONE,
        Scalar::from_f64(2.0),
    )
    .unwrap();
    assert_closed(cylinder.topology(), 12);

    let mut drilled = NamedModel::new();
    let namer = geop_ops_model::Namer::new("extrude", "d").unwrap();
    let loops = [
        geop_ops_model::Profile::closed(crate::shapes::square(1.0)),
        geop_ops_model::Profile::closed(crate::shapes::circle_at(0.2, 0.1, 0.4, true))
            .with_prefix("h,"),
    ]
    .map(crate::SweepLoop::plain);
    crate::extrude(
        &mut drilled,
        &namer,
        Some(&namer.root()),
        &geop_core_math::primitives::CoordinateSystem::world_at(Vector3::zero()),
        &[Scalar::ZERO, Scalar::ONE],
        &loops,
    )
    .unwrap();
    for n in [4, 12, 24] {
        assert_closed(drilled.topology(), n);
    }
}
