//! Test-only fixture: a plain axis-aligned unit-cube-shaped solid, built
//! directly from this crate's own low-level `insert_*` methods.
//!
//! This crate sits *below* `geop-ops-extrude-revolve` in the workspace's
//! dependency order (that crate is what actually builds shapes like a real
//! `cube_solid` via `extrude`), so validation/euler/edit tests here that
//! just need "some realistic populated `Model`" to run structural or
//! numerical checks against can't reach for it — a dev-dependency back on
//! `geop-ops-extrude-revolve` would make Cargo compile this crate twice
//! (once under `#[cfg(test)]`, once as that crate's plain library
//! dependency), and the two copies' `Model<S>` types don't unify, which
//! surfaces as `expected Model, found Model` at every call site. This is a
//! small, self-contained stand-in scoped to exactly what these tests need:
//! six planar faces, twelve shared edges, eight shared vertices — no
//! `extrude` generality (arbitrary footprints, holes) required.

use geop_core_geometry::{nurb_curve::NurbCurve, nurb_surface::NurbSurface3D};
use geop_core_math::{scalars::Scalar, vector::Vector3};
use std::collections::HashMap;

use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, Face, Model, Sense, Shell, Solid, SolidId, Vertex,
    VertexId, boundary::BoundaryType,
};

/// Build `model`'s unit cube (corners at `0`/`1` on every axis) and return
/// its `SolidId`. Every face is a flat, degree-`1` bilinear patch; every
/// edge a straight degree-`1` line — plenty for the structural/numerical
/// checks these tests actually exercise.
pub fn test_cube_solid<S: Scalar>(model: &mut Model<S>) -> SolidId {
    let mut f = |x: f64, y: f64, z: f64| {
        model.insert_vertex(Vertex {
            point: Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)]),
        })
    };
    // v0..v3 the bottom (z=0) ring, v4..v7 the matching top (z=1) ring.
    let v0 = f(0.0, 0.0, 0.0);
    let v1 = f(1.0, 0.0, 0.0);
    let v2 = f(1.0, 1.0, 0.0);
    let v3 = f(0.0, 1.0, 0.0);
    let v4 = f(0.0, 0.0, 1.0);
    let v5 = f(1.0, 0.0, 1.0);
    let v6 = f(1.0, 1.0, 1.0);
    let v7 = f(0.0, 1.0, 1.0);

    let shell_id = model.insert_shell(Shell {
        faces: Vec::new(),
        solid: SolidId(0), // patched once the solid itself exists, below.
    });

    // Each face names its own boundary loop's corners in `(0,0) -> (1,0) ->
    // (1,1) -> (0,1)` order — chosen per face (see the comment inside
    // `planar_face`) so `Su x Sv` always comes out pointing away from the
    // solid's interior.
    let mut edges: HashMap<(VertexId, VertexId), EdgeRef> = HashMap::new();
    planar_face(model, shell_id, [v0, v3, v2, v1], &mut edges); // bottom
    planar_face(model, shell_id, [v4, v5, v6, v7], &mut edges); // top
    planar_face(model, shell_id, [v0, v1, v5, v4], &mut edges); // front (y=0)
    planar_face(model, shell_id, [v3, v7, v6, v2], &mut edges); // back (y=1)
    planar_face(model, shell_id, [v0, v4, v7, v3], &mut edges); // left (x=0)
    planar_face(model, shell_id, [v1, v2, v6, v5], &mut edges); // right (x=1)

    let solid_id = model.insert_solid(Solid {
        shells: vec![shell_id],
    });
    model.shells.get_mut(&shell_id).unwrap().solid = solid_id;
    solid_id
}

/// An already-built edge, keyed by its two endpoints in the direction its
/// `Curve3` itself runs (`start_vertex -> end_vertex`) — a face whose own
/// loop walks the same pair the other way round gets `Sense::Reversed`.
struct EdgeRef {
    id: crate::EdgeId,
    start: VertexId,
}

/// Insert one planar quad face into `shell_id`, spanning corners
/// `[a, b, c, d]` walked in `(0,0) -> (1,0) -> (1,1) -> (0,1)` order (so the
/// surface's control points, row-major `[P(0,0), P(0,1), P(1,0), P(1,1)]`,
/// are `[a, d, b, c]`). Reuses an already-built edge (in whichever
/// direction it was first created) for any side shared with an
/// already-inserted face, so the cube ends up with exactly 12 edges/8
/// vertices, not 24/48.
fn planar_face<S: Scalar>(
    model: &mut Model<S>,
    shell_id: crate::ShellId,
    [a, b, c, d]: [VertexId; 4],
    edges: &mut HashMap<(VertexId, VertexId), EdgeRef>,
) -> crate::FaceId {
    let pt = |model: &Model<S>, v: VertexId| model.vertices[&v].point;
    let surface = NurbSurface3D::try_new(
        1,
        1,
        vec![
            homogeneous(pt(model, a)),
            homogeneous(pt(model, d)),
            homogeneous(pt(model, b)),
            homogeneous(pt(model, c)),
        ],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
    .unwrap();

    let face_id = model.insert_face(Face {
        surface,
        outer: BoundaryType::Vertex(a), // patched to a real loop below.
        holes: Vec::new(),
        shell: shell_id,
    });

    // The loop's four sides, each as `(from, to, pcurve-domain corners)` —
    // `(u, v)` runs `(0,0) -> (1,0) -> (1,1) -> (0,1) -> (0,0)`.
    let sides = [
        (a, b, [S::ZERO, S::ZERO], [S::ONE, S::ZERO]),
        (b, c, [S::ONE, S::ZERO], [S::ONE, S::ONE]),
        (c, d, [S::ONE, S::ONE], [S::ZERO, S::ONE]),
        (d, a, [S::ZERO, S::ONE], [S::ZERO, S::ZERO]),
    ];

    let coedge_ids: Vec<CoedgeId> = sides
        .iter()
        .map(|&(from, to, uv0, uv1)| {
            let (edge_id, sense) = match edges.get(&(to, from)) {
                // The opposite face already built this edge the other way.
                Some(existing) if existing.start == to => (existing.id, Sense::Reversed),
                _ => match edges.get(&(from, to)) {
                    Some(existing) if existing.start == from => (existing.id, Sense::Forward),
                    _ => {
                        let curve = NurbCurve::try_new(
                            1,
                            vec![homogeneous(pt(model, from)), homogeneous(pt(model, to))],
                            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                        )
                        .unwrap();
                        let edge_id = model.insert_edge(Edge {
                            curve,
                            start_vertex: from,
                            end_vertex: to,
                        });
                        edges.insert(
                            (from, to),
                            EdgeRef {
                                id: edge_id,
                                start: from,
                            },
                        );
                        (edge_id, Sense::Forward)
                    }
                },
            };
            let pcurve = NurbCurve::try_new(
                1,
                vec![homogeneous2(uv0), homogeneous2(uv1)],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )
            .unwrap();
            model.insert_coedge(Coedge {
                geometry: CoedgeGeometry::Edge(edge_id),
                sense,
                pcurve,
                next: CoedgeId(0),
                prev: CoedgeId(0),
                face: face_id,
            })
        })
        .collect();

    for i in 0..4 {
        let cur = coedge_ids[i];
        let next = coedge_ids[(i + 1) % 4];
        let prev = coedge_ids[(i + 3) % 4];
        let coedge = model.coedges.get_mut(&cur).unwrap();
        coedge.next = next;
        coedge.prev = prev;
    }
    model.get_face_mut(face_id).unwrap().outer = BoundaryType::Loop(coedge_ids[0]);
    model.shells.get_mut(&shell_id).unwrap().faces.push(face_id);

    face_id
}

fn homogeneous<S: Scalar>(p: Vector3<S>) -> geop_core_math::vector::Vector4<S> {
    geop_core_math::vector::Vector4::from_array([p[0], p[1], p[2], S::ONE])
}

fn homogeneous2<S: Scalar>(uv: [S; 2]) -> geop_core_math::vector::Vector3<S> {
    geop_core_math::vector::Vector3::from_array([uv[0], uv[1], S::ONE])
}

/// A straight degree-`1` pcurve from `a` to `b`, in `(u, v)` parameter space.
pub fn line2<S: Scalar>(
    a: geop_core_math::vector::Vector2<S>,
    b: geop_core_math::vector::Vector2<S>,
) -> geop_core_geometry::nurb_curve::NurbCurve2D<S> {
    NurbCurve::try_new(
        1,
        vec![homogeneous2([a[0], a[1]]), homogeneous2([b[0], b[1]])],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
    .unwrap()
}
