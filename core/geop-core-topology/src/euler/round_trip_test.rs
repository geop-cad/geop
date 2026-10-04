use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, Model, Sense, Vertex, VertexId,
    boundary::BoundaryType,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    for_all_scalars,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

fn line3<S: Scalar>(a: (f64, f64, f64), b: (f64, f64, f64)) -> NurbCurve3D<S> {
    let p = |x: f64, y: f64, z: f64| {
        Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
    };
    NurbCurve3D::try_new(
        1,
        vec![p(a.0, a.1, a.2), p(b.0, b.1, b.2)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
    .unwrap()
}

// `NurbSurface3D::everything()` maps every (u, v) to a point that
// `could_be_equal`s anything, so the pcurve's actual shape never matters for
// validation here.
fn dummy_pcurve<S: Scalar>() -> NurbCurve2D<S> {
    let p = |x: f64, y: f64| Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE]);
    NurbCurve2D::try_new(
        1,
        vec![p(0.0, 0.0), p(1.0, 0.0)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
    .unwrap()
}

/// Builds a bare `n`-coedge cycle on `face`, with vertex `i` at point
/// `(offset + i, 0, 0)`. Returns the coedges in cycle order.
fn make_ring<S: Scalar>(
    model: &mut Model<S>,
    face: crate::FaceId,
    n: usize,
    offset: f64,
) -> Vec<CoedgeId> {
    let verts: Vec<VertexId> = (0..n)
        .map(|i| {
            model.insert_vertex(Vertex {
                point: Vector3::from_array([S::from_f64(offset + i as f64), S::ZERO, S::ZERO]),
            })
        })
        .collect();
    let edges: Vec<EdgeId> = (0..n)
        .map(|i| {
            let a = offset + i as f64;
            let b = offset + ((i + 1) % n) as f64;
            model.insert_edge(Edge {
                curve: line3((a, 0.0, 0.0), (b, 0.0, 0.0)),
                start_vertex: verts[i],
                end_vertex: verts[(i + 1) % n],
            })
        })
        .collect();
    let coedges: Vec<CoedgeId> = (0..n)
        .map(|i| {
            model.insert_coedge(Coedge {
                geometry: CoedgeGeometry::Edge(edges[i]),
                sense: Sense::Forward,
                pcurve: dummy_pcurve(),
                next: CoedgeId(0),
                prev: CoedgeId(0),
                face,
            })
        })
        .collect();
    for i in 0..n {
        model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % n];
        model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + n - 1) % n];
    }
    coedges
}

fn make_face<S: Scalar>(model: &mut Model<S>) -> crate::FaceId {
    let solid_id = model.insert_solid(crate::Solid { shells: vec![] });
    let shell_id = model.insert_shell(crate::Shell {
        faces: vec![],
        solid: Some(solid_id),
    });
    // A face always has exactly one outer boundary, so "no edges yet" is a
    // bare-vertex boundary — the state `mvfs` produces. `VertexId(0)` is a
    // deliberate non-id (real ids start at 1): these fixtures never look the
    // vertex up, they only need `outer` to read as "not a loop yet" so the
    // first `mer`/`mef` promotes it to the real ring. Using a freshly
    // inserted vertex instead would shift every vertex-count assertion here.
    let face_id = model.insert_face(Face {
        surface: NurbSurface3D::everything(),
        outer: BoundaryType::Vertex(crate::VertexId(0)),
        holes: Vec::new(),
        shell: shell_id,
    });
    model.shells.get_mut(&shell_id).unwrap().faces.push(face_id);
    face_id
}

fn ring_len<S: Scalar>(model: &Model<S>, start: CoedgeId) -> usize {
    let mut cursor = model.get_coedge(start).unwrap().next;
    let mut count = 1;
    while cursor != start {
        cursor = model.get_coedge(cursor).unwrap().next;
        count += 1;
    }
    count
}

fn check_mvfs_kvfs_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let (_v, _f, s) = model.mvfs(Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]));
    assert_eq!(model.vertices.len(), 1);

    model.kvfs(s).unwrap();
    assert!(model.vertices.is_empty());
    assert!(model.faces.is_empty());
    assert!(model.shells.is_empty());
    assert!(model.solids.is_empty());
}

fn check_mve_kve_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 3, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    // `ring[0]`'s end vertex is at `(1, 0, 0)` — `mve` now splices the new
    // edge in right after `ring[0]`, so the curve must start there.
    let curve = line3((1.0, 0.0, 0.0), (5.0, 0.0, 0.0));
    let new_point = Vector3::from_array([S::from_f64(5.0), S::ZERO, S::ZERO]);
    let (v, coedge_out, coedge_in, _edge_id) = model
        .mve(
            ring[0],
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            new_point,
        )
        .unwrap();
    assert_eq!(model.coedges.len(), 5);
    assert_eq!(ring_len(&model, ring[0]), 5);

    model.kve(coedge_out, coedge_in, v).unwrap();
    assert_eq!(model.vertices.len(), 3);
    assert_eq!(model.edges.len(), 3);
    assert_eq!(model.coedges.len(), 3);
    assert_eq!(ring_len(&model, ring[0]), 3);
}

fn check_mef_kef_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 4, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    // `ring[0]`'s end vertex is at `(1, 0, 0)`; `ring[2]`'s start vertex is
    // at `(2, 0, 0)` — the new edge mef's own convention now expects.
    let curve = line3((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
    let (edge_id, new_face_id, _coedge_a, _coedge_b) = model
        .mef(
            ring[0],
            ring[2],
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            NurbSurface3D::everything(),
        )
        .unwrap();
    assert_eq!(model.faces.len(), 2);
    assert_eq!(model.coedges.len(), 6);
    assert!(model.get_face(face).unwrap().holes.is_empty());
    assert!(model.get_face(new_face_id).unwrap().holes.is_empty());
    // `coedge1` (`ring[0]`) moved onto the new face, together with `ring[2]`
    // and `ring[3]`, plus the new chord — 4 coedges; the old face keeps just
    // `ring[1]` plus the chord's other coedge — 2 coedges.
    assert_eq!(ring_len(&model, ring[0]), 4);
    assert_eq!(ring_len(&model, ring[1]), 2);

    model.kef(edge_id, new_face_id).unwrap();
    assert_eq!(model.faces.len(), 1);
    assert_eq!(model.coedges.len(), 4);
    assert_eq!(model.edges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 4);
}

// Same shape as `check_mef_kef_round_trip`, but `coedge1` is a `Vertex`-backed
// coedge (as spliced in by `add_vertex_coedge`, e.g. bridging a pole's pcurve
// gap in `revolve`) rather than an ordinary edge-backed one — `mef` must
// resolve its start/end vertex through `CoedgeGeometry::Vertex` correctly
// (both the same vertex) instead of erroring or misresolving it via `.edge()`.
fn check_mef_from_vertex_coedge_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 4, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    // Splice a vertex-backed coedge in right after `ring[0]`, sitting at its
    // end vertex (`(1, 0, 0)`, shared with `ring[1]`'s start).
    let pole = model.coedge_end_vertex_id(ring[0]).unwrap();
    let vertex_coedge = model
        .add_vertex_coedge(ring[0], pole, dummy_pcurve())
        .unwrap();
    assert_eq!(ring_len(&model, ring[0]), 5);

    // The new chord must go from `vertex_coedge`'s (degenerate) end vertex —
    // `(1, 0, 0)` — to `ring[2]`'s start vertex — `(2, 0, 0)`.
    let curve = line3((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
    let (edge_id, new_face_id, _coedge_a, _coedge_b) = model
        .mef(
            vertex_coedge,
            ring[2],
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            NurbSurface3D::everything(),
        )
        .unwrap();
    assert_eq!(model.faces.len(), 2);
    // 5 original (4 edge-backed + 1 vertex-backed) + 2 new chord coedges.
    assert_eq!(model.coedges.len(), 7);
    assert!(model.get_face(face).unwrap().holes.is_empty());
    assert!(model.get_face(new_face_id).unwrap().holes.is_empty());
    // New face: vertex_coedge, chord forward, ring[2], ring[3], ring[0].
    assert_eq!(ring_len(&model, ring[0]), 5);
    // Old face: just ring[1] plus the chord's other coedge.
    assert_eq!(ring_len(&model, ring[1]), 2);

    model.kef(edge_id, new_face_id).unwrap();
    assert_eq!(model.faces.len(), 1);
    assert_eq!(model.coedges.len(), 5);
    assert_eq!(model.edges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 5);
}

// Same idea as `check_mef_from_vertex_coedge_round_trip`, but for `mer`:
// `coedge2` (rather than `coedge1`) is the `Vertex`-backed one.
fn check_mer_from_vertex_coedge_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face_a = make_face(&mut model);
    let ring = make_ring(&mut model, face_a, 4, 0.0);
    model.faces.get_mut(&face_a).unwrap().outer = BoundaryType::Loop(ring[0]);
    let face_b = make_face(&mut model);

    // Splice a vertex-backed coedge in right after `ring[1]`, sitting at its
    // end vertex (`(2, 0, 0)`, shared with `ring[2]`'s start).
    let pole = model.coedge_end_vertex_id(ring[1]).unwrap();
    let vertex_coedge = model
        .add_vertex_coedge(ring[1], pole, dummy_pcurve())
        .unwrap();
    assert_eq!(ring_len(&model, ring[0]), 5);

    // The new chord must go from `ring[0]`'s end vertex — `(1, 0, 0)` — to
    // `vertex_coedge`'s (degenerate) start vertex — `(2, 0, 0)`.
    let curve = line3((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
    let (edge_id, coedge_backward, coedge_forward) = model
        .mer(
            ring[0],
            vertex_coedge,
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            face_b,
        )
        .unwrap();
    assert_eq!(model.faces.len(), 2);
    assert_eq!(model.coedges.len(), 7);
    assert!(model.get_face(face_a).unwrap().holes.is_empty());
    assert!(model.get_face(face_b).unwrap().holes.is_empty());
    // face_b: ring[0], chord forward, vertex_coedge, ring[2], ring[3].
    assert_eq!(ring_len(&model, ring[0]), 5);
    // face_a: just ring[1] plus the chord's other coedge.
    assert_eq!(ring_len(&model, ring[1]), 2);

    model.ker(coedge_backward, coedge_forward).unwrap();
    assert_eq!(model.faces.len(), 2);
    assert!(model.get_face(face_a).unwrap().holes.is_empty());
    assert!(model.get_face(face_b).unwrap().holes.is_empty());
    assert_eq!(model.coedges.len(), 5);
    assert_eq!(model.edges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 5);
    let _ = edge_id;
}

// `mve`'s own `coedge` argument may itself be `Vertex`-backed (e.g. growing a
// fresh spoke straight off a pole's bridging coedge) — the new edge must
// start from that coedge's (degenerate) end vertex, and splicing must work
// the same way as for an ordinary edge-backed `coedge`.
fn check_mve_from_vertex_coedge_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 3, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    let pole = model.coedge_end_vertex_id(ring[0]).unwrap();
    let vertex_coedge = model
        .add_vertex_coedge(ring[0], pole, dummy_pcurve())
        .unwrap();
    assert_eq!(ring_len(&model, ring[0]), 4);

    // `vertex_coedge`'s end vertex is `(1, 0, 0)` (same as its start).
    let curve = line3((1.0, 0.0, 0.0), (5.0, 0.0, 0.0));
    let new_point = Vector3::from_array([S::from_f64(5.0), S::ZERO, S::ZERO]);
    let (v, coedge_out, coedge_in, _edge_id) = model
        .mve(
            vertex_coedge,
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            new_point,
        )
        .unwrap();
    assert_eq!(model.coedges.len(), 6);
    assert_eq!(ring_len(&model, ring[0]), 6);

    model.kve(coedge_out, coedge_in, v).unwrap();
    assert_eq!(model.vertices.len(), 3);
    assert_eq!(model.edges.len(), 3);
    assert_eq!(model.coedges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 4);
}

fn check_mekr_kemr_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring_a = make_ring(&mut model, face, 3, 0.0);
    let ring_b = make_ring(&mut model, face, 3, 10.0);
    {
        // Two rings on one face: the first bounds it, the second is a hole.
        let f = model.faces.get_mut(&face).unwrap();
        f.outer = BoundaryType::Loop(ring_a[0]);
        f.holes = vec![BoundaryType::Loop(ring_b[0])];
    }

    // `ring_a[0]`'s end vertex is at `(1, 0, 0)`; `ring_b[0]`'s start vertex
    // is at `(10, 0, 0)` — the new edge mekr's own convention now expects.
    let curve = line3((1.0, 0.0, 0.0), (10.0, 0.0, 0.0));
    let (_edge_id, coedge_a, coedge_b) = model
        .mekr(ring_a[0], ring_b[0], curve, dummy_pcurve())
        .unwrap();
    assert!(model.get_face(face).unwrap().holes.is_empty());
    assert_eq!(model.coedges.len(), 8);
    assert_eq!(ring_len(&model, ring_a[0]), 8);

    model.kemr(coedge_a, coedge_b).unwrap();
    assert_eq!(model.get_face(face).unwrap().holes.len(), 1);
    assert_eq!(model.coedges.len(), 6);
    assert_eq!(model.edges.len(), 6);
    assert_eq!(ring_len(&model, ring_a[0]), 3);
    assert_eq!(ring_len(&model, ring_b[0]), 3);
}

// Same shape as `check_mekr_kemr_round_trip`, but `coedge2` is a
// `Vertex`-backed coedge on `ring_b` rather than an ordinary edge-backed one.
fn check_mekr_from_vertex_coedge_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring_a = make_ring(&mut model, face, 3, 0.0);
    let ring_b = make_ring(&mut model, face, 3, 10.0);
    {
        // Two rings on one face: the first bounds it, the second is a hole.
        let f = model.faces.get_mut(&face).unwrap();
        f.outer = BoundaryType::Loop(ring_a[0]);
        f.holes = vec![BoundaryType::Loop(ring_b[0])];
    }

    // Splice a vertex-backed coedge in right before `ring_b[0]` (i.e. right
    // after `ring_b[2]`), sitting at `ring_b[0]`'s own start vertex.
    let pole = model.coedge_start_vertex_id(ring_b[0]).unwrap();
    let vertex_coedge = model
        .add_vertex_coedge(ring_b[2], pole, dummy_pcurve())
        .unwrap();
    assert_eq!(ring_len(&model, ring_b[0]), 4);

    // `ring_a[0]`'s end vertex is at `(1, 0, 0)`; `vertex_coedge`'s
    // (degenerate) start vertex is at `(10, 0, 0)`, same as `ring_b[0]`'s.
    let curve = line3((1.0, 0.0, 0.0), (10.0, 0.0, 0.0));
    let (_edge_id, coedge_a, coedge_b) = model
        .mekr(ring_a[0], vertex_coedge, curve, dummy_pcurve())
        .unwrap();
    assert!(model.get_face(face).unwrap().holes.is_empty());
    assert_eq!(model.coedges.len(), 9);
    assert_eq!(ring_len(&model, ring_a[0]), 9);

    model.kemr(coedge_a, coedge_b).unwrap();
    assert_eq!(model.get_face(face).unwrap().holes.len(), 1);
    assert_eq!(model.coedges.len(), 7);
    assert_eq!(model.edges.len(), 6);
    assert_eq!(ring_len(&model, ring_a[0]), 3);
    assert_eq!(ring_len(&model, ring_b[0]), 4);
}

fn check_mer_ker_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face_a = make_face(&mut model);
    let ring = make_ring(&mut model, face_a, 4, 0.0);
    model.faces.get_mut(&face_a).unwrap().outer = BoundaryType::Loop(ring[0]);
    let face_b = make_face(&mut model);

    // Same geometry as `check_mef_kef_round_trip`: `ring[0]`'s end vertex is
    // at `(1, 0, 0)`; `ring[2]`'s start vertex is at `(2, 0, 0)`.
    let curve = line3((1.0, 0.0, 0.0), (2.0, 0.0, 0.0));
    let (_edge_id, coedge_backward, coedge_forward) = model
        .mer(
            ring[0],
            ring[2],
            curve,
            dummy_pcurve(),
            dummy_pcurve().reverse(),
            face_b,
        )
        .unwrap();
    assert_eq!(model.faces.len(), 2);
    assert_eq!(model.coedges.len(), 6);
    assert!(model.get_face(face_a).unwrap().holes.is_empty());
    assert!(model.get_face(face_b).unwrap().holes.is_empty());
    // `coedge1` (`ring[0]`) moved onto face_b, together with `ring[2]` and
    // `ring[3]`, plus the new chord — 4 coedges; face_a keeps just `ring[1]`
    // plus the chord's other coedge — 2 coedges.
    assert_eq!(ring_len(&model, ring[0]), 4);
    assert_eq!(ring_len(&model, ring[1]), 2);

    model.ker(coedge_backward, coedge_forward).unwrap();
    assert_eq!(model.faces.len(), 2);
    assert!(model.get_face(face_a).unwrap().holes.is_empty());
    assert!(model.get_face(face_b).unwrap().holes.is_empty());
    assert_eq!(model.coedges.len(), 4);
    assert_eq!(model.edges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 4);
}

fn check_mvr_kvr_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 3, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    let point = Vector3::from_array([S::from_f64(5.0), S::ZERO, S::ZERO]);
    let vertex = model.mvr(face, point).unwrap();
    assert_eq!(model.vertices.len(), 4);
    assert_eq!(model.get_face(face).unwrap().holes.len(), 1);
    assert_eq!(
        model.get_face(face).unwrap().holes[0],
        BoundaryType::Vertex(vertex)
    );

    model.kvr(face, vertex).unwrap();
    assert_eq!(model.vertices.len(), 3);
    assert!(model.get_face(face).unwrap().holes.is_empty());
}

#[test]
fn mvfs_kvfs_round_trip() {
    for_all_scalars!(check_mvfs_kvfs_round_trip);
}

#[test]
fn mvr_kvr_round_trip() {
    for_all_scalars!(check_mvr_kvr_round_trip);
}

#[test]
fn mer_ker_round_trip() {
    for_all_scalars!(check_mer_ker_round_trip);
}

#[test]
fn mve_kve_round_trip() {
    for_all_scalars!(check_mve_kve_round_trip);
}

#[test]
fn mef_from_vertex_coedge_round_trip() {
    for_all_scalars!(check_mef_from_vertex_coedge_round_trip);
}

#[test]
fn mer_from_vertex_coedge_round_trip() {
    for_all_scalars!(check_mer_from_vertex_coedge_round_trip);
}

#[test]
fn mve_from_vertex_coedge_round_trip() {
    for_all_scalars!(check_mve_from_vertex_coedge_round_trip);
}

fn check_add_kill_vertex_coedge_round_trip<S: Scalar>() {
    let mut model = Model::<S>::new();
    let face = make_face(&mut model);
    let ring = make_ring(&mut model, face, 3, 0.0);
    model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(ring[0]);

    // No new edge or vertex at all — just one more coedge spliced into the
    // loop, sitting at `ring[0]`'s own end vertex.
    let pole = model.coedge_end_vertex_id(ring[0]).unwrap();
    let new_coedge = model
        .add_vertex_coedge(ring[0], pole, dummy_pcurve())
        .unwrap();
    assert!(matches!(
        model.get_coedge(new_coedge).unwrap().geometry,
        CoedgeGeometry::Vertex(v) if v == pole
    ));
    assert_eq!(model.vertices.len(), 3);
    assert_eq!(model.edges.len(), 3);
    assert_eq!(model.coedges.len(), 4);
    assert_eq!(ring_len(&model, ring[0]), 4);

    model.kill_vertex_coedge(new_coedge).unwrap();
    assert_eq!(model.vertices.len(), 3);
    assert_eq!(model.edges.len(), 3);
    assert_eq!(model.coedges.len(), 3);
    assert_eq!(ring_len(&model, ring[0]), 3);
}

#[test]
fn add_kill_vertex_coedge_round_trip() {
    for_all_scalars!(check_add_kill_vertex_coedge_round_trip);
}

#[test]
fn mekr_kemr_round_trip() {
    for_all_scalars!(check_mekr_kemr_round_trip);
}

#[test]
fn mekr_from_vertex_coedge_round_trip() {
    for_all_scalars!(check_mekr_from_vertex_coedge_round_trip);
}

#[test]
fn mef_kef_round_trip() {
    for_all_scalars!(check_mef_kef_round_trip);
}
