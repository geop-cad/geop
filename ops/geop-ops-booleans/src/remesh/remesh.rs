use geop_core_geometry::search;
use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use std::collections::HashMap;

use geop_core_topology::{Body, EdgeId, FaceId, VertexId};
use geop_ops::{NamedModel, Namer};

use crate::naming::BooleanNaming;
use crate::remesh::{
    remesh_edges_x_edges::remesh_edges_x_edges, remesh_edges_x_faces::remesh_edges_x_faces,
    remesh_tangent_branches::remesh_tangent_branches, remesh_vertices::remesh_vertices,
    remesh_vertices_x_edges::remesh_vertices_x_edges,
};

/// Every tunable [`remesh`] and its phases need. Bundled rather than passed
/// positionally: the phases don't share one tolerance (see
/// `curve_curve_min_subdivision_size`'s own note below), so the list is
/// long enough that positional arguments stop being readable at the call
/// site — and each field's meaning is documented once here instead of in
/// every signature it's threaded through.
///
/// Every field only affects *how hard a search tries* (budget, tolerance,
/// step length), never what a converged answer means — see `AGENTS.md`'s
/// "never use epsilons" rule.
#[derive(Clone, Copy, Debug)]
pub struct RemeshParams<S: Scalar> {
    /// Solution cap for the pairwise `curve_curve_intersect` /
    /// `curve_surface_intersect` searches: how many points one returns at
    /// most — its crossings, or, where the two coincide along an arc, that
    /// overlap's ends and points spread over it. The searches find overlaps
    /// directly, not by hitting the cap.
    pub max_edge_intersections: usize,
    /// Node budget for every subdivision search.
    pub max_nodes: usize,
    /// Convergence tolerance for the *single*-shape searches
    /// (`curve_could_contain` / `surface_could_contain`).
    pub min_subdivision_size: S,
    /// Convergence tolerance for the *pairwise* searches. Deliberately its
    /// own, looser value: empirically a pairwise search needs a much
    /// looser tolerance than a single-shape query to reliably converge
    /// within `max_nodes` at all — see `ValidationParameters`'s own doc
    /// comment.
    pub curve_curve_min_subdivision_size: S,
    /// How many marching steps one traced curve may take before it's
    /// considered to have failed to reach a terminating vertex. An effort
    /// budget only: a trace that reaches its vertex does so in the same
    /// steps whatever the budget. The steps scale with the surfaces traced
    /// between and their curvature, not with the model's units, so a curve
    /// takes about 16 steps across a flat face and 64 around a turn.
    pub max_trace_steps: usize,
}

impl<S: Scalar> Default for RemeshParams<S> {
    fn default() -> Self {
        Self {
            max_edge_intersections: 3,
            max_nodes: search::MAX_NODES,
            min_subdivision_size: search::min_subdivision_size(),
            curve_curve_min_subdivision_size: S::from_f64(1e-4),
            max_trace_steps: 1_000,
        }
    }
}

/// Split `edge` at its parameter `t`, where it crosses `face` at `point`:
/// at `vertex`, the vertex already there, or at a new one named as the
/// piercing of `face`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn split_at_crossing<S: Scalar>(
    model: &mut NamedModel<S>,
    naming: &mut BooleanNaming<S>,
    edge: EdgeId,
    t: S,
    vertex: Option<VertexId>,
    point: Vector3<S>,
    face: FaceId,
    params: &RemeshParams<S>,
) -> GeopResult<()> {
    let vertex = match vertex {
        Some(vertex) => vertex,
        None => {
            let vertex = model.insert_vertex(point, naming.provisional())?;
            naming.piercing(vertex, edge, t, face)?;
            vertex
        }
    };
    let new_edge = model
        .split_edge_at_vertex(
            edge,
            t,
            vertex,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
            naming.provisional(),
        )
        .with_context(with_context!(
            "edge={edge}, t={t:?}, vertex={vertex}, crossing face={face}"
        ))?;
    naming.edge_split(edge, new_edge, vertex)
}

/// Prepare `body_a` and `body_b`'s topology for a boolean operation or a
/// split: every coincidence between them (vertex/vertex, vertex/edge,
/// edge/edge, edge/face) is resolved into shared topology, so that
/// afterwards the two bodies meet only along entities they genuinely share.
/// Either may be a sheet: imprinting asks nothing of a body but its faces,
/// edges and vertices.
///
/// Runs in strictly increasing order of dimension — vertices, then edges
/// against vertices, then edges against edges, then edges against faces
/// (but for the branch points of tangent edges, which are vertices) —
/// so each phase can assume everything lower-dimensional has already
/// settled (e.g. `remesh_vertices_x_edges` treats a point-coincidence with
/// a *different* vertex id as a bug, since `remesh_vertices` should
/// already have merged them).
///
/// Everything it creates is named by `namer`, following
/// [`crate::naming`]'s scheme.
///
/// Returns what every face and edge of either body came from (see
/// [`Origins`]).
pub fn remesh<S: Scalar>(
    model: &mut NamedModel<S>,
    namer: &Namer,
    body_a: impl Into<Body>,
    body_b: impl Into<Body>,
    params: RemeshParams<S>,
) -> GeopResult<Origins> {
    let (body_a, body_b) = (body_a.into(), body_b.into());
    let ctx = with_context!("remesh(body_a={body_a}, body_b={body_b}, params={params:?})");

    let mut naming = BooleanNaming::new(model, namer, &[body_a, body_b]).with_context(&ctx)?;

    remesh_vertices(model, body_a, body_b).with_context(&ctx)?;

    // The one place an edge x face question creates a vertex ahead of the
    // edge phases: where an intersection branch leaves an edge along which
    // the two solids' faces touch tangentially (see
    // `remesh_tangent_branches`). Such an edge is usually one of a pair —
    // the shared profile, as each solid built it — so the new vertex lies on
    // the other solid's copy too, and `remesh_vertices_x_edges` below splits
    // that one like any other vertex on an edge.
    for (edge_solid, face_solid) in [(body_a, body_b), (body_b, body_a)] {
        remesh_tangent_branches(model, &mut naming, edge_solid, face_solid, &params)
            .with_context(&ctx)?;
    }

    // `remesh_vertices_x_edges` only splits `body_a`'s edges at `body_b`'s
    // vertices — it has no idea a `body_a` vertex might just as well be
    // sitting on one of `body_b`'s edges, so that direction needs its own,
    // separate call with the solids swapped. Vertex positions never change
    // (splitting only refines topology), so running direction A then
    // direction B can't reopen the other: a coincidence direction B finds
    // was already there when direction A ran and would have been found
    // then too, and vice versa.
    remesh_vertices_x_edges(model, &mut naming, body_a, body_b, &params).with_context(&ctx)?;
    remesh_vertices_x_edges(model, &mut naming, body_b, body_a, &params).with_context(&ctx)?;

    // Unlike the vertex-onto-edge step above, this one already checks every
    // `body_a` edge against every `body_b` edge directly (both directions
    // in one nested loop), so it doesn't need a second, swapped call.
    remesh_edges_x_edges(model, &mut naming, body_a, body_b, &params).with_context(&ctx)?;

    // Last, and highest-dimensional: every edge that pierces or lies within
    // one of the other solid's faces. Like `remesh_edges_x_edges` this
    // handles both directions internally (see its own doc comment).
    remesh_edges_x_faces(model, &mut naming, body_a, body_b, &params).with_context(&ctx)?;

    let origins = Origins {
        faces: naming.face_origins().clone(),
        edges: naming.edge_origins().clone(),
    };
    naming.finish(model).with_context(&ctx)?;
    Ok(origins)
}

/// What the faces and edges of two remeshed bodies came from: the name each
/// had before, or, for a piece split off one, the name that one had — its
/// *origin* (see [`crate::naming`]). Entities created otherwise, and since
/// deleted, may be missing or left over.
#[derive(Clone, Debug, Default)]
pub struct Origins {
    pub faces: HashMap<FaceId, String>,
    pub edges: HashMap<EdgeId, String>,
}
