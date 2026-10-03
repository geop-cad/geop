use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_core_topology::SolidId;
use geop_ops::{Namer, Part};

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
    /// `curve_surface_intersect` searches. Hitting it is how coincidence
    /// (an extended shared region, rather than finitely many crossings)
    /// is detected.
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
    /// Marching step length along a traced face x face intersection curve.
    pub trace_step_size: S,
    /// How many marching steps one traced curve may take before it's
    /// considered to have failed to reach a terminating vertex.
    pub max_trace_steps: usize,
}

impl<S: Scalar> Default for RemeshParams<S> {
    fn default() -> Self {
        Self {
            max_edge_intersections: 3,
            max_nodes: 20000,
            min_subdivision_size: S::from_f64(1e-7),
            curve_curve_min_subdivision_size: S::from_f64(1e-4),
            trace_step_size: S::from_f64(0.1),
            max_trace_steps: 200,
        }
    }
}

/// Prepare `solid_a` and `solid_b`'s topology for a boolean operation:
/// every coincidence between them (vertex/vertex, vertex/edge, edge/edge,
/// edge/face) is resolved into shared topology, so that afterwards the two
/// solids meet only along entities they genuinely share.
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
pub fn remesh<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid_a: SolidId,
    solid_b: SolidId,
    params: RemeshParams<S>,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "remesh(solid_a={solid_a}, solid_b={solid_b}, params={params:?})"
        ))
    };

    let mut naming = BooleanNaming::new(part, namer, &[solid_a, solid_b]).with_context(&ctx)?;

    remesh_vertices(part, solid_a, solid_b).with_context(&ctx)?;

    // The one place an edge x face question creates a vertex ahead of the
    // edge phases: where an intersection branch leaves an edge along which
    // the two solids' faces touch tangentially (see
    // `remesh_tangent_branches`). Such an edge is usually one of a pair —
    // the shared profile, as each solid built it — so the new vertex lies on
    // the other solid's copy too, and `remesh_vertices_x_edges` below splits
    // that one like any other vertex on an edge.
    for (edge_solid, face_solid) in [(solid_a, solid_b), (solid_b, solid_a)] {
        remesh_tangent_branches(
            part,
            &mut naming,
            edge_solid,
            face_solid,
            params.max_edge_intersections,
            params.max_nodes,
            params.curve_curve_min_subdivision_size,
        )
        .with_context(&ctx)?;
    }

    // `remesh_vertices_x_edges` only splits `solid_a`'s edges at `solid_b`'s
    // vertices — it has no idea a `solid_a` vertex might just as well be
    // sitting on one of `solid_b`'s edges, so that direction needs its own,
    // separate call with the solids swapped. Vertex positions never change
    // (splitting only refines topology), so running direction A then
    // direction B can't reopen the other: a coincidence direction B finds
    // was already there when direction A ran and would have been found
    // then too, and vice versa.
    remesh_vertices_x_edges(
        part,
        &mut naming,
        solid_a,
        solid_b,
        params.max_nodes,
        params.min_subdivision_size,
    )
    .with_context(&ctx)?;
    remesh_vertices_x_edges(
        part,
        &mut naming,
        solid_b,
        solid_a,
        params.max_nodes,
        params.min_subdivision_size,
    )
    .with_context(&ctx)?;

    // Unlike the vertex-onto-edge step above, this one already checks every
    // `solid_a` edge against every `solid_b` edge directly (both directions
    // in one nested loop), so it doesn't need a second, swapped call.
    remesh_edges_x_edges(
        part,
        &mut naming,
        solid_a,
        solid_b,
        params.max_edge_intersections,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
    )
    .with_context(&ctx)?;

    // Last, and highest-dimensional: every edge that pierces or lies within
    // one of the other solid's faces. Like `remesh_edges_x_edges` this
    // handles both directions internally (see its own doc comment).
    remesh_edges_x_faces(
        part,
        &mut naming,
        solid_a,
        solid_b,
        params.max_edge_intersections,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        params.trace_step_size,
        params.max_trace_steps,
    )
    .with_context(&ctx)?;

    naming.finish(part).with_context(&ctx)
}
