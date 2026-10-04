use geop_core_geometry::contains::curve::curve_could_contain;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_core_topology::{Body, EdgeId, Model, VertexId};
use geop_ops::Part;

use crate::naming::BooleanNaming;

/// The first `(edge_a, t, vertex_b)` split — `edge_a` from `solid_a`,
/// `vertex_b` a `solid_b` vertex lying on it at parameter `t` — if any.
/// Split out from `remesh_vertices_x_edges` for the same reason as
/// `remesh_vertices::find_coincident_vertex_pair`: `iter_body_edges`/
/// `iter_body_vertices` borrow `model`, so this search has to finish and
/// hand back plain ids before the caller is free to call
/// `split_edge_at_vertex` (`&mut self`).
fn find_edge_vertex_split<S: Scalar>(
    model: &Model<S>,
    solid_a: Body,
    solid_b: Body,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<(EdgeId, S, VertexId)>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "find_edge_vertex_split(solid_a={solid_a}, solid_b={solid_b}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    for edge_a in model.iter_body_edges(solid_a).with_context(&ctx)? {
        for vertex_b in model.iter_body_vertices(solid_b).with_context(&ctx)? {
            let edge_vertex_ctx =
                |e: GeopError| e.with_context(format!("edge_a={edge_a}, vertex_b={vertex_b}"));

            let Some(t) = curve_could_contain(
                &model.get_edge(edge_a).with_context(&ctx)?.curve,
                &model.get_vertex(vertex_b).with_context(&ctx)?.point,
                max_nodes,
                min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(&edge_vertex_ctx)?
            else {
                continue;
            };

            let edge = model.get_edge(edge_a).with_context(&ctx)?;
            let (start_vertex, end_vertex) = (edge.start_vertex, edge.end_vertex);
            let vertex_point = model.get_vertex(vertex_b).with_context(&ctx)?.point;
            let start_point = model.get_vertex(start_vertex).with_context(&ctx)?.point;
            let end_point = model.get_vertex(end_vertex).with_context(&ctx)?.point;

            // Coinciding with an endpoint's *point* while being a
            // *different* vertex id means `remesh_vertices` should already
            // have merged the two — that's the real bug this guards
            // against. Coinciding while genuinely *being* that endpoint
            // (same id) is the expected, healthy terminal state once it has
            // been merged — e.g. `iter_body_vertices(solid_b)` legitimately
            // re-surfaces a vertex shared by both solids — so that's just
            // "nothing to split here", not an error.
            if vertex_point.could_be_equal(&start_point) && vertex_b != start_vertex {
                return Err(ctx(edge_vertex_ctx(GeopError::new(format!(
                    "Assertion violated, vertex have to be remeshed before this function (vertex_b={vertex_b} coincides with start_vertex={start_vertex} of edge_a={edge_a} but is a different id, t={t})"
                )))));
            }
            if vertex_point.could_be_equal(&end_point) && vertex_b != end_vertex {
                return Err(ctx(edge_vertex_ctx(GeopError::new(format!(
                    "Assertion violated, vertex have to be remeshed before this function (vertex_b={vertex_b} coincides with end_vertex={end_vertex} of edge_a={edge_a} but is a different id, t={t})"
                )))));
            }
            if vertex_b == start_vertex || vertex_b == end_vertex {
                continue;
            }

            return Ok(Some((edge_a, t, vertex_b)));
        }
    }
    Ok(None)
}

pub fn remesh_vertices_x_edges<S: Scalar>(
    part: &mut Part<S>,
    naming: &mut BooleanNaming<S>,
    solid_a: Body,
    solid_b: Body,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "remesh_vertices_x_edges(solid_a={solid_a}, solid_b={solid_b}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
        ))
    };

    while let Some((edge_a, t, vertex_b)) = find_edge_vertex_split(
        part.topology(),
        solid_a,
        solid_b,
        max_nodes,
        min_subdivision_size,
    )
    .with_context(&ctx)?
    {
        let new_edge = part
            .split_edge_at_vertex(
                edge_a,
                t,
                vertex_b,
                max_nodes,
                min_subdivision_size,
                naming.provisional(),
            )
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "split_edge_at_vertex(edge_a={edge_a}, t={t}, vertex_b={vertex_b})"
                ))
            })
            .with_context(&ctx)?;
        naming.edge_split(edge_a, new_edge, vertex_b)?;
    }
    Ok(())
}
