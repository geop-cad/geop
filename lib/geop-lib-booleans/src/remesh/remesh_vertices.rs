use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    solvers::repeat::rewrite,
    with_context,
};
use geop_core_topology::{Body, Model, VertexId};
use geop_ops_model::NamedModel;

/// The first `(vertex_a, vertex_b)` pair — `vertex_a` from `solid_a`,
/// `vertex_b` from `solid_b`, distinct ids — whose points coincide, if any.
/// Split out from `remesh_vertices` because `iter_body_vertices` borrows
/// `model`: this search and any resulting `merge_vertex` call can't
/// interleave (the borrow checker won't allow a `&mut self` call while the
/// iterators from this function are still live), so the search has to run
/// to completion and hand back plain `VertexId`s before the caller is free
/// to mutate.
///
/// `settled` are the `solid_a` vertices already found to coincide with
/// none, which are not asked again, and every vertex found so is added to
/// it. That stays true while merging goes on: a merge deletes a `solid_b`
/// vertex and grows only the `solid_a` vertex it merges into, which was
/// found coinciding — so the search does not start over from every pair
/// after every merge.
fn find_coincident_vertex_pair(
    model: &Model,
    settled: &mut std::collections::HashSet<VertexId>,
    solid_a: Body,
    solid_b: Body,
) -> GeopResult<Option<(VertexId, VertexId)>> {
    let ctx = with_context!("find_coincident_vertex_pair(solid_a={solid_a}, solid_b={solid_b})");

    for vertex_a in model.iter_body_vertices(solid_a).with_context(&ctx)? {
        if settled.contains(&vertex_a) {
            continue;
        }
        for vertex_b in model.iter_body_vertices(solid_b).with_context(&ctx)? {
            if vertex_b == vertex_a {
                continue;
            }
            if model
                .get_vertex(vertex_a)
                .with_context(&ctx)?
                .point
                .could_be_equal(&model.get_vertex(vertex_b).with_context(&ctx)?.point)
            {
                return Ok(Some((vertex_a, vertex_b)));
            }
        }
        settled.insert(vertex_a);
    }
    Ok(None)
}

/// Merge every `solid_b` vertex into the `solid_a` vertex it coincides
/// with. Creates nothing, so needs no names: each survivor keeps its own.
pub fn remesh_vertices(model: &mut NamedModel, solid_a: Body, solid_b: Body) -> GeopResult<()> {
    let ctx = with_context!("remesh_vertices(solid_a={solid_a}, solid_b={solid_b})");

    // Every merge changes which vertices exist, so the search runs again
    // after each, skipping the vertices it settled (see
    // `find_coincident_vertex_pair`). Every merge removes one of the
    // vertices, so the searches are as many as there are vertices, and one
    // more that finds nothing.
    let vertices = model
        .topology()
        .iter_body_vertices(solid_a)
        .with_context(&ctx)?
        .count()
        + model
            .topology()
            .iter_body_vertices(solid_b)
            .with_context(&ctx)?
            .count();
    let mut settled = std::collections::HashSet::new();
    rewrite(
        model,
        vertices + 1,
        |model| find_coincident_vertex_pair(model.topology(), &mut settled, solid_a, solid_b),
        |model, (vertex_a, vertex_b)| {
            model
                .merge_vertex(vertex_a, vertex_b)
                .with_context(with_context!(
                    "merge_vertex(vertex_a={vertex_a}, vertex_b={vertex_b})"
                ))
        },
    )
    .with_context(&ctx)?
    .done("merging coincident vertices")
    .with_context(&ctx)?;
    Ok(())
}
