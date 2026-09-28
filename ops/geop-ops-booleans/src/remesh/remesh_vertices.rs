use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};
use geop_ops::Part;
use geop_core_topology::{Model, SolidId, VertexId};

/// The first `(vertex_a, vertex_b)` pair — `vertex_a` from `solid_a`,
/// `vertex_b` from `solid_b`, distinct ids — whose points coincide, if any.
/// Split out from `remesh_vertices` because `iter_solid_vertices` borrows
/// `model`: this search and any resulting `merge_vertex` call can't
/// interleave (the borrow checker won't allow a `&mut self` call while the
/// iterators from this function are still live), so the search has to run
/// to completion and hand back plain `VertexId`s before the caller is free
/// to mutate.
fn find_coincident_vertex_pair<S: Scalar>(
    model: &Model<S>,
    solid_a: SolidId,
    solid_b: SolidId,
) -> GeopResult<Option<(VertexId, VertexId)>> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "find_coincident_vertex_pair(solid_a={solid_a}, solid_b={solid_b})"
        ))
    };

    for vertex_a in model.iter_solid_vertices(solid_a).with_context(&ctx)? {
        for vertex_b in model.iter_solid_vertices(solid_b).with_context(&ctx)? {
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
    }
    Ok(None)
}

/// Merge every `solid_b` vertex into the `solid_a` vertex it coincides
/// with. Creates nothing, so needs no names: each survivor keeps its own.
pub fn remesh_vertices<S: Scalar>(
    part: &mut Part<S>,
    solid_a: SolidId,
    solid_b: SolidId,
) -> GeopResult<()> {
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "remesh_vertices(solid_a={solid_a}, solid_b={solid_b})"
        ))
    };

    // Every merge changes which vertices exist and which coincide with which
    // (a vertex not yet visited this pass may now already be welded to one
    // visited earlier), so each one invalidates the search that found it —
    // the loop below always re-searches from scratch afterwards rather than
    // pressing on with anything from before the merge.
    while let Some((vertex_a, vertex_b)) =
        find_coincident_vertex_pair(part.topology(), solid_a, solid_b).with_context(&ctx)?
    {
        part.merge_vertex(vertex_a, vertex_b)
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "merge_vertex(vertex_a={vertex_a}, vertex_b={vertex_b})"
                ))
            })?;
    }
    Ok(())
}
