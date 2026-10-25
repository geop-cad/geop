use geop_core_geometry::intersection::{curve_curve_intersect, refine_curve_curve_crossing};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::repeat::rewrite,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{Body, EdgeId, Model, VertexId};
use geop_ops_model::NamedModel;

use crate::naming::BooleanNaming;
use crate::remesh::RemeshParams;

/// What to do next for a `solid_a` edge x `solid_b` edge pair — see
/// `find_edge_edge_action`'s own doc comment.
enum EdgeEdgeAction {
    /// The two edges are (at least partly) coincident and already share
    /// endpoints — `edge_deleted` folds into `edge_into`. `reversed` says
    /// whether `edge_deleted` ran start<->end the other way round.
    Merge {
        edge_into: EdgeId,
        edge_deleted: EdgeId,
        reversed: bool,
    },
    /// The two edges cross transversally at `point` — `edge_a`/`edge_b` each
    /// get split there at their own `t_a`/`t_b`, unless `split_a`/`split_b`
    /// is `false` (that edge already terminates there). `vertex` is an
    /// already-existing vertex to reuse at `point`, if one was found — e.g.
    /// a third edge crossing through the same point got there first —
    /// otherwise a new one needs to be created there.
    Split {
        edge_a: EdgeId,
        t_a: Scalar,
        split_a: bool,
        edge_b: EdgeId,
        t_b: Scalar,
        split_b: bool,
        vertex: Option<VertexId>,
        point: Vector3,
    },
}

/// A vertex of `bodies` whose point could be `point`, if any — so a crossing
/// shared by more than two edges (e.g. three edges meeting at one point)
/// reuses the single vertex the first pair created there instead of each
/// pair minting its own. Only the two bodies' own: a vertex of another
/// body, or of a wire, that happens to lie there is near, not theirs.
pub(super) fn find_vertex_at_point(
    model: &Model,
    bodies: [Body; 2],
    point: &Vector3,
) -> GeopResult<Option<VertexId>> {
    for body in bodies {
        for v in model.iter_body_vertices(body)? {
            if model.get_vertex(v)?.point.could_be_equal(point) {
                return Ok(Some(v));
            }
        }
    }
    Ok(None)
}

/// The first actionable `solid_a` edge x `solid_b` edge pair, if any — see
/// [`EdgeEdgeAction`]. Split out from `remesh_edges_x_edges` for the same
/// reason as `remesh_vertices::find_coincident_vertex_pair`:
/// `iter_body_edges` borrows `model`, so this search has to finish and hand
/// back plain data before the caller is free to mutate.
///
/// Coincidence is detected by vertex identity — the two edges already share
/// the same two endpoints (in either order) — checked *before* ever running
/// an intersection search. Vertex identity is cheap, exact, and (post
/// vertex/edge-vertex remesh) authoritative. Every found
/// crossing then just flows through the same per-solution split logic
/// below, whose own endpoint check no-ops correctly on a solution that
/// turns out to be a vertex the pair already shares.
///
/// The search runs to `params`' `curve_curve_min_subdivision_size`, not the
/// `min_subdivision_size` of the vertex-onto-edge searches (see
/// [`RemeshParams`]).
fn find_edge_edge_action(
    model: &Model,
    solid_a: Body,
    solid_b: Body,
    params: &RemeshParams,
) -> GeopResult<Option<EdgeEdgeAction>> {
    let ctx = with_context!(
        "find_edge_edge_action(solid_a={solid_a}, solid_b={solid_b}, params={params:?})"
    );

    for edge_a in model.iter_body_edges(solid_a).with_context(&ctx)? {
        for edge_b in model.iter_body_edges(solid_b).with_context(&ctx)? {
            // Already the same edge (e.g. an earlier merge this pass already
            // welded them together) — nothing to do.
            if edge_a == edge_b {
                continue;
            }

            let edge_ctx = with_context!("edge_a={edge_a}, edge_b={edge_b}");

            let ea = model.get_edge(edge_a).with_context(&ctx)?;
            let eb = model.get_edge(edge_b).with_context(&ctx)?;
            let (start_a, end_a) = (ea.start_vertex, ea.end_vertex);
            let (start_b, end_b) = (eb.start_vertex, eb.end_vertex);

            if start_a == start_b && end_a == end_b {
                return Ok(Some(EdgeEdgeAction::Merge {
                    edge_into: edge_a,
                    edge_deleted: edge_b,
                    reversed: false,
                }));
            }
            if start_a == end_b && end_a == start_b {
                return Ok(Some(EdgeEdgeAction::Merge {
                    edge_into: edge_a,
                    edge_deleted: edge_b,
                    reversed: true,
                }));
            }

            let intersections = curve_curve_intersect(
                &model.get_edge(edge_a)?.curve,
                &model.get_edge(edge_b)?.curve,
                params.max_edge_intersections,
                params.max_nodes,
                params.curve_curve_min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(&edge_ctx)?;

            for (t_a, t_b) in intersections.into_vec() {
                // These become *split* parameters, and the vertex point is
                // evaluated from them — so their width matters here in a way
                // it does not for a caller that only counts crossings. A
                // `min_subdivision_size`-wide parameter cannot be fed to
                // `NurbCurve::split`, and the point evaluated at one is just
                // as wide, which makes a fat vertex and then a fat sub-curve.
                let (t_a, t_b) = refine_curve_curve_crossing(&ea.curve, &eb.curve, t_a, t_b);
                let point_a = ea
                    .curve
                    .evaluate(t_a)
                    .with_context(&ctx)
                    .with_context(&edge_ctx)?;
                let point_b = eb
                    .curve
                    .evaluate(t_b)
                    .with_context(&ctx)
                    .with_context(&edge_ctx)?;
                // A crossing is a point on both curves. The search encloses
                // one, and the refinement only tightens, so two evaluations
                // that do not overlap are a search that reported a crossing
                // where there is none: an error naming the edges, never a
                // vertex. Their union is the honest enclosure of the point
                // (see `AGENTS.md`, "Combine two enclosures ... with `union`").
                if !point_a.could_be_equal(&point_b) {
                    return Err(GeopError::new(format!(
                        "edge x edge crossing reported at t_a={t_a:?}, t_b={t_b:?}, but the curves evaluate to disjoint points there: {point_a:?} on edge_a, {point_b:?} on edge_b"
                    )))
                    .with_context(&ctx)
                    .with_context(&edge_ctx);
                }
                let point = point_a.union(&point_b);

                let start_a_pt = model.get_vertex(ea.start_vertex).with_context(&ctx)?.point;
                let end_a_pt = model.get_vertex(ea.end_vertex).with_context(&ctx)?.point;
                let start_b_pt = model.get_vertex(eb.start_vertex).with_context(&ctx)?.point;
                let end_b_pt = model.get_vertex(eb.end_vertex).with_context(&ctx)?.point;

                let split_a =
                    !point.could_be_equal(&start_a_pt) && !point.could_be_equal(&end_a_pt);
                let split_b =
                    !point.could_be_equal(&start_b_pt) && !point.could_be_equal(&end_b_pt);

                // Both edges already terminate here (e.g. a shared corner) —
                // nothing to split.
                if !split_a && !split_b {
                    continue;
                }

                let vertex =
                    find_vertex_at_point(model, [solid_a, solid_b], &point).with_context(&ctx)?;

                return Ok(Some(EdgeEdgeAction::Split {
                    edge_a,
                    t_a,
                    split_a,
                    edge_b,
                    t_b,
                    split_b,
                    vertex,
                    point,
                }));
            }
        }
    }
    Ok(None)
}

pub fn remesh_edges_x_edges(
    model: &mut NamedModel,
    naming: &mut BooleanNaming,
    solid_a: Body,
    solid_b: Body,
    params: &RemeshParams,
) -> GeopResult<()> {
    let ctx = with_context!(
        "remesh_edges_x_edges(solid_a={solid_a}, solid_b={solid_b}, params={params:?})"
    );

    rewrite(
        model,
        params.max_rewrites,
        |model| find_edge_edge_action(model.topology(), solid_a, solid_b, params),
        |model, action| {
            match action {
                EdgeEdgeAction::Merge {
                    edge_into,
                    edge_deleted,
                    reversed,
                } => {
                    model.merge_edge(edge_into, edge_deleted, reversed)
                        .with_context(with_context!("merge_edge(edge_into={edge_into}, edge_deleted={edge_deleted}, reversed={reversed})"))
                        .with_context(&ctx)?;
                }
                EdgeEdgeAction::Split {
                    edge_a,
                    t_a,
                    split_a,
                    edge_b,
                    t_b,
                    split_b,
                    vertex,
                    point,
                } => {
                    let name_a = model.name_of(edge_a).unwrap_or("?").to_string();
                    let name_b = model.name_of(edge_b).unwrap_or("?").to_string();
                    let split_ctx = with_context!(
                        "edge_a={edge_a} ({name_a}), t_a={t_a:?}, edge_b={edge_b} ({name_b}), t_b={t_b:?}, point={point:?}"
                    );

                    let vertex_id = match vertex {
                        Some(v) => v,
                        None => {
                            // `find_vertex_at_point` ruled out `point` being
                            // `could_be_equal` to any existing vertex of the
                            // two bodies, and `find_edge_edge_action` checked
                            // that both curves pass through it: a vertex here
                            // is new. How far it is from its neighbours is
                            // the geometry's business, not ours. A bolt
                            // hole's cap seam crossing the flange cap's seam
                            // 7e-4 from the hole's centre is a genuine
                            // crossing, and an edge piece 7e-4 long is
                            // legitimate topology.
                            let vertex = model
                                .insert_vertex(point, naming.provisional())
                                .with_context(&split_ctx)
                                .with_context(&ctx)?;
                            naming.edge_crossing(vertex, (edge_a, t_a), (edge_b, t_b))?;
                            vertex
                        }
                    };

                    // `point` (and `t_a`/`t_b`) is only ever as precise as
                    // `curve_curve_intersect`'s own convergence — asking
                    // `split_edge_at_vertex` to relocate it on a coedge's
                    // surface/pcurve with a *tighter* tolerance than that
                    // wouldn't add real precision, just search effort (or
                    // outright non-convergence) for nothing, so it gets the same
                    // `curve_curve_min_subdivision_size` this whole search
                    // already used to find it.
                    for (split, edge, t) in [(split_a, edge_a, t_a), (split_b, edge_b, t_b)] {
                        if !split {
                            continue;
                        }
                        let new_edge = model
                            .split_edge_at_vertex(
                                edge,
                                t,
                                vertex_id,
                                params.max_nodes,
                                params.curve_curve_min_subdivision_size,
                                naming.provisional(),
                            )
                            .with_context(&split_ctx)
                            .with_context(&ctx)?;
                        naming.edge_split(edge, new_edge, vertex_id)?;
                    }
                }
            }
            Ok(())
        },
    )
    .with_context(&ctx)?
    .done("splitting edges where they cross")
    .with_context(&ctx)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::scenes::remeshed;

    /// Regression test for a `curve_curve_intersect` bug where two edges
    /// meeting at exactly one shared vertex (here: a genuine, single,
    /// well-converged crossing created by an earlier split in this same
    /// fixed-point loop) could blow up the search on the very next
    /// iteration when it re-paired those same two now-shrunk edges — the
    /// search would fail to subdivide one side at all, reporting a `t`
    /// interval spanning that side's *entire* domain, wide enough to
    /// extrapolate past the curve's own boundary and crash `evaluate` with
    /// a zero/negative weight. Root-caused to `NurbCurve::split`'s Boehm
    /// knot insertion: repeated splits let a new control point's interval
    /// width compound across generations (fixed by `Scalar::interpolate`,
    /// which collapses exactly when two control points coincide, e.g. a
    /// straight edge's weight staying exactly `1.0`), and, more subtly, let
    /// unavoidable division-rounding noise in `alpha` compound as
    /// `denom` (the knot span being divided into) kept shrinking every
    /// split (fixed by sharpening `t` before `split` and `alpha` right
    /// after computing it — both are fully determined by already-sharp
    /// inputs, so any interval width they pick up is pure representation
    /// noise, not real uncertainty, and is safe to collapse away rather
    /// than let compound).
    #[test]
    fn figure8_quarter_overlap_hole_corner_remesh_succeeds() {
        remeshed("figure8_cylinder_quarter_overlap_hole_corner");
    }
}
