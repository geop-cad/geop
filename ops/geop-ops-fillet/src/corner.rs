//! Vertex blends: rounding a corner every edge of which is filleted, the
//! way a box with all its edges rounded has its corners rounded.
//!
//! Where three faces meet at a corner and all three edges between them are
//! filleted with one radius `r`, the three fillets end at the ball of that
//! radius touching all three faces — behind them at a convex corner, in
//! front at a concave one — and the ball's piece between them rounds the
//! corner. Its center `C` is `r` from each face, its contacts `P_k` are the
//! feet of `C` on them, and each fillet meets it in a great arc: the
//! fillet's section through `C`, from the contact on one of its faces to
//! the contact on the other. So each edge's tool is cut off square to the
//! edge through `C` — set back from the corner — and the corner's piece of
//! the ball is the spherical triangle between the three arcs.
//!
//! The tools are joined into one at the corner (see
//! [`crate::tool::build_tools`]): the spherical triangle closes them off
//! towards the ball, and on the far side the three tools' apexes are joined
//! to the corner's own apex, `2V - C` for the corner `V`, by a patch round
//! each contact — in front of that contact's face, so outside the solid at
//! a convex corner and inside at a concave one, where the boolean cuts or
//! fills nothing but the region between the faces and the ball.
//!
//! Only corners of three planes with straight edges are rounded: there the
//! fillets are swept, and their sections square to the edges are the ball's
//! great arcs exactly. A corner where every edge is filleted but some edge
//! is not straight, or more faces meet, or the edges bend different ways —
//! a pocket's rim, its upright edge filled in where the rim is cut away —
//! is refused by name rather than left with the fillets crossing in a point
//! or overlapping. A chamfered corner is left to its chamfers, which cross
//! in a point.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{EdgeId, FaceId, VertexId};
use geop_ops::{Namer, Part};

use crate::{
    blend::{BlendShape, End, Plan, Sweep, ToolProfile, check_touch},
    rolling::Rolled,
    tool::{Corner, CornerEnd},
};

/// Plans the corner blends of `plans` and `rolled`, the blends of the edges
/// `covered`, into `shape`, named by `namer` (see the module docs): every
/// corner every edge of which is filleted, the same way, each with its
/// tools' ends — by index into `plans` — and those ends set to it.
pub(crate) fn plan_corners<S: Scalar>(
    part: &Part<S>,
    namer: &Namer,
    plans: &mut [(Plan<S>, ToolProfile<S>)],
    rolled: &[(String, Rolled<S>)],
    covered: &[EdgeId],
    shape: &BlendShape,
) -> GeopResult<Vec<Corner<S>>> {
    let BlendShape::Fillet { radii } = shape else {
        return Ok(Vec::new());
    };
    let model = part.topology();
    let mut vertices: Vec<VertexId> = Vec::new();
    for &edge in covered {
        let e = model.get_edge(edge)?;
        for v in [e.start_vertex, e.end_vertex] {
            if !vertices.contains(&v) {
                vertices.push(v);
            }
        }
    }
    let mut corners = Vec::new();
    for vertex in vertices {
        let incident: Vec<EdgeId> = model
            .edges
            .iter()
            .filter(|(_, e)| e.start_vertex == vertex || e.end_vertex == vertex)
            .map(|(&id, _)| id)
            .collect();
        // A vertex inside a tangent chain, or one not every edge of which
        // is blended, is no corner to round.
        if incident.len() < 3 || !incident.iter().all(|e| covered.contains(e)) {
            continue;
        }
        let name = part.name_of(vertex).unwrap_or("?").to_string();
        let ctx = with_context!("rounding the corner at vertex {name:?}");
        // Which tool blends each edge, and which way it bends.
        let mut swept = Vec::new();
        let mut bends = Vec::new();
        let mut unsupported = None;
        for &edge in &incident {
            let found = plans.iter().position(|(plan, _)| {
                part.edge_id(&plan.edge).ok() == Some(edge)
                    && matches!(plan.section.sweep, Sweep::Straight { .. })
            });
            match found {
                Some(i) => {
                    bends.push(plans[i].0.section.bend);
                    swept.push(i);
                }
                None => {
                    let r = rolled.iter().find(|(_, r)| r.chain.edges().contains(&edge));
                    if let Some((_, r)) = r {
                        bends.push(r.bend);
                    }
                    unsupported = Some(part.name_of(edge).unwrap_or("?").to_string());
                }
            }
        }
        if bends.iter().any(|b| *b != bends[0]) {
            return Err(GeopError::new(
                "every edge at the corner is filleted, but some are convex and some concave: a corner is only rounded where all its edges bend the same way",
            ))
            .with_context(ctx);
        }
        if let Some(edge) = unsupported {
            return Err(GeopError::new(format!(
                "every edge at the corner is filleted, but edge {edge:?} is not straight between two planes: a corner is only rounded where three straight edges between planes meet"
            )))
            .with_context(ctx);
        }
        if incident.len() != 3 {
            return Err(GeopError::new(format!(
                "every edge at the corner is filleted, but {} edges meet there: a corner is only rounded where three meet",
                incident.len()
            )))
            .with_context(ctx);
        }
        if !radii.is_constant() {
            return Err(GeopError::new(
                "every edge at the corner is filleted with a radius that varies: a corner is only rounded where the radius is the same all along",
            ))
            .with_context(ctx);
        }
        let corner = plan_corner(part, namer, vertex, &name, plans, &swept, radii.radius)
            .with_context(ctx)?;
        corners.push(corner);
    }
    Ok(corners)
}

/// The corner at `vertex`, named `name`, where the straight edges of
/// `plans[swept]` meet, filleted with `radius` (see the module docs): its
/// ball, contacts and apex, and its tools' ends — set to it in `plans`.
fn plan_corner<S: Scalar>(
    part: &Part<S>,
    namer: &Namer,
    vertex: VertexId,
    name: &str,
    plans: &mut [(Plan<S>, ToolProfile<S>)],
    swept: &[usize],
    radius: f64,
) -> GeopResult<Corner<S>> {
    let model = part.topology();
    let at = model.get_vertex(vertex)?.point;
    let mut faces: Vec<FaceId> = Vec::new();
    for &i in swept {
        for face in plans[i].0.section.faces {
            if !faces.contains(&face) {
                faces.push(face);
            }
        }
    }
    let mut normals = Vec::new();
    for &face in &faces {
        let Some(plane) = model.get_face(face)?.surface.as_plane()? else {
            return Err(GeopError::new(format!(
                "face {face} is not planar: a corner is only rounded where three planes meet"
            )));
        };
        normals.push(plane.normal.normalize()?);
    }
    let [n0, n1, n2] = normals[..] else {
        return Err(GeopError::new(format!(
            "{} faces meet at the corner: a corner is only rounded where three meet",
            faces.len()
        )));
    };
    // `r` from each face on the side the fillets' balls are, as theirs.
    let r = S::from_f64(radius);
    let sr = plans[swept[0]].0.section.bend.side::<S>().mul(r);
    let offset = solve_linear_system(
        &Matrix::from_rows([n0.to_array(), n1.to_array(), n2.to_array()]),
        &Vector3::from_array([sr, sr, sr]),
    )
    .map_err(|e| e.with_context("the corner's faces could meet along a line"))?;
    let center = at.add(&offset);
    let contacts: Vec<Vector3<S>> = normals
        .iter()
        .map(|n| center.sub(&n.prod_scalar(sr)))
        .collect();
    for (face, p) in faces.iter().zip(&contacts) {
        let ctx = with_context!("where the corner's ball touches face {face}, at {p:?}");
        check_touch(model, *face, p).with_context(ctx)?;
    }

    let mut ends = Vec::new();
    for &i in swept {
        let (plan, _) = &mut plans[i];
        let ctx = with_context!("edge {:?}", plan.edge);
        let Sweep::Straight {
            start,
            end,
            ends: tool_ends,
            vertices,
            ..
        } = &mut plan.section.sweep
        else {
            unreachable!("only straight edges are planned into corners");
        };
        let length = end.sub(start).norm();
        let setback = center.sub(start).prod_dot(&end.sub(start).normalize()?);
        let at_end = vertices[1] == vertex;
        let k = usize::from(at_end);
        // The corner's ball touches the edge's faces within the edge, short
        // of the other end — and of the other end's own corner.
        let other = match &tool_ends[1 - k] {
            End::Corner { setback } => *setback,
            _ if at_end => S::ZERO,
            _ => length,
        };
        let within = if at_end {
            setback.definitely_greater(other) && setback.definitely_less(length)
        } else {
            setback.definitely_greater(S::ZERO) && setback.definitely_less(other)
        };
        if !within {
            return Err(GeopError::new(format!(
                "the fillet is too large for the edge: the ball rounding the corner at vertex {name:?} touches its faces {setback:?} along it, past its other end or that end's corner"
            )))
            .with_context(ctx);
        }
        tool_ends[k] = End::Corner { setback };
        let index = |face: FaceId| {
            faces
                .iter()
                .position(|&f| f == face)
                .expect("a corner face")
        };
        ends.push(CornerEnd {
            tool: i,
            at_end,
            contacts: plan.section.faces.map(index),
        });
    }
    Ok(Corner {
        namer: namer.scoped(name),
        center,
        radius: r,
        contacts,
        apex: at.prod_scalar(S::TWO).sub(&center),
        inside: at,
        ends,
    })
}
