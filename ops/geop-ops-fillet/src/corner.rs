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
//! the contact on the other. So each edge's tool ends at its section
//! through `C` — set back from the corner — and the corner's piece of the
//! ball is the spherical triangle between the three arcs.
//!
//! A straight edge between planes is swept, and its section through `C` is
//! the one square to the edge, there. Any other edge is rolled along (see
//! [`crate::rolling`]), up to a station built from the corner's ball
//! exactly: its arc the ball's great arc, not tilted off the faces as the
//! stations between are.
//!
//! The tools are joined into one at the corner (see
//! [`crate::tool::build_tools`]): the spherical triangle closes them off
//! towards the ball, and on the far side the three tools' apexes are joined
//! to the corner's own apex, `2V - C` for the corner `V`, by a patch round
//! each contact — in front of that contact's face, so outside the solid at
//! a convex corner and inside at a concave one, where the boolean cuts or
//! fills nothing but the region between the faces and the ball.
//!
//! Where the radius varies along the edges, they have to meet the corner
//! with one radius — the corner's vertex given a radius of its own, say —
//! the ball's, which each fillet then reaches where it meets the ball.
//!
//! A corner where every edge is filleted but more than three meet, or the
//! fillets meet with different radii, or the edges bend different ways — a
//! pocket's rim, its upright edge filled in where the rim is cut away — is
//! refused by name rather than left with the fillets crossing in a point
//! or overlapping. A chamfered corner is left to its chamfers, which cross
//! in a point.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{EdgeId, FaceId, Model, VertexId};
use geop_ops::{Namer, Part};

use crate::{
    blend::{Bend, BlendShape, End, Plan, Sweep, ToolProfile, check_touch},
    rolling::{Chain, ChainCorner, RadiusLaw, chain_bend, foot, normal_at, seed_on, sharp_in},
    tool::{Corner, CornerEnd},
};

/// Newton iterations placing a corner's ball against curved faces.
const NEWTON_ITERATIONS: usize = 30;

/// The corners every edge of which is filleted, planned (see
/// [`plan_corners`]): each with its tools' ends — by index into the plans,
/// and past them into the chains — and, per chain, the corners' balls at its
/// start and end.
pub(crate) struct Corners<S: Scalar> {
    pub corners: Vec<Corner<S>>,
    pub chains: Vec<[Option<ChainCorner<S>>; 2]>,
}

/// How a tool reaches a corner: a straight swept edge, by its index into
/// the plans, or a chain, by its index into the chains and which end of it.
#[derive(Clone, Copy)]
enum Reaching {
    Swept(usize),
    Chain(usize, bool),
}

/// Plans the corner blends of the straight edges of `plans` and of
/// `chains`, the blends of the edges `covered`, into `shape`, named by
/// `namer` (see the module docs): every corner every edge of which is
/// filleted. The plans' ends there are set to it.
pub(crate) fn plan_corners<S: Scalar>(
    part: &Part<S>,
    namer: &Namer,
    plans: &mut [(Plan<S>, ToolProfile<S>)],
    chains: &[(String, Chain)],
    covered: &[EdgeId],
    shape: &BlendShape,
) -> GeopResult<Corners<S>> {
    let mut planned = Corners {
        corners: Vec::new(),
        chains: vec![[None, None]; chains.len()],
    };
    let BlendShape::Fillet { radii } = shape else {
        return Ok(planned);
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
        let mut reaching = Vec::new();
        let mut bends = Vec::new();
        for &edge in &incident {
            let edge_name = part.name_of(edge).unwrap_or("?");
            let swept = plans.iter().position(|(plan, _)| {
                part.edge_id(&plan.edge).ok() == Some(edge)
                    && matches!(plan.section.sweep, Sweep::Straight { .. })
            });
            if let Some(i) = swept {
                bends.push(plans[i].0.section.bend);
                reaching.push(Reaching::Swept(i));
                continue;
            }
            let Some(k) = chains.iter().position(|(_, c)| c.edges().contains(&edge)) else {
                return Err(GeopError::new(format!(
                    "every edge at the corner is filleted, but edge {edge_name:?} is blended round a whole circle: a corner is only rounded where the fillets end at it"
                )))
                .with_context(ctx);
            };
            let chain = &chains[k].1;
            let ends = chain.vertices(model)?;
            let at_end = if chain.closed {
                None
            } else if ends[0] == vertex && chain.links[0].edge == edge {
                Some(false)
            } else if ends.last() == Some(&vertex)
                && chain.links.last().map(|l| l.edge) == Some(edge)
            {
                Some(true)
            } else {
                None
            };
            let Some(at_end) = at_end else {
                return Err(GeopError::new(format!(
                    "every edge at the corner is filleted, but the fillet of edge {edge_name:?} runs on through it: a corner is only rounded where the fillets end at it"
                )))
                .with_context(ctx);
            };
            bends.push(chain_bend(model, chain).with_context(ctx)?);
            reaching.push(Reaching::Chain(k, at_end));
        }
        if bends.iter().any(|b| *b != bends[0]) {
            return Err(GeopError::new(
                "every edge at the corner is filleted, but some are convex and some concave: a corner is only rounded where all its edges bend the same way",
            ))
            .with_context(ctx);
        }
        if incident.len() != 3 {
            return Err(GeopError::new(format!(
                "every edge at the corner is filleted, but {} edges meet there: a corner is only rounded where three meet",
                incident.len()
            )))
            .with_context(ctx);
        }
        // The fillets' radii at the corner: the ball's, where they agree.
        let mut at_corner = Vec::new();
        for &reach in &reaching {
            at_corner.push(match reach {
                Reaching::Swept(_) => radii.radius,
                Reaching::Chain(k, at_end) => RadiusLaw::new(part, &chains[k].1, radii)
                    .with_context(ctx)?
                    .at_end(usize::from(at_end)),
            });
        }
        if at_corner.iter().any(|&r| r != at_corner[0]) {
            return Err(GeopError::new(format!(
                "every edge at the corner is filleted, but with the radii {at_corner:?} there: a corner is only rounded where its fillets meet with one radius — give the corner's vertex a radius of its own"
            )))
            .with_context(ctx);
        }
        let corner = plan_corner(
            part,
            namer,
            vertex,
            &name,
            plans,
            chains,
            &reaching,
            bends[0],
            at_corner[0],
            &mut planned.chains,
        )
        .with_context(ctx)?;
        planned.corners.push(corner);
    }
    Ok(planned)
}

/// The faces either side of the tool reaching a corner as `reaching` does.
fn faces_of<S: Scalar>(
    plans: &[(Plan<S>, ToolProfile<S>)],
    chains: &[(String, Chain)],
    reaching: Reaching,
) -> [FaceId; 2] {
    match reaching {
        Reaching::Swept(i) => plans[i].0.section.faces,
        Reaching::Chain(k, at_end) => {
            let links = &chains[k].1.links;
            let link = if at_end { links.last() } else { links.first() };
            link.expect("a chain has links").faces
        }
    }
}

/// The corner at `vertex`, named `name`, where the tools `reaching` it
/// meet, filleted with `radius`, bending `bend` (see the module docs): its
/// ball, contacts and apex, and its tools' ends — set to it in `plans`, and
/// in `chain_corners` for the chains.
#[allow(clippy::too_many_arguments)]
fn plan_corner<S: Scalar>(
    part: &Part<S>,
    namer: &Namer,
    vertex: VertexId,
    name: &str,
    plans: &mut [(Plan<S>, ToolProfile<S>)],
    chains: &[(String, Chain)],
    reaching: &[Reaching],
    bend: Bend,
    radius: f64,
    chain_corners: &mut [[Option<ChainCorner<S>>; 2]],
) -> GeopResult<Corner<S>> {
    let model = part.topology();
    let at = model.get_vertex(vertex)?.point;
    let mut faces: Vec<FaceId> = Vec::new();
    for &r in reaching {
        for face in faces_of(plans, chains, r) {
            if !faces.contains(&face) {
                faces.push(face);
            }
        }
    }
    let faces: [FaceId; 3] = faces.try_into().map_err(|faces: Vec<FaceId>| {
        GeopError::new(format!(
            "{} faces meet at the corner: a corner is only rounded where three meet",
            faces.len()
        ))
    })?;
    let r = S::from_f64(radius);
    let (center, contacts) = ball(model, faces, &at, bend.side::<S>().mul(r))?;
    for (face, p) in faces.iter().zip(&contacts) {
        let ctx = with_context!("where the corner's ball touches face {face}, at {p:?}");
        check_touch(model, *face, p).with_context(ctx)?;
    }
    let contact_on = |face: FaceId| {
        contacts[faces
            .iter()
            .position(|&f| f == face)
            .expect("a corner face")]
    };

    let mut ends = Vec::new();
    for &reach in reaching {
        match reach {
            Reaching::Swept(i) => {
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
                    unreachable!("only straight edges are swept into corners");
                };
                let length = end.sub(start).norm();
                let setback = center.sub(start).prod_dot(&end.sub(start).normalize()?);
                let at_end = vertices[1] == vertex;
                let k = usize::from(at_end);
                // The corner's ball touches the edge's faces within the
                // edge, short of the other end — and of its own corner.
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
                ends.push(CornerEnd {
                    tool: i,
                    at_end,
                    contacts: [0, 0],
                });
            }
            Reaching::Chain(k, at_end) => {
                let link_faces = faces_of(plans, chains, reach);
                chain_corners[k][usize::from(at_end)] = Some(ChainCorner {
                    center,
                    contacts: link_faces.map(contact_on),
                });
                ends.push(CornerEnd {
                    tool: plans.len() + k,
                    at_end,
                    contacts: [0, 0],
                });
            }
        }
    }
    Ok(Corner {
        namer: namer.scoped(name),
        center,
        radius: r,
        faces: faces.to_vec(),
        contacts: contacts.to_vec(),
        apex: at.prod_scalar(S::TWO).sub(&center),
        inside: at,
        ends,
    })
}

/// The center of the ball `sr` from each of `faces` along its outward
/// normal — `r` behind them for a negative `sr` — near their corner `at`,
/// and where it touches each.
///
/// Three planes give it exactly, by one linear solve. Against curved faces
/// it is Newton's method, from the planes tangent to the faces at the
/// corner: each step the center `sr` from the planes tangent at its feet,
/// every iterate sharpened — the center is where the ball is chosen to be,
/// and its contacts are evaluated on the faces, so they lie on them however
/// exactly it was found. Checked against every face at the end.
fn ball<S: Scalar>(
    model: &Model<S>,
    faces: [FaceId; 3],
    at: &Vector3<S>,
    sr: S,
) -> GeopResult<(Vector3<S>, [Vector3<S>; 3])> {
    let solve = |normals: [Vector3<S>; 3], offsets: [S; 3]| -> GeopResult<Vector3<S>> {
        solve_linear_system(
            &Matrix::from_rows(normals.map(|n| n.to_array())),
            &Vector3::from_array(offsets),
        )
        .map_err(|e| e.with_context("the corner's faces could meet along a line"))
    };
    let mut planes = Vec::new();
    for face in faces {
        planes.push(model.get_face(face)?.surface.as_plane()?);
    }
    if let [Some(a), Some(b), Some(c)] = &planes[..] {
        let normals = [
            a.normal.normalize()?,
            b.normal.normalize()?,
            c.normal.normalize()?,
        ];
        let center = at.add(&solve(normals, [sr; 3])?);
        return Ok((center, normals.map(|n| center.sub(&n.prod_scalar(sr)))));
    }
    let surfaces = faces.map(|f| &model.get_face(f).expect("a face").surface);
    let mut normals = [Vector3::zero(); 3];
    for k in 0..3 {
        normals[k] = normal_at(model, faces[k], at)?;
    }
    let mut center = at.add(&solve(normals, [sr; 3])?).sharpen();
    let mut uv = [(S::ZERO, S::ZERO); 3];
    for k in 0..3 {
        uv[k] = seed_on(surfaces[k], at)?;
    }
    for _ in 0..NEWTON_ITERATIONS {
        let mut offsets = [S::ZERO; 3];
        for k in 0..3 {
            uv[k] = sharp_in(surfaces[k], foot(surfaces[k], &center, uv[k])?);
            let p = surfaces[k].evaluate(uv[k].0, uv[k].1)?;
            normals[k] = surfaces[k].normal(uv[k].0, uv[k].1)?.normalize()?;
            offsets[k] = normals[k].prod_dot(&p).add(sr);
        }
        let rows = normals;
        let next = solve_linear_system(
            &Matrix::from_rows(rows.map(|n| n.to_array())),
            &Vector3::from_array(offsets),
        )?
        .sharpen();
        let same =
            (0..3).all(|k| next[k].is_subset_of(center[k]) && center[k].is_subset_of(next[k]));
        center = next;
        if same {
            break;
        }
    }
    let mut contacts = [Vector3::zero(); 3];
    for k in 0..3 {
        let (u, v) = foot(surfaces[k], &center, uv[k])?;
        contacts[k] = surfaces[k].evaluate(u, v)?;
        let n = surfaces[k].normal(u, v)?.normalize()?;
        let off = center.sub(&contacts[k]);
        let along = off.prod_dot(&n);
        let across = off.sub(&n.prod_scalar(along)).norm();
        // `sr` along the normal, and nothing across it, to within how far
        // the blend may stray from the ball anywhere.
        let slack = S::from_f64(crate::rolling::DEVIATION * sr.abs().upper().to_f64());
        if !(along.sub(sr).abs().definitely_less(slack) && across.definitely_less(slack)) {
            return Err(GeopError::new(format!(
                "the ball rounding the corner, centered at {center:?}, does not touch face {} truly: {along:?} off it along its normal, {across:?} across it",
                faces[k]
            )));
        }
    }
    Ok((center, contacts))
}
