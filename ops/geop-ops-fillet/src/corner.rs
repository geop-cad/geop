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
//! Each edge is rolled along (see [`crate::rolling`]), up to a station
//! built from the corner's ball
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
//! Where the radius varies along the edges and they meet the corner with
//! one radius — the corner's vertex given a radius of its own, say — that
//! is the ball's, which each fillet then reaches where it meets the ball.
//! Where they meet it with different radii, no one ball touches the three
//! faces at once: the corner is *set back* (SMLib's
//! `SmFilletConvexNxNCorner` with `m_dSetBackDist`). Each fillet ends at
//! its section a setback from the vertex — twice as far as it needs to for
//! its contacts to lie clear of its neighbours' on the faces they share, a
//! free choice — and the loop of those end arcs and the straight lines on
//! the faces between them is filled smoothly
//! ([`NurbSurface3D::fill`](geop_core_geometry::nurb_surface::NurbSurface3D::fill)),
//! tangent to the fillets along the arcs and to the faces along the lines
//! (see [`crate::tool`]). The lines need flat faces; where a face at the
//! corner is curved, the corner is bevelled instead, each fillet ending on
//! its own as where not every edge is filleted (see
//! [`crate::blend::tool_end`]: at more than three faces, a cut running out
//! of the solid through all the other faces), and the boolean meeting
//! them.
//!
//! Where more than three edges meet — a pyramid's apex — no ball touches
//! all the faces either (four planes generally have none): the corner is
//! set back and filled alike, its loop of `2N` sides. A corner where
//! every edge is filleted but the edges bend different ways — a pocket's
//! rim, its upright edge filled in where the rim is cut away — is refused
//! by name rather than left with the fillets overlapping. A chamfered
//! corner is left to its chamfers, which cross in a point.

use geop_core_geometry::{
    meet::{Offset, meet},
    search::{MAX_NODES, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::Matrix,
    scalars::Scalar,
    solvers::linear::solve_linear_system,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{EdgeId, FaceId, Model, VertexId};
use geop_ops_model::tangent::TangentChain as Chain;
use geop_ops_model::{NamedModel, Namer};

use crate::{
    blend::{Bend, BlendShape, check_touch},
    rolling::{self, ChainCorner, RadiusLaw, chain_bend, foot, seed_on},
    tool::{Corner, CornerEnd, Rounding},
};

/// The corners every edge of which is filleted, planned (see
/// [`plan_corners`]): each with its tools' ends — by index into the chains
/// — and, per chain, the corners' balls at its start and end.
pub(crate) struct Corners {
    pub corners: Vec<Corner>,
    pub chains: Vec<[Option<ChainCorner>; 2]>,
}

/// How a chain reaches a corner: by its index into the chains, and whether
/// at its end.
type Reaching = (usize, bool);

/// Plans the corner blends of `chains`, the blends of the edges `covered`,
/// into `shape`, named by `namer` (see the module docs): every corner every
/// edge of which is filleted.
pub(crate) fn plan_corners(
    model: &NamedModel,
    namer: &Namer,
    chains: &[(String, Chain)],
    covered: &[EdgeId],
    shape: &BlendShape,
) -> GeopResult<Corners> {
    let mut planned = Corners {
        corners: Vec::new(),
        chains: vec![[None, None]; chains.len()],
    };
    let BlendShape::Fillet { radii } = shape else {
        return Ok(planned);
    };
    let topology = model.topology();
    let mut vertices: Vec<VertexId> = Vec::new();
    for &edge in covered {
        let e = topology.get_edge(edge)?;
        for v in [e.start_vertex, e.end_vertex] {
            if !vertices.contains(&v) {
                vertices.push(v);
            }
        }
    }
    for vertex in vertices {
        // In the order they were made.
        let incident = topology.edges_at_vertex(vertex);
        // A vertex inside a tangent chain, or one not every edge of which
        // is blended, is no corner to round.
        if incident.len() < 3 || !incident.iter().all(|e| covered.contains(e)) {
            continue;
        }
        let name = model.name_of(vertex).unwrap_or("?").to_string();
        let ctx = with_context!("rounding the corner at vertex {name:?}");
        let mut reaching = Vec::new();
        let mut bends = Vec::new();
        for &edge in &incident {
            let edge_name = model.name_of(edge).unwrap_or("?");
            let Some(k) = chains.iter().position(|(_, c)| c.edges().contains(&edge)) else {
                return Err(GeopError::new(format!(
                    "every edge at the corner is filleted, but edge {edge_name:?} is blended round a whole circle: a corner is only rounded where the fillets end at it"
                )))
                .with_context(ctx);
            };
            let chain = &chains[k].1;
            let ends = chain.vertices(topology)?;
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
            bends.push(chain_bend(topology, chain).with_context(ctx)?);
            reaching.push((k, at_end));
        }
        if bends.iter().any(|b| *b != bends[0]) {
            return Err(GeopError::new(
                "every edge at the corner is filleted, but some are convex and some concave: a corner is only rounded where all its edges bend the same way",
            ))
            .with_context(ctx);
        }
        // The fillets' radii at the corner: the ball's, where they agree.
        let mut at_corner = Vec::new();
        for &(k, at_end) in &reaching {
            at_corner.push(
                RadiusLaw::new(model, &chains[k].1, radii)
                    .with_context(ctx)?
                    .at_end(usize::from(at_end)),
            );
        }
        let corner = if radii.setback.is_none()
            && incident.len() == 3
            && at_corner.iter().all(|&r| r == at_corner[0])
        {
            plan_corner(
                model,
                namer,
                vertex,
                &name,
                chains,
                &reaching,
                bends[0],
                at_corner[0],
                &mut planned.chains,
            )
        } else {
            // More than three faces, or fillets meeting with different
            // radii, have no one ball touching them all: the corner is set
            // back and filled, as it is wherever a setback is asked for.
            // The fill runs along the faces in straight lines, so only
            // between flat faces; elsewhere the corner is bevelled, each
            // fillet ending on its own, unless a setback was asked for.
            if let Some(face) = curved_face(topology, chains, &reaching)? {
                if let Some(d) = radii.setback {
                    return Err(GeopError::new(format!(
                        "the corner is set back {d}, but its face {face} is curved: a set-back corner is closed along its faces in straight lines, so only where they are flat"
                    )))
                    .with_context(ctx);
                }
                continue;
            }
            plan_set_back(
                model,
                namer,
                vertex,
                &name,
                chains,
                &reaching,
                &at_corner,
                radii.setback,
                &mut planned.chains,
            )
        }
        .with_context(ctx)?;
        planned.corners.push(corner);
    }
    Ok(planned)
}

/// The faces either side of the chain reaching a corner as `reaching` does.
fn faces_of(chains: &[(String, Chain)], (k, at_end): Reaching) -> [FaceId; 2] {
    let links = &chains[k].1.links;
    let link = if at_end { links.last() } else { links.first() };
    link.expect("a chain has links").faces
}

/// The corner at `vertex`, named `name`, where the tools `reaching` it
/// meet, filleted with `radius`, bending `bend` (see the module docs): its
/// ball, contacts and apex, and its tools' ends — set to it in
/// `chain_corners`.
#[allow(clippy::too_many_arguments)]
fn plan_corner(
    model: &NamedModel,
    namer: &Namer,
    vertex: VertexId,
    name: &str,
    chains: &[(String, Chain)],
    reaching: &[Reaching],
    bend: Bend,
    radius: f64,
    chain_corners: &mut [[Option<ChainCorner>; 2]],
) -> GeopResult<Corner> {
    let topology = model.topology();
    let at = topology.get_vertex(vertex)?.point;
    let mut faces: Vec<FaceId> = Vec::new();
    for &r in reaching {
        for face in faces_of(chains, r) {
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
    let r = Scalar::from_f64(radius);
    let (center, contacts) = ball(topology, faces, &at, bend.side().mul(r))?;
    for (face, p) in faces.iter().zip(&contacts) {
        let ctx = with_context!("where the corner's ball touches face {face}, at {p:?}");
        check_touch(topology, *face, p).with_context(ctx)?;
    }
    let contact_on = |face: FaceId| {
        contacts[faces
            .iter()
            .position(|&f| f == face)
            .expect("a corner face")]
    };

    let mut ends = Vec::new();
    for &reach in reaching {
        let (k, at_end) = reach;
        let link_faces = faces_of(chains, reach);
        chain_corners[k][usize::from(at_end)] = Some(ChainCorner {
            center,
            contacts: link_faces.map(contact_on),
        });
        ends.push(CornerEnd {
            tool: k,
            at_end,
            contacts: link_faces.map(|face| {
                faces
                    .iter()
                    .position(|&f| f == face)
                    .expect("a corner face")
            }),
        });
    }
    Ok(Corner {
        namer: namer.scoped(name),
        rounding: Rounding::Ball { center, radius: r },
        faces: faces.to_vec(),
        contacts: contacts.to_vec(),
        apex: at.prod_scalar(Scalar::TWO).sub(&center),
        inside: at,
        ends,
    })
}

/// How much further than needed a corner is set back where no setback is
/// given (see [`plan_set_back`]): a free choice of the patch's size, so that
/// the lines between the fillets on the faces are as long again as the
/// fillets are near each other there.
const SET_BACK_MARGIN: f64 = 2.0;

/// The corner at `vertex`, named `name`, where the tools `reaching` it
/// meet with the radii `radii` there, set back (see the module docs): each
/// fillet ends `setback` from the vertex — where none is given, twice as
/// far as each needs to be clear of its neighbours on the faces they share
/// — and a smooth patch fills the loop of their end arcs and the lines on
/// the faces between them. Its tools' ends are set in `chain_corners`.
///
/// Refused, naming the edges and the face: a setback given too short for
/// two fillets on one face to stay clear of each other.
#[allow(clippy::too_many_arguments)]
fn plan_set_back(
    model: &NamedModel,
    namer: &Namer,
    vertex: VertexId,
    name: &str,
    chains: &[(String, Chain)],
    reaching: &[Reaching],
    radii: &[f64],
    setback: Option<f64>,
    chain_corners: &mut [[Option<ChainCorner>; 2]],
) -> GeopResult<Corner> {
    let topology = model.topology();
    let at = topology.get_vertex(vertex)?.point;
    // Each edge's unit direction away from the vertex, in plain numbers.
    let mut away = Vec::new();
    for &(k, at_end) in reaching {
        let links = &chains[k].1.links;
        let (link, fraction) = match at_end {
            true => (links.last().expect("links"), Scalar::ONE),
            false => (&links[0], Scalar::ZERO),
        };
        let (_, t) = link.at(topology, fraction)?;
        let t = t.to_f64_array();
        let sign = if at_end { -1.0 } else { 1.0 };
        let length = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
        away.push(t.map(|x| sign * x / length));
    }
    // How far from the vertex each fillet has to end for its contact on a
    // face it shares with another to lie clear of that one's: on the face,
    // with the edges at an angle `θ`, the contact `r_i` off edge `i` lies
    // `s sin θ - r_i cos θ` off edge `j`, at least `r_j`.
    let mut needed = vec![(0.0f64, String::new()); reaching.len()];
    for i in 0..reaching.len() {
        for j in 0..reaching.len() {
            let shared = faces_of(chains, reaching[i])
                .into_iter()
                .find(|f| faces_of(chains, reaching[j]).contains(f));
            let Some(face) = shared.filter(|_| i != j) else {
                continue;
            };
            let (a, b) = (away[i], away[j]);
            let cos = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            let sin = (1.0 - cos * cos).max(0.0).sqrt();
            let s = (radii[j] + radii[i] * cos) / sin;
            if s.is_nan() || s > needed[i].0 {
                let edges = (&chains[reaching[i].0].0, &chains[reaching[j].0].0);
                needed[i] = (s, format!("{edges:?} on face {face}"));
            }
        }
    }
    if needed.iter().any(|(s, _)| !s.is_finite()) {
        return Err(GeopError::new(format!(
            "two edges at the corner leave it along one line, their fillets never clear of each other: {needed:?}"
        )));
    }
    let most = (needed.iter()).fold(0.0f64, |m, (s, _)| m.max(*s));
    let setback = match setback {
        Some(d) => {
            if let Some((s, which)) = needed.iter().find(|(s, _)| d <= *s) {
                return Err(GeopError::new(format!(
                    "the corner's setback {d} is too short: the fillets of edges {which} would meet within {s} of the vertex"
                )));
            }
            d
        }
        None => SET_BACK_MARGIN * most,
    };
    let mut faces = Vec::new();
    let mut contacts = Vec::new();
    let mut planes = Vec::new();
    let mut ends = Vec::new();
    for (i, &reach) in reaching.iter().enumerate() {
        let (k, at_end) = reach;
        let ctx = with_context!("the fillet of edge {:?} set back {setback}", chains[k].0);
        let ball = rolling::set_back(
            topology,
            &chains[k].1,
            usize::from(at_end),
            radii[i],
            setback,
        )
        .with_context(ctx)?;
        let link_faces = faces_of(chains, reach);
        let first = contacts.len();
        for (face, p) in link_faces.iter().zip(&ball.contacts) {
            let ctx = with_context!("where the fillet ends set back on face {face}, at {p:?}");
            check_touch(topology, *face, p).with_context(ctx)?;
            faces.push(*face);
            contacts.push(*p);
            planes.push(topology.get_face(*face)?.surface.clone());
        }
        chain_corners[k][usize::from(at_end)] = Some(ball);
        ends.push(CornerEnd {
            tool: k,
            at_end,
            contacts: [first, first + 1],
        });
    }
    let middle = (contacts.iter())
        .fold(Vector3::zero(), |sum, p| sum.add(p))
        .prod_scalar(Scalar::ONE.div(Scalar::from_i64(contacts.len() as i64))?)
        .sharpen();
    Ok(Corner {
        namer: namer.scoped(name),
        rounding: Rounding::Fill { planes },
        faces,
        contacts,
        apex: at.prod_scalar(Scalar::TWO).sub(&middle),
        inside: at,
        ends,
    })
}

/// A face at the corner the tools `reaching` meet at that is curved, if
/// any: a set-back corner's fill runs along its faces in straight lines
/// between the fillets (see [`plan_set_back`]), so only along flat ones.
fn curved_face(
    topology: &Model,
    chains: &[(String, Chain)],
    reaching: &[Reaching],
) -> GeopResult<Option<FaceId>> {
    for &reach in reaching {
        for face in faces_of(chains, reach) {
            if topology.get_face(face)?.surface.as_plane()?.is_none() {
                return Ok(Some(face));
            }
        }
    }
    Ok(None)
}

/// The center of the ball `sr` from each of `faces` along its outward
/// normal — `r` behind them for a negative `sr` — near their corner `at`,
/// and where it touches each.
///
/// Three planes give it exactly, by one linear solve. Against curved faces
/// it is Newton's method ([`meet`], which checks it against every face),
/// from the planes tangent to the faces at the corner.
fn ball(
    model: &Model,
    faces: [FaceId; 3],
    at: &Vector3,
    sr: Scalar,
) -> GeopResult<(Vector3, [Vector3; 3])> {
    let solve = |normals: [Vector3; 3], offsets: [Scalar; 3]| -> GeopResult<Vector3> {
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
        normals[k] = surfaces[k].normal_at(at, MAX_NODES, min_subdivision_size())?;
    }
    let center = at.add(&solve(normals, [sr; 3])?).sharpen();
    let mut offsets = Vec::new();
    for surface in surfaces {
        offsets.push(Offset::new(surface, seed_on(surface, at)?, sr));
    }
    // A construction aid, sharp: the contacts are evaluated on the faces.
    let center = meet(center, &mut offsets, &[])?.sharpen();
    let mut contacts = [Vector3::zero(); 3];
    for (k, offset) in offsets.iter().enumerate() {
        let (u, v) = foot(surfaces[k], &center, offset.pieces[0].1)?;
        contacts[k] = surfaces[k].evaluate(u, v)?;
    }
    Ok((center, contacts))
}
