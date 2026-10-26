//! Blending a solid's edges: rounding them with a fillet, or bevelling them
//! with a chamfer — by cutting away the material between a convex edge and
//! its blend face with a boolean difference, or filling a concave edge in
//! up to it with a union.
//!
//! Every blend is decided in the edge's *cross-section*: the plane through a
//! point `E` of the edge, perpendicular to it. There the two faces meeting
//! at the edge trace two straight lines from `E`, each running *into* its
//! face along `t` and facing out of the solid along `n`. The edge is convex
//! where each face runs into the side of the other's line the solid lies
//! on — less than a half turn of material between them — and concave where
//! it runs out of it. The blend replaces the corner between them:
//!
//! - a **fillet** of radius `r` by the arc of the circle tangent to both
//!   lines — centered at `C`, `r` from both lines on the side `s` of them,
//!   behind them (`s = -1`) at a convex edge and in front (`s = +1`) at a
//!   concave one, touching them at `T_a = C - s r n_a` and
//!   `T_b = C - s r n_b`. Its tangents at both ends meet at `E`, so it is
//!   the exact rational quadratic with middle control point `E`, weighted
//!   `r / |E - C|` (the cosine of half the angle it turns).
//! - a **chamfer** by the chord from `T_a = E + d_a t_a` to `T_b = E + d_b
//!   t_b`.
//!
//! The *tool* is the region between the blend and the corner, closed off on
//! the far side of the corner: for a fillet, the arc and two lines from its
//! ends to the apex `Q = 2E - C`; for a chamfer, the triangle on the chord
//! extended past both ends, with the apex as far beyond the corner as the
//! chord's middle is before it. Each line stays on the far side of the face
//! its end lies on, so the tool meets the solid — or, at a concave edge, the
//! space around it — exactly in the corner the blend removes or fills.
//!
//! Every edge — straight or curved, between any faces, of a radius that
//! varies or not — is blended one way: by a ball rolled along it and its
//! tangent chain, the tool skinned through the ball's sections (see
//! [`crate::rolling`]); bevelled alike, its sections the chords. Where the
//! section is carried along the edge unchanged — a straight edge between
//! planes, a circle between faces turning round its axis — the skinned
//! tool is that section swept along it, exactly. A chain ends at corners
//! of exactly one more face (see [`tool_end`]): run out past it where the
//! edge leaves the solid through it and the blend cuts, flush with it
//! otherwise.
//!
//! A blend has to meet both faces inside them — checked along the whole of
//! where it touches them (see [`check_contacts`]) — so one too large for
//! its faces, or running exactly along another of their edges, is refused.
//! Where its contact runs off a face across an edge onto another face at a
//! crease — over a cliff, a convex edge, or into a wall, a concave one —
//! the tool is cut off by that face's surface carried on, the part beyond
//! it dropped (see [`Cut`]): a round under a bar narrower than it ends
//! flush with the bar's sides, one running into a rib flush with the rib.
//! A contact that leaves its face that way and comes back to it further
//! on is refused: the cut would drop the blend beyond too.
//!
//! Several edges are blended one after the other, every tool built from the
//! solid as it was before the first. Where two blended edges meet at a
//! corner:
//!
//! - at an outward corner — two edges leaving the solid there — the tools
//!   run out past it, and what they cut together meets along the plane
//!   halving the corner;
//! - where *every* edge of a corner is filleted, the fillets end at the
//!   ball touching all its faces, whose piece rounds the corner, and the
//!   tools are joined there into one (see [`crate::corner`]);
//! - at an inward corner — two convex edges running into each other's
//!   walls, a pocket's rim — each tool ends flush with the other's wall,
//!   which leaves the corner between the walls uncut. What either blend
//!   cuts beyond the other's wall is what both would: the corner is cut by
//!   the two tools run on past it, intersected (see [`mitre`]) — for any
//!   two blends, of any radius, the walls at any angle.
//!
//! Tools joined at corners are built whole, each such set one solid, and
//! applied by one boolean. Two of them ending on their own at one vertex
//! would overlap there, which one solid cannot: refused, naming the edges
//! and the vertex.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    intersection::{Intersections, curve_curve::curve_curve_intersect},
    nurb_curve::NurbCurve3D,
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{
    Body, EdgeId, FaceId, Model, SolidId, VertexId,
    build::BodySpec,
    contains::face::{PointClassification, face_contains, face_interior_point},
};
use geop_lib_booleans::{RemeshParams, boolean, split};
use geop_ops_model::{BodyNames, BooleanOp};
use geop_ops_model::{NamedModel, Namer};

use crate::corner;
use crate::rolling::{self, Radii, Rolled};
use crate::tool::{self, Corner, CornerEnd, Tool};
use geop_ops_model::tangent::{TangentChain as Chain, tangent_chain};

/// What a blend replaces an edge's corner with (see the module docs).
#[derive(Clone, Debug, PartialEq)]
pub enum BlendShape {
    /// A round tangent to both faces, of the radius `radii` lay along the
    /// edges.
    Fillet { radii: Radii },
    /// A bevel `distances[0]` into the face on the left of the edge as it
    /// runs (seen from outside the solid), `distances[1]` into the one on
    /// its right.
    Chamfer { distances: [f64; 2] },
}

impl BlendShape {
    /// A round of `radius`, the same all along.
    #[cfg(test)]
    pub fn round(radius: f64) -> Self {
        BlendShape::Fillet {
            radii: Radii::constant(radius),
        }
    }
}

/// How far a tool `width` wide runs out past the end of its edge (see
/// [`End::RunOut`]): as far again as it is wide, over how squarely the edge
/// crosses the face it is run out past — a free choice, only ever rounded
/// up.
pub(crate) fn run_out_length(width: f64, squareness: &Scalar) -> Scalar {
    Scalar::from_f64(2.0 * width / squareness.lower().to_f64())
}

/// How a blend's open chain (see [`crate::rolling`]) ends, at one end of
/// it.
#[derive(Clone, Debug)]
pub(crate) enum End {
    /// Run out past the end — as far again as the tool is wide, over how
    /// squarely the edge leaves the solid there. What a cut does where the
    /// edge leaves the solid.
    RunOut,
    /// Flush with the face at the end: what material added does, which
    /// must not stick out past it. Why it cannot be capped in the face,
    /// `unless` the face is a plane square to the edge: then the blend runs
    /// on through the face and is cut off by it (see
    /// [`crate::rolling::RunOn`]).
    Flush { unless: Option<String> },
    /// Flush with a wall the edge runs into: what a cut does there, which
    /// must not cut what lies behind the wall — and where the wall's own
    /// edge along the same face is blended too, the corner between them is
    /// cut by a [`mitre`]. Why it cannot be capped in the wall, `unless` the
    /// wall is a plane square to the edge: then the blend runs on through
    /// the wall and is cut off by it.
    Wall { unless: Option<String> },
    /// At the ball rounding a corner every edge of which is filleted: where
    /// the tool is joined to the corner's (see [`crate::corner`]).
    Corner,
}

/// Which way an edge bends: whether its blend cuts material away or adds it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Bend {
    /// Less than a half turn of material between the faces: cut away.
    Convex,
    /// More: filled in.
    Concave,
}

impl Bend {
    /// Which side of both faces the blend's circle lies on: `-1` behind
    /// them, in the material, for a convex edge; `+1` in front for a concave
    /// one.
    pub(crate) fn side(self) -> Scalar {
        match self {
            Bend::Convex => Scalar::ONE.neg(),
            Bend::Concave => Scalar::ONE,
        }
    }

    /// The boolean that applies the tool.
    fn op(self) -> BooleanOp {
        match self {
            Bend::Convex => BooleanOp::Difference,
            Bend::Concave => BooleanOp::Union,
        }
    }
}

/// The two faces of `edge`, left first (see [`Model::faces_of_edge`]).
pub(crate) fn faces(model: &Model, edge: EdgeId) -> GeopResult<[FaceId; 2]> {
    model.faces_of_edge(edge).ok_or_else(|| {
        GeopError::new(format!(
            "edge {edge} does not join two faces, so it has no corner to blend"
        ))
    })
}

/// Which way an edge bends, from `bend` — the direction running into the
/// face on its left, dotted with the right one's outward normal: convex
/// where the left face runs into the side of the right one's tangent plane
/// the solid lies on. An error if the faces could be tangent there — there
/// is no corner to blend.
pub(crate) fn bend(bend: Scalar) -> GeopResult<Bend> {
    if bend.definitely_less(Scalar::ZERO) {
        Ok(Bend::Convex)
    } else if bend.definitely_greater(Scalar::ZERO) {
        Ok(Bend::Concave)
    } else {
        Err(GeopError::new(
            "the faces could meet tangentially at the edge: there is no corner to blend",
        ))
    }
}

/// How the tool of a straight edge between `faces`, bending `bend`, ends at
/// the edge's end `vertex`, the edge running on along `leaving` (see the
/// module docs). It is decided by the one other face meeting there: run
/// out past it where the edge leaves the solid through it and the tool
/// cuts — any face, the tool running on into the air — flush with it
/// otherwise — where the tool fills, or the edge runs into it — which needs
/// it a plane square to the edge.
/// Anything else, and a tool along the edge would cut or fill what it
/// should not: it is said `unless`, for a rolled tool to run on through the
/// face and be cut off by it. The end, and that third face.
pub(crate) fn tool_end(
    model: &Model,
    vertex: VertexId,
    faces: [FaceId; 2],
    leaving: &Vector3,
    bend: Bend,
) -> GeopResult<(End, FaceId)> {
    let mut others: Vec<FaceId> = Vec::new();
    for (&id, coedge) in &model.coedges {
        if coedge.edge().is_ok()
            && model.coedge_start_vertex_id(id)? == vertex
            && !faces.contains(&coedge.face)
            && !others.contains(&coedge.face)
        {
            others.push(coedge.face);
        }
    }
    // In id order, not the coedges' hash-map order: of two flush faces the
    // first is the wall's, and it is what the end is measured against.
    others.sort_by_key(|f| f.0);
    // A face a union left in two — a plate standing flush with the side of
    // what it stands on — is one wall: what is told of the third face is
    // told of the plane they share.
    let mut walls: Vec<FaceId> = Vec::new();
    for &face in &others {
        let plane = model.get_face(face)?.surface.as_plane()?;
        let flush = |wall: &FaceId| -> GeopResult<bool> {
            let (Some(a), Some(b)) = (&plane, model.get_face(*wall)?.surface.as_plane()?) else {
                return Ok(false);
            };
            let (na, nb) = (a.normal.normalize()?, b.normal.normalize()?);
            Ok(na.prod_dot(&nb).definitely_greater(Scalar::ZERO)
                && na.prod_cross(&nb).norm_sq().could_be_equal(Scalar::ZERO)
                && a.signed_distance(&b.point).could_be_equal(Scalar::ZERO))
        };
        let mut flush_with_one = false;
        for wall in &walls {
            if flush(wall)? {
                flush_with_one = true;
                break;
            }
        }
        if !flush_with_one {
            walls.push(face);
        }
    }
    let others = walls;
    let unsupported = |why: &str| {
        Err(GeopError::new(format!(
            "the edge ends at vertex {vertex}, where {why}: a blend ends only at a corner of exactly three faces — or a plane in two flush faces — run out through the third face where the edge leaves the solid through it, flush with it where it runs into it, and filled up to it only where it is a plane square to the edge"
        )))
    };
    let [other] = others.as_slice() else {
        return unsupported(&format!("{} more faces meet", others.len()));
    };
    let plane = model.get_face(*other)?.surface.as_plane()?;
    let normal = match &plane {
        Some(plane) => plane.normal.normalize()?,
        None => model.get_face(*other)?.surface.normal_at(
            &model.get_vertex(vertex)?.point,
            MAX_NODES,
            min_subdivision_size(),
        )?,
    };
    let squareness = leaving.prod_dot(&normal);
    let leaves = if squareness.definitely_greater(Scalar::ZERO) {
        true
    } else if squareness.definitely_less(Scalar::ZERO) {
        false
    } else {
        return unsupported("the third face could run along the edge");
    };
    let why = if plane.is_none() {
        Some("the third face is not planar")
    } else if !normal.could_be_parallel(leaving) {
        Some(if leaves {
            "the edge is filled in up to the third face, which does not stand square to it"
        } else {
            "the edge runs into the third face, which does not stand square to it"
        })
    } else {
        None
    };
    // Out of the solid through any face, a cut runs on past it into the
    // air: what it does there cuts nothing, so the face may be curved or
    // stand at any angle.
    if leaves && bend == Bend::Convex {
        return Ok((End::RunOut, *other));
    }
    // Filled in up to a face the edge leaves the solid through — an outward
    // corner — the fill ends flush with it, touching the corner's other
    // edge, the one between that face and the face the fill runs along. In
    // the plane of a face square to the edge, that is the face's own
    // business; anywhere else, the end would be a face of its own ending on
    // that edge, the very place no boolean can cut (a blade's root, filled
    // round to its sharp trailing edge, ended in a notch along the edge).
    if leaves && let Some(why) = why {
        return unsupported(&format!(
            "{why}, and the edge leaves the solid there, an outward corner: a fill up to it would end on the corner's edge between the third face and the face it is filled along"
        ));
    }
    if leaves || bend == Bend::Concave {
        let unless = why.map(|why| {
            format!(
                "the edge ends at vertex {vertex}, where {why}: a straight edge between planes is filled up to the third face only where it is a plane square to the edge — an edge rolled between curved faces up to any face it runs into"
            )
        });
        return Ok((End::Flush { unless }, *other));
    }
    let unless = why.map(|why| {
        format!(
            "the edge ends at vertex {vertex}, where {why}: a straight edge between planes ends flush only with a plane square to it; an edge rolled between curved faces with any wall"
        )
    });
    Ok((End::Wall { unless }, *other))
}

/// Seed of the ray casting behind `face_contains` here, whose answer does
/// not depend on it — a constant only keeps runs reproducible.
const SEED: u64 = 0xB1E2_D000_0000_0001;

/// Where a blend's contact runs off a face it touches, across one of the
/// face's edges, onto the face `beyond` — at the points `at` — the unit
/// direction `into` the face it runs off, square to the edge at the first,
/// and `on`, points of the contact on its faces between crossings, which
/// every cut has to keep.
#[derive(Clone, Debug)]
pub(crate) struct Crossing {
    pub(crate) beyond: FaceId,
    pub(crate) at: Vec<Vector3>,
    pub(crate) into: Vector3,
    pub(crate) on: Vec<Vector3>,
}

/// Checks a blend touching the faces `sides` — those on the left of its
/// edges and those on their right — along the curves `contacts`, per
/// piece of the blend its contacts on either side, meets them where they
/// are: that each contact, the whole of it, crosses no edge of the faces
/// on its side but those it is meant to — an edge between two of them,
/// which it rolls across, the blend's own edges `own`, and an edge at one
/// of its ends `ends`, past which it runs on or out. (The question is
/// about the curve, not a point of it: a contact circle running off a bar
/// on both sides of the point halfway along it was let through when only
/// that point was asked, and built an invalid solid.)
///
/// A contact that crosses another edge runs off its face onto the face
/// beyond (a cliff where the edge is convex, a wall where it is concave):
/// there the blend is cut off by that face's surface, the part on the far
/// side of it dropped (see [`Cut`]). Those crossings, one per face beyond.
/// A contact running exactly along an edge asks the boolean to cut along
/// it, and one rolling onto a face tangent to its own is no crossing a
/// cut makes: both refused, naming the edge.
///
/// A contact on one side that crosses none of those edges lies on one side
/// of them all, which one point of it decides: each of `middles`, per link
/// of the blend a point of its contact on either side within it, has to
/// lie on a face of its side — inside it, or on an edge it may meet. One
/// wholly beyond its faces, too large for them, is refused.
pub(crate) fn check_contacts(
    model: &Model,
    sides: &[Vec<FaceId>; 2],
    contacts: &[[NurbCurve3D; 2]],
    middles: &[[Vector3; 2]],
    ends: &[VertexId],
    own: &[EdgeId],
) -> GeopResult<Vec<Crossing>> {
    let size = RemeshParams::default().curve_curve_min_subdivision_size;
    let mut crossings: Vec<Crossing> = Vec::new();
    // The points of the contacts on their faces between crossings.
    let mut kept: Vec<Vector3> = Vec::new();
    for (k, faces) in sides.iter().enumerate() {
        let crossed = crossings.len();
        // The edges the contact on this side must not cross.
        let mut fences: Vec<EdgeId> = Vec::new();
        for &face in faces {
            for edge in model.face_edges(face) {
                let e = model.get_edge(edge)?;
                let across = model
                    .faces_of_edge(edge)
                    .is_some_and(|pair| pair.iter().all(|f| faces.contains(f)));
                let at_end = ends.contains(&e.start_vertex) || ends.contains(&e.end_vertex);
                if !across && !at_end && !own.contains(&edge) && !fences.contains(&edge) {
                    fences.push(edge);
                }
            }
        }
        fences.sort_by_key(|e| e.0);
        for (piece, contact) in contacts.iter().enumerate() {
            let mut params: Vec<Scalar> = Vec::new();
            for &edge in &fences {
                let curve = &model.get_edge(edge)?.curve;
                let ctx = with_context!(
                    "the blend's contact with the faces {faces:?} on side {k}, piece {piece}, against edge {edge}"
                );
                let hits =
                    curve_curve_intersect(&contact[k], curve, MOST_CROSSINGS, MAX_NODES, size)
                        .with_context(ctx)?;
                let mut at = Vec::new();
                for &(t, _) in hits.as_slice() {
                    at.push(contact[k].evaluate(t)?);
                    params.push(t);
                }
                let found = match hits {
                    Intersections::Found(found) => found,
                    Intersections::Coincident(_) => {
                        return Err(GeopError::new(format!(
                            "the blend is too large for its faces: where it touches faces {faces:?} it runs exactly along edge {edge}, at {:?}",
                            at.first()
                        )));
                    }
                };
                if found.is_empty() {
                    continue;
                }
                let crossing = crossing_at(model, faces, edge, at).with_context(ctx)?;
                match crossings.iter_mut().find(|c| c.beyond == crossing.beyond) {
                    Some(c) => c.at.extend(crossing.at),
                    None => crossings.push(crossing),
                }
            }
            if params.is_empty() {
                continue;
            }
            // Between two crossings — or a crossing and an end — the
            // contact lies on its faces or off them throughout, which its
            // middle says: those on them, to be kept by every cut.
            let (t0, t1) = contact[k].domain();
            params.extend([t0, t1]);
            params.sort_by(|a, b| a.midpoint().to_f64().total_cmp(&b.midpoint().to_f64()));
            let mut on = Vec::new();
            for pair in params.windows(2) {
                let t = pair[0]
                    .upper()
                    .add(pair[1].lower())
                    .div(Scalar::TWO)?
                    .sharpen();
                let p = contact[k].evaluate(t)?;
                let mut lies = false;
                for &face in faces {
                    lies |= on_face(model, face, &p)?;
                }
                if lies {
                    on.push(p);
                }
            }
            kept.extend(on);
        }
        if crossings.len() > crossed {
            continue;
        }
        for points in middles {
            let p = &points[k];
            let mut on = false;
            for &face in faces {
                on |= on_face(model, face, p)?;
            }
            if !on {
                return Err(GeopError::new(format!(
                    "the blend is too large for its faces: where it touches faces {faces:?}, at {p:?}, it lies beyond them"
                )));
            }
        }
    }
    for c in &mut crossings {
        c.on = kept.clone();
    }
    Ok(crossings)
}

/// How many crossings of a contact with one edge are asked for at most:
/// a contact runs off a face and back onto it at most this often across
/// one edge — a free choice of how hard to look, enough for a circle
/// crossing a straight edge, twice.
const MOST_CROSSINGS: usize = 4;

/// Where a contact on the faces `faces` crosses `edge`, at `at`: onto the
/// face across it, and the way into the face it runs off. Refused where
/// the edge bounds one face only, or the face across it is tangent to the
/// one it runs off there (the ball would roll on onto it, which a cut does
/// not make).
fn crossing_at(
    model: &Model,
    faces: &[FaceId],
    edge: EdgeId,
    at: Vec<Vector3>,
) -> GeopResult<Crossing> {
    let Some(pair) = model.faces_of_edge(edge) else {
        return Err(GeopError::new(format!(
            "the blend's contact runs off its face across edge {edge}, which bounds one face only"
        )));
    };
    // The face on the edge's left runs along it the way it runs.
    let (from, beyond, way) = if faces.contains(&pair[0]) {
        (pair[0], pair[1], Scalar::ONE)
    } else {
        (pair[1], pair[0], Scalar::ONE.neg())
    };
    let normal = |face: FaceId, p: &Vector3| -> GeopResult<Vector3> {
        let surface = &model.get_face(face)?.surface;
        surface.normal_at(p, MAX_NODES, min_subdivision_size())
    };
    for p in &at {
        if normal(from, p)?.could_be_parallel(&normal(beyond, p)?) {
            return Err(GeopError::new(format!(
                "the blend's contact runs off face {from} across edge {edge}, at {p:?}, onto face {beyond}, which could be tangent to it there: a blend is cut off only by a face it meets at a crease"
            )));
        }
    }
    let curve = &model.get_edge(edge)?.curve;
    let t = curve
        .closest_parameter_from(&at[0], curve.mid_parameter()?)?
        .sharpen();
    let along = curve.tangent(t)?.prod_scalar(way);
    let into = normal(from, &at[0])?
        .normalize()?
        .prod_cross(&along)
        .normalize()?;
    Ok(Crossing {
        beyond,
        at,
        into,
        on: Vec::new(),
    })
}

/// A blend's tool cut off by the surface of a face it runs onto (see
/// [`check_contacts`]): split along `sheet`, a piece of the surface
/// carried on round where the contact crosses onto it, and every piece of
/// it on the far side of `side`, the surface carried on, from `keep` —
/// a point on the blend's own side — dropped.
#[derive(Clone, Debug)]
pub(crate) struct Cut {
    sheet: NurbSurface3D,
    side: NurbSurface3D,
    keep: Vector3,
}

/// The cuts of a tool `reach` wide whose contacts make `crossings` (see
/// [`check_contacts`]): each face crossed onto, its surface carried on
/// past every crossing as far as the tool reaches, the blend kept on the
/// side of it the face it runs off lies on — `reach` into it from the
/// first crossing, a free choice: on a plane any distance is. Refused
/// where a cut would drop part of the contact on its faces: a wall the
/// contact runs across and back off (a rib standing on the face), whose
/// sides would cut away the blend on its far side too.
pub(crate) fn cuts(model: &Model, crossings: &[Crossing], reach: f64) -> GeopResult<Vec<Cut>> {
    let mut cuts = Vec::new();
    for crossing in crossings {
        let ctx = with_context!("cutting the blend off by face {}", crossing.beyond);
        let mut side = model.get_face(crossing.beyond)?.surface.clone();
        for p in &crossing.at {
            side = rolling::carried_on_past(&side, p, reach).with_context(ctx)?;
        }
        let first = crossing.at[0];
        let spread = (crossing.at.iter())
            .map(|p| p.sub(&first).norm().upper().to_f64())
            .fold(0.0, f64::max);
        let sheet = rolling::around(&side, &first, 2.0 * reach + spread).with_context(ctx)?;
        let keep = first.add(&crossing.into.prod_scalar(Scalar::from_f64(reach)));
        let keep_side = side_of(&side, &keep)?;
        for p in &crossing.on {
            if keep_side.is_none() || side_of(&side, p)? != keep_side {
                return Err(GeopError::new(format!(
                    "the blend's contact runs off its face onto face {} and on beyond it back onto its face, at {p:?}: a blend is cut off only by a face its contact leaves for good",
                    crossing.beyond
                )))
                .with_context(ctx);
            }
        }
        cuts.push(Cut { sheet, side, keep });
    }
    Ok(cuts)
}

/// Which side of `surface` the point `p` lies on, along the normal at its
/// nearest point: `true` in front, `false` behind, none if it could lie on
/// it.
fn side_of(surface: &NurbSurface3D, p: &Vector3) -> GeopResult<Option<bool>> {
    let (u, v, q) = surface.closest_point(p)?;
    let off = p.sub(&q).prod_dot(&surface.normal(u, v)?);
    Ok(if off.definitely_greater(Scalar::ZERO) {
        Some(true)
    } else if off.definitely_less(Scalar::ZERO) {
        Some(false)
    } else {
        None
    })
}

/// `tool` cut off as `cut` says, named by `namer` as a split names what it
/// makes: the one piece left on the blend's side.
fn cut_off(model: &mut NamedModel, namer: &Namer, tool: SolidId, cut: &Cut) -> GeopResult<SolidId> {
    let keep = side_of(&cut.side, &cut.keep)?.ok_or_else(|| {
        GeopError::new(format!(
            "the blend's contact at {:?}, which says which side of the face it runs onto it is on, could lie on that face",
            cut.keep
        ))
    })?;
    let (pieces, sheet) = split_by_sheet(model, &namer.scoped("cut"), tool, &cut.sheet)?;
    let mut kept = Vec::new();
    let mut rest = vec![Body::Sheet(sheet)];
    for piece in pieces {
        // A point inside one of its faces, other than those the sheet cut,
        // which lie on the surface: the piece lies on one side of the
        // sheet, and any of its points off it says which.
        let mut side = None;
        let topology = model.topology();
        for face in topology.solid_faces(piece)? {
            let (u, v) =
                face_interior_point(topology, face, MAX_NODES, min_subdivision_size(), SEED)?;
            let p = topology.get_face(face)?.surface.evaluate(u, v)?;
            if let Some(s) = side_of(&cut.side, &p)? {
                side = Some(s);
                break;
            }
        }
        match side {
            Some(s) if s == keep => kept.push(piece),
            Some(_) => rest.push(Body::Solid(piece)),
            None => {
                return Err(GeopError::new(
                    "a piece of the blend's tool cut off by a face it runs onto lies on that face throughout",
                ));
            }
        }
    }
    let [piece] = kept[..] else {
        return Err(GeopError::new(format!(
            "the blend's tool cut off by a face it runs onto came apart in {} pieces on the blend's side, not one",
            kept.len()
        )));
    };
    model.delete_bodies(&rest)?;
    Ok(piece)
}

/// Whether the point `p` lies on `face`: on its surface, inside its
/// boundary or on it.
fn on_face(model: &Model, face: FaceId, p: &Vector3) -> GeopResult<bool> {
    let surface = &model.get_face(face)?.surface;
    let Some((u, v)) = surface_could_contain(surface, p, MAX_NODES, min_subdivision_size())? else {
        return Ok(false);
    };
    let at = face_contains(model, face, u, v, MAX_NODES, min_subdivision_size(), SEED)?;
    Ok(at != PointClassification::Outside)
}

/// Checks a blend touching `face` at `p` touches it where the face is:
/// inside it. A blend too large for a face runs past its boundary, and one
/// ending exactly on another of its edges asks the boolean to cut along
/// that edge.
pub(crate) fn check_touch(model: &Model, face: FaceId, p: &Vector3) -> GeopResult<()> {
    let (max_nodes, size) = (MAX_NODES, min_subdivision_size());
    let surface = &model.get_face(face)?.surface;
    let Some((u, v)) = surface_could_contain(surface, p, max_nodes, size)? else {
        return Err(GeopError::new(format!(
            "the blend is too large for face {face}: it would meet the face's surface beyond its end"
        )));
    };
    check_touch_at(model, face, u, v)
}

/// Checks a blend touching `face` at `(u, v)` on its surface touches it
/// inside it (see [`check_touch`]).
pub(crate) fn check_touch_at(model: &Model, face: FaceId, u: Scalar, v: Scalar) -> GeopResult<()> {
    let (max_nodes, size) = (MAX_NODES, min_subdivision_size());
    match face_contains(model, face, u, v, max_nodes, size, SEED)? {
        PointClassification::Inside => Ok(()),
        PointClassification::Outside => Err(GeopError::new(format!(
            "the blend is too large for face {face}: it would run past the face's boundary"
        ))),
        PointClassification::OnCoedge | PointClassification::OnVertex => {
            Err(GeopError::new(format!(
                "the blend would end exactly on another edge of face {face}: make it a little smaller or larger"
            )))
        }
    }
}

/// A tool's end running into a wall (see [`End::Wall`]): which tool — by
/// index into the plans, and past them the chains — at its end or its
/// start, the vertex, and the faces either side.
struct WallEnd {
    tool: usize,
    at_end: bool,
    vertex: VertexId,
    faces: [FaceId; 2],
}

/// The inward corners between `walls`: every two ends running into each
/// other's walls at a vertex, along a face they share (see the module
/// docs) — by index into `walls`. More than two there is refused.
fn inward_corners(walls: &[WallEnd]) -> GeopResult<Vec<[usize; 2]>> {
    let mut pairs = Vec::new();
    let mut used = vec![false; walls.len()];
    for a in 0..walls.len() {
        for b in a + 1..walls.len() {
            let (wa, wb) = (&walls[a], &walls[b]);
            let shares = wa.faces.iter().any(|f| wb.faces.contains(f));
            if !shares || wa.vertex != wb.vertex || wa.tool == wb.tool {
                continue;
            }
            if used[a] || used[b] {
                return Err(GeopError::new(format!(
                    "more than two blended edges run into each other's walls at vertex {}: only two meet at an inward corner",
                    wa.vertex
                )));
            }
            used[a] = true;
            used[b] = true;
            pairs.push([a, b]);
        }
    }
    Ok(pairs)
}

/// Which faces meet at each end of `edge`, by name: what a blend that
/// cannot be built at one of them (see [`tool_end`]) says about it.
fn corners_of(model: &NamedModel, edge: EdgeId) -> String {
    match model.topology().get_edge(edge) {
        Ok(e) => corners_at(model, &[e.start_vertex, e.end_vertex]),
        Err(_) => format!("edge {edge}"),
    }
}

/// Which faces meet at each of `vertices`, by name, and which of them are
/// planes: what a blend that cannot end at one of them says about it.
fn corners_at(model: &NamedModel, vertices: &[VertexId]) -> String {
    let topology = model.topology();
    let name = |face: FaceId| {
        let named = model.name_of(face).map_or(format!("{face}"), String::from);
        match topology.get_face(face).map(|f| f.surface.as_plane()) {
            Ok(Ok(Some(_))) => format!("{named} (a plane)"),
            Ok(Ok(None)) => format!("{named} (curved)"),
            _ => named,
        }
    };
    vertices
        .iter()
        .map(|&vertex| {
            let faces = topology.faces_at_vertex(vertex).unwrap_or_default();
            format!(
                "vertex {} meets faces {}",
                model
                    .name_of(vertex)
                    .map_or(format!("{vertex}"), String::from),
                faces.into_iter().map(name).collect::<Vec<_>>().join(", ")
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}

/// Blends the edges named `edges`, all of one solid, into `shape` (see the
/// module docs), as the step whose names `namer` builds. The result is
/// named `namer`'s root, and the solid is consumed.
///
/// Each tool is named after the edge it was built for, `N(E,tool)`, and its
/// faces after their curves of the cross-section — the blend face itself
/// `N(E,fillet)` or `N(E,chamfer)` (`N(E,fillet,s0)`... where it is skinned
/// in several spans, see [`crate::rolling`]). What cutting it away creates is named
/// as a boolean names it, scoped by the edge: `N(E,...)`. Tools joined at
/// corners and mitres are one, named after the first.
///
/// A corner of three filleted edges that bend different ways — a pocket's
/// rim, cut away, and its upright edge, filled in; a boss's upright edge
/// and its foot — has no ball touching its faces on one side of them all.
/// The edge bending the other way from its two is rounded first: then the
/// two run on into each other along the round it left, one tangent chain,
/// and are rounded along it, round the corner (see [`passes`]).
pub fn blend(
    model: &mut NamedModel,
    namer: &Namer,
    edges: &[String],
    shape: &BlendShape,
) -> GeopResult<()> {
    if edges.is_empty() {
        return Err(GeopError::new("pick the edges to blend"));
    }
    let Some((first, then)) = passes(model, edges, shape)? else {
        return blend_once(model, namer, edges, shape);
    };
    blend_once(model, namer, &first, shape)
        .with_context(with_context!("rounding first the edges {first:?}, each bending the other way from the two it meets at a corner"))?;
    // What is left of the others: each as it was, or trimmed by the first
    // round — the pieces of it a tool of the first named (see the naming of
    // `geop_lib_booleans`) — or gone into a chain with a piece of another.
    let mut left: Vec<String> = Vec::new();
    for name in then {
        if model.edge_id(&name).is_ok() {
            left.push(name);
            continue;
        }
        let mut pieces: Vec<(EdgeId, String)> = (model.topology().edges.keys())
            .filter_map(|&e| {
                let piece = model.name_of(e)?;
                (first.iter())
                    .any(|f| namer.scoped(f).is_piece_of(piece, &name))
                    .then(|| (e, piece.to_string()))
            })
            .collect();
        pieces.sort_by_key(|(e, _)| e.0);
        left.extend(pieces.into_iter().map(|(_, piece)| piece));
    }
    if left.is_empty() {
        return Err(GeopError::new(format!(
            "rounding first the edges {first:?} left none of the others to round"
        )));
    }
    blend_once(model, namer, &left, shape)
        .with_context(with_context!("rounding then the edges {left:?}"))
}

/// The edges named `edges` in the two passes they are filleted in, where
/// some corner of three of them is of mixed convexity (see [`blend`]):
/// first each edge bending the other way from the two it meets at such a
/// corner, and with it every edge it reaches through corners every edge
/// of which is filleted and bends one way (rounded by a ball together);
/// then the rest. None where there is no such corner, or the shape is a
/// chamfer, whose corners are left to its chamfers.
fn passes(
    model: &NamedModel,
    edges: &[String],
    shape: &BlendShape,
) -> GeopResult<Option<(Vec<String>, Vec<String>)>> {
    if !matches!(shape, BlendShape::Fillet { .. }) {
        return Ok(None);
    }
    let topology = model.topology();
    // Each picked edge's chain, once, and which way it bends.
    let mut chains: Vec<(Vec<String>, Chain, Bend)> = Vec::new();
    for name in edges {
        let edge = model.edge_id(name)?;
        if let Some(c) = chains
            .iter_mut()
            .find(|(_, c, _)| c.edges().contains(&edge))
        {
            c.0.push(name.clone());
            continue;
        }
        let chain = tangent_chain(topology, edge)?;
        let bend = rolling::chain_bend(topology, &chain)?;
        chains.push((vec![name.clone()], chain, bend));
    }
    let chain_of = |edge: EdgeId| {
        chains
            .iter()
            .position(|(_, c, _)| c.edges().contains(&edge))
    };
    let mut vertices: Vec<VertexId> = Vec::new();
    for (_, chain, _) in &chains {
        for edge in chain.edges() {
            let e = topology.get_edge(edge)?;
            for v in [e.start_vertex, e.end_vertex] {
                if !vertices.contains(&v) {
                    vertices.push(v);
                }
            }
        }
    }
    vertices.sort_by_key(|v| v.0);
    // Chains joined by a corner rounded by a ball, and per chain whether it
    // is odd one out at a mixed corner (`Some(true)`) or one of the two
    // there (`Some(false)`).
    let mut group: Vec<usize> = (0..chains.len()).collect();
    let find = |group: &[usize], mut i: usize| {
        while group[i] != i {
            i = group[i];
        }
        i
    };
    let mut odd: Vec<Option<bool>> = vec![None; chains.len()];
    for vertex in vertices {
        let incident = topology.edges_at_vertex(vertex);
        let reaching: Option<Vec<usize>> = incident.iter().map(|&e| chain_of(e)).collect();
        let (Some(reaching), 3) = (reaching, incident.len()) else {
            continue;
        };
        let bends: Vec<Bend> = reaching.iter().map(|&c| chains[c].2).collect();
        if bends.iter().all(|&b| b == bends[0]) {
            for &c in &reaching[1..] {
                let (a, b) = (find(&group, reaching[0]), find(&group, c));
                group[b] = a;
            }
            continue;
        }
        for (k, &c) in reaching.iter().enumerate() {
            let alone = bends.iter().filter(|&&b| b == bends[k]).count() == 1;
            match odd[c] {
                Some(was) if was != alone => {
                    return Err(GeopError::new(format!(
                        "the fillet of edge {:?} bends the other way from the two it meets at one corner of mixed convexity, and the same way as another it meets at vertex {}: no order rounds both corners",
                        chains[c].0[0],
                        model.name_of(vertex).unwrap_or("?")
                    )));
                }
                _ => odd[c] = Some(alone),
            }
        }
    }
    if odd.iter().all(Option::is_none) {
        return Ok(None);
    }
    let first_groups: Vec<usize> = (0..chains.len())
        .filter(|&c| odd[c] == Some(true))
        .map(|c| find(&group, c))
        .collect();
    let (mut first, mut then) = (Vec::new(), Vec::new());
    for (c, (names, _, _)) in chains.iter().enumerate() {
        let early = first_groups.contains(&find(&group, c));
        if early && odd[c] == Some(false) {
            return Err(GeopError::new(format!(
                "the fillet of edge {:?} meets a corner of mixed convexity as one of its two edges bending one way, and is rounded by a ball together with an edge bending the other way from its two at another: no order rounds both corners",
                names[0]
            )));
        }
        if early {
            first.extend(names.iter().cloned());
        } else {
            then.extend(names.iter().cloned());
        }
    }
    Ok(Some((first, then)))
}

/// [`blend`], in one pass: every edge at a corner of three filleted edges
/// bending one way.
fn blend_once(
    model: &mut NamedModel,
    namer: &Namer,
    edges: &[String],
    shape: &BlendShape,
) -> GeopResult<()> {
    if let BlendShape::Fillet { radii } = shape {
        radii.check()?;
        for (vertex, _) in &radii.at_vertices {
            let ctx = with_context!("the radius at vertex {vertex:?}");
            model.vertex_id(vertex).with_context(ctx)?;
        }
    }
    let mut solid: Option<SolidId> = None;
    let mut covered: Vec<EdgeId> = Vec::new();
    let mut chains: Vec<(String, Chain)> = Vec::new();
    for name in edges {
        let ctx = with_context!("edge {name:?}");
        let edge = model.edge_id(name).with_context(ctx)?;
        let [face, _] = faces(model.topology(), edge).with_context(ctx)?;
        let of = model
            .solid_of_face(face, "only edges of a solid are blended")
            .with_context(ctx)?;
        match solid {
            None => solid = Some(of),
            Some(s) if s == of => {}
            Some(_) => {
                return Err(GeopError::new(
                    "the edges to blend are not all of one solid",
                ))
                .with_context(ctx);
            }
        }
        if covered.contains(&edge) {
            continue;
        }
        let chain = tangent_chain(model.topology(), edge)
            .map_err(|e| e.with_context(corners_of(model, edge)))
            .with_context(ctx)?;
        covered.extend(chain.edges());
        chains.push((name.clone(), chain));
    }
    let topology = model.topology();
    let corners = corner::plan_corners(model, namer, &chains, &covered, shape)?;

    // Inward corners: the ends running into a wall where another does.
    let mut walls = Vec::new();
    for (c, (name, chain)) in chains.iter().enumerate() {
        let ctx = with_context!("edge {name:?}");
        for (k, wall) in rolling::chain_walls(topology, chain)
            .with_context(ctx)?
            .into_iter()
            .enumerate()
        {
            if let (Some((vertex, faces)), None) = (wall, &corners.chains[c][k]) {
                walls.push(WallEnd {
                    tool: c,
                    at_end: k == 1,
                    vertex,
                    faces,
                });
            }
        }
    }
    let inward = inward_corners(&walls)?;

    let mut rolled: Vec<(String, Rolled)> = Vec::new();
    // Per tool, the faces it runs onto and is cut off by.
    let mut all_cuts: Vec<Vec<Cut>> = Vec::new();
    for ((name, chain), ends) in chains.into_iter().zip(corners.chains) {
        let ctx = with_context!("edge {name:?}");
        let ends_of: Vec<VertexId> = match chain.closed {
            true => Vec::new(),
            false => chain.ends(model.topology())?.map(|(v, _, _)| v).to_vec(),
        };
        let r = match shape {
            BlendShape::Fillet { radii } => {
                rolling::plan_rolled(model, chain, radii, ends, [false; 2])
            }
            BlendShape::Chamfer { distances } => {
                rolling::plan_chamfer(model, chain, *distances, [false; 2])
            }
        }
        .map_err(|e| e.with_context(corners_at(model, &ends_of)))
        .with_context(ctx)?;
        let crossings = rolling::check_touches(model.topology(), &r).with_context(ctx)?;
        all_cuts.push(cuts(model.topology(), &crossings, 2.0 * r.width()).with_context(ctx)?);
        rolled.push((name, r));
    }
    let tools = Tools {
        rolled: &rolled,
        corners: &corners.corners,
    };
    // The tools joined at corners are built whole, each such component one
    // solid, and applied by one boolean. The inward corners' tools run on
    // past them, planned on the solid as it is before the first tool.
    let mut mitres = Vec::new();
    for &[a, b] in &inward {
        let (a, b) = (&walls[a], &walls[b]);
        let ctx = format!(
            "the inward corner of edges {:?} and {:?} at vertex {}",
            tools.edge(a.tool),
            tools.edge(b.tool),
            a.vertex
        );
        let runs = mitre_runs(model, &tools, shape, [a, b]).with_context(with_context!("{ctx}"))?;
        // Named after the corner's vertex too: one edge may reach two.
        let corner = model.name_of(a.vertex).unwrap_or("?").to_string();
        let scope = namer
            .scoped(tools.edge(a.tool))
            .scoped("mitre")
            .scoped(&corner);
        mitres.push((scope, ctx, runs));
    }
    let components = tools.components(rolled.len());
    for component in &components {
        tools.check_apart(model.topology(), component)?;
    }
    let mut target = solid.expect("at least one edge");
    for component in components {
        let edge = tools.edge(component[0]);
        let ctx = with_context!("blending edge {edge:?}");
        // A chain whose tool's ends meet at one vertex, overlapping: its
        // halves one after the other (see [`Rolled::halves`]).
        if let Some(rolled) = tools.alone_rolled(&component)
            && rolled.is_halved()
        {
            for (k, half) in rolled.halves().with_context(ctx)?.iter().enumerate() {
                let scope = namer.scoped(edge).scoped(["1", "2"][k]);
                let ctx = with_context!("the tool of edge {edge:?}, half {}", k + 1);
                let mut tool =
                    tool::build_tools(model, &[(&scope, half)], &[], scope.name(&["tool"]))
                        .with_context(ctx)?;
                // Cut off at its own end, the piece past the face dropped.
                if let Some(surface) = &rolled.trims[k] {
                    let station = if k == 0 {
                        &half.stations[0]
                    } else {
                        half.stations.last().expect("stations")
                    };
                    tool = trimmed(model, &scope, tool, surface, &station.vertices[2])
                        .with_context(ctx)?;
                }
                for cut in &all_cuts[component[0]] {
                    tool = cut_off(model, &scope, tool, cut).with_context(ctx)?;
                }
                target = apply_tool(model, &scope, target, tool, rolled.bend).with_context(ctx)?;
            }
            continue;
        }
        let scope = namer.scoped(edge);
        let mut tool = tools
            .build(model, namer, &component)
            .with_context(with_context!("the tool of edge {edge:?}"))?;
        // Each rolled tool cut off by the faces it runs on through, the
        // piece holding its end past each dropped. (Not a piece kept by a
        // station between: the stations are where the spans break, and a
        // tool of one span has none but its ends.)
        for &i in &component {
            let (_, rolled) = &tools.rolled[i];
            let stations = &rolled.tool.stations;
            let ends = [&stations[0], stations.last().expect("stations")];
            for (surface, end) in rolled.trims.iter().zip(ends) {
                let Some(surface) = surface else {
                    continue;
                };
                tool = trimmed(model, &scope, tool, surface, &end.vertices[2])
                    .with_context(with_context!("the tool of edge {:?}", tools.edge(i)))?;
            }
        }
        // Each tool cut off by the faces it runs onto.
        for &i in &component {
            for cut in &all_cuts[i] {
                tool = cut_off(model, &scope, tool, cut)
                    .with_context(with_context!("the tool of edge {:?}", tools.edge(i)))?;
            }
        }
        let bend = tools.bend(component[0]);
        target = apply_tool(model, &scope, target, tool, bend).with_context(ctx)?;
    }
    // The inward corners, each cut by its two tools run on past it.
    for (scope, ctx, runs) in mitres {
        let ctx = with_context!("{ctx}");
        if let Some(tool) = mitre(model, &scope, &runs).with_context(ctx)? {
            target = apply_tool(model, &scope, target, tool, Bend::Convex).with_context(ctx)?;
        }
    }
    model.rename(target, namer.root())
}

/// The two tools cutting the inward corner where the tools ending at the
/// walls `ends` meet (see [`mitre`]): each replanned run on past its wall —
/// out through it, as where it left the solid — while the solid is as it
/// was before the first tool, which the rolling reads.
fn mitre_runs(
    model: &NamedModel,
    tools: &Tools,
    shape: &BlendShape,
    ends: [&WallEnd; 2],
) -> GeopResult<[Rolled; 2]> {
    let mut runs = Vec::new();
    for end in ends {
        let (_, rolled) = &tools.rolled[end.tool];
        let mut run_out = [false; 2];
        run_out[usize::from(end.at_end)] = true;
        let chain = rolled.chain.clone();
        let run = match shape {
            BlendShape::Fillet { radii } => {
                rolling::plan_rolled(model, chain, radii, [None, None], run_out)
            }
            BlendShape::Chamfer { distances } => {
                rolling::plan_chamfer(model, chain, *distances, run_out)
            }
        }?;
        if run.is_halved() {
            return Err(GeopError::new(
                "a blend whose chain starts and ends at one vertex meets another at an inward corner there: not supported",
            ));
        }
        runs.push(run);
    }
    let [a, b]: [Rolled; 2] = runs.try_into().map_err(|_| GeopError::new("two runs"))?;
    Ok([a, b])
}

/// What cuts an inward corner (see the module docs), named by `namer`: the
/// two tools `runs` run on past their walls, intersected. Each tool ends
/// flush with the other's wall, so what each would cut past it is cut
/// where both would: in front of both walls each cuts on its own, behind
/// both only where both do. None where they do nowhere.
fn mitre(model: &mut NamedModel, namer: &Namer, runs: &[Rolled; 2]) -> GeopResult<Option<SolidId>> {
    let mut solids = Vec::new();
    for (run, side) in runs.iter().zip(["a", "b"]) {
        let namer = namer.scoped(side);
        let mut tool =
            tool::build_tools(model, &[(&namer, &run.tool)], &[], namer.name(&["tool"]))?;
        let stations = &run.tool.stations;
        let past = [&stations[0], stations.last().expect("stations")];
        for (surface, station) in run.trims.iter().zip(past) {
            if let Some(surface) = surface {
                tool = trimmed(model, &namer, tool, surface, &station.vertices[2])?;
            }
        }
        solids.push(tool);
    }
    boolean(
        model,
        namer,
        solids[0],
        solids[1],
        BooleanOp::Intersection,
        RemeshParams::default(),
    )
}

/// The tool `tool` cut off by `surface` (see [`crate::rolling::Rolled::trims`]):
/// split along a sheet on it, the piece with a vertex at `past` — its end
/// run on past the face — and the sheet deleted, and the one other piece
/// kept. Named by `namer` as a split names what it makes.
fn trimmed(
    model: &mut NamedModel,
    namer: &Namer,
    tool: SolidId,
    surface: &NurbSurface3D,
    past: &Vector3,
) -> GeopResult<SolidId> {
    let (pieces, sheet) = split_by_sheet(model, &namer.scoped("trim"), tool, surface)?;
    let mut dropped = false;
    let mut kept = Vec::new();
    let mut rest = vec![Body::Sheet(sheet)];
    for piece in pieces {
        let topology = model.topology();
        let mut holds = false;
        for face in topology.solid_faces(piece)? {
            for coedge in topology.iterate_face_coedges(face) {
                holds |= topology
                    .coedge_start_vertex(coedge)?
                    .point
                    .could_be_equal(past);
            }
        }
        if holds && !dropped {
            dropped = true;
            rest.push(Body::Solid(piece));
        } else if holds {
            return Err(GeopError::new(format!(
                "the blend's tool cut off by a face it runs on through has two pieces holding its end past the face, at {past:?}"
            )));
        } else {
            kept.push(piece);
        }
    }
    if !dropped {
        return Err(GeopError::new(format!(
            "the blend's tool cut off by a face it runs on through came apart in no piece holding its end past the face, at {past:?}"
        )));
    }
    let [piece] = kept[..] else {
        return Err(GeopError::new(format!(
            "the blend's tool cut off by a face it runs on through came apart in {} pieces short of the face, not one",
            kept.len()
        )));
    };
    model.delete_bodies(&rest)?;
    Ok(piece)
}

/// `tool` split along a sheet on `surface`, named by `namer` as a split
/// names what it makes: the pieces, and the sheet.
fn split_by_sheet(
    model: &mut NamedModel,
    namer: &Namer,
    tool: SolidId,
    surface: &NurbSurface3D,
) -> GeopResult<(Vec<SolidId>, geop_core_topology::ShellId)> {
    let spec = BodySpec::sheet_on(surface.clone())?;
    let names = BodyNames {
        vertices: (0..4).map(|k| namer.name(&[&format!("v{k}")])).collect(),
        edges: (0..4).map(|k| namer.name(&[&format!("e{k}")])).collect(),
        faces: vec![namer.name(&["sheet"])],
        solid: None,
    };
    let built = model.build_body(spec, names)?;
    let sheet = *built
        .shells
        .first()
        .ok_or_else(|| GeopError::new("a sheet came out as no shell"))?;
    let pieces = split(model, namer, tool, sheet, RemeshParams::default())?;
    Ok((pieces, sheet))
}

/// Every tool of a blend, and the corners joining them.
struct Tools<'a> {
    rolled: &'a [(String, Rolled)],
    corners: &'a [Corner],
}

impl Tools<'_> {
    /// The name of the edge tool `i` was planned for.
    fn edge(&self, i: usize) -> &str {
        &self.rolled[i].0
    }

    /// The rolled tool `component` is, if it is one joined to no corner.
    fn alone_rolled(&self, component: &[usize]) -> Option<&Rolled> {
        let &[i] = component else {
            return None;
        };
        let joined = (self.corners.iter()).any(|c| c.ends.iter().any(|e| e.tool == i));
        (!joined).then_some(&self.rolled[i].1)
    }

    /// The vertices tool `i` ends at on its own — run out, flush or at a
    /// wall, not joined to a corner's ball.
    fn free_ends(&self, model: &Model, i: usize) -> GeopResult<Vec<VertexId>> {
        let chain = &self.rolled[i].1.chain;
        let vertices = if chain.closed {
            Vec::new()
        } else {
            chain.ends(model)?.map(|(v, _, _)| v).to_vec()
        };
        let joined = |at_end: bool| {
            let at = |e: &CornerEnd| e.tool == i && e.at_end == at_end;
            self.corners.iter().any(|c| c.ends.iter().any(at))
        };
        Ok(vertices
            .into_iter()
            .zip([false, true])
            .filter(|&(_, at_end)| !joined(at_end))
            .map(|(vertex, _)| vertex)
            .collect())
    }

    /// Checks no two tools of `component`, built into one solid, end on
    /// their own at one vertex: there both run out — or end flush — over
    /// the same corner, and overlap, which one solid cannot (the blend of a
    /// cylinder's flat corner: the top rim and the flat's top edge, joined
    /// at one end of the flat, both run out at its other end). Applied one
    /// after the other they would not, but tools joined at a corner are one.
    fn check_apart(&self, model: &Model, component: &[usize]) -> GeopResult<()> {
        let mut seen: Vec<(VertexId, usize)> = Vec::new();
        for &i in component {
            for vertex in self.free_ends(model, i)? {
                if let Some(&(_, j)) = seen.iter().find(|(v, j)| *v == vertex && *j != i) {
                    return Err(GeopError::new(format!(
                        "edges {:?} and {:?} are blended together, joined at a corner, and both end on their own at vertex {vertex}, where their blends would overlap in one tool: only blends joined at one end and apart at the other are built — round the third edge at vertex {vertex} too, or blend them one at a time",
                        self.edge(j),
                        self.edge(i)
                    )));
                }
                seen.push((vertex, i));
            }
        }
        Ok(())
    }

    /// Which way the edge of tool `i` bends.
    fn bend(&self, i: usize) -> Bend {
        self.rolled[i].1.bend
    }

    /// The tools `0..all` in components: those joined at corners together;
    /// each other on its own.
    fn components(&self, all: usize) -> Vec<Vec<usize>> {
        let mut components: Vec<Vec<usize>> = (0..all).map(|i| vec![i]).collect();
        let links =
            (self.corners.iter()).map(|c| c.ends.iter().map(|e| e.tool).collect::<Vec<_>>());
        for tools in links {
            let mut joined: Vec<usize> = Vec::new();
            components.retain(|c| {
                if tools.iter().any(|t| c.contains(t)) {
                    joined.extend(c);
                    false
                } else {
                    true
                }
            });
            joined.sort();
            components.push(joined);
        }
        components.sort_by_key(|c| c[0]);
        components
    }

    /// The tools `component` — joined at corners, or one on its own —
    /// built into one solid with the corners (see [`tool::build_tools`]),
    /// named after the first one's tool.
    fn build(
        &self,
        model: &mut NamedModel,
        namer: &Namer,
        component: &[usize],
    ) -> GeopResult<SolidId> {
        let corners: Vec<&Corner> = self
            .corners
            .iter()
            .filter(|c| c.ends.iter().any(|e| component.contains(&e.tool)))
            .collect();
        let namers: Vec<Namer> = component
            .iter()
            .map(|&i| namer.scoped(self.edge(i)))
            .collect();
        // Each tool, and the faces its section touches first and second at
        // its first and last station.
        let mut tools = Vec::new();
        let mut touching = Vec::new();
        for &i in component {
            let rolled = &self.rolled[i].1;
            tools.push(rolled.tool.clone());
            touching.push([rolled.contact_faces(false), rolled.contact_faces(true)]);
        }
        let index = |tool: usize| {
            component
                .iter()
                .position(|&i| i == tool)
                .ok_or_else(|| GeopError::new("a corner joins tools of another component"))
        };
        // The corners' ends, by tool of the component, and the contacts
        // their sections run between.
        let mut joined_corners = Vec::new();
        for &corner in &corners {
            let mut corner = corner.clone();
            for end in &mut corner.ends {
                let t = index(end.tool)?;
                let contact = |face: FaceId| corner_contact(&corner.faces, face);
                let faces = touching[t][usize::from(end.at_end)];
                end.contacts = [contact(faces[0])?, contact(faces[1])?];
                end.tool = t;
            }
            joined_corners.push(corner);
        }
        let refs: Vec<(&Namer, &Tool)> = namers.iter().zip(&tools).collect();
        tool::build_tools(model, &refs, &joined_corners, namers[0].name(&["tool"]))
    }
}

/// Which of a corner's contacts, the corner touching `faces`, lies on
/// `face`.
fn corner_contact(faces: &[FaceId], face: FaceId) -> GeopResult<usize> {
    faces.iter().position(|&f| f == face).ok_or_else(|| {
        GeopError::new(format!(
            "a tool ending at a corner touches face {face}, which the corner's ball does not"
        ))
    })
}

/// Cuts `tool` away from `target`, or fills it in, as `bend` says — named
/// by `scope`: the solid that is left. The blend face, the one piece of
/// its tool kept, keeps its own name (see the naming of
/// `geop_lib_booleans`).
///
/// A fill capped flush with a face it ends at — a pocket's upright edge
/// filled up to the top — leaves its cap a face of its own in that face's
/// surface, and the edges between them on the solid, where nothing bends:
/// each such piece of the tool is made one with the face beside it.
fn apply_tool(
    model: &mut NamedModel,
    scope: &Namer,
    target: SolidId,
    tool: SolidId,
    bend: Bend,
) -> GeopResult<SolidId> {
    let tool_faces: Vec<String> = (model.topology().solid_faces(tool)?.into_iter())
        .filter_map(|f| model.name_of(f).map(str::to_string))
        .collect();
    let solid = boolean(
        model,
        scope,
        target,
        tool,
        bend.op(),
        RemeshParams::default(),
    )?
    .ok_or_else(|| GeopError::new("the blend cut the whole solid away"))?;
    if bend == Bend::Convex {
        return Ok(solid);
    }
    let from_tool: Vec<FaceId> = (model.topology().solid_faces(solid)?.into_iter())
        .filter(|&f| {
            model.name_of(f).is_some_and(|name| {
                tool_faces
                    .iter()
                    .any(|t| name == t || scope.is_piece_of(name, t))
            })
        })
        .collect();
    match model
        .merge_faces_on_one_surface(solid.into(), |f, g| {
            from_tool.contains(&f) != from_tool.contains(&g)
        })
        .with_context(with_context!(
            "making the fill's faces one with those beside them in one surface"
        ))? {
        Body::Solid(solid) => Ok(solid),
        Body::Sheet(_) => Err(GeopError::new(
            "making the fill's faces one with those beside them left no solid",
        )),
    }
}
