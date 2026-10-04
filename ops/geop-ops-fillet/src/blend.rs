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
//! The tool is the cross-section swept along the edge:
//!
//! - a **straight** edge between two planes is extruded along it. The edge
//!   has to end at corners of exactly one more face, a plane. Where the edge
//!   leaves the solid through it, a cut is run out past it, out of the
//!   solid, and a fill ends flush with it. Where the edge runs into it — a
//!   wall the material goes on behind — cut and fill both end flush with
//!   it, as running on would blend what lies behind. A flush end needs that
//!   face square to the edge — but for a mitre (below), which ends the tool
//!   at any wall. Anything else is refused rather than blended wrongly.
//! - a **circular** edge between faces turning around its axis — planes
//!   across the axis, cylinders and cones on it — is revolved a full turn.
//!   So the edge has to be part of a whole circle of such edges (a circle
//!   is built of quarter arcs): the whole circle is blended, once, whichever
//!   of its arcs are picked.
//! - **any other edge** — or any edge whose fillet radius varies — is
//!   rounded by a ball rolled along it and its tangent chain, its tool
//!   skinned through the ball's sections (see [`crate::rolling`]); bevelled
//!   alike, its sections the chords.
//!
//! A blend has to meet both faces inside them — checked halfway along the
//! edge — so one too large for its faces, or ending exactly on another of
//! their edges, is refused.
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
//!   walls, a pocket's rim — stopping flush would leave the corner between
//!   the walls uncut, so the two tools are *mitred* instead: each runs on
//!   to the plane halving the corner, where their sections are one, and
//!   they are joined there into one tool (see [`tool::build_tools`]). That
//!   is exact where their cross-sections mirror each other in that plane;
//!   elsewhere it is refused, and so are rolled blends meeting there.
//!
//! Tools joined at corners and mitres are built whole, each such set one
//! solid, and applied by one boolean.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::NurbCurve3D,
    nurb_surface::NurbSurface3D,
    shape::{Axis, Circle},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    Body, CoedgeId, EdgeId, FaceId, Model, Sense, SolidId, VertexId,
    contains::face::{PointClassification, face_contains},
};
use geop_ops::{Namer, Part};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::{
    common::{Profile, arc2, line2},
    extrude::extrude,
    revolve::revolve,
    sweep::SweepLoop,
};

use crate::corner;
use crate::rolling::{self, Chain, Radii, Rolled, normal_at, tangent_chain};
use crate::tool::{self, Cap, Corner, CornerEnd, Joint, Tool, ToolSpan, ToolStation, embed};

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
    pub fn round(radius: f64) -> Self {
        BlendShape::Fillet {
            radii: Radii::constant(radius),
        }
    }

    /// The radius, where it is the same all along: what the swept blends
    /// of straight and circular edges need.
    fn constant_radius(&self) -> Option<f64> {
        match self {
            BlendShape::Fillet { radii } if radii.is_constant() => Some(radii.radius),
            _ => None,
        }
    }
}

/// How an edge's cross-section is swept into a tool.
#[derive(Clone, Debug)]
pub(crate) enum Sweep<S: Scalar> {
    /// Along the straight edge from `start` to `end`, the section's `(x,
    /// y)` at `start + x e1 + y e2`.
    Straight {
        start: Vector3<S>,
        end: Vector3<S>,
        e1: Vector3<S>,
        e2: Vector3<S>,
        ends: [End<S>; 2],
        /// The vertices the edge starts and ends at.
        vertices: [VertexId; 2],
    },
    /// A full turn around the circle's axis: the section's `(x, y)` are
    /// `(r, z)`, `r` along `radial` from `center` and `z` along `axis`.
    Round {
        center: Vector3<S>,
        radial: Vector3<S>,
        axis: Vector3<S>,
    },
}

/// How a tool along a straight edge — or a rolling-ball blend's open chain
/// (see [`crate::rolling`]) — ends, at one end of it.
#[derive(Clone, Debug)]
pub(crate) enum End<S: Scalar> {
    /// Run out past the end — as far again as the tool is wide, over how
    /// squarely the edge leaves the solid there: `d · n` of the face it
    /// leaves through, `d` running on along the edge. What a cut does where
    /// the edge leaves the solid.
    RunOut { squareness: S },
    /// Flush with the face square to the edge at the end: what material
    /// added does, which must not stick out past it.
    Flush,
    /// Flush with a wall the edge runs into, square to it: what a cut does
    /// there, which must not cut what lies behind the wall — unless the
    /// wall's own edge along the same face is blended too: then the two
    /// are mitred. Why it cannot end flush, `unless` the wall is a plane
    /// square to the edge: then only a mitre ends it.
    Wall { unless: Option<String> },
    /// Run out past the corner at `at` and cut off at the plane through it
    /// halving the corner, keeping the side `keep` points to: the end of a
    /// tool mitred with another's.
    Mitre { at: Vector3<S>, keep: Vector3<S> },
    /// Cut off square to the edge `setback` along it from its start, at
    /// the plane through the center of the ball rounding a corner every
    /// edge of which is filleted: where the tool is joined to the corner's
    /// (see [`crate::corner`]).
    Corner { setback: S },
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
    pub(crate) fn side<S: Scalar>(self) -> S {
        match self {
            Bend::Convex => S::ONE.neg(),
            Bend::Concave => S::ONE,
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

/// An edge's corner, in its cross-section (see the module docs): which way
/// it bends, the corner `E`, and for the face on the edge's left (`[0]`)
/// and right (`[1]`) the face, the direction running into it and the one
/// facing out of the solid.
#[derive(Clone, Debug)]
pub(crate) struct Section<S: Scalar> {
    pub(crate) sweep: Sweep<S>,
    pub(crate) bend: Bend,
    pub(crate) faces: [FaceId; 2],
    corner: Vector2<S>,
    into: [Vector2<S>; 2],
    out: [Vector2<S>; 2],
}

impl<S: Scalar> Section<S> {
    /// The section's point `p` in space, in the section halfway along the
    /// edge — where a round section is taken.
    fn point(&self, p: &Vector2<S>) -> GeopResult<Vector3<S>> {
        Ok(match &self.sweep {
            Sweep::Straight {
                start, end, e1, e2, ..
            } => Vector3::interpolate(start, end, S::ONE.div(S::TWO)?)
                .add(&e1.prod_scalar(p[0]))
                .add(&e2.prod_scalar(p[1])),
            Sweep::Round {
                center,
                radial,
                axis,
            } => center
                .add(&radial.prod_scalar(p[0]))
                .add(&axis.prod_scalar(p[1])),
        })
    }
}

/// The 3-D vector `v`, in the section plane spanned by the unit vectors
/// `e1` and `e2`.
fn in_plane<S: Scalar>(v: &Vector3<S>, e1: &Vector3<S>, e2: &Vector3<S>) -> Vector2<S> {
    Vector2::from_array([v.prod_dot(e1), v.prod_dot(e2)])
}

/// The point the homogeneous control point `p` stands for.
fn point<S: Scalar>(p: &Vector4<S>) -> GeopResult<Vector3<S>> {
    Ok(Vector3::from_array([
        p[0].div(p[3])?,
        p[1].div(p[3])?,
        p[2].div(p[3])?,
    ]))
}

/// The two coedges of `edge`: the one running along it (the face on its
/// left) first, the one running against it second.
fn coedges<S: Scalar>(model: &Model<S>, edge: EdgeId) -> GeopResult<[CoedgeId; 2]> {
    let mut forward = None;
    let mut reversed = None;
    for c in model.coedges_of_edge(edge) {
        match model.get_coedge(c)?.sense {
            Sense::Forward => forward = Some(c),
            Sense::Reversed => reversed = Some(c),
        }
    }
    match (forward, reversed) {
        (Some(f), Some(r)) => Ok([f, r]),
        _ => Err(GeopError::new(format!(
            "edge {edge} does not join two faces, so it has no corner to blend"
        ))),
    }
}

/// The two faces of `edge`, left first (see [`coedges`]).
pub(crate) fn faces<S: Scalar>(model: &Model<S>, edge: EdgeId) -> GeopResult<[FaceId; 2]> {
    let [l, r] = coedges(model, edge)?;
    Ok([model.get_coedge(l)?.face, model.get_coedge(r)?.face])
}

/// The unit outward normal `n` of `face` at the edge, and the unit
/// direction `n x c` running into it, `c` the direction the face's coedge
/// runs along the edge there.
fn face_frame<S: Scalar>(n: &Vector3<S>, c: &Vector3<S>) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let n = n.normalize()?;
    Ok((n, n.prod_cross(c).normalize()?))
}

/// Which way an edge bends, from `bend` — the direction running into the
/// face on its left, dotted with the right one's outward normal: convex
/// where the left face runs into the side of the right one's tangent plane
/// the solid lies on. An error if the faces could be tangent there — there
/// is no corner to blend.
pub(crate) fn bend<S: Scalar>(bend: S) -> GeopResult<Bend> {
    if bend.definitely_less(S::ZERO) {
        Ok(Bend::Convex)
    } else if bend.definitely_greater(S::ZERO) {
        Ok(Bend::Concave)
    } else {
        Err(GeopError::new(
            "the faces could meet tangentially at the edge: there is no corner to blend",
        ))
    }
}

/// The cross-section of the straight edge `edge`, from `start` to `end`,
/// between two planes (see the module docs).
fn straight_section<S: Scalar>(
    model: &Model<S>,
    edge: EdgeId,
    line: &Axis<S>,
) -> GeopResult<Section<S>> {
    let e = model.get_edge(edge)?;
    let start = model.get_vertex(e.start_vertex)?.point;
    let end = model.get_vertex(e.end_vertex)?.point;
    let d = end.sub(&start).normalize()?;
    let [left, right] = faces(model, edge)?;
    let mut normals = Vec::new();
    for face in [left, right] {
        let Some(plane) = model.get_face(face)?.surface.as_plane()? else {
            return Err(GeopError::new(format!(
                "face {face} is not planar: a straight edge is only blended between two planes"
            )));
        };
        normals.push(plane.normal);
    }
    let (n_l, t_l) = face_frame(&normals[0], &d)?;
    let (n_r, t_r) = face_frame(&normals[1], &d.neg())?;
    let basis = line.direction.orthonormal_complement()?;
    let (e1, e2) = (basis[0], basis[1]);
    let into = [in_plane(&t_l, &e1, &e2), in_plane(&t_r, &e1, &e2)];
    let out = [in_plane(&n_l, &e1, &e2), in_plane(&n_r, &e1, &e2)];
    let bend = bend(into[0].prod_dot(&out[1]))?;
    let mut ends = Vec::new();
    for (vertex, leaving) in [(e.start_vertex, d.neg()), (e.end_vertex, d)] {
        let ctx = with_context!("the edge's end at vertex {vertex}");
        let (end, _) = tool_end(model, vertex, [left, right], &leaving, bend).with_context(ctx)?;
        ends.push(end);
    }
    let ends = [ends[0].clone(), ends[1].clone()];
    Ok(Section {
        sweep: Sweep::Straight {
            start,
            end,
            e1,
            e2,
            ends,
            vertices: [e.start_vertex, e.end_vertex],
        },
        bend,
        faces: [left, right],
        corner: Vector2::from_array([S::ZERO, S::ZERO]),
        into,
        out,
    })
}

/// How the tool of a straight edge between `faces`, bending `bend`, ends at
/// the edge's end `vertex`, the edge running on along `leaving` (see the
/// module docs). It is decided by the one other face meeting there, which
/// has to be a plane: run out past it where the edge leaves the solid
/// through it and the tool cuts, flush with it otherwise — where the tool
/// fills, or the edge runs into it — which needs it square to the edge.
/// Anything else, and a tool along the edge would cut or fill what it
/// should not. Only a wall the tool runs into may be any face, if it is
/// mitred with the blend of that wall's own edge. The end, and that third
/// face.
pub(crate) fn tool_end<S: Scalar>(
    model: &Model<S>,
    vertex: VertexId,
    faces: [FaceId; 2],
    leaving: &Vector3<S>,
    bend: Bend,
) -> GeopResult<(End<S>, FaceId)> {
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
    let unsupported = |why: &str| {
        Err(GeopError::new(format!(
            "the edge ends at vertex {vertex}, where {why}: only an edge ending at a corner of exactly three faces, the third a plane, is blended"
        )))
    };
    let [other] = others.as_slice() else {
        return unsupported(&format!("{} more faces meet", others.len()));
    };
    let plane = model.get_face(*other)?.surface.as_plane()?;
    let normal = match &plane {
        Some(plane) => plane.normal.normalize()?,
        None => normal_at(model, *other, &model.get_vertex(vertex)?.point)?,
    };
    let squareness = leaving.prod_dot(&normal);
    let leaves = if squareness.definitely_greater(S::ZERO) {
        true
    } else if squareness.definitely_less(S::ZERO) {
        false
    } else {
        return unsupported("the third face could run along the edge");
    };
    let why = if plane.is_none() {
        Some("the third face is not planar")
    } else if !normal.prod_cross(leaving).norm_sq().could_be_equal(S::ZERO) {
        Some(if leaves {
            "the edge is filled in up to the third face, which does not stand square to it"
        } else {
            "the edge runs into the third face, which does not stand square to it"
        })
    } else {
        None
    };
    if leaves && bend == Bend::Convex {
        if plane.is_none() {
            return unsupported("the third face is not planar");
        }
        return Ok((End::RunOut { squareness }, *other));
    }
    if leaves || bend == Bend::Concave {
        if let Some(why) = why {
            return unsupported(why);
        }
        return Ok((End::Flush, *other));
    }
    let unless = why.map(|why| {
        format!(
            "the edge ends at vertex {vertex}, where {why}: only an edge ending at a corner of exactly three faces, the third a plane, is blended — or one running into a wall whose own edge along the same face is blended too"
        )
    });
    Ok((End::Wall { unless }, *other))
}

/// The circle `curve` is an arc of, if it is one.
fn circle_of<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Option<Circle<S>>> {
    Ok(curve.as_arc()?.map(|arc| arc.circle))
}

/// Whether `a` and `b` could be one circle, either way round.
fn same_circle<S: Scalar>(a: &Circle<S>, b: &Circle<S>) -> bool {
    a.center.could_be_equal(&b.center)
        && a.radius.could_be_equal(b.radius)
        && a.normal
            .prod_cross(&b.normal)
            .norm_sq()
            .could_be_equal(S::ZERO)
}

/// The outward normal of `face`, turning around `circle`'s axis, at the
/// point `p` of the circle — with `radial` the unit direction from the
/// center to `p` — or none where the face is not one a revolved blend
/// handles. A plane has to stand across the axis; any other face has to be
/// a surface of revolution around it whose profile is one straight line (a
/// cylinder, a cone), so that it traces a line in every cross-section.
fn round_face_normal<S: Scalar>(
    surface: &NurbSurface3D<S>,
    circle: &Circle<S>,
    radial: &Vector3<S>,
) -> GeopResult<Option<Vector3<S>>> {
    let axis = &circle.normal;
    if let Some(plane) = surface.as_plane()? {
        let across = plane
            .normal
            .prod_cross(axis)
            .norm_sq()
            .could_be_equal(S::ZERO);
        return Ok(across.then_some(plane.normal));
    }
    let Some(turns) = surface.axis_of_revolution()? else {
        return Ok(None);
    };
    if !turns
        .direction
        .prod_cross(axis)
        .norm_sq()
        .could_be_equal(S::ZERO)
        || !turns.could_contain(&circle.center)
    {
        return Ok(None);
    }
    // The profile: the one straight direction, of two rows. Its first
    // control point lies on the surface, where the profile starts.
    let profile = if surface.degree_u == 1 && surface.num_u == 2 {
        [0, surface.num_v]
    } else if surface.degree_v == 1 && surface.num_v == 2 {
        [0, 1]
    } else {
        return Ok(None);
    };
    let start = point(&surface.control_points[profile[0]])?;
    // The normal where the profile starts, in its own half-plane through the
    // axis — the same in every half-plane, turned.
    let (u0, v0) = (surface.domain_u().0, surface.domain_v().0);
    let normal = surface.normal(u0, v0)?;
    let off = start.sub(&circle.center);
    let off_axis = off.sub(&axis.prod_scalar(off.prod_dot(axis)));
    let Ok(start_radial) = off_axis.normalize() else {
        return Ok(None);
    };
    let (nr, nz) = (normal.prod_dot(&start_radial), normal.prod_dot(axis));
    Ok(Some(radial.prod_scalar(nr).add(&axis.prod_scalar(nz))))
}

/// The cross-section of the circular edge `edge`, an arc of `circle`,
/// halfway along it — none where one of its faces is not one a revolved
/// blend handles (see [`round_face_normal`]).
fn round_section<S: Scalar>(
    model: &Model<S>,
    edge: EdgeId,
    circle: &Circle<S>,
) -> GeopResult<Option<Section<S>>> {
    let e = model.get_edge(edge)?;
    let (t0, t1) = e.curve.domain();
    let t = t0.add(t1).div(S::TWO)?;
    let p = e.curve.evaluate(t)?;
    let c = e.curve.tangent(t)?.normalize()?;
    let axis = circle.normal;
    let off = p.sub(&circle.center);
    let radial = off
        .sub(&axis.prod_scalar(off.prod_dot(&axis)))
        .normalize()?;
    let [left, right] = faces(model, edge)?;
    let Some(n_l) = round_face_normal(&model.get_face(left)?.surface, circle, &radial)? else {
        return Ok(None);
    };
    let Some(n_r) = round_face_normal(&model.get_face(right)?.surface, circle, &radial)? else {
        return Ok(None);
    };
    let (n_l, t_l) = face_frame(&n_l, &c)?;
    let (n_r, t_r) = face_frame(&n_r, &c.neg())?;
    let into = [
        in_plane(&t_l, &radial, &axis),
        in_plane(&t_r, &radial, &axis),
    ];
    let out = [
        in_plane(&n_l, &radial, &axis),
        in_plane(&n_r, &radial, &axis),
    ];
    let bend = bend(into[0].prod_dot(&out[1]))?;
    Ok(Some(Section {
        sweep: Sweep::Round {
            center: circle.center,
            radial,
            axis,
        },
        bend,
        faces: [left, right],
        corner: in_plane(&off, &radial, &axis),
        into,
        out,
    }))
}

/// The edges of the whole circle `edge` is an arc of, `edge` first: walking
/// on from its end, through every vertex to the one other edge there that
/// is an arc of the same circle, back to it. An error if the arcs do not
/// close the circle — a blend revolves a full turn, which would cut where
/// nothing was picked — or branch.
fn circle_edges<S: Scalar>(
    model: &Model<S>,
    edge: EdgeId,
    circle: &Circle<S>,
) -> GeopResult<Vec<EdgeId>> {
    let mut chain = vec![edge];
    let mut vertex = model.get_edge(edge)?.end_vertex;
    loop {
        let mut next: Vec<EdgeId> = Vec::new();
        for (&id, e) in &model.edges {
            if (e.start_vertex == vertex || e.end_vertex == vertex)
                && id != *chain.last().expect("never empty")
                && matches!(circle_of(&e.curve)?, Some(c) if same_circle(&c, circle))
            {
                next.push(id);
            }
        }
        let [next] = next.as_slice() else {
            return Err(GeopError::new(format!(
                "at vertex {vertex}, the circle of the edge goes on along {} arcs: only edges of a whole circle are blended",
                next.len()
            )));
        };
        if *next == edge {
            return Ok(chain);
        }
        if chain.contains(next) {
            return Err(GeopError::new(format!(
                "the arcs of the circle of the edge run round in a loop not through it, at vertex {vertex}"
            )));
        }
        chain.push(*next);
        let e = model.get_edge(*next)?;
        vertex = if e.start_vertex == vertex {
            e.end_vertex
        } else {
            e.start_vertex
        };
    }
}

/// Whether two sections could describe the same corner: every arc of a
/// circle has to be blended alike, or one tool cannot blend them all.
fn same_corner<S: Scalar>(a: &Section<S>, b: &Section<S>) -> bool {
    a.corner.could_be_equal(&b.corner)
        && (0..2)
            .all(|i| a.into[i].could_be_equal(&b.into[i]) && a.out[i].could_be_equal(&b.out[i]))
}

/// The closed cross-section of a tool, counter-clockwise, the control
/// polygon it lies in — for a fillet the arc's three control points and the
/// apex — and where its blend meets the face on the edge's left and right.
pub(crate) struct ToolProfile<S: Scalar> {
    profile: Profile<S>,
    control: Vec<Vector2<S>>,
    touches: [Vector2<S>; 2],
    /// The weight of the arc's middle control point, `control[1]`: one
    /// for a chamfer's chord.
    weight: S,
}

/// The cross-section of the tool that blends `section`'s corner into
/// `shape` (see the module docs).
fn tool_profile<S: Scalar>(section: &Section<S>, shape: &BlendShape) -> GeopResult<ToolProfile<S>> {
    let e = section.corner;
    let (curves, control, touches, weight) = match shape {
        BlendShape::Fillet { radii } => {
            radii.check()?;
            let r = S::from_f64(radii.radius);
            let [t_a, t_b] = &section.into;
            let [n_a, n_b] = &section.out;
            // `C = E + alpha t_a + beta t_b`, `r` from both lines on the
            // side `s` of them: `(C - E) · n_a = s r = (C - E) · n_b`.
            let sr = section.bend.side::<S>().mul(r);
            let alpha = sr.div(t_a.prod_dot(n_b))?;
            let beta = sr.div(t_b.prod_dot(n_a))?;
            let center = e.add(&t_a.prod_scalar(alpha).add(&t_b.prod_scalar(beta)));
            let tangent_a = center.sub(&n_a.prod_scalar(sr));
            let tangent_b = center.sub(&n_b.prod_scalar(sr));
            let weight = r.div(e.sub(&center).norm())?;
            let apex = e.prod_scalar(S::TWO).sub(&center);
            (
                vec![
                    ("fillet", arc2(tangent_a, e, tangent_b, weight)?),
                    ("b", line2(tangent_b, apex)?),
                    ("a", line2(apex, tangent_a)?),
                ],
                vec![tangent_a, e, tangent_b, apex],
                [tangent_a, tangent_b],
                weight,
            )
        }
        BlendShape::Chamfer { distances } => {
            if !distances.iter().all(|&d| d > 0.0) {
                return Err(GeopError::new(format!(
                    "a chamfer's distances have to be positive, not {distances:?}"
                )));
            }
            let [d_a, d_b] = distances.map(S::from_f64);
            let a = e.add(&section.into[0].prod_scalar(d_a));
            let b = e.add(&section.into[1].prod_scalar(d_b));
            // The chord, extended past both ends, and the apex as far beyond
            // the corner as the chord's middle is before it.
            let a_out = a.prod_scalar(S::TWO).sub(&b);
            let b_out = b.prod_scalar(S::TWO).sub(&a);
            let middle = a.add(&b).prod_scalar(S::ONE.div(S::TWO)?);
            let apex = e.prod_scalar(S::TWO).sub(&middle);
            (
                vec![
                    ("chamfer", line2(a_out, b_out)?),
                    ("b", line2(b_out, apex)?),
                    ("a", line2(apex, a_out)?),
                ],
                vec![a_out, b_out, apex],
                [a, b],
                S::ONE,
            )
        }
    };
    let profile = Profile {
        curve_names: curves.iter().map(|(n, _)| n.to_string()).collect(),
        joint_names: ["ta", "tb", "q"].map(String::from).to_vec(),
        curves: curves.into_iter().map(|(_, c)| c).collect(),
    };
    // Which way round: the sign of the control polygon's area, which winds
    // the way the profile does — the arc stays inside its control triangle.
    let area = (0..control.len()).fold(S::ZERO, |sum, i| {
        sum.add(control[i].prod_cross(&control[(i + 1) % control.len()]))
    });
    if area.definitely_greater(S::ZERO) {
        Ok(ToolProfile {
            profile,
            control,
            touches,
            weight,
        })
    } else if area.definitely_less(S::ZERO) {
        Ok(ToolProfile {
            profile: profile.reversed(),
            control,
            touches,
            weight,
        })
    } else {
        Err(GeopError::new(format!(
            "the blend's cross-section could have no area ({area:?})"
        )))
    }
}

/// Seed of the ray casting behind `face_contains` here, whose answer does
/// not depend on it — a constant only keeps runs reproducible.
const SEED: u64 = 0xB1E2_D000_0000_0001;

/// Checks the blend of `section` by `tool` meets each face where the face
/// is: inside it, halfway along the edge (see [`check_touch`]).
fn check_touches<S: Scalar>(
    model: &Model<S>,
    section: &Section<S>,
    tool: &ToolProfile<S>,
) -> GeopResult<()> {
    for (face, touch) in section.faces.iter().zip(&tool.touches) {
        let p = section.point(touch)?;
        let ctx = with_context!("where the blend meets face {face}, at {p:?}");
        check_touch(model, *face, &p).with_context(ctx)?;
    }
    Ok(())
}

/// Checks a blend touching `face` at `p` touches it where the face is:
/// inside it. A blend too large for a face runs past its boundary, and one
/// ending exactly on another of its edges asks the boolean to cut along
/// that edge.
pub(crate) fn check_touch<S: Scalar>(
    model: &Model<S>,
    face: FaceId,
    p: &Vector3<S>,
) -> GeopResult<()> {
    let params = RemeshParams::<S>::default();
    let (max_nodes, size) = (params.max_nodes, params.min_subdivision_size);
    let surface = &model.get_face(face)?.surface;
    let Some((u, v)) = surface_could_contain(surface, p, max_nodes, size)? else {
        return Err(GeopError::new(format!(
            "the blend is too large for face {face}: it would meet the face's surface beyond its end"
        )));
    };
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

/// The largest distance of a point of `control` from `from`, as a plain
/// number: how wide the tool is, for how far to run it out — a free choice,
/// only ever rounded up.
fn reach<S: Scalar>(control: &[Vector2<S>], from: &Vector2<S>) -> f64 {
    control
        .iter()
        .map(|p| p.sub(from).norm().upper().to_f64())
        .fold(0.0, f64::max)
}

/// Sweeps `tool` as `section` says into a solid named `N(tool)`, its faces
/// and edges named by `namer`.
fn sweep_tool<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    section: &Section<S>,
    tool: &ToolProfile<S>,
) -> GeopResult<SolidId> {
    let root = namer.name(&["tool"]);
    let loops = [SweepLoop::plain(tool.profile.clone())];
    let built = match &section.sweep {
        Sweep::Straight {
            start,
            end,
            e1,
            e2,
            ends,
            ..
        } => {
            let d = end.sub(start);
            let length = d.norm();
            let plane = CoordinateSystem::try_new(*start, *e1, *e2, d.normalize()?)?;
            let width = reach(&tool.control, &section.corner);
            // As far again as the tool is wide, over how squarely the edge
            // crosses the face it is run out past.
            let run_out = |end: &End<S>| -> GeopResult<Option<S>> {
                match end {
                    End::RunOut { squareness } => {
                        Ok(Some(S::from_f64(2.0 * width / squareness.lower().to_f64())))
                    }
                    End::Flush | End::Wall { .. } => Ok(None),
                    End::Mitre { .. } | End::Corner { .. } => Err(GeopError::new(
                        "a tool joined to another at a corner or a mitre is built whole, not swept",
                    )),
                }
            };
            let from = run_out(&ends[0])?.map_or(S::ZERO, |r| r.neg());
            let to = run_out(&ends[1])?.map_or(length, |r| length.add(r));
            extrude(part, namer, Some(&root), &plane, from, to, &loops)?
        }
        Sweep::Round {
            center,
            radial,
            axis,
        } => {
            if !tool
                .control
                .iter()
                .all(|p| p[0].definitely_greater(S::ZERO))
            {
                return Err(GeopError::new(
                    "the blend is too large for the circle: its tool would reach across the axis",
                ));
            }
            let axes = CoordinateSystem::try_new(*center, *radial, axis.prod_cross(radial), *axis)?;
            revolve(part, namer, Some(&root), &axes, 0.0, 360.0, &loops)?
        }
    };
    built
        .solid
        .ok_or_else(|| GeopError::new("the blend's tool came out as no solid"))
}

/// The tool of the straight edge of `section`, cross-section `tool`, as a
/// [`Tool`] to build whole — what a tool joined to a corner is: one span
/// along the edge between a station at each end, capped square to the edge
/// where it ends on its own — run out as far as [`sweep_tool`] runs it, or
/// flush at the end — and open where it is joined to a corner. Also whether
/// its section runs from the contact on the face on the edge's right to the
/// left's — the other way round from `section` — which it does where that
/// is what makes the blend face out of the tool.
pub(crate) fn straight_tool<S: Scalar>(
    section: &Section<S>,
    tool: &ToolProfile<S>,
) -> GeopResult<(Tool<S>, bool)> {
    let Sweep::Straight {
        start,
        end,
        e1,
        e2,
        ends,
        ..
    } = &section.sweep
    else {
        return Err(GeopError::new(
            "only the tool of a straight edge is built whole",
        ));
    };
    // A fillet's arc; or a chamfer's chord, extended past the faces as
    // when it is swept — so that it crosses them rather than ending on
    // them — its middle of weight one.
    let chord = tool.control.len() == 3;
    let (ta, e, tb, q) = match tool.control[..] {
        [ta, e, tb, q] => (ta, e, tb, q),
        [a, b, q] => (a, Vector2::interpolate(&a, &b, S::ONE.div(S::TWO)?), b, q),
        _ => return Err(GeopError::new("a blend's section of no known shape")),
    };
    let along = end.sub(start).normalize()?;
    let width = reach(&tool.control, &section.corner);
    let space = |v: &Vector2<S>| e1.prod_scalar(v[0]).add(&e2.prod_scalar(v[1]));
    // The blend faces out of the tool, towards its ball, away from the apex.
    let facing = along
        .prod_cross(&space(&tb.sub(&ta)))
        .prod_dot(&space(&e.sub(&q)));
    let swapped = if facing.definitely_greater(S::ZERO) {
        false
    } else if facing.definitely_less(S::ZERO) {
        true
    } else {
        return Err(GeopError::new(format!(
            "cannot tell which way the blend faces ({facing:?})"
        )));
    };
    let (first, second) = if swapped { (tb, ta) } else { (ta, tb) };
    let w = tool.weight;
    let flat = [
        Vector3::from_array([first[0], first[1], S::ONE]),
        if chord {
            Vector3::from_array([e[0], e[1], S::ONE])
        } else {
            Vector3::from_array([e[0].mul(w), e[1].mul(w), w])
        },
        Vector3::from_array([second[0], second[1], S::ONE]),
        Vector3::from_array([q[0], q[1], S::ONE]),
    ];
    let mut stations = Vec::new();
    for (k, out) in [(0, along.neg()), (1, along)] {
        let name = format!("st{k}");
        let from = if k == 0 { start } else { end };
        // As far again as the tool is wide, over how squarely the edge
        // crosses the face it is run out past.
        let run_out = |squareness: &S| S::from_f64(2.0 * width / squareness.lower().to_f64());
        let base = match &ends[k] {
            End::Flush | End::Wall { .. } => *from,
            End::RunOut { squareness } => from.add(&out.prod_scalar(run_out(squareness))),
            End::Corner { setback } => {
                // The section's corner is the edge's own point.
                let base = start.add(&along.prod_scalar(*setback));
                let at = |p: &Vector2<S>| base.add(&space(p));
                let h = |p: Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
                let (a, b, apex) = (at(&first), at(&second), at(&q));
                stations.push(ToolStation {
                    points: [
                        h(a),
                        Vector4::from_array([base[0].mul(w), base[1].mul(w), base[2].mul(w), w]),
                        h(b),
                        h(apex),
                    ],
                    vertices: [a, b, apex],
                    name,
                    cap: None,
                });
                continue;
            }
            End::Mitre { at, keep } => {
                // The section at the corner slid along the edge onto the
                // plane halving the corner — an affine map, so the arc
                // stays one, of the same weight — where it is joined to
                // the other tool's.
                let keep = keep.normalize()?;
                let slide = |p: Vector3<S>| -> GeopResult<Vector3<S>> {
                    let t = at.sub(&p).prod_dot(&keep).div(along.prod_dot(&keep))?;
                    Ok(p.add(&along.prod_scalar(t)))
                };
                let basis = keep.orthonormal_complement()?;
                let (mut a1, mut a2) = (basis[0], basis[1]);
                if a1
                    .prod_cross(&a2)
                    .prod_dot(&keep)
                    .definitely_greater(S::ZERO)
                {
                    std::mem::swap(&mut a1, &mut a2);
                }
                let plane = [*at, a1, a2];
                let in_plane = |p: Vector3<S>| -> GeopResult<[S; 2]> {
                    let d = slide(p)?.sub(at);
                    Ok([d.prod_dot(&a1), d.prod_dot(&a2)])
                };
                let one = |p: &Vector2<S>| -> GeopResult<Vector3<S>> {
                    let [x, y] = in_plane(from.add(&space(p)))?;
                    Ok(Vector3::from_array([x, y, S::ONE]))
                };
                // A fillet's middle control point is the edge's own point.
                let middle = if chord {
                    one(&e)?
                } else {
                    let [x, y] = in_plane(*from)?;
                    Vector3::from_array([x.mul(w), y.mul(w), w])
                };
                let flat = [one(&first)?, middle, one(&second)?, one(&q)?];
                let points = flat.map(|p| embed(&p, &plane));
                stations.push(ToolStation {
                    points,
                    vertices: [points[0], points[2], points[3]].map(|p| p.head::<3>()),
                    name,
                    cap: None,
                });
                continue;
            }
        };
        // The cap, `e1 x e2` out of the tool: the section's own axes, or
        // swapped — and the section's coordinates with them.
        let (plane, flat) = if e1.prod_cross(e2).prod_dot(&out).definitely_greater(S::ZERO) {
            ([base, *e1, *e2], flat)
        } else {
            (
                [base, *e2, *e1],
                flat.map(|p| Vector3::from_array([p[1], p[0], p[2]])),
            )
        };
        let points = flat.map(|p| embed(&p, &plane));
        stations.push(ToolStation {
            points,
            vertices: [points[0], points[2], points[3]].map(|p| p.head::<3>()),
            name,
            cap: Some(Cap { plane, flat }),
        });
    }
    let span = ToolSpan {
        degree: 1,
        knots: vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        rows: vec![stations[0].points, stations[1].points],
        name: None,
    };
    Ok((
        Tool {
            stations,
            spans: vec![span],
            closed: false,
            sides: if swapped { ["b", "a"] } else { ["a", "b"] },
            blend: if chord { "chamfer" } else { "fillet" },
        },
        swapped,
    ))
}

/// One tool to sweep: the section of the edge it was built for, by name.
pub(crate) struct Plan<S: Scalar> {
    pub(crate) edge: String,
    pub(crate) section: Section<S>,
}

/// How an edge is blended, and with it every other edge the same tool
/// blends: swept along a straight edge or around a circle — every arc of
/// it — or rolled along a tangent chain (see [`crate::rolling`]).
enum Planned<S: Scalar> {
    Swept(Plan<S>, Vec<EdgeId>),
    Rolled(Chain),
}

/// Whether both faces of `edge` are planes.
fn between_planes<S: Scalar>(model: &Model<S>, edge: EdgeId) -> GeopResult<bool> {
    for face in faces(model, edge)? {
        if model.get_face(face)?.surface.as_plane()?.is_none() {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The revolved blend of the circular edge `edge`, an arc of `circle`, and
/// every arc of the circle with it — none where the arcs do not close the
/// circle, or do not all meet faces a revolved blend handles alike.
fn round_plan<S: Scalar>(
    model: &Model<S>,
    name: &str,
    edge: EdgeId,
    circle: &Circle<S>,
) -> GeopResult<Option<Planned<S>>> {
    let Some(section) = round_section(model, edge, circle)? else {
        return Ok(None);
    };
    let Ok(arcs) = circle_edges(model, edge, circle) else {
        return Ok(None);
    };
    for &other in &arcs[1..] {
        let ctx = with_context!("arc {other} of the same circle");
        match round_section(model, other, circle).with_context(ctx)? {
            Some(theirs) if same_corner(&section, &theirs) => {}
            _ => return Ok(None),
        }
    }
    Ok(Some(Planned::Swept(
        Plan {
            edge: name.to_string(),
            section,
        },
        arcs,
    )))
}

/// How the edge named `name` is blended into `shape`. A straight edge
/// between two planes, on its own, and a whole circle between planes across
/// its axis and cylinders and cones around it, are swept exactly — when the
/// radius is the same all along. Every other blend is rolled along the
/// edge's tangent chain.
fn plan_edge<S: Scalar>(part: &Part<S>, name: &str, shape: &BlendShape) -> GeopResult<Planned<S>> {
    let model = part.topology();
    let edge = part.edge_id(name)?;
    let curve = &model.get_edge(edge)?.curve;
    let chain = tangent_chain(model, edge)?;
    let swept = match shape {
        BlendShape::Fillet { .. } => shape.constant_radius().is_some(),
        BlendShape::Chamfer { .. } => true,
    };
    if swept {
        if let Some(line) = curve.as_line()?
            && chain.links.len() == 1
            && between_planes(model, edge)?
        {
            let section = straight_section(model, edge, &line)?;
            return Ok(Planned::Swept(
                Plan {
                    edge: name.to_string(),
                    section,
                },
                vec![edge],
            ));
        }
        if let Some(circle) = circle_of(curve)?
            && let Some(planned) = round_plan(model, name, edge, &circle)?
        {
            return Ok(planned);
        }
    }
    Ok(Planned::Rolled(chain))
}

/// Gives the one face of `solid` left of each of the tool's faces named
/// `tool_faces` that face's name back. The boolean `namer` names (see
/// [`geop_ops_booleans::naming`]) names a piece split off a face `F` along
/// an edge `X` `N(F,X)` — and a piece of that piece alike — while the piece
/// keeping `F` may well be the one it drops. A blend keeps one piece of its
/// tool, the blend face, which is better known by its own name. A face of
/// which several pieces are left keeps the boolean's names.
fn name_blend_faces<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: SolidId,
    tool_faces: &[String],
) -> GeopResult<()> {
    let faces = part.topology().solid_faces(solid)?;
    for name in tool_faces {
        if part.id_of(name).is_some() {
            continue;
        }
        // `N(F,` — the name of a piece of `F` without its edge and `)`.
        let mut prefix = namer.name(&[name.as_str(), ""]);
        prefix.pop();
        let pieces: Vec<FaceId> = faces
            .iter()
            .copied()
            .filter(|&f| part.name_of(f).is_some_and(|n| n.starts_with(&prefix)))
            .collect();
        if let [piece] = pieces.as_slice() {
            part.rename(*piece, name.clone())?;
        }
    }
    Ok(())
}

/// A tool's end running into a wall (see [`End::Wall`]): which tool — by
/// index into the plans, and past them the chains — at its end or its
/// start, the vertex, the unit direction from it back along the edge, and
/// the faces either side.
struct WallEnd<S: Scalar> {
    tool: usize,
    at_end: bool,
    vertex: VertexId,
    back: Vector3<S>,
    faces: [FaceId; 2],
}

/// The mitres between `walls`: every two ends running into each other's
/// walls at a vertex, along a face they share — an inward corner, see the
/// module docs. Each joint, its ends' contacts to be filled in once their
/// tools are built, and per end where the tool is cut off.
fn mitres<S: Scalar>(
    model: &Model<S>,
    walls: &[WallEnd<S>],
) -> GeopResult<Vec<(Joint, [End<S>; 2])>> {
    let mut joints = Vec::new();
    let mut used = vec![false; walls.len()];
    for a in 0..walls.len() {
        for b in a + 1..walls.len() {
            let (wa, wb) = (&walls[a], &walls[b]);
            let shared = wa.faces.iter().find(|f| wb.faces.contains(f));
            let (Some(&shared), true) = (shared, wa.vertex == wb.vertex && wa.tool != wb.tool)
            else {
                continue;
            };
            if used[a] || used[b] {
                return Err(GeopError::new(format!(
                    "more than two blended edges run into each other's walls at vertex {}: only two are mitred",
                    wa.vertex
                )));
            }
            used[a] = true;
            used[b] = true;
            let at = model.get_vertex(wa.vertex)?.point;
            let keep = wa.back.sub(&wb.back).normalize()?;
            let end = |w: &WallEnd<S>| CornerEnd {
                tool: w.tool,
                at_end: w.at_end,
                contacts: [0, 0],
            };
            let (first, second) = if wa.tool < wb.tool {
                ((wa, keep), (wb, keep.neg()))
            } else {
                ((wb, keep.neg()), (wa, keep))
            };
            joints.push((
                Joint {
                    shared,
                    ends: [end(first.0), end(second.0)],
                },
                [
                    End::Mitre { at, keep: first.1 },
                    End::Mitre { at, keep: second.1 },
                ],
            ));
        }
    }
    Ok(joints)
}

/// Checks two straight edges' blends mitred at `joint` mirror each other in
/// the plane halving the corner — the same opening between the faces, and
/// the blend as far into the shared face, and into the wall, on both — or
/// the mitre would leave a step: refused.
fn mirrored<S: Scalar>(plans: &[(Plan<S>, ToolProfile<S>)], joint: &Joint) -> GeopResult<()> {
    let shape = |i: usize| -> Option<(S, S, S)> {
        let (plan, tool) = plans.get(i)?;
        let f = plan.section.faces.iter().position(|&f| f == joint.shared)?;
        let c = plan.section.corner;
        Some((
            plan.section.into[0].prod_dot(&plan.section.into[1]),
            tool.touches[f].sub(&c).norm(),
            tool.touches[1 - f].sub(&c).norm(),
        ))
    };
    let (Some(a), Some(b)) = (shape(joint.ends[0].tool), shape(joint.ends[1].tool)) else {
        return Ok(());
    };
    if a.0.could_be_equal(b.0) && a.1.could_be_equal(b.1) && a.2.could_be_equal(b.2) {
        Ok(())
    } else {
        Err(GeopError::new(format!(
            "edges {:?} and {:?} meet at an inward corner, where two blends are mitred only where they mirror each other",
            plans[joint.ends[0].tool].0.edge, plans[joint.ends[1].tool].0.edge
        )))
    }
}

/// Blends the edges named `edges`, all of one solid, into `shape` (see the
/// module docs), as the step whose names `namer` builds. The result is
/// named `namer`'s root, and the solid is consumed.
///
/// Each tool is named after the edge it was built for, `N(E,tool)`, and its
/// faces after their curves of the cross-section — the blend face itself
/// `N(E,fillet)` or `N(E,chamfer)` (`N(E,fillet,q0)`... for a circle, by
/// quarter turn, see [`revolve`]). What cutting it away creates is named
/// as a boolean names it, scoped by the edge: `N(E,...)`. Tools joined at
/// corners and mitres are one, named after the first.
pub fn blend<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    edges: &[String],
    shape: &BlendShape,
) -> GeopResult<()> {
    if edges.is_empty() {
        return Err(GeopError::new("pick the edges to blend"));
    }
    if let BlendShape::Fillet { radii } = shape {
        radii.check()?;
        for (vertex, _) in &radii.at_vertices {
            let ctx = with_context!("the radius at vertex {vertex:?}");
            part.vertex_id(vertex).with_context(ctx)?;
        }
    }
    let mut solid: Option<SolidId> = None;
    let mut covered: Vec<EdgeId> = Vec::new();
    let mut plans: Vec<(Plan<S>, ToolProfile<S>)> = Vec::new();
    let mut chains: Vec<(String, Chain)> = Vec::new();
    for name in edges {
        let ctx = with_context!("edge {name:?}");
        let edge = part.edge_id(name).with_context(ctx)?;
        let [face, _] = faces(part.topology(), edge).with_context(ctx)?;
        let Body::Solid(of) = part.topology().body_of_face(face)? else {
            return Err(GeopError::new("the edge is not part of a solid")).with_context(ctx);
        };
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
        match plan_edge(part, name, shape).with_context(ctx)? {
            Planned::Swept(plan, blended) => {
                let profile = tool_profile(&plan.section, shape).with_context(ctx)?;
                check_touches(part.topology(), &plan.section, &profile).with_context(ctx)?;
                covered.extend(blended);
                plans.push((plan, profile));
            }
            Planned::Rolled(chain) => {
                covered.extend(chain.edges());
                chains.push((name.clone(), chain));
            }
        }
    }
    let model = part.topology();
    let corners = corner::plan_corners(part, namer, &mut plans, &chains, &covered, shape)?;

    // Inward corners: the ends running into a wall where another does.
    let mut walls = Vec::new();
    for (i, (plan, _)) in plans.iter().enumerate() {
        if let Sweep::Straight {
            start,
            end,
            ends,
            vertices,
            ..
        } = &plan.section.sweep
        {
            let back = [end.sub(start).normalize()?, start.sub(end).normalize()?];
            for k in 0..2 {
                if matches!(ends[k], End::Wall { .. }) {
                    walls.push(WallEnd {
                        tool: i,
                        at_end: k == 1,
                        vertex: vertices[k],
                        back: back[k],
                        faces: plan.section.faces,
                    });
                }
            }
        }
    }
    for (c, (name, chain)) in chains.iter().enumerate() {
        let ctx = with_context!("edge {name:?}");
        for (k, wall) in rolling::chain_walls(model, chain)
            .with_context(ctx)?
            .into_iter()
            .enumerate()
        {
            if let (Some((vertex, back, faces)), None) = (wall, &corners.chains[c][k]) {
                walls.push(WallEnd {
                    tool: plans.len() + c,
                    at_end: k == 1,
                    vertex,
                    back,
                    faces,
                });
            }
        }
    }
    let mut joints = Vec::new();
    for (joint, cut) in mitres(model, &walls)? {
        let edge = |i: usize| match plans.get(i) {
            Some((plan, _)) => plan.edge.clone(),
            None => chains[i - plans.len()].0.clone(),
        };
        if joint.ends.iter().any(|end| end.tool >= plans.len()) {
            return Err(GeopError::new(format!(
                "edges {:?} and {:?} meet at an inward corner: blends meeting there are only mitred between straight edges of constant radius",
                edge(joint.ends[0].tool),
                edge(joint.ends[1].tool)
            )));
        }
        mirrored(&plans, &joint)?;
        for (end, cut) in joint.ends.iter().zip(cut) {
            if let Sweep::Straight { ends, .. } = &mut plans[end.tool].0.section.sweep {
                ends[usize::from(end.at_end)] = cut;
            }
        }
        joints.push(joint);
    }
    // A wall no mitre ends has to take the tool flush.
    for (plan, _) in &plans {
        if let Sweep::Straight { ends, .. } = &plan.section.sweep {
            for end in ends {
                if let End::Wall { unless: Some(why) } = end {
                    return Err(GeopError::new(why.clone()))
                        .with_context(with_context!("edge {:?}", plan.edge));
                }
            }
        }
    }

    let mut rolled: Vec<(String, Rolled<S>)> = Vec::new();
    for ((name, chain), ends) in chains.into_iter().zip(corners.chains) {
        let ctx = with_context!("edge {name:?}");
        let r = match shape {
            BlendShape::Fillet { radii } => rolling::plan_rolled(part, chain, radii, ends),
            BlendShape::Chamfer { distances } => rolling::plan_chamfer(part, chain, *distances),
        }
        .with_context(ctx)?;
        rolling::check_touches(part.topology(), &r).with_context(ctx)?;
        rolled.push((name, r));
    }
    let tools = Tools {
        plans: &plans,
        rolled: &rolled,
        corners: &corners.corners,
        joints: &joints,
    };
    // The tools — the plans', then the rolled ones', by index past them —
    // joined at corners and mitres are built whole, each such component
    // one solid, and applied by one boolean. The rolled ones first.
    let all = plans.len() + rolled.len();
    let mut components = tools.components(all);
    components.sort_by_key(|c| c.iter().all(|&i| i < plans.len()));
    let mut target = solid.expect("at least one edge");
    for component in components {
        let edge = tools.edge(component[0]);
        let ctx = with_context!("blending edge {edge:?}");
        let scope = namer.scoped(edge);
        let tool = tools
            .build(part, namer, &component)
            .with_context(with_context!("the tool of edge {edge:?}"))?;
        let bend = tools.bend(component[0]);
        target = apply_tool(part, &scope, target, tool, bend).with_context(ctx)?;
    }
    part.rename(target, namer.root())
}

/// Every tool of a blend: the swept ones' plans, the rolled ones — by index
/// past the plans — and the corners and mitres joining them.
struct Tools<'a, S: Scalar> {
    plans: &'a [(Plan<S>, ToolProfile<S>)],
    rolled: &'a [(String, Rolled<S>)],
    corners: &'a [Corner<S>],
    joints: &'a [Joint],
}

impl<S: Scalar> Tools<'_, S> {
    /// The name of the edge tool `i` was planned for.
    fn edge(&self, i: usize) -> &str {
        match self.plans.get(i) {
            Some((plan, _)) => &plan.edge,
            None => &self.rolled[i - self.plans.len()].0,
        }
    }

    /// Which way the edge of tool `i` bends.
    fn bend(&self, i: usize) -> Bend {
        match self.plans.get(i) {
            Some((plan, _)) => plan.section.bend,
            None => self.rolled[i - self.plans.len()].1.bend,
        }
    }

    /// The tools `0..all` in components: those joined, at corners and
    /// mitres, together; each other on its own.
    fn components(&self, all: usize) -> Vec<Vec<usize>> {
        let mut components: Vec<Vec<usize>> = (0..all).map(|i| vec![i]).collect();
        let links = self
            .corners
            .iter()
            .map(|c| c.ends.iter().map(|e| e.tool).collect::<Vec<_>>())
            .chain(
                self.joints
                    .iter()
                    .map(|j| j.ends.iter().map(|e| e.tool).collect()),
            );
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

    /// The tools `component` — joined at corners and mitres, or one on its
    /// own — built into one solid: swept, or built whole with the corners
    /// and mitres (see [`tool::build_tools`]), named after the first one's
    /// tool.
    fn build(&self, part: &mut Part<S>, namer: &Namer, component: &[usize]) -> GeopResult<SolidId> {
        let corners: Vec<&Corner<S>> = self
            .corners
            .iter()
            .filter(|c| c.ends.iter().any(|e| component.contains(&e.tool)))
            .collect();
        let joints: Vec<&Joint> = self
            .joints
            .iter()
            .filter(|j| j.ends.iter().any(|e| component.contains(&e.tool)))
            .collect();
        if let ([i], true, true) = (component, corners.is_empty(), joints.is_empty())
            && let Some((plan, profile)) = self.plans.get(*i)
        {
            return sweep_tool(part, &namer.scoped(&plan.edge), &plan.section, profile);
        }
        let namers: Vec<Namer> = component
            .iter()
            .map(|&i| namer.scoped(self.edge(i)))
            .collect();
        // Each tool, and the faces its section touches first and second at
        // its first and last station.
        let mut tools = Vec::new();
        let mut touching = Vec::new();
        for &i in component {
            match self.plans.get(i) {
                Some((plan, profile)) => {
                    let (tool, swapped) = straight_tool(&plan.section, profile)?;
                    let [l, r] = plan.section.faces;
                    let faces = if swapped { [r, l] } else { [l, r] };
                    tools.push(tool);
                    touching.push([faces, faces]);
                }
                None => {
                    let rolled = &self.rolled[i - self.plans.len()].1;
                    tools.push(rolled.tool.clone());
                    touching.push([rolled.contact_faces(false), rolled.contact_faces(true)]);
                }
            }
        }
        let index = |tool: usize| {
            component
                .iter()
                .position(|&i| i == tool)
                .ok_or_else(|| GeopError::new("a corner or mitre joins tools of another component"))
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
        // The mitres' ends likewise — contact 0 on the shared face — and
        // the second tool's section there made the first's.
        let mut joined = Vec::new();
        for &joint in &joints {
            let mut joint = joint.clone();
            for end in &mut joint.ends {
                let t = index(end.tool)?;
                let faces = touching[t][usize::from(end.at_end)];
                end.contacts = faces.map(|f| usize::from(f != joint.shared));
                end.tool = t;
            }
            let [lead, follow] = joint.ends;
            let row = |t: &Tool<S>, end: &CornerEnd| t.stations[end.station(t)].points;
            let theirs = row(&tools[lead.tool], &lead);
            let at = |contact: usize| {
                lead.contacts
                    .iter()
                    .position(|&c| c == contact)
                    .expect("a contact of the joint")
            };
            let (a, b) = (at(follow.contacts[0]), at(follow.contacts[1]));
            let lead_of = |k: usize| [0, 2][k];
            let mine = [theirs[lead_of(a)], theirs[1], theirs[lead_of(b)], theirs[3]];
            let tool = &mut tools[follow.tool];
            let s = follow.station(tool);
            let own = tool.stations[s].points;
            if !(0..4).all(|k| own[k].could_be_equal(&mine[k])) {
                return Err(GeopError::new(format!(
                    "edges {:?} and {:?} meet at an inward corner, where two blends are mitred only where they mirror each other: their sections there are {own:?} and {mine:?}",
                    self.edge(component[lead.tool]),
                    self.edge(component[follow.tool])
                )));
            }
            tool.stations[s].points = mine;
            let vertices = tools[lead.tool].stations[lead.station(&tools[lead.tool])].vertices;
            let tool = &mut tools[follow.tool];
            tool.stations[s].vertices = [vertices[a], vertices[b], vertices[2]];
            let span = if follow.at_end {
                tool.spans.len() - 1
            } else {
                0
            };
            let rows = &mut tool.spans[span].rows;
            let k = if follow.at_end { rows.len() - 1 } else { 0 };
            rows[k] = mine;
            joined.push(joint);
        }
        let refs: Vec<(&Namer, &Tool<S>)> = namers.iter().zip(&tools).collect();
        tool::build_tools(
            part,
            &refs,
            &joined_corners,
            &joined,
            namers[0].name(&["tool"]),
        )
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
/// by `scope` — and gives the blend faces left their names back (see
/// [`name_blend_faces`]): the solid that is left.
fn apply_tool<S: Scalar>(
    part: &mut Part<S>,
    scope: &Namer,
    target: SolidId,
    tool: SolidId,
    bend: Bend,
) -> GeopResult<SolidId> {
    let tool_faces = part
        .topology()
        .solid_faces(tool)?
        .into_iter()
        .filter_map(|f| part.name_of(f).map(str::to_string))
        .collect::<Vec<_>>();
    let target = boolean(
        part,
        scope,
        target,
        tool,
        bend.op(),
        RemeshParams::default(),
    )?
    .ok_or_else(|| GeopError::new("the blend cut the whole solid away"))?;
    name_blend_faces(part, scope, target, &tool_faces)?;
    Ok(target)
}
