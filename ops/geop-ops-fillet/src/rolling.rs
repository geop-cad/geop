//! Rolling-ball fillets of edges between any two faces: what
//! [`crate::blend`] does for an edge that is neither straight between two
//! planes nor a circle around its faces' common axis, or whose radius
//! varies along it.
//!
//! A ball of the fillet's radius `r` rolls along the edge, touching both
//! faces — behind them, in the material, at a convex edge; in front of them
//! at a concave one. Its center runs along the *spine*, the intersection of
//! the two faces' offsets by `r`; where it touches a face is that face's
//! *contact curve*, the spine projected back onto the face. The blend is
//! what the ball sweeps between the two contact curves.
//!
//! **Stations.** The ball is placed at *stations* along the edge: at each,
//! its center `C` is found by Newton's method on three conditions — `r`
//! from each face, measured along the face's normal at the foot point of
//! `C` (the gradient of the distance to a face is that normal), and in the
//! plane through the edge's point square to the edge. That is the
//! offsets' intersection without building either offset surface, which
//! for a free-form face is no NURBS at all. The two foot points are the
//! contact points `T_a`, `T_b`.
//!
//! **Section.** The ball's section through `T_a` and `T_b` is the arc of
//! its great circle through them, in the plane of `C`, `T_a` and `T_b`: the
//! rational quadratic from `T_a` to `T_b` whose middle control point `E` is
//! where the faces' tangent planes at the contact points meet that plane,
//! weighted by the cosine of the angle between the arc's chord and its
//! tangent. However exactly `C` was found, that arc touches each face's
//! tangent plane at its contact point — but for a tilt of [`TILT`] towards
//! the ball, which keeps the approximation between stations from dipping
//! through the face (checked there). (With a varying radius the ball's true
//! envelope touches along a small circle, not a great one; the great circle
//! is as tangent to both faces, and the difference is of the order of how
//! fast the radius changes.) The *tool* — what a boolean cuts away or fills
//! in, as for every blend — closes the arc off by two lines from its ends
//! to the apex `Q = 2E - C`, on the far side of the faces.
//!
//! **Surfaces.** The stations' sections are skinned: every control point of
//! the section interpolated along the edge, all at the same parameters, by
//! one cubic each (`NurbCurve::interpolate_homogeneous`), which makes the
//! blend rational quadratic across and cubic along, through every station's
//! arc exactly. Between stations it only approximates the rolling ball, so
//! the stations are doubled until it follows it to within [`DEVIATION`] of
//! the radius, measured at points the ball is rolled to between them;
//! failing that within [`MOST_STATIONS`], the edge is refused, saying how
//! far it got. The contact curves are the one place an approximation must
//! not lie: they bound the faces the boolean trims, so they are widened to
//! *enclose* the true contact curves (see `NurbCurve::enclosing_pad`), and
//! the blend's boundary rows with them.
//!
//! **Chains.** An edge is blended together with every edge it runs on into
//! tangentially, between faces running on tangentially — the tangent chain:
//! a circle made of quarter arcs, a slot's rim of lines and arcs. The
//! stations run evenly along it, half a step off its vertices; the faces
//! either side may change along it, from one face to the next tangent to
//! it, and the ball touches whichever of them it reaches. Where it rolls
//! from one onto the next, a station is put exactly on the edge between
//! them, and the tool's spline breaks there (see [`crossing`]): across it
//! the faces' curvature may jump. So it does at every vertex of a chain
//! whose radius varies, where the radius changes its rate. Rolling over a
//! crease is refused: a ball rolling over it would touch the crease's edge
//! itself, along a stretch where the blend's boundary is that edge, which
//! no boolean takes. A closed chain's tool breaks at four stations at
//! least, so that no face of it closes onto itself; every chain's at its
//! vertices, where its edges, and the faces either side, are only tangent
//! (see [`turns`]).
//!
//! **Ends.** A closed chain has none. An open one ends at a corner of
//! exactly three faces (see [`crate::blend::tool_end`]), decided at each
//! vertex on its own by the third face there:
//!
//! - a plane its section at the corner lies in — the faces either side
//!   standing square to it where the ball touches them: the tool crosses
//!   it exactly there, run out straight past it where a cut leaves the
//!   solid through it, and capped in it otherwise;
//! - any other face — curved, or a plane at an angle: the blend runs on
//!   past the vertex, rolled on the end link's faces carried on past their
//!   ends, through the third face, until its section is clear of the
//!   face's tangent plane there (see [`RunOn`]), and is capped there. A cut
//!   leaving the solid through the face ends in the air past it; a cut
//!   running into it, or a fill, is cut off by it — the tool split along a
//!   sheet on the face carried on, round the vertex (see
//!   [`Rolled::trims`]) — so that it meets the face where the face is. A
//!   fill up to a face it leaves the solid through, an outward corner, is
//!   refused: its end would lie on the corner's other edge.
//!
//! Where every edge of a corner is filleted, it ends at the ball rounding
//! the corner instead, set back from it (see [`crate::corner`]): its last
//! station that ball's great arc, untilted, the corner's piece of the ball
//! sharing it. Two blends meeting at an inward corner are only mitred
//! between straight edges of constant radius. A chain that ends where it
//! starts — round a face with one corner — has its tool's ends overlap
//! there, and is built in halves (see [`Rolled::halves`]).
//!
//! **Chamfers.** A chamfer of any other edge than a straight one between
//! planes or a whole circle is rolled the same way (see [`plan_chamfer`]),
//! its sections chords. Placing them is what differs (see [`Placing`]):
//! a chord at each station, and where its point on a side moves from face
//! to face — across a seam or a crease, both of which a chord crosses —
//! the chord whose point lies on the edge between them.

use geop_core_geometry::{
    meet::{Offset, meet},
    nurb_curve::{NurbCurve, NurbCurve3D, bezier_knots, true_point_fractions},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::Matrix,
    scalars::Scalar,
    solvers::{
        bisect::{Bracket, bisect, sign_changes},
        iterate::{Repeat, Step, iterate},
        linear::solve_linear_system,
        newton::{Arrive, newton_1d},
        probe::{Probe, probe},
        refine::{Refine, Refined, refine},
    },
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{EdgeId, FaceId, Model, VertexId};
use geop_lib_booleans::RemeshParams;
use geop_ops_model::NamedModel;

use crate::blend::{Bend, Crossing, End, bend, check_contacts, run_out_length, tool_end};
use crate::tool::{Cap, Tool, ToolSpan, ToolStation, embed, flatten, flatten_point, wall_surface};
use geop_ops_model::tangent::{Link, TangentChain as Chain};

/// Stations per edge of a chain to start with.
const FIRST_STATIONS: usize = 8;
/// The most stations per edge the blend is refined to — how hard it tries
/// to follow the rolling ball, not what following it means.
const MOST_STATIONS: usize = 256;
/// How closely the blend has to follow the rolling ball between stations,
/// as a fraction of the radius: the deviation every blend is checked to
/// stay within.
pub const DEVIATION: f64 = 1e-6;
/// The angle, in radians, at which the blend leaves each face towards the
/// ball rather than touching it. Between stations the blend is tangent to
/// the faces only as closely as it follows the ball; it would dip through
/// a face as often as not, and a boolean has to cut the face along every
/// such dip. Leaving at this angle keeps it on the ball's side — checked
/// between stations — so that it meets the face in its contact curve
/// alone: a crease of a millionth of a radian nobody sees, where exact
/// tangency was never to be had.
const TILT: f64 = DEVIATION;
/// How often the reach for a bracket of where the ball rolls across an
/// edge doubles, at most: `2^64` times the first reach is past any edge's
/// domain, which ends the search first.
const MOST_WIDENINGS: usize = 64;

/// The `(u, v)` of `point` on `surface`, sharp — a seed: where the point
/// lies on the surface ([`NurbSurface3D::locate`]), else that of the point
/// of the surface nearest it — on a cylinder, not the one across its axis.
pub(crate) fn seed_on(surface: &NurbSurface3D, point: &Vector3) -> GeopResult<(Scalar, Scalar)> {
    let (u, v) = match surface.locate(point, MAX_NODES, min_subdivision_size())? {
        Some(uv) => uv,
        None => {
            let (u, v, _) = surface.closest_point(point)?;
            foot(surface, point, (u, v))?
        }
    };
    Ok(sharp_in(surface, (u, v)))
}

/// The foot point of `target` on `surface`, Newton from `seed`: its
/// enclosure, within the surface's domain — the foot point lies in it, so
/// that holds too, and the surface is only evaluated there.
pub(crate) fn foot(
    surface: &NurbSurface3D,
    target: &Vector3,
    seed: (Scalar, Scalar),
) -> GeopResult<(Scalar, Scalar)> {
    surface.project_in_domain(*target, seed.0, seed.1, PROJECT_ITERATIONS)
}

/// `(u, v)` sharpened, within the domain of `surface`: a seed.
pub(crate) fn sharp_in(surface: &NurbSurface3D, (u, v): (Scalar, Scalar)) -> (Scalar, Scalar) {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    (u.sharpen().clamped(u0, u1), v.sharpen().clamped(v0, v1))
}

/// How a fillet's radius runs along its edges: `radius` at the start of
/// every chain — where the first picked edge of it starts — and
/// `end_radius`, if given, at the end of an open one, changing linearly
/// with the length along the chain in between; or, at the vertices named
/// in `at_vertices`, the radius given there, linearly between those.
#[derive(Clone, Debug, PartialEq)]
pub struct Radii {
    pub radius: f64,
    pub end_radius: Option<f64>,
    pub at_vertices: Vec<(String, f64)>,
}

impl Radii {
    /// The same radius everywhere.
    #[cfg(test)]
    pub fn constant(radius: f64) -> Self {
        Radii {
            radius,
            end_radius: None,
            at_vertices: Vec::new(),
        }
    }

    /// Whether the radius is the same everywhere.
    pub fn is_constant(&self) -> bool {
        self.end_radius.is_none() && self.at_vertices.is_empty()
    }

    /// Checks every radius is positive.
    pub fn check(&self) -> GeopResult<()> {
        let all = std::iter::once(self.radius)
            .chain(self.end_radius)
            .chain(self.at_vertices.iter().map(|(_, r)| *r));
        for r in all {
            if r.is_nan() || r <= 0.0 {
                return Err(GeopError::new(format!(
                    "a fillet's radius has to be positive, not {r}"
                )));
            }
        }
        Ok(())
    }
}

/// A fillet's radius along one chain (see [`Radii`]): at every vertex of
/// the chain, linear in the length along it between the vertices given a
/// radius — lengths measured in plain numbers, since the law is the
/// designer's choice and this is how it is laid out — and along each edge
/// linear in its parameter, so that the radius is as smooth along an edge
/// as the edge is and changes its rate only at the chain's vertices.
///
/// Where an open chain ends at a corner's ball (see [`crate::corner`]),
/// the radius at its end vertex is the ball's, and is reached where the
/// chain meets the ball, set back from the vertex: the end link's
/// fraction there, `ends`, its last for the radius.
pub(crate) struct RadiusLaw {
    /// Per vertex of the chain, in order — round a closed one to the first
    /// again — its radius.
    at_vertices: Vec<f64>,
    ends: [Option<f64>; 2],
}

impl RadiusLaw {
    pub(crate) fn new(model: &NamedModel, chain: &Chain, radii: &Radii) -> GeopResult<Self> {
        let topology = model.topology();
        // The length along the chain to each of its vertices: each edge's
        // integrated, which way it runs makes no difference.
        let mut lengths = vec![0.0];
        for link in &chain.links {
            let (length, _) = topology.get_edge(link.edge)?.curve.length()?;
            lengths.push(lengths.last().expect("a length") + length.midpoint().to_f64());
        }
        let mut vertices = chain.vertices(topology)?;
        if chain.closed {
            if radii.end_radius.is_some() {
                return Err(GeopError::new(
                    "the edge's tangent chain is closed, so it has no end for an end radius: give radii at its vertices instead",
                ));
            }
            vertices.push(vertices[0]);
        }
        let given = |v: VertexId| {
            model
                .name_of(v)
                .and_then(|name| radii.at_vertices.iter().find(|(n, _)| n == name))
                .map(|(_, r)| *r)
        };
        let last = vertices.len() - 1;
        let mut knots: Vec<(f64, f64)> = Vec::new();
        for (k, &v) in vertices.iter().enumerate() {
            let r = match given(v) {
                Some(r) => Some(r),
                None if k == 0 => Some(radii.radius),
                None if k == last && chain.closed => Some(knots[0].1),
                None if k == last => Some(radii.end_radius.unwrap_or(radii.radius)),
                None => None,
            };
            if let Some(r) = r {
                knots.push((lengths[k], r));
            }
        }
        let at_length = |s: f64| match knots.iter().position(|&(at, _)| at >= s) {
            None => knots.last().expect("a knot").1,
            Some(0) => knots[0].1,
            Some(j) => {
                let ((s0, r0), (s1, r1)) = (knots[j - 1], knots[j]);
                if s1 > s0 {
                    r0 + (s - s0) / (s1 - s0) * (r1 - r0)
                } else {
                    r1
                }
            }
        };
        Ok(RadiusLaw {
            at_vertices: lengths.iter().map(|&s| at_length(s)).collect(),
            ends: [None, None],
        })
    }

    /// The radius at the chain's start (`k = 0`) or end vertex: what a
    /// corner's ball there has to have.
    pub(crate) fn at_end(&self, k: usize) -> f64 {
        if k == 0 {
            self.at_vertices[0]
        } else {
            *self.at_vertices.last().expect("vertices")
        }
    }

    /// The radius `fraction` of the way along link `link`, sharp: the law is
    /// a choice, and this is it.
    fn radius(&self, link: usize, fraction: f64) -> Scalar {
        let (r0, r1) = (self.at_vertices[link], self.at_vertices[link + 1]);
        let last = self.at_vertices.len() - 2;
        let mut f = fraction;
        if let (0, Some(from)) = (link, self.ends[0]) {
            f = ((f - from) / (1.0 - from)).max(0.0);
        }
        if let (true, Some(to)) = (link == last, self.ends[1]) {
            let start = if link == 0 {
                self.ends[0].unwrap_or(0.0)
            } else {
                0.0
            };
            f = ((fraction - start) / (to - start)).min(1.0);
        }
        Scalar::from_f64(r0 + f * (r1 - r0))
    }

    /// The smallest radius along the chain.
    fn smallest(&self) -> f64 {
        self.at_vertices
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }
}

/// Where a point is on a link's two faces, the left and the right: its
/// `(u, v)` on each.
type Feet = [(Scalar, Scalar); 2];

/// A chamfer's section, as [`chamfer_frame`] finds it: the chain's point,
/// the direction the chain runs there, and the feet of the point on the
/// link's faces.
type ChamferSection = (Vector3, Vector3, Feet);

/// The ball at one station: its center, where it touches each face — the
/// faces on the chain's left and right — the faces' outward normals there
/// and the contacts' `(u, v)` on them (sharp, seeds), and the section's
/// middle control point, its weight and the tool's apex (see the module
/// docs).
///
/// Or, a `chord`, a chamfer's section (see [`plan_chamfer`]): its contacts
/// where the chord meets the faces, its middle control point the chord's
/// middle, of weight one, and its center as far beyond the chord as the
/// edge is before it.
#[derive(Clone, Debug)]
struct Station {
    center: Vector3,
    contact: [Vector3; 2],
    faces: [FaceId; 2],
    normals: [Vector3; 2],
    uv: Feet,
    corner: Vector3,
    weight: Scalar,
    apex: Vector3,
    chord: bool,
    /// Whether the section lies wholly past the third face a blend runs
    /// on through (see [`RunOn`]): there, in the air past a face it leaves
    /// the solid through or in what a face it is cut off by cuts away, how
    /// closely it follows the ball, and where it touches the faces carried
    /// on, decides nothing.
    aloof: bool,
}

/// The ball of signed radius `sr` — `r` along both normals, negative
/// behind them — touching `surfaces` in the plane through `p` square to the
/// unit `tangent`, [`meet`] from the foot points `seeds` (see the module
/// docs): its center, sharp — a construction aid: the contacts are
/// evaluated on the surfaces, so they lie on them however exactly the
/// center was found — and the foot points' `(u, v)`, sharp seeds.
fn roll(
    surfaces: [&NurbSurface3D; 2],
    seeds: Feet,
    p: &Vector3,
    tangent: &Vector3,
    sr: Scalar,
) -> GeopResult<(Vector3, Feet)> {
    let normal = |k: usize, (u, v): (Scalar, Scalar)| surfaces[k].normal(u, v)?.normalize();
    let (n0, n1) = (normal(0, seeds[0])?, normal(1, seeds[1])?);
    // Exact for two planes through `p`: `r` from both.
    let spread = Scalar::ONE.add(n0.prod_dot(&n1));
    let center = p.add(&n0.add(&n1).prod_scalar(sr.div(spread)?)).sharpen();
    let mut offsets = [0, 1].map(|k| Offset::new(surfaces[k], seeds[k], sr));
    let center = meet(center, &mut offsets, &[(*tangent, *p)])?.sharpen();
    Ok((center, offsets.map(|o| o.pieces[0].1)))
}

/// How far, as a plain number, the foot point `foot` of `center` on a
/// surface with unit normal `n` there is from being one: the part of
/// `center - foot` along the surface. Zero, but for rounding, for a true
/// foot point; a foot point clamped to the edge of the surface's domain —
/// where the ball would touch another face — leaves the rest.
fn off_foot(center: &Vector3, foot: &Vector3, n: &Vector3) -> f64 {
    let d = center.sub(foot);
    d.sub(&n.prod_scalar(d.prod_dot(n)))
        .norm()
        .midpoint()
        .to_f64()
}

/// The faces along either side of `chain`: where its ball may touch.
fn sides(chain: &Chain) -> [Vec<FaceId>; 2] {
    let mut sides = [Vec::new(), Vec::new()];
    for link in &chain.links {
        for (side, &face) in sides.iter_mut().zip(&link.faces) {
            if !side.contains(&face) {
                side.push(face);
            }
        }
    }
    sides
}

/// Places the ball at the station `fraction` of the way along link `link`
/// of `chain`, with signed radius `sr`, from the foot points of the
/// station before, `previous` (see the module docs).
///
/// It touches the link's own faces where it can. Where it would touch one
/// beyond its edge — its foot point clamped to the face's domain — it is
/// tried against the other faces along that side, and touches whichever it
/// meets truly; an error if none.
fn place(
    model: &Model,
    chain: &Chain,
    link: usize,
    fraction: Scalar,
    sr: Scalar,
    previous: Option<&Station>,
) -> GeopResult<Station> {
    let own = &chain.links[link];
    let (p, along) = own.at(model, fraction)?;
    // The edge runs along both faces' tangent planes: their normals there
    // state its direction exactly, where the edge's curve — traced as an
    // intersection, say — only encloses its points. At a vertex where the
    // faces change, that is what puts the ball's section through it.
    let mut normals = [Vector3::zero(); 2];
    for (k, normal) in normals.iter_mut().enumerate() {
        let surface = &model.get_face(own.faces[k])?.surface;
        let seed = match previous {
            Some(prev) if prev.faces[k] == own.faces[k] => sharp_in(surface, prev.uv[k]),
            _ => seed_on(surface, &p)?,
        };
        let (u, v) = foot(surface, &p, seed)?;
        *normal = surface.normal(u, v)?;
    }
    let tangent = own.tangent(normals, &along, &p)?;
    let sides = sides(chain);
    // The pairs of faces to try: the link's own first.
    let mut pairs = vec![own.faces];
    for &a in &sides[0] {
        for &b in &sides[1] {
            if !pairs.contains(&[a, b]) {
                pairs.push([a, b]);
            }
        }
    }
    let scale = sr.abs().upper().to_f64();
    let mut best: Option<(f64, Station)> = None;
    let mut tried: Vec<([FaceId; 2], String)> = Vec::new();
    for pair in pairs {
        let attempt = || -> GeopResult<(f64, Station)> {
            let surfaces = [
                &model.get_face(pair[0])?.surface,
                &model.get_face(pair[1])?.surface,
            ];
            let mut seeds = [(Scalar::ZERO, Scalar::ZERO); 2];
            for k in 0..2 {
                seeds[k] = match previous {
                    Some(prev) if prev.faces[k] == pair[k] => sharp_in(surfaces[k], prev.uv[k]),
                    Some(prev) => seed_on(surfaces[k], &prev.contact[k])?,
                    None => seed_on(surfaces[k], &p)?,
                };
            }
            let (center, uv) = roll(surfaces, seeds, &p, &tangent, sr)?;
            let mut contact = [Vector3::zero(); 2];
            let mut normals = [Vector3::zero(); 2];
            let mut final_uv = uv;
            let mut off = 0.0f64;
            for k in 0..2 {
                // The foot point's own enclosure: the contact point lies on
                // the surface, as evaluated there.
                let (u, v) = foot(surfaces[k], &center, uv[k])?;
                contact[k] = surfaces[k].evaluate(u, v)?;
                normals[k] = surfaces[k].normal(u, v)?.normalize()?;
                final_uv[k] = sharp_in(surfaces[k], (u, v));
                off = off.max(off_foot(&center, &contact[k], &normals[k]));
            }
            Ok((
                off,
                section(center, contact, normals, pair, final_uv, true)?,
            ))
        };
        let (off, station) = match attempt() {
            Ok(found) => found,
            Err(e) => {
                tried.push((pair, e.root_message().to_string()));
                continue;
            }
        };
        // A true foot point on both of the edge's own faces — off it by no
        // more than the blend may stray from the ball anywhere — is the
        // ball, seeded where the edge is: no need to try other faces.
        if pair == own.faces && off <= DEVIATION * scale {
            return Ok(station);
        }
        tried.push((pair, format!("{off:e} off")));
        // Off its true foot point by no more than the blend may stray from
        // the ball anywhere, a contact is as good as the blend is. Of the
        // balls that good, the one in the corner at the edge's point is the
        // nearest to it: the plane square to the edge may well hold others,
        // touching the faces across from it — on a cylinder, across its
        // axis.
        if off <= DEVIATION * scale {
            let near = distance(&station.center, &p);
            if best.as_ref().is_none_or(|(b, _)| near < *b) {
                best = Some((near, station));
            }
        }
    }
    match best {
        Some((_, station)) => Ok(station),
        None => Err(GeopError::new(format!(
            "the ball of radius {:?} touches no pair of the faces along the chain truly at {p:?}: it would touch a face beyond them, or none — each pair tried, and how far off its foot points it leaves the ball: {tried:?}",
            sr.abs(),
        ))),
    }
}

/// Where an open chain's blend runs on past its end vertex, through the
/// third face there — out of the solid, or into what the face cuts it off
/// at (see [`Rolled::trims`]) — rolled, where its section at the vertex
/// does not lie in that face, so that running straight on would leave the
/// ball (see [`run_ons`]): the ball rolled on along the end
/// link's edge carried on past its end, touching the link's two faces'
/// surfaces carried on alike ([`NurbCurve::extended`],
/// [`NurbSurface3D::extended`]), so that the blend up to the third face is
/// the ball's on the faces as they go on, and only the part in the air,
/// past it, is on their continuations. Each goes on as the polynomial it
/// is, so the sections turn on past the vertex as smoothly as before it,
/// and the tool's spans need not break there — a station there would put
/// its corner on the vertex. A position past the chain's end is a fraction
/// of the end link past it, as along the link.
#[derive(Clone, Debug)]
pub(crate) struct RunOn {
    /// The third face's tangent plane at the vertex, `(point, normal)`, the
    /// normal along the way the chain runs on: what a section has to be
    /// wholly past to be aloof (see [`Station::aloof`]).
    past: (Vector3, Vector3),
    link: Link,
    curve: NurbCurve3D,
    /// The edge's own domain, which the link's fractions are of.
    domain: (Scalar, Scalar),
    /// The edge's length: how far a fraction of it is.
    scale: f64,
    surfaces: [NurbSurface3D; 2],
}

impl RunOn {
    /// The run-on past the end `at_end` of `chain` — or its start — out of
    /// its end vertex `vertex`, for `reach` on past it.
    fn new(
        model: &Model,
        chain: &Chain,
        at_end: bool,
        vertex: VertexId,
        reach: f64,
        past: (Vector3, Vector3),
    ) -> GeopResult<Self> {
        let link = if at_end {
            chain.last()?
        } else {
            chain.first()?
        };
        let edge = &model.get_edge(link.edge)?.curve;
        let domain = edge.domain();
        let (length, _) = edge.length()?;
        let scale = length.midpoint().to_f64();
        // The edge's end the chain leaves through: its end where the link
        // runs forward and the chain ends there, or both start.
        let width = domain.1.sub(domain.0);
        let by = width.mul(Scalar::from_f64(carried_fraction(reach / scale)));
        let curve = edge.extended(at_end == link.forward, by.sharpen())?;
        let at = model.get_vertex(vertex)?.point;
        let mut surfaces = Vec::new();
        for face in link.faces {
            let surface = &model.get_face(face)?.surface;
            let ctx = with_context!("carrying face {face} on past vertex {vertex}");
            surfaces.push(carried_on_past(surface, &at, reach).with_context(ctx)?);
        }
        let [a, b]: [NurbSurface3D; 2] = surfaces.try_into().expect("two faces");
        Ok(RunOn {
            past,
            link: link.clone(),
            curve,
            domain,
            scale,
            surfaces: [a, b],
        })
    }

    /// The ball `fraction` of the way along the end link — past its end,
    /// above one, or before its start, below zero — of signed radius `sr`,
    /// from the station before, `previous` (see [`place`]).
    fn place(&self, fraction: f64, sr: Scalar, previous: Option<&Station>) -> GeopResult<Station> {
        let f = Scalar::from_f64(fraction);
        let along_edge = if self.link.forward {
            f
        } else {
            Scalar::ONE.sub(f)
        };
        let t = Scalar::interpolate(self.domain.0, self.domain.1, along_edge);
        let ctx = with_context!(
            "edge {} (forward {}), its domain {:?} carried on to {:?}, at {t:?}",
            self.link.edge,
            self.link.forward,
            self.domain,
            self.curve.domain()
        );
        let p = self.curve.evaluate(t).with_context(ctx)?;
        let along = self.curve.tangent(t).with_context(ctx)?.normalize()?;
        let along = if self.link.forward {
            along
        } else {
            along.neg()
        };
        let faces = self.link.faces;
        let surfaces = [&self.surfaces[0], &self.surfaces[1]];
        let mut seeds = [(Scalar::ZERO, Scalar::ZERO); 2];
        let mut normals = [Vector3::zero(); 2];
        for k in 0..2 {
            seeds[k] = match previous {
                Some(prev) if prev.faces[k] == faces[k] => sharp_in(surfaces[k], prev.uv[k]),
                _ => seed_on(surfaces[k], &p)?,
            };
            let (u, v) = foot(surfaces[k], &p, seeds[k])?;
            normals[k] = surfaces[k].normal(u, v)?;
        }
        let tangent = self.link.tangent(normals, &along, &p)?;
        let (center, uv) = roll(surfaces, seeds, &p, &tangent, sr)?;
        let mut contact = [Vector3::zero(); 2];
        let mut normals = [Vector3::zero(); 2];
        let mut final_uv = uv;
        let mut off = 0.0f64;
        for k in 0..2 {
            let (u, v) = foot(surfaces[k], &center, uv[k])?;
            contact[k] = surfaces[k].evaluate(u, v)?;
            normals[k] = surfaces[k].normal(u, v)?.normalize()?;
            final_uv[k] = sharp_in(surfaces[k], (u, v));
            off = off.max(off_foot(&center, &contact[k], &normals[k]));
        }
        if off > DEVIATION * sr.abs().upper().to_f64() {
            return Err(GeopError::new(format!(
                "the ball rolled on to {fraction} of the way along edge {} touches faces {} and {} carried on only {off:e} off its foot points, beyond where they were carried",
                self.link.edge, faces[0], faces[1]
            )));
        }
        let mut station = section(center, contact, normals, faces, final_uv, true)?;
        station.aloof = self.clear_of_past(&station);
        Ok(station)
    }

    /// Whether `station`'s control polygon, which holds its section, lies
    /// wholly past the third face's tangent plane.
    fn clear_of_past(&self, station: &Station) -> bool {
        let (point, normal) = &self.past;
        [
            station.contact[0],
            station.corner,
            station.contact[1],
            station.apex,
        ]
        .iter()
        .all(|p| {
            p.sub(point)
                .prod_dot(normal)
                .definitely_greater(Scalar::ZERO)
        })
    }
}

/// How much of a curve or surface's domain it is carried on by, to reach
/// `fraction` of it on: twice as far, and half the domain at least — a
/// free choice, only ever rounded up.
fn carried_fraction(fraction: f64) -> f64 {
    (2.0 * fraction).max(0.5)
}

/// `surface` carried on ([`NurbSurface3D::extended`]) past every side of
/// it nearer the point `at` than `reach`, far enough to reach `reach` on
/// past it (see [`carried_fraction`]), by how fast it runs there: where a
/// face ending at `at` would go on — at its surface's side, or at a trim
/// short of it.
pub(crate) fn carried_on_past(
    surface: &NurbSurface3D,
    at: &Vector3,
    reach: f64,
) -> GeopResult<NurbSurface3D> {
    let Some((u, v)) = surface.locate(at, MAX_NODES, min_subdivision_size())? else {
        return Err(GeopError::new(format!(
            "the point {at:?} does not lie on the face's surface"
        )));
    };
    let (su, sv) = surface.derivatives(u.sharpen(), v.sharpen())?;
    let mut carried = surface.clone();
    for (along_u, t, speed) in [(true, u, su), (false, v, sv)] {
        let (t0, t1) = if along_u {
            surface.domain_u()
        } else {
            surface.domain_v()
        };
        let width = t1.sub(t0);
        // The fraction of the domain `reach` is, at the speed there.
        let span = speed.norm().mul(width).lower().to_f64();
        let fraction = if span > 0.0 { reach / span } else { 0.0 };
        let by = width
            .mul(Scalar::from_f64(carried_fraction(fraction)))
            .sharpen();
        // Every side nearer `at` than `reach`, at the speed there: the
        // side `at` lies on, and one a face trimmed short of its surface's
        // end has beyond its boundary — a free choice, only ever farther.
        for (at_end, side) in [(false, t0), (true, t1)] {
            let gap = side.sub(t).abs().mul(speed.norm()).lower().to_f64();
            if t.could_be_equal(side) || gap < reach {
                carried = carried.extended(along_u, at_end, by)?;
            }
        }
    }
    Ok(carried)
}

/// The piece of `surface` within about `reach` of its point `at`, by how
/// fast it runs there, either way along each parameter: where a tool's end
/// meets the face it is cut off by, and only there — elsewhere the tool may
/// run along the face (the blade's root fill, along the face it is cut off
/// by at the trailing edge, beyond it).
pub(crate) fn around(
    surface: &NurbSurface3D,
    at: &Vector3,
    reach: f64,
) -> GeopResult<NurbSurface3D> {
    let Some((u, v)) = surface.locate(at, MAX_NODES, min_subdivision_size())? else {
        return Err(GeopError::new(format!(
            "the point {at:?} does not lie on the face's surface"
        )));
    };
    let (u, v) = (u.sharpen(), v.sharpen());
    let (su, sv) = surface.derivatives(u, v)?;
    let mut bounds = Vec::new();
    for (t, speed) in [(u, su), (v, sv)] {
        let speed = speed.norm().lower().to_f64();
        if speed <= 0.0 {
            return Err(GeopError::new(format!(
                "the face's surface does not run along one of its parameters at {at:?}"
            )));
        }
        let half = Scalar::from_f64(reach / speed);
        bounds.push((t.sub(half).sharpen(), t.add(half).sharpen()));
    }
    surface.sub_surface(bounds[0], bounds[1])
}

/// The station of the ball centered at `center` touching `faces` at
/// `contact`, where their outward unit normals are `normals`: its section's
/// middle control point, weight and apex (see the module docs) — free
/// choices of the blend's shape, sharp. With `tilt` the arc leaves the
/// faces turned by [`TILT`] towards the ball; without, it is the ball's
/// great arc exactly — where the blend meets a corner's ball (see
/// [`crate::corner`]), whose piece shares the arc.
fn section(
    center: Vector3,
    contact: [Vector3; 2],
    normals: [Vector3; 2],
    faces: [FaceId; 2],
    uv: Feet,
    tilt: bool,
) -> GeopResult<Station> {
    let [ta, tb] = contact;
    let m = normals[0].prod_cross(&normals[1]);
    // The arc's tangent at each contact: along the face, in the section's
    // plane, turned by `TILT` towards the ball — and the plane through the
    // contact holding it and the section's normal.
    let mut tilted = [Vector3::zero(); 2];
    for k in 0..2 {
        let (at, other) = (contact[k], contact[1 - k]);
        let mut along = m.prod_cross(&normals[k]).normalize()?;
        if along
            .prod_dot(&other.sub(&at))
            .definitely_less(Scalar::ZERO)
        {
            along = along.neg();
        }
        if tilt {
            let toward = center.sub(&at).normalize()?;
            along = along.add(&toward.prod_scalar(Scalar::from_f64(TILT)));
        }
        tilted[k] = along.prod_cross(&m);
    }
    let [na, nb] = tilted;
    let planes = Matrix::from_rows([na.to_array(), nb.to_array(), m.to_array()]);
    let corner = solve_linear_system(
        &planes,
        &Vector3::from_array([na.prod_dot(&ta), nb.prod_dot(&tb), m.prod_dot(&center)]),
    )
    .map_err(|e| {
        e.with_context("the faces could be tangent where the ball touches them: no section")
    })?
    .sharpen();
    let chord = tb.sub(&ta);
    let along = corner.sub(&ta);
    let weight = chord
        .prod_dot(&along)
        .div(chord.norm().mul(along.norm()))?
        .sharpen();
    if !weight.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "the blend's section turns by half a turn or more there: weight {weight:?}"
        )));
    }
    let apex = corner.prod_scalar(Scalar::TWO).sub(&center).sharpen();
    Ok(Station {
        center,
        contact,
        faces,
        normals,
        uv,
        corner,
        weight,
        apex,
        chord: false,
        aloof: false,
    })
}

/// The section's control points, homogeneous: the contact on the left,
/// the middle (weighted — a chord's of weight one, taken as it is), the
/// contact on the right, the apex.
fn section_points(station: &Station) -> [Vector4; 4] {
    let w = station.weight;
    let e = station.corner;
    [
        station.contact[0].sharpen().homogeneous(Scalar::ONE),
        if station.chord {
            e.homogeneous(Scalar::ONE)
        } else {
            e.homogeneous(w)
        },
        station.contact[1].sharpen().homogeneous(Scalar::ONE),
        station.apex.homogeneous(Scalar::ONE),
    ]
}

/// Where a chamfer's section is taken at the point `fraction` of the way
/// along link `link` of `chain`: the edge's point there — one point of its
/// enclosure, which is a free choice, as is where along the edge the
/// section is — and the unit tangent, stated by the faces' normals there as
/// in [`place`]; and the foot points' `(u, v)` on the link's faces, from the
/// section before, `previous`.
fn chamfer_frame(
    model: &Model,
    chain: &Chain,
    link: usize,
    fraction: Scalar,
    previous: Option<&Station>,
) -> GeopResult<ChamferSection> {
    let own = &chain.links[link];
    let (p, along) = own.at(model, fraction)?;
    let p = p.sharpen();
    let mut normals = Vec::new();
    let mut feet = [(Scalar::ZERO, Scalar::ZERO); 2];
    for (k, at) in feet.iter_mut().enumerate() {
        let surface = &model.get_face(own.faces[k])?.surface;
        let seed = match previous {
            Some(prev) if prev.faces[k] == own.faces[k] => prev.uv[k],
            _ => seed_on(surface, &p)?,
        };
        let (u, v) = foot(surface, &p, seed)?;
        normals.extend(surface.normal(u, v));
        *at = sharp_in(surface, (u, v));
    }
    // Where a face's coordinates cannot state its normal at the edge — a
    // hair off a pole, where the face's own `(u, v)` collapse — the edge's
    // curve states its direction.
    let [n0, n1] = normals[..] else {
        return Ok((p, along.normalize()?, feet));
    };
    let tangent = own.tangent([n0, n1], &along, &p)?;
    Ok((p, tangent, feet))
}

/// The point of `surface` in the plane through `p` square to the unit
/// `tangent`, `d` from `p`, Newton's method from `seed`: its `(u, v)`.
/// Every iterate is sharpened — where a chamfer meets a face is its shape,
/// chosen — and the answer checked against both conditions, to within how
/// far the chamfer may stray from them anywhere.
fn chamfer_point(
    surface: &NurbSurface3D,
    p: &Vector3,
    tangent: &Vector3,
    d: f64,
    seed: (Scalar, Scalar),
) -> GeopResult<(Scalar, Scalar)> {
    let dd = Scalar::from_f64(d);
    let at = iterate(
        seed,
        seed,
        PROJECT_ITERATIONS,
        Repeat::FixedPoint,
        |&next| (sharp_in(surface, next), next),
        |&at| -> GeopResult<Step<(Scalar, Scalar)>> {
            let x = surface.evaluate(at.0, at.1)?;
            let (su, sv) = surface.derivatives(at.0, at.1)?;
            let off = x.sub(p);
            let residual =
                Vector2::from_array([off.prod_dot(tangent), off.prod_dot(&off).sub(dd.mul(dd))]);
            let jacobian = Matrix::from_rows([
                [su.prod_dot(tangent), sv.prod_dot(tangent)],
                [
                    Scalar::TWO.mul(off.prod_dot(&su)),
                    Scalar::TWO.mul(off.prod_dot(&sv)),
                ],
            ]);
            let step = solve_linear_system(&jacobian, &residual)?;
            Ok(Step::Next((at.0.sub(step[0]), at.1.sub(step[1]))))
        },
    )?
    .seed;
    let x = surface.evaluate(at.0, at.1)?;
    let off = x.sub(p);
    let square = off.prod_dot(tangent).abs().upper().to_f64();
    let far = off.norm().sub(dd).abs().upper().to_f64();
    if square.max(far) > DEVIATION * d {
        return Err(GeopError::new(format!(
            "the chamfer cannot meet the face {d} from the edge at {p:?}: the nearest it comes is {x:?}, beyond the face's surface"
        )));
    }
    Ok(at)
}

/// A seed for the point a chamfer meets `surface`, on side `k` of the
/// chain, `d` from the edge's point `p` with unit tangent `tangent`: the
/// foot of the point `d` into the face — square to the edge, along the
/// plane tangent to the face at the foot of `p`, `foot`.
fn seed_into(
    surface: &NurbSurface3D,
    p: &Vector3,
    tangent: &Vector3,
    foot_of_p: (Scalar, Scalar),
    k: usize,
    d: f64,
) -> GeopResult<(Scalar, Scalar)> {
    let n = surface.normal(foot_of_p.0, foot_of_p.1)?.normalize()?;
    let into = if k == 0 {
        n.prod_cross(tangent)
    } else {
        tangent.prod_cross(&n)
    }
    .normalize()?;
    let guess = p.add(&into.prod_scalar(Scalar::from_f64(d)));
    Ok(sharp_in(surface, foot(surface, &guess, foot_of_p)?))
}

/// The chamfer's section from the edge's point `p` — the station as
/// [`Station`] holds a chord — meeting `faces` at `contact`, where their
/// outward unit normals are `normals` and their `(u, v)` are `uv`.
fn chord(
    p: &Vector3,
    contact: [Vector3; 2],
    normals: [Vector3; 2],
    faces: [FaceId; 2],
    uv: Feet,
) -> GeopResult<Station> {
    let middle =
        Vector3::interpolate(&contact[0], &contact[1], Scalar::ONE.div(Scalar::TWO)?).sharpen();
    Ok(Station {
        center: middle.prod_scalar(Scalar::TWO).sub(p).sharpen(),
        contact,
        faces,
        normals,
        uv,
        corner: middle,
        weight: Scalar::ONE,
        apex: p.prod_scalar(Scalar::TWO).sub(&middle).sharpen(),
        chord: true,
        aloof: false,
    })
}

/// The edge's point a chord's section `station` was taken from.
fn chord_edge_point(station: &Station) -> GeopResult<Vector3> {
    Ok(Vector3::interpolate(
        &station.corner,
        &station.apex,
        Scalar::ONE.div(Scalar::TWO)?,
    ))
}

/// The chamfer's section at the point `fraction` of the way along link
/// `link` of `chain` (see [`plan_chamfer`]), `distances` into the faces on
/// its left and right, from the points of the section before, `previous`.
fn chamfer_station(
    model: &Model,
    chain: &Chain,
    link: usize,
    fraction: Scalar,
    distances: [f64; 2],
    previous: Option<&Station>,
) -> GeopResult<Station> {
    let own = &chain.links[link];
    let (p, tangent, feet) = chamfer_frame(model, chain, link, fraction, previous)?;
    let mut contact = [Vector3::zero(); 2];
    let mut normals = [Vector3::zero(); 2];
    let mut uv = feet;
    for k in 0..2 {
        let surface = &model.get_face(own.faces[k])?.surface;
        let seed = match previous {
            Some(prev) if prev.faces[k] == own.faces[k] => prev.uv[k],
            _ => seed_into(surface, &p, &tangent, feet[k], k, distances[k])?,
        };
        let ctx = with_context!("on face {}", own.faces[k]);
        uv[k] = chamfer_point(surface, &p, &tangent, distances[k], seed).with_context(ctx)?;
        contact[k] = surface.evaluate(uv[k].0, uv[k].1)?;
        normals[k] = surface.normal(uv[k].0, uv[k].1)?.normalize()?;
    }
    chord(&p, contact, normals, own.faces, uv)
}

/// The chamfer's section where its side `k` moves from the face of the
/// section `before`, at position `from` along `chain`, onto the face of the
/// one `after`, at `to` — onto the next face across the edge between them,
/// smooth or a crease: there its point is found on that edge, so that it
/// lies on both faces, and the tool's spline breaks there. Its position as
/// a fraction of the way from `from` to `to`, and the section.
#[allow(clippy::too_many_arguments)]
fn chamfer_crossing(
    model: &Model,
    chain: &Chain,
    k: usize,
    before: &Station,
    after: &Station,
    (from, to): (f64, f64),
    distances: [f64; 2],
) -> GeopResult<(f64, Station)> {
    let (face, onto) = (before.faces[k], after.faces[k]);
    let Some((edge, seed, _)) = crossed_edge(model, (face, onto), before, after, k)? else {
        return Err(GeopError::new(format!(
            "the chamfer moves from face {face} onto face {onto}, which meet in no edge"
        )));
    };
    let curve = &model.get_edge(edge)?.curve;
    let (lo, hi) = curve.domain();
    // At position `c`: the edge's point the chamfer's distance from the
    // chain's — Newton's method along it — and how far ahead of the plane
    // square to the chain there it is. The edge runs across the chain's
    // side, so it meets that sphere once, near where the chamfer meets it.
    let d2 = Scalar::from_f64(distances[k] * distances[k]);
    let at = |c: f64, seed: Scalar| -> GeopResult<(f64, Scalar, ChamferSection)> {
        let (link, x) = locate(chain, c);
        let (p, tangent, feet) =
            chamfer_frame(model, chain, link, Scalar::from_f64(x), Some(before)).with_context(
                with_context!("the chamfer's section {x} of the way along link {link}"),
            )?;
        let t = newton_1d(
            seed,
            (lo, hi),
            PROJECT_ITERATIONS,
            Arrive::Exhausted,
            |t| -> GeopResult<_> {
                let off = curve.evaluate(t)?.sub(&p);
                let rate = Scalar::TWO.mul(off.prod_dot(&curve.tangent(t)?));
                Ok((off.prod_dot(&off).sub(d2), rate))
            },
        )?
        .0;
        let ahead = curve.evaluate(t)?.sub(&p).prod_dot(&tangent);
        Ok((ahead.midpoint().to_f64(), t, (p, tangent, feet)))
    };
    let (lo_c, hi_c) = (from, to);
    let (g_lo, mut t) = {
        let (g, t, ..) = at(lo_c, seed)?;
        (g, t)
    };
    let (g_hi, ..) = at(hi_c, t)?;
    if g_lo.signum() == g_hi.signum() {
        return Err(GeopError::new(format!(
            "cannot find where the chamfer moves from face {face} onto face {onto} across edge {edge}: its point is {g_lo:e} and {g_hi:e} off the edge either side"
        )));
    }
    // Bisection: where along the chain is a free choice up to where the
    // chamfer's point lies on the edge, and the halves only close in on it.
    let bracket = bisect(lo_c, hi_c, 60, false, |mid| -> GeopResult<_> {
        let (g, tm, ..) = at(mid, t)?;
        t = tm;
        Ok(Some(g.signum() == g_lo.signum()))
    })?;
    let c = 0.5 * (bracket.lo + bracket.hi);
    let (_, t, (p, tangent, feet)) = at(c, t)?;
    let (link, _) = locate(chain, c);
    let own = &chain.links[link];
    let mut faces = own.faces;
    faces[k] = face;
    let mut contact = [Vector3::zero(); 2];
    let mut normals = [Vector3::zero(); 2];
    let mut uv = feet;
    // On the edge, so on both faces: its foot on the face it leaves.
    contact[k] = curve.evaluate(t)?;
    let surface = &model.get_face(face)?.surface;
    let ctx = with_context!("where the chamfer meets edge {edge} at {:?}", contact[k]);
    uv[k] = sharp_in(
        surface,
        foot(surface, &contact[k], before.uv[k]).with_context(ctx)?,
    );
    normals[k] = surface
        .normal(uv[k].0, uv[k].1)
        .and_then(|n| n.normalize())
        .with_context(ctx)?;
    let o = 1 - k;
    let other = &model.get_face(faces[o])?.surface;
    let seed = if before.faces[o] == faces[o] {
        before.uv[o]
    } else {
        seed_into(other, &p, &tangent, feet[o], o, distances[o]).with_context(with_context!(
            "seeding the chamfer's point on face {}",
            faces[o]
        ))?
    };
    let ctx = with_context!("on face {}", faces[o]);
    uv[o] = chamfer_point(other, &p, &tangent, distances[o], seed).with_context(ctx)?;
    contact[o] = other.evaluate(uv[o].0, uv[o].1)?;
    normals[o] = other.normal(uv[o].0, uv[o].1)?.normalize()?;
    Ok((
        (c - from) / (to - from),
        chord(&p, contact, normals, faces, uv)?,
    ))
}

/// How a blend's sections are placed along a chain (see [`roll_chain`]):
/// a section at a position, from the one before; where one side moves from
/// face to face between two sections — its position as a fraction of the
/// way, and the section there; the one section where both sides move at
/// one place, if two such are one; and the positions that need a section
/// of their own, where the blend changes its rate (see [`turns`]).
struct Placing<'a> {
    place: Place<'a>,
    cross: Cross<'a>,
    merge: Merge<'a>,
    kinks: Vec<f64>,
    /// How far, in positions, the blend runs on past the start and the end
    /// of an open chain (see [`RunOn`]).
    reach: [f64; 2],
    /// Where the tool is halved, if it is (see [`Rolled::halves`]): a
    /// station there, where its spans break.
    halve: Option<f64>,
}

/// The section at a position, from the one before (see [`Placing`]).
type Place<'a> = Box<dyn Fn(f64, Option<&Station>) -> GeopResult<Station> + 'a>;

/// Where side `k` moves from face to face between two sections, between
/// two positions: the position, and the section there (see [`Placing`]).
type Cross<'a> =
    Box<dyn Fn(usize, &Station, &Station, (f64, f64)) -> GeopResult<(f64, Station)> + 'a>;

/// The one section where both sides move at one place, if two such are one
/// (see [`Placing`]).
type Merge<'a> = Box<dyn Fn(&Station, &Station) -> GeopResult<Option<Station>> + 'a>;

/// The balls rolled along a chain: at the stations, by their positions
/// along it — link index plus fraction of its edge — and between each two
/// where `true_point_fractions` says, by the fraction of the way; and where
/// the tool's spans start and end, as indices of stations: at both ends,
/// and wherever the ball rolls from one face onto the next.
struct Rolling {
    stations: Vec<Station>,
    positions: Vec<f64>,
    breaks: Vec<usize>,
    between: Vec<Vec<(f64, Station)>>,
}

impl Rolling {
    /// The spans, as their first and last station.
    fn spans(&self) -> Vec<(usize, usize)> {
        self.breaks.windows(2).map(|w| (w[0], w[1])).collect()
    }

    /// The parameter along the span from station `a` to `b` of the
    /// position `c`: evenly by position, exactly 0 and 1 at its ends.
    fn param(&self, (a, b): (usize, usize), c: f64) -> Scalar {
        let (ca, cb) = (self.positions[a], self.positions[b]);
        if c == ca {
            Scalar::ZERO
        } else if c == cb {
            Scalar::ONE
        } else {
            Scalar::from_f64((c - ca) / (cb - ca))
        }
    }

    /// The stations' parameters along `span`.
    fn params(&self, span: (usize, usize)) -> Vec<Scalar> {
        (span.0..=span.1)
            .map(|i| self.param(span, self.positions[i]))
            .collect()
    }

    /// The balls between the stations of `span`, by their parameters.
    fn between(&self, span: (usize, usize)) -> Vec<Vec<(Scalar, Station)>> {
        (span.0..span.1)
            .map(|i| {
                let (c, next) = (self.positions[i], self.positions[i + 1]);
                self.between[i]
                    .iter()
                    .map(|(f, ball)| (self.param(span, c + f * (next - c)), ball.clone()))
                    .collect()
            })
            .collect()
    }
}

/// Where along `chain` its stations are, `n` to an edge: positions — link
/// index plus fraction of its edge — half a step off every vertex between
/// two links, where the faces either side may change and the ball would
/// touch the edge between them exactly at a station; an open chain also at
/// both ends of `range`, the part of it blended, and only between them. A
/// closed one runs round once, not back to its first.
fn station_positions(chain: &Chain, n: usize, (from, to): (f64, f64)) -> Vec<f64> {
    if chain.closed {
        let count = n * chain.links.len();
        return (0..count).map(|k| (k as f64 + 0.5) / n as f64).collect();
    }
    // The range may run on past the chain's ends (see [`RunOn`]), at the
    // same spacing.
    let first = (from * n as f64).floor() as i64 - 1;
    let last = (to * n as f64).ceil() as i64 + 1;
    let inner = (first..=last).map(|k| (k as f64 + 0.5) / n as f64);
    // Clear of the ends by a quarter step: no span next to one shorter.
    let clear = 0.25 / n as f64;
    std::iter::once(from)
        .chain(inner.filter(|&c| c - from > clear && to - c > clear))
        .chain(std::iter::once(to))
        .collect()
}

/// The position `c` along `chain` as a link and a fraction of its edge —
/// round a closed one again past its end.
fn locate(chain: &Chain, c: f64) -> (usize, f64) {
    let links = chain.links.len();
    let c = if chain.closed {
        c.rem_euclid(links as f64)
    } else {
        c
    };
    let link = (c.floor().max(0.0) as usize).min(links - 1);
    (link, c - link as f64)
}

/// The edges joining faces `a` and `b`.
fn edges_between(model: &Model, a: FaceId, b: FaceId) -> GeopResult<Vec<EdgeId>> {
    let mut found = Vec::new();
    for &edge in model.edges.keys() {
        let mut on = Vec::new();
        for c in model.coedges_of_edge(edge) {
            on.push(model.get_coedge(c)?.face);
        }
        if on.contains(&a) && on.contains(&b) {
            found.push(edge);
        }
    }
    Ok(found)
}

/// The edge between the faces `from` and `onto` that side `k` of the
/// blend crosses going from the station `before` to `after`: of the edges
/// between them, the one their contacts there lie nearest, with where along
/// it each lies nearest. None if the faces meet in no edge.
fn crossed_edge(
    model: &Model,
    (from, onto): (FaceId, FaceId),
    before: &Station,
    after: &Station,
    k: usize,
) -> GeopResult<Option<(EdgeId, Scalar, Scalar)>> {
    let mut best: Option<(f64, EdgeId, Scalar, Scalar)> = None;
    for edge in edges_between(model, from, onto)? {
        let curve = &model.get_edge(edge)?.curve;
        let middle = curve.mid_parameter()?;
        let t0 = curve.closest_parameter_from(&before.contact[k], middle)?;
        let t1 = curve.closest_parameter_from(&after.contact[k], middle)?;
        let near = distance(&curve.evaluate(t0)?, &before.contact[k])
            + distance(&curve.evaluate(t1)?, &after.contact[k]);
        if best.as_ref().is_none_or(|(b, ..)| near < *b) {
            best = Some((near, edge, t0, t1));
        }
    }
    Ok(best.map(|(_, edge, t0, t1)| (edge, t0, t1)))
}

/// The ball where it rolls, on side `k` of the chain, from the face it
/// touches at `before` onto the one it touches at `after`: touching both
/// on the edge between them, and the face on the other side, with the
/// signed radius `radius`. A station there is where the tool's spans
/// break: across it the faces' curvature may jump, which no one cubic
/// follows, and its contact lies exactly on the edge the boolean cuts the
/// face along.
///
/// Refused where the two faces meet at a crease rather than tangentially:
/// a ball rolling over it touches both at once, which no rolling-ball
/// blend of two faces is.
fn crossing(
    model: &Model,
    k: usize,
    before: &Station,
    after: &Station,
    radius: Scalar,
) -> GeopResult<Station> {
    let (from, onto) = (before.faces[k], after.faces[k]);
    let Some((edge, t0, t1)) = crossed_edge(model, (from, onto), before, after, k)? else {
        return Err(GeopError::new(format!(
            "the ball rolls from face {from} onto face {onto}, which meet in no edge"
        )));
    };
    let curve = &model.get_edge(edge)?.curve;
    let o = 1 - k;
    let surface = &model.get_face(from)?.surface;
    let other = &model.get_face(before.faces[o])?.surface;
    let (f0, f1) = (t0.midpoint().to_f64(), t1.midpoint().to_f64());
    // The ball through the edge's point at `t`, square to the face there:
    // how far it is from touching the other side, and its makings.
    #[allow(clippy::type_complexity)]
    let ball = |t: f64| -> GeopResult<(f64, Vector3, [Vector3; 2], [Vector3; 2], Feet)> {
        let sr = radius;
        let ts = Scalar::from_f64(t);
        let x = curve.evaluate(ts)?;
        let (u, v) = foot(surface, &x, before.uv[k])?;
        let n = surface.normal(u, v)?.normalize()?;
        let center = x.add(&n.prod_scalar(sr)).sharpen();
        let (uo, vo) = foot(other, &center, before.uv[o])?;
        let xo = other.evaluate(uo, vo)?;
        let no = other.normal(uo, vo)?.normalize()?;
        let gap = center.sub(&xo).prod_dot(&no).sub(sr);
        let mut contact = [x; 2];
        let mut normals = [n; 2];
        let mut uv = [sharp_in(surface, (u, v)); 2];
        contact[o] = xo;
        normals[o] = no;
        uv[o] = sharp_in(other, (uo, vo));
        Ok((gap.midpoint().to_f64(), center, contact, normals, uv))
    };
    // A bracket: from between where the contacts lie along the edge,
    // widened until the ball goes from one side of touching to the other —
    // along an edge running across the chain, both contacts lie at about
    // the same place on it, so they bracket nothing.
    let (domain_lo, domain_hi) = curve.domain();
    let (domain_lo, domain_hi) = (domain_lo.midpoint().to_f64(), domain_hi.midpoint().to_f64());
    let speed = curve.tangent(t0)?.norm().midpoint().to_f64();
    let middle = 0.5 * (f0 + f1);
    let first_reach = (0.5 * (f1 - f0).abs()).max(0.01 * radius.abs().midpoint().to_f64() / speed);
    let bracket = probe(
        first_reach,
        MOST_WIDENINGS,
        |reach| Ok(reach * 2.0),
        |&reach| -> GeopResult<_> {
            let lo = (middle - reach).max(domain_lo);
            let hi = (middle + reach).min(domain_hi);
            let g_lo = ball(lo)?.0;
            let g_hi = ball(hi)?.0;
            if g_lo.signum() != g_hi.signum() {
                return Ok(Probe::Found((lo, hi, g_lo)));
            }
            if lo <= domain_lo && hi >= domain_hi {
                return Err(GeopError::new(format!(
                    "cannot find where the ball rolls from face {from} onto face {onto} across edge {edge}: it is {g_lo:e} and {g_hi:e} off touching face {} at either end of it",
                    before.faces[o]
                )));
            }
            Ok(Probe::Next)
        },
    )?;
    let Some((lo, hi, g_lo)) = bracket else {
        return Err(GeopError::new(format!(
            "cannot find where the ball rolls from face {from} onto face {onto} across edge {edge}: a reach of {first_reach:e} doubled {MOST_WIDENINGS} times brackets no crossing"
        )));
    };
    // Bisection: where along the edge is a free choice up to where the
    // ball touches both, and the halves only ever close in on that.
    let bracket = bisect(lo, hi, 60, true, |mid| -> GeopResult<_> {
        Ok(Some(ball(mid)?.0.signum() == g_lo.signum()))
    })?;
    let t = 0.5 * (bracket.lo + bracket.hi);
    let (_, center, contact, normals, uv) = ball(t)?;
    // Tangent faces either side of the edge, where the ball touches it.
    let onto_surface = &model.get_face(onto)?.surface;
    let (u2, v2) = foot(onto_surface, &contact[k], after.uv[k])?;
    let n2 = onto_surface.normal(u2, v2)?.normalize()?;
    if !normals[k].could_be_parallel(&n2) {
        return Err(GeopError::new(format!(
            "faces {from} and {onto} meet at a crease along edge {edge} where the ball rolls across it: a rolling-ball blend only rolls from one face onto another tangent to it"
        )));
    }
    let mut faces = before.faces;
    faces[k] = from;
    section(center, contact, normals, faces, uv, true)
}

/// The blend's sections along `chain` with `n` stations to an edge (see
/// [`station_positions`]), placed as `placing` says — the balls of a
/// fillet, the chords of a chamfer — between the sections `corners` where
/// it ends at a corner's ball; where a side moves from one face onto the
/// next, and where the blend changes its rate, a section too.
fn roll_chain(
    chain: &Chain,
    n: usize,
    corners: &[Option<(f64, Station)>; 2],
    placing: &Placing,
) -> GeopResult<Rolling> {
    let links = chain.links.len() as f64;
    let place_at = &placing.place;
    // How near, in model units, the boolean applying the tool tells two
    // points apart: its searches' tolerance.
    let resolution = RemeshParams::default()
        .curve_curve_min_subdivision_size
        .upper()
        .to_f64();

    // The stations, evenly along the chain — between the corners' balls
    // where it ends at one, which are its first and last.
    let range = (
        corners[0].as_ref().map_or(-placing.reach[0], |(c, _)| *c),
        corners[1]
            .as_ref()
            .map_or(links + placing.reach[1], |(c, _)| *c),
    );
    let positions = station_positions(chain, n, range);
    let mut regular: Vec<Station> = Vec::with_capacity(positions.len());
    for (i, &c) in positions.iter().enumerate() {
        let at_corner = match i {
            0 => corners[0].as_ref(),
            _ if i == positions.len() - 1 => corners[1].as_ref(),
            _ => None,
        };
        let station = match at_corner {
            Some((_, station)) => station.clone(),
            None => place_at(c, regular.last())?,
        };
        regular.push(station);
    }

    // Where a side moves from one face onto the next between two of them;
    // a station too near one of those makes way for it.
    let m = regular.len();
    let intervals = if chain.closed { m } else { m - 1 };
    let mut keep = vec![true; m];
    let mut crossings: Vec<(f64, Station)> = Vec::new();
    for i in 0..intervals {
        let j = (i + 1) % m;
        let (ca, mut cb) = (positions[i], positions[j]);
        if j == 0 {
            cb += links;
        }
        let (a, b) = (&regular[i], &regular[j]);
        let mut found: Vec<(f64, Station)> = Vec::new();
        let mut sides = Vec::new();
        for k in 0..2 {
            if a.faces[k] != b.faces[k] {
                let ctx = with_context!("moving from face {} onto face {}", a.faces[k], b.faces[k]);
                found.push((placing.cross)(k, a, b, (ca, cb)).with_context(ctx)?);
                sides.push(k);
            }
        }
        // Both sides at once — where the faces either side change at the
        // same place, as at the seams of two surfaces of revolution on one
        // axis: one station, meeting each on its own edge.
        if let [(f0, s0), (_, s1)] = found.as_slice()
            && let Some(merged) = (placing.merge)(s0, s1)?
        {
            found = vec![(*f0, merged)];
        }
        // Both sides, apart: the side that moves second does so with the
        // other side already on its next face — not, as found from the
        // station before, on the face it has left, which the ball would
        // touch beyond its edge (by a lofted blade's seam, a hair past it).
        // So near the first that the boolean applying the tool tells them
        // apart no better — where a lofted blade's sections cross its sharp
        // edge nearly square to it, both sides cross the seam a hair apart —
        // the first alone: a span between them would put the tool's edges
        // at the seam that close together.
        if found.len() == 2 {
            let first = usize::from(found[1].0 < found[0].0);
            let (k1, k2) = (sides[first], sides[1 - first]);
            let (f1, station) = found[first].clone();
            // The first's ball, one side on its new face, the other on the
            // face it leaves — seeds from the stations either side.
            let mut moved = station.clone();
            (moved.faces[k1], moved.uv[k1]) = (b.faces[k1], b.uv[k1]);
            (moved.faces[k2], moved.uv[k2]) = (a.faces[k2], a.uv[k2]);
            let from = ca + f1 * (cb - ca);
            let ctx = with_context!(
                "moving from face {} onto face {}, after the other side moved",
                a.faces[k2],
                b.faces[k2]
            );
            let (f2, second) = (placing.cross)(k2, &moved, b, (from, cb)).with_context(ctx)?;
            found = if distance(&station.center, &second.center) < resolution {
                vec![(f1, station)]
            } else {
                vec![(f1, station), (f1 + f2 * (1.0 - f1), second)]
            };
        }
        for (f, station) in found {
            let first_end = !chain.closed && i == 0;
            let last_end = !chain.closed && j == m - 1;
            if f < 0.25 && !first_end {
                keep[i] = false;
            }
            if f > 0.75 && !last_end {
                keep[j] = false;
            }
            crossings.push((
                (ca + f * (cb - ca)).rem_euclid(if chain.closed { links } else { f64::INFINITY }),
                station,
            ));
        }
    }
    // Where the blend changes its rate, a station too, where the spans
    // break, and where the tool is halved — unless a side moves onto the
    // next face right there, its contact on the edge between them nearer
    // this station's than the boolean that applies the tool resolves: then
    // that station is this one's, which would put the tool's edges a hair
    // from that edge (a loft's seam, a station at the lateral edge's vertex
    // on it 1.5e-5 off). Near it, the regular stations make way, as for a
    // crossing.
    let near = |x: f64, c: f64| {
        let d = (x - c).abs();
        let d = if chain.closed { d.min(links - d) } else { d };
        d < 0.25 / n as f64
    };
    for &c in placing.kinks.iter().chain(&placing.halve) {
        let before = positions
            .iter()
            .rposition(|&x| x < c)
            .unwrap_or(positions.len() - 1);
        let station = place_at(c, Some(&regular[before]))?;
        let there = crossings.iter().any(|(_, crossing)| {
            (0..2).any(|k| distance(&crossing.contact[k], &station.contact[k]) < resolution)
        });
        if there {
            continue;
        }
        for (i, &x) in positions.iter().enumerate() {
            let end = !chain.closed && (i == 0 || i == m - 1);
            if near(x, c) && !end {
                keep[i] = false;
            }
        }
        crossings.push((c, station));
    }
    let mut all: Vec<(f64, Station, bool)> = positions
        .iter()
        .zip(regular)
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|((&c, s), _)| (c, s, false))
        .collect();
    all.extend(crossings.into_iter().map(|(c, s)| (c, s, true)));
    all.sort_by(|a, b| a.0.total_cmp(&b.0));

    // Between two breaks closer than the stations' spacing — where the two
    // sides move onto the next faces a little apart, across a lofted
    // blade's seam — no station falls, however the spacing is refined: the
    // span between them gets stations of its own, as many more as the
    // chain's, so that it is refined with it.
    let least = (n / FIRST_STATIONS).max(1);
    let mut filled: Vec<(f64, Station, bool)> = Vec::with_capacity(all.len());
    let last = all.len() - 1;
    let is_break = |i: usize, all: &[(f64, Station, bool)]| {
        all[i].2 || (!chain.closed && (i == 0 || i == last))
    };
    let mut from = 0;
    for i in 0..all.len() {
        if i > from && is_break(i, &all) {
            let inside = i - from - 1;
            if is_break(from, &all) && inside < least {
                let (a, b) = (all[from].0, all[i].0);
                let mut previous = all[from].1.clone();
                for j in 1..=least {
                    let c = a + (b - a) * j as f64 / (least + 1) as f64;
                    let station = place_at(c, Some(&previous))?;
                    previous = station.clone();
                    filled.push((c, station, false));
                }
            } else {
                filled.extend(all[from + 1..i].iter().cloned());
            }
            from = i;
        }
        if i == from {
            filled.push(all[i].clone());
        }
    }
    filled.extend(all[from + 1..].iter().cloned());
    let mut all = filled;

    // Where the spans break: at the crossings, at the ends of an open
    // chain; round a closed one at least at four stations about evenly
    // apart — no face of the tool closing onto itself along a seam, as a
    // revolve's quarters do not.
    let mut breaks: Vec<usize> = (0..all.len()).filter(|&i| all[i].2).collect();
    if chain.closed {
        let count = all.len();
        for q in 0..4 {
            let target = all[0].0 + q as f64 * links / 4.0;
            let nearest = (0..count)
                .min_by(|&a, &b| {
                    (all[a].0 - target)
                        .abs()
                        .total_cmp(&(all[b].0 - target).abs())
                })
                .expect("stations");
            let clear = breaks.iter().all(|&b| {
                (b as isize - nearest as isize).rem_euclid(count as isize) > 1
                    && (nearest as isize - b as isize).rem_euclid(count as isize) > 1
            });
            if clear && !breaks.contains(&nearest) {
                breaks.push(nearest);
            }
        }
        breaks.sort();
        // Start at a break, and round to it again.
        let start = breaks[0];
        all.rotate_left(start);
        for b in &mut breaks {
            *b = (*b + count - start) % count;
        }
        breaks.sort();
        for i in 0..count {
            if i > 0 && all[i].0 < all[i - 1].0 {
                all[i].0 += links;
            }
        }
        let first = all[0].clone();
        all.push((first.0 + links, first.1, true));
        breaks.push(count);
    } else {
        breaks.insert(0, 0);
        breaks.push(all.len() - 1);
        breaks.dedup();
    }

    // The sections between stations, to measure each span against.
    let mut between = Vec::with_capacity(all.len() - 1);
    for w in breaks.windows(2) {
        let span_intervals = w[1] - w[0];
        for (local, i) in (w[0]..w[1]).enumerate() {
            let (c, next) = (all[i].0, all[i + 1].0);
            let mut inside = Vec::new();
            let mut previous = all[i].1.clone();
            for &(a, b) in true_point_fractions(local, span_intervals) {
                let f = a as f64 / b as f64;
                let section = place_at(c + f * (next - c), Some(&previous))?;
                previous = section.clone();
                inside.push((f, section));
            }
            between.push(inside);
        }
    }
    Ok(Rolling {
        positions: all.iter().map(|(c, ..)| *c).collect(),
        stations: all.into_iter().map(|(_, s, _)| s).collect(),
        breaks,
        between,
    })
}

/// How a fillet's balls are placed along `chain` of `model` (see
/// [`Placing`]), their radius as `law` says, on the side `side` of the
/// faces: rolled ([`place`]), rolling from face to face ([`crossing`]),
/// and where a varying radius changes its rate — at the chain's vertices —
/// a ball of its own.
fn fillet_placing<'a>(
    model: &'a NamedModel,
    chain: &'a Chain,
    law: &'a RadiusLaw,
    side: Scalar,
    run_on: &'a [Option<(RunOn, f64)>; 2],
) -> Placing<'a> {
    let topology = model.topology();
    let links = chain.links.len() as f64;
    // Past the chain's ends the radius stays what it is there.
    let radius_at = move |c: f64| {
        let (link, x) = locate(chain, c.clamp(0.0, links));
        side.mul(law.radius(link, x))
    };
    let place_at = move |c: f64, previous: Option<&Station>| -> GeopResult<Station> {
        let past = if c < 0.0 {
            run_on[0].as_ref().map(|(on, _)| (on, c))
        } else if c > links {
            run_on[1].as_ref().map(|(on, _)| (on, c - (links - 1.0)))
        } else {
            None
        };
        if let Some((on, fraction)) = past {
            let ctx = with_context!(
                "rolling the ball on to {fraction} of the way along edge {}, past the chain's end",
                on.link.edge
            );
            return on.place(fraction, radius_at(c), previous).with_context(ctx);
        }
        let (link, x) = locate(chain, c);
        let ctx = with_context!(
            "placing the ball {x} of the way along edge {}",
            chain.links[link].edge
        );
        let placed = place(
            topology,
            chain,
            link,
            Scalar::from_f64(x),
            radius_at(c),
            previous,
        )
        .with_context(ctx);
        // In the half of an end link next to an end it runs on past, the
        // ball may touch one of the link's faces beyond its edge already —
        // the edge it shares with the third face there — and the face goes
        // on past it, in the air: the ball touches the link's faces as they
        // are carried on there, not some other face across a crease. Only
        // the third face meets them at that vertex.
        let near_end = if link == 0 && x < 0.5 {
            run_on[0].as_ref()
        } else if link + 1 == chain.links.len() && x >= 0.5 {
            run_on[1].as_ref()
        } else {
            None
        };
        match (placed, near_end) {
            (Ok(station), Some(_)) if station.faces == chain.links[link].faces => Ok(station),
            (_, Some((on, _))) => on.place(x, radius_at(c), previous).with_context(ctx),
            (placed, None) => placed,
        }
    };
    let cross = move |k: usize,
                      a: &Station,
                      b: &Station,
                      (ca, cb): (f64, f64)|
          -> GeopResult<(f64, Station)> {
        // How far along it is: where the plane square to the chain through
        // its center crosses the chain, as every station's does — then the
        // radius there, and the ball again with it.
        let along = |x: &Station| -> GeopResult<f64> {
            let off = |c: f64| -> GeopResult<f64> {
                let (link, f) = locate(chain, c);
                let (p, t) = chain.links[link].at(topology, Scalar::from_f64(f))?;
                Ok(x.center.sub(&p).prod_dot(&t).midpoint().to_f64())
            };
            let side = |c| -> GeopResult<_> { Ok(Some(off(c)?.is_sign_positive())) };
            let Some(Bracket { lo, hi, .. }) = sign_changes([ca, cb], side)
                .bisected(60, false)
                .next()
                .transpose()?
            else {
                return Ok(0.5);
            };
            Ok((0.5 * (lo + hi) - ca) / (cb - ca))
        };
        let guess = crossing(topology, k, a, b, radius_at(0.5 * (ca + cb)))?;
        let f = along(&guess)?;
        let station = crossing(topology, k, a, b, radius_at(ca + f * (cb - ca)))?;
        Ok((along(&station)?, station))
    };
    let merge = move |s0: &Station, s1: &Station| -> GeopResult<Option<Station>> {
        let scale = s0.center.sub(&s0.contact[0]).norm().upper().to_f64();
        if distance(&s0.center, &s1.center) > DEVIATION * scale {
            return Ok(None);
        }
        Ok(Some(section(
            s0.center,
            [s0.contact[0], s1.contact[1]],
            [s0.normals[0], s1.normals[1]],
            [s0.faces[0], s1.faces[1]],
            [s0.uv[0], s1.uv[1]],
            true,
        )?))
    };
    Placing {
        place: Box::new(place_at),
        cross: Box::new(cross),
        merge: Box::new(merge),
        kinks: Vec::new(),
        reach: run_on
            .each_ref()
            .map(|r| r.as_ref().map_or(0.0, |(_, reach)| *reach)),
        halve: None,
    }
}

/// The positions of the vertices of `chain` between two links, where the
/// blend changes its rate: where the radius varies, it does so linearly
/// between them; and the edges of a chain, and the faces either side, are
/// only tangent where they meet, so the sections, square to them, turn
/// there at a rate that jumps — as where a lofted blade's edge passes one
/// of its sections.
fn turns(chain: &Chain) -> Vec<f64> {
    let inner = if chain.closed { 0 } else { 1 };
    (inner..chain.links.len()).map(|v| v as f64).collect()
}

/// How a chamfer's chords are placed along `chain` of `model`, `distances`
/// into the faces on its left and right (see [`Placing`], [`plan_chamfer`]).
fn chamfer_placing<'a>(
    model: &'a NamedModel,
    chain: &'a Chain,
    distances: [f64; 2],
) -> Placing<'a> {
    let topology = model.topology();
    let place_at = move |c: f64, previous: Option<&Station>| -> GeopResult<Station> {
        let (link, x) = locate(chain, c);
        let ctx = with_context!(
            "chamfering {x} of the way along edge {}",
            chain.links[link].edge
        );
        chamfer_station(
            topology,
            chain,
            link,
            Scalar::from_f64(x),
            distances,
            previous,
        )
        .with_context(ctx)
    };
    let cross = move |k: usize, a: &Station, b: &Station, range: (f64, f64)| {
        chamfer_crossing(topology, chain, k, a, b, range, distances)
    };
    let merge = move |s0: &Station, s1: &Station| -> GeopResult<Option<Station>> {
        let (p0, p1) = (chord_edge_point(s0)?, chord_edge_point(s1)?);
        if distance(&p0, &p1) > DEVIATION * distances[0].min(distances[1]) {
            return Ok(None);
        }
        Ok(Some(chord(
            &p0,
            [s0.contact[0], s1.contact[1]],
            [s0.normals[0], s1.normals[1]],
            [s0.faces[0], s1.faces[1]],
            [s0.uv[0], s1.uv[1]],
        )?))
    };
    Placing {
        place: Box::new(place_at),
        cross: Box::new(cross),
        merge: Box::new(merge),
        kinks: Vec::new(),
        reach: [0.0; 2],
        halve: None,
    }
}

/// Everything a rolling-ball blend of one chain needs: the chain, which way
/// it bends, its tool, and the contact points to check against the faces
/// (see [`check_touches`]).
#[derive(Clone, Debug)]
pub(crate) struct Rolled {
    /// The name of the station the tool is halved at, if it is (see
    /// [`Rolled::halves`]).
    halved_at: Option<String>,
    /// The surfaces the tool is cut off by at its start and its end, if it
    /// is (see [`run_ons`]): the third face's, carried on past the vertex,
    /// which the tool runs on through. Only what is on the chain's side of
    /// it is the blend's.
    pub trims: [Option<NurbSurface3D>; 2],
    pub chain: Chain,
    pub bend: Bend,
    pub tool: Tool,
    /// Per span of the tool, the curves enclosing where it touches the
    /// faces on the chain's left and right (see [`check_touches`]).
    contacts: Vec<[NurbCurve3D; 2]>,
    /// Per link, where the ball nearest its middle touches the faces: a
    /// point of each contact within the chain (see [`check_touches`]).
    middles: Vec<[Vector3; 2]>,
}

impl Rolled {
    /// How wide its tool is, as a plain number: the farthest any of its
    /// stations' vertices lies from another — how far the faces it is
    /// cut off by are carried on (see [`crate::blend::cuts`]).
    pub fn width(&self) -> f64 {
        let mut widest = 0.0f64;
        for station in &self.tool.stations {
            for a in &station.vertices {
                for b in &station.vertices {
                    widest = widest.max(distance(a, b));
                }
            }
        }
        widest
    }

    /// Whether the tool is built in halves: where its chain is open and
    /// starts and ends at one vertex, on its own (see [`halving`]) — a
    /// loop round a face with one corner, a lofted blade's tip round to
    /// its trailing edge. Its tool's two ends meet there, overlapping —
    /// both blend the corner — and one solid cannot overlap itself: it is
    /// built in two halves (see [`Rolled::halves`]), applied one after
    /// the other as two blends meeting at an outward corner are.
    pub fn is_halved(&self) -> bool {
        self.halved_at.is_some()
    }

    /// The tool in two at the station it is halved at (see [`halving`]),
    /// where its spans break (see [`roll_chain`]), each half capped there
    /// in the plane of the section: the first's facing on along the chain,
    /// the second's back. Both caps hold the same section, to the bit.
    pub fn halves(&self) -> GeopResult<[Tool; 2]> {
        let tool = &self.tool;
        let count = tool.stations.len();
        let b = (self.halved_at.as_ref())
            .and_then(|name| tool.stations.iter().position(|s| &s.name == name))
            .filter(|&b| b > 0 && b + 1 < count)
            .ok_or_else(|| {
                GeopError::new(format!(
                    "the tool has no station of its own to be halved at ({:?})",
                    self.halved_at
                ))
            })?;
        let station = &tool.stations[b];
        let [a, e, c] = [0, 1, 2].map(|k| station.points[k].dehomogenized());
        let (a, e, c) = (a?, e?, c?);
        let mut m = a.sub(&e).prod_cross(&c.sub(&e)).normalize()?;
        // Along the chain: from the station before to the one after.
        let along = tool.stations[b + 1].vertices[0].sub(&tool.stations[b - 1].vertices[0]);
        let facing = m.prod_dot(&along);
        if facing.definitely_less(Scalar::ZERO) {
            m = m.neg();
        } else if !facing.definitely_greater(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "the tool's section at {:?}, where it is halved, could stand along the chain",
                station.vertices[0]
            )));
        }
        let towards = a.sub(&e);
        let e1 = towards
            .sub(&m.prod_scalar(towards.prod_dot(&m)))
            .normalize()?;
        let forward = [e, e1, m.prod_cross(&e1)];
        // The same plane facing back: `-e1` crossed with `e2` is `-m`, and
        // a point's coordinates in it are its own with `x` negated — so
        // that it is embedded back to the very same point.
        let back = [e, e1.neg(), forward[2]];
        let flat = station.points.map(|p| flatten(&p, &forward));
        let points = flat.map(|p| embed(&p, &forward));
        let vertices = station
            .vertices
            .map(|v| embed(&flatten_point(&v, &forward), &forward).head::<3>());
        let mirrored = flat.map(|p| Vector3::from_array([p[0].neg(), p[1], p[2]]));
        let mut first = Tool {
            stations: tool.stations[..=b].to_vec(),
            spans: tool.spans[..b].to_vec(),
            ..tool.clone()
        };
        let mut second = Tool {
            stations: tool.stations[b..].to_vec(),
            spans: tool.spans[b..].to_vec(),
            ..tool.clone()
        };
        let last = first.stations.len() - 1;
        first.stations[last] = ToolStation {
            points,
            vertices,
            name: station.name.clone(),
            cap: Some(Cap {
                plane: forward,
                flat,
            }),
        };
        second.stations[0] = ToolStation {
            points,
            vertices,
            name: station.name.clone(),
            cap: Some(Cap {
                plane: back,
                flat: mirrored,
            }),
        };
        let rows = &mut first.spans[b - 1].rows;
        let k = rows.len() - 1;
        rows[k] = points;
        second.spans[0].rows[0] = points;
        Ok([first, second])
    }

    /// The faces the tool's section touches first and second at its last
    /// station, `at_end`, or its first.
    pub fn contact_faces(&self, at_end: bool) -> [FaceId; 2] {
        let link = if at_end {
            self.chain.links.last().expect("links")
        } else {
            &self.chain.links[0]
        };
        if self.tool.sides == ["a", "b"] {
            link.faces
        } else {
            [link.faces[1], link.faces[0]]
        }
    }
}

/// The plain distance between two points.
fn distance(a: &Vector3, b: &Vector3) -> f64 {
    a.sub(b).norm().midpoint().to_f64()
}

/// The control rows of the tool's span `span` through the stations of
/// `rolling`, interpolated at their parameters (see the module docs); the
/// contact rows not yet widened.
fn span_curves(rolling: &Rolling, span: (usize, usize)) -> GeopResult<[NurbCurve3D; 4]> {
    let params = rolling.params(span);
    let points: Vec<[Vector4; 4]> = rolling.stations[span.0..=span.1]
        .iter()
        .map(section_points)
        .collect();
    let curve = |k: usize| {
        let values: Vec<Vector4> = points.iter().map(|p| p[k]).collect();
        NurbCurve::interpolate_homogeneous(&values, &params, 3)
    };
    Ok([curve(0)?, curve(1)?, curve(2)?, curve(3)?])
}

/// How far the span `curves` strays from the balls `between` at their
/// parameters: from either contact, and from the middle of the section —
/// or, for a chamfer's chords, from each of their control points.
fn span_deviation(
    curves: &[NurbCurve3D; 4],
    between: &[Vec<(Scalar, Station)>],
) -> GeopResult<(f64, String)> {
    // Past the third face a blend runs on through, it is free (see
    // [`Station::aloof`]).
    let between = || {
        between
            .iter()
            .flatten()
            .filter(|(_, station)| !station.aloof)
    };
    if between().any(|(_, station)| station.chord) {
        let mut worst = (0.0f64, String::new());
        for (s, station) in between() {
            for (k, truth) in section_points(station).iter().enumerate() {
                let (got, truth) = (curves[k].evaluate(*s)?, truth.dehomogenized()?);
                let d = distance(&got, &truth);
                if d > worst.0 {
                    worst = (
                        d,
                        format!(
                            "control point {k} of the chamfer's section at {:?} of the span: {got:?} where it is {truth:?}",
                            s.midpoint().to_f64()
                        ),
                    );
                }
            }
        }
        return Ok(worst);
    }
    let blend = wall_surface(&span_of(curves, None), 0)?;
    let half = Scalar::ONE.div(Scalar::TWO)?;
    let mut worst = (0.0f64, String::new());
    let mut note = |d: f64, what: &str, s: &Scalar, got: &Vector3, truth: &Vector3| {
        if d > worst.0 {
            worst = (
                d,
                format!(
                    "{what} at {:?} of the span: {got:?} where the ball has {truth:?}",
                    s.midpoint().to_f64()
                ),
            );
        }
    };
    // Measured from where each lies nearest: a ball placed a little further
    // along — where the edge it was placed by is only known that well — is
    // no deviation.
    for (s, station) in between() {
        for k in [0, 2] {
            let truth = station.contact[k / 2];
            let t = curves[k].closest_parameter_from(&truth, *s)?;
            let got = curves[k].evaluate(t)?;
            note(distance(&got, &truth), "a contact", s, &got, &truth);
            // The blend has to leave the face towards the ball, never dip
            // through it: where it is tangent only approximately, that is
            // what keeps its one meeting with the face its contact curve.
            let (_, across) =
                blend.derivatives(t, if k == 0 { Scalar::ZERO } else { Scalar::ONE })?;
            let leaving = if k == 0 { across } else { across.neg() };
            let towards = leaving.prod_dot(&station.center.sub(&truth));
            if !towards.definitely_greater(Scalar::ZERO) {
                // How steeply it leaves, in radians: the tilt it should.
                let steep = towards.div(leaving.norm().mul(station.center.sub(&truth).norm()));
                return Ok((
                    f64::INFINITY,
                    format!(
                        "at {:?} of the span the blend could dip through face {} at {truth:?}: it leaves the face {steep:?} towards the ball, where it should {TILT:e}",
                        s.midpoint().to_f64(),
                        station.faces[k / 2]
                    ),
                ));
            }
        }
        let truth = section_points(station);
        let middle = truth[0]
            .add(&truth[1].prod_scalar(Scalar::TWO))
            .add(&truth[2])
            .dehomogenized()?;
        let (u, v) = foot(&blend, &middle, (*s, half))?;
        let got = blend.evaluate(u, v)?;
        note(
            distance(&got, &middle),
            "the section's middle",
            s,
            &got,
            &middle,
        );
    }
    Ok(worst)
}

/// The span whose rows are the control points of `curves` — compatible,
/// one per section control point (see [`section_points`]) — named `name`.
fn span_of(curves: &[NurbCurve3D; 4], name: Option<String>) -> ToolSpan {
    ToolSpan {
        degree: curves[0].degree,
        knots: curves[0].knot_vector.clone(),
        rows: (0..curves[0].control_points.len())
            .map(|i| [0, 1, 2, 3].map(|k| curves[k].control_points[i]))
            .collect(),
        name,
    }
}

/// The contact checks of the span `span`: every station's contact point at
/// its parameter, and every ball between, for the contact row `k` (0 or 2).
fn contact_checks(rolling: &Rolling, span: (usize, usize), k: usize) -> Vec<(Scalar, Vector3)> {
    let params = rolling.params(span);
    let stations = params.into_iter().zip(&rolling.stations[span.0..=span.1]);
    let between = rolling.between(span);
    // Past the third face a blend runs on through, where it touches the
    // faces carried on bounds no face (see [`Station::aloof`]).
    stations
        .filter(|(_, station)| !station.aloof)
        .map(|(s, station)| (s, station.contact[k / 2]))
        .chain(
            between
                .iter()
                .flatten()
                .filter(|(_, station)| !station.aloof)
                .map(|(s, station)| (*s, station.contact[k / 2])),
        )
        .collect()
}

/// How an open chain's tool ends at a vertex: the vertex, the end, the
/// third face there and the way out of the chain.
type ChainEnd = (VertexId, End, Option<FaceId>, Vector3);

/// How the tool of `chain`, bending `bend`, ends at the start and the end
/// of it, if it is open (see [`tool_end`]) — as `meeting` says where it is
/// joined to a corner's ball or mitred: decided before anything is rolled,
/// so that an end that is not supported is refused by name.
fn chain_ends(
    model: &Model,
    chain: &Chain,
    bend: Bend,
    meeting: [Option<End>; 2],
) -> GeopResult<Option<[ChainEnd; 2]>> {
    if chain.closed {
        return Ok(None);
    }
    let mut ends = Vec::new();
    for (k, (vertex, faces, out)) in chain.ends(model)?.into_iter().enumerate() {
        if let Some(end) = &meeting[k] {
            ends.push((vertex, end.clone(), None, out));
            continue;
        }
        let ctx = with_context!("the blend's end at vertex {vertex}");
        let (kind, face) = tool_end(model, vertex, faces, &out, bend).with_context(ctx)?;
        ends.push((vertex, kind, Some(face), out));
    }
    let [a, b]: [ChainEnd; 2] = ends
        .try_into()
        .map_err(|_| GeopError::new("an open chain has two ends"))?;
    Ok(Some([a, b]))
}

/// Where an open chain ends at a corner's ball (see [`crate::corner`]):
/// the ball's center, and where it touches the faces on the chain's left
/// and right there.
#[derive(Clone, Debug)]
pub(crate) struct ChainCorner {
    pub center: Vector3,
    pub contacts: [Vector3; 2],
}

/// Where the open `chain`'s blend runs into a wall at its start and its end
/// (see [`End::Wall`]): the vertex, the unit direction from it back along
/// the chain, and the faces either side there — for the mitres where two
/// do at one vertex (see [`crate::blend`]).
#[allow(clippy::type_complexity)]
pub(crate) fn chain_walls(
    model: &Model,
    chain: &Chain,
) -> GeopResult<[Option<(VertexId, Vector3, [FaceId; 2])>; 2]> {
    let mut walls = [None, None];
    if chain.closed {
        return Ok(walls);
    }
    let bend = chain_bend(model, chain)?;
    for (k, (vertex, faces, out)) in chain.ends(model)?.into_iter().enumerate() {
        // An end that is not supported is refused when the chain is rolled.
        if let Ok((End::Wall { .. }, _)) = tool_end(model, vertex, faces, &out, bend) {
            walls[k] = Some((vertex, out.neg(), faces));
        }
    }
    Ok(walls)
}

/// Which way the faces either side of `chain` bend: the same all along, or
/// an error.
pub(crate) fn chain_bend(model: &Model, chain: &Chain) -> GeopResult<Bend> {
    // Which way it bends, where the chain starts.
    let first = &chain.links[0];
    // The way into the face on the left — its normal crossed with the way
    // its coedge runs, the chain's — against the right one's normal.
    let bend_at = |link: &Link, fraction: Scalar| -> GeopResult<Bend> {
        let (p, t) = link.at(model, fraction)?;
        let normal = |face| {
            let surface = &model.get_face(face)?.surface;
            surface.normal_at(&p, MAX_NODES, min_subdivision_size())
        };
        let (n_l, n_r) = (normal(link.faces[0])?, normal(link.faces[1])?);
        if n_l.could_be_parallel(&n_r) {
            return Err(GeopError::new(format!(
                "faces {} and {} could meet tangentially at edge {}: there is no corner to blend",
                link.faces[0], link.faces[1], link.edge
            )));
        }
        bend(n_l.prod_cross(&t).prod_dot(&n_r))
    };
    let bend = bend_at(first, Scalar::from_f64(0.5))?;
    for (i, link) in chain.links.iter().enumerate().skip(1) {
        if bend_at(link, Scalar::from_f64(0.5))? != bend {
            return Err(GeopError::new(format!(
                "the edge's tangent chain turns from {bend:?} to the other way along edge {} (link {i}): blend its parts separately",
                link.edge
            )));
        }
    }
    Ok(bend)
}

/// Where along the open `chain` — near its start (`k = 0`) or its end
/// (`k = 1`) — the plane square to it holds the corner ball's `center`:
/// where the blend meets the ball, set back from the chain's end. The
/// first such place from the end, in plain numbers: where along it is a
/// free choice up to where the ball's center is, and the station there is
/// built from the ball exactly.
fn corner_position(model: &Model, chain: &Chain, k: usize, center: &Vector3) -> GeopResult<f64> {
    let links = chain.links.len() as f64;
    // How far further into the chain than the plane square to it at `c`
    // the center is.
    let ahead = |c: f64| -> GeopResult<f64> {
        let (link, f) = locate(chain, c);
        let (p, t) = chain.links[link].at(model, Scalar::from_f64(f))?;
        let off = center.sub(&p).prod_dot(&t).midpoint().to_f64();
        Ok(if k == 0 { off } else { -off })
    };
    let end = if k == 0 { 0.0 } else { links };
    let inward = if k == 0 { 1.0 } else { -1.0 };
    let step = 1.0 / 32.0;
    // Marched on from the end until the center is no longer ahead: a
    // bracket, the step before and the one there.
    let missed = || {
        GeopError::new(format!(
            "the corner's ball, centered at {center:?}, meets no section of the blend along its chain"
        ))
    };
    let mut near = end;
    let far = probe(
        end + inward * step,
        (links / step) as usize + 2,
        |far| Ok(far + inward * step),
        |&far| -> GeopResult<_> {
            if (far - end).abs() > links {
                return Err(missed());
            }
            if ahead(far)? <= 0.0 {
                return Ok(Probe::Found(far));
            }
            near = far;
            Ok(Probe::Next)
        },
    )?
    .ok_or_else(missed)?;
    let Bracket { lo, hi, .. } = bisect(near, far, 60, false, |mid| -> GeopResult<_> {
        Ok(Some(ahead(mid)? > 0.0))
    })?;
    Ok(0.5 * (lo + hi))
}

/// Plans the rolling-ball blend of `chain` with `radii` (see the module
/// docs): the ball rolled along it — up to the balls `corners` where it
/// ends at one, at its start and its end — the tool skinned through its
/// stations, refined until it follows the ball, and its ends decided.
///
/// Where it meets a corner's ball, its last station is that ball's great
/// arc exactly, untilted, as the corner's piece of it is.
pub(crate) fn plan_rolled(
    model: &NamedModel,
    chain: Chain,
    radii: &Radii,
    corners: [Option<ChainCorner>; 2],
) -> GeopResult<Rolled> {
    let topology = model.topology();
    let mut law = RadiusLaw::new(model, &chain, radii)?;
    let bend = chain_bend(topology, &chain)?;
    let side = bend.side();
    // The stations where the chain meets a corner's ball, exactly, and how
    // its tool ends there.
    let mut at_corners: [Option<(f64, Station)>; 2] = [None, None];
    let mut meeting_ends: [Option<End>; 2] = [None, None];
    for (k, corner) in corners.iter().enumerate() {
        let link = if k == 0 {
            &chain.links[0]
        } else {
            chain.links.last().expect("links")
        };
        let Some(corner) = corner else {
            continue;
        };
        let ctx = with_context!("the blend's end at the corner ball at {:?}", corner.center);
        let mut uv = [(Scalar::ZERO, Scalar::ZERO); 2];
        let mut normals = [Vector3::zero(); 2];
        for side in 0..2 {
            let surface = &topology.get_face(link.faces[side])?.surface;
            uv[side] = seed_on(surface, &corner.contacts[side]).with_context(ctx)?;
            normals[side] = surface.normal(uv[side].0, uv[side].1)?.normalize()?;
        }
        let station = section(
            corner.center,
            corner.contacts,
            normals,
            link.faces,
            uv,
            false,
        )
        .with_context(ctx)?;
        let position = corner_position(topology, &chain, k, &corner.center).with_context(ctx)?;
        // The radius reaches the corner's where the chain meets its ball.
        let (link, fraction) = locate(&chain, position);
        if (k == 0 && link == 0) || (k == 1 && link == chain.links.len() - 1) {
            law.ends[k] = Some(fraction);
        }
        at_corners[k] = Some((position, station));
        meeting_ends[k] = Some(End::Corner {
            setback: Scalar::from_f64(position),
        });
    }
    let decided_ends = chain_ends(topology, &chain, bend, meeting_ends)?;
    let (run_on, trims) = run_ons(model, &chain, &law, side, &decided_ends)?;
    let halve = halving(topology, &chain, &corners)?;
    let (rolling, curves) = {
        let mut placing = fillet_placing(model, &chain, &law, side, &run_on);
        placing.kinks = turns(&chain);
        placing.halve = halve;
        refined(law.smallest(), |n| {
            roll_chain(&chain, n, &at_corners, &placing)
        })?
    };
    let halved_at = halved_at(&rolling, halve)?;
    let rolled_on = run_on.each_ref().map(Option::is_some);
    let mut rolled = assemble(
        topology,
        chain,
        bend,
        rolling,
        curves,
        decided_ends,
        rolled_on,
    )?;
    rolled.trims = trims;
    rolled.halved_at = halved_at;
    Ok(rolled)
}

/// The name of the station of the tool `rolling` makes (see [`assemble`])
/// at the position `halve`, where it is halved, if it is: the one where its
/// span there starts. An error if no span starts there — a side moving
/// from face to face right there took the station's place.
fn halved_at(rolling: &Rolling, halve: Option<f64>) -> GeopResult<Option<String>> {
    let Some(halve) = halve else {
        return Ok(None);
    };
    let spans = rolling.spans();
    let j = (spans.iter())
        .position(|&(a, _)| rolling.positions[a] == halve)
        .ok_or_else(|| {
            GeopError::new(format!(
                "the blend's tool was to be halved at position {halve} along its chain, where no station of its own is: a side moves from face to face right there"
            ))
        })?;
    Ok(Some(format!("st{j}")))
}

/// Where along `chain` its tool is halved (see [`Rolled::halves`]), if it
/// ends where it starts, on its own — joined to no corner's ball: the
/// middle of its longest link, where it runs along one face either side,
/// clear of where it moves from face to face and of its ends. A section
/// on the edge between two faces would put the cap between the halves on
/// that edge's point.
fn halving(
    model: &Model,
    chain: &Chain,
    corners: &[Option<ChainCorner>; 2],
) -> GeopResult<Option<f64>> {
    let vertices = chain.vertices(model)?;
    if chain.closed || vertices.first() != vertices.last() || corners.iter().any(Option::is_some) {
        return Ok(None);
    }
    let mut longest = (0, f64::NEG_INFINITY);
    for (i, link) in chain.links.iter().enumerate() {
        let (length, _) = model.get_edge(link.edge)?.curve.length()?;
        let length = length.midpoint().to_f64();
        if length > longest.1 {
            longest = (i, length);
        }
    }
    Ok(Some(longest.0 as f64 + 0.5))
}

/// Where the fillet of `chain` runs on, rolled, past its start and its end
/// (see [`RunOn`]), through the third face there, and is cut off by it:
/// where its section at the vertex does not lie in that face — a curved
/// face, or a plane standing at an angle to the faces either side. A cut
/// running out of the solid through it ends in the air, past it; any other
/// end is cut off by the face itself (see [`Trim`]). With each, how far, in
/// positions: as far again as the tool is wide, over how squarely the
/// chain crosses the face, as a straight blend runs out — doubled until
/// the last section is clear of the face's tangent plane at the vertex, its
/// control polygon, which holds it, wholly past it. And the trims.
#[allow(clippy::type_complexity)]
fn run_ons(
    model: &NamedModel,
    chain: &Chain,
    law: &RadiusLaw,
    side: Scalar,
    ends: &Option<[ChainEnd; 2]>,
) -> GeopResult<([Option<(RunOn, f64)>; 2], [Option<NurbSurface3D>; 2])> {
    let mut run_on = [None, None];
    let mut trims = [None, None];
    let Some(ends) = ends else {
        return Ok((run_on, trims));
    };
    let topology = model.topology();
    let links = chain.links.len() as f64;
    let none = [None, None];
    let placing = fillet_placing(model, chain, law, side, &none);
    for (k, (vertex, kind, face, out)) in ends.iter().enumerate() {
        let (Some(face), trimmed) = (
            face,
            match kind {
                End::RunOut { .. } => false,
                End::Flush { unless: Some(_) } | End::Wall { unless: Some(_) } => true,
                _ => continue,
            },
        ) else {
            continue;
        };
        let ctx = with_context!("the blend's end through face {face} at vertex {vertex}");
        let at = if k == 0 { 0.0 } else { links };
        let sr = side.mul(Scalar::from_f64(law.at_end(k)));
        let surface = &topology.get_face(*face)?.surface;
        let point = topology.get_vertex(*vertex)?.point;
        let mut normal = surface.normal_at(&point, MAX_NODES, min_subdivision_size())?;
        if normal.prod_dot(out).definitely_less(Scalar::ZERO) {
            normal = normal.neg();
        }
        // At the vertex, the ball may touch a face beyond its edge already,
        // as carried on past it (see [`fillet_placing`]).
        let station = match (placing.place)(at, None) {
            Ok(station) => station,
            Err(_) => {
                let reach = 4.0 * law.at_end(k);
                let on = RunOn::new(topology, chain, k == 1, *vertex, reach, (point, normal))
                    .with_context(ctx)?;
                let fraction = if k == 0 { 0.0 } else { 1.0 };
                on.place(fraction, sr, None).with_context(ctx)?
            }
        };
        if !trimmed
            && surface.as_plane()?.is_some()
            && end_plane(topology, &station, *face, out).is_ok()
        {
            continue;
        }
        let squareness = normal.prod_dot(out);
        let width = [station.contact[0], station.contact[1], station.apex]
            .iter()
            .map(|p| distance(p, &station.corner))
            .fold(0.0, f64::max);
        let mut length = run_out_length(width, &squareness).upper().to_f64();
        // Past a face it leaves nearly along, a blend would run on farther
        // than its edge is long, and its faces be carried on as far.
        let link = if k == 0 {
            chain.first()?
        } else {
            chain.last()?
        };
        let (edge_length, _) = topology.get_edge(link.edge)?.curve.length()?;
        if length > edge_length.lower().to_f64() {
            return Err(GeopError::new(format!(
                "the edge crosses face {face} at vertex {vertex} so nearly along it (the way it runs on, dotted with the face's normal, is {squareness:?}) that the blend would run on {length} past it, farther than the edge is long: only an edge crossing the third face more squarely is blended there"
            )))
            .with_context(ctx);
        }
        let mut found = None;
        // How far past the face's tangent plane each try's section got,
        // its least control point: what to say if none got clear.
        let mut tried: Vec<(f64, Scalar)> = Vec::new();
        for _ in 0..MOST_RUN_OUT_DOUBLINGS {
            let ctx = with_context!(
                "{ctx_name}, rolled on {length} (past the face's tangent plane by {tried:?} before)",
                ctx_name = format!("the blend's end through face {face} at vertex {vertex}")
            );
            let on = RunOn::new(topology, chain, k == 1, *vertex, length, (point, normal))
                .with_context(ctx)?;
            let reach = length / on.scale;
            let fraction = if k == 0 { -reach } else { 1.0 + reach };
            let last = on.place(fraction, sr, Some(&station)).with_context(ctx)?;
            let past = [last.contact[0], last.corner, last.contact[1], last.apex]
                .iter()
                .map(|p| p.sub(&point).prod_dot(&normal))
                .fold(None, |least: Option<Scalar>, d| {
                    Some(least.map_or(d, |l| l.min(d)))
                })
                .expect("four points");
            if past.definitely_greater(Scalar::ZERO) {
                found = Some((on, reach));
                break;
            }
            tried.push((length, past));
            length *= 2.0;
        }
        let Some(found) = found else {
            return Err(GeopError::new(format!(
                "the blend runs on through face {face} at vertex {vertex}, but rolled on {length} past it its section is still not clear of the face (past its tangent plane by {tried:?}): the face turns back across it"
            )))
            .with_context(ctx);
        };
        if trimmed {
            let ctx = with_context!("carrying face {face} on past vertex {vertex}");
            let carried = carried_on_past(surface, &point, length).with_context(ctx)?;
            trims[k] = Some(around(&carried, &point, 2.0 * length).with_context(ctx)?);
        }
        run_on[k] = Some(found);
    }
    Ok((run_on, trims))
}

/// How often the length a blend runs on past a face (see [`run_ons`]) is
/// doubled at most: `2^8` times as far again as the tool is wide is past
/// any face it leaves the solid through.
const MOST_RUN_OUT_DOUBLINGS: usize = 8;

/// The plane `station`'s section lies in, oriented along `out`, as a cap's
/// `(origin, e1, e2)`: where a blend run on past the solid (see [`RunOn`])
/// is capped, in the air.
fn own_plane(station: &Station, out: &Vector3) -> GeopResult<[Vector3; 3]> {
    let mut m = station.normals[0]
        .prod_cross(&station.normals[1])
        .normalize()?;
    let along = m.prod_dot(out);
    if along.definitely_less(Scalar::ZERO) {
        m = m.neg();
    } else if !along.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "the blend's last section, at {:?}, could stand along the way it runs on",
            station.center
        )));
    }
    let origin = station.center;
    let towards = station.contact[0].sub(&origin);
    let e1 = towards
        .sub(&m.prod_scalar(towards.prod_dot(&m)))
        .normalize()?;
    Ok([origin, e1, m.prod_cross(&e1)])
}

/// Plans the chamfer of `chain`, `distances` into the faces on its left and
/// right: rolled along it as a fillet is (see the module docs), its
/// sections chords. At each station, in the plane square to the chain, the
/// chord runs between the faces' points that distance from the edge — as
/// the crow flies, which on a plane is as far along it — and its apex is
/// as far beyond the edge as the chord's middle is before it. Its contact
/// rows are widened to enclose the true contacts as a fillet's are; unlike
/// a fillet's, the chamfer crosses the faces there, so needs no tilt.
pub(crate) fn plan_chamfer(
    model: &NamedModel,
    chain: Chain,
    distances: [f64; 2],
) -> GeopResult<Rolled> {
    if !distances.iter().all(|&d| d > 0.0) {
        return Err(GeopError::new(format!(
            "a chamfer's distances have to be positive, not {distances:?}"
        )));
    }
    let topology = model.topology();
    let bend = chain_bend(topology, &chain)?;
    let decided_ends = chain_ends(topology, &chain, bend, [None, None])?;
    let halve = halving(topology, &chain, &[None, None])?;
    let smallest = distances[0].min(distances[1]);
    let (rolling, curves) = {
        let mut placing = chamfer_placing(model, &chain, distances);
        placing.kinks = turns(&chain);
        placing.halve = halve;
        refined(smallest, |n| roll_chain(&chain, n, &[None, None], &placing))?
    };
    let halved_at = halved_at(&rolling, halve)?;
    let mut rolled = assemble(
        topology,
        chain,
        bend,
        rolling,
        curves,
        decided_ends,
        [false; 2],
    )?;
    rolled.halved_at = halved_at;
    Ok(rolled)
}

/// The sections `roll` places along a chain, `n` to an edge, doubled from
/// [`FIRST_STATIONS`] until the spans skinned through them follow those
/// between to within [`DEVIATION`] of `size`, and those spans' control
/// rows — or an error saying how far they got.
#[allow(clippy::type_complexity)]
fn refined(
    size: f64,
    roll: impl Fn(usize) -> GeopResult<Rolling>,
) -> GeopResult<(Rolling, Vec<[NurbCurve3D; 4]>)> {
    let smallest = size;
    let mut before = f64::INFINITY;
    // The cap ends it, with an error: `n` doubles each step, so it passes
    // any cap within as many steps as it has bits.
    let refined = refine(FIRST_STATIONS, usize::BITS as usize, |n| -> GeopResult<_> {
        let rolling = roll(n)?;
        let mut curves = Vec::new();
        let mut deviation = 0.0f64;
        let mut worst = String::new();
        for (j, span) in rolling.spans().into_iter().enumerate() {
            let c = span_curves(&rolling, span)?;
            let (d, where_) = span_deviation(&c, &rolling.between(span))?;
            if d > deviation {
                deviation = d;
                // Which faces its balls touch, from where to where: a span
                // between two places the ball rolls onto the next faces.
                let mut touching: Vec<(f64, [FaceId; 2])> = Vec::new();
                for i in span.0..=span.1 {
                    let faces = rolling.stations[i].faces;
                    if touching.last().is_none_or(|(_, f)| *f != faces) {
                        touching.push((rolling.positions[i], faces));
                    }
                }
                worst = format!(
                    "in span {j}, from position {} to {} along the chain (its vertices at whole positions), its {} stations touching faces {touching:?} from the positions given, {where_}",
                    rolling.positions[span.0],
                    rolling.positions[span.1],
                    span.1 - span.0 + 1
                );
            }
            curves.push(c);
        }
        if deviation <= DEVIATION * smallest {
            return Ok(Refine::Done((rolling, curves)));
        }
        // Doubling the stations of a cubic shrinks its deviation sixteen
        // times over; one that does not even halve is not converging, and
        // more stations will not help. A blend that could dip through a
        // face strays infinitely, which says nothing of convergence: the
        // error in its tangent behind that shrinks only as fast as the
        // stations' spacing, and the blend near a curved face's end (a
        // lofted blade's tip, rounded) needed 128 to leave it truly. It is
        // tried up to the cap.
        if n >= MOST_STATIONS || deviation > before / 2.0 {
            return Err(GeopError::new(format!(
                "the blend strays {deviation:e} from the rolling ball with {n} stations per edge ({before:e} with half as many), more than the {:e} it may (a {DEVIATION:e} of its radius): {worst}",
                DEVIATION * smallest
            )));
        }
        before = deviation;
        Ok(Refine::Finer(n * 2))
    })?;
    match refined {
        Refined::Done(done) => Ok(done),
        Refined::Exhausted(n) => Err(GeopError::new(format!(
            "the blend's stations doubled to {n} per edge past every cap"
        ))),
    }
}

/// Checks the ball's section at the end `station` of an open chain lies in
/// the plane of `face`, the third face at the corner — the faces either side
/// stand square to it where the ball touches them — so that the tool, cut
/// off there or run straight on out of the solid, crosses it exactly there.
/// The plane, oriented along `out`, as the cap's `(origin, e1, e2)`.
fn end_plane(
    model: &Model,
    station: &Station,
    face: FaceId,
    out: &Vector3,
) -> GeopResult<[Vector3; 3]> {
    let Some(plane) = model.get_face(face)?.surface.as_plane()? else {
        return Err(GeopError::new(format!("face {face} is not planar")));
    };
    let mut m = plane.normal.normalize()?;
    let along = m.prod_dot(out);
    if along.definitely_less(Scalar::ZERO) {
        m = m.neg();
    } else if !along.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "face {face} could run along the edge at its end"
        )));
    }
    for (k, n) in station.normals.iter().enumerate() {
        if !n.prod_dot(&m).could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "face {} does not stand square to face {face} where the ball touches it ({:?}): a rolling-ball blend only ends at a plane its section lies in",
                station.faces[k], station.contact[k]
            )));
        }
    }
    let c = station.center;
    let origin = c.sub(&m.prod_scalar(c.sub(&plane.point).prod_dot(&m)));
    let towards = station.contact[0].sub(&origin);
    let e1 = towards
        .sub(&m.prod_scalar(towards.prod_dot(&m)))
        .normalize()?;
    Ok([origin, e1, m.prod_cross(&e1)])
}

/// The tool of a chain from its stations and their span curves: widened to
/// enclose the contact curves, oriented, and its ends decided and built.
fn assemble(
    model: &Model,
    chain: Chain,
    bend: Bend,
    rolling: Rolling,
    mut curves: Vec<[NurbCurve3D; 4]>,
    decided_ends: Option<[ChainEnd; 2]>,
    rolled_on: [bool; 2],
) -> GeopResult<Rolled> {
    // One pad for every contact row, so that spans meeting at a station
    // meet in the same control points there.
    let chord = rolling.stations[0].chord;
    let mut pad = [Scalar::ZERO; 3];
    for (span, c) in rolling.spans().into_iter().zip(&curves) {
        for k in [0, 2] {
            let p = c[k].enclosing_pad(&contact_checks(&rolling, span, k))?;
            for i in 0..3 {
                pad[i] = pad[i].max(p[i]).upper();
            }
        }
    }
    for c in &mut curves {
        c[0].widen(&pad);
        c[2].widen(&pad);
    }
    let contacts = curves
        .iter()
        .map(|c| [c[0].clone(), c[2].clone()])
        .collect();

    // Which way round the section runs: the blend's normal — along the
    // chain, crossed with the way from the first contact to the second —
    // has to face the ball, out of the tool.
    let first = &rolling.stations[0];
    let along = rolling.stations[1].center.sub(&first.center);
    let facing = along
        .prod_cross(&first.contact[1].sub(&first.contact[0]))
        .prod_dot(&first.center.sub(&first.corner));
    let swap = if facing.definitely_greater(Scalar::ZERO) {
        false
    } else if facing.definitely_less(Scalar::ZERO) {
        true
    } else {
        return Err(GeopError::new(
            "cannot tell which way the blend faces at the start of the chain",
        ));
    };
    let order = |p: [Vector4; 4]| if swap { [p[2], p[1], p[0], p[3]] } else { p };
    let vertices_of = |s: &Station| {
        if swap {
            [s.contact[1], s.contact[0], s.apex]
        } else {
            [s.contact[0], s.contact[1], s.apex]
        }
    };

    // A span between every two breaks, a station where each starts — and,
    // open, where the last ends.
    let mut stations = Vec::new();
    let mut spans = Vec::new();
    let count = curves.len();
    for (j, ((a, _), c)) in rolling.spans().into_iter().zip(&curves).enumerate() {
        let mut span = span_of(c, (count > 1).then(|| format!("s{j}")));
        span.rows = span.rows.into_iter().map(order).collect();
        stations.push(ToolStation {
            points: span.rows[0],
            vertices: vertices_of(&rolling.stations[a]),
            name: format!("st{j}"),
            cap: None,
        });
        spans.push(span);
    }
    let last_row = *spans.last().expect("spans").rows.last().expect("rows");
    let last_station = rolling.stations.last().expect("stations");
    if !chain.closed {
        stations.push(ToolStation {
            points: last_row,
            vertices: vertices_of(last_station),
            name: format!("st{count}"),
            cap: None,
        });
    }

    if let Some(decided_ends) = decided_ends {
        for (at_end, (vertex, kind, face, out)) in decided_ends
            .into_iter()
            .enumerate()
            .map(|(i, e)| (i == 1, e))
        {
            let station = if at_end { last_station } else { first };
            let ctx = with_context!("the blend's end at vertex {vertex}");
            match &kind {
                End::Corner { .. } => {
                    // Joined to a corner's ball: open, its contacts the
                    // corner's.
                    continue;
                }
                End::Mitre { .. } => {
                    return Err(GeopError::new(
                        "a rolling-ball blend is only mitred between straight edges of constant radius",
                    ))
                    .with_context(ctx);
                }
                _ => {}
            }
            let Some(face) = face else {
                return Err(GeopError::new("the blend's end has no face to end at"))
                    .with_context(ctx);
            };
            let on = rolled_on[usize::from(at_end)];
            let plane = if on {
                own_plane(station, &out)
            } else {
                end_plane(model, station, face, &out)
            }
            .with_context(ctx)?;
            let index = if at_end { stations.len() - 1 } else { 0 };
            let name = if at_end { "end" } else { "start" };
            // The section's control points and vertices in the plane.
            let flat = stations[index].points.map(|p| flatten(&p, &plane));
            let flat_vertices = stations[index].vertices.map(|v| flatten_point(&v, &plane));
            match &kind {
                // Rolled on through the face, the blend is capped where it
                // ends, past the face: in the air where it leaves the
                // solid through it, else to be cut off by it (see
                // [`Rolled::trims`]).
                _ if on => {
                    let station = &mut stations[index];
                    station.points = flat.map(|p| embed(&p, &plane));
                    station.vertices = flat_vertices.map(|p| embed(&p, &plane).head::<3>());
                    station.cap = Some(Cap { plane, flat });
                }
                End::Flush { .. } | End::Wall { .. } => {
                    let station = &mut stations[index];
                    station.points = flat.map(|p| embed(&p, &plane));
                    station.vertices = flat_vertices.map(|p| embed(&p, &plane).head::<3>());
                    station.cap = Some(Cap { plane, flat });
                }
                End::RunOut { .. } => {
                    // As far again as the tool is wide, out of the solid
                    // through the plane, as a straight blend runs out.
                    let width = [station.contact[0], station.contact[1], station.apex]
                        .iter()
                        .map(|p| distance(p, &station.corner))
                        .fold(0.0, f64::max);
                    let [o, e1, e2] = plane;
                    let normal = e1.prod_cross(&e2);
                    let shifted = [
                        o.add(&normal.prod_scalar(Scalar::from_f64(2.0 * width))),
                        e1,
                        e2,
                    ];
                    let far = ToolStation {
                        points: flat.map(|p| embed(&p, &shifted)),
                        vertices: flat_vertices.map(|p| embed(&p, &shifted).head::<3>()),
                        name: format!("{name}_out"),
                        cap: Some(Cap {
                            plane: shifted,
                            flat,
                        }),
                    };
                    let line = |a: [Vector4; 4], b: [Vector4; 4]| ToolSpan {
                        degree: 1,
                        knots: bezier_knots(1),
                        rows: vec![a, b],
                        name: Some(name.to_string()),
                    };
                    if chord {
                        // A chamfer runs straight on out in its last span,
                        // with no station in the face it runs out through:
                        // its section's curves would lie in that face.
                        let span = if at_end {
                            spans.last_mut().expect("spans")
                        } else {
                            &mut spans[0]
                        };
                        run_on(span, far.points, at_end)?;
                        stations[index] = far;
                    } else if at_end {
                        spans.push(line(stations[index].points, far.points));
                        stations.push(far);
                    } else {
                        spans.insert(0, line(far.points, stations[index].points));
                        stations.insert(0, far);
                    }
                }
                End::Mitre { .. } | End::Corner { .. } => {
                    unreachable!("met above")
                }
            }
        }
    }

    // Spans meeting at a station share its control points exactly.
    let count = stations.len();
    for j in 0..spans.len() {
        let next = (j + 1) % count;
        let last = spans[j].rows.len() - 1;
        spans[j].rows[0] = stations[j].points;
        spans[j].rows[last] = stations[next].points;
    }

    // The ball nearest the middle of each link.
    let middles = (0..chain.links.len())
        .map(|link| {
            let middle = link as f64 + 0.5;
            let nearest = (0..rolling.positions.len())
                .min_by(|&a, &b| {
                    let d = |i: usize| (rolling.positions[i] - middle).abs();
                    d(a).total_cmp(&d(b))
                })
                .expect("positions");
            rolling.stations[nearest].contact
        })
        .collect();
    let closed = chain.closed;
    Ok(Rolled {
        trims: [None, None],
        halved_at: None,
        chain,
        bend,
        tool: Tool {
            stations,
            spans,
            closed,
            sides: if swap { ["b", "a"] } else { ["a", "b"] },
            blend: if chord { "chamfer" } else { "fillet" },
        },
        contacts,
        middles,
    })
}

/// Runs `span` on straight to the row `to`, past its end — `at_end` — or
/// its start: the line to it, raised to the span's degree, joined on with
/// the span's knot there repeated as often as its degree, so that the span
/// passes through its old end row and bends there. The old span keeps the
/// first three quarters of its parameters, the run-on the last.
fn run_on(span: &mut ToolSpan, to: [Vector4; 4], at_end: bool) -> GeopResult<()> {
    let p = span.degree;
    let share = Scalar::from_ratio(3, 4)?;
    let line = |from: &[Vector4; 4], i: usize| -> GeopResult<[Vector4; 4]> {
        let f = Scalar::from_ratio(i as i64, p as i64)?;
        let mut row = *from;
        for k in 0..4 {
            row[k] = Vector4::interpolate(&from[k], &to[k], f);
        }
        Ok(row)
    };
    if at_end {
        let from = *span.rows.last().expect("rows");
        let mut knots: Vec<Scalar> = span.knots[..span.knots.len() - 1]
            .iter()
            .map(|k| k.mul(share))
            .collect();
        knots.extend(vec![Scalar::ONE; p + 1]);
        for i in 1..p {
            span.rows.push(line(&from, i)?);
        }
        span.rows.push(to);
        span.knots = knots;
    } else {
        let from = span.rows[0];
        let start = Scalar::ONE.sub(share);
        let mut knots = vec![Scalar::ZERO; p + 1];
        knots.extend(span.knots[1..].iter().map(|k| start.add(k.mul(share))));
        let mut rows = vec![to];
        for i in (1..p).rev() {
            rows.push(line(&from, i)?);
        }
        rows.extend(span.rows.iter().copied());
        span.rows = rows;
        span.knots = knots;
    }
    Ok(())
}

/// Checks the blend meets the faces where they are, along the whole of
/// where it touches them (see [`check_contacts`]): the faces either side of
/// its chain, rolled across at the edges between them, run on past at the
/// chain's ends.
pub(crate) fn check_touches(model: &Model, rolled: &Rolled) -> GeopResult<Vec<Crossing>> {
    let chain = &rolled.chain;
    let ends: Vec<VertexId> = match chain.closed {
        true => Vec::new(),
        false => chain.ends(model)?.map(|(v, _, _)| v).to_vec(),
    };
    check_contacts(
        model,
        &sides(chain),
        &rolled.contacts,
        &rolled.middles,
        &ends,
        &chain.edges(),
    )
}
