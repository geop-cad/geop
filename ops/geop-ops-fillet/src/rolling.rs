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
//! crease is refused. A closed chain's tool breaks at four stations at
//! least, so that no face of it closes onto itself.
//!
//! **Ends.** A closed chain has none. An open one ends as a straight blend
//! does (see [`crate::blend`]), at a corner of exactly three faces, the
//! third a plane — run out past it where a cut leaves the solid through it,
//! flush with it otherwise — but only where the ball's section at the
//! corner lies in that plane, so that the tool crosses it exactly there.
//! Two blends meeting at an inward corner are only mitred between straight
//! edges of constant radius.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::{NurbCurve, NurbCurve3D, true_point_fractions},
    nurb_surface::{NurbSurface3D, clamp},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    EdgeId, FaceId, Model, VertexId,
    contains::face::{PointClassification, face_contains},
};
use geop_ops::Part;
use geop_ops_booleans::remesh::remesh::RemeshParams;

use crate::blend::{Bend, End, bend, faces, tool_end};
use crate::tool::{Cap, Tool, ToolSpan, ToolStation, embed, flatten, flatten_point, wall_surface};

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
/// Newton iterations placing the ball at a station.
const NEWTON_ITERATIONS: usize = 30;
/// Newton iterations of each foot-point projection.
const PROJECT_ITERATIONS: usize = 20;
/// Seed of the ray casting behind `face_contains`, whose answer does not
/// depend on it.
const SEED: u64 = 0xB1E2_D000_0000_0002;

/// One edge of a chain, as the chain runs along it: along the edge's own
/// direction, `forward`, or against it; with the faces on its left and
/// right as it runs (seen from outside the solid).
#[derive(Clone, Debug)]
pub(crate) struct Link {
    pub edge: EdgeId,
    pub forward: bool,
    pub faces: [FaceId; 2],
}

impl Link {
    fn new<S: Scalar>(model: &Model<S>, edge: EdgeId, forward: bool) -> GeopResult<Self> {
        let [l, r] = faces(model, edge)?;
        Ok(Link {
            edge,
            forward,
            faces: if forward { [l, r] } else { [r, l] },
        })
    }

    fn reversed(&self) -> Self {
        Link {
            edge: self.edge,
            forward: !self.forward,
            faces: [self.faces[1], self.faces[0]],
        }
    }

    /// The vertex the chain enters the edge at, and the one it leaves at.
    fn vertices<S: Scalar>(&self, model: &Model<S>) -> GeopResult<[VertexId; 2]> {
        let e = model.get_edge(self.edge)?;
        Ok(if self.forward {
            [e.start_vertex, e.end_vertex]
        } else {
            [e.end_vertex, e.start_vertex]
        })
    }

    /// The edge's point `fraction` of the way along it as the chain runs —
    /// of its parameter — and the unit tangent there, the way it runs.
    fn at<S: Scalar>(&self, model: &Model<S>, fraction: S) -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let curve = &model.get_edge(self.edge)?.curve;
        let (t0, t1) = curve.domain();
        let f = if self.forward {
            fraction
        } else {
            S::ONE.sub(fraction)
        };
        let t = S::interpolate(t0, t1, f);
        let tangent = curve.tangent(t)?.normalize()?;
        Ok((
            curve.evaluate(t)?,
            if self.forward { tangent } else { tangent.neg() },
        ))
    }
}

/// The edges blended as one: a tangent chain (see the module docs), in the
/// order it runs, around to the first again if `closed`.
#[derive(Clone, Debug)]
pub(crate) struct Chain {
    pub links: Vec<Link>,
    pub closed: bool,
}

impl Chain {
    /// The chain's vertices in order: where each link starts, and — for an
    /// open chain — where the last one ends.
    pub fn vertices<S: Scalar>(&self, model: &Model<S>) -> GeopResult<Vec<VertexId>> {
        let mut vertices = Vec::new();
        for link in &self.links {
            vertices.push(link.vertices(model)?[0]);
        }
        if !self.closed {
            let last = self.links.last().expect("a chain has links");
            vertices.push(last.vertices(model)?[1]);
        }
        Ok(vertices)
    }

    pub fn edges(&self) -> Vec<EdgeId> {
        self.links.iter().map(|l| l.edge).collect()
    }
}

/// Whether the unit vectors `a` and `b` could point the same way.
fn same_direction<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> bool {
    a.prod_cross(b).norm_sq().could_be_equal(S::ZERO) && a.prod_dot(b).definitely_greater(S::ZERO)
}

/// The `(u, v)` of `point` on `surface`, sharp — a seed — found globally
/// where the point lies on the surface, else from the middle of its domain.
pub(crate) fn seed_on<S: Scalar>(
    surface: &NurbSurface3D<S>,
    point: &Vector3<S>,
) -> GeopResult<(S, S)> {
    let params = RemeshParams::<S>::default();
    let found = surface_could_contain(
        surface,
        point,
        params.max_nodes,
        params.min_subdivision_size,
    )?;
    let (u, v) = match found {
        Some(uv) => uv,
        None => {
            // Off the surface: from whichever of a grid over its domain
            // lies nearest, so that Newton finds the foot point near the
            // point — on a cylinder, not the one across its axis.
            let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
            let mut best = (f64::INFINITY, (u0, v0));
            for i in 0..=4 {
                for j in 0..=4 {
                    let u = S::interpolate(u0, u1, S::from_f64(i as f64 / 4.0)).sharpen();
                    let v = S::interpolate(v0, v1, S::from_f64(j as f64 / 4.0)).sharpen();
                    let d = distance(&surface.evaluate(u, v)?, point);
                    if d < best.0 {
                        best = (d, (u, v));
                    }
                }
            }
            best.1
        }
    };
    let (u, v) = foot(surface, point, (u.sharpen(), v.sharpen()))?;
    Ok(sharp_in(surface, (u, v)))
}

/// The foot point of `target` on `surface`, Newton from `seed`: its
/// enclosure, within the surface's domain — the foot point lies in it, so
/// that holds too, and the surface is only evaluated there.
pub(crate) fn foot<S: Scalar>(
    surface: &NurbSurface3D<S>,
    target: &Vector3<S>,
    seed: (S, S),
) -> GeopResult<(S, S)> {
    let (u, v) = surface.project(*target, seed.0, seed.1, PROJECT_ITERATIONS)?;
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    Ok((u.intersect(u0.union(u1)), v.intersect(v0.union(v1))))
}

/// `(u, v)` sharpened, within the domain of `surface`: a seed.
pub(crate) fn sharp_in<S: Scalar>(surface: &NurbSurface3D<S>, (u, v): (S, S)) -> (S, S) {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    (clamp(u.sharpen(), u0, u1), clamp(v.sharpen(), v0, v1))
}

/// The unit outward normal of `face` at its point `point`.
pub(crate) fn normal_at<S: Scalar>(
    model: &Model<S>,
    face: FaceId,
    point: &Vector3<S>,
) -> GeopResult<Vector3<S>> {
    let surface = &model.get_face(face)?.surface;
    let (u, v) = seed_on(surface, point)?;
    surface.normal(u, v)?.normalize()
}

/// The link the chain goes on along from `link`, past the vertex it leaves
/// at: the one other edge there it runs on into tangentially, between faces
/// tangent to its own — or none. An error if more than one could be.
///
/// Tangency is asked of the faces, whose normals at the vertex the
/// surfaces state exactly: where both faces either side go on tangentially,
/// so does the edge between them, which runs along both tangent planes.
/// The edges' own tangents only tell which way the next one runs on — an
/// edge traced as an intersection only encloses its points, not its
/// direction.
fn next_link<S: Scalar>(model: &Model<S>, link: &Link) -> GeopResult<Option<Link>> {
    let vertex = link.vertices(model)?[1];
    let at = model.get_vertex(vertex)?.point;
    let (_, arriving) = link.at(model, S::ONE)?;
    let normals = [
        normal_at(model, link.faces[0], &at)?,
        normal_at(model, link.faces[1], &at)?,
    ];
    let mut found = Vec::new();
    for (&id, e) in &model.edges {
        if id == link.edge {
            continue;
        }
        for forward in [true, false] {
            let leaves = if forward {
                e.start_vertex
            } else {
                e.end_vertex
            };
            if leaves != vertex {
                continue;
            }
            let candidate = Link::new(model, id, forward)?;
            let (_, leaving) = candidate.at(model, S::ZERO)?;
            if !arriving.prod_dot(&leaving).definitely_greater(S::ZERO) {
                continue;
            }
            let theirs = [
                normal_at(model, candidate.faces[0], &at)?,
                normal_at(model, candidate.faces[1], &at)?,
            ];
            if (0..2).all(|k| same_direction(&normals[k], &theirs[k])) {
                found.push(candidate);
            }
        }
    }
    match found.len() {
        0 => Ok(None),
        1 => Ok(found.pop()),
        n => Err(GeopError::new(format!(
            "at vertex {vertex}, the edge runs on tangentially into {n} edges: no one chain to blend"
        ))),
    }
}

/// The tangent chain of `edge` (see the module docs), running the way the
/// edge does.
pub(crate) fn tangent_chain<S: Scalar>(model: &Model<S>, edge: EdgeId) -> GeopResult<Chain> {
    let walk = |first: Link| -> GeopResult<(Vec<Link>, bool)> {
        let mut links = vec![first];
        loop {
            let last = links.last().expect("never empty");
            let Some(next) = next_link(model, last)? else {
                return Ok((links, false));
            };
            if next.edge == edge {
                return Ok((links, true));
            }
            if links.iter().any(|l| l.edge == next.edge) {
                return Err(GeopError::new(format!(
                    "the tangent chain of the edge runs round a loop not through it, at edge {}",
                    next.edge
                )));
            }
            links.push(next);
        }
    };
    let (ahead, closed) = walk(Link::new(model, edge, true)?)?;
    if closed {
        return Ok(Chain {
            links: ahead,
            closed,
        });
    }
    let (behind, _) = walk(Link::new(model, edge, false)?)?;
    let mut links: Vec<Link> = behind[1..].iter().rev().map(Link::reversed).collect();
    links.extend(ahead);
    Ok(Chain {
        links,
        closed: false,
    })
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

/// Samples per edge its length is measured by, to lay a varying radius
/// out along a chain.
const LENGTH_SAMPLES: usize = 32;

/// A fillet's radius along one chain (see [`Radii`]): at every vertex of
/// the chain, linear in the length along it between the vertices given a
/// radius — lengths measured in plain numbers, since the law is the
/// designer's choice and this is how it is laid out — and along each edge
/// linear in its parameter, so that the radius is as smooth along an edge
/// as the edge is and changes its rate only at the chain's vertices.
struct RadiusLaw {
    /// Per vertex of the chain, in order — round a closed one to the first
    /// again — its radius.
    at_vertices: Vec<f64>,
}

impl RadiusLaw {
    fn new<S: Scalar>(part: &Part<S>, chain: &Chain, radii: &Radii) -> GeopResult<Self> {
        let model = part.topology();
        // The length along the chain to each of its vertices.
        let mut lengths = vec![0.0];
        for link in &chain.links {
            let mut last = link.at(model, S::ZERO)?.0;
            let mut length = *lengths.last().expect("a length");
            for i in 1..=LENGTH_SAMPLES {
                let f = S::from_ratio(i as i64, LENGTH_SAMPLES as i64)?;
                let p = link.at(model, f)?.0;
                length += p.sub(&last).norm().midpoint().to_f64();
                last = p;
            }
            lengths.push(length);
        }
        let mut vertices = chain.vertices(model)?;
        if chain.closed {
            if radii.end_radius.is_some() {
                return Err(GeopError::new(
                    "the edge's tangent chain is closed, so it has no end for an end radius: give radii at its vertices instead",
                ));
            }
            vertices.push(vertices[0]);
        }
        let given = |v: VertexId| {
            part.name_of(v)
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
        })
    }

    /// The radius `fraction` of the way along link `link`, sharp: the law is
    /// a choice, and this is it.
    fn radius<S: Scalar>(&self, link: usize, fraction: f64) -> S {
        let (r0, r1) = (self.at_vertices[link], self.at_vertices[link + 1]);
        S::from_f64(r0 + fraction * (r1 - r0))
    }

    /// Whether the radius changes along the chain.
    fn varies(&self) -> bool {
        self.at_vertices.windows(2).any(|w| w[0] != w[1])
    }

    /// The smallest radius along the chain.
    fn smallest(&self) -> f64 {
        self.at_vertices
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }
}

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
struct Station<S: Scalar> {
    center: Vector3<S>,
    contact: [Vector3<S>; 2],
    faces: [FaceId; 2],
    normals: [Vector3<S>; 2],
    uv: [(S, S); 2],
    corner: Vector3<S>,
    weight: S,
    apex: Vector3<S>,
    chord: bool,
}

/// The ball of signed radius `sr` — `r` along both normals, negative
/// behind them — touching `surfaces` in the plane through `p` square to the
/// unit `tangent`, Newton's method from the foot points `seeds` (see the
/// module docs): its center and the foot points' `(u, v)`.
///
/// Every iterate is sharpened, the last too: the center is a construction
/// aid, not an answer anything is compared against — the contact points
/// are evaluated on the surfaces, so they lie on them however exactly the
/// center was found.
fn roll<S: Scalar>(
    surfaces: [&NurbSurface3D<S>; 2],
    seeds: [(S, S); 2],
    p: &Vector3<S>,
    tangent: &Vector3<S>,
    sr: S,
) -> GeopResult<(Vector3<S>, [(S, S); 2])> {
    let normal = |k: usize, (u, v): (S, S)| surfaces[k].normal(u, v)?.normalize();
    let (n0, n1) = (normal(0, seeds[0])?, normal(1, seeds[1])?);
    // Exact for two planes through `p`: `r` from both.
    let spread = S::ONE.add(n0.prod_dot(&n1));
    let mut center = p.add(&n0.add(&n1).prod_scalar(sr.div(spread)?)).sharpen();
    let mut uv = seeds;
    for _ in 0..NEWTON_ITERATIONS {
        let mut rows = Vec::with_capacity(3);
        let mut residual = Vec::with_capacity(3);
        for k in 0..2 {
            uv[k] = sharp_in(surfaces[k], foot(surfaces[k], &center, uv[k])?);
            let at = surfaces[k].evaluate(uv[k].0, uv[k].1)?;
            let n = normal(k, uv[k])?;
            residual.push(center.sub(&at).prod_dot(&n).sub(sr));
            rows.push(n);
        }
        rows.push(*tangent);
        residual.push(center.sub(p).prod_dot(tangent));
        let jacobian =
            Matrix::from_rows([rows[0].to_array(), rows[1].to_array(), rows[2].to_array()]);
        let step = solve_linear_system(
            &jacobian,
            &Vector3::from_array([residual[0], residual[1], residual[2]]),
        )?;
        let next = center.sub(&step).sharpen();
        let same =
            (0..3).all(|k| next[k].is_subset_of(center[k]) && center[k].is_subset_of(next[k]));
        center = next;
        if same {
            break;
        }
    }
    Ok((center, uv))
}

/// How far, as a plain number, the foot point `foot` of `center` on a
/// surface with unit normal `n` there is from being one: the part of
/// `center - foot` along the surface. Zero, but for rounding, for a true
/// foot point; a foot point clamped to the edge of the surface's domain —
/// where the ball would touch another face — leaves the rest.
fn off_foot<S: Scalar>(center: &Vector3<S>, foot: &Vector3<S>, n: &Vector3<S>) -> f64 {
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
        for k in 0..2 {
            if !sides[k].contains(&link.faces[k]) {
                sides[k].push(link.faces[k]);
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
fn place<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    link: usize,
    fraction: S,
    sr: S,
    previous: Option<&Station<S>>,
) -> GeopResult<Station<S>> {
    let own = &chain.links[link];
    let (p, along) = own.at(model, fraction)?;
    // The edge runs along both faces' tangent planes: their normals there
    // state its direction exactly, where the edge's curve — traced as an
    // intersection, say — only encloses its points. At a vertex where the
    // faces change, that is what puts the ball's section through it.
    let mut normals = [Vector3::zero(); 2];
    for k in 0..2 {
        let surface = &model.get_face(own.faces[k])?.surface;
        let seed = match previous {
            Some(prev) if prev.faces[k] == own.faces[k] => prev.uv[k],
            _ => seed_on(surface, &p)?,
        };
        let (u, v) = foot(surface, &p, seed)?;
        normals[k] = surface.normal(u, v)?;
    }
    let across = normals[0].prod_cross(&normals[1]);
    let tangent = if across.prod_dot(&along).definitely_less(S::ZERO) {
        across.neg()
    } else {
        across
    }
    .normalize()
    .map_err(|_| {
        GeopError::new(format!(
            "faces {} and {} could meet tangentially at {p:?} on edge {}: there is no corner to blend there (normals {:?} and {:?})",
            own.faces[0], own.faces[1], own.edge, normals[0], normals[1]
        ))
    })?;
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
    let mut best: Option<(f64, Station<S>)> = None;
    let mut tried: Vec<([FaceId; 2], String)> = Vec::new();
    for pair in pairs {
        let attempt = || -> GeopResult<(f64, Station<S>)> {
            let surfaces = [
                &model.get_face(pair[0])?.surface,
                &model.get_face(pair[1])?.surface,
            ];
            let mut seeds = [(S::ZERO, S::ZERO); 2];
            for k in 0..2 {
                seeds[k] = match previous {
                    Some(prev) if prev.faces[k] == pair[k] => prev.uv[k],
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

/// The station of the ball centered at `center` touching `faces` at
/// `contact`, where their outward unit normals are `normals`: its section's
/// middle control point, weight and apex (see the module docs) — free
/// choices of the blend's shape, sharp. With `tilt` the arc leaves the
/// faces turned by [`TILT`] towards the ball; without, it is the ball's
/// great arc exactly — where the blend meets a corner's ball (see
/// [`crate::corner`]), whose piece shares the arc.
fn section<S: Scalar>(
    center: Vector3<S>,
    contact: [Vector3<S>; 2],
    normals: [Vector3<S>; 2],
    faces: [FaceId; 2],
    uv: [(S, S); 2],
    tilt: bool,
) -> GeopResult<Station<S>> {
    let [ta, tb] = contact;
    let m = normals[0].prod_cross(&normals[1]);
    // The arc's tangent at each contact: along the face, in the section's
    // plane, turned by `TILT` towards the ball — and the plane through the
    // contact holding it and the section's normal.
    let mut tilted = [Vector3::zero(); 2];
    for k in 0..2 {
        let (at, other) = (contact[k], contact[1 - k]);
        let mut along = m.prod_cross(&normals[k]).normalize()?;
        if along.prod_dot(&other.sub(&at)).definitely_less(S::ZERO) {
            along = along.neg();
        }
        if tilt {
            let toward = center.sub(&at).normalize()?;
            along = along.add(&toward.prod_scalar(S::from_f64(TILT)));
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
    if !weight.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "the blend's section turns by half a turn or more there: weight {weight:?}"
        )));
    }
    let apex = corner.prod_scalar(S::TWO).sub(&center).sharpen();
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
    })
}

/// The section's control points, homogeneous: the contact on the left,
/// the middle (weighted — a chord's of weight one, taken as it is), the
/// contact on the right, the apex.
fn section_points<S: Scalar>(station: &Station<S>) -> [Vector4<S>; 4] {
    let h = |p: &Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
    let w = station.weight;
    let e = station.corner;
    [
        h(&station.contact[0].sharpen()),
        if station.chord {
            h(&e)
        } else {
            Vector4::from_array([e[0].mul(w), e[1].mul(w), e[2].mul(w), w])
        },
        h(&station.contact[1].sharpen()),
        h(&station.apex),
    ]
}

/// Where a chamfer's section is taken at the point `fraction` of the way
/// along link `link` of `chain`: the edge's point there — one point of its
/// enclosure, which is a free choice, as is where along the edge the
/// section is — and the unit tangent, stated by the faces' normals there as
/// in [`place`]; and the foot points' `(u, v)` on the link's faces, from the
/// section before, `previous`.
fn chamfer_frame<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    link: usize,
    fraction: S,
    previous: Option<&Station<S>>,
) -> GeopResult<(Vector3<S>, Vector3<S>, [(S, S); 2])> {
    let own = &chain.links[link];
    let (p, along) = own.at(model, fraction)?;
    let p = p.sharpen();
    let mut normals = [Vector3::zero(); 2];
    let mut feet = [(S::ZERO, S::ZERO); 2];
    for k in 0..2 {
        let surface = &model.get_face(own.faces[k])?.surface;
        let seed = match previous {
            Some(prev) if prev.faces[k] == own.faces[k] => prev.uv[k],
            _ => seed_on(surface, &p)?,
        };
        let (u, v) = foot(surface, &p, seed)?;
        normals[k] = surface.normal(u, v)?.normalize()?;
        feet[k] = sharp_in(surface, (u, v));
    }
    let across = normals[0].prod_cross(&normals[1]);
    let tangent = if across.prod_dot(&along).definitely_less(S::ZERO) {
        across.neg()
    } else {
        across
    }
    .normalize()
    .map_err(|_| {
        GeopError::new(format!(
            "faces {} and {} could meet tangentially at {p:?} on edge {}: there is no corner to blend there",
            own.faces[0], own.faces[1], own.edge
        ))
    })?;
    Ok((p, tangent, feet))
}

/// The point of `surface` in the plane through `p` square to the unit
/// `tangent`, `d` from `p`, Newton's method from `seed`: its `(u, v)`.
/// Every iterate is sharpened — where a chamfer meets a face is its shape,
/// chosen — and the answer checked against both conditions, to within how
/// far the chamfer may stray from them anywhere.
fn chamfer_point<S: Scalar>(
    surface: &NurbSurface3D<S>,
    p: &Vector3<S>,
    tangent: &Vector3<S>,
    d: f64,
    seed: (S, S),
) -> GeopResult<(S, S)> {
    let dd = S::from_f64(d);
    let mut at = seed;
    for _ in 0..PROJECT_ITERATIONS {
        let x = surface.evaluate(at.0, at.1)?;
        let (su, sv) = surface.derivatives(at.0, at.1)?;
        let off = x.sub(p);
        let residual =
            Vector2::from_array([off.prod_dot(tangent), off.prod_dot(&off).sub(dd.mul(dd))]);
        let jacobian = Matrix::from_rows([
            [su.prod_dot(tangent), sv.prod_dot(tangent)],
            [S::TWO.mul(off.prod_dot(&su)), S::TWO.mul(off.prod_dot(&sv))],
        ]);
        let step = solve_linear_system(&jacobian, &residual)?;
        let next = sharp_in(surface, (at.0.sub(step[0]), at.1.sub(step[1])));
        let same = next.0.is_subset_of(at.0)
            && at.0.is_subset_of(next.0)
            && next.1.is_subset_of(at.1)
            && at.1.is_subset_of(next.1);
        at = next;
        if same {
            break;
        }
    }
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
fn seed_into<S: Scalar>(
    surface: &NurbSurface3D<S>,
    p: &Vector3<S>,
    tangent: &Vector3<S>,
    foot_of_p: (S, S),
    k: usize,
    d: f64,
) -> GeopResult<(S, S)> {
    let n = surface.normal(foot_of_p.0, foot_of_p.1)?.normalize()?;
    let into = if k == 0 {
        n.prod_cross(tangent)
    } else {
        tangent.prod_cross(&n)
    }
    .normalize()?;
    let guess = p.add(&into.prod_scalar(S::from_f64(d)));
    Ok(sharp_in(surface, foot(surface, &guess, foot_of_p)?))
}

/// The chamfer's section from the edge's point `p` — the station as
/// [`Station`] holds a chord — meeting `faces` at `contact`, where their
/// outward unit normals are `normals` and their `(u, v)` are `uv`.
fn chord<S: Scalar>(
    p: &Vector3<S>,
    contact: [Vector3<S>; 2],
    normals: [Vector3<S>; 2],
    faces: [FaceId; 2],
    uv: [(S, S); 2],
) -> GeopResult<Station<S>> {
    let middle = Vector3::interpolate(&contact[0], &contact[1], S::ONE.div(S::TWO)?).sharpen();
    Ok(Station {
        center: middle.prod_scalar(S::TWO).sub(p).sharpen(),
        contact,
        faces,
        normals,
        uv,
        corner: middle,
        weight: S::ONE,
        apex: p.prod_scalar(S::TWO).sub(&middle).sharpen(),
        chord: true,
    })
}

/// The edge's point a chord's section `station` was taken from.
fn chord_edge_point<S: Scalar>(station: &Station<S>) -> GeopResult<Vector3<S>> {
    Ok(Vector3::interpolate(
        &station.corner,
        &station.apex,
        S::ONE.div(S::TWO)?,
    ))
}

/// The chamfer's section at the point `fraction` of the way along link
/// `link` of `chain` (see [`plan_chamfer`]), `distances` into the faces on
/// its left and right, from the points of the section before, `previous`.
fn chamfer_station<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    link: usize,
    fraction: S,
    distances: [f64; 2],
    previous: Option<&Station<S>>,
) -> GeopResult<Station<S>> {
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
fn chamfer_crossing<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    k: usize,
    before: &Station<S>,
    after: &Station<S>,
    (from, to): (f64, f64),
    distances: [f64; 2],
) -> GeopResult<(f64, Station<S>)> {
    let (face, onto) = (before.faces[k], after.faces[k]);
    let mut best: Option<(f64, EdgeId)> = None;
    for edge in edges_between(model, face, onto)? {
        let curve = &model.get_edge(edge)?.curve;
        let (lo, hi) = curve.domain();
        let middle = S::interpolate(lo, hi, S::from_f64(0.5)).sharpen();
        let mut near = 0.0;
        for x in [&before.contact[k], &after.contact[k]] {
            let t = nearest_parameter(curve, x, middle)?;
            near += distance(&curve.evaluate(t)?, x);
        }
        if best.is_none_or(|(b, _)| near < b) {
            best = Some((near, edge));
        }
    }
    let Some((_, edge)) = best else {
        return Err(GeopError::new(format!(
            "the chamfer moves from face {face} onto face {onto}, which meet in no edge"
        )));
    };
    let curve = &model.get_edge(edge)?.curve;
    let (lo, hi) = curve.domain();
    let seed = nearest_parameter(
        curve,
        &before.contact[k],
        S::interpolate(lo, hi, S::from_f64(0.5)).sharpen(),
    )?;
    // At position `c`: the edge's point the chamfer's distance from the
    // chain's — Newton's method along it — and how far ahead of the plane
    // square to the chain there it is. The edge runs across the chain's
    // side, so it meets that sphere once, near where the chamfer meets it.
    let d2 = S::from_f64(distances[k] * distances[k]);
    let at = |c: f64, seed: S| -> GeopResult<(f64, S, Vector3<S>, Vector3<S>, [(S, S); 2])> {
        let (link, x) = locate(chain, c);
        let (p, tangent, feet) = chamfer_frame(model, chain, link, S::from_f64(x), Some(before))?;
        let mut t = seed;
        for _ in 0..PROJECT_ITERATIONS {
            let off = curve.evaluate(t)?.sub(&p);
            let rate = S::TWO.mul(off.prod_dot(&curve.tangent(t)?));
            t = clamp(
                t.sub(off.prod_dot(&off).sub(d2).div(rate)?).sharpen(),
                lo,
                hi,
            );
        }
        let ahead = curve.evaluate(t)?.sub(&p).prod_dot(&tangent);
        Ok((ahead.midpoint().to_f64(), t, p, tangent, feet))
    };
    let (mut lo_c, mut hi_c) = (from, to);
    let (g_lo, mut t) = {
        let (g, t, ..) = at(lo_c, seed)?;
        (g, t)
    };
    let (g_hi, _, ..) = at(hi_c, t)?;
    if g_lo.signum() == g_hi.signum() {
        return Err(GeopError::new(format!(
            "cannot find where the chamfer moves from face {face} onto face {onto} across edge {edge}: its point is {g_lo:e} and {g_hi:e} off the edge either side"
        )));
    }
    // Bisection: where along the chain is a free choice up to where the
    // chamfer's point lies on the edge, and the halves only close in on it.
    for _ in 0..60 {
        let mid = 0.5 * (lo_c + hi_c);
        let (g, tm, ..) = at(mid, t)?;
        t = tm;
        if g.signum() == g_lo.signum() {
            lo_c = mid;
        } else {
            hi_c = mid;
        }
    }
    let c = 0.5 * (lo_c + hi_c);
    let (_, t, p, tangent, feet) = at(c, t)?;
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
    uv[k] = sharp_in(surface, foot(surface, &contact[k], before.uv[k])?);
    normals[k] = surface.normal(uv[k].0, uv[k].1)?.normalize()?;
    let o = 1 - k;
    let other = &model.get_face(faces[o])?.surface;
    let seed = if before.faces[o] == faces[o] {
        before.uv[o]
    } else {
        seed_into(other, &p, &tangent, feet[o], o, distances[o])?
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
/// of their own, where the blend changes its rate.
struct Placing<'a, S: Scalar> {
    place: Box<dyn Fn(f64, Option<&Station<S>>) -> GeopResult<Station<S>> + 'a>,
    #[allow(clippy::type_complexity)]
    cross: Box<
        dyn Fn(usize, &Station<S>, &Station<S>, (f64, f64)) -> GeopResult<(f64, Station<S>)> + 'a,
    >,
    #[allow(clippy::type_complexity)]
    merge: Box<dyn Fn(&Station<S>, &Station<S>) -> GeopResult<Option<Station<S>>> + 'a>,
    kinks: Vec<f64>,
}

/// The balls rolled along a chain: at the stations, by their positions
/// along it — link index plus fraction of its edge — and between each two
/// where `true_point_fractions` says, by the fraction of the way; and where
/// the tool's spans start and end, as indices of stations: at both ends,
/// and wherever the ball rolls from one face onto the next.
struct Rolling<S: Scalar> {
    stations: Vec<Station<S>>,
    positions: Vec<f64>,
    breaks: Vec<usize>,
    between: Vec<Vec<(f64, Station<S>)>>,
}

impl<S: Scalar> Rolling<S> {
    /// The spans, as their first and last station.
    fn spans(&self) -> Vec<(usize, usize)> {
        self.breaks.windows(2).map(|w| (w[0], w[1])).collect()
    }

    /// The parameter along the span from station `a` to `b` of the
    /// position `c`: evenly by position, exactly 0 and 1 at its ends.
    fn param(&self, (a, b): (usize, usize), c: f64) -> S {
        let (ca, cb) = (self.positions[a], self.positions[b]);
        if c == ca {
            S::ZERO
        } else if c == cb {
            S::ONE
        } else {
            S::from_f64((c - ca) / (cb - ca))
        }
    }

    /// The stations' parameters along `span`.
    fn params(&self, span: (usize, usize)) -> Vec<S> {
        (span.0..=span.1)
            .map(|i| self.param(span, self.positions[i]))
            .collect()
    }

    /// The balls between the stations of `span`, by their parameters.
    fn between(&self, span: (usize, usize)) -> Vec<Vec<(S, Station<S>)>> {
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
    let count = n * chain.links.len();
    let inner = (0..count).map(|k| (k as f64 + 0.5) / n as f64);
    if chain.closed {
        inner.collect()
    } else {
        // Clear of the ends by a quarter step: no span next to one shorter.
        let clear = 0.25 / n as f64;
        std::iter::once(from)
            .chain(inner.filter(|&c| c - from > clear && to - c > clear))
            .chain(std::iter::once(to))
            .collect()
    }
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
fn edges_between<S: Scalar>(model: &Model<S>, a: FaceId, b: FaceId) -> GeopResult<Vec<EdgeId>> {
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

/// The parameter of `curve` nearest `point`, Newton from `seed` — sharp:
/// where to look, a free choice.
fn nearest_parameter<S: Scalar>(
    curve: &NurbCurve3D<S>,
    point: &Vector3<S>,
    seed: S,
) -> GeopResult<S> {
    let (lo, hi) = curve.domain();
    let mut t = seed;
    for _ in 0..PROJECT_ITERATIONS {
        let d = curve.evaluate(t)?.sub(point);
        let tangent = curve.tangent(t)?;
        let step = d.prod_dot(&tangent).div(tangent.prod_dot(&tangent))?;
        t = clamp(t.sub(step).sharpen(), lo, hi);
    }
    Ok(t)
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
fn crossing<S: Scalar>(
    model: &Model<S>,
    k: usize,
    before: &Station<S>,
    after: &Station<S>,
    radius: S,
) -> GeopResult<Station<S>> {
    let (from, onto) = (before.faces[k], after.faces[k]);
    let mut best: Option<(f64, EdgeId, S, S)> = None;
    for edge in edges_between(model, from, onto)? {
        let curve = &model.get_edge(edge)?.curve;
        let (lo, hi) = curve.domain();
        let middle = S::interpolate(lo, hi, S::from_f64(0.5)).sharpen();
        let t0 = nearest_parameter(curve, &before.contact[k], middle)?;
        let t1 = nearest_parameter(curve, &after.contact[k], middle)?;
        let near = distance(&curve.evaluate(t0)?, &before.contact[k])
            + distance(&curve.evaluate(t1)?, &after.contact[k]);
        if best.as_ref().is_none_or(|(b, ..)| near < *b) {
            best = Some((near, edge, t0, t1));
        }
    }
    let Some((_, edge, t0, t1)) = best else {
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
    let ball = |t: f64| -> GeopResult<(
        f64,
        Vector3<S>,
        [Vector3<S>; 2],
        [Vector3<S>; 2],
        [(S, S); 2],
        S,
    )> {
        let sr = radius;
        let ts = S::from_f64(t);
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
        Ok((gap.midpoint().to_f64(), center, contact, normals, uv, sr))
    };
    // A bracket: from between where the contacts lie along the edge,
    // widened until the ball goes from one side of touching to the other —
    // along an edge running across the chain, both contacts lie at about
    // the same place on it, so they bracket nothing.
    let (domain_lo, domain_hi) = curve.domain();
    let (domain_lo, domain_hi) = (domain_lo.midpoint().to_f64(), domain_hi.midpoint().to_f64());
    let speed = curve.tangent(t0)?.norm().midpoint().to_f64();
    let middle = 0.5 * (f0 + f1);
    let mut reach = (0.5 * (f1 - f0).abs()).max(0.01 * radius.abs().midpoint().to_f64() / speed);
    let (mut lo, mut hi, mut g_lo);
    loop {
        lo = (middle - reach).max(domain_lo);
        hi = (middle + reach).min(domain_hi);
        g_lo = ball(lo)?.0;
        let g_hi = ball(hi)?.0;
        if g_lo.signum() != g_hi.signum() {
            break;
        }
        if lo <= domain_lo && hi >= domain_hi {
            return Err(GeopError::new(format!(
                "cannot find where the ball rolls from face {from} onto face {onto} across edge {edge}: it is {g_lo:e} and {g_hi:e} off touching face {} at either end of it",
                before.faces[o]
            )));
        }
        reach *= 2.0;
    }
    // Bisection: where along the edge is a free choice up to where the
    // ball touches both, and the halves only ever close in on that.
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if mid <= lo.min(hi) || mid >= lo.max(hi) {
            break;
        }
        let (g, ..) = ball(mid)?;
        if g.signum() == g_lo.signum() {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = 0.5 * (lo + hi);
    let (_, center, contact, normals, uv, _) = ball(t)?;
    // Tangent faces either side of the edge, where the ball touches it.
    let onto_surface = &model.get_face(onto)?.surface;
    let (u2, v2) = foot(onto_surface, &contact[k], after.uv[k])?;
    let n2 = onto_surface.normal(u2, v2)?.normalize()?;
    if !normals[k].prod_cross(&n2).norm_sq().could_be_equal(S::ZERO) {
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
fn roll_chain<S: Scalar>(
    chain: &Chain,
    n: usize,
    corners: &[Option<(f64, Station<S>)>; 2],
    placing: &Placing<S>,
) -> GeopResult<Rolling<S>> {
    let links = chain.links.len() as f64;
    let place_at = &placing.place;

    // The stations, evenly along the chain — between the corners' balls
    // where it ends at one, which are its first and last.
    let range = (
        corners[0].as_ref().map_or(0.0, |(c, _)| *c),
        corners[1].as_ref().map_or(links, |(c, _)| *c),
    );
    let positions = station_positions(chain, n, range);
    let mut regular: Vec<Station<S>> = Vec::with_capacity(positions.len());
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
    let mut crossings: Vec<(f64, Station<S>)> = Vec::new();
    for i in 0..intervals {
        let j = (i + 1) % m;
        let (ca, mut cb) = (positions[i], positions[j]);
        if j == 0 {
            cb += links;
        }
        let (a, b) = (&regular[i], &regular[j]);
        let mut found: Vec<(f64, Station<S>)> = Vec::new();
        for k in 0..2 {
            if a.faces[k] != b.faces[k] {
                let ctx = with_context!("moving from face {} onto face {}", a.faces[k], b.faces[k]);
                found.push((placing.cross)(k, a, b, (ca, cb)).with_context(ctx)?);
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
    // Where the blend changes its rate, a station too, unless a side moves
    // onto the next face right there.
    for &c in &placing.kinks {
        let near = |x: f64| {
            let d = (x - c).abs();
            let d = if chain.closed { d.min(links - d) } else { d };
            d < 0.25 / n as f64
        };
        if crossings.iter().any(|(x, _)| near(*x)) {
            continue;
        }
        let before = positions
            .iter()
            .rposition(|&x| x < c)
            .unwrap_or(positions.len() - 1);
        crossings.push((c, place_at(c, Some(&regular[before]))?));
    }
    let mut all: Vec<(f64, Station<S>, bool)> = positions
        .iter()
        .zip(regular)
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|((&c, s), _)| (c, s, false))
        .collect();
    all.extend(crossings.into_iter().map(|(c, s)| (c, s, true)));
    all.sort_by(|a, b| a.0.total_cmp(&b.0));

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

/// How a fillet's balls are placed along `chain` of `part` (see
/// [`Placing`]), their radius as `law` says, on the side `side` of the
/// faces: rolled ([`place`]), rolling from face to face ([`crossing`]),
/// and where a varying radius changes its rate — at the chain's vertices —
/// a ball of its own.
fn fillet_placing<'a, S: Scalar>(
    part: &'a Part<S>,
    chain: &'a Chain,
    law: &'a RadiusLaw,
    side: S,
) -> Placing<'a, S> {
    let model = part.topology();
    let radius_at = move |c: f64| {
        let (link, x) = locate(chain, c);
        side.mul(law.radius(link, x))
    };
    let place_at = move |c: f64, previous: Option<&Station<S>>| -> GeopResult<Station<S>> {
        let (link, x) = locate(chain, c);
        let ctx = with_context!(
            "placing the ball {x} of the way along edge {}",
            chain.links[link].edge
        );
        place(model, chain, link, S::from_f64(x), radius_at(c), previous).with_context(ctx)
    };
    let cross = move |k: usize,
                      a: &Station<S>,
                      b: &Station<S>,
                      (ca, cb): (f64, f64)|
          -> GeopResult<(f64, Station<S>)> {
        // How far along it is: where the plane square to the chain through
        // its center crosses the chain, as every station's does — then the
        // radius there, and the ball again with it.
        let along = |x: &Station<S>| -> GeopResult<f64> {
            let off = |c: f64| -> GeopResult<f64> {
                let (link, f) = locate(chain, c);
                let (p, t) = chain.links[link].at(model, S::from_f64(f))?;
                Ok(x.center.sub(&p).prod_dot(&t).midpoint().to_f64())
            };
            let (mut lo, mut hi) = (ca, cb);
            let off_lo = off(lo)?;
            if off_lo.signum() == off(hi)?.signum() {
                return Ok(0.5);
            }
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                if off(mid)?.signum() == off_lo.signum() {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            Ok((0.5 * (lo + hi) - ca) / (cb - ca))
        };
        let guess = crossing(model, k, a, b, radius_at(0.5 * (ca + cb)))?;
        let f = along(&guess)?;
        let station = crossing(model, k, a, b, radius_at(ca + f * (cb - ca)))?;
        Ok((along(&station)?, station))
    };
    let merge = move |s0: &Station<S>, s1: &Station<S>| -> GeopResult<Option<Station<S>>> {
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
    let kinks = if law.varies() {
        let inner = if chain.closed { 0 } else { 1 };
        (inner..chain.links.len()).map(|v| v as f64).collect()
    } else {
        Vec::new()
    };
    Placing {
        place: Box::new(place_at),
        cross: Box::new(cross),
        merge: Box::new(merge),
        kinks,
    }
}

/// How a chamfer's chords are placed along `chain` of `part`, `distances`
/// into the faces on its left and right (see [`Placing`], [`plan_chamfer`]).
fn chamfer_placing<'a, S: Scalar>(
    part: &'a Part<S>,
    chain: &'a Chain,
    distances: [f64; 2],
) -> Placing<'a, S> {
    let model = part.topology();
    let place_at = move |c: f64, previous: Option<&Station<S>>| -> GeopResult<Station<S>> {
        let (link, x) = locate(chain, c);
        let ctx = with_context!(
            "chamfering {x} of the way along edge {}",
            chain.links[link].edge
        );
        chamfer_station(model, chain, link, S::from_f64(x), distances, previous).with_context(ctx)
    };
    let cross = move |k: usize, a: &Station<S>, b: &Station<S>, range: (f64, f64)| {
        chamfer_crossing(model, chain, k, a, b, range, distances)
    };
    let merge = move |s0: &Station<S>, s1: &Station<S>| -> GeopResult<Option<Station<S>>> {
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
    }
}

/// Everything a rolling-ball blend of one chain needs: the chain, which way
/// it bends, its tool, and the contact points to check against the faces
/// (see [`check_touches`]).
#[derive(Clone, Debug)]
pub(crate) struct Rolled<S: Scalar> {
    pub chain: Chain,
    pub bend: Bend,
    pub tool: Tool<S>,
    /// Ends of an open chain: at its start and its end vertex, how the tool
    /// ends there.
    pub ends: Option<[(VertexId, End<S>); 2]>,
    /// Per link, the ball at its middle: where it must touch its faces
    /// inside them.
    middles: Vec<Station<S>>,
}

impl<S: Scalar> Rolled<S> {
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

/// A homogeneous point's position.
fn point<S: Scalar>(h: &Vector4<S>) -> GeopResult<Vector3<S>> {
    Ok(Vector3::from_array([
        h[0].div(h[3])?,
        h[1].div(h[3])?,
        h[2].div(h[3])?,
    ]))
}

/// The plain distance between two points.
fn distance<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> f64 {
    a.sub(b).norm().midpoint().to_f64()
}

/// The control rows of the tool's span `span` through the stations of
/// `rolling`, interpolated at their parameters (see the module docs); the
/// contact rows not yet widened.
fn span_curves<S: Scalar>(
    rolling: &Rolling<S>,
    span: (usize, usize),
) -> GeopResult<[NurbCurve3D<S>; 4]> {
    let params = rolling.params(span);
    let points: Vec<[Vector4<S>; 4]> = rolling.stations[span.0..=span.1]
        .iter()
        .map(section_points)
        .collect();
    let mut curves = Vec::new();
    for k in 0..4 {
        let values: Vec<Vector4<S>> = points.iter().map(|p| p[k]).collect();
        curves.push(NurbCurve::interpolate_homogeneous(&values, &params, 3)?);
    }
    Ok([
        curves[0].clone(),
        curves[1].clone(),
        curves[2].clone(),
        curves[3].clone(),
    ])
}

/// How far the span `curves` strays from the balls `between` at their
/// parameters: from either contact, and from the middle of the section —
/// or, for a chamfer's chords, from each of their control points.
fn span_deviation<S: Scalar>(
    curves: &[NurbCurve3D<S>; 4],
    between: &[Vec<(S, Station<S>)>],
) -> GeopResult<(f64, String)> {
    if between.iter().flatten().any(|(_, station)| station.chord) {
        let mut worst = (0.0f64, String::new());
        for (s, station) in between.iter().flatten() {
            for (k, truth) in section_points(station).iter().enumerate() {
                let (got, truth) = (curves[k].evaluate(*s)?, point(truth)?);
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
    let half = S::ONE.div(S::TWO)?;
    let mut worst = (0.0f64, String::new());
    let mut note = |d: f64, what: &str, s: &S, got: &Vector3<S>, truth: &Vector3<S>| {
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
    let (lo, hi) = curves[0].domain();
    for (s, station) in between.iter().flatten() {
        for k in [0, 2] {
            let truth = station.contact[k / 2];
            let mut t = *s;
            for _ in 0..PROJECT_ITERATIONS {
                let d = curves[k].evaluate(t)?.sub(&truth);
                let tangent = curves[k].tangent(t)?;
                let step = d.prod_dot(&tangent).div(tangent.prod_dot(&tangent))?;
                t = clamp(t.sub(step).sharpen(), lo, hi);
            }
            let got = curves[k].evaluate(t)?;
            note(distance(&got, &truth), "a contact", s, &got, &truth);
            // The blend has to leave the face towards the ball, never dip
            // through it: where it is tangent only approximately, that is
            // what keeps its one meeting with the face its contact curve.
            let (_, across) = blend.derivatives(t, if k == 0 { S::ZERO } else { S::ONE })?;
            let leaving = if k == 0 { across } else { across.neg() };
            if !leaving
                .prod_dot(&station.center.sub(&truth))
                .definitely_greater(S::ZERO)
            {
                return Ok((
                    f64::INFINITY,
                    format!(
                        "at {:?} of the span the blend could dip through face {} at {truth:?}",
                        s.midpoint().to_f64(),
                        station.faces[k / 2]
                    ),
                ));
            }
        }
        let truth = section_points(station);
        let middle = point(&truth[0].add(&truth[1].prod_scalar(S::TWO)).add(&truth[2]))?;
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
fn span_of<S: Scalar>(curves: &[NurbCurve3D<S>; 4], name: Option<String>) -> ToolSpan<S> {
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
fn contact_checks<S: Scalar>(
    rolling: &Rolling<S>,
    span: (usize, usize),
    k: usize,
) -> Vec<(S, Vector3<S>)> {
    let params = rolling.params(span);
    let stations = params.into_iter().zip(&rolling.stations[span.0..=span.1]);
    let between = rolling.between(span);
    stations
        .map(|(s, station)| (s, station.contact[k / 2]))
        .chain(
            between
                .iter()
                .flatten()
                .map(|(s, station)| (*s, station.contact[k / 2])),
        )
        .collect()
}

/// How an open chain's tool ends at a vertex: the vertex, the end, the
/// third face there and the way out of the chain.
type ChainEnd<S> = (VertexId, End<S>, Option<FaceId>, Vector3<S>);

/// How the tool of `chain`, bending `bend`, ends at the start and the end
/// of it, if it is open (see [`tool_end`]) — joined to a corner's ball
/// where `corners` gives its position along the chain: decided before
/// anything is rolled, so that an end that is not supported is refused by
/// name.
fn chain_ends<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    bend: Bend,
    corners: [Option<f64>; 2],
) -> GeopResult<Option<[ChainEnd<S>; 2]>> {
    if chain.closed {
        return Ok(None);
    }
    let vertices = chain.vertices(model)?;
    let first_link = &chain.links[0];
    let last_link = chain.links.last().expect("links");
    let (_, t0) = first_link.at(model, S::ZERO)?;
    let (_, t1) = last_link.at(model, S::ONE)?;
    let mut ends = Vec::new();
    for (k, (vertex, faces, out)) in [
        (vertices[0], first_link.faces, t0.neg()),
        (*vertices.last().expect("vertices"), last_link.faces, t1),
    ]
    .into_iter()
    .enumerate()
    {
        if let Some(position) = corners[k] {
            let setback = S::from_f64(position);
            ends.push((vertex, End::Corner { setback }, None, out));
            continue;
        }
        let ctx = with_context!("the blend's end at vertex {vertex}");
        let (kind, face) = tool_end(model, vertex, faces, &out, bend).with_context(ctx)?;
        if matches!(kind, End::Mitre { .. }) {
            return Err(GeopError::new("a rolling-ball blend is not mitred")).with_context(ctx);
        }
        ends.push((vertex, kind, Some(face), out));
    }
    let [a, b]: [ChainEnd<S>; 2] = ends
        .try_into()
        .map_err(|_| GeopError::new("an open chain has two ends"))?;
    Ok(Some([a, b]))
}

/// Where an open chain ends at a corner's ball (see [`crate::corner`]):
/// the ball's center, and where it touches the faces on the chain's left
/// and right there.
#[derive(Clone, Debug)]
pub(crate) struct ChainCorner<S: Scalar> {
    pub center: Vector3<S>,
    pub contacts: [Vector3<S>; 2],
}

/// Which way the faces either side of `chain` bend: the same all along, or
/// an error.
pub(crate) fn chain_bend<S: Scalar>(model: &Model<S>, chain: &Chain) -> GeopResult<Bend> {
    // Which way it bends, where the chain starts.
    let first = &chain.links[0];
    // The way into the face on the left — its normal crossed with the way
    // its coedge runs, the chain's — against the right one's normal.
    let bend_at = |link: &Link, fraction: S| -> GeopResult<Bend> {
        let (p, t) = link.at(model, fraction)?;
        let n_l = normal_at(model, link.faces[0], &p)?;
        let n_r = normal_at(model, link.faces[1], &p)?;
        if n_l.prod_cross(&n_r).norm_sq().could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "faces {} and {} could meet tangentially at edge {}: there is no corner to blend",
                link.faces[0], link.faces[1], link.edge
            )));
        }
        bend(n_l.prod_cross(&t).prod_dot(&n_r))
    };
    let bend = bend_at(first, S::from_f64(0.5))?;
    for (i, link) in chain.links.iter().enumerate().skip(1) {
        if bend_at(link, S::from_f64(0.5))? != bend {
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
fn corner_position<S: Scalar>(
    model: &Model<S>,
    chain: &Chain,
    k: usize,
    center: &Vector3<S>,
) -> GeopResult<f64> {
    let links = chain.links.len() as f64;
    // How far further into the chain than the plane square to it at `c`
    // the center is.
    let ahead = |c: f64| -> GeopResult<f64> {
        let (link, f) = locate(chain, c);
        let (p, t) = chain.links[link].at(model, S::from_f64(f))?;
        let off = center.sub(&p).prod_dot(&t).midpoint().to_f64();
        Ok(if k == 0 { off } else { -off })
    };
    let end = if k == 0 { 0.0 } else { links };
    let inward = if k == 0 { 1.0 } else { -1.0 };
    let step = 1.0 / 32.0;
    let mut near = end;
    let mut far = end;
    loop {
        far += inward * step;
        if (far - end).abs() > links {
            return Err(GeopError::new(format!(
                "the corner's ball, centered at {center:?}, meets no section of the blend along its chain"
            )));
        }
        if ahead(far)? <= 0.0 {
            break;
        }
        near = far;
    }
    for _ in 0..60 {
        let mid = 0.5 * (near + far);
        if ahead(mid)? > 0.0 {
            near = mid;
        } else {
            far = mid;
        }
    }
    Ok(0.5 * (near + far))
}

/// Plans the rolling-ball blend of `chain` with `radii` (see the module
/// docs): the ball rolled along it — up to the balls `corners` where it
/// ends at one, at its start and its end — the tool skinned through its
/// stations, refined until it follows the ball, and its ends decided.
pub(crate) fn plan_rolled<S: Scalar>(
    part: &Part<S>,
    chain: Chain,
    radii: &Radii,
    corners: [Option<ChainCorner<S>>; 2],
) -> GeopResult<Rolled<S>> {
    let model = part.topology();
    let law = RadiusLaw::new(part, &chain, radii)?;
    let bend = chain_bend(model, &chain)?;
    // The stations where the chain meets a corner's ball, exactly.
    let mut at_corners: [Option<(f64, Station<S>)>; 2] = [None, None];
    for (k, corner) in corners.iter().enumerate() {
        let Some(corner) = corner else {
            continue;
        };
        let link = if k == 0 {
            &chain.links[0]
        } else {
            chain.links.last().expect("links")
        };
        let ctx = with_context!("the blend's end at the corner ball at {:?}", corner.center);
        let mut uv = [(S::ZERO, S::ZERO); 2];
        let mut normals = [Vector3::zero(); 2];
        for side in 0..2 {
            let surface = &model.get_face(link.faces[side])?.surface;
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
        let position = corner_position(model, &chain, k, &corner.center).with_context(ctx)?;
        at_corners[k] = Some((position, station));
    }
    let positions = [0, 1].map(|k| at_corners[k].as_ref().map(|(c, _)| *c));
    let decided_ends = chain_ends(model, &chain, bend, positions)?;
    let side = bend.side::<S>();
    let (rolling, curves) = {
        let placing = fillet_placing(part, &chain, &law, side);
        refined(law.smallest(), |n| {
            roll_chain(&chain, n, &at_corners, &placing)
        })?
    };
    assemble(model, chain, bend, rolling, curves, decided_ends)
}

/// Plans the chamfer of `chain`, `distances` into the faces on its left and
/// right: rolled along it as a fillet is (see the module docs), its
/// sections chords. At each station, in the plane square to the chain, the
/// chord runs between the faces' points that distance from the edge — as
/// the crow flies, which on a plane is as far along it — and its apex is
/// as far beyond the edge as the chord's middle is before it. Its contact
/// rows are widened to enclose the true contacts as a fillet's are; unlike
/// a fillet's, the chamfer crosses the faces there, so needs no tilt.
pub(crate) fn plan_chamfer<S: Scalar>(
    part: &Part<S>,
    chain: Chain,
    distances: [f64; 2],
) -> GeopResult<Rolled<S>> {
    if !distances.iter().all(|&d| d > 0.0) {
        return Err(GeopError::new(format!(
            "a chamfer's distances have to be positive, not {distances:?}"
        )));
    }
    let model = part.topology();
    let bend = chain_bend(model, &chain)?;
    let decided_ends = chain_ends(model, &chain, bend, [None, None])?;
    let smallest = distances[0].min(distances[1]);
    let (rolling, curves) = {
        let placing = chamfer_placing(part, &chain, distances);
        refined(smallest, |n| roll_chain(&chain, n, &[None, None], &placing))?
    };
    assemble(model, chain, bend, rolling, curves, decided_ends)
}

/// The sections `roll` places along a chain, `n` to an edge, doubled from
/// [`FIRST_STATIONS`] until the spans skinned through them follow those
/// between to within [`DEVIATION`] of `size`, and those spans' control
/// rows — or an error saying how far they got.
#[allow(clippy::type_complexity)]
fn refined<S: Scalar>(
    size: f64,
    roll: impl Fn(usize) -> GeopResult<Rolling<S>>,
) -> GeopResult<(Rolling<S>, Vec<[NurbCurve3D<S>; 4]>)> {
    let smallest = size;
    let mut n = FIRST_STATIONS;
    let mut before = f64::INFINITY;
    loop {
        let rolling = roll(n)?;
        let mut curves = Vec::new();
        let mut deviation = 0.0f64;
        let mut worst = String::new();
        for (j, span) in rolling.spans().into_iter().enumerate() {
            let c = span_curves(&rolling, span)?;
            let (d, where_) = span_deviation(&c, &rolling.between(span))?;
            if d > deviation {
                deviation = d;
                worst = format!("in span {j}, {where_}");
            }
            curves.push(c);
        }
        if deviation <= DEVIATION * smallest {
            return Ok((rolling, curves));
        }
        // Doubling the stations of a cubic shrinks its deviation sixteen
        // times over; one that does not even halve is not converging, and
        // more stations will not help.
        if n >= MOST_STATIONS
            || deviation > before / 2.0
            || before.is_infinite() && deviation.is_infinite() && n >= 64
        {
            return Err(GeopError::new(format!(
                "the blend strays {deviation:e} from the rolling ball with {n} stations per edge ({before:e} with half as many), more than the {:e} it may (a {DEVIATION:e} of its radius): {worst}",
                DEVIATION * smallest
            )));
        }
        before = deviation;
        n *= 2;
    }
}

/// Checks the ball's section at the end `station` of an open chain lies in
/// the plane of `face`, the third face at the corner — the faces either side
/// stand square to it where the ball touches them — so that the tool, cut
/// off there or run straight on out of the solid, crosses it exactly there.
/// The plane, oriented along `out`, as the cap's `(origin, e1, e2)`.
fn end_plane<S: Scalar>(
    model: &Model<S>,
    station: &Station<S>,
    face: FaceId,
    out: &Vector3<S>,
) -> GeopResult<[Vector3<S>; 3]> {
    let Some(plane) = model.get_face(face)?.surface.as_plane()? else {
        return Err(GeopError::new(format!("face {face} is not planar")));
    };
    let mut m = plane.normal.normalize()?;
    let along = m.prod_dot(out);
    if along.definitely_less(S::ZERO) {
        m = m.neg();
    } else if !along.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "face {face} could run along the edge at its end"
        )));
    }
    for (k, n) in station.normals.iter().enumerate() {
        if !n.prod_dot(&m).could_be_equal(S::ZERO) {
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
fn assemble<S: Scalar>(
    model: &Model<S>,
    chain: Chain,
    bend: Bend,
    rolling: Rolling<S>,
    mut curves: Vec<[NurbCurve3D<S>; 4]>,
    decided_ends: Option<[ChainEnd<S>; 2]>,
) -> GeopResult<Rolled<S>> {
    // One pad for every contact row, so that spans meeting at a station
    // meet in the same control points there.
    let chord = rolling.stations[0].chord;
    let mut pad = [S::ZERO; 3];
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

    // Which way round the section runs: the blend's normal — along the
    // chain, crossed with the way from the first contact to the second —
    // has to face the ball, out of the tool.
    let first = &rolling.stations[0];
    let along = rolling.stations[1].center.sub(&first.center);
    let facing = along
        .prod_cross(&first.contact[1].sub(&first.contact[0]))
        .prod_dot(&first.center.sub(&first.corner));
    let swap = if facing.definitely_greater(S::ZERO) {
        false
    } else if facing.definitely_less(S::ZERO) {
        true
    } else {
        return Err(GeopError::new(
            "cannot tell which way the blend faces at the start of the chain",
        ));
    };
    let order = |p: [Vector4<S>; 4]| if swap { [p[2], p[1], p[0], p[3]] } else { p };
    let vertices_of = |s: &Station<S>| {
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

    let mut ends = None;
    if let Some(decided_ends) = decided_ends {
        let mut decided = Vec::new();
        for (at_end, (vertex, kind, face, out)) in decided_ends
            .into_iter()
            .enumerate()
            .map(|(i, e)| (i == 1, e))
        {
            let station = if at_end { last_station } else { first };
            let ctx = with_context!("the blend's end at vertex {vertex}");
            let Some(face) = face else {
                // Joined to a corner's ball: open, its contacts the
                // corner's.
                decided.push((vertex, kind));
                continue;
            };
            let plane = end_plane(model, station, face, &out).with_context(ctx)?;
            let index = if at_end { stations.len() - 1 } else { 0 };
            let name = if at_end { "end" } else { "start" };
            // The section's control points and vertices in the plane.
            let flat = stations[index].points.map(|p| flatten(&p, &plane));
            let flat_vertices = stations[index].vertices.map(|v| flatten_point(&v, &plane));
            match &kind {
                End::Flush | End::Wall => {
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
                    let shifted = [o.add(&normal.prod_scalar(S::from_f64(2.0 * width))), e1, e2];
                    let far = ToolStation {
                        points: flat.map(|p| embed(&p, &shifted)),
                        vertices: flat_vertices.map(|p| embed(&p, &shifted).head::<3>()),
                        name: format!("{name}_out"),
                        cap: Some(Cap {
                            plane: shifted,
                            flat,
                        }),
                    };
                    let line = |a: [Vector4<S>; 4], b: [Vector4<S>; 4]| ToolSpan {
                        degree: 1,
                        knots: vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                        rows: vec![a, b],
                        name: Some(name.to_string()),
                    };
                    if at_end {
                        spans.push(line(stations[index].points, far.points));
                        stations.push(far);
                    } else {
                        spans.insert(0, line(far.points, stations[index].points));
                        stations.insert(0, far);
                    }
                }
                End::Mitre { .. } | End::Corner { .. } => {
                    return Err(GeopError::new(
                        "a rolling-ball blend is neither mitred nor joined to a corner",
                    ))
                    .with_context(ctx);
                }
            }
            decided.push((vertex, kind));
        }
        let [a, b]: [(VertexId, End<S>); 2] = decided
            .try_into()
            .map_err(|_| GeopError::new("an open chain has two ends"))?;
        ends = Some([a, b]);
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
            rolling.stations[nearest].clone()
        })
        .collect();
    let closed = chain.closed;
    Ok(Rolled {
        chain,
        bend,
        tool: Tool {
            stations,
            spans,
            closed,
            sides: if swap { ["b", "a"] } else { ["a", "b"] },
            blend: if chord { "chamfer" } else { "fillet" },
        },
        ends,
        middles,
    })
}

/// Checks the blend meets each face where the face is: the ball halfway
/// along each edge of the chain touches its faces inside them. A blend too
/// large for a face runs past its boundary, and one ending exactly on
/// another of its edges asks the boolean to cut along that edge.
pub(crate) fn check_touches<S: Scalar>(model: &Model<S>, rolled: &Rolled<S>) -> GeopResult<()> {
    let params = RemeshParams::<S>::default();
    let (max_nodes, size) = (params.max_nodes, params.min_subdivision_size);
    for station in &rolled.middles {
        for k in 0..2 {
            let face = station.faces[k];
            let (u, v) = station.uv[k];
            let ctx = with_context!(
                "where the blend meets face {face}, at {:?}",
                station.contact[k]
            );
            match face_contains(model, face, u, v, max_nodes, size, SEED).with_context(ctx)? {
                PointClassification::Inside => {}
                PointClassification::Outside => {
                    return Err(GeopError::new(format!(
                        "the blend is too large for face {face}: it would run past the face's boundary"
                    )))
                    .with_context(ctx);
                }
                PointClassification::OnCoedge | PointClassification::OnVertex => {
                    return Err(GeopError::new(format!(
                        "the blend would end exactly on another edge of face {face}: make it a little smaller or larger"
                    )))
                    .with_context(ctx);
                }
            }
        }
    }
    Ok(())
}
