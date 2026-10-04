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
//! tangent plane at its contact point, so the blend is tangent to both
//! faces at every station. (With a varying radius the ball's true
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
//! a circle made of quarter arcs, a slot's rim of lines and arcs. Each edge
//! of the chain gets a span of the tool of its own, all meeting at stations
//! on the chain's vertices; the faces either side may change along it, from
//! one face to the next tangent to it, and the ball touches whichever of
//! them it reaches.
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
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D, true_point_fractions},
    nurb_surface::{NurbSurface, NurbSurface3D, clamp},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    EdgeId, FaceId, Model, Sense, SolidId, VertexId,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
    contains::face::{PointClassification, face_contains},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_booleans::remesh::remesh::RemeshParams;
use geop_ops_extrude_revolve::common::{bilinear, embed_point, line2};

use crate::blend::{Bend, End, bend, faces, tool_end};

/// Stations per edge of a chain to start with.
const FIRST_STATIONS: usize = 8;
/// The most stations per edge the blend is refined to — how hard it tries
/// to follow the rolling ball, not what following it means.
const MOST_STATIONS: usize = 256;
/// How closely the blend has to follow the rolling ball between stations,
/// as a fraction of the radius: the deviation every blend is checked to
/// stay within.
pub const DEVIATION: f64 = 1e-6;
/// Newton iterations placing the ball at a station.
const NEWTON_ITERATIONS: usize = 30;
/// Newton iterations of each foot-point projection.
const PROJECT_ITERATIONS: usize = 20;
/// How far off its true foot point, relative to the ball, a contact found
/// on the faces an edge runs between may be and count as found exactly:
/// rounding, and no more. Only spares trying other faces (see [`place`]).
const ROUNDING: f64 = 1e3 * f64::EPSILON;
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
    fn vertices<S: Scalar>(&self, model: &Model<S>) -> GeopResult<Vec<VertexId>> {
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
fn seed_on<S: Scalar>(surface: &NurbSurface3D<S>, point: &Vector3<S>) -> GeopResult<(S, S)> {
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
fn foot<S: Scalar>(
    surface: &NurbSurface3D<S>,
    target: &Vector3<S>,
    seed: (S, S),
) -> GeopResult<(S, S)> {
    let (u, v) = surface.project(*target, seed.0, seed.1, PROJECT_ITERATIONS)?;
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    Ok((u.intersect(u0.union(u1)), v.intersect(v0.union(v1))))
}

/// `(u, v)` sharpened, within the domain of `surface`: a seed.
fn sharp_in<S: Scalar>(surface: &NurbSurface3D<S>, (u, v): (S, S)) -> (S, S) {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    (clamp(u.sharpen(), u0, u1), clamp(v.sharpen(), v0, v1))
}

/// The unit outward normal of `face` at its point `point`.
fn normal_at<S: Scalar>(model: &Model<S>, face: FaceId, point: &Vector3<S>) -> GeopResult<Vector3<S>> {
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
            let leaves = if forward { e.start_vertex } else { e.end_vertex };
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

/// Samples per edge of the length table a varying radius is laid out by.
const LENGTH_SAMPLES: usize = 32;

/// A fillet's radius along one chain (see [`Radii`]): the length along the
/// chain of points of each link — a table of plain numbers, since the
/// radius law is the designer's choice and this is how it is laid out —
/// and the radius at lengths along it, linear between.
struct RadiusLaw {
    /// Per link: the length along the chain at `i / LENGTH_SAMPLES` of its
    /// parameter.
    lengths: Vec<Vec<f64>>,
    /// `(length, radius)`, by increasing length; for a closed chain the
    /// first again at the total length.
    knots: Vec<(f64, f64)>,
}

impl RadiusLaw {
    fn new<S: Scalar>(part: &Part<S>, chain: &Chain, radii: &Radii) -> GeopResult<Self> {
        let model = part.topology();
        let mut lengths = Vec::new();
        let mut total = 0.0;
        for link in &chain.links {
            let mut table = vec![total];
            let mut last = link.at(model, S::ZERO)?.0;
            for i in 1..=LENGTH_SAMPLES {
                let f = S::from_ratio(i as i64, LENGTH_SAMPLES as i64)?;
                let p = link.at(model, f)?.0;
                total += p.sub(&last).norm().midpoint().to_f64();
                table.push(total);
                last = p;
            }
            lengths.push(table);
        }
        let vertices = chain.vertices(model)?;
        let given = |v: VertexId| {
            part.name_of(v)
                .and_then(|name| radii.at_vertices.iter().find(|(n, _)| n == name))
                .map(|(_, r)| *r)
        };
        let mut knots = Vec::new();
        for (k, &v) in vertices.iter().enumerate() {
            let at = if k < chain.links.len() {
                lengths[k][0]
            } else {
                total
            };
            let first = k == 0;
            let last = !chain.closed && k + 1 == vertices.len();
            let r = match given(v) {
                Some(r) => Some(r),
                None if first => Some(radii.radius),
                None if last => Some(radii.end_radius.unwrap_or(radii.radius)),
                None => None,
            };
            if let Some(r) = r {
                knots.push((at, r));
            }
        }
        if chain.closed {
            if radii.end_radius.is_some() {
                return Err(GeopError::new(
                    "the edge's tangent chain is closed, so it has no end for an end radius: give radii at its vertices instead",
                ));
            }
            knots.push((total, knots[0].1));
        }
        Ok(RadiusLaw { lengths, knots })
    }

    /// The radius `fraction` of the way along link `link`, sharp: the law is
    /// a choice, and this is it.
    fn radius<S: Scalar>(&self, link: usize, fraction: f64) -> S {
        let table = &self.lengths[link];
        let x = fraction * LENGTH_SAMPLES as f64;
        let i = (x.floor() as usize).min(LENGTH_SAMPLES - 1);
        let s = table[i] + (x - i as f64) * (table[i + 1] - table[i]);
        let r = match self.knots.iter().position(|&(at, _)| at >= s) {
            None => self.knots.last().expect("a knot").1,
            Some(0) => self.knots[0].1,
            Some(j) => {
                let ((s0, r0), (s1, r1)) = (self.knots[j - 1], self.knots[j]);
                if s1 > s0 {
                    r0 + (s - s0) / (s1 - s0) * (r1 - r0)
                } else {
                    r1
                }
            }
        };
        S::from_f64(r)
    }
}

/// The ball at one station: its center, where it touches each face — the
/// faces on the chain's left and right — the faces' outward normals there
/// and the contacts' `(u, v)` on them (sharp, seeds), and the section's
/// middle control point, its weight and the tool's apex (see the module
/// docs).
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
    let mut center = p
        .add(&n0.add(&n1).prod_scalar(sr.div(spread)?))
        .sharpen();
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
        let jacobian = Matrix::from_rows([
            rows[0].to_array(),
            rows[1].to_array(),
            rows[2].to_array(),
        ]);
        let step = solve_linear_system(
            &jacobian,
            &Vector3::from_array([residual[0], residual[1], residual[2]]),
        )?;
        let next = center.sub(&step).sharpen();
        let same = (0..3).all(|k| next[k].is_subset_of(center[k]) && center[k].is_subset_of(next[k]));
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
    d.sub(&n.prod_scalar(d.prod_dot(n))).norm().midpoint().to_f64()
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
    let across = normal_at(model, own.faces[0], &p)?.prod_cross(&normal_at(model, own.faces[1], &p)?);
    let tangent = if across.prod_dot(&along).definitely_less(S::ZERO) {
        across.neg()
    } else {
        across
    }
    .normalize()?;
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
            Ok((off, section(center, contact, normals, pair, final_uv)?))
        };
        let (off, station) = match attempt() {
            Ok(found) => found,
            Err(e) => {
                tried.push((pair, e.root_message().to_string()));
                continue;
            }
        };
        // A true foot point on both of the edge's own faces, but for
        // rounding, is the ball: no need to try other faces.
        if pair == own.faces && off <= scale * ROUNDING {
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
/// choices of the blend's shape, sharp.
fn section<S: Scalar>(
    center: Vector3<S>,
    contact: [Vector3<S>; 2],
    normals: [Vector3<S>; 2],
    faces: [FaceId; 2],
    uv: [(S, S); 2],
) -> GeopResult<Station<S>> {
    let [na, nb] = normals;
    let [ta, tb] = contact;
    let m = na.prod_cross(&nb);
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
    })
}

/// The section's control points, homogeneous: the contact on the left,
/// the middle (weighted), the contact on the right, the apex.
fn section_points<S: Scalar>(station: &Station<S>) -> [Vector4<S>; 4] {
    let h = |p: &Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
    let w = station.weight;
    let e = station.corner;
    [
        h(&station.contact[0].sharpen()),
        Vector4::from_array([e[0].mul(w), e[1].mul(w), e[2].mul(w), w]),
        h(&station.contact[1].sharpen()),
        h(&station.apex),
    ]
}

/// The balls rolled along a chain: at the stations, at their parameters
/// along the tool's span and their positions along the chain — link index
/// plus fraction of its edge — and between each two where
/// `true_point_fractions` says, at their parameters, to measure the blend
/// against.
struct Rolling<S: Scalar> {
    stations: Vec<Station<S>>,
    params: Vec<S>,
    positions: Vec<f64>,
    between: Vec<Vec<(S, Station<S>)>>,
}

/// Where along `chain` its stations are, `n` to an edge: positions — link
/// index plus fraction of its edge — half a step off every vertex between
/// two links, where the faces either side may change and the ball would
/// touch the edge between them exactly at a station; an open chain also at
/// both its ends, a closed one round to its first again.
fn station_positions(chain: &Chain, n: usize) -> Vec<f64> {
    let count = n * chain.links.len();
    let inner = (0..count).map(|k| (k as f64 + 0.5) / n as f64);
    if chain.closed {
        let mut positions: Vec<f64> = inner.collect();
        positions.push(positions[0] + chain.links.len() as f64);
        positions
    } else {
        std::iter::once(0.0)
            .chain(inner)
            .chain(std::iter::once(chain.links.len() as f64))
            .collect()
    }
}

/// The ball rolled along `chain` with `n` stations to an edge (see
/// [`station_positions`]), its radius as `law` says, on the side `side` of
/// the faces.
fn roll_chain<S: Scalar>(
    part: &Part<S>,
    chain: &Chain,
    law: &RadiusLaw,
    side: S,
    n: usize,
) -> GeopResult<Rolling<S>> {
    let model = part.topology();
    let links = chain.links.len();
    let positions = station_positions(chain, n);
    let (first, last) = (positions[0], *positions.last().expect("positions"));
    // The position `c` as a link and a fraction of its edge.
    let locate = |c: f64| {
        let c = if c >= links as f64 && chain.closed {
            c - links as f64
        } else {
            c
        };
        let link = (c.floor() as usize).min(links - 1);
        (link, c - link as f64)
    };
    let mut previous: Option<Station<S>> = None;
    let mut place_at = |c: f64| -> GeopResult<Station<S>> {
        let (link, x) = locate(c);
        let ctx = with_context!(
            "placing the ball {x} of the way along edge {}",
            chain.links[link].edge
        );
        let station = place(
            model,
            chain,
            link,
            S::from_f64(x),
            side.mul(law.radius(link, x)),
            previous.as_ref(),
        )
        .with_context(ctx)?;
        previous = Some(station.clone());
        Ok(station)
    };
    let param_of = |c: f64| {
        if c == first {
            S::ZERO
        } else if c == last {
            S::ONE
        } else {
            S::from_f64((c - first) / (last - first))
        }
    };
    let mut stations: Vec<Station<S>> = Vec::with_capacity(positions.len());
    let mut between = Vec::with_capacity(positions.len() - 1);
    let intervals = positions.len() - 1;
    for (i, &c) in positions.iter().enumerate() {
        // Round a closed chain, the last station is the first again.
        let station = if chain.closed && i == intervals {
            stations[0].clone()
        } else {
            place_at(c)?
        };
        stations.push(station);
        if i == intervals {
            break;
        }
        let next = positions[i + 1];
        let mut inside = Vec::new();
        for &(a, b) in true_point_fractions(i, intervals) {
            let at = c + (next - c) * a as f64 / b as f64;
            inside.push((param_of(at), place_at(at)?));
        }
        between.push(inside);
    }
    Ok(Rolling {
        stations,
        params: positions.iter().map(|&c| param_of(c)).collect(),
        positions,
        between,
    })
}

/// One span of a tool: its control rows along it — each the four section
/// control points (see [`section_points`]) — of `degree` on `knots`, and
/// what its faces are called after, if anything.
#[derive(Clone, Debug)]
struct ToolSpan<S: Scalar> {
    degree: usize,
    knots: Vec<S>,
    rows: Vec<[Vector4<S>; 4]>,
    name: Option<String>,
}

/// A flat cap of a tool: the plane it lies in, `(origin, e1, e2)` with
/// `e1 x e2` out of the tool, and the section's control points in it,
/// homogeneous — what both the cap's pcurves and the section's edges in
/// space are made of, so that they agree exactly.
#[derive(Clone, Debug)]
struct Cap<S: Scalar> {
    plane: [Vector3<S>; 3],
    flat: [Vector3<S>; 4],
}

/// Where a tool's section lies at one of its stations: the section's
/// control points, its vertices — the two contacts and the apex — what it
/// is called, and its cap, if it is capped there.
#[derive(Clone, Debug)]
struct ToolStation<S: Scalar> {
    points: [Vector4<S>; 4],
    vertices: [Vector3<S>; 3],
    name: String,
    cap: Option<Cap<S>>,
}

/// A blend's tool, ready to build: its stations and the spans between
/// them, around to the first again if `closed` — else capped at both ends.
#[derive(Clone, Debug)]
pub(crate) struct Tool<S: Scalar> {
    stations: Vec<ToolStation<S>>,
    spans: Vec<ToolSpan<S>>,
    closed: bool,
    /// What the walls rising from the first and the second contact are
    /// called: `a` and `b` after the faces on the edge's left and right.
    sides: [&'static str; 2],
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
    /// How far the blend strays from the rolling ball, at most, measured.
    pub deviation: f64,
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

/// The control rows of the tool's span through the stations of
/// `rolling`, interpolated at their parameters (see the module docs); the
/// contact rows not yet widened.
fn span_curves<S: Scalar>(rolling: &Rolling<S>) -> GeopResult<[NurbCurve3D<S>; 4]> {
    let points: Vec<[Vector4<S>; 4]> = rolling.stations.iter().map(section_points).collect();
    let mut curves = Vec::new();
    for k in 0..4 {
        let values: Vec<Vector4<S>> = points.iter().map(|p| p[k]).collect();
        curves.push(NurbCurve::interpolate_homogeneous(&values, &rolling.params, 3)?);
    }
    Ok([
        curves[0].clone(),
        curves[1].clone(),
        curves[2].clone(),
        curves[3].clone(),
    ])
}

/// How far the span `curves` strays from the balls `between` at their
/// parameters: from either contact, and from the middle of the section.
fn span_deviation<S: Scalar>(
    curves: &[NurbCurve3D<S>; 4],
    between: &[Vec<(S, Station<S>)>],
) -> GeopResult<(f64, String)> {
    let blend = wall_surface(&span_of(curves, None), 0)?;
    let half = S::ONE.div(S::TWO)?;
    let mut worst = (0.0f64, String::new());
    let mut note = |d: f64, what: &str, s: &S, got: &Vector3<S>, truth: &Vector3<S>| {
        if d > worst.0 {
            worst = (
                d,
                format!("{what} at {:?} of the span: {got:?} where the ball has {truth:?}", s.midpoint().to_f64()),
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
        }
        let truth = section_points(station);
        let middle = point(&truth[0].add(&truth[1].prod_scalar(S::TWO)).add(&truth[2]))?;
        let (u, v) = foot(&blend, &middle, (*s, half))?;
        let got = blend.evaluate(u, v)?;
        note(distance(&got, &middle), "the section's middle", s, &got, &middle);
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

/// The wall section curve `c` (see [`CURVE_POINTS`]) sweeps through
/// `span`: `u` along the span, `v` along the curve.
fn wall_surface<S: Scalar>(span: &ToolSpan<S>, c: usize) -> GeopResult<NurbSurface3D<S>> {
    let control_points = span
        .rows
        .iter()
        .flat_map(|row| CURVE_POINTS[c].iter().map(move |&k| row[k]))
        .collect();
    let (degree_v, knots_v) = if c == 0 {
        (2, vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE])
    } else {
        (1, vec![S::ZERO, S::ZERO, S::ONE, S::ONE])
    };
    NurbSurface::try_new(
        span.degree,
        degree_v,
        control_points,
        span.knots.clone(),
        knots_v,
    )
}

/// The contact checks of the span: every station's contact point at its
/// parameter, and every ball between, for the contact row `k` (0 or 2).
fn contact_checks<S: Scalar>(rolling: &Rolling<S>, k: usize) -> Vec<(S, Vector3<S>)> {
    let stations = rolling.params.iter().zip(&rolling.stations);
    let between = rolling.between.iter().flatten().map(|(s, b)| (s, b));
    stations
        .chain(between)
        .map(|(s, station)| (*s, station.contact[k / 2]))
        .collect()
}

/// Plans the rolling-ball blend of `chain` with `radii` (see the module
/// docs): the ball rolled along it, the tool skinned through its stations,
/// refined until it follows the ball, and its ends decided.
pub(crate) fn plan_rolled<S: Scalar>(
    part: &Part<S>,
    chain: Chain,
    radii: &Radii,
) -> GeopResult<Rolled<S>> {
    let model = part.topology();
    let law = RadiusLaw::new(part, &chain, radii)?;
    // Which way it bends, where the chain starts.
    let first = &chain.links[0];
    // The way into the face on the left — its normal crossed with the way
    // its coedge runs, the chain's — against the right one's normal.
    let bend_at = |link: &Link, fraction: S| -> GeopResult<Bend> {
        let (p, t) = link.at(model, fraction)?;
        let n_l = normal_at(model, link.faces[0], &p)?;
        let n_r = normal_at(model, link.faces[1], &p)?;
        bend(n_l.prod_cross(&t).prod_dot(&n_r))
    };
    let bend = bend_at(first, S::ZERO)?;
    for (i, link) in chain.links.iter().enumerate().skip(1) {
        if bend_at(link, S::from_f64(0.5))? != bend {
            return Err(GeopError::new(format!(
                "the edge's tangent chain turns from {bend:?} to the other way along edge {} (link {i}): blend its parts separately",
                link.edge
            )));
        }
    }
    let side = bend.side::<S>();
    let smallest = law
        .knots
        .iter()
        .map(|&(_, r)| r)
        .fold(f64::INFINITY, f64::min);

    let mut n = FIRST_STATIONS;
    let mut before = f64::INFINITY;
    loop {
        let rolling = roll_chain(part, &chain, &law, side, n)?;
        let curves = span_curves(&rolling)?;
        let (deviation, worst) = span_deviation(&curves, &rolling.between)?;
        if deviation <= DEVIATION * smallest {
            return assemble(model, chain, bend, rolling, curves, deviation);
        }
        // Doubling the stations of a cubic shrinks its deviation sixteen
        // times over; one that does not even halve is not converging, and
        // more stations will not help.
        if n >= MOST_STATIONS || deviation > before / 2.0 {
            return Err(GeopError::new(format!(
                "the blend strays {deviation:e} from the rolling ball with {n} stations per edge ({before:e} with half as many), more than the {:e} it may (a {DEVIATION:e} of its radius): {worst}",
                DEVIATION * smallest
            )));
        }
        before = deviation;
        n *= 2;
    }
}


/// The homogeneous point `h`'s coordinates in the plane `(origin, e1, e2)`,
/// homogeneous: `(w x, w y, w)`.
fn flatten<S: Scalar>(h: &Vector4<S>, [o, e1, e2]: &[Vector3<S>; 3]) -> Vector3<S> {
    let w = h[3];
    let d = Vector3::from_array([h[0], h[1], h[2]]).sub(&o.prod_scalar(w));
    Vector3::from_array([d.prod_dot(e1), d.prod_dot(e2), w])
}

/// The point `v`'s coordinates in the plane `plane`, homogeneous.
fn flatten_point<S: Scalar>(v: &Vector3<S>, plane: &[Vector3<S>; 3]) -> Vector3<S> {
    flatten(&Vector4::from_array([v[0], v[1], v[2], S::ONE]), plane)
}

/// The homogeneous plane coordinates `p` in space, on `plane`.
fn embed<S: Scalar>(p: &Vector3<S>, [o, e1, e2]: &[Vector3<S>; 3]) -> Vector4<S> {
    embed_point(p, o, e1, e2)
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
    mut curves: [NurbCurve3D<S>; 4],
    deviation: f64,
) -> GeopResult<Rolled<S>> {
    for k in [0, 2] {
        let pad = curves[k].enclosing_pad(&contact_checks(&rolling, k))?;
        curves[k].widen(&pad);
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

    // One span along the whole chain, from a station at its start — and,
    // open, to one at its end.
    let mut span = span_of(&curves, None);
    span.rows = span.rows.into_iter().map(order).collect();
    let last_row = *span.rows.last().expect("rows");
    let mut stations = vec![ToolStation {
        points: span.rows[0],
        vertices: vertices_of(first),
        name: "st0".to_string(),
        cap: None,
    }];
    let mut spans = vec![span];
    let last_station = rolling.stations.last().expect("stations");
    if !chain.closed {
        stations.push(ToolStation {
            points: last_row,
            vertices: vertices_of(last_station),
            name: "st1".to_string(),
            cap: None,
        });
    }

    let mut ends = None;
    if !chain.closed {
        let vertices = chain.vertices(model)?;
        let first_link = &chain.links[0];
        let last_link = chain.links.last().expect("links");
        let (_, t0) = first_link.at(model, S::ZERO)?;
        let (_, t1) = last_link.at(model, S::ONE)?;
        let mut decided = Vec::new();
        for (at_end, vertex, faces, out, station) in [
            (false, vertices[0], first_link.faces, t0.neg(), first),
            (
                true,
                *vertices.last().expect("vertices"),
                last_link.faces,
                t1,
                last_station,
            ),
        ] {
            let ctx = with_context!("the blend's end at vertex {vertex}");
            let (kind, face) = tool_end(model, vertex, faces, &out, bend).with_context(ctx)?;
            let plane = end_plane(model, station, face, &out).with_context(ctx)?;
            let index = if at_end { stations.len() - 1 } else { 0 };
            let name = if at_end { "end" } else { "start" };
            // The section's control points and vertices in the plane.
            let flat = stations[index].points.map(|p| flatten(&p, &plane));
            let flat_vertices = stations[index]
                .vertices
                .map(|v| flatten_point(&v, &plane));
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
                End::Mitre { .. } => {
                    return Err(GeopError::new("a rolling-ball blend is not mitred"))
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
        },
        ends,
        middles,
        deviation,
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

/// The joints a section curve runs between, by curve: the arc from the
/// first contact to the second, the line on to the apex, the line back.
const CURVE_JOINTS: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
/// The section control points (see [`section_points`]) each curve is made
/// of.
const CURVE_POINTS: [&[usize]; 3] = [&[0, 1, 2], &[2, 3], &[3, 0]];
/// The section control point each joint is.
const JOINT_POINT: [usize; 3] = [0, 2, 3];

/// The section's curve `c` (see [`CURVE_POINTS`]) of control points
/// `points`: rational quadratic for the arc, lines else.
fn section_curve<S: Scalar, const D: usize>(
    points: &[Vector<S, D>; 4],
    c: usize,
) -> GeopResult<NurbCurve<S, D>> {
    let control_points = CURVE_POINTS[c].iter().map(|&k| points[k]).collect();
    let knots = if c == 0 {
        vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE]
    } else {
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE]
    };
    NurbCurve::try_new(if c == 0 { 2 } else { 1 }, control_points, knots)
}

/// A wall's sides in its own parameters: the curve at the span's first
/// station, run backwards along `u = 0`; the path of its start along
/// `v = 0`; the curve at the last station along `u = 1`; the path of its
/// end, backwards along `v = 1`.
fn wall_pcurves<S: Scalar>() -> GeopResult<[NurbCurve2D<S>; 4]> {
    let p = |u: f64, v: f64| Vector2::from_array([S::from_f64(u), S::from_f64(v)]);
    Ok([
        line2(p(0.0, 1.0), p(0.0, 0.0))?,
        line2(p(0.0, 0.0), p(1.0, 0.0))?,
        line2(p(1.0, 0.0), p(1.0, 1.0))?,
        line2(p(1.0, 1.0), p(0.0, 1.0))?,
    ])
}

/// The flat cap on `cap`'s plane holding its section: the box around the
/// section in the plane, as a bilinear patch facing out of the tool, and
/// the section's curves in its parameters.
fn cap_face<S: Scalar>(cap: &Cap<S>) -> GeopResult<(NurbSurface3D<S>, [NurbCurve2D<S>; 3])> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in &cap.flat {
        for k in 0..2 {
            let x = p[k].div(p[2])?;
            lo[k] = lo[k].min(x.lower().to_f64());
            hi[k] = hi[k].max(x.upper().to_f64());
        }
    }
    // The cap spans exactly this box, which encloses the section: a NURBS
    // curve stays within the convex hull of its control points.
    let lo = [S::from_f64(lo[0]), S::from_f64(lo[1])];
    let size = [
        S::from_f64(hi[0]).sub(lo[0]).upper(),
        S::from_f64(hi[1]).sub(lo[1]).upper(),
    ];
    let [o, e1, e2] = &cap.plane;
    let corner = |i: usize, j: usize| {
        let x = if i == 1 { lo[0].add(size[0]) } else { lo[0] };
        let y = if j == 1 { lo[1].add(size[1]) } else { lo[1] };
        o.add(&e1.prod_scalar(x)).add(&e2.prod_scalar(y))
    };
    let surface = bilinear(corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1))?;
    let mut uv = [Vector3::zero(); 4];
    for (k, p) in cap.flat.iter().enumerate() {
        uv[k] = Vector3::from_array([
            p[0].sub(p[2].mul(lo[0])).div(size[0])?,
            p[1].sub(p[2].mul(lo[1])).div(size[1])?,
            p[2],
        ]);
    }
    Ok((
        surface,
        [
            section_curve(&uv, 0)?,
            section_curve(&uv, 1)?,
            section_curve(&uv, 2)?,
        ],
    ))
}

/// Builds `tool` into a solid named `N(tool)`, `namer` the blended edge's
/// (see [`crate::blend::blend`]): a vertex at each joint of every station,
/// `N(ta,st0)`, `N(tb,st0)`, `N(q,st0)`, ...; the section's curves at every
/// station, `N(fillet,st0)`, `N(b,st0)`, `N(a,st0)`; each joint's path
/// through every span, `N(ta,s0)`, ...; the walls each curve sweeps through
/// every span, the blend `N(fillet,s0)` — the span left out of a tool of
/// one, and the run-out spans `start` and `end` — and the caps of an open
/// one, `N(start)` and `N(end)`.
pub(crate) fn build_tool<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    tool: &Tool<S>,
) -> GeopResult<SolidId> {
    let joint_names = [
        format!("t{}", tool.sides[0]),
        format!("t{}", tool.sides[1]),
        "q".to_string(),
    ];
    let curve_names = ["fillet", tool.sides[1], tool.sides[0]];
    let count = tool.stations.len();
    let next = |j: usize| (j + 1) % count;
    let mut spec = BodySpec {
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: true,
    };
    let mut names = BodyNames {
        solid: Some(namer.name(&["tool"])),
        ..BodyNames::default()
    };
    let qualified = |name: &str, span: &Option<String>| match span {
        Some(s) => namer.name(&[name, s]),
        None => namer.name(&[name]),
    };

    let mut vertex = Vec::new();
    for station in &tool.stations {
        let mut ids = [0; 3];
        for k in 0..3 {
            spec.vertices.push(station.vertices[k]);
            names
                .vertices
                .push(namer.name(&[&joint_names[k], &station.name]));
            ids[k] = spec.vertices.len() - 1;
        }
        vertex.push(ids);
    }
    let mut station_edge = Vec::new();
    for (s, station) in tool.stations.iter().enumerate() {
        let mut ids = [0; 3];
        for c in 0..3 {
            let (a, b) = CURVE_JOINTS[c];
            spec.edges.push(EdgeSpec {
                curve: section_curve(&station.points, c)?,
                start: vertex[s][a],
                end: vertex[s][b],
            });
            names
                .edges
                .push(namer.name(&[curve_names[c], &station.name]));
            ids[c] = spec.edges.len() - 1;
        }
        station_edge.push(ids);
    }
    let mut lateral = Vec::new();
    for (j, span) in tool.spans.iter().enumerate() {
        let mut ids = [0; 3];
        for k in 0..3 {
            spec.edges.push(EdgeSpec {
                curve: NurbCurve::try_new(
                    span.degree,
                    span.rows.iter().map(|r| r[JOINT_POINT[k]]).collect(),
                    span.knots.clone(),
                )?,
                start: vertex[j][k],
                end: vertex[next(j)][k],
            });
            names.edges.push(qualified(&joint_names[k], &span.name));
            ids[k] = spec.edges.len() - 1;
        }
        lateral.push(ids);
    }

    let [first_back, start_side, last_forward, end_side] = wall_pcurves::<S>()?;
    for (j, span) in tool.spans.iter().enumerate() {
        for c in 0..3 {
            let (a, b) = CURVE_JOINTS[c];
            let surface = wall_surface(span, c)?;
            let on = |edge: usize, sense: Sense, pcurve: &NurbCurve2D<S>| CoedgeSpec {
                on: CoedgeOn::Edge(edge, sense),
                pcurve: pcurve.clone(),
            };
            spec.faces.push(FaceSpec {
                surface,
                outer: vec![
                    on(station_edge[j][c], Sense::Reversed, &first_back),
                    on(lateral[j][a], Sense::Forward, &start_side),
                    on(station_edge[next(j)][c], Sense::Forward, &last_forward),
                    on(lateral[j][b], Sense::Reversed, &end_side),
                ],
                holes: Vec::new(),
            });
            names.faces.push(qualified(curve_names[c], &span.name));
        }
    }

    if !tool.closed {
        for (s, forward, name) in [(0, true, "start"), (count - 1, false, "end")] {
            let Some(cap) = &tool.stations[s].cap else {
                return Err(GeopError::new(format!("the tool's {name} has no cap")));
            };
            let (surface, pcurves) = cap_face(cap)?;
            let mut outer: Vec<CoedgeSpec<S>> = (0..3)
                .map(|c| {
                    if forward {
                        CoedgeSpec {
                            on: CoedgeOn::Edge(station_edge[s][c], Sense::Forward),
                            pcurve: pcurves[c].clone(),
                        }
                    } else {
                        CoedgeSpec {
                            on: CoedgeOn::Edge(station_edge[s][c], Sense::Reversed),
                            pcurve: pcurves[c].reverse(),
                        }
                    }
                })
                .collect();
            if !forward {
                outer.reverse();
            }
            spec.faces.push(FaceSpec {
                surface,
                outer,
                holes: Vec::new(),
            });
            names.faces.push(namer.name(&[name]));
        }
    }
    spec.shells = vec![(0..spec.faces.len()).collect()];
    part.build_body(spec, names)?
        .solid
        .ok_or_else(|| GeopError::new("the blend's tool came out as no solid"))
}
