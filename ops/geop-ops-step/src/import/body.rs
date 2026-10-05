//! One body of a STEP file — a solid or a sheet — as a [`BodySpec`] the
//! kernel builds.
//!
//! A STEP B-rep and a geop one differ in three ways, and reading one is
//! mostly bridging them:
//!
//! - **Periodic surfaces.** STEP parametrizes a cylinder, cone, sphere or
//!   torus all the way round, so a face that wraps around its axis either
//!   runs along a *seam* — one edge its loop uses twice — or is bounded by
//!   loops that each go once round. A geop face lies on a NURBS patch that
//!   does not wrap: its loops close in the patch's `(u, v)`. So a wrapping
//!   face loses its seam and is cut along meridians into sectors, each on a
//!   patch of its own (see [`Builder::cut_wrapping_face`]); one going round
//!   a torus' tube — a pipe bend's — is cut along parallels instead
//!   ([`Builder::cut_tube_wrapping_face`]), and a whole torus into bands
//!   round its axis first ([`Builder::cut_whole_torus`]). A face turning
//!   about its axis more than once without going round it — a thread's
//!   flank — is cut along meridians too ([`Builder::cut_helical_face`]).
//! - **Closed edges.** A STEP edge may start and end at one vertex — a whole
//!   circle. A geop edge may not, so one is split in two.
//! - **Tolerance.** A STEP file's vertices, curves and surfaces agree only
//!   to within the file's accuracy. A geop vertex is an enclosure, so each
//!   is the union of every place the file says it is: its point, the ends
//!   of its edges' curves, and its foot points on its faces' surfaces (see
//!   `AGENTS.md` on combining enclosures of one value with `union`). An
//!   edge is widened until it reaches the midpoints of its pcurve's points
//!   on each of its faces. What lies further apart than the kernel's
//!   accuracy is rebuilt from the surfaces, which a B-rep is defined by:
//!   a vertex where its faces' surfaces meet, an edge along where they
//!   meet — and where the surfaces themselves contradict each other, the
//!   smallest faces there from their edges (see [`heal`]). The file's
//!   uncertainty bounds it: what is rebuilt lies on its surfaces to within
//!   it, and what cannot be is refused, naming the entities and saying how
//!   far. What was rebuilt, and how far it moved, is reported
//!   ([`Healing`]).
//!
//! Every surface is built as an exact NURBS patch covering its face — or,
//! for a B-spline, taken as it is — oriented so its normal points out of
//! the material, and every pcurve is fitted by projecting the edge onto it
//! (`NurbSurface::fit_pcurve`), as the kernel fits all its pcurves.

use std::collections::HashMap;

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};

use super::{
    geometry::{
        Frame, Midpoints, P3, Profile, Revolved, Scope, SurfaceDef, SurfaceKind, add, cross,
        distance, dot, norm, normalize, parameter_of, scale, sub, to_p3,
    },
    reader::{Reader, unsupported},
    structure::Item,
};

/// A body read from a file, ready to build: its description, and the name
/// of every entity in it — the arguments a step's names are made of, as
/// `["f3", "q1"]` for the second sector of the file's fourth face.
pub struct ImportedBody<S: Scalar> {
    /// What the file calls it.
    pub label: String,
    pub spec: BodySpec<S>,
    pub vertex_names: Vec<Vec<String>>,
    pub edge_names: Vec<Vec<String>>,
    pub face_names: Vec<Vec<String>>,
    /// What was rebuilt where the file disagrees with itself further than
    /// the kernel can carry (see `heal`).
    pub healed: Healing,
}

/// What [`heal`] rebuilt of a body where its file disagrees with itself
/// further than the kernel can carry, and how far it moved.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Healing {
    /// Each vertex moved to where its faces meet, and how far.
    pub vertices: Vec<(String, f64)>,
    /// Each edge rebuilt where its faces meet, and how far it moved at most.
    pub edges: Vec<(String, f64)>,
    /// Each face rebuilt from its edges.
    pub faces: Vec<String>,
    /// The file's uncertainty, which everything rebuilt meets to.
    pub uncertainty: f64,
}

impl Healing {
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() && self.edges.is_empty() && self.faces.is_empty()
    }

    /// How far anything moved, at most.
    pub fn furthest(&self) -> f64 {
        self.vertices
            .iter()
            .chain(&self.edges)
            .map(|m| m.1)
            .fold(0.0, f64::max)
    }

    /// In one sentence, for a user: how far, how much, which faces.
    pub fn summary(&self) -> String {
        let mut out = format!(
            "its faces disagree with each other by up to {:.1e} mm (the file states {:e} mm): {} vertices and {} edges were moved onto them",
            self.furthest(),
            self.uncertainty,
            self.vertices.len(),
            self.edges.len()
        );
        if !self.faces.is_empty() {
            out += &format!(
                ", and {} faces rebuilt from their edges: {}",
                self.faces.len(),
                self.faces.join(", ")
            );
        }
        out
    }

    /// Entity by entity, how far each moved.
    pub fn details(&self) -> Vec<String> {
        let line = |what: &str, moved: &[(String, f64)]| {
            let names: Vec<String> = moved
                .iter()
                .map(|(name, by)| format!("{name} by {by:.1e}"))
                .collect();
            format!("{what}: {}", names.join(", "))
        };
        let mut out = Vec::new();
        if !self.vertices.is_empty() {
            out.push(line("vertices put where their faces meet", &self.vertices));
        }
        if !self.edges.is_empty() {
            out.push(line("edges rebuilt where their faces meet", &self.edges));
        }
        if !self.faces.is_empty() {
            out.push(format!(
                "faces rebuilt from their edges, their surfaces contradicting their neighbours': {}",
                self.faces.join(", ")
            ));
        }
        out
    }
}

/// Search budget for the containment searches `fit_pcurve` falls back on.
/// Every pcurve here is pinned at both ends, so they are not run; the
/// values are the kernel's usual ones.
const MAX_NODES: usize = 5000;
const MIN_SUBDIVISION_SIZE: f64 = 1e-4;

/// Samples per edge when walking a loop around an axis: enough that two
/// consecutive ones are less than half a turn apart even on an edge going
/// all the way round.
const LOOP_SAMPLES: usize = 16;

/// Candidate axes for a sphere whose file's axis does not serve (see
/// [`Builder::sphere_axis`]).
const SPHERE_AXES: usize = 256;

struct Vertex<S: Scalar> {
    /// Every place the file says the vertex is, united (see the module
    /// docs): grows as its edges and faces are built.
    point: Vector3<S>,
    /// Where the vertex itself says it is: what its foot points on its
    /// faces are projected from, so that one face's foot point does not
    /// widen what the next one is projected from.
    origin: Vector3<S>,
    name: Vec<String>,
    /// The file's `VERTEX_POINT`, for one read from it.
    id: Option<u64>,
}

impl<S: Scalar> Vertex<S> {
    /// Its name, and the file's id of it where it has one: for messages.
    fn label(&self) -> String {
        label(&self.name, self.id)
    }
}

struct Edge<S: Scalar> {
    curve: NurbCurve3D<S>,
    start: usize,
    end: usize,
    name: Vec<String>,
    /// False once split into pieces or found to be a seam.
    alive: bool,
    /// The file's `EDGE_CURVE`, for one read from it or a piece of one.
    id: Option<u64>,
}

impl<S: Scalar> Edge<S> {
    /// Its name, and the file's id of it where it has one: for messages.
    fn label(&self) -> String {
        label(&self.name, self.id)
    }
}

/// `name`, and the file's id `#id` where there is one: `e3,p1 (#120)`.
fn label(name: &[String], id: Option<u64>) -> String {
    match id {
        Some(id) => format!("{} (#{id})", name.join(",")),
        None => name.join(","),
    }
}

/// A coedge of a loop: an edge, and whether the loop runs along it.
type Use = (usize, bool);

struct Face {
    surface: SurfaceDef,
    /// Whether the face's outward normal is the surface's normal as the
    /// file defines it.
    same_sense: bool,
    /// Loops, each running counter-clockwise about the outward normal
    /// around the face — the outer one — or clockwise around a hole.
    loops: Vec<Vec<Use>>,
    /// Which loop is the outer one, where that is known by construction.
    outer: Option<usize>,
    /// The patch, where cutting the face decided it: the angles and
    /// profile parameters of a sector.
    patch: Option<[f64; 4]>,
    shell: usize,
    name: Vec<String>,
    /// For messages: the file's face.
    id: u64,
}

impl Face {
    /// Whether the NURBS built for the surface points out of the material
    /// as it is, or has to be turned around.
    fn outward_is_natural(&self) -> bool {
        self.same_sense != self.surface.flipped
    }
}

struct Builder<'r, S: Scalar> {
    reader: Reader<'r>,
    scope: Scope,
    vertices: Vec<Vertex<S>>,
    vertex_index: HashMap<u64, usize>,
    edges: Vec<Edge<S>>,
    edge_index: HashMap<u64, usize>,
    faces: Vec<Face>,
    surfaces: HashMap<u64, SurfaceDef>,
    shells: usize,
}

fn s3<S: Scalar>(p: P3) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

fn name(prefix: &str, k: usize) -> Vec<String> {
    vec![format!("{prefix}{k}")]
}

/// The body `item` of the file, as a description to build.
pub fn read_body<S: Scalar>(reader: Reader<'_>, item: &Item) -> GeopResult<ImportedBody<S>> {
    let mut builder = Builder::<S> {
        reader,
        scope: item.scope,
        vertices: Vec::new(),
        vertex_index: HashMap::new(),
        edges: Vec::new(),
        edge_index: HashMap::new(),
        faces: Vec::new(),
        surfaces: HashMap::new(),
        shells: 0,
    };
    let instance = reader.instance(item.id)?;
    let solid = !instance.is("SHELL_BASED_SURFACE_MODEL");
    if instance.is("MANIFOLD_SOLID_BREP") {
        let args = reader.args(item.id, "MANIFOLD_SOLID_BREP")?;
        builder.shell(args.reference(1)?, false)?;
    } else if instance.is("BREP_WITH_VOIDS") {
        let args = reader.args(item.id, "BREP_WITH_VOIDS")?;
        builder.shell(args.reference(1)?, false)?;
        for void in args.references(2)? {
            let v = reader.args(void, "ORIENTED_CLOSED_SHELL")?;
            builder.shell(v.reference(2)?, !v.logical(3)?)?;
        }
    } else {
        let args = reader.args(item.id, "SHELL_BASED_SURFACE_MODEL")?;
        for shell in args.references(1)? {
            let s = reader.instance(shell)?;
            if s.is("ORIENTED_OPEN_SHELL") || s.is("ORIENTED_CLOSED_SHELL") {
                let (_, o) =
                    reader.args_of_any(shell, &["ORIENTED_OPEN_SHELL", "ORIENTED_CLOSED_SHELL"])?;
                builder.shell(o.reference(2)?, !o.logical(3)?)?;
            } else {
                builder.shell(shell, false)?;
            }
        }
    }
    builder.normalize()?;
    builder.spec(solid, item.label.clone())
}

impl<S: Scalar> Builder<'_, S> {
    fn shell(&mut self, id: u64, reversed: bool) -> GeopResult<()> {
        let (_, args) = self
            .reader
            .args_of_any(id, &["CLOSED_SHELL", "OPEN_SHELL"])?;
        let shell = self.shells;
        self.shells += 1;
        for face in args.references(1)? {
            self.face(face, shell, reversed)
                .map_err(|e| e.with_context(format!("reading the face #{face}")))?;
        }
        Ok(())
    }

    fn vertex(&mut self, id: u64) -> GeopResult<usize> {
        if let Some(&v) = self.vertex_index.get(&id) {
            return Ok(v);
        }
        let args = self.reader.args(id, "VERTEX_POINT")?;
        let point = self.reader.point(&self.scope, args.reference(1)?)?;
        let v = self.vertices.len();
        self.vertices.push(Vertex {
            point: s3(point),
            origin: s3(point),
            name: name("v", self.vertex_index.len()),
            id: Some(id),
        });
        self.vertex_index.insert(id, v);
        Ok(v)
    }

    fn edge(&mut self, id: u64) -> GeopResult<usize> {
        if let Some(&e) = self.edge_index.get(&id) {
            return Ok(e);
        }
        let ctx = |e: GeopError| e.with_context(format!("reading the edge #{id}"));
        let args = self.reader.args(id, "EDGE_CURVE").map_err(ctx)?;
        let start = self.vertex(args.reference(1)?).map_err(ctx)?;
        let end = self.vertex(args.reference(2)?).map_err(ctx)?;
        let (curve, reversed) = self
            .reader
            .curve(&self.scope, args.reference(3)?)
            .map_err(ctx)?;
        let along = args.logical(4).map_err(ctx)? != reversed;
        let (a, b) = (
            to_p3(&self.vertices[start].point),
            to_p3(&self.vertices[end].point),
        );
        let closed = start == end;
        let curve = if along {
            curve.edge(a, b, closed, self.scope.uncertainty)
        } else {
            curve
                .edge(b, a, closed, self.scope.uncertainty)
                .map(|c| c.reverse())
        }
        .map_err(ctx)?;
        let e = self.edges.len();
        self.edges.push(Edge {
            curve,
            start,
            end,
            name: name("e", self.edge_index.len()),
            alive: true,
            id: Some(id),
        });
        self.edge_index.insert(id, e);
        Ok(e)
    }

    /// The edge an `ORIENTED_EDGE` runs along, and whether it runs along
    /// it forwards.
    fn oriented_edge(&mut self, id: u64) -> GeopResult<Use> {
        let instance = self.reader.instance(id)?;
        if instance.is("EDGE_CURVE") {
            return Ok((self.edge(id)?, true));
        }
        let args = self.reader.args(id, "ORIENTED_EDGE")?;
        let (edge, forward) = self.oriented_edge(args.reference(3)?)?;
        Ok((edge, forward == args.logical(4)?))
    }

    fn face(&mut self, id: u64, shell: usize, reversed: bool) -> GeopResult<()> {
        let (_, args) = self
            .reader
            .args_of_any(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let surface_id = args.reference(2)?;
        let surface = match self.surfaces.get(&surface_id) {
            Some(s) => s.clone(),
            None => {
                let s = self.reader.surface(&self.scope, surface_id)?;
                self.surfaces.insert(surface_id, s.clone());
                s
            }
        };
        let mut loops = Vec::new();
        for bound in args.references(1)? {
            let (_, b) = self
                .reader
                .args_of_any(bound, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
            let lp = b.reference(1)?;
            let forward = b.logical(2)? != reversed;
            let lp_instance = self.reader.instance(lp)?;
            if lp_instance.is("VERTEX_LOOP") {
                // A loop of one vertex marks a pole or an apex, which the
                // face's patch has anyway.
                continue;
            }
            if !lp_instance.is("EDGE_LOOP") {
                return Err(unsupported(
                    lp,
                    lp_instance,
                    "a face bound that is not a loop of edges",
                ));
            }
            let mut coedges = Vec::new();
            for oriented in self.reader.args(lp, "EDGE_LOOP")?.references(1)? {
                coedges.push(self.oriented_edge(oriented)?);
            }
            if !forward {
                coedges = coedges.into_iter().rev().map(|(e, f)| (e, !f)).collect();
            }
            if coedges.is_empty() {
                continue;
            }
            loops.push(coedges);
        }
        let surface = match surface.revolved() {
            Some(revolved) if matches!(revolved.profile, Profile::Sphere { .. }) => SurfaceDef {
                kind: SurfaceKind::Revolved(self.sphere_axis(revolved, &loops)?),
                ..surface
            },
            _ => surface,
        };
        // A face on a circle crossing its axis lies on one of its two
        // sheets: the one past the axis is a surface of its own. Decided at
        // the vertex that tells them apart best — not one at a pole.
        let surface = match surface.revolved() {
            Some(revolved) => {
                let mut best: Option<f64> = None;
                for &u in loops.iter().flatten() {
                    let p = to_p3(&self.vertices[self.ends(u).0].point);
                    if let Some(m) = revolved.past_axis(p)
                        && best.is_none_or(|b| m.abs() > b.abs())
                    {
                        best = Some(m);
                    }
                }
                match best {
                    Some(m) if m > 0.0 => SurfaceDef {
                        kind: SurfaceKind::Revolved(revolved.mirrored()),
                        flipped: !surface.flipped,
                    },
                    _ => surface,
                }
            }
            None => surface,
        };
        let same_sense = args.logical(3)? != reversed;
        let index = self.faces.len();
        self.faces.push(Face {
            surface,
            same_sense,
            loops,
            outer: None,
            patch: None,
            shell,
            name: name("f", index),
            id,
        });
        Ok(())
    }

    /// The axis a face on the sphere `sphere` is built about. Which axis a
    /// sphere has is a free choice: the file's, unless one of its poles
    /// lies on an edge of the face's `loops` away from the edge's ends. A
    /// loop running through a pole has no angle about the axis there, and
    /// its angle turns by half a turn across it, which neither the
    /// windings nor the cuts along meridians can follow. Then the axis is
    /// chosen whose poles lie furthest from the loops — inside the face,
    /// where it is cut into sectors round it, or outside it.
    fn sphere_axis(&self, sphere: &Revolved, loops: &[Vec<Use>]) -> GeopResult<Revolved> {
        let frame = &sphere.frame;
        let poles = [frame.point([0.0, 0.0, 1.0]), frame.point([0.0, 0.0, -1.0])];
        let Profile::Sphere { radius } = sphere.profile else {
            unreachable!("a sphere's profile");
        };
        let poles = poles.map(|p| add(frame.origin, scale(sub(p, frame.origin), radius)));
        let mut through = false;
        for &(e, _) in loops.iter().flatten() {
            let edge = &self.edges[e];
            let ends = [edge.start, edge.end].map(|v| to_p3(&self.vertices[v].origin));
            for pole in poles {
                if ends
                    .iter()
                    .any(|&end| distance(end, pole) <= sphere.on_axis)
                {
                    continue;
                }
                let t = parameter_of(&edge.curve, pole)?;
                if distance(to_p3(&edge.curve.evaluate(t)?), pole) <= sphere.on_axis {
                    through = true;
                }
            }
        }
        if !through {
            return Ok(sphere.clone());
        }
        let mut points = Vec::new();
        for &u in loops.iter().flatten() {
            points.extend(
                self.samples(u, LOOP_SAMPLES)?
                    .into_iter()
                    .filter_map(|(_, p)| normalize(sub(p, frame.origin))),
            );
        }
        // Candidates spread evenly over a half sphere (both ends of an
        // axis are poles): each scored by how near its poles come to a
        // point of the loops, as the cosine of the angle between.
        let nearest = |axis: P3| {
            points
                .iter()
                .map(|p| dot(*p, axis).abs())
                .fold(0.0, f64::max)
        };
        let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
        let axis = (0..SPHERE_AXES)
            .map(|k| {
                let z = (k as f64 + 0.5) / SPHERE_AXES as f64;
                let r = (1.0 - z * z).sqrt();
                let (s, c) = (golden * k as f64).sin_cos();
                [r * c, r * s, z]
            })
            .min_by(|a, b| nearest(*a).total_cmp(&nearest(*b)))
            .expect("candidates");
        Ok(Revolved {
            frame: Frame::new(frame.origin, axis, frame.x)?,
            ..sphere.clone()
        })
    }

    /// The vertex a coedge starts at, and the one it ends at.
    fn ends(&self, (e, forward): Use) -> (usize, usize) {
        let edge = &self.edges[e];
        if forward {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        }
    }

    /// Points along a coedge, as it runs, with the edge parameter of each:
    /// `n + 1` of them, ends included.
    fn samples(&self, (e, forward): Use, n: usize) -> GeopResult<Vec<(f64, P3)>> {
        let curve = &self.edges[e].curve;
        let (lo, hi) = curve.domain();
        let (lo, hi) = (lo.to_f64(), hi.to_f64());
        let mut out = Vec::with_capacity(n + 1);
        for i in 0..=n {
            let f = i as f64 / n as f64;
            let f = if forward { f } else { 1.0 - f };
            let t = lo + (hi - lo) * f;
            out.push((t, super::geometry::point_at(curve, t)?));
        }
        Ok(out)
    }

    /// Points along a loop, each coedge's last one left out as the next
    /// one's first.
    fn loop_points(&self, lp: &[Use]) -> GeopResult<Vec<P3>> {
        let mut points = Vec::new();
        for &u in lp {
            let samples = self.samples(u, LOOP_SAMPLES)?;
            points.extend(samples[..LOOP_SAMPLES].iter().map(|(_, p)| *p));
        }
        Ok(points)
    }

    /// Splits `edge` at the parameters `ts`, strictly inside its domain
    /// and increasing, into pieces, with a new vertex at each — in every
    /// loop of every face. Returns the new vertices.
    fn split_edge(&mut self, e: usize, ts: &[S]) -> GeopResult<Vec<usize>> {
        let edge = &self.edges[e];
        let (start, end, base, id) = (edge.start, edge.end, edge.name.clone(), edge.id);
        let curve = edge.curve.clone();
        let (lo, hi) = curve.domain();
        let mut bounds = vec![lo];
        bounds.extend_from_slice(ts);
        bounds.push(hi);
        let mut new_vertices = Vec::new();
        for (k, &t) in ts.iter().enumerate() {
            let v = self.vertices.len();
            let mut name = base.clone();
            name.push(format!("c{k}"));
            self.vertices.push(Vertex {
                point: curve.evaluate(t)?,
                origin: curve.evaluate(t)?,
                name,
                id: None,
            });
            new_vertices.push(v);
        }
        let mut chain = vec![start];
        chain.extend(&new_vertices);
        chain.push(end);
        let mut pieces = Vec::new();
        for k in 0..bounds.len() - 1 {
            let piece = curve.sub_curve(bounds[k], bounds[k + 1])?;
            let mut name = base.clone();
            name.push(format!("p{k}"));
            pieces.push(self.edges.len());
            self.edges.push(Edge {
                curve: piece,
                start: chain[k],
                end: chain[k + 1],
                name,
                alive: true,
                id,
            });
        }
        self.edges[e].alive = false;
        for face in &mut self.faces {
            for lp in &mut face.loops {
                let mut out = Vec::with_capacity(lp.len() + pieces.len());
                for &(edge, forward) in lp.iter() {
                    if edge != e {
                        out.push((edge, forward));
                    } else if forward {
                        out.extend(pieces.iter().map(|&p| (p, true)));
                    } else {
                        out.extend(pieces.iter().rev().map(|&p| (p, false)));
                    }
                }
                *lp = out;
            }
        }
        Ok(new_vertices)
    }

    /// Brings the faces read into the shape geop needs: wrapping faces cut
    /// into sectors, closed edges split.
    fn normalize(&mut self) -> GeopResult<()> {
        let mut f = 0;
        while f < self.faces.len() {
            let face = &self.faces[f];
            let id = face.id;
            let revolved = face.surface.revolved().cloned();
            let had_seam = self
                .drop_doubled_edges(f)
                .map_err(|e| e.with_context(format!("taking the seams out of the face #{id}")))?;
            let wraps = match &revolved {
                Some(revolved) => {
                    had_seam
                        || self.faces[f].loops.is_empty()
                        || self.windings(f, revolved)?.iter().any(|&w| w != 0)
                }
                None => false,
            };
            if let (true, Some(revolved)) = (wraps, &revolved) {
                let round_axis = self.windings(f, revolved)?.iter().any(|&w| w != 0);
                if revolved.v_is_angle() && !round_axis {
                    if self.tube_windings(f, revolved)?.iter().any(|&w| w != 0) {
                        self.cut_tube_wrapping_face(f, revolved).map_err(|e| {
                            e.with_context(format!(
                                "cutting the face #{id}, which goes round its torus' tube, into pieces along parallels"
                            ))
                        })?;
                    } else {
                        self.cut_whole_torus(f, revolved).map_err(|e| {
                            e.with_context(format!(
                                "cutting the face #{id}, a whole torus, into bands round its axis"
                            ))
                        })?;
                    }
                } else {
                    self.cut_wrapping_face(f).map_err(|e| {
                        e.with_context(format!(
                            "cutting the face #{id}, which wraps around its axis, into sectors"
                        ))
                    })?;
                }
                // Its pieces were pushed at the end; it is gone.
                continue;
            }
            if let Some(revolved) = &revolved
                && self.turns_more_than_once(f, revolved)?
            {
                self.cut_helical_face(f, revolved).map_err(|e| {
                    e.with_context(format!(
                        "cutting the face #{id}, which turns about its axis more than once without going round it, along meridians"
                    ))
                })?;
                continue;
            }
            f += 1;
        }
        // Closed edges left: split each in two.
        for e in 0..self.edges.len() {
            let edge = &self.edges[e];
            if edge.alive && edge.start == edge.end {
                let (lo, hi) = edge.curve.domain();
                let mid = lo.add(hi).div(S::TWO)?.sharpen();
                self.split_edge(e, &[mid])?;
            }
        }
        Ok(())
    }

    /// Takes out of face `f` every edge it runs along twice, once each
    /// way, which no other face uses: a seam, where a periodic surface's
    /// parametrization wraps, or a bridge joining a hole to the loop around
    /// it. Either runs within the face, so it bounds nothing; taking it out
    /// splits its loop in two — the part between its two uses and the
    /// rest. Says whether there was any.
    fn drop_doubled_edges(&mut self, f: usize) -> GeopResult<bool> {
        let mut count: HashMap<usize, usize> = HashMap::new();
        for lp in &self.faces[f].loops {
            for &(e, _) in lp {
                *count.entry(e).or_default() += 1;
            }
        }
        let mut doubled: Vec<usize> = count
            .into_iter()
            .filter(|&(_, n)| n == 2)
            .map(|(e, _)| e)
            .collect();
        doubled.sort();
        for &e in &doubled {
            if self
                .faces
                .iter()
                .enumerate()
                .any(|(g, face)| g != f && face.loops.iter().flatten().any(|&(x, _)| x == e))
            {
                return Err(GeopError::new(format!(
                    "the edge {} it runs along twice is used by another face too",
                    self.edges[e].label()
                )));
            }
            let loops = std::mem::take(&mut self.faces[f].loops);
            let mut out = Vec::new();
            for lp in loops {
                let uses: Vec<usize> = (0..lp.len()).filter(|&k| lp[k].0 == e).collect();
                match uses.as_slice() {
                    [] => out.push(lp),
                    &[i, j] => {
                        if lp[i].1 == lp[j].1 {
                            return Err(GeopError::new(format!(
                                "it runs along the edge {} twice the same way",
                                self.edges[e].label()
                            )));
                        }
                        let inner: Vec<Use> = lp[i + 1..j].to_vec();
                        let outer: Vec<Use> = lp[j + 1..].iter().chain(&lp[..i]).copied().collect();
                        out.extend([inner, outer].into_iter().filter(|l| !l.is_empty()));
                    }
                    // A loop of the closed edge alone: what is left of a
                    // whole torus' loop round both seams once one is out,
                    // twice, once each way. Neither bounds anything.
                    [_] if lp.len() == 1 => {}
                    _ => {
                        return Err(GeopError::new(format!(
                            "the edge {} is used twice, by different loops of it",
                            self.edges[e].label()
                        )));
                    }
                }
            }
            self.faces[f].loops = out;
            self.edges[e].alive = false;
        }
        for lp in &self.faces[f].loops {
            let (start, _) = self.ends(lp[0]);
            let (_, end) = self.ends(*lp.last().expect("not empty"));
            if start != end {
                return Err(GeopError::new(
                    "taking them out leaves a loop that does not close",
                ));
            }
        }
        Ok(!doubled.is_empty())
    }

    /// How often each loop of face `f` goes round the tube of `revolved`, a
    /// torus, the way its profile parameter runs.
    fn tube_windings(&self, f: usize, revolved: &Revolved) -> GeopResult<Vec<i64>> {
        self.faces[f]
            .loops
            .iter()
            .map(|lp| {
                Ok(winding(&unwrap_tube(
                    revolved,
                    &self.loop_points(lp)?,
                    true,
                )))
            })
            .collect()
    }

    /// How often each loop of face `f` goes round the axis of `revolved`,
    /// counter-clockwise about it.
    fn windings(&self, f: usize, revolved: &Revolved) -> GeopResult<Vec<i64>> {
        self.faces[f]
            .loops
            .iter()
            .map(|lp| {
                let ccw = self.faces[f].outward_is_natural();
                Ok(winding(&unwrap(
                    revolved,
                    &self.loop_points(lp)?,
                    true,
                    ccw,
                )))
            })
            .collect()
    }

    /// Cuts the face `f`, which wraps around its axis, into sectors along
    /// meridians, each on a patch that does not wrap.
    ///
    /// Without its seams, the face is a band around the axis between a
    /// bottom and a top, each a loop going once round — a *ring* — or a
    /// pole, with any holes in between. Where the cuts go is a free
    /// choice, made where nothing is: clear of every vertex and hole, as
    /// far from them as can be.
    fn cut_wrapping_face(&mut self, f: usize) -> GeopResult<()> {
        let revolved = self.faces[f]
            .surface
            .revolved()
            .expect("only a surface of revolution wraps")
            .clone();
        let outward_natural = self.faces[f].outward_is_natural();
        let mut rings: Vec<usize> = Vec::new();
        let mut bottom = None;
        let mut top = None;
        let mut holes = Vec::new();
        // Each loop's winding and mean profile parameter, for messages.
        let mut summary = Vec::new();
        for (k, lp) in self.faces[f].loops.iter().enumerate() {
            let points = self.loop_points(lp)?;
            let angles = unwrap(&revolved, &points, true, outward_natural);
            let mean_v =
                points.iter().map(|&p| revolved.chart(p).1).sum::<f64>() / points.len() as f64;
            summary.push((winding(&angles), mean_v, lp.len()));
            match winding(&angles) {
                0 => holes.push(k),
                w @ (1 | -1) => {
                    // The face lies to the left of its loops, about its
                    // outward normal: above a ring running counter-clockwise
                    // about the natural one.
                    let above = (w > 0) == outward_natural;
                    let slot = if above { &mut bottom } else { &mut top };
                    if slot.replace(k).is_some() {
                        return Err(GeopError::new(format!(
                            "it has more than one loop round its axis with the face {} it",
                            if above { "above" } else { "below" }
                        )));
                    }
                    rings.push(k);
                }
                w => {
                    return Err(GeopError::new(format!(
                        "a loop of it goes round its axis {w} times"
                    )));
                }
            }
        }
        // A torus' profile parameter is an angle too: read it from where
        // the face is not, so it runs on across the face. The face lies
        // above its bottom ring, up to its top one: it is not between the
        // top ring and the bottom one, going on round the tube.
        let v_start = if revolved.v_is_angle() {
            let (Some(b), Some(t)) = (bottom, top) else {
                return Err(GeopError::new(format!(
                    "it goes round its torus' axis without a loop round the axis on either side of it (its loops' windings, mean profile parameters and lengths: {summary:?})"
                )));
            };
            let mean = |k: usize| -> GeopResult<f64> {
                let v = unwrap_tube(
                    &revolved,
                    &self.loop_points(&self.faces[f].loops[k])?,
                    false,
                );
                Ok(v.iter().sum::<f64>() / v.len().max(1) as f64)
            };
            let (b, t) = (mean(b)?, mean(t)?);
            t + (b - t).rem_euclid(std::f64::consts::TAU) / 2.0
        } else {
            f64::NEG_INFINITY
        };
        let v_of = |p: P3| {
            let v = revolved.chart(p).1;
            if v_start.is_finite() {
                v_start + (v - v_start).rem_euclid(std::f64::consts::TAU)
            } else {
                v
            }
        };
        let ring_v = |builder: &Self, k: usize| -> GeopResult<f64> {
            let points = builder.loop_points(&builder.faces[f].loops[k])?;
            Ok(points.iter().map(|&p| v_of(p)).sum::<f64>() / points.len() as f64)
        };
        let bottom_v = bottom.map(|k| ring_v(self, k)).transpose()?;
        let top_v = top.map(|k| ring_v(self, k)).transpose()?;
        let poles = revolved.poles();
        // A side with no ring is closed off by the pole on that side.
        let bottom_pole = match bottom {
            Some(_) => None,
            None => Some(
                poles
                    .iter()
                    .copied()
                    .filter(|&p| top_v.is_none_or(|t| p < t))
                    // The pole nearest below the top, where there is one.
                    .reduce(if top_v.is_some() { f64::max } else { f64::min })
                    .ok_or_else(|| GeopError::new(format!("it has nothing closing it off below: no loop round its axis and no pole (its loops' windings, mean profile parameters and lengths: {summary:?}, the face's outward normal natural: {outward_natural})")))?,
            ),
        };
        let top_pole = match top {
            Some(_) => None,
            None => Some(
                poles
                    .iter()
                    .copied()
                    // The pole nearest above the bottom.
                    .filter(|&p| bottom_v.or(bottom_pole).is_none_or(|b| p > b))
                    .reduce(f64::min)
                    .ok_or_else(|| GeopError::new(format!("it has nothing closing it off above: no loop round its axis and no pole (its loops' windings, mean profile parameters and lengths: {summary:?}, the face's outward normal natural: {outward_natural})")))?,
            ),
        };
        if let (Some(b), Some(t)) = (bottom_v.or(bottom_pole), top_v.or(top_pole))
            && b >= t
        {
            return Err(GeopError::new(format!(
                "its bottom {b} is not below its top {t} (its loops' windings, mean profile parameters and lengths: {summary:?})"
            )));
        }

        // Where not to cut: at any vertex, or through a hole.
        let mut blocked: Vec<(f64, f64)> = Vec::new();
        for (k, lp) in self.faces[f].loops.iter().enumerate() {
            let points = self.loop_points(lp)?;
            if holes.contains(&k) {
                let angles = unwrap(&revolved, &points, false, outward_natural);
                let lo = angles.iter().copied().fold(f64::INFINITY, f64::min);
                let hi = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                blocked.push((lo, hi));
            }
            for &u in lp {
                let (start, _) = self.ends(u);
                if let (Some(a), _) = revolved.chart(to_p3(&self.vertices[start].point)) {
                    blocked.push((a, a));
                }
            }
        }
        let cuts = choose_cuts(&blocked);

        let ring_vertices = self.split_rings(f, &rings, &cuts, &|p| revolved.chart(p).0)?;

        // A meridian along each cut, from bottom to top.
        let point_at = |builder: &Self,
                        side: Option<usize>,
                        pole: Option<f64>,
                        c: usize|
         -> (P3, f64, Option<usize>) {
            match side {
                Some(ring) => {
                    let v = ring_vertices[&ring][c];
                    let p = to_p3(&builder.vertices[v].point);
                    (p, v_of(p), Some(v))
                }
                None => {
                    let pole = pole.expect("a side without a ring has a pole");
                    (revolved.profile_point(pole), pole, None)
                }
            }
        };
        let pole_vertex = |builder: &mut Self, pole: Option<f64>, which: &str| -> Option<usize> {
            pole.map(|v| {
                let p = revolved.profile_point(v);
                // A vertex of the file there already, or a new one.
                let existing = builder
                    .vertices
                    .iter()
                    .position(|x| distance(to_p3(&x.point), p) <= builder.scope.uncertainty);
                existing.unwrap_or_else(|| {
                    builder.vertices.push(Vertex {
                        point: s3(p),
                        origin: s3(p),
                        name: vec![builder.faces[f].name[0].clone(), which.to_string()],
                        id: None,
                    });
                    builder.vertices.len() - 1
                })
            })
        };
        let bottom_pole_vertex = pole_vertex(self, bottom_pole, "south");
        let top_pole_vertex = pole_vertex(self, top_pole, "north");
        let mut meridians = Vec::new();
        for (c, &cut) in cuts.iter().enumerate() {
            let (pb, vb, b) = point_at(self, bottom, bottom_pole, c);
            let (pt, vt, t) = point_at(self, top, top_pole, c);
            let start = b.or(bottom_pole_vertex).expect("a bottom");
            let end = t.or(top_pole_vertex).expect("a top");
            let curve = self.meridian(
                &revolved,
                cut,
                (pb, vb, bottom_pole.is_some()),
                (pt, vt, top_pole.is_some()),
            )?;
            let mut name = self.faces[f].name.clone();
            name.push(format!("m{c}"));
            meridians.push(self.edges.len());
            self.edges.push(Edge {
                curve,
                start,
                end,
                name,
                alive: true,
                id: None,
            });
        }

        // The sectors between consecutive cuts.
        let n = cuts.len();
        let face_loops = self.faces[f].loops.clone();
        let ring_run = |builder: &Self,
                        ring: usize,
                        from: usize,
                        to: usize,
                        up: bool|
         -> GeopResult<Vec<Use>> {
            // The ring's coedges from the vertex at cut `from` to the one at
            // cut `to`, running counter-clockwise about the natural normal
            // (`up`) or clockwise.
            let lp = &face_loops[ring];
            let angles = unwrap(&revolved, &builder.loop_points(lp)?, true, outward_natural);
            let along = (winding(&angles) > 0) == up;
            builder.run_between(
                lp,
                ring_vertices[&ring][from],
                ring_vertices[&ring][to],
                along,
            )
        };
        let face = &self.faces[f];
        let (surface, same_sense, shell, base_name) = (
            face.surface.clone(),
            face.same_sense,
            face.shell,
            face.name.clone(),
        );
        // Each hole goes to the sector it lies in.
        let mut hole_sector = Vec::new();
        for &h in &holes {
            let points = self.loop_points(&face_loops[h])?;
            let a = unwrap(&revolved, &points, false, outward_natural);
            let mid = a.iter().sum::<f64>() / a.len() as f64;
            let sector = (0..n)
                .find(|&s| {
                    let lo = cuts[s];
                    let hi = if s + 1 < n {
                        cuts[s + 1]
                    } else {
                        cuts[0] + std::f64::consts::TAU
                    };
                    lo + (mid - lo).rem_euclid(std::f64::consts::TAU) < hi
                })
                .expect("the sectors cover every angle");
            hole_sector.push((h, sector));
        }
        let v_range_of = |builder: &Self, loops: &[usize]| -> GeopResult<(f64, f64)> {
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for &k in loops {
                for p in builder.loop_points(&face_loops[k])? {
                    let v = v_of(p);
                    lo = lo.min(v);
                    hi = hi.max(v);
                }
            }
            Ok((lo, hi))
        };
        for s in 0..n {
            let next = (s + 1) % n;
            let lo = cuts[s];
            let hi = if s + 1 < n {
                cuts[s + 1]
            } else {
                cuts[0] + std::f64::consts::TAU
            };
            let mut lp: Vec<Use> = Vec::new();
            if let Some(ring) = bottom {
                lp.extend(ring_run(self, ring, s, next, true)?);
            }
            lp.push((meridians[next], true));
            if let Some(ring) = top {
                lp.extend(ring_run(self, ring, next, s, false)?);
            }
            lp.push((meridians[s], false));
            if !outward_natural {
                lp = lp.into_iter().rev().map(|(e, fw)| (e, !fw)).collect();
            }
            let mut loops = vec![lp];
            let mut members: Vec<usize> = bottom.into_iter().chain(top).collect();
            for &(h, sector) in &hole_sector {
                if sector == s {
                    loops.push(face_loops[h].clone());
                    members.push(h);
                }
            }
            let (v_lo, v_hi) = v_range_of(self, &members)?;
            let v0 = bottom_pole.unwrap_or(v_lo);
            let v1 = top_pole.unwrap_or(v_hi);
            // Rings a whole turn of the tube apart are one circle: the face
            // goes round the tube as well as the axis — a whole torus.
            // (Samples a hundredth of a turn short of one still are.)
            if revolved.v_is_angle() && v1 - v0 >= 0.99 * std::f64::consts::TAU {
                return Err(GeopError::new(
                    "it goes all the way round its torus' tube as well as its axis, as a whole torus does: not supported",
                ));
            }
            let (v0, v1) = widen_v(&revolved, v0, v1, bottom_pole.is_some(), top_pole.is_some());
            let mut name = base_name.clone();
            name.push(format!("q{s}"));
            self.faces.push(Face {
                surface: surface.clone(),
                same_sense,
                loops,
                outer: Some(0),
                patch: Some([lo, hi, v0, v1]),
                shell,
                name,
                id: self.faces[f].id,
            });
        }
        self.faces.remove(f);
        Ok(())
    }

    /// Cuts the face `f`, which goes all the way round the tube of its
    /// torus `revolved` but not round its axis — a pipe bend's — into
    /// pieces along parallels, each on a patch that does not wrap.
    ///
    /// Without its seam, the face is a sleeve round the tube between two
    /// rings, each going once round the tube, with any holes in between.
    /// Where the parallels go is a free choice, made as the meridians of
    /// [`Builder::cut_wrapping_face`] are: clear of every vertex and hole.
    fn cut_tube_wrapping_face(&mut self, f: usize, revolved: &Revolved) -> GeopResult<()> {
        use std::f64::consts::TAU;
        let outward_natural = self.faces[f].outward_is_natural();
        let face_loops = self.faces[f].loops.clone();
        let tube = |p: P3| revolved.chart(p).1;
        let mut left = None;
        let mut right = None;
        let mut holes = Vec::new();
        let mut windings = Vec::new();
        for (k, lp) in face_loops.iter().enumerate() {
            let w = winding(&unwrap_tube(revolved, &self.loop_points(lp)?, true));
            windings.push(w);
            match w {
                0 => holes.push(k),
                1 | -1 => {
                    // The face lies to the left of its loops, about its
                    // outward normal: at larger angles about the axis than
                    // a ring running backwards round the tube about the
                    // natural one.
                    let is_left = (w < 0) == outward_natural;
                    let slot = if is_left { &mut left } else { &mut right };
                    if slot.replace(k).is_some() {
                        return Err(GeopError::new(format!(
                            "it has more than one loop round its tube with the face at {} angles about the axis (the loops' windings round the tube: {windings:?})",
                            if is_left { "larger" } else { "smaller" }
                        )));
                    }
                }
                w => {
                    return Err(GeopError::new(format!(
                        "a loop of it goes round its tube {w} times"
                    )));
                }
            }
        }
        let (Some(left), Some(right)) = (left, right) else {
            return Err(GeopError::new(format!(
                "it is not bounded by two loops round its tube, one at either end (the loops' windings round the tube: {windings:?}, the face's outward normal natural: {outward_natural})"
            )));
        };
        // The face runs about the axis from its left ring to its right one,
        // not all the way round: angles are read from the middle of the
        // gap between the right ring and the left one.
        let mean_angle = |builder: &Self, k: usize| -> GeopResult<f64> {
            let angles = unwrap(
                revolved,
                &builder.loop_points(&face_loops[k])?,
                false,
                outward_natural,
            );
            Ok(angles.iter().sum::<f64>() / angles.len().max(1) as f64)
        };
        let (a_left, a_right) = (mean_angle(self, left)?, mean_angle(self, right)?);
        let u_start = a_right + (a_left - a_right).rem_euclid(TAU) / 2.0;
        let u_of = |p: P3| -> GeopResult<f64> {
            let a = revolved
                .chart(p)
                .0
                .ok_or_else(|| GeopError::new(format!("a point of it, {p:?}, is on its axis")))?;
            Ok(u_start + (a - u_start).rem_euclid(TAU))
        };

        // Where not to cut: at any vertex, or through a hole.
        let mut blocked: Vec<(f64, f64)> = Vec::new();
        for (k, lp) in face_loops.iter().enumerate() {
            if holes.contains(&k) {
                let angles = unwrap_tube(revolved, &self.loop_points(lp)?, false);
                let lo = angles.iter().copied().fold(f64::INFINITY, f64::min);
                let hi = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                blocked.push((lo, hi));
            }
            for &u in lp {
                let a = tube(to_p3(&self.vertices[self.ends(u).0].point));
                blocked.push((a, a));
            }
        }
        let cuts = choose_cuts(&blocked);
        let ring_vertices = self.split_rings(f, &[left, right], &cuts, &|p| Some(tube(p)))?;
        // The rings, split: their coedges now run through the cut vertices.
        let face_loops = self.faces[f].loops.clone();

        // A parallel along each cut, from the left ring to the right one.
        let mut parallels = Vec::new();
        for (c, &cut) in cuts.iter().enumerate() {
            let (a, b) = (ring_vertices[&left][c], ring_vertices[&right][c]);
            let from = u_of(to_p3(&self.vertices[a].point))?;
            let to = u_of(to_p3(&self.vertices[b].point))?;
            if from >= to {
                return Err(GeopError::new(format!(
                    "its rings round the tube cross at the angle {cut} round it: the left one at the angle {from} about the axis, the right one at {to}"
                )));
            }
            let curve = revolved.parallel::<S>(cut, from, to)?;
            let mut name = self.faces[f].name.clone();
            name.push(format!("m{c}"));
            parallels.push(self.edges.len());
            self.edges.push(Edge {
                curve,
                start: a,
                end: b,
                name,
                alive: true,
                id: None,
            });
        }

        // The pieces between consecutive parallels, each running
        // counter-clockwise in angle about the axis and round the tube:
        // along the lower parallel, up the right ring, back along the
        // upper parallel, down the left ring.
        let n = cuts.len();
        let upper = |s: usize| {
            if s + 1 < n {
                cuts[s + 1]
            } else {
                cuts[0] + TAU
            }
        };
        let mut hole_piece = Vec::new();
        for &h in &holes {
            let a = unwrap_tube(revolved, &self.loop_points(&face_loops[h])?, false);
            let mid = a.iter().sum::<f64>() / a.len() as f64;
            let piece = (0..n)
                .find(|&s| cuts[s] + (mid - cuts[s]).rem_euclid(TAU) < upper(s))
                .expect("the pieces cover every angle");
            hole_piece.push((h, piece));
        }
        let (w_left, w_right) = (windings[left], windings[right]);
        let face = &self.faces[f];
        let (surface, same_sense, shell, base_name, id) = (
            face.surface.clone(),
            face.same_sense,
            face.shell,
            face.name.clone(),
            face.id,
        );
        for s in 0..n {
            let next = (s + 1) % n;
            let mut lp = vec![(parallels[s], true)];
            lp.extend(self.run_between(
                &face_loops[right],
                ring_vertices[&right][s],
                ring_vertices[&right][next],
                w_right > 0,
            )?);
            lp.push((parallels[next], false));
            lp.extend(self.run_between(
                &face_loops[left],
                ring_vertices[&left][next],
                ring_vertices[&left][s],
                w_left < 0,
            )?);
            let mut loops = vec![lp];
            for &(h, piece) in &hole_piece {
                if piece == s {
                    loops.push(face_loops[h].clone());
                }
            }
            // The angles about the axis the piece spans, a little widened.
            let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
            for lp in &loops {
                for p in self.loop_points(lp)? {
                    let u = u_of(p)?;
                    lo = lo.min(u);
                    hi = hi.max(u);
                }
            }
            let span = hi - lo;
            let margin = (0.02 * span + 1e-3).min((TAU - span) / 4.0);
            if !outward_natural {
                loops[0] = loops[0].iter().rev().map(|&(e, fw)| (e, !fw)).collect();
            }
            let mut name = base_name.clone();
            name.push(format!("q{s}"));
            self.faces.push(Face {
                surface: surface.clone(),
                same_sense,
                loops,
                outer: Some(0),
                patch: Some([lo - margin, hi + margin, cuts[s], upper(s)]),
                shell,
                name,
                id,
            });
        }
        self.faces.remove(f);
        Ok(())
    }

    /// Cuts the face `f`, a whole torus — going round both its tube and
    /// its axis, with no loops but holes — along parallels into bands, each
    /// of which goes round the axis between two of them and is then cut
    /// into sectors as any face going round its axis is.
    fn cut_whole_torus(&mut self, f: usize, revolved: &Revolved) -> GeopResult<()> {
        use std::f64::consts::TAU;
        let outward_natural = self.faces[f].outward_is_natural();
        let holes = self.faces[f].loops.clone();
        // Where not to cut: at any vertex, or through a hole — round the
        // tube for the parallels, round the axis for where they start.
        let mut round_tube: Vec<(f64, f64)> = Vec::new();
        let mut round_axis: Vec<(f64, f64)> = Vec::new();
        for lp in &holes {
            let points = self.loop_points(lp)?;
            let range = |angles: Vec<f64>| {
                let lo = angles.iter().copied().fold(f64::INFINITY, f64::min);
                let hi = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                (lo, hi)
            };
            round_tube.push(range(unwrap_tube(revolved, &points, false)));
            round_axis.push(range(unwrap(revolved, &points, false, outward_natural)));
        }
        let cuts = choose_cuts(&round_tube);
        let start = choose_cuts(&round_axis)[0];
        let base_name = self.faces[f].name.clone();
        let mut parallels = Vec::new();
        for (c, &cut) in cuts.iter().enumerate() {
            let curve = revolved.parallel::<S>(cut, start, start)?;
            let point = curve.evaluate(curve.domain().0)?;
            let mut name = base_name.clone();
            name.push(format!("m{c}"));
            let v = self.vertices.len();
            self.vertices.push(Vertex {
                point,
                origin: point,
                name: [name.clone(), vec!["v".to_string()]].concat(),
                id: None,
            });
            parallels.push(self.edges.len());
            self.edges.push(Edge {
                curve,
                start: v,
                end: v,
                name,
                alive: true,
                id: None,
            });
        }
        let n = cuts.len();
        let upper = |s: usize| {
            if s + 1 < n {
                cuts[s + 1]
            } else {
                cuts[0] + TAU
            }
        };
        let face = &self.faces[f];
        let (surface, same_sense, shell, id) =
            (face.surface.clone(), face.same_sense, face.shell, face.id);
        let mut bands: Vec<Vec<Vec<Use>>> = (0..n)
            .map(|s| {
                // Above the lower parallel, running counter-clockwise about
                // the axis, below the upper one, running back — about the
                // natural normal.
                let next = (s + 1) % n;
                vec![
                    vec![(parallels[s], outward_natural)],
                    vec![(parallels[next], !outward_natural)],
                ]
            })
            .collect();
        for lp in holes {
            let a = unwrap_tube(revolved, &self.loop_points(&lp)?, false);
            let mid = a.iter().sum::<f64>() / a.len() as f64;
            let band = (0..n)
                .find(|&s| cuts[s] + (mid - cuts[s]).rem_euclid(TAU) < upper(s))
                .expect("the bands cover every angle");
            bands[band].push(lp);
        }
        for (s, loops) in bands.into_iter().enumerate() {
            let mut name = base_name.clone();
            name.push(format!("b{s}"));
            self.faces.push(Face {
                surface: surface.clone(),
                same_sense,
                loops,
                outer: None,
                patch: None,
                shell,
                name,
                id,
            });
        }
        self.faces.remove(f);
        Ok(())
    }

    /// Whether a loop of face `f`, which does not wrap round the axis of
    /// `revolved`, still turns about it through a whole turn or more from
    /// end to end — as a thread's flank does, a strip winding round a
    /// cylinder.
    fn turns_more_than_once(&self, f: usize, revolved: &Revolved) -> GeopResult<bool> {
        let natural = self.faces[f].outward_is_natural();
        for lp in &self.faces[f].loops {
            let angles = unwrap(revolved, &self.loop_points(lp)?, false, natural);
            let lo = angles.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            if hi - lo >= std::f64::consts::TAU {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Cuts the face `f`, a strip turning about the axis of `revolved` more
    /// than once without going round it — a thread's flank — along
    /// meridians half a turn apart into pieces each turning less than once.
    ///
    /// Unwrapped, the angle about the axis runs on along the strip, and the
    /// face is a region of the plane of angles and profile parameters. Each
    /// cut, a line of one angle, crosses its loop an even number of times;
    /// in order along the line, the crossings pair up into the stretches of
    /// meridian inside the face. Every piece is then traced along the loop,
    /// turning along a meridian at each crossing it reaches — the face
    /// stays on the left throughout.
    fn cut_helical_face(&mut self, f: usize, revolved: &Revolved) -> GeopResult<()> {
        use std::f64::consts::{PI, TAU};
        if self.faces[f].loops.len() != 1 {
            return Err(GeopError::new(format!(
                "it has {} loops: only a strip bounded by one loop is cut",
                self.faces[f].loops.len()
            )));
        }
        if revolved.v_is_angle() {
            return Err(GeopError::new(
                "it lies on a torus: only strips on cylinders, cones and other surfaces of revolution with a profile from end to end are cut",
            ));
        }
        let lp = self.faces[f].loops[0].clone();
        // The loop's angles, unwrapped from coedge to coedge, sampled.
        let mut samples: Vec<(usize, f64, f64, f64)> = Vec::new(); // (coedge, t, angle, v)
        let mut prev: Option<f64> = None;
        for (k, &u) in lp.iter().enumerate() {
            for (t, p) in self.samples(u, LOOP_SAMPLES)? {
                let (Some(a), v) = revolved.chart(p) else {
                    return Err(GeopError::new(format!(
                        "its loop runs through its axis at {p:?}"
                    )));
                };
                let a = prev.map_or(a, |r| r + (a - r + PI).rem_euclid(TAU) - PI);
                prev = Some(a);
                samples.push((k, t, a, v));
            }
        }
        let lo = samples.iter().map(|s| s.2).fold(f64::INFINITY, f64::min);
        let hi = samples
            .iter()
            .map(|s| s.2)
            .fold(f64::NEG_INFINITY, f64::max);
        // Cuts half a turn apart, as far from every vertex as can be.
        let corners: Vec<f64> = samples
            .iter()
            .enumerate()
            .filter(|&(i, s)| i == 0 || samples[i - 1].0 != s.0)
            .map(|(_, s)| s.2)
            .collect();
        let clearance = |offset: f64| {
            corners
                .iter()
                .map(|&a| {
                    let r = (a - offset).rem_euclid(PI);
                    r.min(PI - r)
                })
                .fold(f64::INFINITY, f64::min)
        };
        let offset = (0..720)
            .map(|i| lo + PI * i as f64 / 720.0)
            .max_by(|a, b| clearance(*a).total_cmp(&clearance(*b)))
            .expect("candidates");
        let cuts: Vec<f64> = (1..)
            .map(|k| offset + PI * k as f64)
            .take_while(|&a| a < hi)
            .collect();

        // Where the loop crosses each cut: the edge, its parameter, and
        // the profile parameter there.
        let mut crossings: Vec<(usize, f64, usize, f64)> = Vec::new(); // (edge, t, cut, v)
        for w in samples.windows(2) {
            let ((k0, t0, a0, _), (k1, t1, a1, _)) = (w[0], w[1]);
            if k0 != k1 {
                continue;
            }
            let u = lp[k0];
            for (c, &cut) in cuts.iter().enumerate() {
                // Once for each crossing, a sample on the cut included.
                if (a0 < cut) == (a1 < cut) {
                    continue;
                }
                // Bisection, as far as `f64` goes: where exactly is a free
                // choice the meridian is built through.
                let angle_at = |t: f64| -> GeopResult<f64> {
                    let p = super::geometry::point_at(&self.edges[u.0].curve, t)?;
                    let a = revolved
                        .chart(p)
                        .0
                        .ok_or_else(|| GeopError::new("its loop runs through its axis"))?;
                    Ok(cut + (a - cut + PI).rem_euclid(TAU) - PI)
                };
                let (mut ta, mut tb) = (t0, t1);
                let below = angle_at(ta)? < cut;
                for _ in 0..100 {
                    let mid = 0.5 * (ta + tb);
                    if mid == ta || mid == tb {
                        break;
                    }
                    if (angle_at(mid)? < cut) == below {
                        ta = mid;
                    } else {
                        tb = mid;
                    }
                }
                let t = 0.5 * (ta + tb);
                let v = revolved
                    .chart(super::geometry::point_at(&self.edges[u.0].curve, t)?)
                    .1;
                crossings.push((u.0, t, c, v));
            }
        }
        // Split every edge where it is crossed.
        let mut by_edge: HashMap<usize, Vec<(f64, usize, f64)>> = HashMap::new();
        for &(e, t, c, v) in &crossings {
            by_edge.entry(e).or_default().push((t, c, v));
        }
        let mut edges: Vec<usize> = by_edge.keys().copied().collect();
        edges.sort();
        let mut on_cut: Vec<Vec<(f64, usize)>> = vec![Vec::new(); cuts.len()]; // (v, vertex)
        for e in edges {
            let mut list = by_edge[&e].clone();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            let ts: Vec<S> = list.iter().map(|x| S::from_f64(x.0)).collect();
            let (d0, d1) = self.edges[e].curve.domain();
            if ts
                .iter()
                .any(|t| !(t.definitely_greater(d0) && t.definitely_less(d1)))
            {
                return Err(GeopError::new(format!(
                    "a cut crosses its loop at a vertex, at the end of the edge {}",
                    self.edges[e].label()
                )));
            }
            let created = self.split_edge(e, &ts)?;
            for ((_, c, v), vertex) in list.into_iter().zip(created) {
                on_cut[c].push((v, vertex));
            }
        }
        // Along each cut, the crossings in order pair up into the
        // stretches of meridian inside the face.
        let mut partner: HashMap<usize, (usize, usize)> = HashMap::new(); // vertex -> (other, meridian)
        let mut count = 0;
        for (c, list) in on_cut.iter_mut().enumerate() {
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            if list.len() % 2 != 0 {
                return Err(GeopError::new(format!(
                    "the cut at the angle {} crosses its loop an odd number of times, {}",
                    cuts[c],
                    list.len()
                )));
            }
            for pair in list.chunks(2) {
                let ((vb, b), (vt, t)) = (pair[0], pair[1]);
                let pb = to_p3(&self.vertices[b].point);
                let pt = to_p3(&self.vertices[t].point);
                let curve = self.meridian(revolved, cuts[c], (pb, vb, false), (pt, vt, false))?;
                let mut name = self.faces[f].name.clone();
                name.push(format!("m{count}"));
                count += 1;
                let m = self.edges.len();
                self.edges.push(Edge {
                    curve,
                    start: b,
                    end: t,
                    name,
                    alive: true,
                    id: None,
                });
                partner.insert(b, (t, m));
                partner.insert(t, (b, m));
            }
        }

        // The pieces, traced along the split loop.
        let lp = self.faces[f].loops[0].clone();
        let starting: HashMap<usize, usize> = lp
            .iter()
            .enumerate()
            .map(|(i, &u)| (self.ends(u).0, i))
            .collect();
        let mut used = vec![false; lp.len()];
        let mut pieces: Vec<Vec<Use>> = Vec::new();
        for first in 0..lp.len() {
            if used[first] {
                continue;
            }
            let mut piece = Vec::new();
            let mut i = first;
            loop {
                if used[i] || piece.len() > 2 * lp.len() + partner.len() {
                    return Err(GeopError::new(
                        "tracing a piece along its loop and the meridians did not close",
                    ));
                }
                used[i] = true;
                piece.push(lp[i]);
                let end = self.ends(lp[i]).1;
                i = match partner.get(&end) {
                    Some(&(other, m)) => {
                        piece.push((m, self.edges[m].start == end));
                        starting[&other]
                    }
                    None => (i + 1) % lp.len(),
                };
                if i == first {
                    break;
                }
            }
            pieces.push(piece);
        }
        let face = &self.faces[f];
        let (surface, same_sense, shell, base_name, id) = (
            face.surface.clone(),
            face.same_sense,
            face.shell,
            face.name.clone(),
            face.id,
        );
        for (q, piece) in pieces.into_iter().enumerate() {
            let mut name = base_name.clone();
            name.push(format!("q{q}"));
            self.faces.push(Face {
                surface: surface.clone(),
                same_sense,
                loops: vec![piece],
                outer: Some(0),
                patch: None,
                shell,
                name,
                id,
            });
        }
        self.faces.remove(f);
        Ok(())
    }

    /// Splits each of the rings `rings` of face `f` where it crosses each
    /// of the angles `cuts`, as `angle_of` measures a point's: the vertex
    /// each ring has at each cut, by ring.
    fn split_rings(
        &mut self,
        f: usize,
        rings: &[usize],
        cuts: &[f64],
        angle_of: &dyn Fn(P3) -> Option<f64>,
    ) -> GeopResult<HashMap<usize, Vec<usize>>> {
        let mut ring_vertices: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut splits: HashMap<usize, Vec<(f64, usize, usize)>> = HashMap::new(); // edge -> (t, ring loop, cut)
        for &k in rings {
            let lp = self.faces[f].loops[k].clone();
            for (c, &cut) in cuts.iter().enumerate() {
                let (e, t) = self.crossing(&lp, cut, angle_of)?;
                splits.entry(e).or_default().push((t, k, c));
            }
        }
        let mut edges: Vec<usize> = splits.keys().copied().collect();
        edges.sort();
        for e in edges {
            let mut list = splits[&e].clone();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            let ts: Vec<S> = list.iter().map(|(t, _, _)| S::from_f64(*t)).collect();
            let (lo, hi) = self.edges[e].curve.domain();
            if ts
                .iter()
                .any(|t| !(t.definitely_greater(lo) && t.definitely_less(hi)))
            {
                return Err(GeopError::new(format!(
                    "a cut crosses a loop round it at a vertex: the edge {} at its parameters {ts:?}, its domain [{lo:?}, {hi:?}]",
                    self.edges[e].label()
                )));
            }
            let created = self.split_edge(e, &ts)?;
            for ((_, ring, cut), v) in list.into_iter().zip(created) {
                let slots = ring_vertices
                    .entry(ring)
                    .or_insert_with(|| vec![usize::MAX; cuts.len()]);
                slots[cut] = v;
            }
        }
        Ok(ring_vertices)
    }

    /// The coedges of the loop `lp` from the vertex `a` to the vertex `b`,
    /// running the loop's way (`along`) or against it.
    fn run_between(&self, lp: &[Use], a: usize, b: usize, along: bool) -> GeopResult<Vec<Use>> {
        let seq: Vec<Use> = if along {
            lp.to_vec()
        } else {
            lp.iter().rev().map(|&(e, fw)| (e, !fw)).collect()
        };
        let start = seq
            .iter()
            .position(|&u| self.ends(u).0 == a)
            .ok_or_else(|| GeopError::new("a cut vertex is not on its ring"))?;
        let mut run = Vec::new();
        for j in 0..seq.len() {
            let u = seq[(start + j) % seq.len()];
            run.push(u);
            if self.ends(u).1 == b {
                return Ok(run);
            }
        }
        Err(GeopError::new("a ring does not reach the next cut"))
    }

    /// Where along the ring `lp` it crosses the angle `cut`: the edge,
    /// and its parameter there. A free choice of where exactly, refined
    /// as far as `f64` bisection goes: the cut's meridian is built through
    /// what the edge has there.
    fn crossing(
        &self,
        lp: &[Use],
        cut: f64,
        angle_of: &dyn Fn(P3) -> Option<f64>,
    ) -> GeopResult<(usize, f64)> {
        use std::f64::consts::{PI, TAU};
        let near = |a: f64, reference: f64| reference + (a - reference + PI).rem_euclid(TAU) - PI;
        let mut prev: Option<f64> = None;
        let mut covered = (f64::INFINITY, f64::NEG_INFINITY);
        for &u in lp {
            let samples = self.samples(u, LOOP_SAMPLES)?;
            let mut last: Option<(f64, f64)> = None; // (parameter, unwrapped angle)
            for &(t, p) in &samples {
                let Some(a) = angle_of(p) else {
                    continue;
                };
                let a = match prev {
                    Some(r) => near(a, r),
                    None => a,
                };
                if let Some((t_prev, a_prev)) = last {
                    let (lo, hi) = (a_prev.min(a), a_prev.max(a));
                    let x = cut + TAU * ((lo - cut) / TAU).ceil();
                    if x <= hi && lo < hi {
                        let theta = |t: f64| -> GeopResult<f64> {
                            let p = super::geometry::point_at(&self.edges[u.0].curve, t)?;
                            let angle = angle_of(p).ok_or_else(|| {
                                GeopError::new("a loop round its axis runs through it")
                            })?;
                            Ok(near(angle, x) - x)
                        };
                        let (mut t0, mut t1) = (t_prev, t);
                        let mut g0 = theta(t0)?;
                        for _ in 0..100 {
                            let mid = 0.5 * (t0 + t1);
                            if mid == t0 || mid == t1 {
                                break;
                            }
                            let g = theta(mid)?;
                            if (g <= 0.0) == (g0 <= 0.0) {
                                t0 = mid;
                                g0 = g;
                            } else {
                                t1 = mid;
                            }
                        }
                        return Ok((u.0, 0.5 * (t0 + t1)));
                    }
                }
                last = Some((t, a));
                prev = Some(a);
                covered = (covered.0.min(a), covered.1.max(a));
            }
        }
        Err(GeopError::new(format!(
            "a loop round its axis never reaches the angle {cut}: it covers the angles {covered:?}"
        )))
    }

    /// The meridian at angle `angle` from `bottom` to `top`, each a point,
    /// its profile parameter, and whether it is a pole.
    fn meridian(
        &self,
        revolved: &Revolved,
        angle: f64,
        (pb, vb, _): (P3, f64, bool),
        (pt, vt, _): (P3, f64, bool),
    ) -> GeopResult<NurbCurve3D<S>> {
        let turn = revolved.turn::<S>(angle)?;
        let profile = match &revolved.profile {
            Profile::Curve(curve) => {
                let curve = curve.to_nurbs::<S>()?;
                let back = turn.inverse();
                let at = |p: P3| -> GeopResult<S> {
                    let p = to_p3(&back.apply(&s3(p)));
                    super::geometry::parameter_of(&curve, p)
                };
                let (tb, tt) = (at(pb)?, at(pt)?);
                let (lo, hi) = curve.domain();
                let (a, b, reversed) = if tb.definitely_less(tt) {
                    (tb, tt, false)
                } else {
                    (tt, tb, true)
                };
                let piece = curve.sub_curve(
                    if a.could_be_equal(lo) { lo } else { a },
                    if b.could_be_equal(hi) { hi } else { b },
                )?;
                if reversed { piece.reverse() } else { piece }
            }
            _ => revolved.profile_curve::<S>(vb, vt)?,
        };
        Ok(profile.transform(&turn.motion()))
    }

    /// The description of the body, and the names of what it is made of.
    #[allow(clippy::type_complexity)]
    /// The body built, `label` what the file calls it.
    fn spec(self, solid: bool, label: String) -> GeopResult<ImportedBody<S>> {
        let Builder {
            vertices,
            edges,
            faces,
            shells,
            scope,
            ..
        } = self;
        let mut vertices = vertices;
        let mut edges = edges;
        // Which edges and vertices are left, renumbered.
        let mut edge_map = vec![usize::MAX; edges.len()];
        let mut used_vertex = vec![false; vertices.len()];
        for face in &faces {
            for &(e, _) in face.loops.iter().flatten() {
                edge_map[e] = 0;
                used_vertex[edges[e].start] = true;
                used_vertex[edges[e].end] = true;
            }
        }
        let mut vertex_map = vec![usize::MAX; vertices.len()];
        let mut next = 0;
        for (v, used) in used_vertex.iter().enumerate() {
            if *used {
                vertex_map[v] = next;
                next += 1;
            }
        }

        // Each face's patch, pointing out of the material.
        let mut face_specs = Vec::with_capacity(faces.len());
        // Each face's patch extent, for messages.
        let mut extents = Vec::with_capacity(faces.len());
        let mut face_names = Vec::with_capacity(faces.len());
        let mut face_shells = vec![Vec::new(); shells];
        let face_ctx = |face: &Face| {
            let what = format!("building the face #{} ({})", face.id, face.name.join(","));
            move |e: GeopError| e.with_context(what.clone())
        };
        // Every face's patch first: what its vertices and edges are healed
        // onto where they disagree with it.
        let mut built = Vec::with_capacity(faces.len());
        for face in &faces {
            let ctx = face_ctx(face);
            let patch = patch_of(face, &edges, &vertices, &scope).map_err(&ctx)?;
            let surface = if face.outward_is_natural() {
                patch.surface.clone()
            } else {
                patch.surface.reverse_u()
            };
            let grid = Grid::new(&surface).map_err(&ctx)?;
            built.push((patch, surface, grid));
        }
        let (rebuild, healed) = heal(&faces, &built, &mut edges, &mut vertices, &scope)?;
        for (f, face) in faces.iter().enumerate() {
            if rebuild[f] {
                let surface = rebuilt_surface(face, &edges).map_err(|e| {
                    e.with_context(format!(
                        "rebuilding the face #{} ({}) from its edges, where its surface contradicts its neighbours'",
                        face.id,
                        face.name.join(",")
                    ))
                })?;
                let grid = Grid::new(&surface).map_err(face_ctx(face))?;
                let patch = Patch {
                    surface: surface.clone(),
                    extent: None,
                    planar: false,
                };
                built[f] = (patch, surface, grid);
            }
        }
        for (face, (patch, surface, grid)) in faces.iter().zip(built) {
            let ctx = face_ctx(face);
            let natural = face.outward_is_natural();
            let mut loops = Vec::new();
            for lp in &face.loops {
                loops.push(
                    fit_loop(lp, &surface, &grid, &patch, &edges, &mut vertices, &scope)
                        .map_err(&ctx)?,
                );
            }
            // The outer loop runs counter-clockwise in the patch.
            let outer = match face.outer {
                Some(k) => k,
                None => {
                    let areas: Vec<f64> = loops
                        .iter()
                        .map(|lp| signed_area(lp))
                        .collect::<GeopResult<_>>()
                        .map_err(&ctx)?;
                    let ccw: Vec<usize> = (0..areas.len()).filter(|&k| areas[k] > 0.0).collect();
                    match ccw.as_slice() {
                        [k] => *k,
                        _ => {
                            return Err(ctx(GeopError::new(format!(
                                "its loops do not bound one region: {} of {} run counter-clockwise about its normal (signed areas {areas:?}; its outward normal the surface's natural one: {natural}; built over the angles and profile parameters {:?})",
                                ccw.len(),
                                areas.len(),
                                patch.extent
                            ))));
                        }
                    }
                }
            };
            let outer_loop = loops.remove(outer);
            face_shells[face.shell].push(face_specs.len());
            face_specs.push((surface, outer_loop, loops));
            extents.push(patch.extent);
            face_names.push(face.name.clone());
        }

        // Every place the file says each edge is, united: its curve, and
        // where its pcurves put it on its faces' surfaces.
        // Widened by what is measured, an edge's enclosure meets what it is
        // measured against only to within rounding: measured again, it is
        // widened by what is left, until nothing is.
        let read: Vec<NurbCurve3D<S>> = edges.iter().map(|e| e.curve.clone()).collect();
        // How far each edge has been widened so far, either way.
        let mut total = vec![0.0f64; edges.len()];
        let mut history: Vec<Vec<f64>> = vec![Vec::new(); edges.len()];
        // Which edges to measure: all at first, then those just widened —
        // an edge left as it was measures as it did.
        let mut measure = vec![true; edges.len()];
        for round in 0..WIDENINGS {
            let mut gaps = vec![[0.0f64; 3]; edges.len()];
            for (((surface, outer, holes), face_name), extent) in
                face_specs.iter().zip(&face_names).zip(&extents)
            {
                for (on, pcurve) in outer.iter().chain(holes.iter().flatten()) {
                    if let CoedgeOnLocal::Edge(e, _) = *on
                        && measure[e]
                    {
                        let (gap, worst) =
                            edge_gap(&edges[e].curve, surface, pcurve).map_err(|err| {
                                err.with_context(format!(
                            "measuring how far the edge {} lies from where its pcurve puts it",
                            edges[e].label()
                        ))
                            })?;
                        // Widened by it either way, the edge must still be
                        // one curve to the kernel.
                        let most = total[e] + gap.iter().copied().fold(0.0, f64::max);
                        if 2.0 * most > ACCURACY {
                            // Whether the file's curve is off the surface,
                            // or the pcurve fitted to it is off the curve.
                            let off = off_surface(&read[e], surface, pcurve).map_or_else(
                                |err| format!("not measured: {err}"),
                                |d| format!("{d:e} mm"),
                            );
                            let furthest = worst.map_or_else(String::new, |w| {
                                format!(
                                    ", furthest {:e} mm at {} of the pcurve, which puts it at {:?}, the edge's nearest point {:?}",
                                    w.apart, w.at, w.point, w.near
                                )
                            });
                            return Err(GeopError::new(format!(
                                "the edge {} lies up to {most:e} mm from where its pcurve puts it on its face {}, its own points up to {off} from the surface — the curve widened by that either way would be wider than the {ACCURACY:e} mm the kernel can carry as one curve (measured {gap:?} per coordinate after widening it by {:?} in {round} rounds{furthest}; the edge from {:?} to {:?}, the pcurve from {:?} to {:?} on the patch over the angles and profile parameters {extent:?})",
                                edges[e].label(),
                                face_name.join(","),
                                history[e],
                                to_p3(&read[e].evaluate(read[e].domain().0)?),
                                to_p3(&read[e].evaluate(read[e].domain().1)?),
                                pcurve.evaluate(pcurve.domain().0)?,
                                pcurve.evaluate(pcurve.domain().1)?,
                            )));
                        }
                        for c in 0..3 {
                            gaps[e][c] = gaps[e][c].max(gap[c]);
                        }
                    }
                }
            }
            if gaps.iter().all(|g| *g == [0.0; 3]) {
                break;
            }
            for ((edge, gap), total) in edges.iter_mut().zip(&gaps).zip(&mut total) {
                edge.curve = widened(&edge.curve, *gap)?;
                *total += gap.iter().copied().fold(0.0, f64::max);
            }
            for (h, gap) in history.iter_mut().zip(&gaps) {
                h.push(gap.iter().copied().fold(0.0, f64::max));
            }
            for (m, gap) in measure.iter_mut().zip(&gaps) {
                *m = *gap != [0.0; 3];
            }
        }

        let mut edge_specs = Vec::new();
        let mut edge_names = Vec::new();
        for (e, edge) in edges.iter().enumerate() {
            if edge_map[e] == usize::MAX {
                continue;
            }
            edge_map[e] = edge_specs.len();
            edge_specs.push(EdgeSpec {
                curve: edge.curve.clone(),
                start: vertex_map[edge.start],
                end: vertex_map[edge.end],
            });
            edge_names.push(edge.name.clone());
        }
        let mut vertex_points = Vec::new();
        let mut vertex_names = Vec::new();
        for (v, vertex) in vertices.into_iter().enumerate() {
            if used_vertex[v] {
                vertex_points.push(vertex.point);
                vertex_names.push(vertex.name);
            }
        }
        let remap = |lp: Vec<(CoedgeOnLocal, NurbCurve2D<S>)>| -> Vec<CoedgeSpec<S>> {
            lp.into_iter()
                .map(|(on, pcurve)| CoedgeSpec {
                    on: match on {
                        CoedgeOnLocal::Edge(e, forward) => CoedgeOn::Edge(
                            edge_map[e],
                            if forward {
                                Sense::Forward
                            } else {
                                Sense::Reversed
                            },
                        ),
                        CoedgeOnLocal::Vertex(v) => CoedgeOn::Vertex(vertex_map[v]),
                    },
                    pcurve,
                })
                .collect()
        };
        let faces = face_specs
            .into_iter()
            .map(|(surface, outer, holes)| FaceSpec {
                surface,
                outer: remap(outer),
                holes: holes.into_iter().map(remap).collect(),
            })
            .collect();
        let spec = BodySpec {
            vertices: vertex_points,
            edges: edge_specs,
            faces,
            shells: face_shells.into_iter().filter(|s| !s.is_empty()).collect(),
            solid,
        };
        Ok(ImportedBody {
            label,
            spec,
            vertex_names,
            edge_names,
            face_names,
            healed,
        })
    }
}

#[derive(Clone, Copy, Debug)]
enum CoedgeOnLocal {
    Edge(usize, bool),
    Vertex(usize),
}

/// The surface a face lies on, built to cover it.
struct Patch<S: Scalar> {
    surface: NurbSurface3D<S>,
    /// For a surface of revolution, the angles and profile parameters it
    /// was built over: for messages.
    extent: Option<[f64; 4]>,
    /// Whether it is a plane's: a parallelogram, on which every pcurve is
    /// its edge in the plane's coordinates (see [`plane_pcurve`]).
    planar: bool,
}

/// A row of a patch's control points collapsed to one point: a pole of its
/// parametrization. The row is where `u` (`fixes_u`) or `v` is `at`.
#[derive(Clone, Copy, Debug)]
struct Pole<S: Scalar> {
    fixes_u: bool,
    at: S,
    point: P3,
}

/// The poles of `surface`: each boundary row whose control points all lie
/// within `uncertainty` of each other — the file's own statement of which
/// points are one.
fn poles_of<S: Scalar>(surface: &NurbSurface3D<S>, uncertainty: f64) -> GeopResult<Vec<Pole<S>>> {
    let point = |i: usize, j: usize| -> GeopResult<P3> {
        let cp = surface.control_points[i * surface.num_v + j];
        let w = cp[3];
        Ok([
            cp[0].div(w)?.to_f64(),
            cp[1].div(w)?.to_f64(),
            cp[2].div(w)?.to_f64(),
        ])
    };
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    let (nu, nv) = (surface.num_u, surface.num_v);
    let mut poles = Vec::new();
    let rows: [(bool, S, Vec<(usize, usize)>); 4] = [
        (true, u_lo, (0..nv).map(|j| (0, j)).collect()),
        (true, u_hi, (0..nv).map(|j| (nu - 1, j)).collect()),
        (false, v_lo, (0..nu).map(|i| (i, 0)).collect()),
        (false, v_hi, (0..nu).map(|i| (i, nv - 1)).collect()),
    ];
    for (fixes_u, at, row) in rows {
        let points: Vec<P3> = row
            .iter()
            .map(|&(i, j)| point(i, j))
            .collect::<GeopResult<_>>()?;
        if points
            .iter()
            .all(|&p| distance(p, points[0]) <= uncertainty)
        {
            poles.push(Pole {
                fixes_u,
                at,
                point: points[0],
            });
        }
    }
    Ok(poles)
}

/// The unwrapped angles about `revolved`'s axis of `points`, a loop's in
/// order: each within half a turn of the one before. With `closing`, the
/// loop is followed back to where it started, so the last angle less the
/// first is how far it turns.
///
/// A point on the axis has no angle. Where a loop runs through a pole, it
/// turns there by whatever angle lies between where it arrives and where
/// it leaves — the long way round as readily as the short — and which way
/// is the face's: a loop running counter-clockwise about the surface's
/// natural normal (`ccw`) runs along a pole at the top of the profile
/// backwards in angle, along one at the bottom forwards, as it runs along
/// the top and bottom of its region.
fn unwrap(revolved: &Revolved, points: &[P3], closing: bool, ccw: bool) -> Vec<f64> {
    use std::f64::consts::{PI, TAU};
    // Start at a point with an angle, so that a pole is always passed
    // between two.
    let start = points
        .iter()
        .position(|&p| revolved.chart(p).0.is_some())
        .unwrap_or(0);
    let n = points.len();
    let mean_v = points.iter().map(|&p| revolved.chart(p).1).sum::<f64>() / n.max(1) as f64;
    let mut out: Vec<f64> = Vec::with_capacity(n + 1);
    let mut pole: Option<f64> = None;
    for k in 0..n + usize::from(closing) {
        let p = points[(start + k) % n];
        let (angle, v) = revolved.chart(p);
        let Some(a) = angle else {
            pole = Some(v);
            continue;
        };
        let a = match (out.last(), pole.take()) {
            (None, _) => a,
            (Some(&prev), None) => prev + (a - prev + PI).rem_euclid(TAU) - PI,
            (Some(&prev), Some(v)) => {
                let turn = (a - prev).rem_euclid(TAU);
                let forwards = ccw != (v > mean_v);
                prev + if forwards || turn == 0.0 {
                    turn
                } else {
                    turn - TAU
                }
            }
        };
        out.push(a);
    }
    out
}

/// The unwrapped angles round the tube of `revolved`, a torus, of
/// `points`, a loop's in order: each within half a turn of the one before.
/// With `closing`, the loop is followed back to where it started.
fn unwrap_tube(revolved: &Revolved, points: &[P3], closing: bool) -> Vec<f64> {
    use std::f64::consts::{PI, TAU};
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    let mut out: Vec<f64> = Vec::with_capacity(n + 1);
    for k in 0..n + usize::from(closing) {
        let a = revolved.chart(points[k % n]).1;
        out.push(match out.last() {
            None => a,
            Some(&prev) => prev + (a - prev + PI).rem_euclid(TAU) - PI,
        });
    }
    out
}

/// How often unwrapped closing angles go round.
fn winding(angles: &[f64]) -> i64 {
    match (angles.first(), angles.last()) {
        (Some(a), Some(b)) => ((b - a) / std::f64::consts::TAU).round() as i64,
        _ => 0,
    }
}

/// Angles to cut a band around an axis at, given the angles (single ones,
/// or ranges, unwrapped) where it must not be cut: two or more, evenly
/// spaced, placed as far from everything blocked as can be — the fewest
/// cuts that keep a clear berth.
fn choose_cuts(blocked: &[(f64, f64)]) -> Vec<f64> {
    use std::f64::consts::TAU;
    let clearance = |a: f64| -> f64 {
        blocked
            .iter()
            .map(|&(lo, hi)| {
                if hi - lo >= TAU {
                    return 0.0;
                }
                // The distance from `a` to the arc from `lo` to `hi`.
                let rel = (a - lo).rem_euclid(TAU);
                if rel <= hi - lo {
                    0.0
                } else {
                    (rel - (hi - lo)).min(TAU - rel)
                }
            })
            .fold(f64::INFINITY, f64::min)
    };
    let mut best: Option<(f64, Vec<f64>)> = None;
    for n in 2..=8usize {
        let step = TAU / n as f64;
        for i in 0..720 {
            let start = step * i as f64 / 720.0;
            let cuts: Vec<f64> = (0..n).map(|k| start + step * k as f64).collect();
            let c = cuts
                .iter()
                .map(|&a| clearance(a))
                .fold(f64::INFINITY, f64::min);
            // More cuts only where they clear things by much more.
            if best.as_ref().is_none_or(|(b, _)| c > 2.0 * b) {
                best = Some((c, cuts));
            }
        }
    }
    best.expect("at least one choice").1
}

/// `[v0, v1]` widened a little either way where it is free to be — not at
/// a pole, nor past one, nor for a torus' tube further than once round.
fn widen_v(revolved: &Revolved, v0: f64, v1: f64, pole0: bool, pole1: bool) -> (f64, f64) {
    if matches!(revolved.profile, Profile::Curve(_)) {
        return (v0, v1);
    }
    let span = (v1 - v0).abs();
    let mut margin = 0.02 * span + 1e-3 * span.max(1e-9);
    if revolved.v_is_angle() {
        margin = margin.min((std::f64::consts::TAU - span) / 2.0 * 0.5);
    }
    let poles = revolved.poles();
    let lowest = poles
        .iter()
        .copied()
        .filter(|&p| p <= v0)
        .fold(f64::NEG_INFINITY, f64::max);
    let highest = poles
        .iter()
        .copied()
        .filter(|&p| p >= v1)
        .fold(f64::INFINITY, f64::min);
    let a = if pole0 { v0 } else { (v0 - margin).max(lowest) };
    let b = if pole1 {
        v1
    } else {
        (v1 + margin).min(highest)
    };
    (a, b)
}

/// The patch a face lies on: the surface, built to cover the face.
fn patch_of<S: Scalar>(
    face: &Face,
    edges: &[Edge<S>],
    vertices: &[Vertex<S>],
    scope: &Scope,
) -> GeopResult<Patch<S>> {
    let mut points = Vec::new();
    let mut loop_points = Vec::new();
    for lp in &face.loops {
        let mut this = Vec::new();
        for &(e, forward) in lp {
            let curve = &edges[e].curve;
            let (lo, hi) = curve.domain();
            let (lo, hi) = (lo.to_f64(), hi.to_f64());
            for i in 0..LOOP_SAMPLES {
                let f = i as f64 / LOOP_SAMPLES as f64;
                let f = if forward { f } else { 1.0 - f };
                this.push(super::geometry::point_at(curve, lo + (hi - lo) * f)?);
            }
        }
        points.extend(this.iter().copied());
        loop_points.push(this);
    }
    let _ = vertices;
    match &face.surface.kind {
        SurfaceKind::Plane(frame) => {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for &p in &points {
                let l = frame.local(p);
                for c in 0..2 {
                    lo[c] = lo[c].min(l[c]);
                    hi[c] = hi[c].max(l[c]);
                }
            }
            let margin = 0.05 * ((hi[0] - lo[0]).max(hi[1] - lo[1])).max(scope.uncertainty);
            let corner = |a: f64, b: f64| s3::<S>(frame.point([a, b, 0.0]));
            let (a0, a1, b0, b1) = (
                lo[0] - margin,
                hi[0] + margin,
                lo[1] - margin,
                hi[1] + margin,
            );
            let h = |p: Vector3<S>| {
                geop_core_math::vector::Vector4::from_array([p[0], p[1], p[2], S::ONE])
            };
            let surface = NurbSurface3D::try_new(
                1,
                1,
                vec![
                    h(corner(a0, b0)),
                    h(corner(a0, b1)),
                    h(corner(a1, b0)),
                    h(corner(a1, b1)),
                ],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )?;
            Ok(Patch {
                surface,
                extent: None,
                planar: true,
            })
        }
        SurfaceKind::Nurbs(nurbs) => Ok(Patch {
            surface: nurbs.to_nurbs()?,
            extent: None,
            planar: false,
        }),
        SurfaceKind::Extrusion { curve, vector } => {
            let length2 = dot(*vector, *vector);
            let dir =
                normalize(*vector).ok_or_else(|| GeopError::new("an extrusion along nothing"))?;
            let along = |p: P3| dot(p, dir);
            let (p_lo, p_hi) = points
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &p| {
                    (a.min(along(p)), b.max(along(p)))
                });
            let (c_lo, c_hi) = curve
                .points
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &p| {
                    (a.min(along(p)), b.max(along(p)))
                });
            let length = length2.sqrt();
            let (v0, v1) = ((p_lo - c_hi) / length, (p_hi - c_lo) / length);
            let margin = 0.05 * (v1 - v0).abs().max(scope.uncertainty / length);
            let (v0, v1) = (v0 - margin, v1 + margin);
            let curve = curve.to_nurbs::<S>()?;
            let start = curve.translate(s3(scale(*vector, v0)));
            Ok(Patch {
                surface: start.sweep(s3(scale(*vector, v1 - v0))),
                extent: None,
                planar: false,
            })
        }
        SurfaceKind::Revolved(revolved) => {
            let [from, to, v0, v1] = match face.patch {
                Some(patch) => patch,
                None => revolved_extent(revolved, face, &loop_points, scope)?,
            };
            Ok(Patch {
                surface: revolved.patch(from, to, v0, v1)?,
                extent: Some([from, to, v0, v1]),
                planar: false,
            })
        }
    }
}

/// The angles and profile parameters a face on a surface of revolution
/// covers, which does not wrap around its axis, a little widened where
/// they are free to be.
fn revolved_extent(
    revolved: &Revolved,
    face: &Face,
    loops: &[Vec<P3>],
    scope: &Scope,
) -> GeopResult<[f64; 4]> {
    // The outer loop spans the face's angles; each hole lies within them.
    let mut best: Option<(f64, f64, f64)> = None;
    for points in loops {
        let a = unwrap(revolved, points, false, face.outward_is_natural());
        if a.is_empty() {
            continue;
        }
        let lo = a.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = a.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if best.is_none_or(|(b_lo, b_hi, _)| hi - lo > b_hi - b_lo) {
            best = Some((lo, hi, 0.0));
        }
    }
    let (lo, hi, _) =
        best.ok_or_else(|| GeopError::new("a face whose every point lies on its axis"))?;
    let span = hi - lo;
    if span >= std::f64::consts::TAU {
        return Err(GeopError::new(format!(
            "#{}: a face spanning a full turn or more around its axis without a loop going round it — as a thread's flank does — is not supported",
            face.id
        )));
    }
    let margin = (0.02 * span + 1e-3).min((std::f64::consts::TAU - span) / 4.0);
    // Profile parameters: the analytic ones, or, for a pole, exactly it.
    let mut v_lo = f64::INFINITY;
    let mut v_hi = f64::NEG_INFINITY;
    let mut torus_angles = Vec::new();
    for points in loops {
        for &p in points {
            let v = revolved.chart(p).1;
            torus_angles.push(v);
            v_lo = v_lo.min(v);
            v_hi = v_hi.max(v);
        }
    }
    if revolved.v_is_angle() {
        // A torus' tube angle wraps at half a turn; the face is where the
        // samples are, so measure from the largest gap between them.
        torus_angles.sort_by(f64::total_cmp);
        let n = torus_angles.len();
        let mut gap = (
            torus_angles[0] + std::f64::consts::TAU - torus_angles[n - 1],
            0usize,
        );
        for i in 1..n {
            let g = torus_angles[i] - torus_angles[i - 1];
            if g > gap.0 {
                gap = (g, i);
            }
        }
        v_lo = torus_angles[gap.1];
        v_hi = if gap.1 == 0 {
            torus_angles[n - 1]
        } else {
            torus_angles[gap.1 - 1] + std::f64::consts::TAU
        };
    }
    let near_pole = |v: f64| {
        revolved.poles().into_iter().find(|&pole| {
            distance(revolved.profile_point(pole), revolved.profile_point(v))
                <= scope.uncertainty.max(1e-9)
        })
    };
    let (pole0, pole1) = match &revolved.profile {
        Profile::Curve(_) => (None, None),
        _ => (near_pole(v_lo), near_pole(v_hi)),
    };
    let (v0, v1) = widen_v(
        revolved,
        pole0.unwrap_or(v_lo),
        pole1.unwrap_or(v_hi),
        pole0.is_some(),
        pole1.is_some(),
    );
    Ok([lo - margin, hi + margin, v0, v1])
}

/// A grid of a patch's points, to seed projections onto it from.
struct Grid<S: Scalar> {
    points: Vec<(S, S, P3)>,
}

const GRID: usize = 12;

/// How many of the nearest grid points a projection tries Newton from.
const SEEDS: usize = 4;

impl<S: Scalar> Grid<S> {
    fn new(surface: &NurbSurface3D<S>) -> GeopResult<Self> {
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let mut points = Vec::new();
        for i in 0..=GRID {
            for j in 0..=GRID {
                // Seeds are free choices; the ends are the domain's own.
                let at = |lo: S, hi: S, k: usize| match k {
                    0 => lo,
                    GRID => hi,
                    _ => S::from_f64(
                        lo.to_f64() + (hi.to_f64() - lo.to_f64()) * k as f64 / GRID as f64,
                    ),
                };
                let u = at(u0, u1, i);
                let v = at(v0, v1, j);
                points.push((u, v, to_p3(&surface.evaluate(u, v)?)));
            }
        }
        Ok(Self { points })
    }

    /// The foot point of `p` on `surface`: Newton from the nearest grid
    /// point.
    /// Newton runs from the few nearest grid points, and the foot point
    /// nearest `p` wins: on a curved patch the nearest grid point can lie
    /// in the basin of another foot point.
    fn project(&self, surface: &NurbSurface3D<S>, p: &Vector3<S>) -> GeopResult<(S, S)> {
        let target = to_p3(p);
        let mut seeds: Vec<&(S, S, P3)> = self.points.iter().collect();
        seeds.sort_by(|a, b| distance(a.2, target).total_cmp(&distance(b.2, target)));
        let mut best: Option<(f64, (S, S))> = None;
        for (u, v, _) in seeds.into_iter().take(SEEDS) {
            let Ok(foot) = surface.project(*p, *u, *v, 30) else {
                continue;
            };
            let Ok(at) = surface.evaluate(foot.0, foot.1) else {
                continue;
            };
            let d = distance(to_p3(&at), target);
            if best.is_none_or(|(b, _)| d < b) {
                best = Some((d, foot));
            }
        }
        best.map(|(_, foot)| foot).ok_or_else(|| {
            GeopError::new(format!(
                "no foot point of {p:?} on the face's surface was found"
            ))
        })
    }
}

/// Fits the pcurves of the loop `lp` on `surface`, pinning both ends of
/// each to its vertex's foot point and joining coedges meeting at a pole
/// with a coedge sitting at it. Widens every vertex to enclose the ends of
/// its edges and its foot points.
fn fit_loop<S: Scalar>(
    lp: &[Use],
    surface: &NurbSurface3D<S>,
    grid: &Grid<S>,
    patch: &Patch<S>,
    edges: &[Edge<S>],
    vertices: &mut [Vertex<S>],
    scope: &Scope,
) -> GeopResult<Vec<(CoedgeOnLocal, NurbCurve2D<S>)>> {
    let n = lp.len();
    let ends = |(e, forward): Use| {
        let edge = &edges[e];
        if forward {
            (edge.start, edge.end)
        } else {
            (edge.end, edge.start)
        }
    };
    let poles = poles_of(surface, scope.uncertainty)?;
    // The pole a vertex sits at, if any.
    let pole_of = |v: usize, vertices: &[Vertex<S>]| -> Option<Pole<S>> {
        let p = to_p3(&vertices[v].origin);
        poles
            .iter()
            .find(|pole| distance(pole.point, p) <= scope.uncertainty)
            .copied()
    };
    // Where each corner — the end of coedge `k`, the start of `k + 1` —
    // is on the patch: its vertex's foot point, unless at a pole.
    let mut corners: Vec<Option<Vector2<S>>> = Vec::with_capacity(n);
    for &u in lp {
        let (_, end) = ends(u);
        corners.push(match pole_of(end, vertices) {
            Some(_) => None,
            None => {
                let (pu, pv) = grid.project(surface, &vertices[end].origin)?;
                Some(Vector2::from_array([pu, pv]))
            }
        });
    }
    let mut out = Vec::new();
    let mut pins = Vec::with_capacity(n);
    for k in 0..n {
        let u = lp[k];
        let (start, end) = ends(u);
        let edge = &edges[u.0];
        let curve = if u.1 {
            edge.curve.clone()
        } else {
            edge.curve.reverse()
        };
        let pin =
            |corner: Option<Vector2<S>>, vertex: usize, at_start: bool| -> GeopResult<Vector2<S>> {
                match corner {
                    Some(uv) => Ok(uv),
                    None => {
                        // At a pole the free parameter names the same point
                        // whatever it is: take the one the curve arrives along,
                        // from a point of it near the pole.
                        let (lo, hi) = curve.domain();
                        let f = S::from_ratio(1, 64)?;
                        let t = if at_start {
                            lo.add(hi.sub(lo).mul(f))
                        } else {
                            hi.sub(hi.sub(lo).mul(f))
                        };
                        let near = curve.evaluate(t.sharpen())?;
                        let (pu, pv) = grid.project(surface, &near)?;
                        let pole = pole_of(vertex, vertices)
                            .expect("a corner without a foot point is at a pole");
                        Ok(if pole.fixes_u {
                            Vector2::from_array([pole.at, pv])
                        } else {
                            Vector2::from_array([pu, pole.at])
                        })
                    }
                }
            };
        let pin_start = pin(corners[(k + n - 1) % n], start, true)?;
        let pin_end = pin(corners[k], end, false)?;
        let ctx = |e: GeopError| {
            e.with_context(format!("fitting the pcurve of the edge {}", edge.label()))
        };
        let pcurve = if patch.planar {
            plane_pcurve(surface, &curve, pin_start, pin_end).with_context(&ctx)?
        } else {
            surface
                .fit_pcurve(
                    &curve,
                    Some(pin_start),
                    Some(pin_end),
                    MAX_NODES,
                    S::from_f64(MIN_SUBDIVISION_SIZE),
                )
                .with_context(&ctx)?
        };
        out.push((CoedgeOnLocal::Edge(u.0, u.1), pcurve));
        pins.push((pin_start, pin_end));
    }
    // Every place each vertex is said to be, united.
    for k in 0..n {
        let u = lp[k];
        let (start, end) = ends(u);
        let edge = &edges[u.0];
        let (lo, hi) = edge.curve.domain();
        let (curve_start, curve_end) = if u.1 {
            (edge.curve.evaluate(lo)?, edge.curve.evaluate(hi)?)
        } else {
            (edge.curve.evaluate(hi)?, edge.curve.evaluate(lo)?)
        };
        // The pins, not the pcurve's ends: those are widened by the fit's
        // drift, which says where the pcurve may be, not the vertex.
        let (a, b) = pins[k];
        let on_start = surface.evaluate(a[0], a[1])?;
        let on_end = surface.evaluate(b[0], b[1])?;
        for (v, extra) in [(start, [curve_start, on_start]), (end, [curve_end, on_end])] {
            for (p, source) in extra.into_iter().zip([
                "the end of the edge",
                "its foot point on this face, at the end of the edge",
            ]) {
                let off = distance(to_p3(&p), to_p3(&vertices[v].origin));
                if off > ACCURACY {
                    return Err(GeopError::new(format!(
                        "the vertex {} is at {:?}, but {source} {} is {off:e} mm from it — more than the {ACCURACY:e} mm the kernel can carry as one point (the face built over the angles and profile parameters {:?})",
                        vertices[v].label(),
                        to_p3(&vertices[v].origin),
                        edge.label(),
                        patch.extent,
                    )));
                }
                vertices[v].point = vertices[v].point.union(&p);
            }
        }
    }
    // Coedges meeting at a pole, where their pcurves end at different `u`,
    // are joined along the pole's row.
    let mut joined = Vec::with_capacity(out.len());
    for k in 0..n {
        joined.push(out[k].clone());
        let next = (k + 1) % n;
        let (_, end) = ends(lp[k]);
        if corners[k].is_none() {
            let (_, t1) = out[k].1.domain();
            let (t0, _) = out[next].1.domain();
            let from = out[k].1.evaluate(t1)?;
            let to = out[next].1.evaluate(t0)?;
            if !from.could_be_equal(&to) {
                let h = |p: Vector2<S>| {
                    geop_core_math::vector::Vector3::from_array([p[0], p[1], S::ONE])
                };
                let pcurve = NurbCurve::try_new(
                    1,
                    vec![h(from), h(to)],
                    vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                )?;
                joined.push((CoedgeOnLocal::Vertex(end), pcurve));
            }
        }
    }
    Ok(joined)
}

/// The nearest point of `surface` to `p`, and the surface's unit normal
/// there — none at an apex, where it has no single one.
fn foot<S: Scalar>(
    surface: &NurbSurface3D<S>,
    grid: &Grid<S>,
    p: P3,
) -> GeopResult<(P3, Option<P3>)> {
    let (u, v) = sharp_inside(surface, grid.project(surface, &s3(p))?);
    Ok((
        to_p3(&surface.evaluate(u, v)?),
        surface.normal(u, v).ok().map(|n| to_p3(&n)),
    ))
}

/// A point of the enclosure `(u, v)` of a foot point on `surface`, sharp:
/// which is a free choice — one in the domain, which its midpoint may not
/// be by a rounding at its end.
fn sharp_inside<S: Scalar>(surface: &NurbSurface3D<S>, (u, v): (S, S)) -> (S, S) {
    let inside = |t: S, (lo, hi): (S, S)| {
        let t = t.sharpen();
        if t.to_f64() < lo.to_f64() {
            lo
        } else if t.to_f64() > hi.to_f64() {
            hi
        } else {
            t
        }
    };
    (inside(u, surface.domain_u()), inside(v, surface.domain_v()))
}

/// How far `p` lies from `surface`.
fn off<S: Scalar>(surface: &NurbSurface3D<S>, grid: &Grid<S>, p: P3) -> GeopResult<f64> {
    Ok(distance(foot(surface, grid, p)?.0, p))
}

/// Samples along an edge telling whether it strays from its faces.
const STRAY_SAMPLES: usize = 16;

/// How far `curve` strays from `surface`, sampled: the first sample's foot
/// point found from the grid, each next one's by Newton from the one before
/// — a walk along the curve, as `fit_pcurve` takes.
fn strays<S: Scalar>(
    curve: &Midpoints,
    surface: &NurbSurface3D<S>,
    grid: &Grid<S>,
) -> GeopResult<f64> {
    let (lo, hi) = curve.domain();
    let mut seed: Option<(S, S)> = None;
    let mut most = 0.0f64;
    for k in 0..=STRAY_SAMPLES {
        let p = curve.point(lo + (hi - lo) * k as f64 / STRAY_SAMPLES as f64);
        let (u, v) = sharp_inside(
            surface,
            match seed {
                None => grid.project(surface, &s3(p))?,
                Some((u, v)) => surface.project(s3(p), u, v, 30)?,
            },
        );
        seed = Some((u, v));
        most = most.max(distance(to_p3(&surface.evaluate(u, v)?), p));
    }
    Ok(most)
}

/// Newton steps taken towards where surfaces meet.
const MEET_ITERATIONS: usize = 40;

/// How much a step towards where surfaces meet is held back along a
/// direction they barely constrain (Levenberg–Marquardt damping, against
/// unit normals): where surfaces meet at an angle under a thousandth of a
/// radian, the point stays near where it was along their tangent instead of
/// running off along it. A free choice of which point, among those the
/// surfaces leave free; whether it lies on them is checked after.
const MEET_DAMPING: f64 = 1e-6;

/// The point near `start` lying on every one of `surfaces`, where they
/// meet: Newton's step on their tangent planes, from `start`, each moving
/// the point as little as satisfies them. Whether it got there is for the
/// caller to check ([`off`]): surfaces that do not meet leave it where they
/// come nearest to each other.
fn meet<S: Scalar>(start: P3, surfaces: &[(&NurbSurface3D<S>, &Grid<S>)]) -> GeopResult<P3> {
    let mut x = start;
    for _ in 0..MEET_ITERATIONS {
        // (Σ n nᵀ + μ I) dx = -Σ n (n · (x - f)).
        let mut a: [[f64; 3]; 3] = std::array::from_fn(|i| {
            std::array::from_fn(|j| if i == j { MEET_DAMPING } else { 0.0 })
        });
        let mut b = [0.0f64; 3];
        for (surface, grid) in surfaces {
            let (f, normal) = foot(surface, grid, x)?;
            // Off the surface, the way to its nearest point is its normal
            // there, whatever the surface's own: what an apex has instead.
            let Some(n) = normal.or_else(|| normalize(sub(x, f))) else {
                continue;
            };
            let r = dot(n, sub(x, f));
            for i in 0..3 {
                for j in 0..3 {
                    a[i][j] += n[i] * n[j];
                }
                b[i] -= n[i] * r;
            }
        }
        let det = |m: [[f64; 3]; 3]| dot(m[0], cross(m[1], m[2]));
        let d = det(a);
        // Cramer's rule: `a` is symmetric positive definite.
        let dx: P3 = std::array::from_fn(|c| {
            let mut m = a;
            for (row, value) in m.iter_mut().zip(b) {
                row[c] = value;
            }
            det(m) / d
        });
        let next = add(x, dx);
        if next == x {
            break;
        }
        x = next;
    }
    Ok(x)
}

/// Samples along an edge rebuilt where its faces' surfaces meet, at first
/// and at most.
const REBUILT_SAMPLES: usize = 16;
const MOST_REBUILT_SAMPLES: usize = 256;

/// Where the file's vertices and edges lie further from the surfaces of
/// their faces than the kernel can carry as one point or one curve (see
/// [`ACCURACY`]), they are rebuilt from those surfaces, which a B-rep is
/// defined by: a vertex where the surfaces of its faces meet nearest to the
/// file's point, an edge through where the surfaces of its faces meet along
/// it, between its vertices. Nearer, the file's places are kept and united
/// (see `fit_loop` and the edges' widening in `Builder::spec`), as the
/// enclosure of one point or curve.
///
/// The file's uncertainty bounds what is rebuilt: the surfaces have to meet
/// to within it — they are the file's statement of where the solid is, to
/// its accuracy. Where they contradict each other — those of a vertex's
/// faces do not meet near it, or those of an edge's along it — the
/// smallest of the faces there are left out, one by one, until the rest
/// meet: their surfaces are rebuilt from their edges instead (see
/// [`rebuilt_surface`]), and returned as such. A face that small is one the
/// file approximated more coarsely than its neighbours: a narrow wall
/// running tangent into a hole, say. An edge between such faces only is
/// moved with its vertices. Refused, by name, where a vertex or an edge
/// would move further than half the length of an edge there: that is no
/// longer the same vertex or edge.
fn heal<S: Scalar>(
    faces: &[Face],
    built: &[(Patch<S>, NurbSurface3D<S>, Grid<S>)],
    edges: &mut [Edge<S>],
    vertices: &mut [Vertex<S>],
    scope: &Scope,
) -> GeopResult<(Vec<bool>, Healing)> {
    let mut faces_of_edge: Vec<Vec<usize>> = vec![Vec::new(); edges.len()];
    let mut faces_of_vertex: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    let mut edges_of_vertex: Vec<Vec<usize>> = vec![Vec::new(); vertices.len()];
    for (f, face) in faces.iter().enumerate() {
        for &(e, _) in face.loops.iter().flatten() {
            let add = |list: &mut Vec<usize>, item: usize| {
                if !list.contains(&item) {
                    list.push(item);
                }
            };
            add(&mut faces_of_edge[e], f);
            for v in [edges[e].start, edges[e].end] {
                add(&mut faces_of_vertex[v], f);
                add(&mut edges_of_vertex[v], e);
            }
        }
    }
    let surfaces = |fs: &[usize]| -> Vec<(&NurbSurface3D<S>, &Grid<S>)> {
        fs.iter().map(|&f| (&built[f].1, &built[f].2)).collect()
    };
    let names = |fs: &[usize]| -> String {
        fs.iter()
            .map(|&f| format!("{} (#{})", faces[f].name.join(","), faces[f].id))
            .collect::<Vec<_>>()
            .join(", ")
    };
    // How far `p` lies from each of `fs`, at most.
    let furthest = |p: P3, fs: &[usize]| -> GeopResult<f64> {
        let mut most = 0.0f64;
        for (surface, grid) in surfaces(fs) {
            most = most.max(off(surface, grid, p)?);
        }
        Ok(most)
    };
    let chord = |e: &Edge<S>, vertices: &[Vertex<S>]| {
        distance(
            to_p3(&vertices[e.start].origin),
            to_p3(&vertices[e.end].origin),
        )
    };
    // How large each face is, to tell which to leave out first: the area
    // its loops' samples enclose, as vectors summed — the face's own area
    // where it is flat, about it where it is not.
    let mut size = Vec::with_capacity(faces.len());
    for face in faces {
        let mut area = [0.0f64; 3];
        for lp in &face.loops {
            let mut points = Vec::new();
            for &(e, forward) in lp {
                let curve = Midpoints::of(&edges[e].curve);
                let (lo, hi) = curve.domain();
                for i in 0..LOOP_SAMPLES {
                    let f = i as f64 / LOOP_SAMPLES as f64;
                    let f = if forward { f } else { 1.0 - f };
                    points.push(curve.point(lo + (hi - lo) * f));
                }
            }
            for (k, &p) in points.iter().enumerate() {
                area = add(area, scale(cross(p, points[(k + 1) % points.len()]), 0.5));
            }
        }
        size.push(norm(area));
    }

    // Of the faces `fs`, those whose surfaces are kept: all but those to
    // be rebuilt, the smallest first.
    let kept = |fs: &[usize], rebuild: &[bool]| -> Vec<usize> {
        let mut kept: Vec<usize> = fs.iter().copied().filter(|&f| !rebuild[f]).collect();
        kept.sort_by(|&a, &b| size[a].total_cmp(&size[b]));
        kept
    };
    // Where the surfaces of the faces `fs` meet within `reach` of `p` —
    // where they contradict each other, the least area of faces that can be
    // rebuilt (see `rebuilt_surface`) left out, to be rebuilt, so that the
    // rest do. `None` where all of them are rebuilt; an error naming them
    // where no choice serves.
    let settle = |p: P3,
                  fs: &[usize],
                  reach: f64,
                  rebuild: &mut [bool]|
     -> GeopResult<Option<P3>> {
        let kept = kept(fs, rebuild);
        let free: Vec<usize> = kept
            .iter()
            .copied()
            .filter(|&f| rebuildable(&faces[f]))
            .take(MOST_REBUILT_AT_ONCE)
            .collect();
        let mut choices: Vec<(f64, Vec<usize>)> = (0..1usize << free.len())
            .map(|mask| {
                let out: Vec<usize> = (0..free.len())
                    .filter(|k| mask & (1 << k) != 0)
                    .map(|k| free[k])
                    .collect();
                (out.iter().map(|&f| size[f]).sum(), out)
            })
            .collect();
        choices.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut nearest = None;
        for (_, out) in choices {
            let rest: Vec<usize> = kept.iter().copied().filter(|f| !out.contains(f)).collect();
            let healed = if rest.is_empty() {
                None
            } else {
                let healed = meet(p, &surfaces(&rest))?;
                let left = furthest(healed, &rest)?;
                nearest.get_or_insert((left, healed));
                if left > scope.uncertainty || distance(healed, p) > reach {
                    continue;
                }
                Some(healed)
            };
            for f in out {
                rebuild[f] = true;
            }
            return Ok(healed);
        }
        let (left, healed) = nearest.unwrap_or((0.0, p));
        Err(GeopError::new(format!(
            "the surfaces of its faces {} {} — and no choice of the faces of one loop of four edges or more among them, rebuilt from their edges instead, leaves the rest meeting there",
            names(&kept),
            if left > scope.uncertainty {
                format!(
                    "do not meet near {p:?} to within the file's uncertainty of {:e} mm: they come no nearer each other than {left:e} mm, at {healed:?}",
                    scope.uncertainty
                )
            } else {
                format!(
                    "meet only {:e} mm from {p:?}, at {healed:?}, more than the {reach:e} mm it may move: half the shortest edge there",
                    distance(healed, p)
                )
            },
        )))
    };

    let mut rebuild = vec![false; faces.len()];
    let mut moved = vec![false; vertices.len()];
    // What was moved, and how far, for the report.
    let mut moved_vertices: Vec<(String, f64)> = Vec::new();
    let mut moved_edges: Vec<(String, f64)> = Vec::new();
    // How far each vertex and edge lies from the surfaces of its faces kept,
    // as the first pass measured it: the second measures again only where
    // a face of it is to be rebuilt since.
    let mut vertex_apart: Vec<Option<(Vec<usize>, f64)>> = vec![None; vertices.len()];
    let mut edge_apart: Vec<Option<(Vec<usize>, f64)>> = vec![None; edges.len()];
    // Twice: first deciding which faces are rebuilt, everywhere, then
    // moving what is to move onto the surfaces kept — so that a vertex is
    // not put where a surface meets the others that a later vertex finds
    // contradicting its neighbours.
    for apply in [false, true] {
        moved.fill(false);
        for v in 0..vertices.len() {
            if faces_of_vertex[v].is_empty() {
                continue;
            }
            let at = to_p3(&vertices[v].origin);
            let what = format!("healing the vertex {} at {at:?}", vertices[v].label());
            let vctx = |e: GeopError| e.with_context(what.clone());
            let fs = kept(&faces_of_vertex[v], &rebuild);
            let apart = match &vertex_apart[v] {
                Some((measured, apart)) if *measured == fs => *apart,
                _ => furthest(at, &fs).map_err(&vctx)?,
            };
            vertex_apart[v] = Some((fs.clone(), apart));
            if 2.0 * apart <= ACCURACY {
                continue;
            }
            let shortest = edges_of_vertex[v]
                .iter()
                .map(|&e| chord(&edges[e], vertices))
                .fold(f64::INFINITY, f64::min);
            let Some(healed) = settle(at, &fs, shortest / 2.0, &mut rebuild).map_err(&vctx)? else {
                // Every face of it rebuilt: through it, where it is.
                continue;
            };
            moved[v] = true;
            if !apply {
                continue;
            }
            vertices[v].origin = s3(healed);
            vertices[v].point = s3(healed);
            moved_vertices.push((vertices[v].label(), distance(healed, at)));
        }

        for e in 0..edges.len() {
            if faces_of_edge[e].is_empty() || !edges[e].alive {
                continue;
            }
            let what = format!("healing the edge {}", edges[e].label());
            let ectx = |e: GeopError| e.with_context(what.clone());
            let old = edges[e].curve.clone();
            let (lo, hi) = old.domain();
            let (lo, hi) = (lo.to_f64(), hi.to_f64());
            let at = |f: f64| super::geometry::point_at(&old, lo + (hi - lo) * f);
            let (start, end) = (edges[e].start, edges[e].end);
            let fs = kept(&faces_of_edge[e], &rebuild);
            let apart = match &edge_apart[e] {
                Some((measured, apart)) if *measured == fs => *apart,
                _ => {
                    let midpoints = Midpoints::of(&old);
                    let mut apart = 0.0f64;
                    for (surface, grid) in surfaces(&fs) {
                        apart = apart.max(strays(&midpoints, surface, grid).map_err(&ectx)?);
                    }
                    apart
                }
            };
            edge_apart[e] = Some((fs.clone(), apart));
            if !moved[start] && !moved[end] && 2.0 * apart <= ACCURACY {
                continue;
            }
            // Which faces' surfaces the edge is rebuilt on: decided at its
            // middle.
            let length = chord(&edges[e], vertices);
            let middle =
                settle(at(0.5).map_err(&ectx)?, &fs, length / 2.0, &mut rebuild).map_err(&ectx)?;
            if !apply {
                continue;
            }
            let fs = kept(&fs, &rebuild);
            // How far its ends move with its vertices.
            let ends = [at(0.0).map_err(&ectx)?, at(1.0).map_err(&ectx)?];
            let shifts = [
                sub(to_p3(&vertices[start].origin), ends[0]),
                sub(to_p3(&vertices[end].origin), ends[1]),
            ];
            // Where the old curve's point `p`, `f` of the way along it, goes:
            // where the kept surfaces meet, or — none kept — along with its
            // vertices.
            let furthest_shift = std::cell::Cell::new(0.0f64);
            let onto = |p: P3, f: f64| -> GeopResult<P3> {
                if middle.is_none() {
                    let moved = add(p, add(scale(shifts[0], 1.0 - f), scale(shifts[1], f)));
                    furthest_shift.set(furthest_shift.get().max(distance(moved, p)));
                    return Ok(moved);
                }
                let healed = meet(p, &surfaces(&fs))?;
                let left = furthest(healed, &fs)?;
                let shift = distance(healed, p);
                furthest_shift.set(furthest_shift.get().max(shift));
                if left > scope.uncertainty || 2.0 * shift > length {
                    return Err(GeopError::new(format!(
                        "the edge {} lies {apart:e} mm off the surfaces of its faces {}{}, and they {} — so it cannot be rebuilt where they meet",
                        edges[e].label(),
                        names(&fs),
                        if moved[start] || moved[end] {
                            ", and a vertex of it was moved to where they meet"
                        } else {
                            ""
                        },
                        if left > scope.uncertainty {
                            format!(
                                "do not meet along it to within the file's uncertainty of {:e} mm: near its point {p:?} they come no nearer each other than {left:e} mm",
                                scope.uncertainty
                            )
                        } else {
                            format!(
                                "meet only {shift:e} mm from its point {p:?}, more than half its length of {length:e} mm"
                            )
                        },
                    )));
                }
                Ok(healed)
            };
            // Sampled more densely while the cubic through the samples strays
            // between them from where the surfaces meet by more than the
            // file's uncertainty.
            let mut samples = REBUILT_SAMPLES;
            let rebuilt = loop {
                let mut points = vec![vertices[start].origin];
                for k in 1..samples {
                    let f = k as f64 / samples as f64;
                    points.push(s3(onto(at(f).map_err(&ectx)?, f).map_err(&ectx)?));
                }
                points.push(vertices[end].origin);
                let rebuilt = NurbCurve3D::interpolate(&points, 3).map_err(|err| {
                    err.with_context(format!(
                        "rebuilding the edge {} where its faces {} meet",
                        edges[e].label(),
                        names(&fs)
                    ))
                })?;
                if samples >= MOST_REBUILT_SAMPLES || middle.is_none() {
                    break rebuilt;
                }
                let (r0, r1) = rebuilt.domain();
                let (r0, r1) = (r0.to_f64(), r1.to_f64());
                let mut strays = 0.0f64;
                for k in 0..samples {
                    let t = r0 + (r1 - r0) * (k as f64 + 0.5) / samples as f64;
                    let p = super::geometry::point_at(&rebuilt, t).map_err(&ectx)?;
                    strays = strays.max(furthest(p, &fs).map_err(&ectx)?);
                }
                if strays <= scope.uncertainty {
                    break rebuilt;
                }
                samples *= 2;
            };
            edges[e].curve = rebuilt;
            moved_edges.push((edges[e].label(), furthest_shift.get()));
        }
    }
    let healing = Healing {
        vertices: moved_vertices,
        edges: moved_edges,
        faces: (0..faces.len())
            .filter(|&f| rebuild[f])
            .map(|f| names(&[f]))
            .collect(),
        uncertainty: scope.uncertainty,
    };
    Ok((rebuild, healing))
}

/// The most faces at one vertex or edge among which [`heal`] chooses which to
/// rebuild: every choice is tried.
const MOST_REBUILT_AT_ONCE: usize = 6;

/// Whether [`rebuilt_surface`] can rebuild the surface of `face`.
fn rebuildable(face: &Face) -> bool {
    matches!(face.loops.as_slice(), [lp] if lp.len() >= 4)
}

/// The surface of `face` rebuilt from its edges, where its own contradicts
/// its neighbours' (see [`heal`]): the Coons patch of its loop, cut into
/// four sides at the four corners where it turns most sharply — a narrow
/// wall's two long edges and its two short ones. Which corners is a free
/// choice; the patch passes through every edge whichever it is. Its normal
/// is the one the loop winds about, which runs counter-clockwise about the
/// face's outward normal. Only for a face of one loop of four edges or
/// more.
fn rebuilt_surface<S: Scalar>(face: &Face, edges: &[Edge<S>]) -> GeopResult<NurbSurface3D<S>> {
    let [lp] = face.loops.as_slice() else {
        return Err(GeopError::new(format!(
            "it has {} loops: only a face of one loop is rebuilt from its edges",
            face.loops.len()
        )));
    };
    let n = lp.len();
    if n < 4 {
        return Err(GeopError::new(format!(
            "its loop has {n} edges: only one of four or more is rebuilt from its edges, cut into four sides"
        )));
    }
    let curves: Vec<NurbCurve3D<S>> = lp
        .iter()
        .map(|&(e, forward)| {
            if forward {
                edges[e].curve.clone()
            } else {
                edges[e].curve.reverse()
            }
        })
        .collect();
    let direction = |c: &NurbCurve3D<S>, at_end: bool| -> GeopResult<P3> {
        let (lo, hi) = c.domain();
        let t = if at_end { hi } else { lo };
        normalize(to_p3(&c.tangent(t)?))
            .ok_or_else(|| GeopError::new("an edge of it has no direction at an end"))
    };
    // How sharply the loop turns at the end of each coedge.
    let mut turns = Vec::with_capacity(n);
    for k in 0..n {
        let arriving = direction(&curves[k], true)?;
        let leaving = direction(&curves[(k + 1) % n], false)?;
        turns.push((1.0 - dot(arriving, leaving), k));
    }
    turns.sort_by(|a, b| b.0.total_cmp(&a.0));
    let mut corners: Vec<usize> = turns[..4].iter().map(|&(_, k)| k).collect();
    corners.sort();
    let mut sides = Vec::with_capacity(4);
    for j in 0..4 {
        let (from, to) = (corners[j] + 1, corners[(j + 1) % 4]);
        let count = (to + n - from) % n + 1;
        let chain: Vec<NurbCurve3D<S>> =
            (0..count).map(|i| curves[(from + i) % n].clone()).collect();
        sides.push(NurbCurve3D::join(&chain)?);
    }
    NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]])
}

/// The pcurve of `curve` on `surface`, a plane's parallelogram patch
/// `P00 + u e1 + v e2` on `[0, 1]²`: the curve in the plane's coordinates,
/// exact — its homogeneous control points mapped by the affine map's
/// inverse, its weights and knots its own — and pinned at `start` and `end`,
/// its ends' foot points, as every pcurve of a loop is (see `fit_loop`).
/// Fitted instead, a cubic through samples of a large circle drifts from it
/// by more than the kernel's accuracy.
fn plane_pcurve<S: Scalar>(
    surface: &NurbSurface3D<S>,
    curve: &NurbCurve3D<S>,
    start: Vector2<S>,
    end: Vector2<S>,
) -> GeopResult<NurbCurve2D<S>> {
    let corner = |i: usize, j: usize| {
        let cp = surface.control_points[i * surface.num_v + j];
        Vector3::from_array([cp[0], cp[1], cp[2]])
    };
    let origin = corner(0, 0);
    let (e1, e2) = (corner(1, 0).sub(&origin), corner(0, 1).sub(&origin));
    let (a, b, c) = (e1.prod_dot(&e1), e1.prod_dot(&e2), e2.prod_dot(&e2));
    let det = a.mul(c).sub(b.mul(b));
    let n = curve.control_points.len();
    let mut control_points = Vec::with_capacity(n);
    for (k, cp) in curve.control_points.iter().enumerate() {
        let w = cp[3];
        let pin = match k {
            0 => Some(start),
            _ if k + 1 == n => Some(end),
            _ => None,
        };
        let (u, v) = match pin {
            Some(p) => (p[0].mul(w), p[1].mul(w)),
            None => {
                let d = Vector3::from_array([cp[0], cp[1], cp[2]]).sub(&origin.prod_scalar(w));
                let (s1, s2) = (d.prod_dot(&e1), d.prod_dot(&e2));
                (
                    c.mul(s1).sub(b.mul(s2)).div(det)?,
                    a.mul(s2).sub(b.mul(s1)).div(det)?,
                )
            }
        };
        control_points.push(Vector3::from_array([u, v, w]));
    }
    NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())
}

/// The widest a vertex or a control point may be in the kernel (its
/// validation's bound): places the file says are one point lying further
/// apart than this cannot be one geop vertex.
const ACCURACY: f64 = 1e-4;

/// How often an edge is measured against its faces and widened: the first
/// time by the file's disagreement, after by rounding.
const WIDENINGS: usize = 4;

/// Samples along an edge's pcurve, at fractions of its domain: those the
/// kernel's validation samples at among them.
const GAP_SAMPLES: usize = 64;

/// Golden-section steps finding the nearest point of an edge to a pcurve's
/// sample: from two table steps down to a rounding of the parameter.
const NEAREST_ITERATIONS: usize = 80;

/// Of those, the last ones, taken on the curve's enclosure rather than its
/// midpoints in `f64`.
const NEAREST_ON_ENCLOSURE: usize = 12;

/// How far, per coordinate, the points a pcurve puts on its surface lie
/// from the edge's curve `curve` — sampled, each against the curve's
/// nearest point: how far the point's midpoint, the pcurve's own best
/// guess, lies outside the curve's enclosure there.
fn edge_gap<S: Scalar>(
    curve: &NurbCurve3D<S>,
    surface: &NurbSurface3D<S>,
    pcurve: &NurbCurve2D<S>,
) -> GeopResult<([f64; 3], Option<Worst>)> {
    let (t0, t1) = pcurve.domain();
    let (c0, c1) = curve.domain();
    let mut gap = [0.0f64; 3];
    let mut worst: Option<Worst> = None;
    // Each sample's nearest curve point: the nearest of a table of the
    // curve's points, polished by Newton.
    let (c0f, c1f) = (c0.to_f64(), c1.to_f64());
    let midpoints = Midpoints::of(curve);
    let rows = 8 * curve.control_points.len() + 32;
    let table: Vec<(f64, P3)> = (0..=rows)
        .map(|k| {
            let t = c0f + (c1f - c0f) * k as f64 / rows as f64;
            (t, midpoints.point(t))
        })
        .collect();
    let step = (c1f - c0f) / rows as f64;
    for i in 0..=GAP_SAMPLES {
        let f = S::from_ratio(i as i64, GAP_SAMPLES as i64)?;
        let t = match i {
            0 => t0,
            GAP_SAMPLES => t1,
            _ => t0.add(t1.sub(t0).mul(f)).sharpen(),
        };
        let uv = pcurve.evaluate(t)?;
        // A pcurve too uncertain to place on the surface at all is for the
        // model's validation to judge; it says nothing about the edge.
        let Ok(point) = surface.evaluate(uv[0], uv[1]) else {
            continue;
        };
        let target = to_p3(&point);
        let &(seed, _) = table
            .iter()
            .min_by(|a, b| distance(a.1, target).total_cmp(&distance(b.1, target)))
            .expect("a table of points");
        // Which parameter is a free choice — a nearer point only measures
        // a smaller gap — so it is found in `f64`, on the curve's
        // midpoints, by golden section between the table's neighbours of
        // the nearest. (Newton's refinement gives its window back for a
        // point at an end, reached a rounding past it, and the window's
        // middle is half a step off; on a widened curve its last step
        // inherits the width, and is off along the curve by about that.)
        // The midpoints in `f64` narrow the window, which is cheap; the
        // enclosure's own midpoints, which the gap is measured against,
        // decide its last bits, where the two differ by roundings.
        let (mut a, mut b) = ((seed - step).max(c0f), (seed + step).min(c1f));
        let gap_at = |t: f64, k: usize| -> GeopResult<f64> {
            if k + NEAREST_ON_ENCLOSURE < NEAREST_ITERATIONS {
                Ok(distance(midpoints.point(t), target))
            } else {
                Ok(distance(super::geometry::point_at(curve, t)?, target))
            }
        };
        let ratio = (5f64.sqrt() - 1.0) / 2.0;
        let (mut x, mut y) = (b - ratio * (b - a), a + ratio * (b - a));
        let (mut fx, mut fy) = (gap_at(x, 0)?, gap_at(y, 0)?);
        for k in 0..NEAREST_ITERATIONS {
            if k + NEAREST_ON_ENCLOSURE == NEAREST_ITERATIONS {
                // From here on, on the enclosure.
                (fx, fy) = (gap_at(x, k)?, gap_at(y, k)?);
            }
            if fx <= fy {
                b = y;
                (y, fy) = (x, fx);
                x = b - ratio * (b - a);
                fx = gap_at(x, k)?;
            } else {
                a = x;
                (x, fx) = (y, fy);
                y = a + ratio * (b - a);
                fy = gap_at(y, k)?;
            }
        }
        // The ends are the domain's own. Which of the three is nearest is
        // told on the curve's enclosure, which is what the gap is measured
        // against: its midpoints can call an interior point next to an end
        // as near as the end, where the enclosure there holds the point
        // and the end's does not.
        let candidates = [
            (c0f, c0),
            (c1f, c1),
            ((a + b) / 2.0, S::from_f64((a + b) / 2.0)),
        ];
        let exact_gap_at = |t: f64| -> GeopResult<f64> {
            Ok(distance(super::geometry::point_at(curve, t)?, target))
        };
        let mut s = candidates[2].1;
        let mut best = exact_gap_at(candidates[2].0)?;
        for &(t, exact) in &candidates[..2] {
            let d = exact_gap_at(t)?;
            if d < best {
                (s, best) = (exact, d);
            }
        }
        let near = curve.evaluate(s)?;
        // How far the point lies outside the curve's enclosure: what the
        // curve must widen by to reach it.
        for c in 0..3 {
            // Up to the point's midpoint, not merely its enclosure: an edge
            // that only touches its pcurve's points, by a rounding, is
            // not found to pass through them by the validation's clipping.
            let mid = point[c].midpoint();
            let apart = (mid.sub(near[c].upper()).upper().to_f64())
                .max(near[c].lower().sub(mid).upper().to_f64())
                .max(0.0);
            gap[c] = gap[c].max(apart);
            if worst.is_none_or(|w: Worst| apart > w.apart) {
                worst = Some(Worst {
                    apart,
                    at: i as f64 / GAP_SAMPLES as f64,
                    point: to_p3(&point),
                    near: to_p3(&near),
                });
            }
        }
    }
    Ok((gap, worst))
}

/// Where along a pcurve [`edge_gap`] found it furthest from its edge: the
/// fraction of its domain, the point it puts on the surface there, and the
/// edge's nearest point.
#[derive(Clone, Copy, Debug)]
struct Worst {
    apart: f64,
    at: f64,
    point: P3,
    near: P3,
}

/// How far the points of `curve` lie from `surface`, sampled: each
/// projected onto it, from where `pcurve`, the curve's on the surface,
/// is at the same fraction of its domain.
fn off_surface<S: Scalar>(
    curve: &NurbCurve3D<S>,
    surface: &NurbSurface3D<S>,
    pcurve: &NurbCurve2D<S>,
) -> GeopResult<f64> {
    let (c0, c1) = curve.domain();
    let (t0, t1) = pcurve.domain();
    let mut most = 0.0f64;
    for i in 0..=GAP_SAMPLES {
        let f = i as f64 / GAP_SAMPLES as f64;
        let p = curve.evaluate(S::from_f64(c0.to_f64() + (c1.to_f64() - c0.to_f64()) * f))?;
        let seed = pcurve.evaluate(S::from_f64(t0.to_f64() + (t1.to_f64() - t0.to_f64()) * f))?;
        let (u, v) = surface.project(p, seed[0].sharpen(), seed[1].sharpen(), 30)?;
        let foot = surface.evaluate(u, v)?;
        most = most.max(distance(to_p3(&foot), to_p3(&p)));
    }
    Ok(most)
}

/// `curve` with every control point widened by `gap` in each coordinate:
/// a curve whose every point may be up to `gap` off where it was.
fn widened<S: Scalar>(curve: &NurbCurve3D<S>, gap: [f64; 3]) -> GeopResult<NurbCurve3D<S>> {
    if gap == [0.0; 3] {
        return Ok(curve.clone());
    }
    let control_points = curve
        .control_points
        .iter()
        .map(|cp| {
            let w = cp[3];
            let mut out = *cp;
            for c in 0..3 {
                let spread = S::from_f64(-gap[c]).union(S::from_f64(gap[c]));
                out[c] = cp[c].add(w.mul(spread));
            }
            out
        })
        .collect();
    NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())
}

/// The signed area a loop of pcurves encloses in the patch: positive
/// counter-clockwise.
fn signed_area<S: Scalar>(lp: &[(CoedgeOnLocal, NurbCurve2D<S>)]) -> GeopResult<f64> {
    let mut points = Vec::new();
    for (_, pcurve) in lp {
        let (t0, t1) = pcurve.domain();
        let (t0, t1) = (t0.to_f64(), t1.to_f64());
        for i in 0..LOOP_SAMPLES {
            let t = t0 + (t1 - t0) * i as f64 / LOOP_SAMPLES as f64;
            let p = pcurve.evaluate(S::from_f64(t))?;
            points.push([p[0].to_f64(), p[1].to_f64()]);
        }
    }
    let n = points.len();
    let mut area = 0.0;
    for i in 0..n {
        let (a, b) = (points[i], points[(i + 1) % n]);
        area += a[0] * b[1] - b[0] * a[1];
    }
    Ok(area / 2.0)
}
