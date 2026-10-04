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
//!   patch of its own (see [`Builder::cut_wrapping_face`]).
//! - **Closed edges.** A STEP edge may start and end at one vertex — a whole
//!   circle. A geop edge may not, so one is split in two.
//! - **Tolerance.** A STEP file's vertices, curves and surfaces agree only
//!   to within the file's accuracy. A geop vertex is an enclosure, so each
//!   is the union of every place the file says it is: its point, the ends
//!   of its edges' curves, and its foot points on its faces' surfaces (see
//!   `AGENTS.md` on combining enclosures of one value with `union`).
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
        P3, Profile, Revolved, Scope, SurfaceDef, SurfaceKind, distance, dot,
        normalize, scale, to_p3,
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

struct Vertex<S: Scalar> {
    point: Vector3<S>,
    name: Vec<String>,
}

struct Edge<S: Scalar> {
    curve: NurbCurve3D<S>,
    start: usize,
    end: usize,
    name: Vec<String>,
    /// False once split into pieces or found to be a seam.
    alive: bool,
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
                let (_, o) = reader.args_of_any(shell, &["ORIENTED_OPEN_SHELL", "ORIENTED_CLOSED_SHELL"])?;
                builder.shell(o.reference(2)?, !o.logical(3)?)?;
            } else {
                builder.shell(shell, false)?;
            }
        }
    }
    builder.normalize()?;
    let (spec, vertex_names, edge_names, face_names) = builder.spec(solid)?;
    Ok(ImportedBody {
        label: item.label.clone(),
        spec,
        vertex_names,
        edge_names,
        face_names,
    })
}

impl<S: Scalar> Builder<'_, S> {
    fn shell(&mut self, id: u64, reversed: bool) -> GeopResult<()> {
        let (_, args) = self.reader.args_of_any(id, &["CLOSED_SHELL", "OPEN_SHELL"])?;
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
            name: name("v", self.vertex_index.len()),
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
        let (curve, reversed) = self.reader.curve(&self.scope, args.reference(3)?).map_err(ctx)?;
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
        let (_, args) = self.reader.args_of_any(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
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
            let (_, b) = self.reader.args_of_any(bound, &["FACE_OUTER_BOUND", "FACE_BOUND"])?;
            let lp = b.reference(1)?;
            let forward = b.logical(2)? != reversed;
            let lp_instance = self.reader.instance(lp)?;
            if lp_instance.is("VERTEX_LOOP") {
                // A loop of one vertex marks a pole or an apex, which the
                // face's patch has anyway.
                continue;
            }
            if !lp_instance.is("EDGE_LOOP") {
                return Err(unsupported(lp, lp_instance, "a face bound that is not a loop of edges"));
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
            out.push((t, to_p3(&curve.evaluate(S::from_f64(t))?)));
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
        let (start, end, base) = (edge.start, edge.end, edge.name.clone());
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
                name,
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
            let seams = self.seams(f);
            let wraps = match face.surface.revolved() {
                Some(revolved) => {
                    let revolved = revolved.clone();
                    !seams.is_empty() || self.windings(f, &revolved)?.iter().any(|&w| w != 0)
                }
                None if !seams.is_empty() => {
                    return Err(GeopError::new(format!(
                        "#{id}: a face running along a seam is only supported on a cylinder, cone, sphere, torus or surface of revolution, not on its {}",
                        self.reader.instance(id).map(|_| "surface").unwrap_or("surface")
                    )));
                }
                None => false,
            };
            if wraps {
                self.cut_wrapping_face(f, &seams)
                    .map_err(|e| e.with_context(format!("cutting the face #{id}, which wraps around its axis, into sectors")))?;
                // Its sectors were pushed at the end; it is gone.
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

    /// The edges face `f` runs along twice: its seams.
    fn seams(&self, f: usize) -> Vec<usize> {
        let mut count: HashMap<usize, usize> = HashMap::new();
        for lp in &self.faces[f].loops {
            for &(e, _) in lp {
                *count.entry(e).or_default() += 1;
            }
        }
        let mut seams: Vec<usize> = count.into_iter().filter(|&(_, n)| n == 2).map(|(e, _)| e).collect();
        seams.sort();
        seams
    }

    /// How often each loop of face `f` goes round the axis of `revolved`,
    /// counter-clockwise about it.
    fn windings(&self, f: usize, revolved: &Revolved) -> GeopResult<Vec<i64>> {
        self.faces[f]
            .loops
            .iter()
            .map(|lp| Ok(winding(&unwrap(revolved, &self.loop_points(lp)?, true))))
            .collect()
    }

    /// Cuts the face `f`, which wraps around its axis, into sectors along
    /// meridians, each on a patch that does not wrap; drops its seams.
    ///
    /// Without its seams, the face is a band around the axis between a
    /// bottom and a top, each a loop going once round — a *ring* — or a
    /// pole, with any holes in between. Where the cuts go is a free
    /// choice, made where nothing is: clear of every vertex and hole, as
    /// far from them as can be.
    fn cut_wrapping_face(&mut self, f: usize, seams: &[usize]) -> GeopResult<()> {
        let revolved = self.faces[f]
            .surface
            .revolved()
            .expect("only a surface of revolution wraps")
            .clone();
        // Seams go: each runs within the face, along where its patch would
        // have had to wrap.
        for &seam in seams {
            let users = self
                .faces
                .iter()
                .enumerate()
                .filter(|(g, face)| *g != f && face.loops.iter().flatten().any(|&(e, _)| e == seam))
                .count();
            if users > 0 {
                return Err(GeopError::new(
                    "an edge this face runs along twice is used by another face too",
                ));
            }
            self.edges[seam].alive = false;
        }
        let mut loops = Vec::new();
        for lp in std::mem::take(&mut self.faces[f].loops) {
            let cuts: Vec<usize> = (0..lp.len()).filter(|&k| seams.contains(&lp[k].0)).collect();
            if cuts.is_empty() {
                loops.push(lp);
                continue;
            }
            for (i, &k) in cuts.iter().enumerate() {
                let next = cuts[(i + 1) % cuts.len()];
                let piece: Vec<Use> = (1..)
                    .map(|j| (k + j) % lp.len())
                    .take_while(|&j| j != next)
                    .map(|j| lp[j])
                    .collect();
                if piece.is_empty() {
                    continue;
                }
                let (start, _) = self.ends(piece[0]);
                let (_, end) = self.ends(*piece.last().expect("not empty"));
                if start != end {
                    return Err(GeopError::new(
                        "taking its seams out leaves a loop that does not close",
                    ));
                }
                loops.push(piece);
            }
        }
        self.faces[f].loops = loops;

        let outward_natural = self.faces[f].outward_is_natural();
        let mut rings: Vec<usize> = Vec::new();
        let mut bottom = None;
        let mut top = None;
        let mut holes = Vec::new();
        for (k, lp) in self.faces[f].loops.iter().enumerate() {
            let points = self.loop_points(lp)?;
            let angles = unwrap(&revolved, &points, true);
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
        let v_of = |p: P3| revolved.chart(p).1;
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
                    .reduce(f64::min)
                    .ok_or_else(|| GeopError::new("it has nothing closing it off below: no loop round its axis and no pole"))?,
            ),
        };
        let top_pole = match top {
            Some(_) => None,
            None => Some(
                poles
                    .iter()
                    .copied()
                    .filter(|&p| bottom_v.is_none_or(|b| p > b))
                    .reduce(f64::max)
                    .ok_or_else(|| GeopError::new("it has nothing closing it off above: no loop round its axis and no pole"))?,
            ),
        };
        if let (Some(b), Some(t)) = (bottom_v.or(bottom_pole), top_v.or(top_pole))
            && b >= t
        {
            return Err(GeopError::new("its bottom is not below its top"));
        }

        // Where not to cut: at any vertex, or through a hole.
        let mut blocked: Vec<(f64, f64)> = Vec::new();
        for (k, lp) in self.faces[f].loops.iter().enumerate() {
            let points = self.loop_points(lp)?;
            if holes.contains(&k) {
                let angles = unwrap(&revolved, &points, false);
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

        // Split each ring where each cut crosses it.
        let mut ring_vertices: HashMap<usize, Vec<usize>> = HashMap::new();
        let mut splits: HashMap<usize, Vec<(f64, usize, usize)>> = HashMap::new(); // edge -> (t, ring loop, cut)
        for &k in &rings {
            let lp = self.faces[f].loops[k].clone();
            for (c, &cut) in cuts.iter().enumerate() {
                let (e, t) = self.crossing(&lp, cut, &revolved)?;
                splits.entry(e).or_default().push((t, k, c));
            }
        }
        for (&e, list) in &splits {
            let mut list = list.clone();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            let ts: Vec<S> = list.iter().map(|(t, _, _)| S::from_f64(*t)).collect();
            let (lo, hi) = self.edges[e].curve.domain();
            if ts.iter().any(|t| !(t.definitely_greater(lo) && t.definitely_less(hi))) {
                return Err(GeopError::new("a cut crosses a loop round its axis at a vertex"));
            }
            let created = self.split_edge(e, &ts)?;
            for ((_, ring, cut), v) in list.into_iter().zip(created) {
                let slots = ring_vertices.entry(ring).or_insert_with(|| vec![usize::MAX; cuts.len()]);
                slots[cut] = v;
            }
        }

        // A meridian along each cut, from bottom to top.
        let point_at = |builder: &Self, side: Option<usize>, pole: Option<f64>, c: usize| -> (P3, f64, Option<usize>) {
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
                        name: vec![builder.faces[f].name[0].clone(), which.to_string()],
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
            let curve = self.meridian(&revolved, cut, (pb, vb, bottom_pole.is_some()), (pt, vt, top_pole.is_some()))?;
            let mut name = self.faces[f].name.clone();
            name.push(format!("m{c}"));
            meridians.push(self.edges.len());
            self.edges.push(Edge {
                curve,
                start,
                end,
                name,
                alive: true,
            });
        }

        // The sectors between consecutive cuts.
        let n = cuts.len();
        let face_loops = self.faces[f].loops.clone();
        let ring_run = |builder: &Self, ring: usize, from: usize, to: usize, up: bool| -> GeopResult<Vec<Use>> {
            // The ring's coedges from the vertex at cut `from` to the one at
            // cut `to`, running counter-clockwise about the natural normal
            // (`up`) or clockwise.
            let lp = &face_loops[ring];
            let winding_ccw = {
                let angles = unwrap(&revolved, &builder.loop_points(lp)?, true);
                winding(&angles) > 0
            };
            let along = winding_ccw == up;
            let seq: Vec<Use> = if along {
                lp.clone()
            } else {
                lp.iter().rev().map(|&(e, fw)| (e, !fw)).collect()
            };
            let (a, b) = (ring_vertices[&ring][from], ring_vertices[&ring][to]);
            let start = seq
                .iter()
                .position(|&u| builder.ends(u).0 == a)
                .ok_or_else(|| GeopError::new("a cut vertex is not on its ring"))?;
            let mut run = Vec::new();
            for j in 0..seq.len() {
                let u = seq[(start + j) % seq.len()];
                run.push(u);
                if builder.ends(u).1 == b {
                    return Ok(run);
                }
            }
            Err(GeopError::new("a ring does not reach the next cut"))
        };
        let face = &self.faces[f];
        let (surface, same_sense, shell, base_name) =
            (face.surface.clone(), face.same_sense, face.shell, face.name.clone());
        // Each hole goes to the sector it lies in.
        let mut hole_sector = Vec::new();
        for &h in &holes {
            let points = self.loop_points(&face_loops[h])?;
            let a = unwrap(&revolved, &points, false);
            let mid = a.iter().sum::<f64>() / a.len() as f64;
            let sector = (0..n)
                .find(|&s| {
                    let lo = cuts[s];
                    let hi = if s + 1 < n { cuts[s + 1] } else { cuts[0] + std::f64::consts::TAU };
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
            let hi = if s + 1 < n { cuts[s + 1] } else { cuts[0] + std::f64::consts::TAU };
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

    /// Where along the ring `lp` it crosses the angle `cut`: the edge,
    /// and its parameter there. A free choice of where exactly, refined
    /// as far as `f64` bisection goes: the cut's meridian is built through
    /// what the edge has there.
    fn crossing(&self, lp: &[Use], cut: f64, revolved: &Revolved) -> GeopResult<(usize, f64)> {
        use std::f64::consts::{PI, TAU};
        let near = |a: f64, reference: f64| reference + (a - reference + PI).rem_euclid(TAU) - PI;
        let mut prev: Option<f64> = None;
        for &u in lp {
            let samples = self.samples(u, LOOP_SAMPLES)?;
            let mut last: Option<(f64, f64)> = None; // (parameter, unwrapped angle)
            for &(t, p) in &samples {
                let Some(a) = revolved.chart(p).0 else { continue };
                let a = match prev {
                    Some(r) => near(a, r),
                    None => a,
                };
                if let Some((t_prev, a_prev)) = last {
                    let (lo, hi) = (a_prev.min(a), a_prev.max(a));
                    let x = cut + TAU * ((lo - cut) / TAU).ceil();
                    if x <= hi && lo < hi {
                        let theta = |t: f64| -> GeopResult<f64> {
                            let p = to_p3(&self.edges[u.0].curve.evaluate(S::from_f64(t))?);
                            let angle = revolved
                                .chart(p)
                                .0
                                .ok_or_else(|| GeopError::new("a loop round its axis runs through it"))?;
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
            }
        }
        Err(GeopError::new(format!(
            "a loop round its axis never reaches the angle {cut}"
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
                let (a, b, reversed) = if tb.definitely_less(tt) { (tb, tt, false) } else { (tt, tb, true) };
                let piece = curve.sub_curve(
                    if a.could_be_equal(lo) { lo } else { a },
                    if b.could_be_equal(hi) { hi } else { b },
                )?;
                if reversed { piece.reverse() } else { piece }
            }
            _ => revolved.profile_curve::<S>(vb, vt)?,
        };
        Ok(profile.place(&turn))
    }

    /// The description of the body, and the names of what it is made of.
    #[allow(clippy::type_complexity)]
    fn spec(
        self,
        solid: bool,
    ) -> GeopResult<(BodySpec<S>, Vec<Vec<String>>, Vec<Vec<String>>, Vec<Vec<String>>)> {
        let Builder {
            vertices,
            edges,
            faces,
            shells,
            scope,
            ..
        } = self;
        let mut vertices = vertices;
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
        let mut face_names = Vec::with_capacity(faces.len());
        let mut face_shells = vec![Vec::new(); shells];
        for face in &faces {
            let ctx = |e: GeopError| {
                e.with_context(format!(
                    "building the face #{} ({})",
                    face.id,
                    face.name.join(",")
                ))
            };
            let patch = patch_of(face, &edges, &vertices, &scope).map_err(ctx)?;
            let natural = face.outward_is_natural();
            let surface = if natural { patch.surface.clone() } else { patch.surface.reverse_u() };
            let grid = Grid::new(&surface).map_err(ctx)?;
            let mut loops = Vec::new();
            for lp in &face.loops {
                loops.push(
                    fit_loop(lp, &surface, &grid, &patch, &edges, &mut vertices, &scope).map_err(ctx)?,
                );
            }
            // The outer loop runs counter-clockwise in the patch.
            let outer = match face.outer {
                Some(k) => k,
                None => {
                    let areas: Vec<f64> = loops.iter().map(|lp| signed_area(lp)).collect::<GeopResult<_>>().map_err(ctx)?;
                    let ccw: Vec<usize> = (0..areas.len()).filter(|&k| areas[k] > 0.0).collect();
                    match ccw.as_slice() {
                        [k] => *k,
                        _ => {
                            return Err(ctx(GeopError::new(format!(
                                "its loops do not bound one region: {} of {} run counter-clockwise about its normal (signed areas {areas:?})",
                                ccw.len(),
                                areas.len()
                            ))));
                        }
                    }
                }
            };
            let outer_loop = loops.remove(outer);
            face_shells[face.shell].push(face_specs.len());
            face_specs.push((surface, outer_loop, loops));
            face_names.push(face.name.clone());
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
                            if forward { Sense::Forward } else { Sense::Reversed },
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
        Ok((spec, vertex_names, edge_names, face_names))
    }
}

#[derive(Clone, Copy, Debug)]
enum CoedgeOnLocal {
    Edge(usize, bool),
    Vertex(usize),
}

/// A face's patch, and where its poles are: the profile parameter bound
/// of the patch (`v` low or high) each collapses at, and the point.
struct Patch<S: Scalar> {
    surface: NurbSurface3D<S>,
    /// `(at the high end of v, the pole)`.
    poles: Vec<(bool, P3)>,
}

/// The unwrapped angles about `revolved`'s axis of `points`, a loop's in
/// order: each within half a turn of the one before. Points on the axis,
/// which have no angle, are left out. With `closing`, the first point is
/// repeated at the end, so the last angle less the first is how far the
/// loop turns.
fn unwrap(revolved: &Revolved, points: &[P3], closing: bool) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::with_capacity(points.len() + 1);
    let iter = points.iter().chain(closing.then(|| &points[0]));
    for &p in iter {
        let Some(a) = revolved.chart(p).0 else { continue };
        let a = match out.last() {
            Some(&prev) => prev + (a - prev + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI,
            None => a,
        };
        out.push(a);
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
            let c = cuts.iter().map(|&a| clearance(a)).fold(f64::INFINITY, f64::min);
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
    let lowest = poles.iter().copied().filter(|&p| p <= v0).fold(f64::NEG_INFINITY, f64::max);
    let highest = poles.iter().copied().filter(|&p| p >= v1).fold(f64::INFINITY, f64::min);
    let a = if pole0 { v0 } else { (v0 - margin).max(lowest) };
    let b = if pole1 { v1 } else { (v1 + margin).min(highest) };
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
                this.push(to_p3(&curve.evaluate(S::from_f64(lo + (hi - lo) * f))?));
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
            let (a0, a1, b0, b1) = (lo[0] - margin, hi[0] + margin, lo[1] - margin, hi[1] + margin);
            let h = |p: Vector3<S>| geop_core_math::vector::Vector4::from_array([p[0], p[1], p[2], S::ONE]);
            let surface = NurbSurface3D::try_new(
                1,
                1,
                vec![h(corner(a0, b0)), h(corner(a0, b1)), h(corner(a1, b0)), h(corner(a1, b1))],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )?;
            Ok(Patch { surface, poles: Vec::new() })
        }
        SurfaceKind::Nurbs(nurbs) => Ok(Patch {
            surface: nurbs.to_nurbs()?,
            poles: Vec::new(),
        }),
        SurfaceKind::Extrusion { curve, vector } => {
            let length2 = dot(*vector, *vector);
            let dir = normalize(*vector).ok_or_else(|| GeopError::new("an extrusion along nothing"))?;
            let along = |p: P3| dot(p, dir);
            let (p_lo, p_hi) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &p| {
                (a.min(along(p)), b.max(along(p)))
            });
            let (c_lo, c_hi) = curve.points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &p| {
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
                poles: Vec::new(),
            })
        }
        SurfaceKind::Revolved(revolved) => {
            let [from, to, v0, v1] = match face.patch {
                Some(patch) => patch,
                None => revolved_extent(revolved, face, &loop_points, scope)?,
            };
            let surface = revolved.patch(from, to, v0, v1)?;
            let mut poles = Vec::new();
            match &revolved.profile {
                Profile::Curve(curve) => {
                    let curve = curve.to_nurbs::<S>()?;
                    let (lo, hi) = curve.domain();
                    for (high, t) in [(false, lo), (true, hi)] {
                        let p = to_p3(&curve.evaluate(t)?);
                        let l = revolved.frame.local(p);
                        if l[0].hypot(l[1]) <= scope.uncertainty {
                            poles.push((high, p));
                        }
                    }
                }
                _ => {
                    for pole in revolved.poles() {
                        if pole == v0 {
                            poles.push((false, revolved.profile_point(pole)));
                        }
                        if pole == v1 {
                            poles.push((true, revolved.profile_point(pole)));
                        }
                    }
                }
            }
            Ok(Patch { surface, poles })
        }
    }
}

/// The angles and profile parameters a face on a surface of revolution
/// covers, which does not wrap around its axis, a little widened where
/// they are free to be.
fn revolved_extent(revolved: &Revolved, face: &Face, loops: &[Vec<P3>], scope: &Scope) -> GeopResult<[f64; 4]> {
    // The outer loop spans the face's angles; each hole lies within them.
    let mut best: Option<(f64, f64, f64)> = None;
    for points in loops {
        let a = unwrap(revolved, points, false);
        if a.is_empty() {
            continue;
        }
        let lo = a.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = a.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if best.is_none_or(|(b_lo, b_hi, _)| hi - lo > b_hi - b_lo) {
            best = Some((lo, hi, 0.0));
        }
    }
    let (lo, hi, _) = best.ok_or_else(|| GeopError::new("a face whose every point lies on its axis"))?;
    let span = hi - lo;
    if span >= std::f64::consts::TAU {
        return Err(GeopError::new(format!(
            "#{}: a face going all the way round its axis without a loop that does",
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
        let mut gap = (torus_angles[0] + std::f64::consts::TAU - torus_angles[n - 1], 0usize);
        for i in 1..n {
            let g = torus_angles[i] - torus_angles[i - 1];
            if g > gap.0 {
                gap = (g, i);
            }
        }
        v_lo = torus_angles[gap.1];
        v_hi = if gap.1 == 0 { torus_angles[n - 1] } else { torus_angles[gap.1 - 1] + std::f64::consts::TAU };
    }
    let near_pole = |v: f64| {
        revolved.poles().into_iter().find(|&pole| {
            distance(revolved.profile_point(pole), revolved.profile_point(v)) <= scope.uncertainty.max(1e-9)
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

impl<S: Scalar> Grid<S> {
    fn new(surface: &NurbSurface3D<S>) -> GeopResult<Self> {
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let mut points = Vec::new();
        for i in 0..=GRID {
            for j in 0..=GRID {
                let fu = S::from_ratio(i as i64, GRID as i64)?;
                let fv = S::from_ratio(j as i64, GRID as i64)?;
                let u = u0.add(u1.sub(u0).mul(fu)).sharpen();
                let v = v0.add(v1.sub(v0).mul(fv)).sharpen();
                points.push((u, v, to_p3(&surface.evaluate(u, v)?)));
            }
        }
        Ok(Self { points })
    }

    /// The foot point of `p` on `surface`: Newton from the nearest grid
    /// point.
    fn project(&self, surface: &NurbSurface3D<S>, p: &Vector3<S>) -> GeopResult<(S, S)> {
        let target = to_p3(p);
        let (u, v, _) = self
            .points
            .iter()
            .min_by(|a, b| distance(a.2, target).total_cmp(&distance(b.2, target)))
            .expect("a grid has points");
        surface.project(*p, *u, *v, 30)
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
        if forward { (edge.start, edge.end) } else { (edge.end, edge.start) }
    };
    let (v_lo, v_hi) = surface.domain_v();
    // The pole a vertex sits at, if any.
    let pole_of = |v: usize, vertices: &[Vertex<S>]| -> Option<S> {
        let p = to_p3(&vertices[v].point);
        patch
            .poles
            .iter()
            .find(|(_, pole)| distance(*pole, p) <= scope.uncertainty.max(1e-9))
            .map(|&(high, _)| if high { v_hi } else { v_lo })
    };
    // Where each corner — the end of coedge `k`, the start of `k + 1` —
    // is on the patch: its vertex's foot point, unless at a pole.
    let mut corners: Vec<Option<Vector2<S>>> = Vec::with_capacity(n);
    for &u in lp {
        let (_, end) = ends(u);
        corners.push(match pole_of(end, vertices) {
            Some(_) => None,
            None => {
                let (pu, pv) = grid.project(surface, &vertices[end].point)?;
                Some(Vector2::from_array([pu, pv]))
            }
        });
    }
    let mut out = Vec::new();
    for k in 0..n {
        let u = lp[k];
        let (start, end) = ends(u);
        let edge = &edges[u.0];
        let curve = if u.1 { edge.curve.clone() } else { edge.curve.reverse() };
        let pin = |corner: Option<Vector2<S>>, vertex: usize, at_start: bool| -> GeopResult<Vector2<S>> {
            match corner {
                Some(uv) => Ok(uv),
                None => {
                    // At a pole any `u` is the same point: take the one the
                    // curve arrives along, from a point of it near the pole.
                    let (lo, hi) = curve.domain();
                    let f = S::from_ratio(1, 64)?;
                    let t = if at_start { lo.add(hi.sub(lo).mul(f)) } else { hi.sub(hi.sub(lo).mul(f)) };
                    let near = curve.evaluate(t.sharpen())?;
                    let (pu, _) = grid.project(surface, &near)?;
                    let pole_v = pole_of(vertex, vertices).expect("a corner without a foot point is at a pole");
                    Ok(Vector2::from_array([pu, pole_v]))
                }
            }
        };
        let pin_start = pin(corners[(k + n - 1) % n], start, true)?;
        let pin_end = pin(corners[k], end, false)?;
        let pcurve = surface
            .fit_pcurve(
                &curve,
                Some(pin_start),
                Some(pin_end),
                MAX_NODES,
                S::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .with_context(&|e: GeopError| {
                e.with_context(format!("fitting the pcurve of the edge {}", edge.name.join(",")))
            })?;
        out.push((CoedgeOnLocal::Edge(u.0, u.1), pcurve));
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
        let pcurve = &out[k].1;
        let (t0, t1) = pcurve.domain();
        let a = pcurve.evaluate(t0)?;
        let b = pcurve.evaluate(t1)?;
        let on_start = surface.evaluate(a[0], a[1])?;
        let on_end = surface.evaluate(b[0], b[1])?;
        for (v, extra) in [(start, [curve_start, on_start]), (end, [curve_end, on_end])] {
            for p in extra {
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
                let h = |p: Vector2<S>| geop_core_math::vector::Vector3::from_array([p[0], p[1], S::ONE]);
                let pcurve = NurbCurve::try_new(1, vec![h(from), h(to)], vec![S::ZERO, S::ZERO, S::ONE, S::ONE])?;
                joined.push((CoedgeOnLocal::Vertex(end), pcurve));
            }
        }
    }
    Ok(joined)
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

