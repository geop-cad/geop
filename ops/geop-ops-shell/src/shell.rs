//! [`shell`]: a solid hollowed out to walls of one thickness, open where
//! faces are taken away.
//!
//! The hollow is built directly, not cut out by a boolean: the walls'
//! inner side is the solid's own boundary moved inward, face for face, so
//! it has exactly the solid's topology, and everything about how the inner
//! and the outer side fit together is known without intersecting
//! anything.
//!
//! **The inner side.** Every face kept is offset inward by the thickness
//! (see [`NurbSurface3D::offset`]), every face taken away stays where it is:
//! its inner copy is what the opening leaves of it. A vertex moves to where
//! the offsets of the faces around it meet, an edge to where the offsets of
//! its two faces meet — a line from one moved vertex to the other for a
//! straight edge, an arc around the same axis for a circular one.
//!
//! **The opening.** Where a face `F` is taken away, its inner copy `F'`
//! lies on `F`'s own surface, inside it, and what is left of `F` is `F`
//! without `F'`: a rim, the walls' cut edge. Its boundary is `F`'s
//! boundary and `F'`'s, run the other way. Two faces taken away side by
//! side share an edge, and their inner copies share the middle of it — the
//! part between the walls; what is left of that edge is a short piece at
//! each end, where a wall reaches it, and the rims run along those. A rim
//! is put together from these pieces by following them end to end, and may
//! come out as several faces — a face taken away together with the faces
//! on two opposite sides of it leaves two strips.
//!
//! **The result.** The faces kept, their inner copies turned around, and
//! the rims, as one solid. Without an opening, the inner side is a void of
//! its own: a second shell.
//!
//! **A sheet** — faces standing on their own — is shelled the same way
//! ([`thicken`]): its faces are the outer side, their inner copies the
//! inner one, and along every free edge, used by one face only, a wall
//! joins the edge to its inner copy — the ruled surface between them, with
//! a straight edge from each of its vertices to that vertex's inner copy.
//! The inner side alone is the sheet offset ([`offset_faces`]).
//!
//! What it does not do (yet): offset faces that are neither planes nor
//! surfaces of revolution with a straight or circular meridian, edges that
//! are neither straight nor circular, take away part of a smooth surface —
//! a face tangent to one that stays — take away a face that meets one kept
//! at an inward corner — a pocket's floor, a step, the face a boss stands
//! on: the wall kept there is thickened away from the face, so the inner
//! copy reaches past it and the opening is no longer the face less its
//! inner copy — or open a solid that already has a void. Walls so thick that a straight edge's inner copy turns around are
//! refused; that the inner side runs into itself otherwise is not noticed.

use std::collections::HashMap;

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::NurbCurve3D,
    nurb_surface::NurbSurface3D,
    shape::{Arc, Circle},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    polygon::{loops_contain, polygon_signed_area},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, Curve2, Curve3, FaceId, Sense, ShellId, SolidId,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec, SpecSources},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_extrude_revolve::common::{line2, line3};

/// Bounds how hard a containment search or a pcurve fit tries: effort, not
/// what an answer means.
const MAX_NODES: usize = 20_000;

/// Where a containment search hands over to Newton (see `AGENTS.md`).
fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

/// Newton steps projecting a point onto a surface.
const PROJECT_ITERATIONS: usize = 20;

/// Newton steps moving a vertex onto the offsets of its faces, at most: a
/// vertex between planes lands in one, and curved faces converge
/// quadratically from a start a thickness away.
const VERTEX_ITERATIONS: usize = 8;

/// Samples per coedge when telling a loop's sense from its area: only the
/// sign is used.
const AREA_SAMPLES: usize = 8;

/// Hollows the solid `solid` of `part` out to walls `thickness` thick,
/// open where its faces `open` were — see the module docs — and returns
/// the hollowed solid, which replaces it and is named `namer`'s root.
///
/// What stays of the solid keeps its names. What is new is named after
/// what it was made from, with `S` the operation:
///
/// - `shell(S,X)`: the inner copy of the face, edge or vertex `X` kept; for
///   a face `X` taken away, the rim it leaves — `shell(S,X,0)`,
///   `shell(S,X,1)`, ... if that is several faces;
/// - `shell(S,E,start)` / `shell(S,E,end)`: what is left of an edge `E`
///   between two faces taken away, at its start and its end.
pub fn shell<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: SolidId,
    open: &[FaceId],
    thickness: S,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "shell({}, {solid}, open={open:?}, {thickness:?})",
        namer.root()
    );
    if !thickness.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "a shell's walls need a thickness greater than zero, not {thickness:?}"
        )))
        .with_context(ctx);
    }
    let model = part.topology();
    if model.get_solid(solid).with_context(ctx)?.shells.len() != 1 {
        return Err(GeopError::new(
            "the solid already has a void; shelling it is not supported",
        ))
        .with_context(ctx);
    }
    let faces = model.solid_faces(solid).with_context(ctx)?;
    if let Some(f) = open.iter().find(|f| !faces.contains(f)) {
        return Err(GeopError::new(format!(
            "face {f} is not a face of the solid being shelled"
        )))
        .with_context(ctx);
    }
    let (spec, sources) = model.body_spec(&faces, true).with_context(ctx)?;
    let names = source_names(part, &sources).with_context(ctx)?;
    let removed: Vec<bool> = sources.faces.iter().map(|f| open.contains(f)).collect();
    if let Some((f, g, e)) = inward_corner(&spec, &removed).with_context(ctx)? {
        return Err(GeopError::new(format!(
            "face {} is taken away where it meets face {} at an inward corner, along edge {}: its opening would reach past it, which is not supported",
            names.faces[f], names.faces[g], names.edges[e]
        )))
        .with_context(ctx);
    }

    let hollow = Hollow::new(&spec, &names, removed, thickness).with_context(ctx)?;
    let (result, result_names) = hollow.result(namer).with_context(ctx)?;
    part.assemble_solid(&[solid.into()], &[], "")
        .with_context(ctx)?;
    let built = part.build_body(result, result_names).with_context(ctx)?;
    built
        .solid
        .ok_or_else(|| GeopError::new("shell: building the result made no solid"))
}

/// Thickens the sheet `sheet` of `part` into a solid of walls `thickness`
/// thick — against its faces' normals, or along them if `along_normal` —
/// and returns it: the sheet shelled (see the module docs), which replaces
/// it and is named `namer`'s root.
///
/// The sheet's faces, edges and vertices keep their names, their inner
/// copies are named as [`shell`] names them, and along a free edge `E` of
/// the sheet the wall is `N(E,side)`, with `N` the operation, the straight
/// edge from a vertex `V` of it to its inner copy `N(V,side)`.
pub fn thicken<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    sheet: ShellId,
    thickness: S,
    along_normal: bool,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "thicken({}, sheet {sheet}, {thickness:?}, along_normal={along_normal})",
        namer.root()
    );
    if !thickness.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "a thickness greater than zero is needed, not {thickness:?}"
        )))
        .with_context(ctx);
    }
    let model = part.topology();
    let faces = model.body_faces(Body::Sheet(sheet)).with_context(ctx)?;
    let (mut spec, sources) = model.body_spec(&faces, false).with_context(ctx)?;
    if along_normal {
        // Turned around, the walls go inward from the other side.
        for face in &mut spec.faces {
            *face = face.reversed();
        }
    }
    let names = source_names(part, &sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, thickness).with_context(ctx)?;
    let (result, result_names) = hollow.result(namer).with_context(ctx)?;
    part.assemble_sheet(&[Body::Sheet(sheet)], &[])
        .with_context(ctx)?;
    let built = part.build_body(result, result_names).with_context(ctx)?;
    built
        .solid
        .ok_or_else(|| GeopError::new("thicken: building the result made no solid"))
}

/// Copies the faces `faces` of `part` — of a solid or of a sheet — into a
/// sheet `distance` along their normals (against them for a negative
/// distance), and returns it: the inner side of the faces shelled (see the
/// module docs), sharing nothing with them, which stay as they are.
///
/// The copy of each face, edge and vertex `X` is named `N(X)`, with `N`
/// the operation.
pub fn offset_faces<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    faces: &[FaceId],
    distance: S,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!("offset_faces({}, {faces:?}, {distance:?})", namer.root());
    if distance.could_be_equal(S::ZERO) {
        return Err(GeopError::new(format!(
            "an offset needs a distance other than zero, not {distance:?}"
        )))
        .with_context(ctx);
    }
    let (spec, sources) = part.topology().body_spec(faces, false).with_context(ctx)?;
    let names = source_names(part, &sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, distance.neg()).with_context(ctx)?;
    let (result, result_names) = hollow.inner_side(namer).with_context(ctx)?;
    part.build_body(result, result_names).with_context(ctx)
}

/// The names of what `sources` says a [`BodySpec`]'s entities came from.
fn source_names<S: Scalar>(part: &Part<S>, sources: &SpecSources) -> GeopResult<BodyNames> {
    let name = |id: geop_ops::RefId| -> GeopResult<String> {
        part.name_of(id)
            .map(str::to_string)
            .ok_or_else(|| GeopError::new(format!("{id:?} has no name")))
    };
    Ok(BodyNames {
        vertices: sources
            .vertices
            .iter()
            .map(|&v| name(v.into()))
            .collect::<GeopResult<_>>()?,
        edges: sources
            .edges
            .iter()
            .map(|&e| name(e.into()))
            .collect::<GeopResult<_>>()?,
        faces: sources
            .faces
            .iter()
            .map(|&f| name(f.into()))
            .collect::<GeopResult<_>>()?,
        solid: None,
    })
}

/// A face taken away, `f`, that meets a face kept, `g`, at an inward
/// corner — along the edge `e` the material between them turns more than
/// a half turn — if there is one: `(f, g, e)`.
///
/// The opening of a face taken away is what its inner copy leaves of it,
/// which assumes the inner copy lies inside the face. At an inward corner
/// it does not: the wall kept there is thickened away from the face, and
/// its end reaches past the face's edge. Decided halfway along the edge,
/// from the faces' outward normals there — those of the surfaces of a
/// [`BodySpec`] of a solid. A corner that could be tangent is no inward
/// one: it is left to the check that walls cannot end on a smooth surface.
fn inward_corner<S: Scalar>(
    spec: &BodySpec<S>,
    removed: &[bool],
) -> GeopResult<Option<(usize, usize, usize)>> {
    // Per edge, the faces it bounds, each with its coedge's sense and pcurve.
    let mut sides: Vec<Vec<(usize, Sense, &Curve2<S>)>> = vec![Vec::new(); spec.edges.len()];
    for (f, face) in spec.faces.iter().enumerate() {
        for lp in loops_of(face) {
            for c in lp {
                if let CoedgeOn::Edge(e, sense) = c.on {
                    sides[e].push((f, sense, &c.pcurve));
                }
            }
        }
    }
    for (e, at) in sides.iter().enumerate() {
        let [(f, f_sense, f_pcurve), (g, _, g_pcurve)] = match at.as_slice() {
            &[a, b] if removed[a.0] && !removed[b.0] => [a, b],
            &[a, b] if removed[b.0] && !removed[a.0] => [b, a],
            _ => continue,
        };
        let curve = &spec.edges[e].curve;
        let (t0, t1) = curve.domain();
        let t = S::interpolate(t0, t1, S::from_f64(0.5));
        let point = curve.evaluate(t)?;
        let tangent = curve.tangent(t)?;
        let along = match f_sense {
            Sense::Forward => tangent,
            Sense::Reversed => tangent.neg(),
        };
        let normal = |face: usize, pcurve: &Curve2<S>| -> GeopResult<Vector3<S>> {
            let surface = &spec.faces[face].surface;
            let (p0, p1) = pcurve.domain();
            let seed = pcurve.evaluate(S::interpolate(p0, p1, S::from_f64(0.5)))?;
            let (u, v) = surface.project(
                point,
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            surface.normal(u, v)
        };
        let (n_f, n_g) = (normal(f, f_pcurve)?, normal(g, g_pcurve)?);
        // Running into `f`, square to the edge: in front of `g` at an
        // inward corner (see `geop_ops_fillet`'s bend).
        if n_f
            .prod_cross(&along)
            .prod_dot(&n_g)
            .definitely_greater(S::ZERO)
        {
            return Ok(Some((f, g, e)));
        }
    }
    Ok(None)
}

/// The loops of `face`, outer first.
fn loops_of<S: Scalar>(face: &FaceSpec<S>) -> Vec<&Vec<CoedgeSpec<S>>> {
    std::iter::once(&face.outer).chain(&face.holes).collect()
}

/// A coedge of the solid: face `f`, its loop `l` (0 the outer one), and
/// the `k`-th coedge of that.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct At {
    f: usize,
    l: usize,
    k: usize,
}

/// What is left of an edge between two faces taken away: a piece at its
/// start and one at its end, each only where a wall reaches it.
type Pieces<T> = (Option<T>, Option<T>);

/// Where the entities of the solid went in the result, by their index
/// there: each walled vertex and its inner copy, each edge that is not
/// open and its inner copy, and what is left of each open one.
struct Numbering {
    vertex: Vec<Option<usize>>,
    inner_vertex: Vec<Option<usize>>,
    edge: Vec<Option<usize>>,
    inner_edge: Vec<Option<usize>>,
    pieces: Vec<Pieces<usize>>,
}

/// The pieces in the order a coedge of `sense` comes across them.
fn in_order<T>(pieces: (T, T), sense: Sense) -> (T, T) {
    match sense {
        Sense::Forward => pieces,
        Sense::Reversed => (pieces.1, pieces.0),
    }
}

/// The solid described as [`BodySpec`], and what hollowing it moves where.
struct Hollow<'a, S: Scalar> {
    spec: &'a BodySpec<S>,
    names: &'a BodyNames,
    /// Per face: whether it is taken away.
    removed: Vec<bool>,
    /// Per vertex: whether a face kept meets it. Where none does, the inner
    /// copy of the vertex is the vertex itself, and nothing of the solid
    /// is left around it.
    walled: Vec<bool>,
    /// Per edge: whether both its faces are taken away.
    open_edge: Vec<bool>,
    /// Per edge: whether it is free, used by one face only — an edge of a
    /// sheet's border, which gets a wall.
    free_edge: Vec<bool>,
    /// Per face: the surface its inner copy lies on — for a face taken
    /// away, its own — with room around the face (see [`extended`]).
    inner_surfaces: Vec<NurbSurface3D<S>>,
    /// Per vertex: its inner copy, where it is walled.
    inner_points: Vec<Option<Vector3<S>>>,
    /// Per edge: its inner copy's curve, unless it is open.
    inner_curves: Vec<Option<Curve3<S>>>,
    /// Per open edge: what is left of it at its start and at its end, where
    /// those are walled.
    pieces: Vec<Pieces<Curve3<S>>>,
    /// Per coedge, the `(u, v)` where it ends on its face's inner surface —
    /// where it ends at a walled vertex.
    inner_ends: HashMap<At, Vector2<S>>,
}

/// The vertices a coedge runs from and to.
fn ends<S: Scalar>(spec: &BodySpec<S>, on: CoedgeOn) -> (usize, usize) {
    match on {
        CoedgeOn::Edge(e, Sense::Forward) => (spec.edges[e].start, spec.edges[e].end),
        CoedgeOn::Edge(e, Sense::Reversed) => (spec.edges[e].end, spec.edges[e].start),
        CoedgeOn::Vertex(v) => (v, v),
    }
}

/// Where `pcurve` starts and ends.
fn pcurve_ends<S: Scalar>(pcurve: &Curve2<S>) -> GeopResult<(Vector2<S>, Vector2<S>)> {
    let (t0, t1) = pcurve.domain();
    Ok((pcurve.evaluate(t0)?, pcurve.evaluate(t1)?))
}

impl<'a, S: Scalar> Hollow<'a, S> {
    fn new(
        spec: &'a BodySpec<S>,
        names: &'a BodyNames,
        removed: Vec<bool>,
        thickness: S,
    ) -> GeopResult<Self> {
        let mut walled = vec![false; spec.vertices.len()];
        let mut open_edge = vec![true; spec.edges.len()];
        let mut uses = vec![0usize; spec.edges.len()];
        // Per vertex, each face around it once, with where the vertex is on
        // the face's surface.
        let mut around: Vec<Vec<(usize, Vector2<S>)>> = vec![Vec::new(); spec.vertices.len()];
        for (f, face) in spec.faces.iter().enumerate() {
            for lp in loops_of(face) {
                for c in lp {
                    let (_, end) = ends(spec, c.on);
                    walled[end] |= !removed[f];
                    if let CoedgeOn::Edge(e, _) = c.on {
                        open_edge[e] &= removed[f];
                        uses[e] += 1;
                    }
                    if !around[end].iter().any(|&(g, _)| g == f) {
                        around[end].push((f, pcurve_ends(&c.pcurve)?.1));
                    }
                }
            }
        }

        let inner_surfaces =
            spec.faces
                .iter()
                .enumerate()
                .map(|(f, face)| {
                    if removed[f] {
                        return extended(face.surface.clone());
                    }
                    let offset = face.surface.offset(thickness.neg()).map_err(|e| {
                        e.with_context(format!("offsetting face {}", names.faces[f]))
                    })?;
                    extended(offset)
                })
                .collect::<GeopResult<Vec<_>>>()?;

        let inner_points = (0..spec.vertices.len())
            .map(|v| {
                if !walled[v] {
                    return Ok(None);
                }
                let constraints: Vec<_> = around[v]
                    .iter()
                    .map(|&(f, uv)| (&inner_surfaces[f], uv, removed[f]))
                    .collect();
                offset_vertex(spec.vertices[v], &constraints)
                    .map(Some)
                    .map_err(|e| {
                        e.with_context(format!("moving vertex {} inward", names.vertices[v]))
                    })
            })
            .collect::<GeopResult<Vec<_>>>()?;

        // Walls thicker than a face is wide turn its inner copy inside out:
        // a straight edge's inner copy then runs the other way.
        for (e, edge) in spec.edges.iter().enumerate() {
            let (Some(start), Some(end)) = (&inner_points[edge.start], &inner_points[edge.end])
            else {
                continue;
            };
            let along = spec.vertices[edge.end].sub(&spec.vertices[edge.start]);
            if edge.curve.as_line()?.is_some()
                && !end.sub(start).prod_dot(&along).definitely_greater(S::ZERO)
            {
                return Err(GeopError::new(format!(
                    "the walls are too thick: the inner copy of edge {} turns around",
                    names.edges[e]
                )));
            }
        }

        let mut hollow = Self {
            spec,
            names,
            removed,
            walled,
            open_edge,
            free_edge: uses.iter().map(|&n| n == 1).collect(),
            inner_surfaces,
            inner_points,
            inner_curves: Vec::new(),
            pieces: Vec::new(),
            inner_ends: HashMap::new(),
        };
        hollow.inner_curves = (0..spec.edges.len())
            .map(|e| hollow.inner_curve(e))
            .collect::<GeopResult<_>>()?;
        hollow.pieces = (0..spec.edges.len())
            .map(|e| hollow.pieces_of(e))
            .collect::<GeopResult<_>>()?;
        hollow.inner_ends = hollow.inner_ends()?;
        Ok(hollow)
    }

    /// The coedges of the solid, each with where it is.
    fn coedges(&self) -> impl Iterator<Item = (At, &'a CoedgeSpec<S>)> + 'a {
        let spec = self.spec;
        spec.faces.iter().enumerate().flat_map(|(f, face)| {
            loops_of(face)
                .into_iter()
                .enumerate()
                .flat_map(move |(l, lp)| {
                    lp.iter().enumerate().map(move |(k, c)| (At { f, l, k }, c))
                })
        })
    }

    fn coedge(&self, at: At) -> &'a CoedgeSpec<S> {
        &loops_of(&self.spec.faces[at.f])[at.l][at.k]
    }

    /// The coedge before `at` in its loop.
    fn previous(&self, at: At) -> At {
        let n = loops_of(&self.spec.faces[at.f])[at.l].len();
        At {
            k: (at.k + n - 1) % n,
            ..at
        }
    }

    /// The faces the edge `e` runs between.
    fn faces_of_edge(&self, e: usize) -> Vec<usize> {
        self.coedges()
            .filter(|(_, c)| matches!(c.on, CoedgeOn::Edge(x, _) if x == e))
            .map(|(at, _)| at.f)
            .collect()
    }

    fn inner_point(&self, v: usize) -> Vector3<S> {
        self.inner_points[v].expect("a walled vertex has an inner copy")
    }

    /// The curve of the inner copy of edge `e`, unless it is open — then it
    /// lies on the edge itself, and is no part of the result.
    fn inner_curve(&self, e: usize) -> GeopResult<Option<Curve3<S>>> {
        if self.open_edge[e] {
            return Ok(None);
        }
        let edge = &self.spec.edges[e];
        let ctx = |err: GeopError| {
            err.with_context(format!("the inner copy of edge {}", self.names.edges[e]))
        };
        let (start, end) = (self.inner_point(edge.start), self.inner_point(edge.end));
        let curve = shaped_like(&edge.curve, start, end, edge.start == edge.end).map_err(ctx)?;
        // It is only right if it lies on both inner surfaces: say so if not,
        // rather than build an edge off its faces.
        let (t0, t1) = curve.domain();
        let middle = curve.evaluate(t0.add(t1).div(S::TWO)?)?;
        for f in self.faces_of_edge(e) {
            let on = surface_could_contain(
                &self.inner_surfaces[f],
                &middle,
                MAX_NODES,
                min_subdivision_size(),
            )
            .map_err(ctx)?;
            if on.is_none() {
                return Err(ctx(GeopError::new(format!(
                    "its curve from {start:?} to {end:?} is not on the offset of face {} at {middle:?}: offsetting this kind of edge is not supported",
                    self.names.faces[f]
                ))));
            }
        }
        Ok(Some(curve))
    }

    /// What is left of the open edge `e` at its start and at its end: from
    /// a walled end to its inner copy, which lies on the edge.
    fn pieces_of(&self, e: usize) -> GeopResult<Pieces<Curve3<S>>> {
        if !self.open_edge[e] {
            return Ok((None, None));
        }
        let edge = &self.spec.edges[e];
        let piece = |from: Vector3<S>, to: Vector3<S>| shaped_like(&edge.curve, from, to, false);
        let (s, t) = (edge.start, edge.end);
        Ok((
            self.walled[s]
                .then(|| piece(self.spec.vertices[s], self.inner_point(s)))
                .transpose()?,
            self.walled[t]
                .then(|| piece(self.inner_point(t), self.spec.vertices[t]))
                .transpose()?,
        ))
    }

    /// Where every coedge's inner copy ends on its face's inner surface, at
    /// a walled vertex: the inner vertex projected onto it, from where the
    /// coedge ends on the face — one `(u, v)` per junction of a loop, so
    /// the coedges meeting there agree on it exactly.
    fn inner_ends(&self) -> GeopResult<HashMap<At, Vector2<S>>> {
        let mut out = HashMap::new();
        for (at, c) in self.coedges() {
            let (_, end) = ends(self.spec, c.on);
            if !self.walled[end] {
                continue;
            }
            let seed = pcurve_ends(&c.pcurve)?.1;
            let surface = &self.inner_surfaces[at.f];
            let (u, v) = surface.project(
                self.inner_point(end),
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            // On the patch, as the vertex is.
            let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
            let (u, v) = (u.intersect(u0.union(u1)), v.intersect(v0.union(v1)));
            out.insert(at, Vector2::from_array([u, v]));
        }
        Ok(out)
    }

    /// Where the inner copy of the coedge `at` starts and ends on its face's
    /// inner surface.
    fn inner_pins(&self, at: At) -> (Vector2<S>, Vector2<S>) {
        (self.inner_ends[&self.previous(at)], self.inner_ends[&at])
    }

    /// The inner copy of the coedge `at`, on its face's inner surface.
    fn inner_coedge(&self, at: At) -> GeopResult<CoedgeSpec<S>> {
        let c = self.coedge(at);
        let (start, end) = self.inner_pins(at);
        let curve = match c.on {
            CoedgeOn::Edge(e, sense) => Some(oriented(
                self.inner_curves[e].as_ref().expect("not open"),
                sense,
            )),
            CoedgeOn::Vertex(_) => None,
        };
        Ok(CoedgeSpec {
            on: c.on,
            pcurve: pcurve(
                &self.inner_surfaces[at.f],
                &c.pcurve,
                curve.as_ref(),
                start,
                end,
            )?,
        })
    }

    /// The hollowed solid, described whole, and the names of what it is
    /// made of.
    fn result(&self, namer: &Namer) -> GeopResult<(BodySpec<S>, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            shells: Vec::new(),
            solid: true,
        };
        let mut names = BodyNames {
            solid: Some(namer.root()),
            ..Default::default()
        };
        let inner_name = |name: &str| namer.name(&[name]);

        let mut at = Numbering {
            vertex: vec![None; spec.vertices.len()],
            inner_vertex: vec![None; spec.vertices.len()],
            edge: vec![None; spec.edges.len()],
            inner_edge: vec![None; spec.edges.len()],
            pieces: vec![(None, None); spec.edges.len()],
        };
        for v in 0..spec.vertices.len() {
            if self.walled[v] {
                at.vertex[v] = Some(out.vertices.len());
                out.vertices.push(spec.vertices[v]);
                names.vertices.push(self.names.vertices[v].clone());
                at.inner_vertex[v] = Some(out.vertices.len());
                out.vertices.push(self.inner_point(v));
                names.vertices.push(inner_name(&self.names.vertices[v]));
            }
        }
        let (vertex, inner_vertex) = (&at.vertex, &at.inner_vertex);
        let mut push_edge = |curve: Curve3<S>, start: usize, end: usize, name: String| {
            out.edges.push(EdgeSpec { curve, start, end });
            names.edges.push(name);
            Some(out.edges.len() - 1)
        };
        for (e, original) in spec.edges.iter().enumerate() {
            let name = &self.names.edges[e];
            let (s, t) = (original.start, original.end);
            if let Some(curve) = &self.inner_curves[e] {
                at.edge[e] = push_edge(
                    original.curve.clone(),
                    vertex[s].unwrap(),
                    vertex[t].unwrap(),
                    name.clone(),
                );
                at.inner_edge[e] = push_edge(
                    curve.clone(),
                    inner_vertex[s].unwrap(),
                    inner_vertex[t].unwrap(),
                    inner_name(name),
                );
            }
            let (first, second) = &self.pieces[e];
            if let Some(curve) = first {
                at.pieces[e].0 = push_edge(
                    curve.clone(),
                    vertex[s].unwrap(),
                    inner_vertex[s].unwrap(),
                    namer.name(&[name, "start"]),
                );
            }
            if let Some(curve) = second {
                at.pieces[e].1 = push_edge(
                    curve.clone(),
                    inner_vertex[t].unwrap(),
                    vertex[t].unwrap(),
                    namer.name(&[name, "end"]),
                );
            }
        }
        let map = |on: CoedgeOn, edges: &[Option<usize>], vertices: &[Option<usize>]| match on {
            CoedgeOn::Edge(e, sense) => CoedgeOn::Edge(edges[e].unwrap(), sense),
            CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertices[v].unwrap()),
        };

        let mut outer_faces = Vec::new();
        let mut inner_faces = Vec::new();
        for (f, face) in spec.faces.iter().enumerate() {
            let name = &self.names.faces[f];
            if self.removed[f] {
                let rims = self.rims(f, &at)?;
                let several = rims.len() > 1;
                for (k, rim) in rims.into_iter().enumerate() {
                    outer_faces.push(out.faces.len());
                    out.faces.push(rim);
                    names.faces.push(if several {
                        namer.name(&[name, &k.to_string()])
                    } else {
                        namer.name(&[name])
                    });
                }
                continue;
            }
            let remap = |lp: &Vec<CoedgeSpec<S>>| -> Vec<CoedgeSpec<S>> {
                lp.iter()
                    .map(|c| CoedgeSpec {
                        on: map(c.on, &at.edge, &at.vertex),
                        pcurve: c.pcurve.clone(),
                    })
                    .collect()
            };
            outer_faces.push(out.faces.len());
            out.faces.push(FaceSpec {
                surface: face.surface.clone(),
                outer: remap(&face.outer),
                holes: face.holes.iter().map(remap).collect(),
            });
            names.faces.push(name.clone());

            let inner_loop = |l: usize| -> GeopResult<Vec<CoedgeSpec<S>>> {
                (0..loops_of(face)[l].len())
                    .map(|k| {
                        let mut c = self.inner_coedge(At { f, l, k })?;
                        c.on = map(c.on, &at.inner_edge, &at.inner_vertex);
                        Ok(c)
                    })
                    .collect()
            };
            let inner = FaceSpec {
                surface: self.inner_surfaces[f].clone(),
                outer: inner_loop(0)?,
                holes: (1..=face.holes.len())
                    .map(inner_loop)
                    .collect::<GeopResult<_>>()?,
            };
            inner_faces.push(out.faces.len());
            out.faces.push(inner.reversed());
            names.faces.push(inner_name(name));
        }
        // Along every free edge, a wall from the edge to its inner copy,
        // with a straight edge from each of its vertices to that vertex's
        // inner copy, `side[v]`.
        let mut side: Vec<Option<usize>> = vec![None; spec.vertices.len()];
        let mut walls = Vec::new();
        for (_, c) in self.coedges() {
            let CoedgeOn::Edge(e, sense) = c.on else {
                continue;
            };
            if !self.free_edge[e] {
                continue;
            }
            let (x, y) = ends(spec, c.on);
            for v in [x, y] {
                if side[v].is_none() {
                    side[v] = Some(out.edges.len());
                    out.edges.push(EdgeSpec {
                        curve: line3(spec.vertices[v], self.inner_point(v))?,
                        start: at.vertex[v].unwrap(),
                        end: at.inner_vertex[v].unwrap(),
                    });
                    names
                        .edges
                        .push(namer.name(&[&self.names.vertices[v], "side"]));
                }
            }
            let outer = oriented(&spec.edges[e].curve, sense);
            let inner = oriented(self.inner_curves[e].as_ref().expect("not open"), sense);
            let square = |k: usize| -> GeopResult<Curve2<S>> {
                let corner = |k: usize| {
                    let (u, v) = [(0, 0), (1, 0), (1, 1), (0, 1)][k % 4];
                    Vector2::from_array([S::from_i64(u), S::from_i64(v)])
                };
                line2(corner(k), corner(k + 1))
            };
            // From the inner copy, at `v = 0`, out to the edge, at `v = 1`:
            // running round it counter-clockwise, the wall runs the edge
            // the other way than its face does, and faces away from it.
            let lp = [
                CoedgeOn::Edge(at.inner_edge[e].unwrap(), sense),
                CoedgeOn::Edge(side[y].unwrap(), Sense::Reversed),
                CoedgeOn::Edge(at.edge[e].unwrap(), sense.opposite()),
                CoedgeOn::Edge(side[x].unwrap(), Sense::Forward),
            ];
            walls.push(out.faces.len());
            out.faces.push(FaceSpec {
                surface: NurbSurface3D::ruled(&inner, &outer).map_err(|err| {
                    err.with_context(format!("the wall along edge {}", self.names.edges[e]))
                })?,
                outer: lp
                    .into_iter()
                    .enumerate()
                    .map(|(k, on)| {
                        Ok(CoedgeSpec {
                            on,
                            pcurve: square(k)?,
                        })
                    })
                    .collect::<GeopResult<_>>()?,
                holes: Vec::new(),
            });
            names
                .faces
                .push(namer.name(&[&self.names.edges[e], "side"]));
        }
        out.shells = if self.removed.iter().any(|&r| r) || !walls.is_empty() {
            vec![
                outer_faces
                    .into_iter()
                    .chain(inner_faces)
                    .chain(walls)
                    .collect(),
            ]
        } else {
            vec![outer_faces, inner_faces]
        };
        Ok((out, names))
    }

    /// The inner side alone, as a sheet of its own: the inner copy of every
    /// face, facing the way the face does, and of every edge and vertex,
    /// each copy of `X` named `N(X)` for `namer`'s `N`.
    fn inner_side(&self, namer: &Namer) -> GeopResult<(BodySpec<S>, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            shells: Vec::new(),
            solid: false,
        };
        let mut names = BodyNames::default();
        let mut vertex = vec![None; spec.vertices.len()];
        for (v, name) in self.names.vertices.iter().enumerate() {
            vertex[v] = Some(out.vertices.len());
            out.vertices.push(self.inner_point(v));
            names.vertices.push(namer.name(&[name]));
        }
        let mut edge = vec![None; spec.edges.len()];
        for (e, original) in spec.edges.iter().enumerate() {
            edge[e] = Some(out.edges.len());
            out.edges.push(EdgeSpec {
                curve: self.inner_curves[e].clone().expect("not open"),
                start: vertex[original.start].unwrap(),
                end: vertex[original.end].unwrap(),
            });
            names.edges.push(namer.name(&[&self.names.edges[e]]));
        }
        for (f, face) in spec.faces.iter().enumerate() {
            let inner_loop = |l: usize| -> GeopResult<Vec<CoedgeSpec<S>>> {
                (0..loops_of(face)[l].len())
                    .map(|k| {
                        let mut c = self.inner_coedge(At { f, l, k })?;
                        c.on = match c.on {
                            CoedgeOn::Edge(e, sense) => CoedgeOn::Edge(edge[e].unwrap(), sense),
                            CoedgeOn::Vertex(v) => CoedgeOn::Vertex(vertex[v].unwrap()),
                        };
                        Ok(c)
                    })
                    .collect()
            };
            out.faces.push(FaceSpec {
                surface: self.inner_surfaces[f].clone(),
                outer: inner_loop(0)?,
                holes: (1..=face.holes.len())
                    .map(inner_loop)
                    .collect::<GeopResult<_>>()?,
            });
            names.faces.push(namer.name(&[&self.names.faces[f]]));
        }
        out.shells = vec![(0..out.faces.len()).collect()];
        Ok((out, names))
    }

    /// What is left of the face `f`, taken away: `f` without its inner copy
    /// (see the module docs), as one face or several, the result's entities
    /// numbered `at`.
    fn rims(&self, f: usize, at: &Numbering) -> GeopResult<Vec<FaceSpec<S>>> {
        let spec = self.spec;
        let (vertex, inner_vertex) = (&at.vertex, &at.inner_vertex);
        let surface = &spec.faces[f].surface;
        let unsupported = |v: usize| {
            GeopError::new(format!(
                "face {} is taken away at vertex {}, where its surface comes to a point: not supported",
                self.names.faces[f], self.names.vertices[v]
            ))
        };
        // Every coedge of the rim, with the result's vertices it runs from
        // and to.
        let mut coedges: Vec<(CoedgeSpec<S>, usize, usize)> = Vec::new();
        for (here, c) in self.coedges().filter(|(here, _)| here.f == f) {
            let CoedgeOn::Edge(e, sense) = c.on else {
                let CoedgeOn::Vertex(v) = c.on else {
                    unreachable!()
                };
                if self.walled[v] {
                    return Err(unsupported(v));
                }
                continue;
            };
            let (x, y) = ends(spec, c.on);
            if !self.open_edge[e] {
                // `f`'s own boundary where a wall stands on it...
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(at.edge[e].unwrap(), sense),
                        pcurve: c.pcurve.clone(),
                    },
                    vertex[x].unwrap(),
                    vertex[y].unwrap(),
                ));
                // ...and its inner copy, the other way.
                let inner = self.inner_coedge(here)?;
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(at.inner_edge[e].unwrap(), sense.opposite()),
                        pcurve: inner.pcurve.reverse(),
                    },
                    inner_vertex[y].unwrap(),
                    inner_vertex[x].unwrap(),
                ));
                continue;
            }
            // An edge to another face taken away: what is left of it, from
            // `x` to its inner copy and from `y`'s to `y`.
            let (orig_start, orig_end) = pcurve_ends(&c.pcurve)?;
            let (start_piece, end_piece) = in_order(at.pieces[e], sense);
            let curves = &self.pieces[e];
            let (start_curve, end_curve) = in_order((&curves.0, &curves.1), sense);
            if let (Some(piece), Some(curve)) = (start_piece, start_curve) {
                let to = self.inner_ends[&self.previous(here)];
                let curve = oriented(curve, sense);
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(piece, sense),
                        pcurve: pcurve(surface, &c.pcurve, Some(&curve), orig_start, to)?,
                    },
                    vertex[x].unwrap(),
                    inner_vertex[x].unwrap(),
                ));
            }
            if let (Some(piece), Some(curve)) = (end_piece, end_curve) {
                let from = self.inner_ends[&here];
                let curve = oriented(curve, sense);
                coedges.push((
                    CoedgeSpec {
                        on: CoedgeOn::Edge(piece, sense),
                        pcurve: pcurve(surface, &c.pcurve, Some(&curve), from, orig_end)?,
                    },
                    inner_vertex[y].unwrap(),
                    vertex[y].unwrap(),
                ));
            }
        }

        // Follow the coedges end to end into loops.
        let mut starting: HashMap<usize, usize> = HashMap::new();
        for (i, (_, start, _)) in coedges.iter().enumerate() {
            if starting.insert(*start, i).is_some() {
                return Err(GeopError::new(format!(
                    "what is left of face {} touches itself at a vertex: not supported",
                    self.names.faces[f]
                )));
            }
        }
        let mut used = vec![false; coedges.len()];
        let mut loops: Vec<Vec<usize>> = Vec::new();
        for first in 0..coedges.len() {
            if used[first] {
                continue;
            }
            let mut lp = Vec::new();
            let mut i = first;
            loop {
                used[i] = true;
                lp.push(i);
                i = *starting.get(&coedges[i].2).ok_or_else(|| {
                    GeopError::new(format!(
                        "what is left of face {} does not close up",
                        self.names.faces[f]
                    ))
                })?;
                if i == first {
                    break;
                }
                if used[i] {
                    return Err(GeopError::new(format!(
                        "what is left of face {} runs into itself",
                        self.names.faces[f]
                    )));
                }
            }
            loops.push(lp);
        }

        // Outer loops run counter-clockwise in `(u, v)`, holes clockwise;
        // each hole lies in one outer loop.
        let mut outers: Vec<(Vec<usize>, Vec<Vector2<S>>, S)> = Vec::new();
        let mut holes: Vec<(Vec<usize>, Vec<Vector2<S>>)> = Vec::new();
        for lp in loops {
            let polygon = lp
                .iter()
                .map(|&i| sample(&coedges[i].0.pcurve))
                .collect::<GeopResult<Vec<_>>>()?
                .concat();
            let area = polygon_signed_area(&polygon);
            if area.definitely_greater(S::ZERO) {
                outers.push((lp, polygon, area));
            } else if area.definitely_less(S::ZERO) {
                holes.push((lp, polygon));
            } else {
                return Err(GeopError::new(format!(
                    "cannot tell which way a loop of what is left of face {} runs",
                    self.names.faces[f]
                )));
            }
        }
        let mut faces: Vec<FaceSpec<S>> = outers
            .iter()
            .map(|(lp, _, _)| FaceSpec {
                surface: surface.clone(),
                outer: lp.iter().map(|&i| coedges[i].0.clone()).collect(),
                holes: Vec::new(),
            })
            .collect();
        // A part of the rim may lie inside another's outer loop — the ring
        // around a hole inside the ring around the walls — so a hole is the
        // innermost one's: of the outer loops around it, the smallest.
        for (lp, polygon) in &holes {
            let mut owner: Option<usize> = None;
            for (i, (_, outer, area)) in outers.iter().enumerate() {
                if loops_contain(std::slice::from_ref(outer), &polygon[0])
                    && owner.is_none_or(|o| area.definitely_less(outers[o].2))
                {
                    owner = Some(i);
                }
            }
            let owner = owner.ok_or_else(|| {
                GeopError::new(format!(
                    "a hole of what is left of face {} lies in none of its parts",
                    self.names.faces[f]
                ))
            })?;
            faces[owner]
                .holes
                .push(lp.iter().map(|&i| coedges[i].0.clone()).collect());
        }
        Ok(faces)
    }
}

/// Points along `pcurve`, its end left out: the next coedge starts there.
fn sample<S: Scalar>(pcurve: &Curve2<S>) -> GeopResult<Vec<Vector2<S>>> {
    let (t0, t1) = pcurve.domain();
    (0..AREA_SAMPLES)
        .map(|i| {
            let fraction = S::from_ratio(i as i64, AREA_SAMPLES as i64)?;
            pcurve.evaluate(S::interpolate(t0, t1, fraction))
        })
        .collect()
}

/// `curve`, run the way a coedge of `sense` runs it.
fn oriented<S: Scalar>(curve: &Curve3<S>, sense: Sense) -> Curve3<S> {
    match sense {
        Sense::Forward => curve.clone(),
        Sense::Reversed => curve.reverse(),
    }
}

/// A curve shaped like `curve` from `start` to `end` — all the way round
/// a circle back to `start`, if `closed`: a line for a straight one, an arc
/// around the same axis for a circular one. Fails for any other curve, and
/// for an `end` not on the circle around that axis through `start`.
fn shaped_like<S: Scalar>(
    curve: &Curve3<S>,
    start: Vector3<S>,
    end: Vector3<S>,
    closed: bool,
) -> GeopResult<Curve3<S>> {
    if curve.as_line()?.is_some() {
        return line3(start, end);
    }
    let Some(arc) = curve.as_arc()? else {
        return Err(GeopError::new(
            "the edge is neither straight nor circular: offsetting it is not supported",
        ));
    };
    let axis = arc.circle.axis();
    let center = axis.project(&start);
    let radius = start.sub(&center).norm();
    if !closed
        && !(axis.project(&end).could_be_equal(&center)
            && end.sub(&center).norm().could_be_equal(radius))
    {
        return Err(GeopError::new(format!(
            "{end:?} is not on the circle around {axis:?} through {start:?}"
        )));
    }
    // Where the copy spans the same angles as the arc — a cylinder's rim
    // moved along its axis, a disc's rim shrunk — it is the arc's own
    // control points moved along the axis and scaled about it, weights and
    // knots kept: the same parametrization, so the ruled surface between
    // the two, a sheet's wall, is the cylinder, cone or ring between them,
    // not a twisted one. Where it spans other angles — a sphere's meridian
    // cut by an offset plane — it is the arc between its ends.
    let scale = radius.div(arc.circle.radius)?;
    let control_points = curve
        .control_points
        .iter()
        .map(|cp| {
            let w = cp[3];
            let p = Vector3::from_array([cp[0].div(w)?, cp[1].div(w)?, cp[2].div(w)?]);
            let moved = center.add(&p.sub(&arc.circle.center).prod_scalar(scale));
            Ok(Vector::from_array([
                moved[0].mul(w),
                moved[1].mul(w),
                moved[2].mul(w),
                w,
            ]))
        })
        .collect::<GeopResult<Vec<_>>>()?;
    let moved = NurbCurve3D::try_new(curve.degree, control_points, curve.knot_vector.clone())?;
    let (t0, t1) = moved.domain();
    let (from, to) = (moved.evaluate(t0)?, moved.evaluate(t1)?);
    let end = if closed { start } else { end };
    if from.could_be_equal(&start) && to.could_be_equal(&end) {
        return Ok(moved);
    }
    Arc {
        circle: Circle {
            center,
            normal: arc.circle.normal,
            radius,
        },
        start,
        end,
    }
    .to_curve()
}

/// The pcurve on `surface` of a coedge that runs along `curve` — or sits at
/// a vertex, without one — from `(u, v)` `start` to `end`. Straight where
/// the coedge it is a copy of had a straight pcurve, `original`, and its
/// inner copy is straight too: on a plane, whose `(u, v)` are affine, or
/// along the same iso-line — the line keeps its `u` (or `v`) — as a
/// cylinder's rim or seam does. Fitted otherwise: a sphere's meridian cut
/// by a plane is straight, but its inner copy, where the offset plane cuts
/// the offset sphere, is a small circle no straight pcurve follows.
pub fn pcurve<S: Scalar>(
    surface: &NurbSurface3D<S>,
    original: &Curve2<S>,
    curve: Option<&NurbCurve3D<S>>,
    start: Vector2<S>,
    end: Vector2<S>,
) -> GeopResult<Curve2<S>> {
    let Some(curve) = curve else {
        return line2(start, end);
    };
    let straight = original.degree == 1 && original.control_points.len() == 2 && {
        let (a, b) = pcurve_ends(original)?;
        let keeps = |k: usize| a[k].could_be_equal(b[k]) && start[k].could_be_equal(end[k]);
        surface.as_plane()?.is_some() || keeps(0) || keeps(1)
    };
    if straight {
        return line2(start, end);
    }
    surface.fit_pcurve(
        curve,
        Some(start),
        Some(end),
        MAX_NODES,
        min_subdivision_size(),
    )
}

/// `surface` over three times its domain in each direction it is straight
/// in — degree 1, two rows of control points of equal weights, neither of
/// them a pole — extended
/// linearly: the same points at the same `(u, v)`, and more around them.
///
/// A face's inner copy can reach past the face where they meet at an
/// inward corner — or, for a plane cut off by the box around the profile,
/// past where the patch ends — so its surface needs room around the face.
/// How much is a free choice; the face's own size is room enough unless
/// the walls are as thick as the face is wide.
pub fn extended<S: Scalar>(mut surface: NurbSurface3D<S>) -> GeopResult<NurbSurface3D<S>> {
    for along_u in [true, false] {
        let (degree, num, other) = if along_u {
            (surface.degree_u, surface.num_u, surface.num_v)
        } else {
            (surface.degree_v, surface.num_v, surface.num_u)
        };
        if degree != 1 || num != 2 {
            continue;
        }
        let index = |row: usize, j: usize| {
            if along_u {
                row * surface.num_v + j
            } else {
                j * surface.num_v + row
            }
        };
        let cps = &surface.control_points;
        if !(0..other).all(|j| cps[index(0, j)][3].could_be_equal(cps[index(1, j)][3])) {
            continue;
        }
        // A row that is a single point — a pole, like a disc's center —
        // would end up inside the extended patch, where it has no normal.
        let pole = |row: usize| {
            let first = dehomogenized(&cps[index(row, 0)]);
            (1..other).all(|j| dehomogenized(&cps[index(row, j)]).could_be_equal(&first))
        };
        if pole(0) || pole(1) {
            continue;
        }
        let mut control_points = cps.clone();
        for j in 0..other {
            let (a, b) = (cps[index(0, j)], cps[index(1, j)]);
            let two = |p: &Vector<S, 4>| p.prod_scalar(S::TWO);
            control_points[index(0, j)] = two(&a).sub(&b);
            control_points[index(1, j)] = two(&b).sub(&a);
        }
        let knots = if along_u {
            &surface.knot_vector_u
        } else {
            &surface.knot_vector_v
        };
        let (lo, hi) = (knots[1], knots[2]);
        let width = hi.sub(lo);
        let (lo, hi) = (lo.sub(width), hi.add(width));
        let knots = vec![lo, lo, hi, hi];
        surface = if along_u {
            NurbSurface3D::try_new(
                1,
                surface.degree_v,
                control_points,
                knots,
                surface.knot_vector_v.clone(),
            )?
        } else {
            NurbSurface3D::try_new(
                surface.degree_u,
                1,
                control_points,
                surface.knot_vector_u.clone(),
                knots,
            )?
        };
    }
    Ok(surface)
}

/// The point the homogeneous control point `p` stands for.
fn dehomogenized<S: Scalar>(p: &Vector<S, 4>) -> Vector3<S> {
    let w = p[3];
    Vector3::from_array([0, 1, 2].map(|k| p[k].div(w).unwrap_or(S::ENTIRE)))
}

/// The inner copy of the vertex at `point`: where the surfaces around it
/// meet, each with where the vertex is on it and whether its face is
/// taken away.
///
/// Gauss-Newton for the point nearest `point` on all of them: each step
/// measures how far off each surface the iterate is, along its normal, and
/// takes the shortest step that corrects all of them at once, to first
/// order. Between planes that is exact in one step.
///
/// Surfaces tangent to each other at the vertex are pieces of one smooth
/// surface — the quarters of a revolved face meeting at its pole — and so
/// one condition, not several: decided once, at the vertex, and each step
/// measures from the nearest foot point on any of the pieces, the foot
/// point on their union. Deciding it at the foot points instead fails
/// where each piece clamps the foot point to its own patch: their normals
/// there differ a little, and the nearly equal conditions leave no step
/// to take. Tangent surfaces where one face is taken away and the other is
/// not cannot be: the walls would have to step across a smooth surface.
///
/// The iterates are seeds for the next step, so they are sharpened — but
/// they are also the answer: where fewer than three surfaces meet, the
/// vertex is free to slide along where they do, and only the steps taken
/// say how far it has. So the answer is carried alongside, unsharpened:
/// `point` less every step, each with the width its own measurement gave
/// it (keep two copies, see `AGENTS.md`). Without that, a vertex on the
/// edge of a patch — where a revolved face's quarters meet — slides a
/// rounding error off it with nothing to say it might not have.
///
/// Public, with [`extended`] and [`pcurve`], for the other operations that
/// rebuild a solid face by face: a draft (`geop_ops_plastic`) moves its
/// vertices onto tilted planes with it.
pub fn offset_vertex<S: Scalar>(
    point: Vector3<S>,
    surfaces: &[(&NurbSurface3D<S>, Vector2<S>, bool)],
) -> GeopResult<Vector3<S>> {
    // The surfaces, grouped by tangency at the vertex.
    let mut groups: Vec<(Vector3<S>, bool, Vec<usize>)> = Vec::new();
    for (i, &(surface, at, removed)) in surfaces.iter().enumerate() {
        let normal = surface.normal(at[0], at[1])?;
        match groups
            .iter_mut()
            .find(|(n, _, _)| n.prod_cross(&normal).norm_sq().could_be_equal(S::ZERO))
        {
            Some((_, other, members)) => {
                if *other != removed {
                    return Err(GeopError::new(
                        "a face taken away is tangent to one kept at a vertex: the walls cannot end on a smooth surface",
                    ));
                }
                members.push(i);
            }
            None => groups.push((normal, removed, vec![i])),
        }
    }
    // The foot point of `p` on surface `i`, and the normal there.
    let foot = |i: usize, p: Vector3<S>| -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let (surface, at, _) = surfaces[i];
        let (u, v) = surface.project(p, at[0].sharpen(), at[1].sharpen(), PROJECT_ITERATIONS)?;
        // The foot point is on the patch: its `(u, v)` are in the domain as
        // much as where the projection says.
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        let (u, v) = (u.intersect(u0.union(u1)), v.intersect(v0.union(v1)));
        Ok((surface.evaluate(u, v)?, surface.normal(u, v)?))
    };
    // The foot point of `p` on the union of a group's pieces: the nearest
    // of theirs — which piece is a free choice where they tie.
    let nearest_foot = |members: &[usize], p: Vector3<S>| -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let mut best: Option<(f64, Vector3<S>, Vector3<S>)> = None;
        for &i in members {
            let (at, normal) = foot(i, p)?;
            let distance = at.sub(&p).norm_sq().upper().to_f64();
            if best.as_ref().is_none_or(|(d, _, _)| distance < *d) {
                best = Some((distance, at, normal));
            }
        }
        let (_, at, normal) = best.expect("a group has members");
        Ok((at, normal))
    };

    let mut seed = point;
    let mut answer = point;
    for _ in 0..VERTEX_ITERATIONS {
        let mut rows: Vec<(Vector3<S>, S)> = Vec::new();
        for (_, _, members) in &groups {
            let (at, normal) = nearest_foot(members, seed)?;
            rows.push((normal, normal.prod_dot(&seed.sub(&at))));
        }
        let step = shortest_step(&rows)?;
        answer = answer.sub(&step);
        // A fixed point of the sharpened iteration: every further step
        // would start from the very same seed and only add its width again
        // — not a tolerance, the seed is bit for bit unchanged.
        let next = seed.sub(&step).sharpen();
        let same = |a: &Vector3<S>, b: &Vector3<S>| {
            (0..3).all(|k| a[k].is_subset_of(b[k]) && b[k].is_subset_of(a[k]))
        };
        if same(&next, &seed) {
            break;
        }
        seed = next;
    }
    // The answer has to be on every group's surface: a condition lost on
    // the way leaves it off one.
    for (g, (_, _, members)) in groups.iter().enumerate() {
        let (at, normal) = nearest_foot(members, answer)?;
        if !at.could_be_equal(&answer) {
            return Err(GeopError::new(format!(
                "the inner copy {answer:?} of the vertex at {point:?} is not on surface group {g} of {} (surfaces {members:?}), whose nearest point is {at:?}, normal {normal:?}",
                groups.len()
            )));
        }
    }
    Ok(answer)
}

/// The shortest `d` with `n_i . d = r_i` for every `(n_i, r_i)` — for more
/// than three conditions, of which three can be met at most, the `d` that
/// misses them least in the least-squares sense.
fn shortest_step<S: Scalar>(rows: &[(Vector3<S>, S)]) -> GeopResult<Vector3<S>> {
    // `d = sum_i lambda_i n_i`, with the Gram matrix `G_ij = n_i . n_j`
    // solving `G lambda = r`.
    fn combine<S: Scalar, const N: usize>(rows: &[(Vector3<S>, S)]) -> GeopResult<Vector3<S>> {
        let mut gram = Matrix::<S, N, N>::zero();
        let mut r = Vector::<S, N>::zero();
        for i in 0..N {
            r[i] = rows[i].1;
            for j in 0..N {
                gram[(i, j)] = rows[i].0.prod_dot(&rows[j].0);
            }
        }
        let lambda = solve_linear_system(&gram, &r)?;
        Ok((0..N).fold(Vector3::zero(), |d, i| {
            d.add(&rows[i].0.prod_scalar(lambda[i]))
        }))
    }
    match rows.len() {
        0 => Err(GeopError::new("a vertex on no surface")),
        1 => combine::<S, 1>(rows),
        2 => combine::<S, 2>(rows),
        3 => combine::<S, 3>(rows),
        _ => {
            let mut normal = Matrix::<S, 3, 3>::zero();
            let mut b = Vector::<S, 3>::zero();
            for (n, r) in rows {
                for i in 0..3 {
                    b[i] = b[i].add(n[i].mul(*r));
                    for j in 0..3 {
                        normal[(i, j)] = normal[(i, j)].add(n[i].mul(n[j]));
                    }
                }
            }
            solve_linear_system(&normal, &b)
        }
    }
}

#[cfg(test)]
mod tests;
