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
    intersection::curve_surface_overlaps,
    meet::{Offset, meet, tangent_groups},
    nurb_curve::{NurbCurve2D, NurbCurve3D, loop_signed_area},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
    shape::{Arc, Circle},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::walk::{Walked, walk},
    vector::{Vector, Vector2, Vector3},
    with_context,
};
use geop_core_topology::{
    Body, Curve2, Curve3, FaceId, Sense, ShellId, SolidId,
    build::{BodySpec, BuiltBody, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
    contains::face::{PointClassification, pcurves_contain},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

/// Seed of the ray casting that finds which outer loop a hole lies in:
/// any gives the same answer (see `pcurves_contain`), a constant keeps it
/// reproducible.
const SEED: u64 = 0x5E11_0000_0000_0001;

/// Hollows the solid `solid` of `part` out to walls `thickness` thick,
/// open where its faces `open` were — see the module docs — and returns
/// the hollowed solid, which replaces it and is named `namer`'s root.
///
/// What stays of the solid keeps its names. What is new is named after
/// what it was made from, with `Scalar` the operation:
///
/// - `shell(Scalar,X)`: the inner copy of the face, edge or vertex `X` kept; for
///   a face `X` taken away, the rim it leaves — `shell(Scalar,X,0)`,
///   `shell(Scalar,X,1)`, ... if that is several faces;
/// - `shell(Scalar,E,start)` / `shell(Scalar,E,end)`: what is left of an edge `E`
///   between two faces taken away, at its start and its end.
pub fn shell(
    part: &mut NamedModel,
    namer: &Namer,
    solid: SolidId,
    open: &[FaceId],
    thickness: Scalar,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "shell({}, {solid}, open={open:?}, {thickness:?})",
        namer.root()
    );
    if !thickness.definitely_greater(Scalar::ZERO) {
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
    let names = part.source_names(&sources).with_context(ctx)?;
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
    part.replace_with_solid(&[solid.into()], result, result_names)
        .with_context(ctx)
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
pub fn thicken(
    part: &mut NamedModel,
    namer: &Namer,
    sheet: ShellId,
    thickness: Scalar,
    along_normal: bool,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "thicken({}, sheet {sheet}, {thickness:?}, along_normal={along_normal})",
        namer.root()
    );
    if !thickness.definitely_greater(Scalar::ZERO) {
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
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, thickness).with_context(ctx)?;
    let (result, result_names) = hollow.result(namer).with_context(ctx)?;
    part.replace_with_solid(&[Body::Sheet(sheet)], result, result_names)
        .with_context(ctx)
}

/// Copies the faces `faces` of `part` — of a solid or of a sheet — into a
/// sheet `distance` along their normals (against them for a negative
/// distance), and returns it: the inner side of the faces shelled (see the
/// module docs), sharing nothing with them, which stay as they are.
///
/// The copy of each face, edge and vertex `X` is named `N(X)`, with `N`
/// the operation.
pub fn offset_faces(
    part: &mut NamedModel,
    namer: &Namer,
    faces: &[FaceId],
    distance: Scalar,
) -> GeopResult<BuiltBody> {
    let ctx = with_context!("offset_faces({}, {faces:?}, {distance:?})", namer.root());
    if distance.could_be_equal(Scalar::ZERO) {
        return Err(GeopError::new(format!(
            "an offset needs a distance other than zero, not {distance:?}"
        )))
        .with_context(ctx);
    }
    let (spec, sources) = part.topology().body_spec(faces, false).with_context(ctx)?;
    let names = part.source_names(&sources).with_context(ctx)?;
    let removed = vec![false; spec.faces.len()];
    let hollow = Hollow::new(&spec, &names, removed, distance.neg()).with_context(ctx)?;
    let (result, result_names) = hollow.inner_side(namer).with_context(ctx)?;
    part.build_body(result, result_names).with_context(ctx)
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
fn inward_corner(spec: &BodySpec, removed: &[bool]) -> GeopResult<Option<(usize, usize, usize)>> {
    for (e, at) in spec.edge_uses().iter().enumerate() {
        let [(f, f_sense, f_pcurve), (g, _, g_pcurve)] = match *at.as_slice() {
            [a, b] if removed[a.0] && !removed[b.0] => [a, b],
            [a, b] if removed[b.0] && !removed[a.0] => [b, a],
            _ => continue,
        };
        let curve = &spec.edges[e].curve;
        let t = curve.mid_parameter()?;
        let point = curve.evaluate(t)?;
        let tangent = curve.tangent(t)?;
        let along = match f_sense {
            Sense::Forward => tangent,
            Sense::Reversed => tangent.neg(),
        };
        let normal = |face: usize, pcurve: &Curve2| -> GeopResult<Vector3> {
            let surface = &spec.faces[face].surface;
            let seed = pcurve.evaluate(pcurve.mid_parameter()?)?;
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
        // inward corner (see the fillet's bend).
        if n_f
            .prod_cross(&along)
            .prod_dot(&n_g)
            .definitely_greater(Scalar::ZERO)
        {
            return Ok(Some((f, g, e)));
        }
    }
    Ok(None)
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
struct Hollow<'a> {
    spec: &'a BodySpec,
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
    inner_surfaces: Vec<NurbSurface3D>,
    /// Per vertex: its inner copy, where it is walled.
    inner_points: Vec<Option<Vector3>>,
    /// Per edge: its inner copy's curve, unless it is open.
    inner_curves: Vec<Option<Curve3>>,
    /// Per open edge: what is left of it at its start and at its end, where
    /// those are walled.
    pieces: Vec<Pieces<Curve3>>,
    /// Per coedge, the `(u, v)` where it ends on its face's inner surface —
    /// where it ends at a walled vertex.
    inner_ends: HashMap<At, Vector2>,
}

impl<'a> Hollow<'a> {
    fn new(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        removed: Vec<bool>,
        thickness: Scalar,
    ) -> GeopResult<Self> {
        let mut walled = vec![false; spec.vertices.len()];
        let mut open_edge = vec![true; spec.edges.len()];
        let mut uses = vec![0usize; spec.edges.len()];
        for (f, face) in spec.faces.iter().enumerate() {
            for lp in face.loops() {
                for c in lp {
                    let (_, end) = spec.ends(c.on)?;
                    walled[end] |= !removed[f];
                    if let CoedgeOn::Edge(e, _) = c.on {
                        open_edge[e] &= removed[f];
                        uses[e] += 1;
                    }
                }
            }
        }
        // Per vertex, each face around it once, with where the vertex is on
        // the face's surface.
        let around = spec.vertex_faces()?;

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
                // Where the inner surfaces around it meet, nearest to it.
                // Surfaces tangent to each other there are pieces of one
                // smooth surface — the quarters of a revolved face meeting
                // at its pole — and so one condition. One whose face is
                // taken away cannot be tangent to one whose face is kept:
                // the walls would have to end on a smooth surface.
                let pieces: Vec<_> = around[v]
                    .iter()
                    .map(|&(f, uv)| (&inner_surfaces[f], (uv[0], uv[1])))
                    .collect();
                let mut offsets = Vec::new();
                for group in tangent_groups(&pieces)? {
                    let f = |i: usize| around[v][i].0;
                    if group.iter().any(|&i| removed[f(i)] != removed[f(group[0])]) {
                        return Err(GeopError::new(format!(
                            "at vertex {}, a face taken away is tangent to one kept: the walls cannot end on a smooth surface",
                            names.vertices[v]
                        )));
                    }
                    offsets.push(Offset {
                        pieces: group.iter().map(|&i| pieces[i]).collect(),
                        offset: Scalar::ZERO,
                    });
                }
                meet(spec.vertices[v], &mut offsets, &[])
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
                && !end
                    .sub(start)
                    .prod_dot(&along)
                    .definitely_greater(Scalar::ZERO)
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
    fn coedges(&self) -> impl Iterator<Item = (At, &'a CoedgeSpec)> + 'a {
        let spec = self.spec;
        spec.faces.iter().enumerate().flat_map(|(f, face)| {
            face.loops()
                .into_iter()
                .enumerate()
                .flat_map(move |(l, lp)| {
                    lp.iter().enumerate().map(move |(k, c)| (At { f, l, k }, c))
                })
        })
    }

    fn coedge(&self, at: At) -> &'a CoedgeSpec {
        &self.spec.faces[at.f].loops()[at.l][at.k]
    }

    /// The coedge before `at` in its loop.
    fn previous(&self, at: At) -> At {
        let n = self.spec.faces[at.f].loops()[at.l].len();
        At {
            k: (at.k + n - 1) % n,
            ..at
        }
    }

    /// The faces the edge `e` runs between.
    fn faces_of_edge(&self, e: usize) -> Vec<usize> {
        self.spec.edge_uses()[e].iter().map(|&(f, ..)| f).collect()
    }

    fn inner_point(&self, v: usize) -> Vector3 {
        self.inner_points[v].expect("a walled vertex has an inner copy")
    }

    /// The curve of the inner copy of edge `e`, unless it is open — then it
    /// lies on the edge itself, and is no part of the result.
    fn inner_curve(&self, e: usize) -> GeopResult<Option<Curve3>> {
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
        if let Some((f, middle)) =
            surface_missed(&curve, self.faces_of_edge(e), &self.inner_surfaces).map_err(ctx)?
        {
            return Err(ctx(GeopError::new(format!(
                "its curve from {start:?} to {end:?} is not on the offset of face {} at {middle:?}: offsetting this kind of edge is not supported",
                self.names.faces[f]
            ))));
        }
        Ok(Some(curve))
    }

    /// What is left of the open edge `e` at its start and at its end: from
    /// a walled end to its inner copy, which lies on the edge.
    fn pieces_of(&self, e: usize) -> GeopResult<Pieces<Curve3>> {
        if !self.open_edge[e] {
            return Ok((None, None));
        }
        let edge = &self.spec.edges[e];
        let piece = |from: Vector3, to: Vector3| shaped_like(&edge.curve, from, to, false);
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
    fn inner_ends(&self) -> GeopResult<HashMap<At, Vector2>> {
        let mut out = HashMap::new();
        for (at, c) in self.coedges() {
            let (_, end) = self.spec.ends(c.on)?;
            if !self.walled[end] {
                continue;
            }
            let seed = c.pcurve.ends()?.1;
            let surface = &self.inner_surfaces[at.f];
            let (u, v) = surface.project_in_domain(
                self.inner_point(end),
                seed[0].sharpen(),
                seed[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            out.insert(at, Vector2::from_array([u, v]));
        }
        Ok(out)
    }

    /// Where the inner copy of the coedge `at` starts and ends on its face's
    /// inner surface.
    fn inner_pins(&self, at: At) -> (Vector2, Vector2) {
        (self.inner_ends[&self.previous(at)], self.inner_ends[&at])
    }

    /// The inner copy of the coedge `at`, on its face's inner surface.
    fn inner_coedge(&self, at: At) -> GeopResult<CoedgeSpec> {
        let c = self.coedge(at);
        let (start, end) = self.inner_pins(at);
        let curve = match c.on {
            CoedgeOn::Edge(e, sense) => {
                Some(sense.orient(self.inner_curves[e].as_ref().expect("not open")))
            }
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
    fn result(&self, namer: &Namer) -> GeopResult<(BodySpec, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec::new(true);
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
        let mut push_edge = |curve: Curve3, start: usize, end: usize, name: String| {
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
            let remap = |lp: &Vec<CoedgeSpec>| -> Vec<CoedgeSpec> {
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

            let inner_loop = |l: usize| -> GeopResult<Vec<CoedgeSpec>> {
                (0..face.loops()[l].len())
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
            let (x, y) = spec.ends(c.on)?;
            for v in [x, y] {
                if side[v].is_none() {
                    side[v] = Some(out.edges.len());
                    out.edges.push(EdgeSpec {
                        curve: NurbCurve3D::line(spec.vertices[v], self.inner_point(v))?,
                        start: at.vertex[v].unwrap(),
                        end: at.inner_vertex[v].unwrap(),
                    });
                    names
                        .edges
                        .push(namer.name(&[&self.names.vertices[v], "side"]));
                }
            }
            let outer = sense.orient(&spec.edges[e].curve);
            let inner = sense.orient(self.inner_curves[e].as_ref().expect("not open"));
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
            let surface = NurbSurface3D::ruled(&inner, &outer).map_err(|err| {
                err.with_context(format!("the wall along edge {}", self.names.edges[e]))
            })?;
            out.faces.push(FaceSpec::on_unit_square(surface, lp)?);
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
    fn inner_side(&self, namer: &Namer) -> GeopResult<(BodySpec, BodyNames)> {
        let spec = self.spec;
        let mut out = BodySpec::new(false);
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
            let inner_loop = |l: usize| -> GeopResult<Vec<CoedgeSpec>> {
                (0..face.loops()[l].len())
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
    fn rims(&self, f: usize, at: &Numbering) -> GeopResult<Vec<FaceSpec>> {
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
        let mut coedges: Vec<(CoedgeSpec, usize, usize)> = Vec::new();
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
            let (x, y) = spec.ends(c.on)?;
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
            let (orig_start, orig_end) = c.pcurve.ends()?;
            let (start_piece, end_piece) = in_order(at.pieces[e], sense);
            let curves = &self.pieces[e];
            let (start_curve, end_curve) = in_order((&curves.0, &curves.1), sense);
            if let (Some(piece), Some(curve)) = (start_piece, start_curve) {
                let to = self.inner_ends[&self.previous(here)];
                let curve = sense.orient(curve);
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
                let curve = sense.orient(curve);
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
            // On from each coedge to the one starting where it ends. One
            // running into a loop already followed runs round that loop
            // into itself.
            let around = walk(first, |&i| {
                let next = starting.get(&coedges[i].2).ok_or_else(|| {
                    GeopError::new(format!(
                        "what is left of face {} does not close up",
                        self.names.faces[f]
                    ))
                })?;
                Ok::<_, GeopError>(Some(*next))
            })?;
            if around.end != Walked::Closed {
                return Err(GeopError::new(format!(
                    "what is left of face {} runs into itself",
                    self.names.faces[f]
                )));
            }
            for &i in &around.nodes {
                used[i] = true;
            }
            loops.push(around.nodes);
        }

        // Outer loops run counter-clockwise in `(u, v)`, holes clockwise;
        // each hole lies in one outer loop.
        let mut outers: Vec<(Vec<usize>, Scalar)> = Vec::new();
        let mut holes: Vec<Vec<usize>> = Vec::new();
        for lp in loops {
            let area = loop_signed_area(lp.iter().map(|&i| &coedges[i].0.pcurve))?;
            if area.definitely_greater(Scalar::ZERO) {
                outers.push((lp, area));
            } else if area.definitely_less(Scalar::ZERO) {
                holes.push(lp);
            } else {
                return Err(GeopError::new(format!(
                    "cannot tell which way a loop of what is left of face {} runs",
                    self.names.faces[f]
                )));
            }
        }
        let mut faces: Vec<FaceSpec> = outers
            .iter()
            .map(|(lp, _)| FaceSpec {
                surface: surface.clone(),
                outer: lp.iter().map(|&i| coedges[i].0.clone()).collect(),
                holes: Vec::new(),
            })
            .collect();
        // A part of the rim may lie inside another's outer loop — the ring
        // around a hole inside the ring around the walls — so a hole is the
        // innermost one's: of the outer loops around it, the smallest. Which
        // one a hole lies in is asked of a point in the middle of one of its
        // pcurves, by ray casting against the outer loop's pcurves.
        for lp in &holes {
            let first = &coedges[lp[0]].0.pcurve;
            let point = first.evaluate(first.mid_parameter()?)?;
            let mut owner: Option<usize> = None;
            for (i, (outer, area)) in outers.iter().enumerate() {
                let pcurves: Vec<&Curve2> = outer.iter().map(|&k| &coedges[k].0.pcurve).collect();
                let inside = match pcurves_contain(
                    surface,
                    &pcurves,
                    point,
                    MAX_NODES,
                    min_subdivision_size(),
                    SEED,
                )? {
                    PointClassification::Inside => true,
                    PointClassification::Outside => false,
                    PointClassification::OnVertex | PointClassification::OnCoedge => {
                        return Err(GeopError::new(format!(
                            "a hole of what is left of face {} meets one of its parts' outer loops",
                            self.names.faces[f]
                        )));
                    }
                };
                if inside && owner.is_none_or(|o| area.definitely_less(outers[o].1)) {
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

/// A curve shaped like `curve` from `start` to `end` — all the way round
/// a circle back to `start`, if `closed`: a line for a straight one, an arc
/// around the same axis for a circular one. Fails for any other curve, and
/// for an `end` not on the circle around that axis through `start`.
fn shaped_like(curve: &Curve3, start: Vector3, end: Vector3, closed: bool) -> GeopResult<Curve3> {
    if curve.as_line()?.is_some() {
        return NurbCurve3D::line(start, end);
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
            let p = cp.dehomogenized()?;
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
    let (from, to) = moved.ends()?;
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

/// The first of the faces `faces`, indices into `surfaces`, whose surface
/// `curve` does not lie on all along, with a point that shows it: the
/// middle of the first stretch of the curve that is off the surface. `None`
/// if the curve could lie on all of them. A curve built for an edge between
/// moved faces is only right if it does.
///
/// Whether it lies on a surface is what [`curve_surface_overlaps`] finds:
/// a stretch of it on the surface runs between two points where the curve
/// or the surface ends, so all of the curve lies on it if one stretch runs
/// from the curve's start to its end.
pub fn surface_missed(
    curve: &NurbCurve3D,
    faces: impl IntoIterator<Item = usize>,
    surfaces: &[NurbSurface3D],
) -> GeopResult<Option<(usize, Vector3)>> {
    let (t0, t1) = curve.domain();
    for f in faces {
        let overlaps =
            curve_surface_overlaps(curve, &surfaces[f], MAX_NODES, min_subdivision_size())?;
        if overlaps
            .iter()
            .any(|o| o.start.t.could_be_equal(t0) && o.end.t.could_be_equal(t1))
        {
            continue;
        }
        // The first stretch off it: up to the first overlap, or after it.
        let (from, to) = match overlaps.first() {
            Some(o) if o.start.t.could_be_equal(t0) => (
                o.end.t.upper(),
                overlaps.get(1).map_or(t1, |n| n.start.t.lower()),
            ),
            Some(o) => (t0, o.start.t.lower()),
            None => (t0, t1),
        };
        let off = curve.evaluate(from.add(to).div(Scalar::TWO)?.sharpen())?;
        return Ok(Some((f, off)));
    }
    Ok(None)
}

/// The pcurve on `surface` of a coedge that runs along `curve` — or sits at
/// a vertex, without one — from `(u, v)` `start` to `end`. Straight where
/// the coedge it is a copy of had a straight pcurve, `original`, and its
/// inner copy is straight too: on a plane, whose `(u, v)` are affine, or
/// along the same iso-line — the line keeps its `u` (or `v`) — as a
/// cylinder's rim or seam does. Fitted otherwise: a sphere's meridian cut
/// by a plane is straight, but its inner copy, where the offset plane cuts
/// the offset sphere, is a small circle no straight pcurve follows.
pub fn pcurve(
    surface: &NurbSurface3D,
    original: &Curve2,
    curve: Option<&NurbCurve3D>,
    start: Vector2,
    end: Vector2,
) -> GeopResult<Curve2> {
    let Some(curve) = curve else {
        return NurbCurve2D::line(start, end);
    };
    let straight = original.degree == 1 && original.control_points.len() == 2 && {
        let (a, b) = original.ends()?;
        let keeps = |k: usize| a[k].could_be_equal(b[k]) && start[k].could_be_equal(end[k]);
        surface.as_plane()?.is_some() || keeps(0) || keeps(1)
    };
    if straight {
        return NurbCurve2D::line(start, end);
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
pub fn extended(mut surface: NurbSurface3D) -> GeopResult<NurbSurface3D> {
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
        let pole = |row: usize| -> GeopResult<bool> {
            let first = cps[index(row, 0)].dehomogenized()?;
            for j in 1..other {
                if !cps[index(row, j)].dehomogenized()?.could_be_equal(&first) {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        if pole(0)? || pole(1)? {
            continue;
        }
        let mut control_points = cps.clone();
        for j in 0..other {
            let (a, b) = (cps[index(0, j)], cps[index(1, j)]);
            let two = |p: &Vector<4>| p.prod_scalar(Scalar::TWO);
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

#[cfg(test)]
mod tests;
