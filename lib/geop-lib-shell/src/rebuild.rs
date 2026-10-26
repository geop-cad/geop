//! [`rebuilt`]: a solid rebuilt face by face — some of its faces on new
//! surfaces, every vertex they reach moved to where the surfaces around it
//! now meet, every edge they reach to where its two faces now meet.
//!
//! A draft, a face moved or offset in place, a face replaced, a face
//! deleted and its neighbours grown over the gap: each changes the
//! surfaces some faces lie on and keeps (or, deleting, edits first) the
//! topology. What follows from the new surfaces is the same every time, and
//! is done here, for any NURBS surfaces:
//!
//! - **Room.** A face's boundary may move past where its patch ends, so
//!   every surface touched gets room around it ([`extended`]).
//! - **Vertices.** A vertex moves to the point nearest where it was that
//!   lies on the smooth surfaces around it ([`meet`]), one condition per
//!   smooth surface ([`tangent_groups`]). Where only two smooth surfaces
//!   meet, the vertex is the end of an edge lying within one of them — a
//!   seam between two pieces of it — and it stays on that edge's curve: the
//!   curve, carried on past its ends, crossing the other surface.
//! - **Edges.** An edge between two smooth surfaces is traced where they
//!   meet between its moved vertices, following the edge it was: each
//!   point of it where the surfaces meet in the plane square to the old
//!   edge there, the old point shifted with its vertices as the seed, and
//!   a cubic through them widened to enclose the curve between them. A
//!   straight edge that stays on both surfaces is kept straight. An edge
//!   within one smooth surface, a seam, is its old curve carried on or cut
//!   back to its moved ends.
//! - **Pcurves.** Every coedge that changed is fitted anew on its face's
//!   surface between `(u, v)` worked out once per junction, so the coedges
//!   meeting there agree on it exactly.
//!
//! Every curve built is checked to lie on its faces' surfaces, and a curve
//! traced to keep running the way the old edge ran: an edge that turns
//! around — a face moved so far that a neighbour vanishes, or two of them
//! now cross — is refused, by name, as is a vertex whose surfaces no longer
//! meet near it. The topology never changes here.

use geop_core_geometry::{
    intersection::{
        Intersections, curve_curve_intersect, curve_surface_intersect, refine_crossing,
    },
    meet::{Offset, meet, tangent_groups},
    nurb_curve::{NurbCurve, NurbCurve3D, true_point_fractions},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{geop_error::WithContext, with_context};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::refine::{Refine, Refined, refine},
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    Curve2, Curve3, Sense,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec, FacesAround},
    contains::face::{PointClassification, pcurves_contain},
};
use geop_core_topology::{FaceId, SolidId};
use geop_ops_model::{BodyNames, NamedModel, Namer};

use crate::shell::{extended, pcurve, surface_missed};

/// The legs a traced edge is fitted over at first, and at most: doubled
/// while the fit's enclosure is wider than [`TRACE_DRIFT`].
const FIRST_LEGS: usize = 8;
const MOST_LEGS: usize = 256;

/// How wide a traced edge's enclosure may come out before its legs are
/// split finer: how fast the fit converges, not what it means — the fit
/// encloses the curve between its points at every resolution.
const TRACE_DRIFT: f64 = 1e-5;

/// Seed of the ray casting that tells whether a hole lies inside its
/// face's outer loop: any gives the same answer (see `pcurves_contain`).
const SEED: u64 = 0x2EB0_0000_0000_0001;

/// Whether [`replace_faces`] takes along the faces a face it puts on a new
/// surface runs smoothly into, or refuses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Smooth {
    Refuse,
    Follow,
}

/// Puts the faces `faces` of one solid of `part` on new surfaces —
/// `surface(name, old)` gives the one for the face named `name`, on the
/// surface `old` — and rebuilds the solid around them ([`rebuilt`]): the
/// result replaces it, named `namer`'s root, and every face, edge and
/// vertex keeps its name. What a draft, a face moved, offset or replaced
/// do. A new surface's normal points out of the solid, as the old one's
/// did.
///
/// A face that runs smoothly into a neighbour — a piece of a cylinder
/// beside the next, a wall into its fillet — takes it along if `smooth`
/// says to follow: the faces the new surfaces are made for are then every
/// face the picked ones run smoothly into, on and on. Otherwise that is
/// refused (see [`tangent_to_kept`]).
///
/// Refused, by name: faces of several solids, or of none.
pub fn replace_faces(
    part: &mut NamedModel,
    namer: &Namer,
    faces: &[FaceId],
    smooth: Smooth,
    mut surface: impl FnMut(&str, &NurbSurface3D) -> GeopResult<NurbSurface3D>,
) -> GeopResult<SolidId> {
    let ctx = with_context!("replace_faces({}, faces={faces:?})", namer.root());
    let Some(&first) = faces.first() else {
        return Err(GeopError::new("pick the faces")).with_context(ctx);
    };
    let solid = part
        .solid_of_face(first, "only faces of a solid can be moved")
        .with_context(ctx)?;
    let model = part.topology();
    let all = model.solid_faces(solid).with_context(ctx)?;
    if let Some(&f) = faces.iter().find(|f| !all.contains(f)) {
        return Err(GeopError::new(format!(
            "face {} is not of the same solid as face {}: pick the faces of one solid at a time",
            part.name(f)?,
            part.name(first)?
        )))
        .with_context(ctx);
    }
    let (mut spec, sources) = model.body_spec(&all, true).with_context(ctx)?;
    let names = BodyNames {
        solid: Some(namer.root()),
        ..part.source_names(&sources)?
    };
    let mut replaced: Vec<bool> = sources.faces.iter().map(|f| faces.contains(f)).collect();
    while let Some((f, g, e)) = tangent_to_kept(&spec, &replaced).with_context(ctx)? {
        if smooth == Smooth::Follow {
            replaced[g] = true;
            continue;
        }
        return Err(GeopError::new(format!(
            "face {} meets face {} tangentially along edge {} — a fillet or a smooth blend: moving a face next to one, without it, is not supported",
            names.faces[f], names.faces[g], names.edges[e]
        )))
        .with_context(ctx);
    }
    for (f, face) in spec.faces.iter_mut().enumerate() {
        if replaced[f] {
            let roomy = room(face.surface.clone()).with_context(ctx)?;
            face.surface = surface(&names.faces[f], &roomy).with_context(ctx)?;
        }
    }
    let reseat = vec![false; spec.vertices.len()];
    let result = rebuilt(&spec, &names, &replaced, &reseat).with_context(ctx)?;
    part.replace_with_solid(&[solid.into()], result, names)
        .with_context(ctx)
}

/// The body `spec` rebuilt: the faces marked `replaced` lie on the
/// surfaces `spec` gives them now — made from the old ones given room
/// first ([`room`]): a fitted offset given room after, its end span of
/// many carried on, is extrapolated and widened, and an exact one given
/// room before and after is widened twice — every vertex they reach — and every
/// vertex marked `reseat`, at the point `spec` gives as a seed — moves to
/// where the surfaces around it meet, every edge they reach to where its
/// faces' surfaces meet (see the module docs). `spec`'s edge curves are
/// where the edges were: the guides the new ones follow. `names` name its
/// entities, for the errors.
pub fn rebuilt(
    spec: &BodySpec,
    names: &BodyNames,
    replaced: &[bool],
    reseat: &[bool],
) -> GeopResult<BodySpec> {
    Rebuild::new(spec, names, replaced, reseat)?.result()
}

/// A face to be put on another surface, `f`, that meets a face kept, `g`,
/// tangentially along the edge `e` — a fillet or a smooth blend along it —
/// if there is one: `(f, g, e)`. Asked of `spec` before the surfaces change.
/// A face kept that runs smoothly into one that moves would have to move
/// with it, which [`rebuilt`] does not do: they would no longer meet along
/// the edge, or cross where they should touch.
pub fn tangent_to_kept(
    spec: &BodySpec,
    replaced: &[bool],
) -> GeopResult<Option<(usize, usize, usize)>> {
    for (e, uses) in spec.edge_uses().iter().enumerate() {
        let (f, g) = match uses.as_slice() {
            [(f, ..), (g, ..)] if replaced[*f] && !replaced[*g] => (*f, *g),
            [(g, ..), (f, ..)] if replaced[*f] && !replaced[*g] => (*f, *g),
            _ => continue,
        };
        if smooth_along(spec, &spec.edges[e], f, g)? {
            return Ok(Some((f, g, e)));
        }
    }
    Ok(None)
}

/// Whether the faces `f` and `g` of `spec` are pieces of one smooth surface
/// along `edge`: their normals halfway along it could be parallel.
pub(crate) fn smooth_along(
    spec: &BodySpec,
    edge: &EdgeSpec,
    f: usize,
    g: usize,
) -> GeopResult<bool> {
    let t = edge.curve.mid_parameter()?;
    let point = edge.curve.evaluate(t)?;
    let normal = |f: usize| {
        spec.faces[f]
            .surface
            .normal_at(&point, MAX_NODES, min_subdivision_size())
    };
    Ok(normal(f)?
        .prod_cross(&normal(g)?)
        .norm_sq()
        .could_be_equal(Scalar::ZERO))
}

/// The body described as a [`BodySpec`], and where rebuilding it moves what.
struct Rebuild<'a> {
    spec: &'a BodySpec,
    names: &'a BodyNames,
    replaced: &'a [bool],
    /// Per face: the surface it lies on afterwards, with room around it
    /// wherever its boundary may move.
    surfaces: Vec<NurbSurface3D>,
    /// Per face: whether a vertex of it moves, or it is replaced.
    touched: Vec<bool>,
    /// Per edge: the curve it lies on, carried on past its ends, if it is a
    /// seam within one smooth surface that changes (see [`Rebuild::seam`]).
    seams: Vec<Option<Curve3>>,
    /// Per vertex: whether it moves, and where to.
    moved: Vec<bool>,
    points: Vec<Vector3>,
    /// Per edge: its new curve, if it changes.
    curves: Vec<Option<Curve3>>,
}

impl<'a> Rebuild<'a> {
    fn new(
        spec: &'a BodySpec,
        names: &'a BodyNames,
        replaced: &'a [bool],
        reseat: &[bool],
    ) -> GeopResult<Self> {
        let around = spec.vertex_faces()?;
        let moved: Vec<bool> = (around.iter().enumerate())
            .map(|(v, faces)| reseat[v] || faces.iter().any(|&(f, _)| replaced[f]))
            .collect();
        let touched: Vec<bool> = (0..spec.faces.len())
            .map(|f| {
                replaced[f]
                    || (around.iter().enumerate())
                        .any(|(v, faces)| moved[v] && faces.iter().any(|&(g, _)| g == f))
            })
            .collect();
        // A face put on a new surface came with room: the new surface was
        // made from the old with room around it (see `room`), not given
        // room after.
        let surfaces = (spec.faces.iter().zip(&touched).zip(replaced))
            .map(|((face, &touched), &replaced)| {
                if touched && !replaced {
                    room(face.surface.clone())
                } else {
                    Ok(face.surface.clone())
                }
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let mut rebuild = Self {
            spec,
            names,
            replaced,
            surfaces,
            touched,
            seams: Vec::new(),
            moved,
            points: spec.vertices.clone(),
            curves: Vec::new(),
        };
        let along: Vec<Vec<(usize, Sense, &Curve2)>> = spec.edge_uses();
        rebuild.seams = (0..spec.edges.len())
            .map(|e| {
                rebuild
                    .seam(e, &along[e])
                    .map_err(|err| err.with_context(format!("edge {}", names.edges[e])))
            })
            .collect::<GeopResult<_>>()?;
        for (v, faces) in around.iter().enumerate() {
            if rebuild.moved[v] {
                rebuild.points[v] = rebuild
                    .vertex(v, faces)
                    .map_err(|e| e.with_context(format!("moving vertex {}", names.vertices[v])))?;
            }
        }
        rebuild.curves = (0..spec.edges.len())
            .map(|e| {
                rebuild
                    .curve(e, &along[e])
                    .map_err(|err| err.with_context(format!("edge {}", names.edges[e])))
            })
            .collect::<GeopResult<_>>()?;
        Ok(rebuild)
    }

    /// Where the vertex `v`, with the faces `faces` around it, goes (see the
    /// module docs).
    fn vertex(&self, v: usize, faces: &FacesAround) -> GeopResult<Vector3> {
        let seed = self.spec.vertices[v];
        let mut pieces = Vec::with_capacity(faces.len());
        for &(f, uv) in faces {
            let surface = &self.surfaces[f];
            // Where it was on the face, if the new surface still has a pole
            // there, or else its foot point on the new surface.
            let at_pole = surface.pole_at(uv[0], uv[1]).is_some();
            let (u, w) = if self.replaced[f] && !at_pole {
                let (u, w) = surface
                    .project_in_domain(seed, uv[0].sharpen(), uv[1].sharpen(), PROJECT_ITERATIONS)
                    .map_err(|e| {
                        e.with_context(format!(
                            "projecting {seed:?} onto the new surface of face {} from {uv:?}",
                            self.names.faces[f]
                        ))
                    })?;
                (u.sharpen(), w.sharpen())
            } else {
                (uv[0], uv[1])
            };
            pieces.push((surface, (u, w)));
        }
        // A vertex at a pole of its faces — on the axis of a revolved face —
        // is the pole of their new surfaces, where those are one point: the
        // topology puts it there. Poles apart are coordinates, not where
        // the faces meet — the walls of a pyramid, each ruled into its tip,
        // offset: their poles move each along its own normal. A face with a
        // pole among faces without — a rounded box's corner patch at the
        // blends' ends — is no such vertex either. Then the surfaces say
        // where it goes, and the conditions below read a pole's normal as
        // its limit.
        let poles: Vec<_> = (pieces.iter())
            .map(|(s, (u, w))| s.pole_at(*u, *w))
            .collect();
        if let Some(pole) = poles.iter().flatten().next()
            && poles
                .iter()
                .all(|p| p.is_some_and(|p| p.could_be_equal(pole)))
        {
            return Ok(*pole);
        }
        let groups = tangent_groups(&pieces)
            .map_err(|e| e.with_context(format!("grouping its faces' surfaces at {pieces:?}")))?;
        // On the border of a sheet, where an edge bounds one face only,
        // nothing but the surfaces holds the vertex: where they meet nearest
        // to where it was is as good as any point of them, a free choice.
        let on_border = (self.spec.edge_uses().iter().zip(&self.spec.edges))
            .any(|(uses, edge)| uses.len() == 1 && (edge.start == v || edge.end == v));
        let fewer = |determined: GeopResult<Vector3>| match determined {
            Err(_) if on_border => nearest(seed, &pieces, &groups),
            determined => determined,
        };
        match groups.len() {
            0 | 1 => fewer(self.within(faces)),
            2 => fewer(self.on_seam(v, faces, &pieces, &groups)),
            // Three surfaces meet in a point; four or more only where they
            // happen to. Elsewhere the vertex would open into an edge,
            // which the rebuild does not do: refused by its faces' names.
            n => nearest(seed, &pieces, &groups).map_err(|e| {
                let names: Vec<&str> = faces.iter().map(|&(f, _)| self.names.faces[f].as_str()).collect();
                GeopError::new(format!(
                    "the {n} smooth surfaces of faces {names:?} around it do not meet in one point: it would open into an edge, which is not supported ({})",
                    e.root_message()
                ))
            }),
        }
        .map_err(|e| e.with_context(format!("where its {} smooth surfaces meet", groups.len())))
    }

    /// Where a vertex goes that lies within one smooth surface, its faces
    /// `faces` — pieces of it, where seams cross — all put on new surfaces that
    /// keep their parametrization, as a shift or an offset does: where each
    /// is at the vertex's `(u, v)`, all enclosures of one point, so their
    /// intersection is too. Not determined otherwise.
    fn within(&self, faces: &FacesAround) -> GeopResult<Vector3> {
        let mut point: Option<Vector3> = None;
        for &(f, uv) in faces {
            let here = self.surfaces[f].evaluate(uv[0], uv[1])?;
            point = Some(match point {
                None => here,
                Some(p) if p.could_be_equal(&here) => {
                    Vector3::from_array(std::array::from_fn(|k| p[k].intersect(here[k])))
                }
                Some(p) => {
                    return Err(GeopError::new(format!(
                        "it lies within one smooth surface, whose pieces now put it at {p:?} and at {here:?}: where it goes is not determined"
                    )));
                }
            });
        }
        point.ok_or_else(|| GeopError::new("no face meets there"))
    }

    /// Where the vertex `v` goes where only two smooth surfaces meet, the
    /// `groups` of `pieces`, the surfaces of `faces` around it: on the
    /// curve of an edge from it within one of them, a seam that stays,
    /// where that curve, carried on past its ends, crosses the other.
    fn on_seam(
        &self,
        v: usize,
        faces: &FacesAround,
        pieces: &[(&NurbSurface3D, (Scalar, Scalar))],
        groups: &[Vec<usize>],
    ) -> GeopResult<Vector3> {
        let group_of = |f: usize| {
            let i = faces.iter().position(|&(g, _)| g == f)?;
            groups.iter().position(|group| group.contains(&i))
        };
        let uses = self.spec.edge_uses();
        let seams: Vec<(usize, &Curve3, usize)> = (self.spec.edges.iter().enumerate())
            .filter(|(_, edge)| edge.start == v || edge.end == v)
            .filter_map(|(e, _)| {
                let curve = self.seams[e].as_ref()?;
                let [(f, ..), (g, ..)] = uses[e].as_slice() else {
                    return None;
                };
                let group = group_of(*f)?;
                (group_of(*g) == Some(group)).then_some((e, curve, group))
            })
            .collect();
        let seed = self.spec.vertices[v];
        // Every seam from it must cross the other surface at one point:
        // seams of both surfaces meeting there, which a move takes apart,
        // leave no vertex where both end.
        let mut found: Option<(usize, Vector3)> = None;
        for &(e, curve, group) in &seams {
            let point = self.crossing(curve, &groups[1 - group], pieces, seed)?;
            match found {
                Some((first, at)) if !at.could_be_equal(&point) => {
                    return Err(GeopError::new(format!(
                        "it ends the seams {} and {}, which no longer meet: they cross the other surface at {at:?} and at {point:?} — taking them apart is not supported",
                        self.names.edges[first], self.names.edges[e]
                    )));
                }
                Some(_) => {}
                None => found = Some((e, point)),
            }
        }
        found.map(|(_, p)| p).ok_or_else(|| {
            GeopError::new(
                "only two smooth surfaces meet there, and no edge from it runs within one of them: where it goes is not determined",
            )
        })
    }

    /// Where `curve`, a seam carried on, crosses the surface `other`, a
    /// group of `pieces`, nearest to `seed`.
    fn crossing(
        &self,
        curve: &Curve3,
        other: &[usize],
        pieces: &[(&NurbSurface3D, (Scalar, Scalar))],
        seed: Vector3,
    ) -> GeopResult<Vector3> {
        let mut best: Option<(Vector3, f64)> = None;
        for &i in other {
            let (surface, _) = pieces[i];
            let hits = match curve_surface_intersect(
                curve,
                surface,
                8,
                MAX_NODES,
                min_subdivision_size(),
            )? {
                Intersections::Found(hits) => hits,
                Intersections::Coincident(_) => {
                    return Err(GeopError::new(
                        "the seam from it lies on the moved surface: where it goes is not determined",
                    ));
                }
            };
            for (t, uv) in hits {
                let (t, uv) = refine_crossing(curve, surface, t, uv);
                let ctx = |e: GeopError| {
                    e.with_context(format!(
                        "the seam crossing surface {i} at t={t:?}, uv={uv:?}, the surface's domain {:?} x {:?}",
                        surface.domain_u(),
                        surface.domain_v()
                    ))
                };
                let on_curve = curve.evaluate(t).map_err(ctx)?;
                // At a pole the collapsed parameter says nothing — the
                // crossing's enclosure of it may run the whole row — and
                // the point is the pole.
                let on_surface = match surface.pole_at(uv[0], uv[1]) {
                    Some(pole) => pole,
                    None => surface.evaluate(uv[0], uv[1]).map_err(ctx)?,
                };
                if !on_curve.could_be_equal(&on_surface) {
                    continue;
                }
                // Both enclose the crossing: so does what they share.
                let point = Vector3::from_array(std::array::from_fn(|k| {
                    on_curve[k].intersect(on_surface[k])
                }));
                let distance = point.sub(&seed).norm_sq().to_f64();
                if best.is_none_or(|(_, d)| distance < d) {
                    best = Some((point, distance));
                }
            }
        }
        best.map(|(p, _)| p).ok_or_else(|| {
            GeopError::new(format!(
                "the seam from it, carried on, does not cross the moved surface: the face around {seed:?} no longer reaches it"
            ))
        })
    }

    /// The new curve of `edge`, on the border of a sheet, bounding only
    /// the face `f` of the use `(f, sense, pcurve)`: its pcurve there, its
    /// ends shifted to where its moved vertices are on the face's surface,
    /// on that surface — straight, where it was straight and that lies on
    /// the surface.
    fn border(
        &self,
        edge: &EdgeSpec,
        (f, sense, pcurve): (usize, Sense, &Curve2),
    ) -> GeopResult<Curve3> {
        let (start, end) = (self.points[edge.start], self.points[edge.end]);
        let surface = &self.surfaces[f];
        let mut curve = None;
        if edge.curve.degree == 1 && edge.curve.control_points.len() == 2 {
            let line = NurbCurve3D::line(start, end)?;
            if surface_missed(&line, [f], &self.surfaces)?.is_none() {
                curve = Some(line);
            }
        }
        let curve = match curve {
            Some(line) => line,
            None => {
                // Each point of the edge carried to its foot point on the
                // new surface, the way its ends were (see `Rebuild::vertex`):
                // from the edge's own curve, not its pcurve, which is as
                // wide as its fit to the curve — evaluated along it, a
                // traced border's copy came out a hundred times wider than
                // the border.
                let along = match sense {
                    Sense::Forward => pcurve.clone(),
                    Sense::Reversed => pcurve.reverse(),
                };
                let (t0, t1) = edge.curve.domain();
                let (p0, p1) = along.domain();
                let at = |frac: Scalar| -> GeopResult<Vector3> {
                    let point = edge
                        .curve
                        .evaluate(Scalar::interpolate(t0, t1, frac).sharpen())?;
                    let seed = along.evaluate(Scalar::interpolate(p0, p1, frac).sharpen())?;
                    let (u, v) = surface.project_in_domain(
                        point,
                        seed[0].sharpen(),
                        seed[1].sharpen(),
                        PROJECT_ITERATIONS,
                    )?;
                    surface.evaluate(u, v)
                };
                fitted(at, (start, end), |_| Ok(()))
                    .map_err(|e| e.with_context("fitting it on its face's new surface"))?
            }
        };
        if let Some((_, middle)) = surface_missed(&curve, [f], &self.surfaces)? {
            return Err(GeopError::new(format!(
                "its new curve from {start:?} to {end:?} is not on face {} at {middle:?}",
                self.names.faces[f]
            )));
        }
        Ok(curve)
    }

    /// The new curve of edge `e`, used by the faces `uses`, if it changes.
    fn curve(&self, e: usize, uses: &[(usize, Sense, &Curve2)]) -> GeopResult<Option<Curve3>> {
        let edge = &self.spec.edges[e];
        let changed = self.moved[edge.start]
            || self.moved[edge.end]
            || uses.iter().any(|&(f, ..)| self.replaced[f]);
        if !changed {
            return Ok(None);
        }
        let (start, end) = (self.points[edge.start], self.points[edge.end]);
        if edge.start != edge.end && start.could_be_equal(&end) {
            return Err(GeopError::new(format!(
                "moved this far, it shrinks to a point at {start:?}: a face it bounds would vanish"
            )));
        }
        if let [border] = uses {
            return self.border(edge, *border).map(Some);
        }
        let [(f, f_sense, f_pcurve), (g, g_sense, g_pcurve)] = uses else {
            return Err(GeopError::new(format!(
                "it bounds {} faces, not one or two: moving it is not supported",
                uses.len()
            )));
        };
        let faces = [*f, *g];
        // Straight where it was, and the line between its moved ends lies
        // on both faces.
        let straight = (edge.curve.degree == 1 && edge.curve.control_points.len() == 2)
            .then(|| NurbCurve3D::line(start, end))
            .transpose()?
            .filter(|line| {
                matches!(surface_missed(line, faces, &self.surfaces), Ok(None))
                    && turns_around(&edge.curve, &[start, end]).is_ok()
            });
        let curve = match (&self.seams[e], straight) {
            // A seam that stays: its moved ends on it. One whose ends left it
            // — two coplanar faces, the vertices at its ends put where other
            // faces now cross — is any curve between them in the surface,
            // and a straight one was straight.
            (Some(seam), straight) => match (restricted(seam, start, end), straight) {
                (Ok(along), _) => along,
                (Err(_), Some(line)) => line,
                (Err(err), None) => return Err(err),
            },
            (None, Some(line)) => line,
            (None, None) => {
                let uses = [(*f, f_sense, *f_pcurve), (*g, g_sense, *g_pcurve)];
                match self.on_iso_line(edge, start, end, uses)? {
                    Some(iso) => iso,
                    None => self.traced(&edge.curve, start, end, uses)?,
                }
            }
        };
        if let Some((face, middle)) = surface_missed(&curve, faces, &self.surfaces)? {
            return Err(GeopError::new(format!(
                "its new curve from {start:?} to {end:?} is not on face {} at {middle:?}",
                self.names.faces[face]
            )));
        }
        Ok(Some(curve))
    }

    /// The new curve of `edge`, from `start` to `end`, if it runs along an
    /// iso-line of a face it bounds, as it did — a cylinder's rim, a wall's
    /// edge on a cap — and that iso-line, on the face's new surface,
    /// passes through both moved ends and lies on the other face's surface
    /// too: then it is that iso-line, exactly ([`iso_line`]), not a trace.
    fn on_iso_line(
        &self,
        edge: &EdgeSpec,
        start: Vector3,
        end: Vector3,
        uses: [(usize, &Sense, &Curve2); 2],
    ) -> GeopResult<Option<Curve3>> {
        let faces = uses.map(|(f, ..)| f);
        for (f, sense, pcurve) in uses {
            let along = match sense {
                Sense::Forward => pcurve.clone(),
                Sense::Reversed => pcurve.reverse(),
            };
            let Some(line) = iso_line(&self.surfaces[f], &along)? else {
                continue;
            };
            let Ok(curve) = restricted(&line, start, end) else {
                continue;
            };
            if surface_missed(&curve, faces, &self.surfaces)?.is_none()
                && turns_around(&edge.curve, &[start, end]).is_ok()
            {
                return Ok(Some(curve));
            }
        }
        Ok(None)
    }

    /// The curve the edge `e`, used by the faces `uses`, lies on if it is a
    /// seam — its faces pieces of one smooth surface — carried on past its
    /// ends, so a moved end can be found on it: the edge's own curve where
    /// both faces stay, the new surface along the edge's pcurve where both
    /// are put on new surfaces (that keep their parametrization, as a shift
    /// or an offset does). None for an edge that is no seam, or that
    /// nothing moves.
    fn seam(&self, e: usize, uses: &[(usize, Sense, &Curve2)]) -> GeopResult<Option<Curve3>> {
        let [(f, f_sense, f_pcurve), (g, _, g_pcurve)] = uses else {
            return Ok(None);
        };
        if !self.touched[*f] && !self.touched[*g] {
            return Ok(None);
        }
        let normal = |f: usize, pcurve: &Curve2| -> GeopResult<Vector3> {
            let uv = pcurve.evaluate(pcurve.mid_parameter()?)?;
            self.surfaces[f].normal(uv[0], uv[1])
        };
        let smooth = normal(*f, f_pcurve)?
            .prod_cross(&normal(*g, g_pcurve)?)
            .norm_sq()
            .could_be_equal(Scalar::ZERO);
        if !smooth {
            return Ok(None);
        }
        match (self.replaced[*f], self.replaced[*g]) {
            (false, false) => Ok(Some(carried_on(&self.spec.edges[e].curve, |_| Ok(true))?)),
            (true, true) => {
                let along = match f_sense {
                    Sense::Forward => (*f_pcurve).clone(),
                    Sense::Reversed => f_pcurve.reverse(),
                };
                let surface = &self.surfaces[*f];
                if let Some(line) = iso_line(surface, &along)? {
                    return Ok(Some(line));
                }
                // Only as far as the surface goes: its ends in its domain.
                let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
                let pcurve = carried_on(&along, |longer| {
                    let (a, b) = longer.ends()?;
                    Ok([a, b].iter().all(|p| {
                        !p[0].definitely_less(u0)
                            && !p[0].definitely_greater(u1)
                            && !p[1].definitely_less(v0)
                            && !p[1].definitely_greater(v1)
                    }))
                })?;
                let (t0, t1) = pcurve.domain();
                let at = |frac: Scalar| -> GeopResult<Vector3> {
                    let uv = pcurve.evaluate(Scalar::interpolate(t0, t1, frac).sharpen())?;
                    surface.evaluate(uv[0], uv[1])
                };
                let ends = (at(Scalar::ZERO)?, at(Scalar::ONE)?);
                Ok(Some(fitted(at, ends, |_| Ok(())).map_err(|err| {
                    err.with_context("fitting the seam on its new surface")
                })?))
            }
            _ => Err(GeopError::new(format!(
                "faces {} and {} are pieces of one smooth surface, and only one of them is put on another surface: not supported",
                self.names.faces[*f], self.names.faces[*g]
            ))),
        }
    }

    /// The curve where the surfaces of the two faces in `uses` meet, from
    /// `start` to `end`, following `guide` (see the module docs). Each use
    /// gives the face, the sense its coedge runs the edge in and its old
    /// pcurve, whose `(u, v)` seed the search on the face's surface.
    fn traced(
        &self,
        guide: &Curve3,
        start: Vector3,
        end: Vector3,
        uses: [(usize, &Sense, &Curve2); 2],
    ) -> GeopResult<Curve3> {
        let (t0, t1) = guide.domain();
        let (a, b) = guide.ends()?;
        let (shift0, shift1) = (start.sub(&a), end.sub(&b));
        // Where the edge is `frac` of the way along: on both surfaces, in
        // the plane square to the guide there through the guide's point
        // shifted with the edge's ends.
        let on_both = |frac: Scalar| -> GeopResult<Vector3> {
            let t = Scalar::interpolate(t0, t1, frac).sharpen();
            let seed = guide
                .evaluate(t)?
                .add(&shift0.prod_scalar(Scalar::ONE.sub(frac)))
                .add(&shift1.prod_scalar(frac))
                .sharpen();
            let normal = guide.tangent(t)?.normalize()?.sharpen();
            let mut offsets = Vec::with_capacity(2);
            for (f, sense, pcurve) in uses {
                let (p0, p1) = pcurve.domain();
                let along = match sense {
                    Sense::Forward => frac,
                    Sense::Reversed => Scalar::ONE.sub(frac),
                };
                let uv = pcurve.evaluate(Scalar::interpolate(p0, p1, along).sharpen())?;
                offsets.push(Offset::new(
                    &self.surfaces[f],
                    (uv[0].sharpen(), uv[1].sharpen()),
                    Scalar::ZERO,
                ));
            }
            meet(seed, &mut offsets, &[(normal, seed)])
        };
        fitted(on_both, (start, end), |points| turns_around(guide, points))
            .map_err(|e| e.with_context("tracing it where its faces meet"))
    }

    /// The rebuilt body, described whole.
    fn result(&self) -> GeopResult<BodySpec> {
        let spec = self.spec;
        let edges = (spec.edges.iter().zip(&self.curves))
            .map(|(edge, curve)| EdgeSpec {
                curve: curve.clone().unwrap_or_else(|| edge.curve.clone()),
                start: edge.start,
                end: edge.end,
            })
            .collect();
        let faces = (spec.faces.iter().enumerate())
            .map(|(f, face)| {
                let lp = |coedges: &Vec<CoedgeSpec>| self.face_loop(f, coedges);
                let rebuilt_face = FaceSpec {
                    surface: self.surfaces[f].clone(),
                    outer: lp(&face.outer)?,
                    holes: face.holes.iter().map(lp).collect::<GeopResult<_>>()?,
                };
                // Moved so far that a wall's two sides cross, the face
                // between them runs into its own hole: a hole's pcurves
                // cross the outer loop's, or it has left it altogether.
                for hole in &rebuilt_face.holes {
                    if !hole_inside(&rebuilt_face.surface, &rebuilt_face.outer, hole)? {
                        return Err(GeopError::new(format!(
                            "face {} runs into its own hole: the sides of a wall would cross",
                            self.names.faces[f]
                        )));
                    }
                }
                Ok(rebuilt_face)
            })
            .collect::<GeopResult<_>>()?;
        Ok(BodySpec {
            vertices: self.points.clone(),
            edges,
            faces,
            shells: spec.shells.clone(),
            solid: spec.solid,
        })
    }

    /// A loop of face `f`, its pcurves on the face's new surface: each
    /// coedge that changed fitted anew between `(u, v)` worked out once
    /// per junction, so the coedges meeting there agree on it exactly.
    fn face_loop(&self, f: usize, coedges: &[CoedgeSpec]) -> GeopResult<Vec<CoedgeSpec>> {
        let surface = &self.surfaces[f];
        let replaced = self.replaced[f];
        // A coedge along a pole whose vertex no longer is the pole of the
        // new surface has nothing left to run along: the loop passes the
        // vertex inside the patch now, and the coedges either side of it
        // meet there.
        let at_pole = |c: &CoedgeSpec| -> GeopResult<bool> {
            let CoedgeOn::Vertex(v) = c.on else {
                return Ok(true);
            };
            let end = c.pcurve.ends()?.1;
            Ok(surface
                .pole_at(end[0], end[1])
                .is_some_and(|pole| pole.could_be_equal(&self.points[v])))
        };
        let mut kept = Vec::with_capacity(coedges.len());
        for c in coedges {
            if at_pole(c)? {
                kept.push(c.clone());
            }
        }
        let coedges = &kept[..];
        let mut junctions = Vec::with_capacity(coedges.len());
        for c in coedges {
            let (_, end) = self.spec.ends(c.on)?;
            let old = c.pcurve.ends()?.1;
            let kept = |old: Vector2| -> GeopResult<bool> {
                Ok(surface
                    .evaluate(old[0], old[1])?
                    .could_be_equal(&self.points[end]))
            };
            junctions.push(if (self.moved[end] || replaced) && !kept(old)? {
                let (u, v) = surface.project_in_domain(
                    self.points[end],
                    old[0].sharpen(),
                    old[1].sharpen(),
                    PROJECT_ITERATIONS,
                )?;
                Vector2::from_array([u, v])
            } else {
                old
            });
        }
        let n = coedges.len();
        let mut out = Vec::with_capacity(n);
        for (k, c) in coedges.iter().enumerate() {
            let changed = match c.on {
                CoedgeOn::Edge(e, _) => self.curves[e].is_some(),
                CoedgeOn::Vertex(v) => self.moved[v],
            };
            let pcurve = if !changed && !replaced {
                c.pcurve.clone()
            } else {
                let (start, end) = (junctions[(k + n - 1) % n], junctions[k]);
                let curve = match c.on {
                    CoedgeOn::Edge(e, sense) => Some(
                        sense.orient(self.curves[e].as_ref().unwrap_or(&self.spec.edges[e].curve)),
                    ),
                    CoedgeOn::Vertex(_) => None,
                };
                pcurve(surface, &c.pcurve, curve.as_ref(), start, end).map_err(|e| {
                    e.with_context(format!("the pcurve on face {}", self.names.faces[f]))
                })?
            };
            out.push(CoedgeSpec { on: c.on, pcurve });
        }
        Ok(out)
    }
}

/// The iso-line of `surface` that `along`, a pcurve on it, runs along —
/// straight, its ends at one sharp `u` (or `v`) — over the whole of the
/// surface's other parameter, running the way `along` does: the surface's
/// row there, exactly, where a straight pcurve is only as good as the
/// surface's parametrization is affine. None for any other pcurve.
fn iso_line(surface: &NurbSurface3D, along: &Curve2) -> GeopResult<Option<Curve3>> {
    if along.degree != 1 || along.control_points.len() != 2 {
        return Ok(None);
    }
    // Read off the control points: at a weight of one, as the numbers they
    // are, which a division would round outwards.
    let end = |i: usize| -> GeopResult<Vector2> {
        let p = along.control_points[i];
        if p[2].is_exactly(Scalar::ONE) {
            Ok(Vector2::from_array([p[0], p[1]]))
        } else {
            p.dehomogenized()
        }
    };
    let (a, b) = (end(0)?, end(1)?);
    let Some(k) = (0..2).find(|&k| a[k].is_exactly(b[k]) && a[k].is_sharp()) else {
        return Ok(None);
    };
    let u_fixed = k == 0;
    let at = a[k];
    let (lo, hi) = if u_fixed {
        surface.domain_u()
    } else {
        surface.domain_v()
    };
    let ends = at.is_exactly(lo) || at.is_exactly(hi);
    if !ends && !(at.definitely_greater(lo) && at.definitely_less(hi)) {
        return Ok(None);
    }
    let row = surface.iso_curve(u_fixed, at)?;
    Ok(if b[1 - k].definitely_greater(a[1 - k]) {
        Some(row)
    } else if b[1 - k].definitely_less(a[1 - k]) {
        Some(row.reverse())
    } else {
        None
    })
}

/// Fails if the points `points`, in order along an edge rebuilt from
/// `guide`, do not keep running the way the guide does: the edge has
/// turned around — the faces at its ends now cross — or shrunk to nothing.
fn turns_around(guide: &Curve3, points: &[Vector3]) -> GeopResult<()> {
    let (t0, t1) = guide.domain();
    let n = points.len() - 1;
    for (k, pair) in points.windows(2).enumerate() {
        let frac = Scalar::from_ratio(2 * k as i64 + 1, 2 * n as i64)?;
        let tangent = guide.tangent(Scalar::interpolate(t0, t1, frac).sharpen())?;
        if !pair[1]
            .sub(&pair[0])
            .prod_dot(&tangent)
            .definitely_greater(Scalar::ZERO)
        {
            return Err(GeopError::new(format!(
                "moved this far, it turns around between {:?} and {:?}: a face it bounds would vanish, or two would cross",
                pair[0], pair[1]
            )));
        }
    }
    Ok(())
}

/// `surface` with room around it: [`extended`] where it is straight, and
/// carried on past each side where it is not ([`NurbSurface3D::extended`])
/// by as much of its parameter as its end span is long that way — or by a
/// half, a quarter or an eighth of that where more would carry it too far:
/// an arc round past where its weights stay positive, a free-form patch
/// past where it folds back on itself and has no normal. Not past a side
/// that is a single point, a pole: the pole would end up inside the patch,
/// where it has no normal. How much room is a free choice; the patch's own
/// size is room enough unless a face moves as far as it is wide.
pub fn room(surface: NurbSurface3D) -> GeopResult<NurbSurface3D> {
    let mut surface = extended(surface)?;
    for along_u in [true, false] {
        let (degree, num) = if along_u {
            (surface.degree_u, surface.num_u)
        } else {
            (surface.degree_v, surface.num_v)
        };
        if degree == 1 && num == 2 {
            continue;
        }
        for at_end in [false, true] {
            if side_is_pole(&surface, along_u, at_end)? {
                continue;
            }
            // By as much as the end span is long: carried further, a short
            // end span of a curved patch extrapolates far, its control
            // points running off by the cube of how much further.
            let breaks = if along_u {
                surface.breakpoints_u()
            } else {
                surface.breakpoints_v()
            };
            let n = breaks.len();
            let mut by = if at_end {
                breaks[n - 1].sub(breaks[n - 2])
            } else {
                breaks[1].sub(breaks[0])
            };
            for _ in 0..4 {
                if let Ok(longer) = surface.extended(along_u, at_end, by.sharpen())
                    && longer.has_normal_everywhere()?
                {
                    surface = longer;
                    break;
                }
                by = by.div(Scalar::TWO)?;
            }
        }
    }
    Ok(surface)
}

/// Whether the side of `surface` at the end (`at_end`) or the start of its
/// `u` (`along_u`) or `v` parameter is a single point: every control point
/// of that boundary row the same.
fn side_is_pole(surface: &NurbSurface3D, along_u: bool, at_end: bool) -> GeopResult<bool> {
    let (nu, nv) = (surface.num_u, surface.num_v);
    let row: Vec<usize> = if along_u {
        let i = if at_end { nu - 1 } else { 0 };
        (0..nv).map(|j| i * nv + j).collect()
    } else {
        let j = if at_end { nv - 1 } else { 0 };
        (0..nu).map(|i| i * nv + j).collect()
    };
    let first = surface.control_points[row[0]].dehomogenized()?;
    for &k in &row[1..] {
        if !surface.control_points[k]
            .dehomogenized()?
            .could_be_equal(&first)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The point nearest `seed` where the smooth surfaces `groups` of `pieces`
/// all meet (see [`meet`]). Where it was is only where to start looking —
/// a free choice, so sharp: a seed as wide as the union of points joined
/// across a gap would carry its width into the answer.
fn nearest(
    seed: Vector3,
    pieces: &[(&NurbSurface3D, (Scalar, Scalar))],
    groups: &[Vec<usize>],
) -> GeopResult<Vector3> {
    let mut offsets: Vec<Offset> = groups
        .iter()
        .map(|group| Offset {
            pieces: group.iter().map(|&i| pieces[i]).collect(),
            offset: Scalar::ZERO,
        })
        .collect();
    meet(seed.sharpen(), &mut offsets, &[])
}

/// The part of `curve`, carried on past its ends or cut back, from `start`
/// to `end`, both on it: what is left of an edge whose ends move along it.
pub(crate) fn along(curve: &Curve3, start: Vector3, end: Vector3) -> GeopResult<Curve3> {
    restricted(&carried_on(curve, |_| Ok(true))?, start, end)
}

/// The part of `longer`, a seam carried on past its ends, from `start` to
/// `end`, both on it.
fn restricted(longer: &Curve3, start: Vector3, end: Vector3) -> GeopResult<Curve3> {
    let at = |p: &Vector3| -> GeopResult<Scalar> {
        let (t, _) = longer.closest_point(p)?;
        if !longer.evaluate(t)?.could_be_equal(p) {
            return Err(GeopError::new(format!(
                "the seam, carried on, does not pass through its moved end {p:?}"
            )));
        }
        Ok(t)
    };
    let (s, t) = (at(&start)?, at(&end)?);
    if !t.definitely_greater(s) {
        return Err(GeopError::new(format!(
            "its moved ends {start:?} and {end:?} lie the other way round along it: a face it bounds has vanished"
        )));
    }
    longer.sub_curve(s, t)
}

/// A cubic from `start` to `end` through the points `at(frac)` of a curve
/// in between — `frac` the fraction of the way along it — widened to
/// enclose the curve between them: fitted over [`FIRST_LEGS`] legs, split
/// finer while the fit is wider than [`TRACE_DRIFT`]. `check` is asked of
/// each set of points before they are fitted.
fn fitted(
    at: impl Fn(Scalar) -> GeopResult<Vector3>,
    (start, end): (Vector3, Vector3),
    check: impl Fn(&[Vector3]) -> GeopResult<()>,
) -> GeopResult<Curve3> {
    let fraction = |i: usize, n: usize| Scalar::from_ratio(i as i64, n as i64);
    let refined = refine(FIRST_LEGS, usize::BITS as usize, |legs| -> GeopResult<_> {
        let mut points = vec![start];
        for i in 1..legs {
            points.push(at(fraction(i, legs)?)?);
        }
        points.push(end);
        check(&points)?;
        let mut between = Vec::with_capacity(legs);
        for i in 0..legs {
            let mut inside = Vec::new();
            for &(p, q) in true_point_fractions(i, legs) {
                let frac = fraction(i, legs)?.add(Scalar::from_ratio(p, q * legs as i64)?);
                inside.push(at(frac)?);
            }
            between.push(inside);
        }
        let curve = NurbCurve3D::interpolate_enclosing(&points, &between, 3)?;
        let wide = curve
            .control_points
            .iter()
            .any(|p| (0..3).any(|k| p[k].width().to_f64() > TRACE_DRIFT));
        if wide && 2 * legs <= MOST_LEGS {
            return Ok(Refine::Finer(2 * legs));
        }
        Ok(Refine::Done(curve))
    })?;
    match refined {
        Refined::Done(curve) => Ok(curve),
        Refined::Exhausted(legs) => Err(GeopError::new(format!(
            "its legs were split past every cap, at {legs}"
        ))),
    }
}

/// `curve` carried on past both its ends by as much as its domain is long,
/// or by less where that would carry it too far — an arc round past where
/// its weights stay positive, or past what `within` accepts — as far as a
/// half, a quarter or an eighth of that: room for an end to move along it.
/// How much is a free choice.
fn carried_on<const D: usize>(
    curve: &NurbCurve<D>,
    within: impl Fn(&NurbCurve<D>) -> GeopResult<bool>,
) -> GeopResult<NurbCurve<D>> {
    let (t0, t1) = curve.domain();
    let width = t1.sub(t0);
    let mut out = curve.clone();
    for at_end in [false, true] {
        let mut by = width;
        for _ in 0..4 {
            if let Ok(longer) = out.extended(at_end, by.sharpen())
                && within(&longer)?
            {
                out = longer;
                break;
            }
            by = by.div(Scalar::TWO)?;
        }
    }
    Ok(out)
}

/// Whether the loop `hole` lies inside the loop `outer` on `surface`: no
/// pcurve of the one meets one of the other, and a point of the hole is
/// inside the outer loop.
fn hole_inside(
    surface: &NurbSurface3D,
    outer: &[CoedgeSpec],
    hole: &[CoedgeSpec],
) -> GeopResult<bool> {
    for h in hole {
        for o in outer {
            match curve_curve_intersect(&h.pcurve, &o.pcurve, 1, MAX_NODES, min_subdivision_size())?
            {
                Intersections::Found(found) if found.is_empty() => {}
                _ => return Ok(false),
            }
        }
    }
    let first = &hole[0].pcurve;
    let point = first.evaluate(first.mid_parameter()?)?;
    let pcurves: Vec<_> = outer.iter().map(|c| &c.pcurve).collect();
    Ok(pcurves_contain(
        surface,
        &pcurves,
        point,
        MAX_NODES,
        min_subdivision_size(),
        SEED,
    )? == PointClassification::Inside)
}

#[cfg(test)]
mod tests;
