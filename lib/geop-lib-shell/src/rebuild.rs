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
            face.surface = surface(&names.faces[f], &face.surface).with_context(ctx)?;
        }
    }
    let reseat = vec![false; spec.vertices.len()];
    let result = rebuilt(&spec, &names, &replaced, &reseat).with_context(ctx)?;
    part.replace_with_solid(&[solid.into()], result, names)
        .with_context(ctx)
}

/// The body `spec` rebuilt: the faces marked `replaced` lie on the
/// surfaces `spec` gives them now, every vertex they reach — and every
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
fn smooth_along(spec: &BodySpec, edge: &EdgeSpec, f: usize, g: usize) -> GeopResult<bool> {
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
        let surfaces = (spec.faces.iter().zip(&touched))
            .map(|(face, &touched)| {
                if touched {
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
                let (u, w) = surface.project_in_domain(
                    seed,
                    uv[0].sharpen(),
                    uv[1].sharpen(),
                    PROJECT_ITERATIONS,
                )?;
                (u.sharpen(), w.sharpen())
            } else {
                (uv[0], uv[1])
            };
            pieces.push((surface, (u, w)));
        }
        // A vertex at a pole of its faces — on the axis of a revolved face —
        // is the pole of their new surfaces: the topology puts it there.
        let poles: Vec<_> = (pieces.iter())
            .map(|(s, (u, w))| s.pole_at(*u, *w))
            .collect();
        if let Some(pole) = poles.iter().flatten().next() {
            if !poles
                .iter()
                .all(|p| p.is_some_and(|p| p.could_be_equal(pole)))
            {
                let names: Vec<&str> = (faces.iter().zip(&poles))
                    .filter(|(_, p)| p.is_none())
                    .map(|(&(f, _), _)| self.names.faces[f].as_str())
                    .collect();
                return Err(GeopError::new(format!(
                    "it is at a pole of some of its faces and not of {names:?}, or of poles apart ({poles:?}): moving it there is not supported"
                )));
            }
            return Ok(*pole);
        }
        let groups = tangent_groups(&pieces)?;
        match groups.len() {
            0 | 1 => self.within(faces),
            2 => self.on_seam(v, faces, &pieces, &groups),
            _ => {
                let mut offsets: Vec<Offset> = groups
                    .iter()
                    .map(|group| Offset {
                        pieces: group.iter().map(|&i| pieces[i]).collect(),
                        offset: Scalar::ZERO,
                    })
                    .collect();
                meet(seed, &mut offsets, &[])
            }
        }
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
                let on_curve = curve.evaluate(t)?;
                let on_surface = surface.evaluate(uv[0], uv[1])?;
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

    /// The new curve of edge `e`, used by the faces `uses`, if it changes.
    fn curve(&self, e: usize, uses: &[(usize, Sense, &Curve2)]) -> GeopResult<Option<Curve3>> {
        let edge = &self.spec.edges[e];
        let changed = self.moved[edge.start]
            || self.moved[edge.end]
            || uses.iter().any(|&(f, ..)| self.replaced[f]);
        if !changed {
            return Ok(None);
        }
        let [(f, f_sense, f_pcurve), (g, g_sense, g_pcurve)] = uses else {
            return Err(GeopError::new(format!(
                "it bounds {} faces, not two: only an edge between two faces can be moved",
                uses.len()
            )));
        };
        let (start, end) = (self.points[edge.start], self.points[edge.end]);
        let faces = [*f, *g];
        let curve = if let Some(seam) = &self.seams[e] {
            restricted(seam, start, end)?
        } else {
            let straight = (edge.curve.degree == 1 && edge.curve.control_points.len() == 2)
                .then(|| NurbCurve3D::line(start, end))
                .transpose()?
                .filter(|line| {
                    matches!(surface_missed(line, faces, &self.surfaces), Ok(None))
                        && turns_around(&edge.curve, &[start, end]).is_ok()
                });
            match straight {
                Some(line) => line,
                None => self.traced(
                    &edge.curve,
                    start,
                    end,
                    [(*f, f_sense, *f_pcurve), (*g, g_sense, *g_pcurve)],
                )?,
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
/// by as much of its parameter as its domain is long that way — or by a
/// half, a quarter or an eighth of that where more would carry it too far,
/// an arc round past where its weights stay positive. Not past a side
/// that is a single point, a pole: the pole would end up inside the patch,
/// where it has no normal. How much room is a free choice; the patch's own
/// size is room enough unless a face moves as far as it is wide.
fn room(surface: NurbSurface3D) -> GeopResult<NurbSurface3D> {
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
            let (lo, hi) = if along_u {
                surface.domain_u()
            } else {
                surface.domain_v()
            };
            let mut by = hi.sub(lo);
            for _ in 0..4 {
                if let Ok(longer) = surface.extended(along_u, at_end, by.sharpen()) {
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
