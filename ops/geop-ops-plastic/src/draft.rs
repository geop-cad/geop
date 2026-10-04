//! [`draft`]: planar faces of a solid tilted about a neutral plane, so the
//! part comes out of its mould.
//!
//! **Why the faces are replaced, not cut.** A draft only changes the
//! planes some faces lie in; the solid keeps its topology — every face,
//! edge and vertex is still there, still between the same neighbours. So
//! the drafted solid is built directly, the way a shell builds its walls'
//! inner side ([`geop_ops_shell::shell`]): each drafted face gets its
//! tilted plane, each vertex of one moves to where the surfaces around it
//! now meet, each edge to where its two faces do. The alternative, a
//! boolean with a wedge per face, would have to put the wedge's tilted face
//! exactly through the face's edge on the neutral plane — a cut along an
//! edge that already exists, the one configuration a boolean cannot stay
//! tight through — and a wedge beside an inward corner would cut into the
//! neighbour. Replacing the geometry needs neither, keeps every name, and
//! re-trims the neighbours exactly: a plane's edges are lines between the
//! moved vertices, a cylinder's are where the tilted plane cuts it.
//!
//! **The tilt.** A face with outward normal `n` turns about the line where
//! its plane meets the neutral plane, by the draft angle, so that its normal
//! leans towards the pull direction `p`, the neutral plane's normal: past
//! the neutral plane, along `p`, the solid gets narrower, and it comes out
//! of a mould pulled that way. A negative angle leans the faces the other
//! way.
//!
//! What it does not do (yet): draft faces that are not planar, faces
//! parallel to the neutral plane (they have no line to turn about), or
//! faces next to anything but planes and cylinders — a fillet, a sphere, a
//! free-form face; nor faces that meet a neighbour tangentially (a fillet
//! along an edge). An angle so large that an edge turns around is refused.

use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_surface::NurbSurface3D,
    shape::{Arc, Axis, Circle, Plane},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    polygon::loops_contain,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    Body, Curve3, FaceId, SolidId,
    build::{BodySpec, CoedgeOn, CoedgeSpec, EdgeSpec, FaceSpec},
};
use geop_ops::{BodyNames, Namer, Part};
use geop_ops_extrude_revolve::common::line3;
use geop_ops_shell::shell::{extended, offset_vertex, pcurve};

use crate::common::{
    MAX_NODES, PROJECT_ITERATIONS, ends, halfway, loops_of, min_subdivision_size, normal_at,
    oriented, pcurve_ends,
};

/// The surface a face lies on, as far as a draft cares.
#[derive(Clone, Debug)]
enum Kind<S: Scalar> {
    Plane(Plane<S>),
    Cylinder(Axis<S>),
    Other,
}

impl<S: Scalar> Kind<S> {
    fn of(surface: &NurbSurface3D<S>) -> GeopResult<Self> {
        if let Some(plane) = surface.as_plane()? {
            return Ok(Kind::Plane(plane));
        }
        if let Some(cylinder) = surface.as_cylinder()? {
            return Ok(Kind::Cylinder(cylinder.axis));
        }
        Ok(Kind::Other)
    }
}

/// Tilts the planar faces `faces` of one solid by `angle` (in radians, a
/// quarter turn either way at most) about where each meets the plane
/// `neutral`, whose normal is the pull direction — see the module docs —
/// and returns the drafted solid, which replaces it and is named
/// `namer`'s root. Every face, edge and vertex keeps its name.
pub fn draft<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    faces: &[FaceId],
    neutral: &Plane<S>,
    angle: S,
) -> GeopResult<SolidId> {
    let ctx = with_context!(
        "draft({}, faces={faces:?}, neutral={neutral:?}, angle={angle:?})",
        namer.root()
    );
    let quarter = S::from_f64(std::f64::consts::FRAC_PI_2);
    if !(angle.abs().definitely_greater(S::ZERO) && angle.abs().definitely_less(quarter)) {
        return Err(GeopError::new(format!(
            "a draft angle is more than zero and less than a quarter turn, not {:?} degrees",
            angle.to_f64().to_degrees()
        )))
        .with_context(ctx);
    }
    let model = part.topology();
    let Some(&first) = faces.first() else {
        return Err(GeopError::new("pick the faces to draft")).with_context(ctx);
    };
    let name = |id: geop_ops::RefId| -> GeopResult<String> {
        part.name_of(id)
            .map(str::to_string)
            .ok_or_else(|| GeopError::new(format!("{id:?} has no name")))
    };
    let Body::Solid(solid) = model.body_of_face(first).with_context(ctx)? else {
        return Err(GeopError::new(format!(
            "face {} stands on its own; only faces of a solid can be drafted",
            name(first.into())?
        )))
        .with_context(ctx);
    };
    let all = model.solid_faces(solid).with_context(ctx)?;
    if let Some(&f) = faces.iter().find(|f| !all.contains(f)) {
        return Err(GeopError::new(format!(
            "face {} is not of the same solid as face {}: draft the faces of one solid at a time",
            name(f.into())?,
            name(first.into())?
        )))
        .with_context(ctx);
    }
    let (spec, sources) = model.body_spec(&all, true).with_context(ctx)?;
    let names = BodyNames {
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
        solid: Some(namer.root()),
    };
    let drafted: Vec<bool> = sources.faces.iter().map(|f| faces.contains(f)).collect();
    let tilt = Tilt {
        neutral,
        cos: angle.cos(),
        sin: angle.sin(),
    };
    let result = Drafted::new(&spec, &names, &drafted, &tilt)
        .with_context(ctx)?
        .result()
        .with_context(ctx)?;
    part.assemble_solid(&[solid.into()], &[], "")
        .with_context(ctx)?;
    let built = part.build_body(result, names).with_context(ctx)?;
    built
        .solid
        .ok_or_else(|| GeopError::new("draft: building the result made no solid"))
}

/// How a drafted face turns: about where its plane meets `neutral`, by the
/// angle whose cosine and sine these are.
struct Tilt<'a, S: Scalar> {
    neutral: &'a Plane<S>,
    cos: S,
    sin: S,
}

impl<S: Scalar> Tilt<'_, S> {
    /// The line the face in `plane` turns about, along `n x p` — turning
    /// about it by a positive angle leans `n` towards `p`.
    fn hinge(&self, plane: &Plane<S>) -> GeopResult<Axis<S>> {
        plane.intersect_plane(self.neutral)
    }

    /// `p` turned about `hinge`, homogeneous: `(w x, w)` to `(w R(x - c) +
    /// w c, w)`, which needs no division by the weight.
    fn turn(&self, hinge: &Axis<S>, p: &Vector4<S>) -> Vector4<S> {
        let w = p[3];
        let c = hinge.point.prod_scalar(w);
        let v = Vector3::from_array([p[0], p[1], p[2]]).sub(&c);
        let turned = self.rotate(&hinge.direction, &v).add(&c);
        Vector4::from_array([turned[0], turned[1], turned[2], w])
    }

    /// `v` turned about the unit vector `k` (Rodrigues).
    fn rotate(&self, k: &Vector3<S>, v: &Vector3<S>) -> Vector3<S> {
        let along = k.prod_scalar(k.prod_dot(v).mul(S::ONE.sub(self.cos)));
        v.prod_scalar(self.cos)
            .add(&k.prod_cross(v).prod_scalar(self.sin))
            .add(&along)
    }
}

/// The solid described as a [`BodySpec`], and where drafting moves what.
struct Drafted<'a, S: Scalar> {
    spec: &'a BodySpec<S>,
    names: &'a BodyNames,
    drafted: &'a [bool],
    /// Per face: the surface it lies on afterwards, with room around it
    /// wherever its boundary may move.
    surfaces: Vec<NurbSurface3D<S>>,
    /// Per face: what that surface is.
    kinds: Vec<Kind<S>>,
    /// Per vertex: whether a drafted face meets it, and where it goes.
    moved: Vec<bool>,
    points: Vec<Vector3<S>>,
    /// Per edge: its new curve, if it changes.
    curves: Vec<Option<Curve3<S>>>,
}

impl<'a, S: Scalar> Drafted<'a, S> {
    fn new(
        spec: &'a BodySpec<S>,
        names: &'a BodyNames,
        drafted: &'a [bool],
        tilt: &Tilt<S>,
    ) -> GeopResult<Self> {
        let n_faces = spec.faces.len();
        // Per vertex, each face around it once, with where the vertex is on
        // the face's surface; per edge, the faces along it.
        let mut around: Vec<Vec<(usize, Vector2<S>)>> = vec![Vec::new(); spec.vertices.len()];
        let mut along: Vec<Vec<usize>> = vec![Vec::new(); spec.edges.len()];
        for (f, face) in spec.faces.iter().enumerate() {
            for lp in loops_of(face) {
                for c in lp {
                    let (_, end) = ends(spec, c.on);
                    if !around[end].iter().any(|&(g, _)| g == f) {
                        around[end].push((f, pcurve_ends(&c.pcurve)?.1));
                    }
                    match c.on {
                        CoedgeOn::Edge(e, _) => along[e].push(f),
                        CoedgeOn::Vertex(v) if drafted[f] => {
                            return Err(GeopError::new(format!(
                                "face {} comes to a point at vertex {}: only planar faces can be drafted",
                                names.faces[f], names.vertices[v]
                            )));
                        }
                        CoedgeOn::Vertex(_) => {}
                    }
                }
            }
        }
        let moved: Vec<bool> = around
            .iter()
            .map(|faces| faces.iter().any(|&(f, _)| drafted[f]))
            .collect();
        let touched: Vec<bool> = (0..n_faces)
            .map(|f| {
                drafted[f]
                    || around
                        .iter()
                        .enumerate()
                        .any(|(v, faces)| moved[v] && faces.iter().any(|&(g, _)| g == f))
            })
            .collect();

        let mut kinds = Vec::with_capacity(n_faces);
        for (f, face) in spec.faces.iter().enumerate() {
            kinds.push(Kind::of(&face.surface)?);
            if drafted[f] && !matches!(kinds[f], Kind::Plane(_)) {
                return Err(GeopError::new(format!(
                    "face {} is not planar: only planar faces can be drafted",
                    names.faces[f]
                )));
            }
        }
        // What a drafted face's neighbours may be, and how they meet it.
        for (v, faces) in around.iter().enumerate() {
            let Some(&(d, _)) = faces.iter().find(|&&(f, _)| drafted[f]) else {
                continue;
            };
            if let Some(&(f, _)) = faces
                .iter()
                .find(|&&(f, _)| matches!(kinds[f], Kind::Other))
            {
                return Err(GeopError::new(format!(
                    "face {} meets the drafted face {} at vertex {}, and is neither planar nor cylindrical: drafting next to it is not supported",
                    names.faces[f], names.faces[d], names.vertices[v]
                )));
            }
        }
        for (e, faces) in along.iter().enumerate() {
            let [f, g] = faces.as_slice() else {
                continue;
            };
            let (d, other) = match (drafted[*f], drafted[*g]) {
                (true, _) => (*f, *g),
                (_, true) => (*g, *f),
                _ => continue,
            };
            if tangent_along(spec, e, d, other)? {
                return Err(GeopError::new(format!(
                    "the drafted face {} meets face {} tangentially along edge {} — a fillet or a smooth blend: drafting next to one is not supported",
                    names.faces[d], names.faces[other], names.edges[e]
                )));
            }
        }

        let mut surfaces = Vec::with_capacity(n_faces);
        for (f, face) in spec.faces.iter().enumerate() {
            let surface = if drafted[f] {
                let Kind::Plane(plane) = &kinds[f] else {
                    unreachable!("checked above")
                };
                let hinge = tilt.hinge(plane).map_err(|e| {
                    e.with_context(format!(
                        "face {} is parallel to the neutral plane: it has no line to be drafted about",
                        names.faces[f]
                    ))
                })?;
                let mut turned = face.surface.clone();
                for cp in &mut turned.control_points {
                    *cp = tilt.turn(&hinge, cp);
                }
                turned.recompute_aabb();
                kinds[f] = Kind::Plane(Plane {
                    point: hinge.point,
                    normal: tilt.rotate(&hinge.direction, &plane.normal),
                });
                extended(turned)?
            } else if touched[f] {
                extended(face.surface.clone())?
            } else {
                face.surface.clone()
            };
            surfaces.push(surface);
        }

        let size = spec
            .vertices
            .iter()
            .map(|p| p.sub(&spec.vertices[0]).norm())
            .fold(S::ONE, |a, b| a.max(b));
        let mut points = spec.vertices.clone();
        for (v, faces) in around.iter().enumerate() {
            if !moved[v] {
                continue;
            }
            points[v] = move_vertex(spec.vertices[v], faces, &surfaces, &kinds, size)
                .map_err(|e| e.with_context(format!("moving vertex {}", names.vertices[v])))?;
        }

        let mut drafted_solid = Self {
            spec,
            names,
            drafted,
            surfaces,
            kinds,
            moved,
            points,
            curves: Vec::new(),
        };
        drafted_solid.curves = (0..spec.edges.len())
            .map(|e| {
                drafted_solid
                    .curve(e, &along[e])
                    .map_err(|err| err.with_context(format!("edge {}", names.edges[e])))
            })
            .collect::<GeopResult<_>>()?;
        Ok(drafted_solid)
    }

    /// The new curve of edge `e`, between the faces `faces`, if it changes:
    /// where those faces' surfaces now meet, between its moved vertices.
    fn curve(&self, e: usize, faces: &[usize]) -> GeopResult<Option<Curve3<S>>> {
        let edge = &self.spec.edges[e];
        let changed = self.moved[edge.start]
            || self.moved[edge.end]
            || faces.iter().any(|&f| self.drafted[f]);
        if !changed {
            return Ok(None);
        }
        let (start, end) = (self.points[edge.start], self.points[edge.end]);
        let closed = edge.start == edge.end;
        let [f, g] = faces else {
            return Err(GeopError::new(format!(
                "it bounds {} faces, not two",
                faces.len()
            )));
        };
        let unsupported = || {
            GeopError::new(format!(
                "it runs between faces {} and {}, which drafting cannot re-intersect",
                self.names.faces[*f], self.names.faces[*g]
            ))
        };
        let curve = match (&self.kinds[*f], &self.kinds[*g]) {
            (Kind::Plane(_), Kind::Plane(_)) if !closed => line3(start, end)?,
            (Kind::Plane(plane), Kind::Cylinder(axis))
            | (Kind::Cylinder(axis), Kind::Plane(plane)) => {
                if plane
                    .normal
                    .prod_dot(&axis.direction)
                    .could_be_equal(S::ZERO)
                {
                    // The plane holds the axis' direction: it meets the
                    // cylinder along a ruling.
                    if closed {
                        return Err(unsupported());
                    }
                    line3(start, end)?
                } else {
                    section(axis, plane, &edge.curve, start, end, closed)?
                }
            }
            (Kind::Cylinder(a), Kind::Cylinder(b))
                if !closed && a.could_be_parallel(b) && edge.curve.as_line()?.is_some() =>
            {
                line3(start, end)?
            }
            _ => return Err(unsupported()),
        };
        // A straight edge turned around: the faces now cross.
        if let Some(axis) = edge.curve.as_line()?
            && !end
                .sub(&start)
                .prod_dot(&axis.direction)
                .definitely_greater(S::ZERO)
        {
            return Err(GeopError::new(format!(
                "the draft is too steep: the edge turns around (from {start:?} to {end:?})"
            )));
        }
        // It is only right if it lies on both surfaces: say so if not,
        // rather than build an edge off its faces.
        let (t0, t1) = curve.domain();
        let middle = curve.evaluate(t0.add(t1).div(S::TWO)?)?;
        for &face in faces {
            let on = surface_could_contain(
                &self.surfaces[face],
                &middle,
                MAX_NODES,
                min_subdivision_size(),
            )?;
            if on.is_none() {
                return Err(GeopError::new(format!(
                    "its new curve from {start:?} to {end:?} is not on face {} at {middle:?}",
                    self.names.faces[face]
                )));
            }
        }
        Ok(Some(curve))
    }

    /// The drafted solid, described whole.
    fn result(&self) -> GeopResult<BodySpec<S>> {
        let spec = self.spec;
        let edges = spec
            .edges
            .iter()
            .zip(&self.curves)
            .map(|(edge, curve)| EdgeSpec {
                curve: curve.clone().unwrap_or_else(|| edge.curve.clone()),
                start: edge.start,
                end: edge.end,
            })
            .collect();
        let faces = spec
            .faces
            .iter()
            .enumerate()
            .map(|(f, face)| {
                let lp = |coedges: &Vec<CoedgeSpec<S>>| self.face_loop(f, coedges);
                let drafted_face = FaceSpec {
                    surface: self.surfaces[f].clone(),
                    outer: lp(&face.outer)?,
                    holes: face.holes.iter().map(lp).collect::<GeopResult<_>>()?,
                };
                // Drafted so far that a wall's two sides cross, the face
                // between them — a shelled rim — runs into its own hole.
                let outline = |lp: &[CoedgeSpec<S>]| -> GeopResult<Vec<Vector2<S>>> {
                    lp.iter().map(|c| Ok(pcurve_ends(&c.pcurve)?.0)).collect()
                };
                let outer = outline(&drafted_face.outer)?;
                for hole in &drafted_face.holes {
                    if !outline(hole)?.iter().all(|p| loops_contain(std::slice::from_ref(&outer), p)) {
                        return Err(GeopError::new(format!(
                            "the draft is too steep: face {} runs into its own hole — the sides of a wall would cross",
                            self.names.faces[f]
                        )));
                    }
                }
                Ok(drafted_face)
            })
            .collect::<GeopResult<_>>()?;
        Ok(BodySpec {
            vertices: self.points.clone(),
            edges,
            faces,
            shells: spec.shells.clone(),
            solid: true,
        })
    }

    /// A loop of face `f`, its pcurves on the face's new surface: each
    /// coedge that changed fitted anew between `(u, v)` worked out once
    /// per junction, so the coedges meeting there agree on it exactly.
    fn face_loop(&self, f: usize, coedges: &[CoedgeSpec<S>]) -> GeopResult<Vec<CoedgeSpec<S>>> {
        let surface = &self.surfaces[f];
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        // Where each coedge ends.
        let mut junctions = Vec::with_capacity(coedges.len());
        for c in coedges {
            let (_, end) = ends(self.spec, c.on);
            let old = pcurve_ends(&c.pcurve)?.1;
            junctions.push(if self.moved[end] || self.drafted[f] {
                let (u, v) = surface.project(
                    self.points[end],
                    old[0].sharpen(),
                    old[1].sharpen(),
                    PROJECT_ITERATIONS,
                )?;
                // On the patch, as the vertex is.
                Vector2::from_array([u.intersect(u0.union(u1)), v.intersect(v0.union(v1))])
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
            let pcurve = if !changed && !self.drafted[f] {
                c.pcurve.clone()
            } else {
                let (start, end) = (junctions[(k + n - 1) % n], junctions[k]);
                let curve = match c.on {
                    CoedgeOn::Edge(e, sense) => Some(oriented(
                        self.curves[e].as_ref().unwrap_or(&self.spec.edges[e].curve),
                        sense,
                    )),
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

/// Whether the faces `f` and `g` could meet tangentially along edge `e`:
/// their normals, halfway along it, could be parallel.
fn tangent_along<S: Scalar>(spec: &BodySpec<S>, e: usize, f: usize, g: usize) -> GeopResult<bool> {
    let (point, _) = halfway(&spec.edges[e].curve)?;
    let n_f = normal_at(&spec.faces[f].surface, &point)?;
    let n_g = normal_at(&spec.faces[g].surface, &point)?;
    Ok(n_f.prod_cross(&n_g).norm_sq().could_be_equal(S::ZERO))
}

/// Where a vertex at `point` goes: where the surfaces of the faces around
/// it, `faces` (each with where the vertex was on it), now meet — nearest
/// to where it was (see [`offset_vertex`]).
///
/// Where only two smooth surfaces meet — a drafted face and a cylinder,
/// at the seam of a hole through it — the vertex is free to slide along
/// their intersection. It is the end of the cylinder's seam or ruling
/// there, so it stays on that: the plane through the cylinder's axis and
/// the vertex is the third condition, `size` how far that plane's patch
/// reaches, which only has to be past where the vertex can go.
fn move_vertex<S: Scalar>(
    point: Vector3<S>,
    faces: &[(usize, Vector2<S>)],
    surfaces: &[NurbSurface3D<S>],
    kinds: &[Kind<S>],
    size: S,
) -> GeopResult<Vector3<S>> {
    let mut normals: Vec<Vector3<S>> = Vec::new();
    for &(f, at) in faces {
        let normal = surfaces[f].normal(at[0], at[1])?;
        if !normals
            .iter()
            .any(|n| n.prod_cross(&normal).norm_sq().could_be_equal(S::ZERO))
        {
            normals.push(normal);
        }
    }
    let mut extra: Option<NurbSurface3D<S>> = None;
    if normals.len() < 3 {
        let axis = faces
            .iter()
            .find_map(|&(f, _)| match &kinds[f] {
                Kind::Cylinder(axis) => Some(axis.clone()),
                _ => None,
            })
            .ok_or_else(|| {
                GeopError::new(format!(
                    "only {} surface(s) meet there and none is a cylinder: where it goes is not determined",
                    normals.len()
                ))
            })?;
        let radial = point.sub(&axis.project(&point));
        extra = Some(plane_patch(
            point,
            axis.direction.prod_scalar(size),
            radial.normalize()?.prod_scalar(size),
        )?);
    }
    let mut constraints: Vec<(&NurbSurface3D<S>, Vector2<S>, bool)> = faces
        .iter()
        .map(|&(f, at)| (&surfaces[f], at, false))
        .collect();
    let half = S::ONE.div(S::TWO)?;
    if let Some(plane) = &extra {
        constraints.push((plane, Vector2::from_array([half, half]), false));
    }
    offset_vertex(point, &constraints)
}

/// The patch of the plane through `center` spanned by `e1` and `e2`, from
/// `center - e1 - e2` to `center + e1 + e2`.
fn plane_patch<S: Scalar>(
    center: Vector3<S>,
    e1: Vector3<S>,
    e2: Vector3<S>,
) -> GeopResult<NurbSurface3D<S>> {
    let corner = |a: S, b: S| {
        let p = center.add(&e1.prod_scalar(a)).add(&e2.prod_scalar(b));
        Vector4::from_array([p[0], p[1], p[2], S::ONE])
    };
    let (lo, hi) = (S::ONE.neg(), S::ONE);
    let knots = vec![S::ZERO, S::ZERO, S::ONE, S::ONE];
    NurbSurface3D::try_new(
        1,
        1,
        vec![
            corner(lo, lo),
            corner(lo, hi),
            corner(hi, lo),
            corner(hi, hi),
        ],
        knots.clone(),
        knots,
    )
}

/// Where `plane` cuts the cylinder around `axis`, from `start` to `end` —
/// all the way round, if `closed` — turning the way `old`, another curve on
/// the cylinder cut by a plane, did.
///
/// Seen along the axis, any plane section of the cylinder is an arc of its
/// cross-section circle; and the section is that arc lifted along the axis
/// onto the plane — an affine map, so the rational quadratic arc lifts to
/// an exact NURBS of the section, an ellipse for a plane at a slant. Which
/// way round the arc runs is what `old`, seen along the axis, says.
fn section<S: Scalar>(
    axis: &Axis<S>,
    plane: &Plane<S>,
    old: &Curve3<S>,
    start: Vector3<S>,
    end: Vector3<S>,
    closed: bool,
) -> GeopResult<Curve3<S>> {
    let a = axis.direction;
    let c = axis.point;
    // Along the axis onto the cross-section through `c`: `x - a (a . (x -
    // c))`, homogeneous.
    let flatten_h = |p: &Vector4<S>| {
        let x = Vector3::from_array([p[0], p[1], p[2]]);
        let off = a.prod_dot(&x).sub(a.prod_dot(&c).mul(p[3]));
        let y = x.sub(&a.prod_scalar(off));
        Vector4::from_array([y[0], y[1], y[2], p[3]])
    };
    let flatten = |x: &Vector3<S>| x.sub(&a.prod_scalar(a.prod_dot(&x.sub(&c))));
    let mut seen = old.clone();
    for cp in &mut seen.control_points {
        *cp = flatten_h(cp);
    }
    seen.recompute_aabb();
    let turning = seen.as_arc()?.ok_or_else(|| {
        GeopError::new("seen along the cylinder's axis, the edge is no arc: it is no plane section")
    })?;
    let (from, to) = (flatten(&start), flatten(&end));
    let arc = Arc {
        circle: Circle {
            center: c,
            normal: turning.circle.normal,
            radius: from.sub(&c).norm(),
        },
        start: from,
        end: if closed { from } else { to },
    }
    .to_curve()?;
    // Along the axis onto the plane: `x + a (d - n . x) / (n . a)`.
    let n = plane.normal;
    let na = n.prod_dot(&a);
    let d = n.prod_dot(&plane.point);
    let mut lifted = arc;
    for cp in &mut lifted.control_points {
        let x = Vector3::from_array([cp[0], cp[1], cp[2]]);
        let rise = d.mul(cp[3]).sub(n.prod_dot(&x)).div(na)?;
        let y = x.add(&a.prod_scalar(rise));
        *cp = Vector4::from_array([y[0], y[1], y[2], cp[3]]);
    }
    lifted.recompute_aabb();
    Ok(lifted)
}
