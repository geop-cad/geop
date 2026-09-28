//! Ray-based hit testing ("picking") against a [`Model`], for interactive
//! selection in a viewer: cast a ray from the camera through the cursor,
//! ask for the nearest vertex/edge/face/solid it hits, and get back its
//! id — which a caller turns into the entity's stable name to put into a
//! program step.
//!
//! Built entirely on a [`RasterizedModel`] the caller passes in — the same
//! sampled points/polylines/triangles its viewer draws — rather than
//! re-deriving triangulation or curve sampling here, so a pick can never
//! disagree with what the viewer actually shows. Sketches likewise, through
//! [`SketchTargets`]. Rasterizing once per build and picking against that
//! also makes a pick cheap enough to run on every pointer move, to show
//! what a click would pick.
//!
//! Generic over the model's own [`Scalar`], using the crate's existing
//! [`Vector3`] linear algebra (`add`/`sub`/`prod_dot`/`prod_cross`/`norm`)
//! throughout rather than a second, ad hoc vector-math implementation.
//! `tolerance` (how close a click has to land to count, for
//! [`PickFilter::Vertex`]/[`PickFilter::Edge`]) is the one deliberate
//! exception, staying a plain `f64`: it is a UI fuzziness knob derived from
//! screen pixels, not a geometric quantity the kernel reasons about, so it
//! is compared against [`Scalar::to_f64`] rather than folded into interval
//! arithmetic.

use geop_core_math::{
    geop_error::GeopResult, primitives::CoordinateSystem, scalars::Scalar, vector::Vector3,
};
use geop_ops::{Part, SketchId};
use geop_core_sketch::{CurveId, profile::curve_polyline};
use geop_core_topology::{FaceId, Model, SolidId};
use geop_ops_rasterize::RasterizedModel;

/// A pickable ray, in world space. `dir` need not be unit length; `t` in
/// [`PickHit`] is in units of `dir`, i.e. the hit point is
/// `origin + dir * t`.
#[derive(Clone, Copy, Debug)]
pub struct Ray<S: Scalar> {
    pub origin: Vector3<S>,
    pub dir: Vector3<S>,
}

/// Which kind of entity a pick found (or was asked to look for).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PickKind {
    Vertex,
    Edge,
    Face,
    Solid,
}

/// What a pick query is looking for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickFilter {
    Vertex,
    Edge,
    Face,
    /// Hit-test faces, but report the *solid* each hit face belongs to.
    Solid,
    /// A vertex, else an edge, else a face: the smallest entity under the
    /// ray — within `tolerance` for a vertex or edge, and not hidden behind
    /// a face — as picking one of several kinds of entity wants.
    Any,
}

#[derive(Clone, Copy, Debug)]
pub struct PickHit<S: Scalar> {
    pub kind: PickKind,
    pub id: u64,
    pub point: Vector3<S>,
    pub t: S,
}

/// Möller–Trumbore ray/triangle intersection. Returns the ray parameter `t`
/// of the hit (in front of the ray origin) if any. Degenerate cases
/// (ray parallel to the triangle's plane, or a division that can't be
/// resolved because the divisor could be zero) are reported as a miss —
/// exactly the honest answer when the enclosure can't rule zero out.
fn ray_triangle<S: Scalar>(ray: &Ray<S>, a: Vector3<S>, b: Vector3<S>, c: Vector3<S>) -> Option<S> {
    let e1 = b.sub(&a);
    let e2 = c.sub(&a);
    let h = ray.dir.prod_cross(&e2);
    let det = e1.prod_dot(&h);
    if det.could_be_equal(S::ZERO) {
        return None;
    }
    let inv_det = S::ONE.div(det).ok()?;
    let s = ray.origin.sub(&a);
    let u = s.prod_dot(&h).mul(inv_det);
    if u.definitely_less(S::ZERO) || u.definitely_greater(S::ONE) {
        return None;
    }
    let q = s.prod_cross(&e1);
    let v = ray.dir.prod_dot(&q).mul(inv_det);
    if v.definitely_less(S::ZERO) || u.add(v).definitely_greater(S::ONE) {
        return None;
    }
    let t = e2.prod_dot(&q).mul(inv_det);
    if t.could_be_greater(S::ZERO) {
        Some(t)
    } else {
        None
    }
}

/// Closest point on `ray` (clamped to `t >= 0`) to point `p`, and that `t`.
fn closest_point_on_ray<S: Scalar>(ray: &Ray<S>, p: &Vector3<S>) -> (Vector3<S>, S) {
    let denom = ray.dir.prod_dot(&ray.dir);
    let t = if denom.could_be_equal(S::ZERO) {
        S::ZERO
    } else {
        let raw = p
            .sub(&ray.origin)
            .prod_dot(&ray.dir)
            .div(denom)
            .unwrap_or(S::ZERO);
        if raw.definitely_less(S::ZERO) {
            S::ZERO
        } else {
            raw
        }
    };
    (ray.origin.add(&ray.dir.prod_scalar(t)), t)
}

/// Nonnegative part of `x`: a ray runs only forwards from its origin. The
/// "is this behind the origin" question is answered with the same
/// three-valued comparisons as everywhere else in the kernel; an ambiguous
/// (`could_be_` but not `definitely_`) value is left alone rather than
/// forced to zero, matching how a UI pick should stay permissive rather
/// than sharpen an uncertain answer.
fn clamp0<S: Scalar>(x: S) -> S {
    if x.definitely_less(S::ZERO) {
        S::ZERO
    } else {
        x
    }
}

/// Closest points between `ray` and the segment `p..q` — the classic
/// "Real-Time Collision Detection" `ClosestPtSegmentSegment`, with the
/// ray's parameter bounded only below — returning `(distance, t)`, `t` the
/// ray parameter of its closest point (in units of `ray.dir`).
///
/// The ray is taken as it is, unbounded, rather than as a long segment:
/// squaring such a segment's length leaves the range of a fixed-point
/// scalar, and every edge pick came back empty with it.
fn closest_ray_segment<S: Scalar>(ray: &Ray<S>, p: Vector3<S>, q: Vector3<S>) -> (S, S) {
    let d1 = ray.dir;
    let d2 = q.sub(&p);
    let r = ray.origin.sub(&p);
    let a = d1.prod_dot(&d1);
    let e = d2.prod_dot(&d2);
    let f = d2.prod_dot(&r);
    let c = d1.prod_dot(&r);

    // The ray parameter nearest the segment's point at `u`.
    let t_at = |num: S| clamp0(num.div(a).unwrap_or(S::ZERO));
    let (t, u);
    if e.could_be_equal(S::ZERO) {
        u = S::ZERO;
        t = t_at(S::ZERO.sub(c));
    } else {
        let b = d1.prod_dot(&d2);
        let denom = a.mul(e).sub(b.mul(b));
        let t0 = if denom.could_be_equal(S::ZERO) {
            S::ZERO
        } else {
            clamp0(b.mul(f).sub(c.mul(e)).div(denom).unwrap_or(S::ZERO))
        };
        let u0 = b.mul(t0).add(f).div(e).unwrap_or(S::ZERO);
        if u0.definitely_less(S::ZERO) {
            u = S::ZERO;
            t = t_at(S::ZERO.sub(c));
        } else if u0.definitely_greater(S::ONE) {
            u = S::ONE;
            t = t_at(b.sub(c));
        } else {
            u = u0;
            t = t0;
        }
    }
    let on_ray = ray.origin.add(&d1.prod_scalar(t));
    let on_segment = p.add(&d2.prod_scalar(u));
    (on_ray.sub(&on_segment).norm(), t)
}

fn pick_vertex<S: Scalar>(
    rasterized: &RasterizedModel<S>,
    ray: &Ray<S>,
    tolerance: f64,
) -> Option<PickHit<S>> {
    let mut best: Option<PickHit<S>> = None;
    for (&id, &p) in rasterized.vertices.iter() {
        let (closest, t) = closest_point_on_ray(ray, &p);
        let dist = p.sub(&closest).norm();
        if dist.to_f64() > tolerance {
            continue;
        }
        if best.is_none_or(|b| t.to_f64() < b.t.to_f64()) {
            best = Some(PickHit {
                kind: PickKind::Vertex,
                id: id.0,
                point: p,
                t,
            });
        }
    }
    best
}

fn pick_edge<S: Scalar>(
    rasterized: &RasterizedModel<S>,
    ray: &Ray<S>,
    tolerance: f64,
) -> Option<PickHit<S>> {
    let mut best: Option<PickHit<S>> = None;
    for (&id, poly) in rasterized.edges.iter() {
        for seg in poly.windows(2) {
            let (dist, t) = closest_ray_segment(ray, seg[0], seg[1]);
            if dist.to_f64() > tolerance {
                continue;
            }
            if best.is_none_or(|b| t.to_f64() < b.t.to_f64()) {
                best = Some(PickHit {
                    kind: PickKind::Edge,
                    id: id.0,
                    point: ray.origin.add(&ray.dir.prod_scalar(t)),
                    t,
                });
            }
        }
    }
    best
}

/// The solid that owns `face_id`, if it can be found (a face is always
/// part of exactly one shell, and a shell of exactly one solid).
pub fn solid_of_face<S: Scalar>(model: &Model<S>, face_id: FaceId) -> Option<SolidId> {
    model
        .get_face(face_id)
        .ok()
        .and_then(|f| model.get_shell(f.shell).ok())
        .map(|s| s.solid)
}

fn pick_face_or_solid<S: Scalar>(
    model: &Model<S>,
    rasterized: &RasterizedModel<S>,
    ray: &Ray<S>,
    report_solid: bool,
) -> GeopResult<Option<PickHit<S>>> {
    let mut best: Option<(S, FaceId, Vector3<S>)> = None;
    for (&face_id, tris) in rasterized.faces.iter() {
        for tri in tris {
            if let Some(t) = ray_triangle(ray, tri.a, tri.b, tri.c) {
                if best.is_none_or(|(bt, ..)| t.to_f64() < bt.to_f64()) {
                    best = Some((t, face_id, ray.origin.add(&ray.dir.prod_scalar(t))));
                }
            }
        }
    }
    Ok(best.map(|(t, face_id, point)| {
        if report_solid {
            let solid_id = solid_of_face(model, face_id).unwrap_or(SolidId(0));
            PickHit {
                kind: PickKind::Solid,
                id: solid_id.0,
                point,
                t,
            }
        } else {
            PickHit {
                kind: PickKind::Face,
                id: face_id.0,
                point,
                t,
            }
        }
    }))
}

/// Cast `ray` against `model`, as `rasterized` samples it, and return the
/// nearest entity matching `filter` within `tolerance` (world-space
/// distance; only meaningful for [`PickFilter::Vertex`]/[`PickFilter::Edge`]
/// — face/solid hits are exact ray/triangle intersections and ignore it).
pub fn pick<S: Scalar>(
    model: &Model<S>,
    rasterized: &RasterizedModel<S>,
    ray: Ray<S>,
    filter: PickFilter,
    tolerance: f64,
) -> GeopResult<Option<PickHit<S>>> {
    match filter {
        PickFilter::Vertex => Ok(pick_vertex(rasterized, &ray, tolerance)),
        PickFilter::Edge => Ok(pick_edge(rasterized, &ray, tolerance)),
        PickFilter::Face => pick_face_or_solid(model, rasterized, &ray, false),
        PickFilter::Solid => pick_face_or_solid(model, rasterized, &ray, true),
        PickFilter::Any => {
            let face = pick_face_or_solid(model, rasterized, &ray, false)?;
            // In front of the face hit, give or take the tolerance: a vertex
            // or edge on the face's own boundary lies right at it.
            let visible = |hit: &PickHit<S>| {
                face.is_none_or(|f| {
                    let slack = tolerance / ray.dir.norm().to_f64();
                    hit.t.to_f64() <= f.t.to_f64() + slack
                })
            };
            Ok(pick_vertex(rasterized, &ray, tolerance)
                .filter(visible)
                .or_else(|| pick_edge(rasterized, &ray, tolerance).filter(visible))
                .or(face))
        }
    }
}

/// A sketch a ray hit, where it hit its plane.
#[derive(Clone, Copy, Debug)]
pub struct SketchHit<S: Scalar> {
    pub sketch: SketchId,
    pub point: Vector3<S>,
    pub t: S,
}

/// One sketch as something to click on and to draw: its plane, the outlines
/// of its closed regions, and every curve as a polyline — all in sketch
/// coordinates.
pub struct SketchTarget<S: Scalar> {
    pub sketch: SketchId,
    pub plane: CoordinateSystem<S>,
    /// Per region, its outer loop and its holes.
    pub regions: Vec<Vec<Vec<[f64; 2]>>>,
    /// Every curve: its id, whether it is construction geometry, and its
    /// points.
    pub curves: Vec<(CurveId, bool, Vec<[f64; 2]>)>,
}

/// Every sketch of a part, outlined once — finding a sketch's regions is
/// the expensive part of picking it — for picking with [`pick_sketch`] and
/// for drawing.
pub struct SketchTargets<S: Scalar>(pub Vec<SketchTarget<S>>);

impl<S: Scalar> SketchTargets<S> {
    pub fn of(part: &Part<S>) -> Self {
        Self(
            part.sketches()
                .map(|(sketch, placed)| {
                    let s = &placed.sketch;
                    let positions = s.positions();
                    // A sketch whose curves form no region is still a
                    // sketch, clicked on its curves.
                    let regions = s
                        .regions()
                        .map(|regions| {
                            regions
                                .iter()
                                .map(|r| {
                                    std::iter::once(&r.outer)
                                        .chain(&r.holes)
                                        .map(|l| l.polyline(s, &positions))
                                        .collect()
                                })
                                .collect()
                        })
                        .unwrap_or_default();
                    let curves = s
                        .curves
                        .iter()
                        .map(|(&id, c)| (id, c.construction, curve_polyline(s, &positions, id)))
                        .collect();
                    SketchTarget {
                        sketch,
                        plane: placed.plane.clone(),
                        regions,
                        curves,
                    }
                })
                .collect(),
        )
    }
}

/// Whether `p` lies inside the closed polylines `loops` (outer boundaries
/// and holes alike): an odd number of crossings of a ray along `+x`.
fn inside_loops(loops: &[Vec<[f64; 2]>], p: [f64; 2]) -> bool {
    let mut inside = false;
    for poly in loops {
        for (i, a) in poly.iter().enumerate() {
            let b = poly[(i + 1) % poly.len()];
            if (a[1] > p[1]) != (b[1] > p[1]) {
                let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                if x > p[0] {
                    inside = !inside;
                }
            }
        }
    }
    inside
}

/// Distance from `p` to the segment `a..b`.
fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let l2 = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if l2 == 0.0 {
        0.0
    } else {
        (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2).clamp(0.0, 1.0)
    };
    (p[0] - a[0] - t * ab[0]).hypot(p[1] - a[1] - t * ab[1])
}

/// Where `ray` hits `target`'s plane, if that is on the sketch: inside one
/// of its closed regions — the area a viewer shades — or within `tolerance`
/// of one of its curves, closed or not.
fn hit_sketch<S: Scalar>(
    target: &SketchTarget<S>,
    ray: &Ray<S>,
    tolerance: f64,
) -> Option<(S, Vector3<S>)> {
    let plane = &target.plane;
    let denom = ray.dir.prod_dot(plane.w());
    // Edge-on: the ray runs within the plane and hits no area of it.
    if !denom.definitely_greater(S::ZERO) && !denom.definitely_less(S::ZERO) {
        return None;
    }
    let t = plane
        .origin()
        .sub(&ray.origin)
        .prod_dot(plane.w())
        .div(denom)
        .ok()?;
    if t.definitely_less(S::ZERO) {
        return None;
    }
    let point = ray.origin.add(&ray.dir.prod_scalar(t));
    let local = point.sub(plane.origin());
    let p = [
        local.prod_dot(plane.u()).to_f64(),
        local.prod_dot(plane.v()).to_f64(),
    ];
    let in_region = target.regions.iter().any(|loops| inside_loops(loops, p));
    let on_curve = || {
        target.curves.iter().any(|(_, _, polyline)| {
            polyline
                .windows(2)
                .any(|w| segment_distance(p, w[0], w[1]) <= tolerance)
        })
    };
    (in_region || on_curve()).then_some((t, point))
}

/// The sketch nearest along `ray` that it hits (see [`hit_sketch`]), with
/// `tolerance` the world-space distance within which a click counts as on
/// a curve.
pub fn pick_sketch<S: Scalar>(
    targets: &SketchTargets<S>,
    ray: Ray<S>,
    tolerance: f64,
) -> Option<SketchHit<S>> {
    targets
        .0
        .iter()
        .filter_map(|target| {
            hit_sketch(target, &ray, tolerance).map(|(t, point)| SketchHit {
                sketch: target.sketch,
                point,
                t,
            })
        })
        .min_by(|a, b| a.t.to_f64().total_cmp(&b.t.to_f64()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;
    use geop_ops::Part;
    use geop_ops_extrude_revolve::cube_solid;
    use geop_ops_rasterize::rasterize_model_tagged;

    fn ray<S: Scalar>(origin: [f64; 3], dir: [f64; 3]) -> Ray<S> {
        Ray {
            origin: Vector3::from_array([
                S::from_f64(origin[0]),
                S::from_f64(origin[1]),
                S::from_f64(origin[2]),
            ]),
            dir: Vector3::from_array([
                S::from_f64(dir[0]),
                S::from_f64(dir[1]),
                S::from_f64(dir[2]),
            ]),
        }
    }

    fn check_pick_face_hits_cube_top<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t1",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        let r = ray::<S>([0.5, 0.5, 5.0], [0.0, 0.0, -1.0]);
        let hit = pick(
            &model,
            &rasterize_model_tagged(&model, 16).unwrap(),
            r,
            PickFilter::Face,
            1e-3,
        )
        .unwrap()
        .unwrap();
        assert_eq!(hit.kind, PickKind::Face);
        assert!(
            (hit.point[2].to_f64() - 1.0).abs() < 1e-6,
            "point={:?}",
            hit.point
        );
    }
    #[test]
    fn pick_face_hits_cube_top() {
        for_all_scalars!(check_pick_face_hits_cube_top);
    }

    fn check_pick_solid_reports_owning_solid<S: Scalar>() {
        let mut part = Part::<S>::new();
        let solid_id = cube_solid(
            &mut part,
            "t2",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        let r = ray::<S>([0.5, 0.5, 5.0], [0.0, 0.0, -1.0]);
        let hit = pick(
            &model,
            &rasterize_model_tagged(&model, 16).unwrap(),
            r,
            PickFilter::Solid,
            1e-3,
        )
        .unwrap()
        .unwrap();
        assert_eq!(hit.kind, PickKind::Solid);
        assert_eq!(hit.id, solid_id.0);
    }
    #[test]
    fn pick_solid_reports_owning_solid() {
        for_all_scalars!(check_pick_solid_reports_owning_solid);
    }

    fn check_pick_misses_when_ray_does_not_cross_cube<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t3",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        let r = ray::<S>([10.0, 10.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(
            pick(
                &model,
                &rasterize_model_tagged(&model, 16).unwrap(),
                r,
                PickFilter::Face,
                1e-3
            )
            .unwrap()
            .is_none()
        );
    }
    #[test]
    fn pick_misses_when_ray_does_not_cross_cube() {
        for_all_scalars!(check_pick_misses_when_ray_does_not_cross_cube);
    }

    fn check_pick_vertex_within_tolerance<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t4",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        // Aim just past the (1,1,1) corner, well within tolerance.
        let r = ray::<S>([1.01, 1.01, 5.0], [0.0, 0.0, -1.0]);
        let hit = pick(
            &model,
            &rasterize_model_tagged(&model, 16).unwrap(),
            r,
            PickFilter::Vertex,
            0.1,
        )
        .unwrap()
        .unwrap();
        assert_eq!(hit.kind, PickKind::Vertex);
    }
    #[test]
    fn pick_vertex_within_tolerance() {
        for_all_scalars!(check_pick_vertex_within_tolerance);
    }

    /// Picking any entity: the corner near the cursor, else the edge, else
    /// the face — but never one hidden behind the face in front.
    fn check_pick_any_prefers_the_smallest_visible<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t5",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        let raster = rasterize_model_tagged(model, 16).unwrap();
        let any = |origin: [f64; 3], dir: [f64; 3]| {
            pick(model, &raster, ray::<S>(origin, dir), PickFilter::Any, 0.05)
                .unwrap()
                .unwrap()
                .kind
        };
        assert_eq!(any([1.01, 1.01, 5.0], [0.0, 0.0, -1.0]), PickKind::Vertex);
        assert_eq!(any([0.5, 0.99, 5.0], [0.0, 0.0, -1.0]), PickKind::Edge);
        assert_eq!(any([0.5, 0.5, 5.0], [0.0, 0.0, -1.0]), PickKind::Face);
        // From the side, through the cube's front face: the edges of the
        // back face lie right behind the cursor, and are not picked.
        assert_eq!(any([0.5, -5.0, 0.99], [0.0, 1.0, 0.0]), PickKind::Edge);
        assert_eq!(any([0.5, -5.0, 0.5], [0.0, 1.0, 0.0]), PickKind::Face);
    }
    #[test]
    fn pick_any_prefers_the_smallest_visible() {
        for_all_scalars!(check_pick_any_prefers_the_smallest_visible);
    }

    /// A sketch is hit inside its closed region and on its curves, not
    /// beside them; of two sketches, the nearer one wins.
    #[test]
    fn pick_sketch_hits_regions_and_curves() {
        use geop_core_math::{primitives::CoordinateSystem, scalars::ScalInF64 as S};
        use geop_core_sketch::Sketch;
        let v = |x: f64, y: f64, z: f64| Vector3::from_array([x, y, z].map(S::from_f64));
        let placed = |z: f64, sketch: &Sketch| geop_ops::PlacedSketch {
            plane: CoordinateSystem::try_new(
                v(0.0, 0.0, z),
                v(1.0, 0.0, 0.0),
                v(0.0, 1.0, 0.0),
                v(0.0, 0.0, 1.0),
            )
            .unwrap(),
            sketch: sketch.clone(),
        };
        let mut square = Sketch::new();
        let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
            .iter()
            .map(|c| square.add_point(c[0], c[1]))
            .collect();
        for i in 0..4 {
            square.add_line(p[i], p[(i + 1) % 4]);
        }
        let mut part = Part::<S>::new();
        let low = part.add_sketch(placed(0.0, &square), "low").unwrap();
        let high = part.add_sketch(placed(1.0, &square), "high").unwrap();
        let targets = SketchTargets::of(&part);
        let down = |x: f64, y: f64| Ray {
            origin: v(x, y, 5.0),
            dir: v(0.0, 0.0, -1.0),
        };

        assert_eq!(
            pick_sketch(&targets, down(0.5, 0.5), 0.01).unwrap().sketch,
            high
        );
        assert_eq!(
            pick_sketch(&targets, down(1.005, 0.5), 0.01)
                .unwrap()
                .sketch,
            high
        );
        assert!(pick_sketch(&targets, down(1.5, 0.5), 0.01).is_none());
        // From below, the lower sketch is the nearer.
        let up = Ray {
            origin: v(0.5, 0.5, -5.0),
            dir: v(0.0, 0.0, 1.0),
        };
        assert_eq!(pick_sketch(&targets, up, 0.01).unwrap().sketch, low);
    }
}
