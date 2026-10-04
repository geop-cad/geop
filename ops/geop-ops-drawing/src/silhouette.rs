//! Silhouettes: where a curved face turns away from the eye, the outline it
//! shows without any edge there — the sides of a cylinder seen side on.
//!
//! For a view along `d`, a point of a surface `S(u, v)` is on its silhouette
//! when its normal is perpendicular to `d`: `g(u, v) = (S_u x S_v) . d = 0`.
//! That is a curve in `(u, v)`, traced here as such:
//!
//! 1. **Seeds.** `g`'s sign is sampled along a grid of iso lines of the
//!    surface's domain. Between two samples of opposite sign a silhouette
//!    crosses the line, and bisection finds where. Sampling only decides
//!    where to look: what is traced is the exact zero set. The interior grid
//!    lines sit at irrational fractions of the domain, a free choice made so
//!    they do not run along the symmetric iso lines silhouettes so often are.
//! 2. **Tracing.** From a seed, predictor-corrector steps follow the curve:
//!    a step along the tangent `(-g_v, g_u)`, sized to a fixed length in 3-D,
//!    then Newton back onto `g = 0`. A trace ends where the curve leaves the
//!    domain, landing exactly on its boundary, or where it closes on itself.
//!    Every grid line a trace crosses marks the seed there as traced.
//! 3. **Trimming.** The traced curve in `(u, v)` is cut where it crosses the
//!    face's trim curves, and only the pieces inside the face are kept.
//! 4. **Curves.** Each piece becomes a 3-D curve through exact silhouette
//!    points along it.
//!
//! The curve interpolates the silhouette, so a point of it is close to the
//! silhouette, not on it. Whoever needs a point exactly on it — the
//! visibility test does: a ray grazing a face at its silhouette tells the
//! truth only from the exact point — asks [`Silhouette::exact_point`].

use geop_core_geometry::{
    intersection::curve_curve_overlaps_and_crossings,
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{
    CoedgeId, FaceId, Model,
    contains::face::{PointClassification, face_contains},
};

use crate::{MAX_NODES, min_subdivision_size};

/// Interior grid lines per direction of the domain, besides its boundary.
const GRID_LINES: usize = 12;
/// Sample intervals along each grid line.
const GRID_SAMPLES: usize = 32;
/// Where the interior grid lines sit within their cells: the golden
/// section, far from the halves and quarters symmetric curves cross at.
const GRID_OFFSET: f64 = 0.381_966_011_250_105;
/// Newton steps bringing a point onto the silhouette.
const CORRECTOR_ITERATIONS: usize = 24;
/// Bisection steps locating a seed between two samples.
const BISECTIONS: usize = 60;
/// Trace steps per face, at most: how hard a trace tries.
const MAX_TRACE_STEPS: usize = 4000;
/// Steps a trace takes along the face's size.
const STEPS_PER_FACE: f64 = 24.0;
/// Seed for the containment tests.
const FACE_CONTAINS_SEED: u64 = 0x5_11_4u64;

/// A piece of a face's silhouette.
#[derive(Clone, Debug)]
pub struct Silhouette<S: Scalar> {
    pub face: FaceId,
    pub curve: NurbCurve3D<S>,
    /// Exact silhouette points along it, as `(u, v)` on the face's surface:
    /// where [`Silhouette::exact_point`] starts from.
    samples: Vec<Vector2<S>>,
}

/// The silhouette function `g = (S_u x S_v) . d` and its gradient.
fn contour<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    u: S,
    v: S,
) -> GeopResult<(S, S, S)> {
    let (su, sv) = surface.derivatives(u, v)?;
    let [suu, suv, svv] = surface.second_derivatives(u, v)?;
    let g = su.prod_cross(&sv).prod_dot(d);
    let gu = suu.prod_cross(&sv).add(&su.prod_cross(&suv)).prod_dot(d);
    let gv = suv.prod_cross(&sv).add(&su.prod_cross(&svv)).prod_dot(d);
    Ok((g, gu, gv))
}

/// `g`'s sign at `(u, v)`, from the unit normal — which, unlike `S_u x S_v`,
/// does not vanish at a pole — or `None` where it has no definite one.
fn sign<S: Scalar>(surface: &NurbSurface3D<S>, d: &Vector3<S>, u: S, v: S) -> Option<bool> {
    let n = surface.normal(u, v).ok()?.prod_dot(d);
    if n.definitely_greater(S::ZERO) {
        Some(true)
    } else if n.definitely_less(S::ZERO) {
        Some(false)
    } else {
        None
    }
}

/// `x` clamped into `[lo, hi]`.
fn clamp<S: Scalar>(x: S, lo: S, hi: S) -> S {
    geop_core_geometry::nurb_surface::clamp(x, lo, hi)
}

/// `(u, v)` moved onto the silhouette by Newton steps along `g`'s gradient
/// (the smallest step that zeroes `g` to first order), or `None` if it does
/// not get there. Every iterate is sharpened: each is only the seed of the
/// next, and what is returned is a point *of* the silhouette, as good as any
/// other there.
fn correct<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    (mut u, mut v): (S, S),
) -> Option<(S, S)> {
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    for _ in 0..CORRECTOR_ITERATIONS {
        let (g, gu, gv) = contour(surface, d, u, v).ok()?;
        if g.could_be_equal(S::ZERO) {
            return Some((u, v));
        }
        let scale = g.div(gu.mul(gu).add(gv.mul(gv))).ok()?;
        u = clamp(u.sub(gu.mul(scale)).sharpen(), u_lo, u_hi);
        v = clamp(v.sub(gv.mul(scale)).sharpen(), v_lo, v_hi);
    }
    None
}

/// `(u, v)` on the domain's boundary line `fixed` moved onto the silhouette
/// along that line, or `None` if it does not get there.
fn correct_on_boundary<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    (mut u, mut v): (S, S),
    u_fixed: bool,
) -> Option<(S, S)> {
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    for _ in 0..CORRECTOR_ITERATIONS {
        let (g, gu, gv) = contour(surface, d, u, v).ok()?;
        if g.could_be_equal(S::ZERO) {
            return Some((u, v));
        }
        if u_fixed {
            v = clamp(v.sub(g.div(gv).ok()?).sharpen(), v_lo, v_hi);
        } else {
            u = clamp(u.sub(g.div(gu).ok()?).sharpen(), u_lo, u_hi);
        }
    }
    None
}

/// An iso line of the domain the seeds are sampled along.
struct GridLine<S: Scalar> {
    /// Whether `u` is fixed along it (else `v` is).
    u_fixed: bool,
    value: S,
    /// The other parameter at each sample.
    params: Vec<S>,
    /// Sign changes between samples: the sample indices on either side.
    brackets: Vec<(usize, usize)>,
    /// Which brackets a trace has crossed.
    traced: Vec<bool>,
}

impl<S: Scalar> GridLine<S> {
    fn point(&self, t: S) -> (S, S) {
        if self.u_fixed {
            (self.value, t)
        } else {
            (t, self.value)
        }
    }

    /// Marks traced the bracket a trace crossed at `t` along this line: the
    /// one whose samples, widened by one interval each way, hold `t`, and the
    /// nearest of them if several do. None, if no bracket is that near —
    /// the crossing is one the samples did not see.
    fn mark(&mut self, t: f64) {
        let p = |k: usize| self.params[k].to_f64();
        let step = p(1) - p(0);
        let near = self
            .brackets
            .iter()
            .enumerate()
            .filter(|(_, (lo, hi))| p(*lo) - step <= t && t <= p(*hi) + step)
            .min_by(|(_, a), (_, b)| {
                let distance = |&(lo, hi): &(usize, usize)| (0.5 * (p(lo) + p(hi)) - t).abs();
                distance(a).total_cmp(&distance(b))
            })
            .map(|(i, _)| i);
        if let Some(i) = near {
            self.traced[i] = true;
        }
    }
}

/// `k / n` of the way from `lo` to `hi`, sharp: a sample position, a free
/// choice.
fn fraction<S: Scalar>(lo: S, hi: S, f: f64) -> S {
    lo.add(hi.sub(lo).mul(S::from_f64(f))).sharpen()
}

/// The grid lines of `surface`'s domain, sampled for `g`'s sign.
fn grid<S: Scalar>(surface: &NurbSurface3D<S>, d: &Vector3<S>) -> Vec<GridLine<S>> {
    let mut fractions = vec![0.0];
    fractions.extend((0..GRID_LINES).map(|k| (k as f64 + GRID_OFFSET) / GRID_LINES as f64));
    fractions.push(1.0);
    let mut lines = Vec::new();
    for u_fixed in [true, false] {
        let ((a_lo, a_hi), (b_lo, b_hi)) = if u_fixed {
            (surface.domain_u(), surface.domain_v())
        } else {
            (surface.domain_v(), surface.domain_u())
        };
        for &f in &fractions {
            let value = match f {
                0.0 => a_lo,
                1.0 => a_hi,
                _ => fraction(a_lo, a_hi, f),
            };
            let params: Vec<S> = (0..=GRID_SAMPLES)
                .map(|k| match k {
                    0 => b_lo,
                    k if k == GRID_SAMPLES => b_hi,
                    k => fraction(b_lo, b_hi, k as f64 / GRID_SAMPLES as f64),
                })
                .collect();
            let mut line = GridLine {
                u_fixed,
                value,
                params,
                brackets: Vec::new(),
                traced: Vec::new(),
            };
            let mut last: Option<(usize, bool)> = None;
            for k in 0..line.params.len() {
                let (u, v) = line.point(line.params[k]);
                let Some(s) = sign(surface, d, u, v) else {
                    continue;
                };
                if let Some((j, previous)) = last
                    && previous != s
                {
                    line.brackets.push((j, k));
                }
                last = Some((k, s));
            }
            line.traced = vec![false; line.brackets.len()];
            lines.push(line);
        }
    }
    lines
}

/// The seed in `line`'s bracket `(lo, hi)`: bisected on `g`'s sign, then
/// moved onto the silhouette.
fn seed<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    line: &GridLine<S>,
    (lo, hi): (usize, usize),
) -> Option<(S, S)> {
    let (mut a, mut b) = (line.params[lo], line.params[hi]);
    let (u, v) = line.point(a);
    let sign_a = sign(surface, d, u, v)?;
    for _ in 0..BISECTIONS {
        let mid = a.add(b).div(S::TWO).ok()?.sharpen();
        let (u, v) = line.point(mid);
        match sign(surface, d, u, v) {
            Some(s) if s == sign_a => a = mid,
            Some(_) => b = mid,
            None => {
                a = mid;
                b = mid;
                break;
            }
        }
    }
    let t = a.add(b).div(S::TWO).ok()?.sharpen();
    correct(surface, d, line.point(t))
}

/// Marks traced every bracket the step from `a` to `b` crosses.
fn mark_crossings<S: Scalar>(lines: &mut [GridLine<S>], a: (S, S), b: (S, S)) {
    let (au, av, bu, bv) = (a.0.to_f64(), a.1.to_f64(), b.0.to_f64(), b.1.to_f64());
    for line in lines.iter_mut() {
        let c = line.value.to_f64();
        let (from, to, other_from, other_to) = if line.u_fixed {
            (au, bu, av, bv)
        } else {
            (av, bv, au, bu)
        };
        if (from - c) * (to - c) <= 0.0 && from != to {
            let t = other_from + (c - from) / (to - from) * (other_to - other_from);
            line.mark(t);
        }
    }
}

/// How a march along the silhouette ended.
enum Ending {
    /// It came back to where it started.
    Closed,
    /// It left the domain, or could not go on.
    Open,
}

/// Marches from `start` along the silhouette, the way `forward` says,
/// pushing each point onto `points` (not `start` itself).
#[allow(clippy::too_many_arguments)]
fn march<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    start: (S, S),
    forward: bool,
    step: S,
    lines: &mut [GridLine<S>],
    points: &mut Vec<(S, S)>,
) -> GeopResult<Ending> {
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    let start_point = surface.evaluate(start.0, start.1)?;
    let mut current = start;
    let mut travelled = S::ZERO;
    for _ in 0..MAX_TRACE_STEPS {
        let (u, v) = current;
        let Ok((_, gu, gv)) = contour(surface, d, u, v) else {
            return Ok(Ending::Open);
        };
        let (mut tu, mut tv) = (gv.neg(), gu);
        if !forward {
            (tu, tv) = (tu.neg(), tv.neg());
        }
        let (su, sv) = surface.derivatives(u, v)?;
        let speed = su.prod_scalar(tu).add(&sv.prod_scalar(tv)).norm();
        let Ok(scale) = step.div(speed) else {
            return Ok(Ending::Open);
        };
        let (du, dv) = (tu.mul(scale), tv.mul(scale));
        let (pu, pv) = (u.add(du).sharpen(), v.add(dv).sharpen());
        let leaves_u = pu.definitely_less(u_lo) || pu.definitely_greater(u_hi);
        let leaves_v = pv.definitely_less(v_lo) || pv.definitely_greater(v_hi);
        if leaves_u || leaves_v {
            // The fraction of the step to the boundary it reaches first, and
            // from there onto the silhouette along that boundary.
            let to = |x: S, dx: S, lo: S, hi: S| -> f64 {
                let bound = if dx.to_f64() < 0.0 { lo } else { hi };
                (bound.sub(x).to_f64() / dx.to_f64()).clamp(0.0, 1.0)
            };
            let fu = if leaves_u { to(u, du, u_lo, u_hi) } else { 1.0 };
            let fv = if leaves_v { to(v, dv, v_lo, v_hi) } else { 1.0 };
            let f = S::from_f64(fu.min(fv));
            let landed = (
                clamp(u.add(du.mul(f)).sharpen(), u_lo, u_hi),
                clamp(v.add(dv.mul(f)).sharpen(), v_lo, v_hi),
            );
            if let Some(end) = correct_on_boundary(surface, d, landed, fu <= fv) {
                mark_crossings(lines, current, end);
                points.push(end);
            }
            return Ok(Ending::Open);
        }
        let Some(next) = correct(surface, d, (pu, pv)) else {
            return Ok(Ending::Open);
        };
        mark_crossings(lines, current, next);
        points.push(next);
        travelled = travelled.add(step);
        let here = surface.evaluate(next.0, next.1)?;
        if travelled.definitely_greater(step.mul(S::TWO))
            && here.sub(&start_point).norm().definitely_less(step)
        {
            mark_crossings(lines, next, start);
            points.push(start);
            return Ok(Ending::Closed);
        }
        current = next;
    }
    Err(GeopError::new(format!(
        "the silhouette trace from {start:?} did not end within {MAX_TRACE_STEPS} steps of \
         {step:?}"
    )))
}

/// The silhouette through `seed`, traced both ways, as `(u, v)` points.
fn trace<S: Scalar>(
    surface: &NurbSurface3D<S>,
    d: &Vector3<S>,
    seed: (S, S),
    step: S,
    lines: &mut [GridLine<S>],
) -> GeopResult<Vec<(S, S)>> {
    let mut forward = Vec::new();
    let points = if let Ending::Closed = march(surface, d, seed, true, step, lines, &mut forward)? {
        let mut points = vec![seed];
        points.extend(forward);
        points
    } else {
        let mut backward = Vec::new();
        march(surface, d, seed, false, step, lines, &mut backward)?;
        backward.reverse();
        backward.push(seed);
        backward.extend(forward);
        backward
    };
    thin(surface, points, step)
}

/// `points` without those crowding the one before: where a curve is
/// sampled is a free choice, and samples a sliver apart — a step that
/// reached the domain's boundary all but, then the boundary itself — make
/// the fit through them ill posed. The ends stay.
fn thin<S: Scalar>(
    surface: &NurbSurface3D<S>,
    points: Vec<(S, S)>,
    step: S,
) -> GeopResult<Vec<(S, S)>> {
    let close = step.div(S::from_f64(8.0))?;
    let at = |p: &(S, S)| surface.evaluate(p.0, p.1);
    let count = points.len();
    let mut kept: Vec<(S, S)> = Vec::with_capacity(count);
    for (k, p) in points.into_iter().enumerate() {
        if let Some(last) = kept.last()
            && at(last)?.sub(&at(&p)?).norm().definitely_less(close)
        {
            if k + 1 == count && kept.len() > 1 {
                kept.pop();
            } else {
                continue;
            }
        }
        kept.push(p);
    }
    Ok(kept)
}

/// The size of `surface`: the diagonal of its control points' box.
fn extent<S: Scalar>(surface: &NurbSurface3D<S>) -> GeopResult<S> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for q in &surface.control_points {
        for k in 0..3 {
            let x = q[k].div(q[3])?.to_f64();
            lo[k] = lo[k].min(x);
            hi[k] = hi[k].max(x);
        }
    }
    let d2: f64 = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum();
    Ok(S::from_f64(d2.sqrt()))
}

/// Whether `surface` is swept straight along `d` in one of its parameter
/// directions — an extruded or turned cylinder seen end on — so that its
/// normal is perpendicular to `d` everywhere and the whole face is seen
/// edge on: then its outline is its edges, and it has no silhouette curve.
fn swept_along<S: Scalar>(surface: &NurbSurface3D<S>, d: &Vector3<S>) -> bool {
    let (nu, nv) = (surface.num_u, surface.num_v);
    let point = |i: usize, j: usize| surface.control_points[i * nv + j];
    let cartesian = |q: geop_core_math::vector::Vector4<S>| -> Option<Vector3<S>> {
        let w = q[3];
        Some(Vector3::from_array([
            q[0].div(w).ok()?,
            q[1].div(w).ok()?,
            q[2].div(w).ok()?,
        ]))
    };
    let along = |a: geop_core_math::vector::Vector4<S>, b: geop_core_math::vector::Vector4<S>| {
        let (Some(pa), Some(pb)) = (cartesian(a), cartesian(b)) else {
            return false;
        };
        a[3].could_be_equal(b[3])
            && pb
                .sub(&pa)
                .prod_cross(d)
                .could_be_equal(&Vector3::zero())
    };
    let along_v = (0..nu).all(|i| (1..nv).all(|j| along(point(i, j - 1), point(i, j))));
    let along_u = (0..nv).all(|j| (1..nu).all(|i| along(point(i - 1, j), point(i, j))));
    along_u || along_v
}

impl<S: Scalar> Silhouette<S> {
    /// The point of this silhouette near `near`, exactly on it: `near`
    /// projected onto the surface from the nearest exact sample, then moved
    /// onto the silhouette.
    pub fn exact_point(
        &self,
        model: &Model<S>,
        d: &Vector3<S>,
        near: &Vector3<S>,
    ) -> GeopResult<Vector3<S>> {
        let surface = &model.get_face(self.face)?.surface;
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Silhouette::exact_point(face={}, near={near:?})",
                self.face
            ))
        };
        let distance = |uv: &Vector2<S>| -> f64 {
            surface
                .evaluate(uv[0], uv[1])
                .map(|p| p.sub(near).norm_sq().to_f64())
                .unwrap_or(f64::INFINITY)
        };
        let seed = self
            .samples
            .iter()
            .min_by(|a, b| distance(a).total_cmp(&distance(b)))
            .ok_or_else(|| GeopError::new("a silhouette without samples"))
            .with_context(&ctx)?;
        let (u, v) = surface
            .project(*near, seed[0], seed[1], CORRECTOR_ITERATIONS)
            .with_context(&ctx)?;
        let (u, v) = correct(surface, d, (u.sharpen(), v.sharpen()))
            .ok_or_else(|| GeopError::new("the point does not converge onto the silhouette"))
            .with_context(&ctx)?;
        surface.evaluate(u, v).with_context(&ctx)
    }
}

/// Samples along an edge looking for where it crosses a silhouette.
const EDGE_SAMPLES: usize = 32;

/// Where the edge `curve`, bounding the face `face_id` along the coedge
/// `coedge_id`, crosses the face's silhouette seen along `d`: the
/// parameters where the face's normal there turns perpendicular to `d`.
///
/// Projected, an edge touches the silhouette of a face it bounds
/// tangentially where it crosses it — a contact no search on the paper can
/// isolate, so it is found here, in space: `g`'s sign sampled along the
/// edge, and each change bisected. Only isolated crossings: an edge along
/// which the face is seen edge on throughout has none.
pub fn edge_crossings<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    coedge_id: CoedgeId,
    curve: &NurbCurve3D<S>,
    d: &Vector3<S>,
) -> GeopResult<Vec<S>> {
    let surface = &model.get_face(face_id)?.surface;
    if surface.as_plane()?.is_some() || swept_along(surface, d) {
        return Ok(Vec::new());
    }
    let coedge = model.get_coedge(coedge_id)?;
    let (t0, t1) = curve.domain();
    let (s0, s1) = coedge.pcurve.domain();
    let edge_domain = model.get_edge(coedge.edge()?)?.curve.domain();
    // Where on the surface the edge's point at `t` is: projected from the
    // pcurve's point as far along it — a seed, a free choice.
    let foot = |t: S| -> Option<(S, S)> {
        let f = t.sub(edge_domain.0).div(edge_domain.1.sub(edge_domain.0)).ok()?;
        let f = match coedge.sense {
            geop_core_topology::Sense::Forward => f,
            geop_core_topology::Sense::Reversed => S::ONE.sub(f),
        };
        let seed = coedge.pcurve.evaluate(s0.add(s1.sub(s0).mul(f)).sharpen()).ok()?;
        let point = curve.evaluate(t).ok()?;
        surface
            .project(point, seed[0], seed[1], CORRECTOR_ITERATIONS)
            .ok()
    };
    let sign_at = |t: S| -> Option<bool> {
        let (u, v) = foot(t)?;
        sign(surface, d, u, v)
    };
    let params: Vec<S> = (0..=EDGE_SAMPLES)
        .map(|k| match k {
            0 => t0,
            k if k == EDGE_SAMPLES => t1,
            k => fraction(t0, t1, k as f64 / EDGE_SAMPLES as f64),
        })
        .collect();
    let mut crossings = Vec::new();
    let mut last: Option<(S, bool)> = None;
    for &t in &params {
        let Some(s) = sign_at(t) else { continue };
        if let Some((a, previous)) = last
            && previous != s
        {
            let (mut lo, mut hi) = (a, t);
            for _ in 0..BISECTIONS {
                let mid = lo.add(hi).div(S::TWO)?.sharpen();
                match sign_at(mid) {
                    Some(m) if m == previous => lo = mid,
                    Some(_) => hi = mid,
                    None => {
                        lo = mid;
                        hi = mid;
                        break;
                    }
                }
            }
            crossings.push(lo.union(hi));
        }
        last = Some((t, s));
    }
    Ok(crossings)
}

/// The silhouette of `face_id` seen along the unit vector `d`, as curves on
/// the face: none for a plane, or for a face seen edge on throughout.
pub fn face_silhouettes<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    d: &Vector3<S>,
) -> GeopResult<Vec<Silhouette<S>>> {
    let ctx = |e: GeopError| e.with_context(format!("face_silhouettes(face={face_id}, d={d:?})"));
    let surface = &model.get_face(face_id).with_context(&ctx)?.surface;
    if surface.as_plane().with_context(&ctx)?.is_some() || swept_along(surface, d) {
        return Ok(Vec::new());
    }
    let step = extent(surface)
        .with_context(&ctx)?
        .div(S::from_f64(STEPS_PER_FACE))
        .with_context(&ctx)?;
    let mut lines = grid(surface, d);
    let mut traces = Vec::new();
    for l in 0..lines.len() {
        for b in 0..lines[l].brackets.len() {
            if lines[l].traced[b] {
                continue;
            }
            lines[l].traced[b] = true;
            let Some(start) = seed(surface, d, &lines[l], lines[l].brackets[b]) else {
                continue;
            };
            traces.push(trace(surface, d, start, step, &mut lines).with_context(&ctx)?);
        }
    }
    let mut silhouettes = Vec::new();
    for points in traces {
        silhouettes.extend(trim(model, face_id, d, &points).with_context(&ctx)?);
    }
    Ok(silhouettes)
}

/// The pieces of the traced silhouette `points` inside the face, as 3-D
/// curves through exact silhouette points.
fn trim<S: Scalar>(
    model: &Model<S>,
    face_id: FaceId,
    d: &Vector3<S>,
    points: &[(S, S)],
) -> GeopResult<Vec<Silhouette<S>>> {
    let surface = &model.get_face(face_id)?.surface;
    let mut uv: Vec<Vector2<S>> = Vec::with_capacity(points.len());
    for &(u, v) in points {
        let p = Vector2::from_array([u, v]);
        if uv.last().is_none_or(|q: &Vector2<S>| !q.could_be_equal(&p)) {
            uv.push(p);
        }
    }
    if uv.len() < 2 {
        return Ok(Vec::new());
    }
    let fit = NurbCurve2D::interpolate(&uv, (uv.len() - 1).min(3))
        .map_err(|e| e.with_context(format!("fitting the traced silhouette through {uv:?}")))?;
    let (t0, t1) = fit.domain();

    // Where the trace crosses or runs along the face's trim.
    let mut cuts = vec![t0, t1];
    for coedge_id in model.iterate_face_coedges(face_id) {
        let pcurve = &model.get_coedge(coedge_id)?.pcurve;
        let (overlaps, crossings) =
            curve_curve_overlaps_and_crossings(&fit, pcurve, MAX_NODES, min_subdivision_size())?;
        cuts.extend(crossings.iter().map(|(s, _)| s.midpoint()));
        cuts.extend(overlaps.iter().flat_map(|o| [o.start.t, o.end.t]));
    }
    let mut cuts: Vec<S> = cuts.into_iter().map(|t| t.midpoint()).collect();
    cuts.sort_by(|a, b| a.to_f64().total_cmp(&b.to_f64()));
    cuts.dedup_by(|b, a| !b.definitely_greater(*a));

    let mut silhouettes = Vec::new();
    for pair in cuts.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let mid = a.add(b).div(S::TWO)?.sharpen();
        let at = fit.evaluate(mid)?;
        let inside = face_contains(
            model,
            face_id,
            at[0],
            at[1],
            MAX_NODES,
            min_subdivision_size(),
            FACE_CONTAINS_SEED,
        )?;
        if inside != PointClassification::Inside {
            continue;
        }
        // Exact points along the piece, as many as the trace had there and
        // at least enough for a cubic.
        let share = b.sub(a).div(t1.sub(t0))?.to_f64();
        let count = ((share * uv.len() as f64).ceil() as usize).max(4);
        let mut samples = Vec::with_capacity(count + 1);
        for k in 0..=count {
            let t = match k {
                0 => a,
                k if k == count => b,
                k => fraction(a, b, k as f64 / count as f64),
            };
            let q = fit.evaluate(t)?;
            let (u, v) = correct(surface, d, (q[0].sharpen(), q[1].sharpen())).unwrap_or((q[0], q[1]));
            samples.push(Vector2::from_array([u, v]));
        }
        let mut points3 = Vec::with_capacity(samples.len());
        for s in &samples {
            let p = surface.evaluate(s[0], s[1])?;
            if points3
                .last()
                .is_none_or(|q: &Vector3<S>| !q.could_be_equal(&p))
            {
                points3.push(p);
            }
        }
        if points3.len() < 2 {
            continue;
        }
        let curve = NurbCurve3D::interpolate(&points3, (points3.len() - 1).min(3))
            .map_err(|e| e.with_context(format!("fitting a silhouette through {points3:?}")))?;
        silhouettes.push(Silhouette {
            face: face_id,
            curve,
            samples,
        });
    }
    Ok(silhouettes)
}
