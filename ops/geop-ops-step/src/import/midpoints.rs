//! Curves and surfaces in `f64`: every control point, weight and knot at the
//! middle of its enclosure. For the work whose answer is a free choice or a
//! measurement in `f64` — where to sample, a seed for a search, how far a
//! point lies off a surface — many times cheaper than evaluating the
//! enclosures, and off them only by roundings.
//! The kernel itself stays in intervals: these serve the importer only,
//! never an answer the kernel relies on (see `Scalar::sharpen`).

use geop_core_geometry::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};
use geop_core_math::scalars::Scalar;

use super::geometry::{P3, dot, norm, sub};

/// A curve's midpoints in `f64`: its control points, weights and knots each
/// at the middle of its enclosure. For searches whose answer is a free
/// choice — which parameter to measure a gap at, where to sample — many
/// times cheaper than evaluating the enclosure, and off it only by
/// roundings.
pub struct CurveMidpoints {
    degree: usize,
    knots: Vec<f64>,
    /// Homogeneous: `(w x, w y, w z, w)`.
    points: Vec<[f64; 4]>,
}

impl CurveMidpoints {
    pub fn of<S: Scalar>(curve: &NurbCurve3D<S>) -> Self {
        Self {
            degree: curve.degree,
            knots: curve.knot_vector.iter().map(|k| k.to_f64()).collect(),
            points: curve
                .control_points
                .iter()
                .map(|p| std::array::from_fn(|c| p[c].to_f64()))
                .collect(),
        }
    }

    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.points.len()])
    }

    /// The point at `t`, clamped into the domain: de Boor's algorithm.
    pub fn point(&self, t: f64) -> P3 {
        let (lo, hi) = self.domain();
        let t = t.clamp(lo, hi);
        let k = span(self.degree, &self.knots, self.points.len(), t);
        let (h, _) = de_boor(
            self.degree,
            &self.knots,
            &self.points[k - self.degree..=k],
            k,
            t,
        );
        cartesian(h)
    }
}

/// A surface's midpoints in `f64` (see [`CurveMidpoints`]): for the searches on
/// a surface whose answer is a free choice or a measurement in `f64` — a
/// seed for a projection, how far a point lies off the surface, where to
/// draw it.
pub struct SurfaceMidpoints {
    degree_u: usize,
    degree_v: usize,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    num_u: usize,
    num_v: usize,
    /// Homogeneous, `u` index major, as the surface's own.
    points: Vec<[f64; 4]>,
}

/// Newton steps of a foot point search on [`SurfaceMidpoints`].
const PROJECT_ITERATIONS: usize = 30;

impl SurfaceMidpoints {
    pub fn of<S: Scalar>(surface: &NurbSurface3D<S>) -> Self {
        Self {
            degree_u: surface.degree_u,
            degree_v: surface.degree_v,
            knots_u: surface.knot_vector_u.iter().map(|k| k.to_f64()).collect(),
            knots_v: surface.knot_vector_v.iter().map(|k| k.to_f64()).collect(),
            num_u: surface.num_u,
            num_v: surface.num_v,
            points: surface
                .control_points
                .iter()
                .map(|p| std::array::from_fn(|c| p[c].to_f64()))
                .collect(),
        }
    }

    pub fn domain_u(&self) -> (f64, f64) {
        (self.knots_u[self.degree_u], self.knots_u[self.num_u])
    }

    pub fn domain_v(&self) -> (f64, f64) {
        (self.knots_v[self.degree_v], self.knots_v[self.num_v])
    }

    /// The point at `(u, v)`, clamped into the domain, and its partial
    /// derivatives there: `(S, S_u, S_v)`.
    pub fn partials(&self, u: f64, v: f64) -> (P3, P3, P3) {
        let (p, q) = (self.degree_u, self.degree_v);
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let (u, v) = (u.clamp(u0, u1), v.clamp(v0, v1));
        let ku = span(p, &self.knots_u, self.num_u, u);
        let kv = span(q, &self.knots_v, self.num_v, v);
        // Each column of the control points acting there at `u`, then
        // those at `v`.
        let (columns, columns_u): (Vec<[f64; 4]>, Vec<[f64; 4]>) = (kv - q..=kv)
            .map(|j| {
                let column: Vec<[f64; 4]> = (ku - p..=ku)
                    .map(|i| self.points[i * self.num_v + j])
                    .collect();
                de_boor(p, &self.knots_u, &column, ku, u)
            })
            .unzip();
        let (a, a_v) = de_boor(q, &self.knots_v, &columns, kv, v);
        let (a_u, _) = de_boor(q, &self.knots_v, &columns_u, kv, v);
        let point = cartesian(a);
        // The quotient rule: `S' = (A' - w' S) / w`.
        let rational =
            |d: [f64; 4]| -> P3 { std::array::from_fn(|c| (d[c] - d[3] * point[c]) / a[3]) };
        (point, rational(a_u), rational(a_v))
    }

    /// The point at `(u, v)`, clamped into the domain.
    pub fn point(&self, u: f64, v: f64) -> P3 {
        self.partials(u, v).0
    }

    /// The foot point of `target` on the surface by Newton from `(u, v)`,
    /// on the first fundamental form, in the domain. A step is taken only
    /// where it makes progress: brings the point nearer `target`, or, no
    /// further from it, nearer meeting the foot point's conditions
    /// `r · S_u = r · S_v = 0` (`r` from the point to `target`) — near the
    /// foot point the distance changes too little to tell in `f64`, the
    /// conditions do not; but alone they also lead to points furthest from
    /// `target`. Where
    /// the full step makes none — at a pole, where one derivative vanishes
    /// up to the roundings of the midpoints and the system is as good as
    /// singular — the step along either derivative alone; and where
    /// neither does, the step along a derivative from the other parameter
    /// at its domain's ends or middle, which at a pole is the same point on
    /// another meridian (as `NurbSurface::project` leaves a pole). Where
    /// none does, the search has arrived.
    pub fn project(&self, target: P3, mut u: f64, mut v: f64) -> (f64, f64) {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        // How far from `target` the point with partials `(p, su, sv)` is,
        // and how far from meeting the foot point's conditions.
        let progress = |(p, su, sv): (P3, P3, P3)| {
            let r = sub(target, p);
            (norm(r), dot(su, r).powi(2) + dot(sv, r).powi(2))
        };
        let mut now = self.partials(u, v);
        for _ in 0..PROJECT_ITERATIONS {
            let (distance_now, residual_now) = progress(now);
            // Where a step from `(u, v)`, with the partials there, leads:
            // the full one, or along `S_u` or `S_v` alone.
            let step = |(u, v): (f64, f64), (p, su, sv): (P3, P3, P3), along: Option<bool>| {
                let r = sub(target, p);
                let (a11, a12, a22) = (dot(su, su), dot(su, sv), dot(sv, sv));
                let (b1, b2) = (dot(su, r), dot(sv, r));
                let (du, dv) = match along {
                    None => {
                        let det = a11 * a22 - a12 * a12;
                        ((b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det)
                    }
                    Some(true) => (b1 / a11, 0.0),
                    Some(false) => (0.0, b2 / a22),
                };
                ((u + du).clamp(u0, u1), (v + dv).clamp(v0, v1))
            };
            let here = [None, Some(true), Some(false)]
                .into_iter()
                .map(|along| step((u, v), now, along));
            let elsewhere = [(u, v0), (u, (v0 + v1) / 2.0), (u, v1)]
                .into_iter()
                .map(|from| (from, Some(true)))
                .chain(
                    [(u0, v), ((u0 + u1) / 2.0, v), (u1, v)]
                        .into_iter()
                        .map(|from| (from, Some(false))),
                )
                .map(|(from, along)| step(from, self.partials(from.0, from.1), along));
            let Some((next, partials)) = here.chain(elsewhere).find_map(|next| {
                if !(next.0.is_finite() && next.1.is_finite()) {
                    return None;
                }
                let partials = self.partials(next.0, next.1);
                let (distance, residual) = progress(partials);
                let nearer = distance < distance_now;
                let closer = distance <= distance_now && residual < residual_now;
                (nearer || closer).then_some((next, partials))
            }) else {
                break;
            };
            (u, v) = next;
            now = partials;
        }
        (u, v)
    }
}

/// The span `k` of `knots` with `knots[k] <= t < knots[k + 1]`, for a
/// spline of `degree` with `n` control points: the last one at the domain's
/// end.
fn span(degree: usize, knots: &[f64], n: usize, t: f64) -> usize {
    (degree..n)
        .rev()
        .find(|&k| knots[k] <= t && knots[k] < knots[k + 1])
        .unwrap_or(degree)
}

/// De Boor's algorithm on the homogeneous control points `local` acting on
/// span `k` of `knots`: the point at `t` and its derivative, which is the
/// difference of the last two points before the last step, over the span
/// (`degree` times).
fn de_boor(
    degree: usize,
    knots: &[f64],
    local: &[[f64; 4]],
    k: usize,
    t: f64,
) -> ([f64; 4], [f64; 4]) {
    let p = degree;
    let mut d = local.to_vec();
    let mut derivative = [0.0; 4];
    for r in 1..=p {
        if r == p {
            let width = knots[k + 1] - knots[k];
            derivative = std::array::from_fn(|c| p as f64 * (d[p][c] - d[p - 1][c]) / width);
        }
        for j in (r..=p).rev() {
            let i = j + k - p;
            let width = knots[i + p + 1 - r] - knots[i];
            let a = if width == 0.0 {
                0.0
            } else {
                (t - knots[i]) / width
            };
            let before = d[j - 1];
            for (x, y) in d[j].iter_mut().zip(before) {
                *x = (1.0 - a) * y + a * *x;
            }
        }
    }
    (d[p], derivative)
}

/// The Cartesian point of the homogeneous `h`.
fn cartesian(h: [f64; 4]) -> P3 {
    [h[0] / h[3], h[1] / h[3], h[2] / h[3]]
}

#[cfg(test)]
mod tests {
    use geop_core_geometry::{nurb_curve::NurbCurve, nurb_surface::NurbSurface3D, shape::Axis};
    use geop_core_math::{
        scalars::{Scalar, scal_in_f64::ScalInF64},
        vector::{Vector3, Vector4},
    };

    use super::SurfaceMidpoints;
    use crate::import::geometry::{distance, to_p3};

    type S = ScalInF64;

    fn v3(p: [f64; 3]) -> Vector3<S> {
        Vector3::from_array(p.map(S::from_f64))
    }

    /// An eighth of the unit sphere: rational, of degree 4 and with a
    /// pole-free but strongly varying parametrization.
    fn octant() -> NurbSurface3D<S> {
        NurbSurface3D::spherical_triangle(
            &v3([0.0, 0.0, 0.0]),
            S::ONE,
            [
                v3([1.0, 0.0, 0.0]),
                v3([0.0, 1.0, 0.0]),
                v3([0.0, 0.0, 1.0]),
            ],
        )
        .unwrap()
    }

    #[test]
    fn surface_midpoints_are_the_surface_and_its_derivatives() {
        let surface = octant();
        let midpoints = SurfaceMidpoints::of(&surface);
        let (u0, u1) = midpoints.domain_u();
        let (v0, v1) = midpoints.domain_v();
        for i in 0..=6 {
            for j in 0..=6 {
                let u = u0 + (u1 - u0) * i as f64 / 6.0;
                let v = v0 + (v1 - v0) * j as f64 / 6.0;
                let (p, su, sv) = midpoints.partials(u, v);
                let (u, v) = (S::from_f64(u), S::from_f64(v));
                let (eu, ev) = surface.derivatives(u, v).unwrap();
                for (got, want) in [
                    (p, to_p3(&surface.evaluate(u, v).unwrap())),
                    (su, to_p3(&eu)),
                    (sv, to_p3(&ev)),
                ] {
                    assert!(
                        distance(got, want) < 1e-12,
                        "{got:?} against {want:?} at ({u:?}, {v:?})"
                    );
                }
            }
        }
    }

    #[test]
    fn surface_midpoints_project_onto_the_sphere() {
        let midpoints = SurfaceMidpoints::of(&octant());
        let (u0, u1) = midpoints.domain_u();
        let (v0, v1) = midpoints.domain_v();
        let target = [0.9, 0.5, 0.4];
        let (u, v) = midpoints.project(target, (u0 + u1) / 2.0, (v0 + v1) / 2.0);
        // How far the target lies from the sphere is what the foot point
        // measures; where along the sphere it is, `f64` pins down only to
        // the square root of a rounding, as the distance hardly changes.
        let r = (0.81f64 + 0.25 + 0.16).sqrt();
        let foot = midpoints.point(u, v);
        assert!((distance(foot, target) - (r - 1.0)).abs() < 1e-12);
        assert!(distance(foot, target.map(|c| c / r)) < 1e-7);
    }

    /// From a seed at the centre of a revolved disc, where every grid point
    /// of a row lies, a point on another meridian is found: leaving the
    /// centre along the seed's own meridian leads away from it.
    #[test]
    fn surface_midpoints_leave_a_pole_along_the_right_meridian() {
        let profile = NurbCurve::try_new(
            1,
            vec![
                Vector4::from_array([0.0, 0.0, 0.0, 1.0].map(S::from_f64)),
                Vector4::from_array([1.0, 0.0, 0.0, 1.0].map(S::from_f64)),
            ],
            [0.0, 0.0, 1.0, 1.0].map(S::from_f64).to_vec(),
        )
        .unwrap();
        let axis = Axis::try_new(v3([0.0, 0.0, 0.0]), v3([0.0, 0.0, 1.0])).unwrap();
        let disc = NurbSurface3D::revolve(&profile, &axis, 0.0, std::f64::consts::PI).unwrap();
        let midpoints = SurfaceMidpoints::of(&disc);
        let target = midpoints.point(0.95, 0.04);
        let (u, v) = midpoints.project(target, 0.0, 0.0);
        assert!(
            distance(midpoints.point(u, v), target) < 1e-12,
            "found ({u}, {v})"
        );
    }
}
