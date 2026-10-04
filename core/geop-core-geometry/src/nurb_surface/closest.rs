//! [`NurbSurface::closest_point`]: the point of a surface nearest a given
//! point.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::{NurbSurface, clamp};

/// Samples per polynomial piece and direction the search starts from.
/// Bounds effort only, as for curves.
const SAMPLES_PER_PIECE: usize = 6;

/// Newton steps from a seed, at most. Bounds effort only: every iterate is
/// a point of the surface nearer than the one before.
const NEWTON_STEPS: usize = 40;

/// How often a step that does not bring the point nearer is halved before
/// the search stops.
const HALVINGS: usize = 10;

/// How many of the nearest samples are refined.
const SEEDS: usize = 3;

impl<S: Scalar> NurbSurface<S, 4> {
    /// A grid of parameters across every polynomial piece, `n` per piece
    /// and direction, ends included — sharp: where to sample is a free
    /// choice.
    pub fn sample_parameters(&self, n: usize) -> GeopResult<Vec<(S, S)>> {
        let along = |breaks: Vec<S>| -> GeopResult<Vec<S>> {
            let mut out = Vec::new();
            for w in breaks.windows(2) {
                for i in 0..n {
                    let alpha = S::from_ratio(i as i64, n as i64)?;
                    out.push(S::interpolate(w[0], w[1], alpha).sharpen());
                }
            }
            out.push(*breaks.last().expect("a domain has two ends"));
            Ok(out)
        };
        let (us, vs) = (along(self.breakpoints_u())?, along(self.breakpoints_v())?);
        Ok(us
            .iter()
            .flat_map(|&u| vs.iter().map(move |&v| (u, v)))
            .collect())
    }

    /// The point of the (untrimmed) surface nearest `target`, with its
    /// parameters: the nearest few of a grid of samples, each refined by
    /// damped Newton (see [`NurbSurface::closest_point_from`]).
    pub fn closest_point(&self, target: &Vector3<S>) -> GeopResult<(S, S, Vector3<S>)> {
        let seeds = self.sample_parameters(SAMPLES_PER_PIECE)?;
        self.closest_point_from(target, &seeds)
    }

    /// [`NurbSurface::closest_point`], starting from the nearest few of
    /// `seeds` rather than a grid of its own.
    ///
    /// Subdivision isolates, Newton refines. The parameters are a free
    /// choice of which point of the surface is returned, so every iterate
    /// is sharp: the point returned lies on the surface, its distance to
    /// `target` is attained, and it is the least found.
    pub fn closest_point_from(
        &self,
        target: &Vector3<S>,
        seeds: &[(S, S)],
    ) -> GeopResult<(S, S, Vector3<S>)> {
        let distance = |p: &Vector3<S>| p.sub(target).norm_sq().to_f64();
        let mut scored = seeds
            .iter()
            .map(|&(u, v)| Ok((distance(&self.evaluate(u, v)?), u, v)))
            .collect::<GeopResult<Vec<_>>>()?;
        scored.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut best: Option<(f64, S, S, Vector3<S>)> = None;
        for &(_, u, v) in scored.iter().take(SEEDS) {
            let (u, v) = self.refine_closest(target, u, v)?;
            let p = self.evaluate(u, v)?;
            let d = distance(&p);
            if best.as_ref().is_none_or(|b| d < b.0) {
                best = Some((d, u, v, p));
            }
        }
        let (_, u, v, p) = best.ok_or_else(|| {
            geop_core_math::geop_error::GeopError::new("closest_point_from: no seeds")
        })?;
        Ok((u, v, p))
    }

    /// Damped Newton on the gradient of half the squared distance to
    /// `target`, from `(u, v)`. The step solves the full Hessian where it
    /// is definitely positive definite — near the nearest point, where it
    /// converges quadratically — and the first fundamental form where not
    /// (Gauss–Newton, a descent direction wherever the parametrization is
    /// regular); at a pole, where that is singular too, it moves along the
    /// one parameter that still moves the point. A step that does not
    /// bring the point nearer is halved, and the search stops once halving
    /// does not help. Every iterate is clamped into the domain.
    fn refine_closest(&self, target: &Vector3<S>, mut u: S, mut v: S) -> GeopResult<(S, S)> {
        let ((u_lo, u_hi), (v_lo, v_hi)) = (self.domain_u(), self.domain_v());
        let distance = |u: S, v: S| -> GeopResult<f64> {
            Ok(self.evaluate(u, v)?.sub(target).norm_sq().to_f64())
        };
        let positive = |a: S, b: S, c: S| {
            a.definitely_greater(S::ZERO) && a.mul(c).sub(b.mul(b)).definitely_greater(S::ZERO)
        };
        let mut current = distance(u, v)?;
        for _ in 0..NEWTON_STEPS {
            let r = self.evaluate(u, v)?.sub(target);
            let (su, sv) = self.derivatives(u, v)?;
            let [suu, suv, svv] = self.second_derivatives(u, v)?;
            let (gu, gv) = (r.prod_dot(&su), r.prod_dot(&sv));
            let (a11, a12, a22) = (su.prod_dot(&su), su.prod_dot(&sv), sv.prod_dot(&sv));
            let (h11, h12, h22) = (
                a11.add(r.prod_dot(&suu)),
                a12.add(r.prod_dot(&suv)),
                a22.add(r.prod_dot(&svv)),
            );
            let solve = |m11: S, m12: S, m22: S| -> GeopResult<(S, S)> {
                let det = m11.mul(m22).sub(m12.mul(m12));
                Ok((
                    gv.mul(m12).sub(gu.mul(m22)).div(det)?,
                    gu.mul(m12).sub(gv.mul(m11)).div(det)?,
                ))
            };
            let (du, dv) = if positive(h11, h12, h22) {
                solve(h11, h12, h22)?
            } else if positive(a11, a12, a22) {
                solve(a11, a12, a22)?
            } else if a11.definitely_greater(S::ZERO) {
                (gu.neg().div(a11)?, S::ZERO)
            } else if a22.definitely_greater(S::ZERO) {
                (S::ZERO, gv.neg().div(a22)?)
            } else {
                break;
            };
            let mut scale = S::ONE;
            let mut moved = false;
            for _ in 0..HALVINGS {
                let nu = clamp(u.add(du.mul(scale)).sharpen(), u_lo, u_hi);
                let nv = clamp(v.add(dv.mul(scale)).sharpen(), v_lo, v_hi);
                let d = distance(nu, nv)?;
                if d < current {
                    (u, v, current, moved) = (nu, nv, d, true);
                    break;
                }
                scale = scale.div(S::TWO)?;
            }
            if !moved {
                break;
            }
        }
        Ok((u, v))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        scalars::{ScalInF64 as S, Scalar},
        vector::{Vector3, Vector4},
    };

    use crate::nurb_surface::NurbSurface3D;

    /// A quarter of a cylinder of radius 1 about z, from +x to +y, from
    /// z = 0 to 2: degree 2 around, 1 along.
    fn quarter_cylinder() -> NurbSurface3D<S> {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let h = |x: f64, y: f64, z: f64, w: f64| {
            Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
        };
        let mut points = Vec::new();
        for (x, y, wt) in [(1.0, 0.0, 1.0), (1.0, 1.0, w), (0.0, 1.0, 1.0)] {
            for z in [0.0, 2.0] {
                points.push(h(x, y, z, wt));
            }
        }
        let knots = |k: &[f64]| k.iter().map(|&k| S::from_f64(k)).collect();
        NurbSurface3D::try_new(
            2,
            1,
            points,
            knots(&[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
            knots(&[0.0, 0.0, 1.0, 1.0]),
        )
        .unwrap()
    }

    /// The nearest point of a cylinder lies on the radius through the
    /// target.
    #[test]
    fn nearest_point_of_a_cylinder() {
        let surface = quarter_cylinder();
        let target = Vector3::from_array([2.0, 0.5, 0.5].map(S::from_f64));
        let (_, _, p) = surface.closest_point(&target).unwrap();
        let r = 4.25f64.sqrt();
        let expected = Vector3::from_array([2.0 / r, 0.5 / r, 0.5].map(S::from_f64));
        assert!(p.sub(&expected).norm().to_f64() < 1e-12, "{p:?}");
    }
}
