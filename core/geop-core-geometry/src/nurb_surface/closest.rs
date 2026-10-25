//! [`NurbSurface::closest_point`]: the point of a surface nearest a given
//! point.

use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    solvers::{linear::solve_symmetric2, minimize::descend},
    vector::Vector3,
};

use super::NurbSurface;
use crate::{
    nearest::{PointPiece, SurfacePatch, least_distance},
    search::{MAX_NODES, NEAREST_HANDOFF},
};

/// Newton steps from a seed, at most. Bounds effort only: every iterate is
/// a point of the surface nearer than the one before.
const NEWTON_STEPS: usize = 40;

/// How often a step that does not bring the point nearer is halved before
/// the search stops.
const HALVINGS: usize = 10;

impl<S: Scalar> NurbSurface<S, 4> {
    /// The point of the (untrimmed) surface nearest `target`, with its
    /// parameters.
    ///
    /// Subdivision isolates, Newton refines: [`least_distance`] bounds the
    /// distance over every patch of the surface, so no dip of it is missed,
    /// and each patch left at the handoff is refined by damped Newton (see
    /// [`NurbSurface::refine_closest`]). The parameters are a free choice
    /// of which point of the surface is returned, so every iterate is
    /// sharp: the point returned lies on the surface, and its distance to
    /// `target` is attained.
    pub fn closest_point(&self, target: &Vector3<S>) -> GeopResult<(S, S, Vector3<S>)> {
        let nearest = least_distance(
            vec![PointPiece(*target)],
            vec![SurfacePatch(self.clone())],
            self.size()?.mul(S::from_f64(NEAREST_HANDOFF)),
            MAX_NODES,
        )?;
        let distance = |p: &Vector3<S>| p.sub(target).norm_sq().to_f64();
        let ((mut u, mut v), mut p) = nearest.b;
        for ((), (su, sv)) in nearest.candidates {
            let (ru, rv) = self.refine_closest(target, su, sv)?;
            let q = self.evaluate(ru, rv)?;
            if distance(&q) < distance(&p) {
                (u, v, p) = (ru, rv, q);
            }
        }
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
    pub fn refine_closest(&self, target: &Vector3<S>, u: S, v: S) -> GeopResult<(S, S)> {
        let ((u_lo, u_hi), (v_lo, v_hi)) = (self.domain_u(), self.domain_v());
        let positive = |a: S, b: S, c: S| {
            a.definitely_greater(S::ZERO) && a.mul(c).sub(b.mul(b)).definitely_greater(S::ZERO)
        };
        descend(
            (u, v),
            NEWTON_STEPS,
            HALVINGS,
            |&(u, v)| -> GeopResult<f64> {
                Ok(self.evaluate(u, v)?.sub(target).norm_sq().to_f64())
            },
            |&(u, v)| -> GeopResult<_> {
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
                let downhill = (gu.neg(), gv.neg());
                let (du, dv) = if positive(h11, h12, h22) {
                    solve_symmetric2((h11, h12, h22), downhill)?
                } else if positive(a11, a12, a22) {
                    solve_symmetric2((a11, a12, a22), downhill)?
                } else if a11.definitely_greater(S::ZERO) {
                    (gu.neg().div(a11)?, S::ZERO)
                } else if a22.definitely_greater(S::ZERO) {
                    (S::ZERO, gv.neg().div(a22)?)
                } else {
                    return Ok(None);
                };
                Ok(Some(move |scale: S| {
                    (
                        u.add(du.mul(scale)).sharpen().clamped(u_lo, u_hi),
                        v.add(dv.mul(scale)).sharpen().clamped(v_lo, v_hi),
                    )
                }))
            },
        )
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
