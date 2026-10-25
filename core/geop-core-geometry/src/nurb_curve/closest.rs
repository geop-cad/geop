//! [`NurbCurve::closest_point`]: the point of a 3-D curve nearest a given
//! point.

use geop_core_math::{
    geop_error::GeopResult, scalars::Scalar, solvers::minimize::descend, vector::Vector3,
};

use super::NurbCurve;
use crate::{
    nearest::{CurveSpan, PointPiece, least_distance},
    search::{MAX_NODES, NEAREST_HANDOFF},
};

/// Newton steps from a seed. Bounds effort only: what is returned is a
/// point of the curve, however many steps it took.
const NEWTON_STEPS: usize = 30;

/// How often a step that does not bring the point nearer is halved before
/// the search stops.
const HALVINGS: usize = 10;

impl<S: Scalar> NurbCurve<S, 4> {
    /// The point of the curve nearest `target`, with its parameter.
    ///
    /// Subdivision isolates, Newton refines: [`least_distance`] bounds the
    /// distance over every span of the curve, so no dip of it is missed,
    /// and Newton on `(C(t) - target) · C'(t) = 0` refines each span left
    /// at the handoff, kept inside the domain — which also reaches an end
    /// of the curve, where the nearest point is one. The parameter is a
    /// free choice of which point of the curve is returned, so every
    /// iterate is sharpened: the point returned lies on the curve, and its
    /// distance to `target` is attained.
    pub fn closest_point(&self, target: &Vector3<S>) -> GeopResult<(S, Vector3<S>)> {
        let nearest = least_distance(
            vec![PointPiece(*target)],
            vec![CurveSpan(self.clone())],
            self.size()?.mul(S::from_f64(NEAREST_HANDOFF)),
            MAX_NODES,
        )?;
        let distance = |p: &Vector3<S>| p.sub(target).norm_sq().to_f64();
        let (mut t, mut p) = nearest.b;
        for ((), seed) in nearest.candidates {
            let refined = self.closest_parameter_from(target, seed)?;
            let q = self.evaluate(refined)?;
            if distance(&q) < distance(&p) {
                (t, p) = (refined, q);
            }
        }
        Ok((t, p))
    }

    /// The parameter of the point of the curve nearest `target` around the
    /// seed `t`, sharp: the local search [`NurbCurve::closest_point`]
    /// refines each of its seeds with, for a caller that already knows
    /// where to look.
    ///
    /// Damped Newton on `f(t) = (C(t) - target) · C'(t)` from `t`: the full
    /// derivative `f'` where it is definitely positive — near the nearest
    /// point — and `|C'|²` where not (Gauss–Newton, a descent direction
    /// wherever the curve is regular). A step that does not bring the point
    /// nearer is halved, and the search stops once halving does not help.
    /// Every iterate is sharpened — which point is returned is a free
    /// choice — and clamped into the domain.
    pub fn closest_parameter_from(&self, target: &Vector3<S>, t: S) -> GeopResult<S> {
        let (t0, t1) = self.domain();
        descend(
            t,
            NEWTON_STEPS,
            HALVINGS,
            |&t| -> GeopResult<f64> { Ok(self.evaluate(t)?.sub(target).norm_sq().to_f64()) },
            |&t| -> GeopResult<_> {
                let r = self.evaluate(t)?.sub(target);
                let d1 = self.tangent(t)?;
                let d2 = self.second_derivative(t)?;
                let f = r.prod_dot(&d1);
                let speed = d1.prod_dot(&d1);
                let full = speed.add(r.prod_dot(&d2));
                let slope = if full.definitely_greater(S::ZERO) {
                    full
                } else if speed.definitely_greater(S::ZERO) {
                    speed
                } else {
                    return Ok(None);
                };
                let step = f.div(slope)?;
                Ok(Some(move |scale: S| {
                    t.sub(step.mul(scale)).sharpen().clamped(t0, t1)
                }))
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        scalars::{Ring, ScalInF64 as S, Scalar},
        vector::{Vector3, Vector4},
    };

    use crate::nurb_curve::NurbCurve;

    /// A quarter circle of radius 2 about the origin, in the xy plane.
    fn quarter() -> NurbCurve<S, 4> {
        let w = S::from_f64(std::f64::consts::FRAC_1_SQRT_2);
        let p = |x: f64, y: f64, w: S| {
            Vector4::from_array([S::from_f64(x).mul(w), S::from_f64(y).mul(w), S::ZERO, w])
        };
        NurbCurve::try_new(
            2,
            vec![p(2.0, 0.0, S::ONE), p(2.0, 2.0, w), p(0.0, 2.0, S::ONE)],
            vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// The nearest point of an arc to a point off it lies on the ray from
    /// the centre through that point; beyond the arc's end, it is the end.
    #[test]
    fn nearest_points_of_an_arc() {
        let arc = quarter();
        let target = Vector3::from_array([3.0, 3.0, 1.0].map(S::from_f64));
        let (_, p) = arc.closest_point(&target).unwrap();
        let expected = Vector3::from_array([2f64.sqrt(), 2f64.sqrt(), 0.0].map(S::from_f64));
        assert!(p.sub(&expected).norm().to_f64() < 1e-12, "{p:?}");

        let beyond = Vector3::from_array([3.0, -1.0, 0.0].map(S::from_f64));
        let (t, p) = arc.closest_point(&beyond).unwrap();
        assert!(t.could_be_equal(S::ZERO));
        assert!(p.could_be_equal(&Vector3::from_array([2.0, 0.0, 0.0].map(S::from_f64))));
    }
}
