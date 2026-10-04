//! [`NurbCurve::closest_point`]: the point of a 3-D curve nearest a given
//! point.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::NurbCurve;

/// Samples per polynomial piece the search starts from. Bounds effort only:
/// every sample is a point of the curve, and Newton takes the best of them
/// to the nearest point around it.
const SAMPLES_PER_PIECE: usize = 8;

/// Newton steps from a seed. Bounds effort only: what is returned is a
/// point of the curve, however many steps it took.
const NEWTON_STEPS: usize = 30;

/// How many of the nearest samples are refined: neighbouring local minima
/// of the distance are told apart by refining more than one.
const SEEDS: usize = 3;

/// How often a step that does not bring the point nearer is halved before
/// the search stops.
const HALVINGS: usize = 10;

impl<S: Scalar> NurbCurve<S, 4> {
    /// The point of the curve nearest `target`, with its parameter.
    ///
    /// Subdivision isolates, Newton refines: the curve is sampled across
    /// every polynomial piece, and the nearest few samples are refined by
    /// Newton on `(C(t) - target) · C'(t) = 0`, kept inside the domain —
    /// which also reaches an end of the curve, where the nearest point is
    /// one. The parameter is a free choice of which point of the curve is
    /// returned, so every iterate is sharpened: the point returned lies on
    /// the curve, its distance to `target` is attained, and it is the least
    /// distance found — a narrow dip between samples could still be missed,
    /// which more samples, not a tolerance, would fix.
    pub fn closest_point(&self, target: &Vector3<S>) -> GeopResult<(S, Vector3<S>)> {
        let breaks = self.breakpoints();
        let (t0, t1) = self.domain();
        let mut samples = Vec::new();
        for w in breaks.windows(2) {
            for i in 0..SAMPLES_PER_PIECE {
                let alpha = S::from_ratio(i as i64, SAMPLES_PER_PIECE as i64)?;
                samples.push(S::interpolate(w[0], w[1], alpha).sharpen());
            }
        }
        samples.push(t1);
        let distance = |p: &Vector3<S>| p.sub(target).norm_sq().to_f64();
        let mut scored = samples
            .into_iter()
            .map(|t| Ok((distance(&self.evaluate(t)?), t)))
            .collect::<GeopResult<Vec<_>>>()?;
        scored.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut best: Option<(f64, S, Vector3<S>)> = None;
        for &(_, seed) in scored.iter().take(SEEDS) {
            let t = self.refine_closest(target, seed, (t0, t1))?;
            let p = self.evaluate(t)?;
            let d = distance(&p);
            if best.as_ref().is_none_or(|(b, ..)| d < *b) {
                best = Some((d, t, p));
            }
        }
        let (_, t, p) = best.expect("a curve has samples");
        Ok((t, p))
    }

    /// Damped Newton on `f(t) = (C(t) - target) · C'(t)` from `t`: the full
    /// derivative `f'` where it is definitely positive — near the nearest
    /// point — and `|C'|²` where not (Gauss–Newton, a descent direction
    /// wherever the curve is regular). A step that does not bring the point
    /// nearer is halved, and the search stops once halving does not help.
    /// Every iterate is clamped into `(t0, t1)`.
    fn refine_closest(&self, target: &Vector3<S>, mut t: S, (t0, t1): (S, S)) -> GeopResult<S> {
        let clamp = |x: S| crate::nurb_surface::clamp(x, t0, t1);
        let distance = |t: S| -> GeopResult<f64> {
            Ok(self.evaluate(t)?.sub(target).norm_sq().to_f64())
        };
        let mut current = distance(t)?;
        for _ in 0..NEWTON_STEPS {
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
                break;
            };
            let step = f.div(slope)?;
            let mut scale = S::ONE;
            let mut moved = false;
            for _ in 0..HALVINGS {
                let next = clamp(t.sub(step.mul(scale)).sharpen());
                let d = distance(next)?;
                if d < current {
                    (t, current, moved) = (next, d, true);
                    break;
                }
                scale = scale.div(S::TWO)?;
            }
            if !moved {
                break;
            }
        }
        Ok(t)
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
