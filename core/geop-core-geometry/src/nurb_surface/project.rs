use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::NurbSurface;

/// Clamp `x` into `[lo, hi]` using three-valued comparisons.
pub fn clamp<S: Scalar>(x: S, lo: S, hi: S) -> S {
    if x.definitely_less(lo) {
        lo
    } else if x.definitely_greater(hi) {
        hi
    } else {
        x
    }
}

impl<S: Scalar> NurbSurface<S, 4> {
    /// Fixed-iteration-count Newton foot-point projection of `target` onto
    /// this surface, starting from `(u0, v0)`.
    ///
    /// Each step solves the exact 2x2 system `J·Δ = r` where
    /// `r = (residual·Su, residual·Sv)` and `J = [[Su·Su, Su·Sv], [Su·Sv,
    /// Sv·Sv]]` (the first fundamental form — exact for the degree-1
    /// bilinear/low-degree patches used in this crate's tests, since second
    /// derivatives are dropped), then clamps `(u, v)` into `domain_u()` /
    /// `domain_v()` before the next iteration. Runs `iterations` times with no
    /// convergence tolerance — interval scalars can't judge "close enough" —
    /// but stops early, with the identical result, once the sharpened
    /// iterate repeats one before it exactly — a fixed point, or a cycle in
    /// the last bit (see the loop).
    ///
    /// `J` is singular exactly where the surface's own parametrization is —
    /// a coordinate-singular pole (e.g. the apex of a `revolve`d disk cap,
    /// where every `v` collapses to one point and `Sv = 0`). There only the
    /// collapsed parameter stops moving the point; the other still does, so
    /// that iteration takes the Newton step along it alone — down the
    /// meridian, off the pole, after which the full step takes over. A seed
    /// on the pole would otherwise never leave it, whatever the target. For
    /// a target at the pole that step is zero, and `(u, v)` stays. Where
    /// neither derivative is definitely nonzero, the update is skipped,
    /// leaving `(u, v)` where the previous iteration left it.
    pub fn project(
        &self,
        target: Vector3<S>,
        u0: S,
        v0: S,
        iterations: usize,
    ) -> GeopResult<(S, S)> {
        let (u_lo, u_hi) = self.domain_u();
        let (v_lo, v_hi) = self.domain_v();
        let mut u = clamp(u0, u_lo, u_hi);
        let mut v = clamp(v0, v_lo, v_hi);
        // Every iteration's seed and its unsharpened step, in order.
        let mut taken: Vec<((S, S), (S, S))> = Vec::with_capacity(iterations);

        for iteration in 0..iterations {
            let p = self.evaluate(u, v)?;
            let (su, sv) = self.derivatives(u, v)?;
            let r = target.sub(&p);

            let a11 = su.prod_dot(&su);
            let a12 = su.prod_dot(&sv);
            let a22 = sv.prod_dot(&sv);
            let b1 = su.prod_dot(&r);
            let b2 = sv.prod_dot(&r);

            let det = a11.mul(a22).sub(a12.mul(a12));
            let du = b1.mul(a22).sub(b2.mul(a12)).div(det);
            let dv = a11.mul(b2).sub(a12.mul(b1)).div(det);
            let (du, dv) = match (du, dv) {
                (Ok(du), Ok(dv)) => (du, dv),
                // Singular: along the one derivative that moves the point.
                _ if a22.definitely_greater(S::ZERO) => {
                    match self.off_pole(target, &p, (v, v_lo, v_hi), (u, u_lo, u_hi), false)? {
                        Some((along, d)) => (along.sub(u), d),
                        None => continue,
                    }
                }
                _ if a11.definitely_greater(S::ZERO) => {
                    match self.off_pole(target, &p, (u, u_lo, u_hi), (v, v_lo, v_hi), true)? {
                        Some((along, d)) => (d, along.sub(v)),
                        None => continue,
                    }
                }
                _ => continue,
            };

            // Sharpened every iteration, not just at the end: Newton is
            // iterative *refinement* of a foot point this routine gets to
            // choose, not propagation of a measured uncertainty, so any
            // single value inside the current iterate is an equally valid
            // starting point for the next step (see `Scalar::sharpen`).
            // Left unsharpened, each step's `du`/`dv` widens `(u, v)`
            // further; `clamp` cannot pull that back, since an interval
            // merely *straddling* a domain bound is neither definitely
            // inside nor definitely outside it. The widened parameter then
            // reaches `evaluate`, where a wide `t` makes the de Boor
            // weight straddle zero and the whole evaluation fail — even
            // though the seed was sharp and well inside the domain.
            // Sharpen *before* clamping, not after: on a sharp value
            // `clamp`'s three-valued comparisons are exact, so the result
            // is guaranteed inside `[lo, hi]`. The other order can escape
            // the domain again — collapsing an interval that straddles a
            // bound to its midpoint can land just past that bound, and the
            // next `evaluate` then rejects it as out of domain.
            //
            // The *last* iteration is the exception: its result is not a
            // seed for anything, it is the answer this function returns.
            // `du`/`dv` inherit the width of `target` (through `r`), so
            // that final width is the honest statement of how precisely an
            // uncertain target pins down a foot point. Sharpening it away
            // would return a single sharp `(u, v)` for a target that only
            // ever determined a range of them — an answer claiming more
            // precision than the input carried, which then fails to agree
            // with anything else derived from the same uncertain point.
            let (next_u, next_v) = (u.add(du), v.add(dv));
            if iteration + 1 == iterations {
                return Ok((clamp(next_u, u_lo, u_hi), clamp(next_v, v_lo, v_hi)));
            }
            let (sharp_u, sharp_v) = (
                clamp(next_u.sharpen(), u_lo, u_hi),
                clamp(next_v.sharpen(), v_lo, v_hi),
            );
            // A seed the sharpened iteration has started from before: each
            // iterate is a deterministic function of the sharp seed before
            // it, so from there on the seeds repeat with that period —
            // a fixed point, or rounding in the last bit cycling between
            // two or three. The last iteration's seed is then known, and its
            // unsharpened step is one already taken: taking it now returns
            // the identical result sooner. Not a convergence tolerance —
            // nothing is judged "close enough", the seed is bit-for-bit one
            // seen before. (Most projections from a nearby seed ended in
            // such a cycle and ran out all their iterations.)
            let same = |a: S, b: S| a.is_subset_of(b) && b.is_subset_of(a);
            taken.push(((u, v), (next_u, next_v)));
            if let Some(k) = taken
                .iter()
                .position(|&((su, sv), _)| same(su, sharp_u) && same(sv, sharp_v))
            {
                let period = taken.len() - k;
                let (_, (last_u, last_v)) = taken[k + (iterations - 1 - k) % period];
                return Ok((clamp(last_u, u_lo, u_hi), clamp(last_v, v_lo, v_hi)));
            }
            u = sharp_u;
            v = sharp_v;
        }

        Ok((u, v))
    }

    /// The step off a pole at `p` towards `target`. The collapsed parameter
    /// does not move the point there, so which meridian to leave along is a
    /// free choice — `collapsed` is its value now and its domain; `fixed`,
    /// the other parameter's value and domain, is where the pole is. Of the
    /// meridians at the collapsed domain's ends, its middle and where it is
    /// now, the one the Newton step along goes furthest towards the target
    /// on — into the domain, off the pole: `(meridian, step)`, or `None`
    /// if none does. `u_collapsed` says the collapsed parameter is `v`.
    fn off_pole(
        &self,
        target: Vector3<S>,
        p: &Vector3<S>,
        (fixed, fixed_lo, fixed_hi): (S, S, S),
        (now, lo, hi): (S, S, S),
        u_collapsed: bool,
    ) -> GeopResult<Option<(S, S)>> {
        let r = target.sub(p);
        let middle = S::interpolate(lo, hi, S::from_f64(0.5));
        let mut best: Option<(S, S, f64)> = None;
        for along in [now, lo, middle, hi] {
            let (su, sv) = if u_collapsed {
                self.derivatives(fixed, along)?
            } else {
                self.derivatives(along, fixed)?
            };
            let meridian = if u_collapsed { su } else { sv };
            let toward = meridian.prod_dot(&r);
            let Ok(step) = toward.div(meridian.prod_dot(&meridian)) else {
                continue;
            };
            // Off the pole: the step leads into the domain.
            let to = fixed.add(step);
            if !(to.definitely_greater(fixed_lo) && to.definitely_less(fixed_hi)) {
                continue;
            }
            // How far towards the target, to first order: `toward * step`.
            let progress = toward.mul(step).midpoint().to_f64();
            if best.as_ref().is_none_or(|(_, _, b)| progress > *b) {
                best = Some((along, step, progress));
            }
        }
        Ok(best.map(|(along, step, _)| (along, step)))
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_surface::NurbSurface;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    /// Flat unit patch in the xy-plane.
    fn flat_xy<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_project_flat_surface_from_directly_above<S: Scalar>() {
        let s = flat_xy::<S>();
        let target = geop_core_math::vector::Vector3::from_array([
            S::from_f64(0.3),
            S::from_f64(0.7),
            S::from_f64(2.0),
        ]);
        let (u, v) = s
            .project(target, S::from_f64(0.5), S::from_f64(0.5), 5)
            .unwrap();
        assert!(u.could_be_equal(S::from_f64(0.3)));
        assert!(v.could_be_equal(S::from_f64(0.7)));
    }
    #[test]
    fn project_flat_surface_from_directly_above() {
        for_all_scalars!(check_project_flat_surface_from_directly_above);
    }

    /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
    /// x,y ∈ [0,2] — same fixture used by `normal.rs`'s tests.
    fn bent_surface<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 2., 1., 1.),
                pt(2., 0., 1., 1.),
                pt(2., 2., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_project_onto_bent_surface_converges<S: Scalar>() {
        let s = bent_surface::<S>();
        let expected_u = S::from_f64(0.4);
        let expected_v = S::from_f64(0.6);
        let on_surface = s.evaluate(expected_u, expected_v).unwrap();
        let (u, v) = s
            .project(on_surface, S::from_f64(0.5), S::from_f64(0.5), 5)
            .unwrap();
        let result = s.evaluate(u, v).unwrap();
        assert!(result[0].could_be_equal(on_surface[0]));
        assert!(result[1].could_be_equal(on_surface[1]));
        assert!(result[2].could_be_equal(on_surface[2]));
    }
    #[test]
    fn project_onto_bent_surface_converges() {
        for_all_scalars!(check_project_onto_bent_surface_converges);
    }
}
