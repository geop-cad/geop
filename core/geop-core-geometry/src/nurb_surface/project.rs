use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    solvers::{
        iterate::{End, Repeat, Step, iterate},
        linear::solve_symmetric2,
    },
    vector::Vector3,
};

use super::NurbSurface;
use crate::{contains::surface::surface_could_contain, search::PROJECT_ITERATIONS};

/// How often [`NurbSurface::enclose_foot`] grows its box before it gives
/// up: how hard it tries, not what a box it proves means.
const TRIES: usize = 6;

/// The foot point of a moving point (see [`NurbSurface::foot_moving`]):
/// its `(u, v)`, the point, its velocity, the surface's unit normal there
/// and how fast that turns — each over the box the foot was proven in.
#[derive(Clone, Debug)]
pub struct MovingFoot {
    pub at: (Scalar, Scalar),
    pub point: Vector3,
    pub velocity: Vector3,
    pub normal: Vector3,
    pub turning: Vector3,
}

impl NurbSurface<4> {
    /// Fixed-iteration-count Newton foot-point projection of `target` onto
    /// this surface, starting from `(u0, v0)`.
    ///
    /// Each step is Newton's on the squared distance: it solves the 2x2
    /// system `H·Δ = r` where `r = (residual·Su, residual·Sv)` and `H` its
    /// Hessian, `[[Su·Su − residual·Suu, Su·Sv − residual·Suv], [·, Sv·Sv −
    /// residual·Svv]]`, then clamps `(u, v)` into `domain_u()` /
    /// `domain_v()` before the next iteration. Where `H` is not definitely
    /// positive definite — far from the foot point, or by a pole — the step
    /// is Gauss-Newton's, `H` without the second derivatives (the first
    /// fundamental form). Without them always, the step over a curved
    /// surface overshoots by the target's distance over the surface's
    /// radius of curvature: a target twice the radius off a cylinder, a
    /// ball rounding a fill of its own radius, swung from side to side of
    /// its foot point for ever. Runs `iterations` times with no
    /// convergence tolerance — interval scalars can't judge "close enough" —
    /// but stops early, with the identical result, once the sharpened
    /// iterate repeats one before it exactly — a fixed point, or a cycle in
    /// the last bit ([`iterate`]).
    ///
    /// `J` is singular exactly where the surface's own parametrization is —
    /// a coordinate-singular pole (e.g. the apex of a `revolve`d disk cap,
    /// where every `v` collapses to one point and `Sv = 0`). There only the
    /// collapsed parameter stops moving the point; the other still does, so
    /// that iteration takes the Newton step along it alone — down the
    /// meridian, off the pole, after which the full step takes over. A seed
    /// on the pole would otherwise never leave it, whatever the target. For
    /// a target at the pole that step is zero, and `(u, v)` stays. Where
    /// neither derivative is definitely nonzero, no step is taken, and
    /// `(u, v)` is the seed the previous iteration left.
    pub fn project(
        &self,
        target: Vector3,
        u0: Scalar,
        v0: Scalar,
        iterations: usize,
    ) -> GeopResult<(Scalar, Scalar)> {
        let (u_lo, u_hi) = self.domain_u();
        let (v_lo, v_hi) = self.domain_v();
        let seed = (u0.clamped(u_lo, u_hi), v0.clamped(v_lo, v_hi));
        // Sharpened every iteration but the last: Newton is iterative
        // *refinement* of a foot point this routine gets to choose, not
        // propagation of a measured uncertainty, so any single value inside
        // the current iterate is an equally valid starting point for the
        // next step (see `Scalar::sharpen`). Left unsharpened, each step's
        // `du`/`dv` widens `(u, v)` further; `clamped` cannot pull that
        // back, since an interval merely *straddling* a domain bound is
        // neither definitely inside nor definitely outside it. The widened
        // parameter then reaches `evaluate`, where a wide `t` makes the de
        // Boor weight straddle zero and the whole evaluation fail — even
        // though the seed was sharp and well inside the domain. Sharpen
        // *before* clamping, not after: on a sharp value `clamped`'s
        // three-valued comparisons are exact, so the result is guaranteed
        // inside `[lo, hi]`.
        //
        // The *last* iteration's answer is not sharpened: it is not a seed
        // for anything. `du`/`dv` inherit the width of `target` (through
        // `r`), so that final width is the honest statement of how
        // precisely an uncertain target pins down a foot point. Sharpening
        // it away would return a single sharp `(u, v)` for a target that
        // only ever determined a range of them.
        //
        // A seed the sharpened iteration has started from before ends it:
        // most projections from a nearby seed end in a fixed point or a
        // cycle in the last bit, and would run out all their iterations
        // ([`Repeat::Cycle`]).
        let clamp = |(u, v): (Scalar, Scalar)| (u.clamped(u_lo, u_hi), v.clamped(v_lo, v_hi));
        let iterated = iterate(
            seed,
            seed,
            iterations,
            Repeat::Cycle,
            |&(u, v)| (clamp((u.sharpen(), v.sharpen())), clamp((u, v))),
            |&(u, v)| -> GeopResult<Step<(Scalar, Scalar)>> {
                let p = self.evaluate(u, v)?;
                let (su, sv) = self.derivatives(u, v)?;
                let r = target.sub(&p);

                let a11 = su.prod_dot(&su);
                let a12 = su.prod_dot(&sv);
                let a22 = sv.prod_dot(&sv);
                let b1 = su.prod_dot(&r);
                let b2 = sv.prod_dot(&r);

                // Newton's step where the Hessian is positive definite.
                let [suu, suv, svv] = self.second_derivatives(u, v)?;
                let (h11, h12, h22) = (
                    a11.sub(r.prod_dot(&suu)),
                    a12.sub(r.prod_dot(&suv)),
                    a22.sub(r.prod_dot(&svv)),
                );
                let definite = h11.definitely_greater(Scalar::ZERO)
                    && h11
                        .mul(h22)
                        .sub(h12.mul(h12))
                        .definitely_greater(Scalar::ZERO);
                if definite && let Ok((du, dv)) = solve_symmetric2((h11, h12, h22), (b1, b2)) {
                    return Ok(Step::Next((u.add(du), v.add(dv))));
                }

                let (du, dv) = match solve_symmetric2((a11, a12, a22), (b1, b2)) {
                    Ok(step) => step,
                    // Singular: along the one derivative that moves the point.
                    _ if a22.definitely_greater(Scalar::ZERO) => {
                        match self.off_pole(target, &p, (v, v_lo, v_hi), (u, u_lo, u_hi), false)? {
                            Some((along, d)) => (along.sub(u), d),
                            None => return Ok(Step::Stuck),
                        }
                    }
                    _ if a11.definitely_greater(Scalar::ZERO) => {
                        match self.off_pole(target, &p, (u, u_lo, u_hi), (v, v_lo, v_hi), true)? {
                            Some((along, d)) => (d, along.sub(v)),
                            None => return Ok(Step::Stuck),
                        }
                    }
                    _ => return Ok(Step::Stuck),
                };
                Ok(Step::Next((u.add(du), v.add(dv))))
            },
        )?;
        // A step that cannot be taken cannot be taken from the same seed
        // later either: the seed is where the iteration stays.
        Ok(match iterated.end {
            End::Stuck => iterated.seed,
            End::Done | End::Exhausted => iterated.answer,
        })
    }

    /// Where `point`, a point of this surface, lies on it: its `(u, v)`, or
    /// `None` if it is not on the surface.
    ///
    /// Subdivide to isolate, then Newton to refine (see `AGENTS.md`): the
    /// containment search ([`surface_could_contain`], with `max_nodes` and
    /// `min_subdivision_size` bounding its effort) proves the point lies in
    /// a box only `min_subdivision_size` narrow, and [`Self::project`] from
    /// the box's midpoint polishes that to what the point's own width
    /// allows. Both enclose the same foot point, so the answer is their
    /// intersection. The refinement only tightens: where Newton fails or
    /// leaves the box, the box is the answer.
    ///
    /// A point that could be a pole lies on the row collapsing to it (see
    /// AGENTS.md, "A pole is one vertex at many `(u, v)`"), whatever Newton
    /// settled on beside it — where the Jacobian is singular, it stops
    /// where it can step no further, a hair off the row: the row's
    /// parameter is kept as one of the point's too.
    pub fn locate(
        &self,
        point: &Vector3,
        max_nodes: usize,
        min_subdivision_size: Scalar,
    ) -> GeopResult<Option<(Scalar, Scalar)>> {
        let Some((u, v)) = surface_could_contain(self, point, max_nodes, min_subdivision_size)?
        else {
            return Ok(None);
        };
        let (mut lu, mut lv) =
            match self.project(*point, u.sharpen(), v.sharpen(), PROJECT_ITERATIONS) {
                Ok((ru, rv)) if ru.could_be_equal(u) && rv.could_be_equal(v) => {
                    (u.intersect(ru), v.intersect(rv))
                }
                _ => (u, v),
            };
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        for (u_fixed, first) in self.rows_touching(u, v) {
            let Some((pole, _)) = self.pole(u_fixed, first) else {
                continue;
            };
            if !point.could_be_equal(&pole) {
                continue;
            }
            match (u_fixed, first) {
                (true, true) => lu = lu.union(u0),
                (true, false) => lu = lu.union(u1),
                (false, true) => lv = lv.union(v0),
                (false, false) => lv = lv.union(v1),
            }
        }
        Ok(Some((lu, lv)))
    }

    /// The `(u, v)` of the foot point of every point of the box `target`
    /// near `near`, proven: a box in which each of those points has one
    /// foot point and no other, `None` where that cannot be shown.
    ///
    /// A foot point is where the gradient of half the squared distance,
    /// `g = (Su·(S − P), Sv·(S − P))`, vanishes. Krawczyk's operator
    /// `K(X) = c − Y g(c) + (I − Y J(X)) (X − c)`, with `J` the Hessian of
    /// the distance over the box `X` (and every `P` of `target`), `c` its
    /// middle and `Y` the inverse of the Hessian at `c` — a preconditioner,
    /// a free choice — holds every zero of `g` in `X`; where it lies within
    /// `X`, there is exactly one, and it is in `K(X)` too. `X` starts as
    /// twice the Newton step from `near` and grows where `K` reaches past
    /// it, a few times: how hard the search tries. It must keep within the
    /// domain, so a target whose foot is on the domain's edge needs the
    /// surface carried on past it ([`Self::extended`]).
    ///
    /// Over a box of targets along a curve on the surface, the box found
    /// holds the curve's `(u, v)` over that stretch: what the normal and
    /// its turn are enclosed over (see [`Self::normal_turning`]).
    pub fn enclose_foot(
        &self,
        target: &Vector3,
        near: (Scalar, Scalar),
    ) -> GeopResult<Option<(Scalar, Scalar)>> {
        // The gradient over `(u, v)` and its Hessian, both over the target.
        let at = |u: Scalar, v: Scalar| -> GeopResult<([Scalar; 2], [Scalar; 3])> {
            let s = self.partials(u, v, 2)?;
            let r = s[0][0].sub(target);
            let (su, sv) = (s[1][0], s[0][1]);
            Ok((
                [su.prod_dot(&r), sv.prod_dot(&r)],
                [
                    su.prod_dot(&su).add(s[2][0].prod_dot(&r)),
                    su.prod_dot(&sv).add(s[1][1].prod_dot(&r)),
                    sv.prod_dot(&sv).add(s[0][2].prod_dot(&r)),
                ],
            ))
        };
        // About `near` first, and where that is too far for the Hessian to
        // stay near its value there, about the foot of the target's middle,
        // Newton's from `near`: a free choice of where the box is, which
        // the operator then proves or not.
        if let Some(feet) = self.krawczyk_foot(near, &at)? {
            return Ok(Some(feet));
        }
        let middle = Vector3::from_array(target.to_array().map(Scalar::midpoint));
        let (u, v) = self.project(
            middle,
            near.0.midpoint(),
            near.1.midpoint(),
            PROJECT_ITERATIONS,
        )?;
        self.krawczyk_foot((u, v), &at)
    }

    /// The foot point of a point moving through the box `target` at
    /// `velocity`, near `near`, proven as [`Self::enclose_foot`] proves it,
    /// and how it moves: `None` where that cannot be shown.
    ///
    /// The foot keeps the gradient `g` of half the squared distance zero,
    /// so `H (u', v') = (Su·P', Sv·P')` for the Hessian `H` of that
    /// distance: the foot moves at `Su u' + Sv v'`, and the normal there
    /// turns as [`Self::normal_turning`] says for that motion. Over the box
    /// of feet, every one an enclosure over the whole box of targets.
    pub fn foot_moving(
        &self,
        target: &Vector3,
        velocity: &Vector3,
        near: (Scalar, Scalar),
    ) -> GeopResult<Option<MovingFoot>> {
        let Some((u, v)) = self.enclose_foot(target, near)? else {
            return Ok(None);
        };
        let s = self.partials(u, v, 2)?;
        let r = s[0][0].sub(target);
        let (su, sv) = (s[1][0], s[0][1]);
        let hessian = (
            su.prod_dot(&su).add(s[2][0].prod_dot(&r)),
            su.prod_dot(&sv).add(s[1][1].prod_dot(&r)),
            sv.prod_dot(&sv).add(s[0][2].prod_dot(&r)),
        );
        let Ok((du, dv)) =
            solve_symmetric2(hessian, (su.prod_dot(velocity), sv.prod_dot(velocity)))
        else {
            return Ok(None);
        };
        let moving = su.prod_scalar(du).add(&sv.prod_scalar(dv));
        let (normal, turning) = self.normal_turning(u, v, &moving)?;
        Ok(Some(MovingFoot {
            at: (u, v),
            point: s[0][0],
            velocity: moving,
            normal,
            turning,
        }))
    }

    /// [`Self::enclose_foot`]'s operator, about `near`, the gradient and its
    /// Hessian over a box `at` gives.
    #[allow(clippy::type_complexity)]
    fn krawczyk_foot(
        &self,
        near: (Scalar, Scalar),
        at: &dyn Fn(Scalar, Scalar) -> GeopResult<([Scalar; 2], [Scalar; 3])>,
    ) -> GeopResult<Option<(Scalar, Scalar)>> {
        let ((u_lo, u_hi), (v_lo, v_hi)) = (self.domain_u(), self.domain_v());
        let mut c = (near.0.midpoint(), near.1.midpoint());
        let (mut gc, h) = at(c.0, c.1)?;
        let [a, b, d] = h.map(|x| x.to_f64());
        let det = a * d - b * b;
        if !det.is_finite() || det == 0.0 {
            return Ok(None);
        }
        // The preconditioner, sharp: `Y = H(c)⁻¹`.
        let y = [[d / det, -b / det], [-b / det, a / det]].map(|row| row.map(Scalar::from_f64));
        let times = |y: &[[Scalar; 2]; 2], g: [Scalar; 2]| {
            [
                y[0][0].mul(g[0]).add(y[0][1].mul(g[1])),
                y[1][0].mul(g[0]).add(y[1][1].mul(g[1])),
            ]
        };
        let step = times(&y, gc);
        let mut radius = step.map(|s| 2.0 * s.abs().upper().to_f64());
        for r in &mut radius {
            *r = r.max(f64::EPSILON);
        }
        for _ in 0..TRIES {
            let x = (
                Scalar::new(c.0.to_f64() - radius[0], c.0.to_f64() + radius[0]),
                Scalar::new(c.1.to_f64() - radius[1], c.1.to_f64() + radius[1]),
            );
            if x.0.could_be_less(u_lo) || x.0.could_be_greater(u_hi) {
                return Ok(None);
            }
            if x.1.could_be_less(v_lo) || x.1.could_be_greater(v_hi) {
                return Ok(None);
            }
            let newton = times(&y, gc);
            let (_, [h11, h12, h22]) = at(x.0, x.1)?;
            // `I − Y J(X)`.
            let m = [
                [
                    Scalar::ONE.sub(y[0][0].mul(h11).add(y[0][1].mul(h12))),
                    y[0][0].mul(h12).add(y[0][1].mul(h22)).neg(),
                ],
                [
                    y[1][0].mul(h11).add(y[1][1].mul(h12)).neg(),
                    Scalar::ONE.sub(y[1][0].mul(h12).add(y[1][1].mul(h22))),
                ],
            ];
            let off = times(&m, [x.0.sub(c.0), x.1.sub(c.1)]);
            let k = (
                c.0.sub(newton[0]).add(off[0]),
                c.1.sub(newton[1]).add(off[1]),
            );
            let inside = |k: Scalar, x: Scalar| {
                k.lower().definitely_greater(x.lower()) && k.upper().definitely_less(x.upper())
            };
            if inside(k.0, x.0) && inside(k.1, x.1) {
                return Ok(Some((k.0.intersect(x.0), k.1.intersect(x.1))));
            }
            // Wider about the middle of what the operator gave.
            c = (k.0.midpoint(), k.1.midpoint());
            gc = at(c.0, c.1)?.0;
            radius = [
                4.0 * radius[0].max(k.0.width().to_f64()),
                4.0 * radius[1].max(k.1.width().to_f64()),
            ];
        }
        Ok(None)
    }

    /// The foot point of `target` as [`Self::project`] finds it, within the
    /// surface's domain: the foot point lies on the patch, so its `(u, v)`
    /// are in the domain as much as where the projection says, and both
    /// enclosures hold at once.
    pub fn project_in_domain(
        &self,
        target: Vector3,
        u0: Scalar,
        v0: Scalar,
        iterations: usize,
    ) -> GeopResult<(Scalar, Scalar)> {
        let (u, v) = self.project(target, u0, v0, iterations)?;
        let ((u_lo, u_hi), (v_lo, v_hi)) = (self.domain_u(), self.domain_v());
        Ok((u.intersect(u_lo.union(u_hi)), v.intersect(v_lo.union(v_hi))))
    }

    /// The step along a meridian at `p`, where the Jacobian is singular,
    /// towards `target`: at a pole, or a hair off one, where the derivative
    /// along the collapsed row is below what the arithmetic resolves. The
    /// collapsed parameter does not move the point there, so which meridian
    /// to go along is a free choice — `collapsed` is its value now and its
    /// domain; `fixed`, the other parameter's value and domain, is where
    /// the pole is. Of the meridians at the collapsed domain's ends, its
    /// middle and where it is now, the one the Newton step along goes
    /// furthest towards the target on: `(meridian, step)`, or `None` if
    /// none moves. `u_collapsed` says the collapsed parameter is `v`.
    ///
    /// The step is kept within the domain, as every step is: a target whose
    /// foot point is the pole itself — behind the patch, on the far side of
    /// the pole from it — is reached by a step onto the pole, exactly.
    /// Refusing that step left the foot point creeping towards the pole and
    /// stopping a hair off it, where the surface's normal, `S_u × S_v` with
    /// `S_u` of the order of that hair, is lost in rounding; the pole itself
    /// has a normal (`pole_normal`). From the pole, a step onto it again is
    /// no step, and a meridian leading away from the target is not taken.
    fn off_pole(
        &self,
        target: Vector3,
        p: &Vector3,
        (fixed, fixed_lo, fixed_hi): (Scalar, Scalar, Scalar),
        (now, lo, hi): (Scalar, Scalar, Scalar),
        u_collapsed: bool,
    ) -> GeopResult<Option<(Scalar, Scalar)>> {
        let r = target.sub(p);
        let middle = Scalar::interpolate(lo, hi, Scalar::from_f64(0.5));
        let mut best: Option<(Scalar, Scalar, f64)> = None;
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
            let to = fixed.add(step).clamped(fixed_lo, fixed_hi);
            let step = to.sub(fixed);
            if step.could_be_equal(Scalar::ZERO) {
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
    use crate::search::PROJECT_ITERATIONS;
    use crate::test_fixtures::{hom, revolved};
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    /// Flat unit patch in the xy-plane.
    fn flat_xy() -> NurbSurface<4> {
        let f = Scalar::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                hom(0., 0., 0., 1.),
                hom(0., 1., 0., 1.),
                hom(1., 0., 0., 1.),
                hom(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    #[test]
    fn project_flat_surface_from_directly_above() {
        let s = flat_xy();
        let target = geop_core_math::vector::Vector3::from_array([
            Scalar::from_f64(0.3),
            Scalar::from_f64(0.7),
            Scalar::from_f64(2.0),
        ]);
        let (u, v) = s
            .project(target, Scalar::from_f64(0.5), Scalar::from_f64(0.5), 5)
            .unwrap();
        assert!(u.could_be_equal(Scalar::from_f64(0.3)));
        assert!(v.could_be_equal(Scalar::from_f64(0.7)));
    }

    /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
    /// x,y ∈ [0,2] — same fixture used by `normal.rs`'s tests.
    fn bent_surface() -> NurbSurface<4> {
        let f = Scalar::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                hom(0., 0., 0., 1.),
                hom(0., 2., 1., 1.),
                hom(2., 0., 1., 1.),
                hom(2., 2., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A target behind a quarter disc's pole, on the far side of it from
    /// the quarter: its foot point is the pole. From a seed a hair off the
    /// pole, where the derivative along the collapsed row is lost in
    /// rounding, the projection lands on the pole exactly — where the
    /// surface has a normal.
    #[test]
    fn project_onto_the_pole_from_a_hair_off_it() {
        let disc = revolved(0., 0.);
        let f = Scalar::from_f64;
        let target = Vector3::from_array([f(-1e-17), f(-8e-14), f(1.)]);
        let (u, v) = disc
            .project_in_domain(target, f(1.), f(2.7755575615628914e-17), PROJECT_ITERATIONS)
            .unwrap();
        assert!(v.could_be_equal(Scalar::ZERO), "{v:?}");
        let normal = disc.normal(u, v).unwrap();
        assert!(normal[2].abs().could_be_equal(Scalar::ONE), "{normal:?}");
    }

    #[test]
    fn project_onto_bent_surface_converges() {
        let s = bent_surface();
        let expected_u = Scalar::from_f64(0.4);
        let expected_v = Scalar::from_f64(0.6);
        let on_surface = s.evaluate(expected_u, expected_v).unwrap();
        let (u, v) = s
            .project(on_surface, Scalar::from_f64(0.5), Scalar::from_f64(0.5), 5)
            .unwrap();
        let result = s.evaluate(u, v).unwrap();
        assert!(result[0].could_be_equal(on_surface[0]));
        assert!(result[1].could_be_equal(on_surface[1]));
        assert!(result[2].could_be_equal(on_surface[2]));
    }

    /// The feet of a stretch of points on a cone, proven to lie in one box
    /// that holds their `(u, v)`; of one point, a box as narrow as the
    /// rounding; and of a point past the domain's edge, none.
    #[test]
    fn enclose_foot_holds_the_feet_of_a_box() {
        let cone = revolved(1., 0.);
        let v = Scalar::from_f64(0.5);
        let stretch = Scalar::new(0.30, 0.32);
        let points = cone.evaluate(stretch, v).unwrap();
        let (u_box, v_box) = cone
            .enclose_foot(&points, (Scalar::from_f64(0.31), v))
            .unwrap()
            .expect("proven");
        assert!(
            stretch.is_subset_of(u_box) && v.is_subset_of(v_box),
            "{u_box:?} {v_box:?}"
        );
        assert!(u_box.width().to_f64() < 0.1 && v_box.width().to_f64() < 0.1);
        let u = Scalar::from_f64(0.31);
        let point = cone.evaluate(u, v).unwrap();
        let (u_box, v_box) = cone
            .enclose_foot(&point, (Scalar::from_f64(0.3), Scalar::from_f64(0.45)))
            .unwrap()
            .expect("proven");
        assert!(
            u.is_subset_of(u_box) && v.is_subset_of(v_box),
            "{u_box:?} {v_box:?}"
        );
        assert!(u_box.width().to_f64() < 1e-12 && v_box.width().to_f64() < 1e-12);
        // On the rim, the box round its foot would leave the domain.
        let rim = cone.evaluate(u, Scalar::ONE).unwrap();
        assert!(cone.enclose_foot(&rim, (u, Scalar::ONE)).unwrap().is_none());
    }
}
