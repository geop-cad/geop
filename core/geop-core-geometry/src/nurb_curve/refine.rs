use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

use super::NurbCurve;

/// Gauss-Newton foot-point iterations. Quadratic convergence means a handful
/// is plenty; unlike `max_nodes` this cannot change whether a correct answer
/// is found, only how tightly an already-isolated one is pinned down.
const ITERATIONS: usize = 12;

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Refine `t` — a parameter enclosure produced by a subdivision search —
    /// into the tightest enclosure of the parameter at which this curve
    /// passes through `point`.
    ///
    /// # Why this exists
    ///
    /// Subdivision is a *global* method: it reliably finds and separates
    /// every solution, and (via the leaf-count signal the intersection
    /// searches rely on) recognizes coincidence even for a partial overlap.
    /// What it is bad at is the last few digits — it converges one bit per
    /// split, so squeezing a parameter down to machine accuracy would take
    /// ~50 levels, which is exponentially more work than the ~7 needed to
    /// isolate the solution in the first place.
    ///
    /// Newton is the opposite: useless for finding solutions, unbeatable for
    /// polishing one that is already isolated, converging quadratically. So
    /// the two compose — subdivide to isolate, then refine here.
    ///
    /// This is what lets [`NurbCurve::split`] be called without sharpening.
    /// A `min_subdivision_size`-wide `t` cannot be fed to Boehm insertion:
    /// its width flows into `alpha = (t - e) / (s - e)`, whose denominator
    /// shrinks with every successive split while the width does not, so the
    /// sub-curves' control points widen without bound. The old answer was to
    /// `sharpen` the parameter at each call site, which moved the split to
    /// the interval's midpoint rather than the point actually located — a
    /// silent geometric error of `|t_mid - t*| x |C'(t)|`, and the reason an
    /// edge endpoint could land ~1e-8 from the vertex it is anchored to. A
    /// refined parameter is narrow *and* still an honest enclosure, so it
    /// needs no sharpening and introduces no such error.
    ///
    /// # Honesty of the result
    ///
    /// Every iterate except the last is sharpened, which is legitimate: it is
    /// only a seed for the next step, and any value inside it is an equally
    /// good one. The final step is left unsharpened, so the returned width is
    /// the honest statement of how precisely `point` pins down a parameter
    /// (see "Sharpen only where the value is a free choice" in `AGENTS.md` —
    /// this is exactly the rule `NurbSurface::project` follows).
    ///
    /// The result is then intersected with the incoming `t`: both are valid
    /// enclosures of the same parameter, so their intersection is too, and is
    /// tighter than either. If they turn out to be disjoint, Newton has
    /// wandered out of the box the search proved the solution lies in — the
    /// incoming enclosure is returned unchanged rather than trusting the
    /// refinement. The same fallback covers a vanishing tangent (the
    /// Gauss-Newton denominator could be zero), so this never turns a usable
    /// answer into a failure.
    pub fn refine_parameter_at_point<const C: usize>(
        &self,
        t: S,
        point: &Vector<S, C>,
    ) -> GeopResult<S>
    where
        NurbCurve<S, D>: ParameterRefinable<S, C>,
    {
        let (lo, hi) = self.domain();

        let mut current = t.sharpen();
        for iteration in 0..ITERATIONS {
            let position = self.evaluate_cartesian(current)?;
            let tangent = self.tangent_cartesian(current)?;

            // Gauss-Newton on |C(t) - P|^2: the step that zeroes the
            // directional residual along the tangent.
            let residual = position.sub(point);
            let numerator = residual.prod_dot(&tangent);
            let denominator = tangent.prod_dot(&tangent);
            let Ok(step) = numerator.div(denominator) else {
                return Ok(t);
            };

            let next = current.sub(step);
            let next = if iteration + 1 == ITERATIONS {
                next
            } else {
                next.sharpen()
            };
            // Clamp rather than bail. A foot point can legitimately step
            // outside the domain on the way in — bailing there silently
            // returns the wide search parameter, which is indistinguishable
            // from a successful refinement to every caller and reintroduces
            // exactly the fat sub-curves this exists to prevent.
            current = if next.definitely_less(lo) {
                lo
            } else if next.definitely_greater(hi) {
                hi
            } else {
                next
            };
        }

        if !current.could_be_equal(t) {
            return Ok(t);
        }
        Ok(t.intersect(current))
    }
}

/// Bridges a `NurbCurve<S, D>`'s Cartesian evaluation (`D = 4` -> 3-D points,
/// `D = 3` -> 2-D pcurve points) so [`NurbCurve::refine_parameter_at_point`]
/// can be written once for both — stable Rust's const generics cannot express
/// `C = D - 1` directly.
pub trait ParameterRefinable<S: Scalar, const C: usize> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, C>>;
    /// The Cartesian tangent, via each dimension's own `tangent`. Not the
    /// `derivative()` curve: that is the *homogeneous* derivative, whose
    /// weight component is zero for a non-rational curve, so evaluating it
    /// as a rational curve fails outright.
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, C>>;
}

impl<S: Scalar> ParameterRefinable<S, 3> for NurbCurve<S, 4> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, 3>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, 3>> {
        self.tangent(t)
    }
}

impl<S: Scalar> ParameterRefinable<S, 2> for NurbCurve<S, 3> {
    fn evaluate_cartesian(&self, t: S) -> GeopResult<Vector<S, 2>> {
        self.evaluate(t)
    }
    fn tangent_cartesian(&self, t: S) -> GeopResult<Vector<S, 2>> {
        self.tangent(t)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64) -> Vector4<S> {
        Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
    }

    /// Degree-2 arc, so the parametrization is genuinely nonlinear.
    fn arc<S: Scalar>() -> NurbCurve<S, 4> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![pt(0.0, 0.0, 0.0), pt(1.0, 2.0, 0.0), pt(2.0, 0.0, 0.0)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A search-width parameter refines to a far tighter one that still
    /// encloses the true parameter, and still lands on the same point.
    fn check_refines_a_wide_parameter<S: Scalar>() {
        let curve = arc::<S>();
        let exact = S::from_f64(0.375);
        let target = curve.evaluate(exact).unwrap();

        // What a 1e-4 subdivision search would hand back.
        let wide = S::from_f64(0.3745).union(S::from_f64(0.3755));
        let refined = curve.refine_parameter_at_point(wide, &target).unwrap();

        assert!(
            refined.could_be_equal(exact),
            "refined parameter must still enclose the true one: {refined:?}"
        );
        let landed = curve.evaluate(refined).unwrap();
        for c in 0..3 {
            assert!(landed[c].could_be_equal(target[c]), "coord {c} moved");
        }
    }
    #[test]
    fn refines_a_wide_parameter() {
        for_all_scalars!(check_refines_a_wide_parameter);
    }

    /// A point nowhere near the curve must not drag the parameter somewhere
    /// arbitrary — the incoming enclosure comes back untouched.
    fn check_off_curve_point_falls_back<S: Scalar>() {
        let curve = arc::<S>();
        let target = geop_core_math::vector::Vector3::from_array([
            S::from_f64(50.0),
            S::from_f64(50.0),
            S::from_f64(50.0),
        ]);
        let wide = S::from_f64(0.3745).union(S::from_f64(0.3755));
        let refined = curve.refine_parameter_at_point(wide, &target).unwrap();
        assert!(refined.could_be_equal(wide));
    }
    #[test]
    fn off_curve_point_falls_back() {
        for_all_scalars!(check_off_curve_point_falls_back);
    }
}
