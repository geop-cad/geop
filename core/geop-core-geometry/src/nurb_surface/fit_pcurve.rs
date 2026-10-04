use crate::{
    contains::surface::surface_could_contain,
    nurb_curve::{NurbCurve, NurbCurve2D, true_point_fractions},
    spline::interior_knots,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
};

use super::NurbSurface;

/// How many intervals a curve is split into for projection to start with,
/// shared out among its smooth pieces by their share of its domain (but at
/// least [`MIN_PIECE_SAMPLES`] each). Each piece's pcurve passes exactly
/// through its samples and is widened to enclose the trace between them
/// (see `NurbCurve2D::interpolate_enclosing`).
///
/// A cubic interpolant's drift falls as `h^4` on a smooth trace, and since
/// the pcurve carries that drift as width, this decides how *wide* the
/// pcurve is, not whether it is right. At 9 samples the drift reached ~2e-5
/// on strongly curved patches; 48 keeps it well inside the accuracy
/// `validation::numerical_accuracy` holds every entity to. Where it does
/// not, a piece is sampled more densely (see [`DRIFT_PER_RESOLUTION`]).
///
/// This bounds effort, not correctness: each sample is one Newton foot-point
/// projection (plus one per true point between samples, see
/// `true_point_fractions`), and a pcurve fitted through more of them is
/// strictly narrower.
const SAMPLES: usize = 48;

/// The fewest intervals a smooth piece of a curve is sampled in.
const MIN_PIECE_SAMPLES: usize = 8;

/// The most intervals a smooth piece of a curve is sampled in. A pcurve
/// still wider than the target then is as wide as its samples (the trace is
/// known no better), or its trace is not smooth where the curve's knots say
/// nothing (a seam of the surface): it is returned as it is, wide and
/// honest.
const MAX_PIECE_SAMPLES: usize = 1536;

/// A piece is sampled twice as densely while its pcurve is wider than
/// `min_subdivision_size` over this: a width that small is lost in any
/// search resolving to that size, which is what every question later asked
/// of the pcurve is. Like `min_subdivision_size` itself, it decides effort,
/// not correctness: the pcurve encloses its trace however wide it is.
const DRIFT_PER_RESOLUTION: i64 = 16;

impl<S: Scalar> NurbSurface<S, 4> {
    /// The `(u, v)` trace of `curve` across this surface: sample the curve,
    /// Newton-project each sample onto the surface (each projection seeded
    /// from the previous one's result, so the walk stays continuous), and
    /// fit a pcurve through the results.
    ///
    /// The pcurve is fitted piece by piece, the curve broken at each knot
    /// where it is less than twice continuously differentiable — the joints
    /// of the kernel's rational arcs and helices, which are only `C1` — and
    /// the pieces joined. The fit is a cubic spline, `C2` at its own knots:
    /// across a jump in the trace's curvature it cannot follow, and its
    /// drift there shrinks only as `h^2` with the spacing of the samples. A
    /// helix of pitch 1 fitted across its joints drifted past the accuracy
    /// limit. Within a piece, the samples are doubled while the pcurve is
    /// wider than the target (see [`DRIFT_PER_RESOLUTION`]).
    ///
    /// The walk's first seed has to be found globally, because Newton only
    /// polishes a foot point it is already near: seeded from anywhere else
    /// on a curved patch it settles on whichever local foot point is closest
    /// — including one clamped against a domain bound, where the residual is
    /// merely orthogonal to the boundary — and every later sample, seeded
    /// from its predecessor, follows it there. So the start is `pin_start`
    /// when there is one (the face's own authoritative `(u, v)`), and is
    /// otherwise isolated by [`surface_could_contain`] (`max_nodes` /
    /// `min_subdivision_size` bound that search) — subdivide to isolate,
    /// then Newton to refine. A curve that does not start on this surface is
    /// an error: there is no trace of it to fit.
    ///
    /// `pin_start` / `pin_end` override the projected `(u, v)` of the first
    /// and last sample. Pass them whenever the curve's endpoint is a place
    /// this surface's face *already* has a coedge for: that coedge's own
    /// pcurve endpoint is the authoritative `(u, v)` there, and an
    /// independently re-projected one lands a hair away from it — enough to
    /// break the exact `could_be_equal` continuity a face's boundary loop
    /// requires between one coedge's pcurve end and the next one's start.
    /// The endpoints are free to pin without disturbing the rest of the
    /// curve because `interpolate` produces a clamped B-spline, which
    /// passes exactly through each sample.
    pub fn fit_pcurve(
        &self,
        curve: &NurbCurve<S, 4>,
        pin_start: Option<Vector2<S>>,
        pin_end: Option<Vector2<S>>,
        max_nodes: usize,
        min_subdivision_size: S,
    ) -> GeopResult<NurbCurve2D<S>> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "NurbSurface::fit_pcurve: curve={curve:?}, domain_u={:?}, domain_v={:?}",
                self.domain_u(),
                self.domain_v(),
            ))
        };

        let (t0, t1) = curve.domain();
        // Seed the first projection where the curve actually starts on this
        // surface (see above); every later one seeds from its predecessor.
        // Sharp, since a seed is a free choice.
        let (seed_u, seed_v) = match pin_start {
            Some(pin) => (pin[0], pin[1]),
            None => {
                let start = curve.evaluate(t0).with_context(&ctx)?;
                surface_could_contain(self, &start, max_nodes, min_subdivision_size)
                    .with_context(&ctx)?
                    .ok_or_else(|| {
                        ctx(GeopError::new(format!(
                            "the curve starts at {start:?}, which is not on this surface"
                        )))
                    })?
            }
        };
        let mut seed = (seed_u.sharpen(), seed_v.sharpen());

        // The foot point of the curve at `frac` of the way from `a` to `b`,
        // projected from `seed`, which then moves on to it. Seeded from a
        // sharp value (any point inside the previous iterate is an equally
        // valid starting guess), but the projection's own enclosure is
        // what's returned: these `(u, v)` end up in the fitted pcurve, which
        // is later compared against the edge's 3D points, so narrowing them
        // here would claim precision the projection did not have.
        let project_at = |seed: &mut (S, S), (a, b): (S, S), frac: S| -> GeopResult<Vector2<S>> {
            let t = a.add(b.sub(a).mul(frac));
            let p = curve.evaluate(t)?;
            let (u, v) = self.project(p, seed.0, seed.1, NEWTON_ITERATIONS)?;
            *seed = (u.sharpen(), v.sharpen());
            Ok(Vector2::from_array([u, v]))
        };

        // Where the curve is less than `C2`: the ends of its domain, and
        // every interior knot of multiplicity at least `degree − 1`.
        let p = curve.degree;
        let mut breaks = vec![t0];
        breaks.extend(
            interior_knots(&curve.knot_vector, p, curve.control_points.len())
                .into_iter()
                .filter(|&(_, m)| m + 1 >= p)
                .map(|(k, _)| k),
        );
        breaks.push(t1);

        let target = min_subdivision_size
            .div(S::from_i64(DRIFT_PER_RESOLUTION))
            .with_context(&ctx)?
            .upper();
        let length = t1.sub(t0).to_f64();
        let mut pieces = Vec::with_capacity(breaks.len() - 1);
        // Where the next piece starts: the pin, then where the last one
        // ended — the same point projected again would land a hair away.
        let mut start = pin_start;
        for (k, piece) in breaks.windows(2).enumerate() {
            let (a, b) = (piece[0], piece[1]);
            let end = if k + 2 == breaks.len() { pin_end } else { None };
            let piece_seed = seed;
            // How many samples is a free choice: the piece's share of 48.
            let share = b.sub(a).to_f64() / length;
            let mut intervals =
                ((SAMPLES as f64 * share).ceil() as usize).clamp(MIN_PIECE_SAMPLES, SAMPLES);
            loop {
                // Each attempt walks the piece afresh, from its start.
                seed = piece_seed;
                // The samples the pcurve passes through, and — walking the
                // same path, so every projection seeds from its neighbour —
                // the true trace between each consecutive pair, which the
                // pcurve is widened to enclose
                // (`NurbCurve2D::interpolate_enclosing`): an interpolant
                // drifts from its trace between samples, and that drift is
                // part of what the pcurve honestly knows about where the
                // trace is.
                let count = S::from_i64(intervals as i64);
                let mut uvs = Vec::with_capacity(intervals + 1);
                let mut between = Vec::with_capacity(intervals);
                for i in 0..=intervals {
                    let frac = S::from_i64(i as i64).div(count).with_context(&ctx)?;
                    uvs.push(project_at(&mut seed, (a, b), frac).with_context(&ctx)?);
                    if i < intervals {
                        let fractions = true_point_fractions(i, intervals);
                        let mut inside = Vec::with_capacity(fractions.len());
                        for &(num, den) in fractions {
                            let frac =
                                S::from_ratio(i as i64 * den + num, intervals as i64 * den)
                                    .with_context(&ctx)?;
                            inside.push(project_at(&mut seed, (a, b), frac).with_context(&ctx)?);
                        }
                        between.push(inside);
                    }
                }
                // Still projected, as the walk's seed.
                if let Some(at) = start {
                    uvs[0] = at;
                }
                if let Some(pin) = end {
                    *uvs.last_mut().expect("uvs is never empty") = pin;
                }
                let fitted =
                    NurbCurve2D::interpolate_enclosing(&uvs, &between, 3).with_context(&ctx)?;
                let wide = fitted
                    .control_points
                    .iter()
                    .any(|cp| (0..2).any(|c| cp[c].width().definitely_greater(target)));
                if !wide || intervals * 2 > MAX_PIECE_SAMPLES {
                    start = uvs.last().copied();
                    pieces.push(fitted);
                    break;
                }
                intervals *= 2;
            }
        }
        NurbCurve2D::join(&pieces).with_context(&ctx)
    }
}

/// Newton iteration count for each sample's foot-point projection. Unlike
/// `max_nodes`/`min_subdivision_size` this doesn't decide whether a search
/// converges to a correct-or-error answer, only how tightly a
/// fixed-iteration projection tracks its target, so it's a constant rather
/// than a threaded parameter.
const NEWTON_ITERATIONS: usize = 20;

#[cfg(test)]
mod tests {
    use geop_core_math::{
        for_all_scalars,
        primitives::CoordinateSystem,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use crate::{
        nurb_curve::{Handedness, NurbCurve3D},
        nurb_surface::NurbSurface,
        shape::Axis,
    };

    /// The pcurve of the kernel's own helix — radius 1, pitch 1, almost a
    /// full turn, so its quarter-turn spans meet at three joints where it is
    /// only `C1` — on the cylinder it lies on: as narrow as a smooth trace's,
    /// and on the helix at both ends. Fitted as one cubic across the joints
    /// it drifted by about 1e-4, past what validation allows any entity to
    /// carry (`a_strip_turning_more_than_once_is_cut_along_meridians` in
    /// `geop-ops-step` had to use a pitch of a quarter).
    fn check_a_helix_is_fitted_between_its_joints<S: Scalar>() {
        let f = S::from_f64;
        let v = |x: f64, y: f64, z: f64| Vector3::from_array([f(x), f(y), f(z)]);
        let helix = NurbCurve3D::helix(
            &CoordinateSystem::world_at(v(0.0, 0.0, 0.0)),
            S::ONE,
            S::ONE,
            0.9,
            Handedness::Right,
        )
        .unwrap();
        let line = NurbCurve3D::try_new(
            1,
            vec![
                Vector4::from_array([f(1.0), f(0.0), f(-0.5), S::ONE]),
                Vector4::from_array([f(1.0), f(0.0), f(1.5), S::ONE]),
            ],
            vec![f(0.0), f(0.0), f(1.0), f(1.0)],
        )
        .unwrap();
        // Its seam a little behind where the helix starts.
        let axis = Axis::try_new(v(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)).unwrap();
        let cylinder =
            NurbSurface::revolve(&line, &axis, -0.5, std::f64::consts::TAU - 0.5).unwrap();
        let pcurve = cylinder
            .fit_pcurve(&helix, None, None, 5000, f(1e-4))
            .unwrap();
        let widest = pcurve
            .control_points
            .iter()
            .flat_map(|cp| (0..2).map(|c| cp[c].width().to_f64()))
            .fold(0.0, f64::max);
        assert!(widest < 1e-5, "widest control point {widest:e}");
        let ((s0, s1), (t0, t1)) = (pcurve.domain(), helix.domain());
        for (s, t) in [(s0, t0), (s1, t1)] {
            let uv = pcurve.evaluate(s).unwrap();
            let on = cylinder.evaluate(uv[0], uv[1]).unwrap();
            assert!(on.could_be_equal(&helix.evaluate(t).unwrap()), "{on:?}");
        }
    }
    #[test]
    fn a_helix_is_fitted_between_its_joints() {
        for_all_scalars!(check_a_helix_is_fitted_between_its_joints);
    }
}
