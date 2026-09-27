use crate::{
    contains::surface::surface_could_contain,
    nurb_curve::{NurbCurve, NurbCurve2D, true_point_fractions},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
};

use super::NurbSurface;

/// How many intervals the curve is split into for projection; the fitted
/// pcurve passes exactly through all `SAMPLES + 1` points and is widened to
/// enclose the trace between them (see `NurbCurve2D::interpolate_enclosing`).
///
/// A cubic interpolant's drift falls as `h^4`, and since the pcurve carries
/// that drift as width, this decides how *wide* the pcurve is, not whether it
/// is right. At 9 samples the drift reached ~2e-5 on strongly curved patches
/// and exceeded 1e-4 on the worst of them — too wide for the accuracy
/// `validation::numerical_accuracy` holds every entity to. 48 keeps it well
/// inside that.
///
/// This bounds effort, not correctness: each sample is one Newton foot-point
/// projection (plus one per true point between samples, see
/// `true_point_fractions`),
/// and a pcurve fitted through more of them is strictly narrower.
const SAMPLES: usize = 48;

impl<S: Scalar> NurbSurface<S, 4> {
    /// The `(u, v)` trace of `curve` across this surface: sample the curve,
    /// Newton-project each sample onto the surface (each projection seeded
    /// from the previous one's result, so the walk stays continuous), and
    /// fit a pcurve through the results.
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
        let (mut seed_u, mut seed_v) = (seed_u.sharpen(), seed_v.sharpen());

        // The foot point of the curve at `frac` of its domain. Seeded from a
        // sharp value (any point inside the previous iterate is an equally
        // valid starting guess), but the projection's own enclosure is what's
        // returned: these `(u, v)` end up in the fitted pcurve, which is later
        // compared against the edge's 3D points, so narrowing them here would
        // claim precision the projection did not have.
        let mut project_at = |frac: S| -> GeopResult<Vector2<S>> {
            let t = t0.add(t1.sub(t0).mul(frac));
            let p = curve.evaluate(t)?;
            let (u, v) = self.project(p, seed_u, seed_v, NEWTON_ITERATIONS)?;
            seed_u = u.sharpen();
            seed_v = v.sharpen();
            Ok(Vector2::from_array([u, v]))
        };

        // The samples the pcurve passes through, and — walking the same path,
        // so every projection seeds from its neighbour — the true trace
        // between each consecutive pair, which the pcurve is widened to
        // enclose (`NurbCurve2D::interpolate_enclosing`): an interpolant
        // drifts from its trace between samples, and that drift is part of
        // what the pcurve honestly knows about where the trace is.
        let samples = S::from_i64(SAMPLES as i64);
        let mut uvs = Vec::with_capacity(SAMPLES + 1);
        let mut between = Vec::with_capacity(SAMPLES);
        for i in 0..=SAMPLES {
            uvs.push(
                project_at(S::from_i64(i as i64).div(samples).with_context(&ctx)?)
                    .with_context(&ctx)?,
            );
            if i < SAMPLES {
                let fractions = true_point_fractions(i, SAMPLES);
                let mut inside = Vec::with_capacity(fractions.len());
                for &(a, b) in fractions {
                    let frac =
                        S::from_ratio(i as i64 * b + a, SAMPLES as i64 * b).with_context(&ctx)?;
                    inside.push(project_at(frac).with_context(&ctx)?);
                }
                between.push(inside);
            }
        }

        if let Some(pin) = pin_start {
            uvs[0] = pin;
        }
        if let Some(pin) = pin_end {
            *uvs.last_mut().expect("uvs is never empty") = pin;
        }

        NurbCurve2D::interpolate_enclosing(&uvs, &between, 3).with_context(&ctx)
    }
}

/// Newton iteration count for each sample's foot-point projection. Unlike
/// `max_nodes`/`min_subdivision_size` this doesn't decide whether a search
/// converges to a correct-or-error answer, only how tightly a
/// fixed-iteration projection tracks its target, so it's a constant rather
/// than a threaded parameter.
const NEWTON_ITERATIONS: usize = 20;
