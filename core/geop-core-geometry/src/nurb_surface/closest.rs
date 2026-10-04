//! [`NurbSurface::closest_point`]: the point of a surface nearest a given
//! point.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::NurbSurface;

/// Samples per polynomial piece and direction the search starts from.
/// Bounds effort only, as for curves.
const SAMPLES_PER_PIECE: usize = 6;

/// Newton steps of [`NurbSurface::project`] from a seed.
const NEWTON_STEPS: usize = 30;

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
    /// [`NurbSurface::project`]. As for curves, the parameters are a free
    /// choice of which point of the surface is returned, so they are
    /// sharpened: the point lies on the surface and its distance is
    /// attained.
    pub fn closest_point(&self, target: &Vector3<S>) -> GeopResult<(S, S, Vector3<S>)> {
        let seeds = self.sample_parameters(SAMPLES_PER_PIECE)?;
        self.closest_point_from(target, &seeds)
    }

    /// [`NurbSurface::closest_point`], starting from the nearest few of
    /// `seeds` rather than a grid of its own.
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
        for &(d, u, v) in scored.iter().take(SEEDS) {
            let (pu, pv) = self.project(*target, u, v, NEWTON_STEPS)?;
            let (pu, pv) = (pu.sharpen(), pv.sharpen());
            let p = self.evaluate(pu, pv)?;
            let candidate = match distance(&p) {
                // A projection that went astray is no better than its seed.
                refined if refined <= d => (refined, pu, pv, p),
                _ => (d, u, v, self.evaluate(u, v)?),
            };
            if best.as_ref().is_none_or(|b| candidate.0 < b.0) {
                best = Some(candidate);
            }
        }
        let (_, u, v, p) = best.ok_or_else(|| {
            geop_core_math::geop_error::GeopError::new("closest_point_from: no seeds")
        })?;
        Ok((u, v, p))
    }
}
