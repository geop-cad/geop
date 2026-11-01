//! [`NurbCurve3D::fit_enclosing`]: a cubic through stations of a curve
//! known only point by point, widened to enclose the curve between them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::refine::{Refine, Refined, refine},
    vector::Vector3,
};

use super::{NurbCurve, NurbCurve3D, true_point_fractions};

/// The fewest legs a fitted curve runs through (see
/// [`NurbCurve3D::fit_enclosing`]): a cubic interpolant needs points enough
/// to follow the curve, however few stations it was given. Like
/// [`FIT_DRIFT`] this bounds effort — every extra leg is one more point
/// located — and decides how *wide* the fit's enclosure is, not whether it
/// holds.
const MIN_FITTED_LEGS: usize = 8;

/// How wide a fit's enclosure may come out before its legs are split finer
/// and it is fitted again: ten times what the usual traced branch leaves.
/// Like [`MIN_FITTED_LEGS`] it bounds effort and decides how wide the
/// enclosure is, not whether it holds.
const FIT_DRIFT: f64 = 1e-5;

/// The most legs a fit is refined to: the budget [`FIT_DRIFT`] is pursued
/// with, each leg a point located and a row of the interpolation's system.
const MAX_FITTED_LEGS: usize = 1024;

/// Where a fit's widest point between stations was, of its last attempt:
/// leg, fraction, point, width.
pub type WidestBetween = Option<(usize, usize, Vector3, f64)>;

impl NurbCurve3D {
    /// A cubic through `stations` of a curve — each a point of it and
    /// whatever `locate` needs to find more — that encloses the curve
    /// between them too. `locate(from, to, frac)` is the point of the curve
    /// a fraction `frac` of the way along the leg from station `from` to
    /// `to`, with what it needs to go on from there.
    ///
    /// Every leg is first split into pieces: a curve given in only a few
    /// stations would leave too few points for a cubic to follow it, so
    /// every fit has at least [`MIN_FITTED_LEGS`] legs. The cubic drifts
    /// from the curve between its points, which is enclosed as width,
    /// honestly (see [`NurbCurve3D::interpolate_enclosing`]); while that
    /// width is more than [`FIT_DRIFT`] and refining the legs still halves
    /// it, the legs that drift are halved: those a halving of all, which
    /// shrinks a cubic's drift sixteen times, would leave the widest. Where
    /// the curve bends evenly that is every leg. Where it turns sharply in
    /// one place — a blend's arc, tangent to a plane it is cut by at three
    /// degrees, meets it in an ellipse nineteen times as long as it is wide,
    /// its tip of radius `5e-3` — it is those at the tip, which legs of an
    /// even length reach only when there are hundreds of them. Once
    /// refining no longer halves the width, the width is the points' own,
    /// and the coarser fit is kept.
    ///
    /// `widest_between` is told the widest point of the curve located
    /// between stations of the last attempt: the curve is widened to hold
    /// it, so one that is wide makes the whole curve wide, and this says
    /// which.
    pub fn fit_enclosing<T: Clone>(
        stations: &[(Vector3, T)],
        locate: impl Fn(&(Vector3, T), &(Vector3, T), Scalar) -> GeopResult<(Vector3, T)>,
        widest_between: &mut WidestBetween,
        ctx: &dyn Fn(GeopError) -> GeopError,
    ) -> GeopResult<NurbCurve<4>> {
        let strides = stations.len() - 1;
        let pieces = MIN_FITTED_LEGS.div_ceil(strides).max(2);
        let mut dense = vec![stations[0].clone()];
        for i in 0..strides {
            for k in 1..pieces {
                let frac = Scalar::from_ratio(k as i64, pieces as i64).with_context(&ctx)?;
                dense.push(locate(&stations[i], &stations[i + 1], frac).with_context(&ctx)?);
            }
            dense.push(stations[i + 1].clone());
        }
        // With each step the fit before, to tell whether refining the legs
        // still halved its width: each step halves it or is the last, so
        // the steps' cap is never met before [`FIT_DRIFT`] is.
        let first = (dense, None::<(NurbCurve<4>, Scalar)>);
        let refined = refine(
            first,
            usize::BITS as usize,
            |(dense, best)| -> GeopResult<_> {
                // The curve inside each of the legs, which
                // `interpolate_enclosing` widens the cubic to hold.
                let legs = dense.len() - 1;
                let mut between = Vec::with_capacity(legs);
                for i in 0..legs {
                    let fractions = true_point_fractions(i, legs);
                    let mut inside = Vec::with_capacity(fractions.len());
                    for (k, &(a, b)) in fractions.iter().enumerate() {
                        let frac = Scalar::from_ratio(a, b).with_context(&ctx)?;
                        let (p, _) = locate(&dense[i], &dense[i + 1], frac).with_context(&ctx)?;
                        let width = (0..3).map(|c| p[c].width().to_f64()).fold(0.0, f64::max);
                        if widest_between.as_ref().is_none_or(|w| width > w.3) {
                            *widest_between = Some((i, k, p, width));
                        }
                        inside.push(p);
                    }
                    between.push(inside);
                }
                let points: Vec<Vector3> = dense.iter().map(|(p, _)| *p).collect();
                let curve =
                    NurbCurve3D::interpolate_enclosing(&points, &between, 3).with_context(&ctx)?;
                let width = curve
                    .control_points
                    .iter()
                    .flat_map(|p| (0..3).map(move |k| p[k].width()))
                    .fold(
                        Scalar::ZERO,
                        |a, w| if w.definitely_greater(a) { w } else { a },
                    );
                let halved = best
                    .as_ref()
                    .is_none_or(|(_, before)| before.definitely_greater(width.mul(Scalar::TWO)));
                if !halved {
                    return Ok(Refine::Done(best.expect("a fit to compare with").0));
                }
                if legs >= MAX_FITTED_LEGS || !width.definitely_greater(Scalar::from_f64(FIT_DRIFT))
                {
                    return Ok(Refine::Done(curve));
                }
                // The width is twice the drift it holds. Halved, a leg's
                // drift falls sixteen times: those that would then still be
                // the widest are all that need it.
                let drifts =
                    NurbCurve3D::drift_by_interval(&points, &between, 3).with_context(&ctx)?;
                let enough = width.upper().to_f64() / (2.0 * 16.0);
                let mut finer = vec![dense[0].clone()];
                for i in 0..legs {
                    if drifts[i] > enough {
                        let half = Scalar::from_ratio(1, 2).with_context(&ctx)?;
                        finer.push(locate(&dense[i], &dense[i + 1], half).with_context(&ctx)?);
                    }
                    finer.push(dense[i + 1].clone());
                }
                Ok(Refine::Finer((finer, Some((curve, width)))))
            },
        )?;
        let Refined::Done(curve) = refined else {
            return Err(ctx(GeopError::new(
                "the fitted curve's legs were split past every cap",
            )));
        };
        Ok(curve)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A quarter of an ellipse nineteen times as long as it is wide, marched
    /// in legs of one length, ten times its tip's radius of curvature at the
    /// tip: a blend's arc cut by a plane at three degrees to its contact.
    /// Only the few legs at the tip drift. Halving them all would soon pass
    /// the legs a fit may have; those few bring the width down.
    #[test]
    fn a_fit_refines_the_legs_where_the_curve_turns_sharply() {
        let (a, b) = (1.9, 0.1);
        let at = |t: f64| {
            Vector3::from_array([
                Scalar::from_f64(a * t.cos()),
                Scalar::from_f64(b * t.sin()),
                Scalar::ZERO,
            ])
        };
        // Stations a length apart along it, from a table of its length.
        let table = 20000;
        let top = std::f64::consts::FRAC_PI_2;
        let mut lengths = vec![0.0];
        for k in 1..=table {
            let (t0, t1) = (
                top * (k - 1) as f64 / table as f64,
                top * k as f64 / table as f64,
            );
            let d =
                ((a * (t1.cos() - t0.cos())).powi(2) + (b * (t1.sin() - t0.sin())).powi(2)).sqrt();
            lengths.push(lengths[k - 1] + d);
        }
        let strides = 108;
        let stations: Vec<(Vector3, f64)> = (0..=strides)
            .map(|k| {
                let want = lengths[table] * k as f64 / strides as f64;
                let i = lengths.partition_point(|&l| l < want).min(table);
                let t = top * i as f64 / table as f64;
                (at(t), t)
            })
            .collect();
        let curve = NurbCurve3D::fit_enclosing(
            &stations,
            |from, to, frac| {
                let t = from.1 + frac.to_f64() * (to.1 - from.1);
                Ok((at(t), t))
            },
            &mut None,
            &|e| e,
        )
        .unwrap();
        let widest = (curve.control_points.iter())
            .flat_map(|p| (0..3).map(move |k| p[k].width().to_f64()))
            .fold(0.0, f64::max);
        assert!(widest <= FIT_DRIFT, "{widest:e}");
        // Two legs a stride, and a few more at the tip.
        assert!(
            curve.control_points.len() < 300,
            "{}",
            curve.control_points.len()
        );
    }
}
