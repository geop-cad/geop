//! Chains of curves in a plane, offset sideways — and the band between two
//! offsets of one chain, the profile a rib or a lip is swept from.
//!
//! **Offsetting.** A line moves sideways, an arc grows or shrinks around
//! its centre: both stay exactly what they were. The offset of any other
//! curve is no NURBS: a cubic is fitted through points of it, as many as
//! it takes to come within [`FIT_TOLERANCE`] of the distance everywhere,
//! proven ([`NurbCurve2D::offset_miss`]). Where two pieces meet smoothly
//! their offsets meet at the joint moved along its normal; where two lines
//! meet at a corner their offsets are cut back, or run on, to where they
//! cross — a mitre. A corner involving any other piece is not supported:
//! its offsets would need a round of their own, or a cut at an angle no
//! piece defines.

use geop_core_geometry::nurb_curve::{NurbCurve2D, loop_signed_area};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector2,
};
use geop_ops_model::Profile;

/// How near a fitted offset comes to the true one, relative to the
/// distance: the accuracy a band's side is made to, not a comparison.
const FIT_TOLERANCE: f64 = 1e-6;

/// The most points a fitted offset is fitted through.
const MOST_FIT_POINTS: usize = 1024;

/// A curve of a chain, from where the previous one ends, named for what it
/// was made from.
#[derive(Clone, Debug)]
pub struct Piece {
    pub curve: NurbCurve2D,
    /// The centre of the circle an arc lies on; none for a line or any
    /// other curve.
    pub center: Option<Vector2>,
    pub name: String,
}

/// A chain of pieces, each starting where the one before ends — closed if
/// the last ends where the first starts — and the names of its joints:
/// `joints[i]` where piece `i` starts, and for an open chain one more, its
/// end.
#[derive(Clone, Debug)]
pub struct Chain {
    pub pieces: Vec<Piece>,
    pub joints: Vec<String>,
    /// Per piece, whether the chain is known to run on smoothly into it
    /// from the piece before — as the faces of a solid say, where a
    /// traced piece's tangent only encloses its direction. Where not, its
    /// pieces' tangents decide.
    pub smooth: Vec<bool>,
    pub closed: bool,
}

impl Piece {
    fn start(&self) -> GeopResult<Vector2> {
        self.curve.start()
    }

    fn end(&self) -> GeopResult<Vector2> {
        self.curve.end()
    }

    /// The unit direction it starts and ends going in.
    fn directions(&self) -> GeopResult<(Vector2, Vector2)> {
        let (t0, t1) = self.curve.domain();
        Ok((
            self.curve.tangent(t0)?.normalize()?,
            self.curve.tangent(t1)?.normalize()?,
        ))
    }

    /// The same piece `distance` to its left — negative: to its right —
    /// its ends not yet fitted to its neighbours'.
    fn offset(&self, distance: Scalar) -> GeopResult<NurbCurve2D> {
        let mut curve = self.curve.clone();
        let (start, _) = self.directions()?;
        match &self.center {
            None if !self.is_straight() => return self.fitted_offset(distance),
            None => {
                let shift = start.perp().prod_scalar(distance);
                for cp in &mut curve.control_points {
                    cp[0] = cp[0].add(shift[0].mul(cp[2]));
                    cp[1] = cp[1].add(shift[1].mul(cp[2]));
                }
            }
            Some(center) => {
                let from = self.start()?;
                let radius = from.sub(center).norm();
                // The centre lies on the left of an arc turning left: an
                // offset to the left comes closer to it.
                let turning_left = start
                    .prod_cross(&center.sub(&from))
                    .definitely_greater(Scalar::ZERO);
                let new_radius = if turning_left {
                    radius.sub(distance)
                } else {
                    radius.add(distance)
                };
                if !new_radius.definitely_greater(Scalar::ZERO) {
                    return Err(GeopError::new(format!(
                        "{} curves too tightly: its radius {:?} leaves no room for an offset of {:?}",
                        self.name,
                        radius.to_f64(),
                        distance.to_f64()
                    )));
                }
                let scale = new_radius.div(radius)?;
                for cp in &mut curve.control_points {
                    for k in 0..2 {
                        let c = center[k].mul(cp[2]);
                        cp[k] = c.add(cp[k].sub(c).mul(scale));
                    }
                }
            }
        }
        curve.recompute_aabb();
        Ok(curve)
    }
}

impl Piece {
    /// Whether it is a line.
    pub(crate) fn is_straight(&self) -> bool {
        self.curve.degree == 1 && self.curve.control_points.len() == 2
    }

    /// A cubic through points of its offset `distance` to its left, at
    /// parameters spaced evenly over each of its spans, the fit's
    /// parameters following its own: twice as many while the fit is
    /// further than [`FIT_TOLERANCE`] of the distance from the offset.
    fn fitted_offset(&self, distance: Scalar) -> GeopResult<NurbCurve2D> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "the offset of {} by {:?}",
                self.name,
                distance.to_f64()
            ))
        };
        let curve = &self.curve;
        let (t0, t1) = curve.domain();
        let tolerance = distance.abs().mul(Scalar::from_f64(FIT_TOLERANCE));
        let breaks = curve.breakpoints();
        let spans = breaks.len() - 1;
        let mut per_span = 4;
        let mut last: Option<(Scalar, NurbCurve2D)> = None;
        loop {
            // The parameters are free choices: sharp ones.
            let mut ts = vec![t0];
            for w in breaks.windows(2) {
                for k in 1..=per_span {
                    let f = Scalar::from_ratio(k as i64, per_span as i64)?;
                    ts.push(w[0].add(w[1].sub(w[0]).mul(f)).sharpen());
                }
            }
            *ts.last_mut().expect("a parameter") = t1;
            let span = t1.sub(t0);
            let mut params = Vec::with_capacity(ts.len());
            let mut values = Vec::with_capacity(ts.len());
            for (k, &t) in ts.iter().enumerate() {
                params.push(match k {
                    0 => Scalar::ZERO,
                    k if k + 1 == ts.len() => Scalar::ONE,
                    _ => t.sub(t0).div(span).map_err(ctx)?.sharpen(),
                });
                let normal = curve
                    .tangent(t)
                    .map_err(ctx)?
                    .perp()
                    .normalize()
                    .map_err(ctx)?;
                // A point of the offset, any of its enclosure: the fit is a
                // free choice near the offset, proven so below.
                let p = curve
                    .evaluate(t)?
                    .add(&normal.prod_scalar(distance))
                    .sharpen();
                values.push(p.homogeneous(Scalar::ONE));
            }
            let fit = NurbCurve2D::interpolate_homogeneous(&values, &params, 3).map_err(ctx)?;
            let miss = curve
                .offset_miss(distance, (t0, t1), &fit, tolerance)
                .map_err(ctx)?;
            if !miss.definitely_greater(tolerance) {
                return Ok(fit);
            }
            // Where twice the points no longer halve the miss, it is the
            // curve's own width, and the coarser fit is as near.
            if let Some((before, coarser)) = last
                && !before.definitely_greater(miss.mul(Scalar::TWO))
            {
                return Ok(coarser);
            }
            if 2 * per_span * spans > MOST_FIT_POINTS {
                return Err(ctx(GeopError::new(format!(
                    "a cubic through {} points of it is still {miss:?} off",
                    per_span * spans + 1
                ))));
            }
            per_span *= 2;
            last = Some((miss, fit));
        }
    }
}

/// `curve` with its ends moved to exactly `start` and `end` — the same
/// points up to rounding, made exact so the pieces of a loop meet.
fn with_ends(curve: &NurbCurve2D, start: Vector2, end: Vector2) -> GeopResult<NurbCurve2D> {
    if curve.degree == 1 && curve.control_points.len() == 2 {
        return NurbCurve2D::line(start, end);
    }
    let mut curve = curve.clone();
    let last = curve.control_points.len() - 1;
    for (i, p) in [(0, start), (last, end)] {
        curve.control_points[i] = p.homogeneous(curve.control_points[i][2]);
    }
    curve.recompute_aabb();
    Ok(curve)
}

impl Chain {
    /// The chain moved `distance` to its left — negative: to its right —
    /// see the module docs.
    pub fn offset(&self, distance: Scalar) -> GeopResult<Vec<NurbCurve2D>> {
        let n = self.pieces.len();
        let raw = self
            .pieces
            .iter()
            .map(|p| p.offset(distance))
            .collect::<GeopResult<Vec<_>>>()?;
        // Where each piece's offset starts: at a joint between two pieces,
        // where their offsets meet.
        let mut starts = Vec::with_capacity(n + 1);
        for i in 0..n {
            starts.push(if i == 0 && !self.closed {
                raw[0].start()?
            } else {
                self.joint(i, distance, &raw)?
            });
        }
        starts.push(if self.closed {
            starts[0]
        } else {
            raw[n - 1].end()?
        });
        (0..n)
            .map(|i| {
                let curve = with_ends(&raw[i], starts[i], starts[i + 1])?;
                // A line cut back past its other end turns around.
                if self.pieces[i].is_straight() {
                    let (old, new) = (
                        self.pieces[i].end()?.sub(&self.pieces[i].start()?),
                        starts[i + 1].sub(&starts[i]),
                    );
                    if !old.prod_dot(&new).definitely_greater(Scalar::ZERO) {
                        return Err(GeopError::new(format!(
                            "{} is too short for an offset of {:?}: it would turn around",
                            self.pieces[i].name,
                            distance.to_f64()
                        )));
                    }
                }
                Ok(curve)
            })
            .collect()
    }

    /// Where the offsets `raw` of the pieces before and after joint `i` meet.
    fn joint(&self, i: usize, distance: Scalar, raw: &[NurbCurve2D]) -> GeopResult<Vector2> {
        let n = self.pieces.len();
        let before = (i + n - 1) % n;
        let (_, arriving) = self.pieces[before].directions()?;
        let (leaving, _) = self.pieces[i].directions()?;
        let at = self.pieces[i].start()?;
        if self.smooth[i]
            || (arriving.prod_cross(&leaving).could_be_equal(Scalar::ZERO)
                && arriving.prod_dot(&leaving).definitely_greater(Scalar::ZERO))
        {
            // Along the normal of a line there, if one meets it: exact.
            let along = if self.pieces[before].is_straight() && !self.pieces[i].is_straight() {
                arriving
            } else {
                leaving
            };
            return Ok(at.add(&along.perp().prod_scalar(distance)));
        }
        if !(self.pieces[before].is_straight() && self.pieces[i].is_straight()) {
            return Err(GeopError::new(format!(
                "{} and {} meet at a corner, at {}: an offset round a corner is only supported between straight pieces",
                self.pieces[before].name, self.pieces[i].name, self.joints[i]
            )));
        }
        // Where the two offset lines cross.
        let a = raw[before].end()?;
        let b = raw[i].start()?;
        let denominator = arriving.prod_cross(&leaving);
        let s = b.sub(&a).prod_cross(&leaving).div(denominator)?;
        Ok(a.add(&arriving.prod_scalar(s)))
    }

    /// The band between the chain moved `near` and `far` to its left
    /// (`near < far`; zero for the chain itself) as the loops of a region,
    /// the outer one first and counter-clockwise, named after the pieces
    /// and joints: `X` and `P` for the near side, `X,far` and `P,far` for
    /// the far one, `first` and `last` the ends of an open chain.
    pub fn band(&self, near: Scalar, far: Scalar) -> GeopResult<Vec<Profile>> {
        let at = |distance: Scalar| -> GeopResult<Vec<NurbCurve2D>> {
            if distance.could_be_equal(Scalar::ZERO) && distance.is_sharp() {
                // The chain itself, not moved by a rounded zero.
                Ok(self.pieces.iter().map(|p| p.curve.clone()).collect())
            } else {
                self.offset(distance)
            }
        };
        let (near_curves, far_curves) = (at(near)?, at(far)?);
        let names: Vec<String> = self.pieces.iter().map(|p| p.name.clone()).collect();
        let far_name = |name: &str| format!("{name},far");
        let near_side = Profile {
            name: String::new(),
            curves: near_curves,
            curve_names: names.clone(),
            joint_names: self.joints.clone(),
        };
        let far_side = Profile {
            name: String::new(),
            curves: far_curves,
            curve_names: names.iter().map(|n| far_name(n)).collect(),
            joint_names: self.joints.iter().map(|n| far_name(n)).collect(),
        };
        if !self.closed {
            // Along the near side, back along the far one: the band is on
            // the left all the way round.
            let near_end = near_side.curves.last().unwrap().end()?;
            let near_start = near_side.curves[0].start()?;
            let far = far_side.reversed();
            let far_start = far.curves[0].start()?;
            let far_end = far.curves.last().unwrap().end()?;
            let mut curves = near_side.curves;
            curves.push(NurbCurve2D::line(near_end, far_start)?);
            curves.extend(far.curves);
            curves.push(NurbCurve2D::line(far_end, near_start)?);
            let mut curve_names = near_side.curve_names;
            curve_names.push("last".into());
            curve_names.extend(far.curve_names);
            curve_names.push("first".into());
            let mut joint_names = near_side.joint_names;
            joint_names.extend(far.joint_names);
            return Ok(vec![Profile {
                name: String::new(),
                curves,
                curve_names,
                joint_names,
            }]);
        }
        // Closed: the band is the ring between the two. The one further
        // left encloses the other if the chain winds clockwise.
        if self.winds_counter_clockwise()? {
            Ok(vec![near_side, far_side.reversed()])
        } else {
            Ok(vec![far_side.reversed(), near_side])
        }
    }

    /// Whether the closed chain winds counter-clockwise, from the signed
    /// area it encloses, integrated along its curves.
    fn winds_counter_clockwise(&self) -> GeopResult<bool> {
        let area = loop_signed_area(self.pieces.iter().map(|p| &p.curve))?;
        if area.definitely_greater(Scalar::ZERO) {
            Ok(true)
        } else if area.definitely_less(Scalar::ZERO) {
            Ok(false)
        } else {
            Err(GeopError::new("cannot tell which way the chain winds"))
        }
    }
}

#[cfg(test)]
mod tests;
