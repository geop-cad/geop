//! Chains of straight and circular curves in a plane, offset sideways —
//! and the band between two offsets of one chain, the profile a rib or a
//! lip is swept from.
//!
//! **Offsetting.** A line moves sideways, an arc grows or shrinks around
//! its centre: both stay exactly what they were. Where two pieces meet
//! smoothly their offsets meet at the joint moved along its normal; where
//! two lines meet at a corner their offsets are cut back, or run on, to
//! where they cross — a mitre. A corner involving an arc is not supported:
//! its offsets would need a round of their own, or a cut at an angle no
//! piece defines.

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_ops_extrude_revolve::common::{Profile, line2};

/// A straight or circular curve of a chain, from where the previous one
/// ends, named for what it was made from.
#[derive(Clone, Debug)]
pub struct Piece<S: Scalar> {
    pub curve: NurbCurve2D<S>,
    /// The centre of the circle an arc lies on; none for a line.
    pub center: Option<Vector2<S>>,
    pub name: String,
}

/// A chain of pieces, each starting where the one before ends — closed if
/// the last ends where the first starts — and the names of its joints:
/// `joints[i]` where piece `i` starts, and for an open chain one more, its
/// end.
#[derive(Clone, Debug)]
pub struct Chain<S: Scalar> {
    pub pieces: Vec<Piece<S>>,
    pub joints: Vec<String>,
    pub closed: bool,
}

/// The point the homogeneous `p` stands for.
fn point<S: Scalar>(p: &Vector3<S>) -> GeopResult<Vector2<S>> {
    Ok(Vector2::from_array([p[0].div(p[2])?, p[1].div(p[2])?]))
}

/// The unit vector to the left of `t`.
fn left<S: Scalar>(t: &Vector2<S>) -> Vector2<S> {
    Vector2::from_array([t[1].neg(), t[0]])
}

fn cross<S: Scalar>(a: &Vector2<S>, b: &Vector2<S>) -> S {
    a[0].mul(b[1]).sub(a[1].mul(b[0]))
}

impl<S: Scalar> Piece<S> {
    fn start(&self) -> GeopResult<Vector2<S>> {
        point(&self.curve.control_points[0])
    }

    fn end(&self) -> GeopResult<Vector2<S>> {
        point(
            self.curve
                .control_points
                .last()
                .expect("a curve has control points"),
        )
    }

    /// The unit direction it starts and ends going in.
    fn directions(&self) -> GeopResult<(Vector2<S>, Vector2<S>)> {
        let (t0, t1) = self.curve.domain();
        Ok((
            self.curve.tangent(t0)?.normalize()?,
            self.curve.tangent(t1)?.normalize()?,
        ))
    }

    /// The same piece `distance` to its left — negative: to its right —
    /// its ends not yet fitted to its neighbours'.
    fn offset(&self, distance: S) -> GeopResult<NurbCurve2D<S>> {
        let mut curve = self.curve.clone();
        let (start, _) = self.directions()?;
        match &self.center {
            None => {
                let shift = left(&start).prod_scalar(distance);
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
                let turning_left = cross(&start, &center.sub(&from)).definitely_greater(S::ZERO);
                let new_radius = if turning_left {
                    radius.sub(distance)
                } else {
                    radius.add(distance)
                };
                if !new_radius.definitely_greater(S::ZERO) {
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

/// `curve` with its ends moved to exactly `start` and `end` — the same
/// points up to rounding, made exact so the pieces of a loop meet.
fn with_ends<S: Scalar>(
    curve: &NurbCurve2D<S>,
    start: Vector2<S>,
    end: Vector2<S>,
) -> GeopResult<NurbCurve2D<S>> {
    if curve.degree == 1 && curve.control_points.len() == 2 {
        return line2(start, end);
    }
    let mut curve = curve.clone();
    let last = curve.control_points.len() - 1;
    for (i, p) in [(0, start), (last, end)] {
        let w = curve.control_points[i][2];
        curve.control_points[i] = Vector3::from_array([p[0].mul(w), p[1].mul(w), w]);
    }
    curve.recompute_aabb();
    Ok(curve)
}

impl<S: Scalar> Chain<S> {
    /// The chain moved `distance` to its left — negative: to its right —
    /// see the module docs.
    pub fn offset(&self, distance: S) -> GeopResult<Vec<NurbCurve2D<S>>> {
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
                point(&raw[0].control_points[0])?
            } else {
                self.joint(i, distance, &raw)?
            });
        }
        starts.push(if self.closed {
            starts[0]
        } else {
            point(raw[n - 1].control_points.last().expect("control points"))?
        });
        (0..n)
            .map(|i| {
                let curve = with_ends(&raw[i], starts[i], starts[i + 1])?;
                // A line cut back past its other end turns around.
                if self.pieces[i].center.is_none() {
                    let (old, new) = (
                        self.pieces[i].end()?.sub(&self.pieces[i].start()?),
                        starts[i + 1].sub(&starts[i]),
                    );
                    if !old.prod_dot(&new).definitely_greater(S::ZERO) {
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
    fn joint(&self, i: usize, distance: S, raw: &[NurbCurve2D<S>]) -> GeopResult<Vector2<S>> {
        let n = self.pieces.len();
        let before = (i + n - 1) % n;
        let (_, arriving) = self.pieces[before].directions()?;
        let (leaving, _) = self.pieces[i].directions()?;
        let at = self.pieces[i].start()?;
        if cross(&arriving, &leaving).could_be_equal(S::ZERO)
            && arriving.prod_dot(&leaving).definitely_greater(S::ZERO)
        {
            return Ok(at.add(&left(&leaving).prod_scalar(distance)));
        }
        if self.pieces[before].center.is_some() || self.pieces[i].center.is_some() {
            return Err(GeopError::new(format!(
                "{} and {} meet at a corner, at {}: an offset round a corner is only supported between straight pieces",
                self.pieces[before].name, self.pieces[i].name, self.joints[i]
            )));
        }
        // Where the two offset lines cross.
        let a = point(raw[before].control_points.last().expect("control points"))?;
        let b = point(&raw[i].control_points[0])?;
        let denominator = cross(&arriving, &leaving);
        let s = cross(&b.sub(&a), &leaving).div(denominator)?;
        Ok(a.add(&arriving.prod_scalar(s)))
    }

    /// The band between the chain moved `near` and `far` to its left
    /// (`near < far`; zero for the chain itself) as the loops of a region,
    /// the outer one first and counter-clockwise, named after the pieces
    /// and joints: `X` and `P` for the near side, `X,far` and `P,far` for
    /// the far one, `first` and `last` the ends of an open chain.
    pub fn band(&self, near: S, far: S) -> GeopResult<Vec<Profile<S>>> {
        let at = |distance: S| -> GeopResult<Vec<NurbCurve2D<S>>> {
            if distance.could_be_equal(S::ZERO) && distance.is_sharp() {
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
            curves: near_curves,
            curve_names: names.clone(),
            joint_names: self.joints.clone(),
        };
        let far_side = Profile {
            curves: far_curves,
            curve_names: names.iter().map(|n| far_name(n)).collect(),
            joint_names: self.joints.iter().map(|n| far_name(n)).collect(),
        };
        if !self.closed {
            // Along the near side, back along the far one: the band is on
            // the left all the way round.
            let near_end = point(
                near_side
                    .curves
                    .last()
                    .unwrap()
                    .control_points
                    .last()
                    .unwrap(),
            )?;
            let near_start = point(&near_side.curves[0].control_points[0])?;
            let far = far_side.reversed();
            let far_start = point(&far.curves[0].control_points[0])?;
            let far_end = point(far.curves.last().unwrap().control_points.last().unwrap())?;
            let mut curves = near_side.curves;
            curves.push(line2(near_end, far_start)?);
            curves.extend(far.curves);
            curves.push(line2(far_end, near_start)?);
            let mut curve_names = near_side.curve_names;
            curve_names.push("last".into());
            curve_names.extend(far.curve_names);
            curve_names.push("first".into());
            let mut joint_names = near_side.joint_names;
            joint_names.extend(far.joint_names);
            return Ok(vec![Profile {
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
    /// area of the polygon through its control points — exact for lines,
    /// and for arcs of less than a half turn on the same side as the arc.
    fn winds_counter_clockwise(&self) -> GeopResult<bool> {
        let mut points = Vec::new();
        for piece in &self.pieces {
            let cps = &piece.curve.control_points;
            for cp in &cps[..cps.len() - 1] {
                points.push(point(cp)?);
            }
        }
        let mut area = S::ZERO;
        for i in 0..points.len() {
            area = area.add(cross(&points[i], &points[(i + 1) % points.len()]));
        }
        if area.definitely_greater(S::ZERO) {
            Ok(true)
        } else if area.definitely_less(S::ZERO) {
            Ok(false)
        } else {
            Err(GeopError::new("cannot tell which way the chain winds"))
        }
    }
}

#[cfg(test)]
mod tests;
