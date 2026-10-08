//! A 3-D sketch's curves as NURBS in space, and its curves joined into
//! chains — the paths and rails built on it.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector3, Vector4},
    with_context,
};

use super::{Constraint3d, CurveKind3d, Enclosure3d, End, Sketch3d, geometry::Arc3};
use crate::{CurveId, PointId, ProfileEdge, ProfileJoint};

/// Samples per piece of a curve drawn as a polyline.
const SAMPLES: usize = 16;

/// One NURBS piece of a chain, on `[0, 1]`, and where it comes from: piece
/// `index` of the sketch curve `source` (counted in the curve's own
/// direction), from joint `start` to joint `end` in the chain's direction —
/// named as a planar sketch's pieces are (see [`crate::ProfilePiece`]).
#[derive(Clone, Debug)]
pub struct Piece3d<S: Scalar> {
    pub curve: NurbCurve3D<S>,
    pub source: CurveId,
    pub index: usize,
    pub start: ProfileJoint,
    pub end: ProfileJoint,
}

impl<S: Scalar> Piece3d<S> {
    /// `c5` for a curve's first piece, `c5#1` for the next.
    pub fn name(&self) -> String {
        if self.index == 0 {
            format!("{}", self.source)
        } else {
            format!("{}#{}", self.source, self.index)
        }
    }
}

/// Curves of a 3-D sketch joined end to end: an open chain, or a closed one
/// whose last curve ends where the first starts. Each curve runs forwards
/// or, `reversed`, backwards along the chain.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chain3d {
    pub edges: Vec<ProfileEdge>,
    pub closed: bool,
}

/// A homogeneous control point of weight `w`.
fn hom<T: Scalar>(p: &Vector3<T>, w: T) -> Vector4<T> {
    Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), w])
}

/// The rational quadratic of the arc from `p` to `q` turning by twice the
/// angle with cosine `cos` and sine `sin` about `normal`: its middle control
/// point where the tangents at its ends meet, `(L/2) tan` out from the
/// chord, towards the arc — exact for any radius, a nearly straight arc's
/// too.
fn conic<T: Scalar>(
    p: &Vector3<T>,
    q: &Vector3<T>,
    normal: &Vector3<T>,
    cos: T,
    sin: T,
) -> GeopResult<NurbCurve3D<T>> {
    let (mid, out) = chord_frame(p, q, normal)?;
    let m = mid.add(&out.prod_scalar(sin.div(cos)?));
    NurbCurve::try_new(
        2,
        vec![hom(p, T::ONE), hom(&m, cos), hom(q, T::ONE)],
        vec![T::ZERO, T::ZERO, T::ZERO, T::ONE, T::ONE, T::ONE],
    )
}

/// The middle of the chord from `p` to `q`, and half its length towards the
/// arc about `normal` through them — whichever its sweep: the chord turned a
/// quarter back about the normal.
fn chord_frame<T: Scalar>(
    p: &Vector3<T>,
    q: &Vector3<T>,
    normal: &Vector3<T>,
) -> GeopResult<(Vector3<T>, Vector3<T>)> {
    let half = T::ONE.div(T::TWO)?;
    let chord = q.sub(p);
    let out = chord
        .prod_cross(normal)
        .normalize()?
        .prod_scalar(chord.try_norm()?.mul(half));
    Ok((p.add(q).prod_scalar(half), out))
}

impl<S: Scalar> Sketch3d<S> {
    /// The curve `curve` as NURBS pieces from its start to its end, each on
    /// `[0, 1]`, built from `geometry`. How many pieces — an arc has one per
    /// quarter turn, or two or four, a rational quadratic's middle weight
    /// staying positive — is decided on the sketch as drawn, which is
    /// design data, so the pieces' names do not depend on how precisely the
    /// geometry is known.
    pub fn curve_nurbs<T: Scalar>(
        &self,
        curve: CurveId,
        geometry: &Enclosure3d<T>,
    ) -> GeopResult<Vec<NurbCurve3D<T>>> {
        let ctx = with_context!("3-D sketch curve {curve} as NURBS");
        let at = |p: &PointId| geometry.points[p];
        match &self.curve(curve).with_context(ctx)?.kind {
            CurveKind3d::Line { start, end } => Ok(vec![
                NurbCurve::try_new(
                    1,
                    vec![hom(&at(start), T::ONE), hom(&at(end), T::ONE)],
                    vec![T::ZERO, T::ZERO, T::ONE, T::ONE],
                )
                .with_context(ctx)?,
            ]),
            CurveKind3d::Arc {
                start,
                through,
                end,
            } => {
                let drawn = Arc3 {
                    s: self.points[start].at,
                    m: self.points[through].at,
                    e: self.points[end].at,
                };
                let (cos, _) = drawn.half_sweep().with_context(ctx)?;
                let pieces = if !cos.definitely_less(S::ONE.div(S::TWO)?.sqrt()?) {
                    1
                } else if !cos.definitely_less(S::ZERO) {
                    2
                } else {
                    4
                };
                let arc = Arc3 {
                    s: at(start),
                    m: at(through),
                    e: at(end),
                };
                let normal = arc.normal().with_context(ctx)?;
                let (cos, sin) = arc.half_sweep().with_context(ctx)?;
                // Halve the arc until it has as many pieces: each bisected
                // at its middle, `(L/2) tan(quarter)` out from its chord.
                let mut ends = vec![arc.s, arc.e];
                let (mut cos, mut sin) = (cos, sin);
                while ends.len() - 1 < pieces {
                    let tan_quarter = sin.div(T::ONE.add(cos))?;
                    let mut halved = vec![ends[0]];
                    for w in ends.windows(2) {
                        let (mid, out) = chord_frame(&w[0], &w[1], &normal)?;
                        halved.push(mid.add(&out.prod_scalar(tan_quarter)));
                        halved.push(w[1]);
                    }
                    ends = halved;
                    let half_cos = T::ONE.add(cos).div(T::TWO)?.sqrt()?;
                    sin = sin.div(T::TWO.mul(half_cos))?;
                    cos = half_cos;
                }
                ends.windows(2)
                    .map(|w| conic(&w[0], &w[1], &normal, cos, sin))
                    .collect::<GeopResult<_>>()
                    .with_context(ctx)
            }
            CurveKind3d::Spline { points } => {
                let through: Vec<Vector3<T>> = points.iter().map(at).collect();
                let start = self
                    .end_tangent(curve, End::Start, geometry)
                    .with_context(ctx)?;
                let end = self
                    .end_tangent(curve, End::End, geometry)
                    .with_context(ctx)?;
                Ok(vec![
                    NurbCurve::cubic_spline(&through, start, end).with_context(ctx)?,
                ])
            }
            CurveKind3d::Reference => Err(GeopError::new(format!(
                "curve {curve} is a reference to an edge of the part, not drawn in the sketch"
            ))),
        }
    }

    /// The direction the spline `curve` leaves (`end` its start) or arrives
    /// (its end) along, if it is given one (see [`Sketch3d::adopts`]): a
    /// [`Constraint3d::TangentTo`]'s, the way the spline runs there; or the
    /// direction of the curve it goes on smoothly from or to.
    fn end_tangent<T: Scalar>(
        &self,
        curve: CurveId,
        end: End,
        geometry: &Enclosure3d<T>,
    ) -> GeopResult<Option<Vector3<T>>> {
        let CurveKind3d::Spline { points } = &self.curve(curve)?.kind else {
            unreachable!("only a spline's ends take a direction");
        };
        for c in self.constraints.values() {
            match *c {
                Constraint3d::TangentTo {
                    curve: k,
                    end: e,
                    direction,
                } if k == curve && e == end => {
                    // Along the direction either way: the way the spline
                    // runs there, from its end point to the next, as drawn.
                    let (from, to) = match end {
                        End::Start => (points[0], points[1]),
                        End::End => (points[points.len() - 2], points[points.len() - 1]),
                    };
                    let run = self.points[&to].at.sub(&self.points[&from].at);
                    let sign = if run.prod_dot(&direction).to_f64() < 0.0 {
                        T::ONE.neg()
                    } else {
                        T::ONE
                    };
                    return Ok(Some(direction.map(|c| c.cast::<T>()).prod_scalar(sign)));
                }
                Constraint3d::Tangent { a, b } if self.adopts(a, b)? == Some((curve, end)) => {
                    let other = if a == curve { b } else { a };
                    let (mine, theirs) = if a == curve {
                        self.shared_end(a, b)?.expect("validated")
                    } else {
                        let (ea, eb) = self.shared_end(a, b)?.expect("validated");
                        (eb, ea)
                    };
                    let tangent = self.tangent_at(other, theirs, geometry)?;
                    // Their way through the joint is mine where one of us
                    // arrives and the other leaves.
                    return Ok(Some(if mine == theirs {
                        tangent.neg()
                    } else {
                        tangent
                    }));
                }
                _ => {}
            }
        }
        Ok(None)
    }

    /// The tangent of the drawn curve `curve` at its end `end`, in its own
    /// direction.
    fn tangent_at<T: Scalar>(
        &self,
        curve: CurveId,
        end: End,
        geometry: &Enclosure3d<T>,
    ) -> GeopResult<Vector3<T>> {
        let pieces = self.curve_nurbs(curve, geometry)?;
        match end {
            End::Start => pieces[0].tangent(T::ZERO),
            End::End => pieces[pieces.len() - 1].tangent(T::ONE),
        }
    }

    /// Points along the curve `curve` as drawn, from its start to its end —
    /// dense enough to draw it.
    pub fn curve_polyline(&self, curve: CurveId) -> GeopResult<Vec<Vector3<S>>> {
        let pieces = self.curve_nurbs(curve, &Enclosure3d::as_drawn(self))?;
        let mut out = Vec::new();
        for piece in &pieces {
            let spans = if piece.degree == 1 {
                1
            } else {
                SAMPLES * (piece.control_points.len() - 1)
            };
            out.pop();
            for i in 0..=spans {
                out.push(piece.evaluate(S::from_ratio(i as i64, spans as i64)?)?);
            }
        }
        Ok(out)
    }

    /// The drawn curves — not construction, not references — joined into
    /// chains where they share a point (see [`Sketch3d::point_classes`]).
    ///
    /// Each chain runs the way its oldest curve (the lowest id) does; a
    /// closed one starts at that curve, an open one at its end before it.
    /// Chains come oldest first. Fails where three curves or more meet at a
    /// point — a branch is no chain — or a single curve closes on itself.
    pub fn chains(&self) -> GeopResult<Vec<Chain3d>> {
        let class = self.point_classes();
        let drawn: Vec<(CurveId, PointId, PointId)> = self
            .curves
            .iter()
            .filter(|(_, c)| c.is_drawn() && !c.construction)
            .filter_map(|(&id, c)| {
                let (s, e) = c.endpoints()?;
                Some((id, class[&s], class[&e]))
            })
            .collect();
        let mut at: BTreeMap<PointId, Vec<CurveId>> = BTreeMap::new();
        for &(id, s, e) in &drawn {
            if s == e {
                return Err(GeopError::new(format!(
                    "curve {id} ends where it starts: a closed chain needs at least two curves"
                )));
            }
            at.entry(s).or_default().push(id);
            at.entry(e).or_default().push(id);
        }
        if let Some((p, curves)) = at.iter().find(|(_, c)| c.len() > 2) {
            return Err(GeopError::new(format!(
                "curves {curves:?} meet at point {p}: a chain does not branch"
            )));
        }
        let ends: BTreeMap<CurveId, (PointId, PointId)> =
            drawn.iter().map(|&(id, s, e)| (id, (s, e))).collect();
        // The other curve at the point `p` of the curve `c`.
        let next = |c: CurveId, p: PointId| at[&p].iter().copied().find(|&k| k != c);
        let mut used = BTreeSet::new();
        let mut chains = Vec::new();
        for &(first, _, _) in &drawn {
            if used.contains(&first) {
                continue;
            }
            // Back from the oldest curve's start to where the chain begins.
            let (mut curve, mut point) = (first, ends[&first].0);
            let mut closed = false;
            while let Some(k) = next(curve, point) {
                if k == first {
                    closed = true;
                    break;
                }
                let (s, e) = ends[&k];
                point = if s == point { e } else { s };
                curve = k;
            }
            let (mut curve, mut from) = if closed {
                (first, ends[&first].0)
            } else {
                (curve, point)
            };
            let mut edges = Vec::new();
            loop {
                let (s, e) = ends[&curve];
                let reversed = s != from;
                edges.push(ProfileEdge { curve, reversed });
                used.insert(curve);
                from = if reversed { s } else { e };
                match next(curve, from) {
                    Some(k) if k != edges[0].curve => curve = k,
                    _ => break,
                }
            }
            chains.push(Chain3d { edges, closed });
        }
        Ok(chains)
    }
}

impl Chain3d {
    /// The chain as NURBS pieces, each on `[0, 1]` and starting exactly
    /// where the one before ends — the sketch's points at `geometry` —
    /// named after the sketch's curves and points (see [`Piece3d`]).
    pub fn to_nurbs<D: Scalar, T: Scalar>(
        &self,
        sketch: &Sketch3d<D>,
        geometry: &Enclosure3d<T>,
    ) -> GeopResult<Vec<Piece3d<T>>> {
        let mut out = Vec::new();
        for edge in &self.edges {
            let curve = edge.curve;
            let pieces = sketch.curve_nurbs(curve, geometry)?;
            let (first, last) = sketch
                .curve(curve)?
                .endpoints()
                .ok_or_else(|| GeopError::new(format!("curve {curve} of a chain has no ends")))?;
            let n = pieces.len();
            let joint = |j: usize| match j {
                0 => ProfileJoint::Point(first),
                j if j == n => ProfileJoint::Point(last),
                index => ProfileJoint::Split { curve, index },
            };
            let pieces = pieces.into_iter().enumerate().map(|(index, c)| Piece3d {
                curve: c,
                source: curve,
                index,
                start: joint(index),
                end: joint(index + 1),
            });
            if edge.reversed {
                out.extend(pieces.rev().map(|p| Piece3d {
                    curve: p.curve.reverse(),
                    start: p.end,
                    end: p.start,
                    ..p
                }));
            } else {
                out.extend(pieces);
            }
        }
        Ok(out)
    }
}
