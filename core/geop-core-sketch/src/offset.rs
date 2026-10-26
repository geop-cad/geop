//! Offsetting chains of lines, arcs and splines — or circles — by a
//! distance, the offset tied to them by constraints so it follows them.
//!
//! **One dimension.** The offset of the chain's first curve is an
//! [`Constraint::Offset`] — the distance, a dimension to give. Every other
//! offset curve is only [`Constraint::Parallel`] to its line or
//! [`Constraint::Concentric`] with its arc, and the distance carries over
//! from one to the next at every corner:
//!
//! - where the chain runs on smoothly — [`Constraint::Tangent`] there, by
//!   constraint — the offsets meet tangentially too;
//! - where it turns away from the offset side, a gap opens: closed by an
//!   arc round the corner, tangent to both offsets ([`Corners::Round`]), or
//!   by extending both offsets until they meet ([`Corners::Extend`]);
//! - where it turns towards the offset side, the offsets cross: both are
//!   cut back to where they meet;
//! - where it runs on straight — the corner's offsets end closer than the
//!   solver tells points apart, as the halves of a mirrored outline meet
//!   on the mirror line — they meet square to the corner.
//!
//! An arc round a corner has the corner as its center, so its radius is the
//! distance of both offsets it touches. Where offsets meet, a construction
//! circle about the corner, touching both, says the same. An open chain's
//! offset ends square to the chain's ends.
//!
//! **Splines.** The offset of a spline is no spline. It is fitted: a cubic
//! spline through points of the true offset, with as many control points
//! as it takes to come within [`FIT_TOLERANCE`] of the sketch's size of it
//! everywhere, proven ([`NurbCurve2D::offset_miss`]). Each of those points
//! is held to the spline, at its parameter, the distance away along its
//! normal ([`Constraint::OffsetAt`]), the distance a construction circle's
//! radius about a corner of the chain — the dimension, if the spline comes
//! first. Where the fit ends at a corner it ends on the true offset, and
//! the curve the chain runs on with meets it there; where it ends where
//! offsets cross, its end is held only across the spline, and slides along
//! the offset to where the other one is.
//!
//! **Crossings.** Where a piece is too short for the corners at its ends,
//! or offsets run into each other — offsetting into a notch narrower than
//! twice the distance — the offset drawn round the chain crosses itself.
//! It is cut where it does, and what lies nearer the chain than the
//! distance dropped: what is left runs on from part to part at the
//! crossings, each part still tied to its source. A part cut off at both
//! ends from the corners that carry the distance holds it by a circle
//! about its source's corner touching it, as large as one about the first
//! part's: one dimension still says how far all of it is.
//!
//! **Several chains at once.** An outline and its holes, offset together,
//! each go into the same area: of two closed chains one inside the other,
//! one inward and the other outward. Each is run the way that has that
//! area on its left, so that where their offsets cross they are cut and
//! run on from one into the other, as a chain's own do; a part of another
//! chain holds the distance as a part cut off does. A circle among them
//! keeps its offset whole, held by a construction circle on it, level with
//! its center, as large as the distance.
//!
//! Refused: an arc offset past its center, a spline offset past its
//! center of curvature, a circle's offset that another crosses, and an
//! offset that leaves nothing.

use std::rc::Rc;

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::solvers::system::RELATIVE_TOLERANCE;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector2,
};
use serde::{Deserialize, Serialize};

use crate::{
    plain::{
        self, P2, Plain, add, angle_between, bounds, cross, distance, dot, perp, scale,
        segments_cross, sub, unit, wrap, xy,
    },
    profile::{Chain, ProfileEdge, basis_at, spline_nurbs},
    sketch::{Constraint, ConstraintId, CurveId, CurveKind, PointId, Sketch},
};

/// How near a spline's fitted offset comes to the true one, at least: this
/// part of the sketch's size. How hard the fit tries; the fit says how near
/// it came.
const FIT_TOLERANCE: f64 = 1e-4;

/// The most control points a spline's fitted offset is given.
const MOST_FIT_POINTS: usize = 256;

/// How finely a spline, or its offset, is sampled to find where it crosses
/// something, and how near a point it comes: each crossing and nearest
/// point is refined on the spline itself after.
const SAMPLES: usize = 256;

/// How an offset closes the gap where its chain turns away from it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Corners {
    /// An arc round the corner.
    #[default]
    Round,
    /// Both offsets extended until they meet.
    Extend,
}

/// An offset made: the curves of the offset, and the dimension of its
/// distance.
#[derive(Clone, Debug, PartialEq)]
pub struct Offsetted {
    pub curves: Vec<CurveId>,
    pub distance: ConstraintId,
}

/// What a part of an offset drawn round a chain is of: the offset of the
/// chain's `i`-th piece, or the arc round the corner after it.
#[derive(Clone, Copy, Debug)]
enum Of {
    Piece(usize),
    Round(usize),
}

/// A part of an offset drawn round a chain, kept where it was cut at its
/// crossings: of the `elem`-th thing drawn, running from its start
/// (`from_start`) or a crossing to its end (`to_end`) or a crossing.
#[derive(Clone, Debug)]
struct Sub {
    elem: usize,
    shape: Shape,
    from_start: bool,
    to_end: bool,
}

/// A spline of the chain moved `distance` to the chain's left — not at
/// all, for the spline itself — in plain numbers: where along it is a
/// fraction `w` of the spline's domain, run the way the chain runs it.
#[derive(Debug)]
struct SplineOffset {
    spline: NurbCurve2D,
    reversed: bool,
    distance: f64,
    /// [`SAMPLES`] points along it, evenly in `w`: where to start looking.
    samples: Vec<P2>,
}

impl SplineOffset {
    fn new(spline: NurbCurve2D, reversed: bool, distance: f64) -> GeopResult<Self> {
        let mut offset = SplineOffset {
            spline,
            reversed,
            distance,
            samples: Vec::new(),
        };
        offset.samples = (0..=SAMPLES)
            .map(|k| offset.at(k as f64 / SAMPLES as f64))
            .collect::<GeopResult<_>>()?;
        Ok(offset)
    }

    /// The spline's own parameter at `w`.
    fn t(&self, w: f64) -> f64 {
        if self.reversed { 1.0 - w } else { w }
    }

    /// The spline's unit tangent at `w`, the way the chain runs.
    fn tangent(&self, w: f64) -> GeopResult<P2> {
        let d = plain::of(&self.spline.tangent(Scalar::from_f64(self.t(w)))?);
        let d = if self.reversed { scale(d, -1.0) } else { d };
        unit(d).ok_or_else(|| GeopError::new("a spline with no tangent"))
    }

    fn at(&self, w: f64) -> GeopResult<P2> {
        let p = plain::of(&self.spline.evaluate(Scalar::from_f64(self.t(w)))?);
        Ok(add(p, scale(perp(self.tangent(w)?), self.distance)))
    }

    /// Where between `lo` and `hi` it comes nearest `q`: the nearest of the
    /// samples, then Newton on the curve itself.
    fn nearest(&self, q: P2, lo: f64, hi: f64) -> f64 {
        let n = SAMPLES as f64;
        let mut best = (f64::INFINITY, lo);
        for k in 0..SAMPLES {
            let (a, b) = (self.samples[k], self.samples[k + 1]);
            let ab = sub(b, a);
            let f = (dot(sub(q, a), ab) / dot(ab, ab).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
            let w = ((k as f64 + f) / n).clamp(lo, hi);
            let d = distance(q, add(a, scale(ab, f)));
            if d < best.0 {
                best = (d, w);
            }
        }
        let mut w = best.1;
        let h = 1e-7;
        for _ in 0..16 {
            let (Ok(p), Ok(a), Ok(b)) = (self.at(w), self.at(w - h), self.at(w + h)) else {
                break;
            };
            let v = scale(sub(b, a), 0.5 / h);
            let step = dot(sub(q, p), v) / dot(v, v).max(f64::MIN_POSITIVE);
            let next = (w + step).clamp(lo, hi);
            if next == w {
                break;
            }
            w = next;
        }
        w
    }
}

/// A piece of the chain or of its offset, in plain numbers: a line, circle
/// or arc, or the part of a spline or its offset from `from` to `to`, each
/// a fraction `w` of the whole (see [`SplineOffset`]).
#[derive(Clone, Debug)]
enum Shape {
    Plain(Plain),
    Spline {
        of: Rc<SplineOffset>,
        from: f64,
        to: f64,
    },
}

impl Shape {
    fn w(from: f64, to: f64, u: f64) -> f64 {
        from + u * (to - from)
    }

    /// The point `u` along it, from 0 at its start to 1 at its end.
    fn at(&self, u: f64) -> P2 {
        match self {
            Shape::Plain(p) => p.at(u),
            Shape::Spline { of, from, to } => of
                .at(Self::w(*from, *to, u))
                .unwrap_or_else(|_| of.samples[0]),
        }
    }

    fn tangent(&self, u: f64) -> P2 {
        match self {
            Shape::Plain(p) => p.tangent(u),
            Shape::Spline { of, from, to } => {
                let t = of.tangent(Self::w(*from, *to, u)).unwrap_or([1.0, 0.0]);
                if to < from { scale(t, -1.0) } else { t }
            }
        }
    }

    /// Where along it `q`, a point of it, is (see [`Plain::param`]).
    fn param(&self, q: P2) -> f64 {
        match self {
            Shape::Plain(p) => p.param(q),
            Shape::Spline { of, from, to } => (of.nearest(q, 0.0, 1.0) - from) / (to - from),
        }
    }

    /// Where along it — in `[0, 1]` — it comes nearest `q`.
    fn nearest(&self, q: P2) -> f64 {
        match self {
            Shape::Plain(p) => p.nearest(q),
            Shape::Spline { of, from, to } => {
                (of.nearest(q, from.min(*to), from.max(*to)) - from) / (to - from)
            }
        }
    }

    fn part(&self, u0: f64, u1: f64) -> Shape {
        match self {
            Shape::Plain(p) => Shape::Plain(p.part(u0, u1)),
            Shape::Spline { of, from, to } => Shape::Spline {
                of: of.clone(),
                from: Self::w(*from, *to, u0),
                to: Self::w(*from, *to, u1),
            },
        }
    }

    /// The whole line or circle it lies on, or the whole spline.
    fn full(&self) -> Shape {
        match self {
            Shape::Plain(p) => Shape::Plain(p.full()),
            Shape::Spline { of, .. } => Shape::Spline {
                of: of.clone(),
                from: 0.0,
                to: 1.0,
            },
        }
    }

    /// Points along it, for finding where it crosses a spline: an
    /// infinite line as a segment far longer than anything near it.
    fn polyline(&self) -> Vec<P2> {
        match self {
            Shape::Plain(Plain::Line {
                a,
                b,
                infinite: true,
            }) => {
                let far = scale(sub(*b, *a), 1e6);
                vec![sub(*a, far), add(*b, far)]
            }
            Shape::Plain(p) => p.polyline(0.0, 1.0),
            Shape::Spline { of, from, to } => {
                let (lo, hi) = (from.min(*to), from.max(*to));
                let n = SAMPLES as f64;
                let mut ws = vec![lo];
                ws.extend(
                    ((lo * n).floor() as usize + 1..=(hi * n).ceil() as usize)
                        .map(|k| k as f64 / n)
                        .filter(|&w| w > lo && w < hi),
                );
                ws.push(hi);
                ws.into_iter()
                    .map(|w| of.at(w).unwrap_or(of.samples[0]))
                    .collect()
            }
        }
    }

    /// Where it and `other` cross, kept where both run: on a spline,
    /// found on the samples and refined on the curves.
    fn crossings(&self, other: &Shape) -> Vec<P2> {
        let (Shape::Plain(a), Shape::Plain(b)) = (self, other) else {
            let (p, q) = (self.polyline(), other.polyline());
            let mut found = Vec::new();
            for s in p.windows(2) {
                for t in q.windows(2) {
                    if let Some(x) = segments_cross(s[0], s[1], t[0], t[1]) {
                        found.push(refined(self, other, x));
                    }
                }
            }
            return found;
        };
        a.crossings(b, false)
    }
}

/// Where `a` and `b` cross near `x`: Newton on the two curves, each
/// stepped along its tangent.
fn refined(a: &Shape, b: &Shape, x: P2) -> P2 {
    let (mut u, mut v) = (a.param(x), b.param(x));
    let h = 1e-7;
    for _ in 0..16 {
        let (p, q) = (a.at(u), b.at(v));
        let da = scale(sub(a.at(u + h), a.at(u - h)), 0.5 / h);
        let db = scale(sub(b.at(v + h), b.at(v - h)), 0.5 / h);
        // p + da du = q + db dv.
        let r = sub(q, p);
        let det = cross(da, db);
        if det == 0.0 {
            break;
        }
        let (du, dv) = (cross(r, db) / det, cross(r, da) / det);
        if du == 0.0 && dv == 0.0 {
            break;
        }
        (u, v) = (u + du, v + dv);
    }
    a.at(u)
}

/// A piece of a chain, along the chain's direction, in plain numbers,
/// moved `d` to its left.
fn oriented(sketch: &Sketch, curve: CurveId, reversed: bool, d: f64) -> GeopResult<Shape> {
    let plain = match &sketch.curve(curve)?.kind {
        CurveKind::Line { start, end } => {
            let (a, b) = (xy(sketch, *start), xy(sketch, *end));
            let (a, b) = if reversed { (b, a) } else { (a, b) };
            Plain::Line {
                a,
                b,
                infinite: false,
            }
        }
        CurveKind::Arc { start, end, sweep } => {
            let (s, e, sweep) = (xy(sketch, *start), xy(sketch, *end), sweep.to_f64());
            let arc = if reversed {
                Plain::arc(e, s, -sweep)
            } else {
                Plain::arc(s, e, sweep)
            };
            arc.ok_or_else(|| {
                GeopError::new(format!("arc {curve} is too straight to have a center"))
            })?
        }
        CurveKind::Circle { center, radius } => Plain::Round {
            center: xy(sketch, *center),
            radius: radius.to_f64(),
            span: None,
        },
        CurveKind::Spline {
            control_points,
            shape,
        } => {
            if shape.as_ref().is_some_and(|s| s.degree > 3) {
                return Err(GeopError::new(format!(
                    "spline {curve} is of degree {}: only splines of degree three or less can be offset",
                    shape.as_ref().map_or(0, |s| s.degree)
                )));
            }
            let points: Vec<Vector2> = control_points
                .iter()
                .map(|p| sketch.points[p].xy())
                .collect();
            let spline = spline_nurbs(&points, shape.as_ref())?;
            let of = SplineOffset::new(spline, reversed, d)?;
            // Past a center of curvature the offset turns back on itself.
            for k in 0..SAMPLES {
                let w = (k as f64 + 0.5) / SAMPLES as f64;
                let along = sub(of.samples[k + 1], of.samples[k]);
                if dot(along, of.tangent(w)?) <= 0.0 {
                    return Err(GeopError::new(format!(
                        "spline {curve} cannot be offset by {d}: it bends tighter than that, and its offset would turn back on itself"
                    )));
                }
            }
            return Ok(Shape::Spline {
                of: Rc::new(of),
                from: 0.0,
                to: 1.0,
            });
        }
    };
    if d == 0.0 {
        return Ok(Shape::Plain(plain));
    }
    offset_plain(plain, d).map(Shape::Plain).ok_or_else(|| {
        GeopError::new(format!(
            "{curve} cannot be offset by {d} towards its center: it is smaller than that"
        ))
    })
}

/// `piece`'s offset by `d` to its left: none for an arc or circle offset
/// past its center.
fn offset_plain(piece: Plain, d: f64) -> Option<Plain> {
    Some(match piece {
        Plain::Line { a, b, infinite } => {
            let n = scale(perp(unit(sub(b, a))?), d);
            Plain::Line {
                a: add(a, n),
                b: add(b, n),
                infinite,
            }
        }
        Plain::Round {
            center,
            radius,
            span,
        } => {
            // To the left is towards the center going counter-clockwise.
            let turn = span.map_or(1.0, |(_, sweep)| sweep.signum());
            let radius = radius - turn * d;
            if radius <= 0.0 {
                return None;
            }
            Plain::Round {
                center,
                radius,
                span,
            }
        }
    })
}

/// Where along `plain` — as [`Plain::param`], but going on past its ends
/// rather than round — the point `q` of its full line or circle is,
/// reckoned from its start for `from_start`, else from its end.
fn unwrapped(plain: &Plain, q: P2, from_start: bool) -> f64 {
    match *plain {
        Plain::Line { .. } => plain.param(q),
        Plain::Round {
            center,
            span: Some((start, sweep)),
            ..
        } => {
            let r = sub(q, center);
            let at = r[1].atan2(r[0]);
            let base = if from_start { start } else { start + sweep };
            let off = wrap(at - base) * sweep.signum() / sweep.abs();
            if from_start { off } else { 1.0 + off }
        }
        Plain::Round { span: None, .. } => 0.0,
    }
}

/// A spline fitted to an offset: its control points, and where it passes
/// through the offset — its own parameter there, and the offset spline's.
type Fit = (Vec<P2>, Vec<(f64, f64)>);

/// A cubic spline fitted to the offset by `d` to its left of `spline`
/// from `t0` to `t1` of its domain: its control points, and the
/// parameters at which it passes through the offset, each with the
/// spline's parameter there — as many as it takes to come within
/// `tolerance` of the offset everywhere (see the module docs).
fn fitted(
    spline: &NurbCurve2D,
    d: f64,
    (t0, t1): (f64, f64),
    tolerance: f64,
    first: usize,
) -> GeopResult<Fit> {
    let mut m = first.max(4);
    loop {
        let knots = cubic_knots(m)?;
        let breaks: Vec<f64> = knots.iter().map(|k| k.to_f64()).collect();
        let spline_breaks: Vec<f64> = spline.breakpoints().iter().map(|k| k.to_f64()).collect();
        // Where it passes through the offset: its ends, and the Greville
        // points between, moved off any knot of either, where a point of a
        // spline depends on more control points than in a span.
        let mut at: Vec<(f64, f64)> = Vec::with_capacity(m);
        for k in 0..m {
            let mut s = (breaks[k + 1] + breaks[k + 2] + breaks[k + 3]) / 3.0;
            if k > 0 && k + 1 < m {
                let gap = 1.0 / (m - 3) as f64 / 8.0;
                let on_knot = |s: f64| {
                    let t = t0 + s * (t1 - t0);
                    breaks.iter().any(|&b| (b - s).abs() < gap)
                        || spline_breaks
                            .iter()
                            .any(|&b| (b - t).abs() < gap * (t1 - t0).abs())
                };
                for nudge in [0.0, 1.0, -1.0, 2.0, -2.0] {
                    if !on_knot(s + nudge * gap) {
                        s += nudge * gap;
                        break;
                    }
                }
            }
            at.push((s, t0 + s * (t1 - t0)));
        }
        let target = |t: f64| -> GeopResult<P2> {
            let t = Scalar::from_f64(t);
            let p = plain::of(&spline.evaluate(t)?);
            let n = perp(
                unit(plain::of(&spline.tangent(t)?))
                    .ok_or_else(|| GeopError::new("a spline with no tangent"))?,
            );
            Ok(add(p, scale(n, d)))
        };
        // The collocation system: row k the basis at `at[k].0`.
        let mut rows: Vec<Vec<f64>> = Vec::with_capacity(m);
        for &(s, _) in &at {
            let mut row = vec![0.0; m];
            for (j, value, _) in basis_at(m, None, Scalar::from_f64(s), false)? {
                row[j] = value.to_f64();
            }
            rows.push(row);
        }
        let targets: Vec<P2> = at
            .iter()
            .map(|&(_, t)| target(t))
            .collect::<GeopResult<_>>()?;
        let control = solve_dense(rows, targets)?;
        let fit = spline_nurbs(
            &control
                .iter()
                .map(|p| Vector2::from_array(p.map(Scalar::from_f64)))
                .collect::<Vec<_>>(),
            None,
        )?;
        let miss = spline.offset_miss(
            Scalar::from_f64(d),
            (Scalar::from_f64(t0), Scalar::from_f64(t1)),
            &fit,
            Scalar::from_f64(tolerance),
        )?;
        if !miss.definitely_greater(Scalar::from_f64(tolerance)) {
            return Ok((control, at));
        }
        if 2 * m > MOST_FIT_POINTS {
            return Err(GeopError::new(format!(
                "a spline's offset by {d} is not fitted within {tolerance} by {m} control points: it is {miss:?} off"
            )));
        }
        m *= 2;
    }
}

/// The clamped uniform cubic knots of `m` control points, as a drawn
/// spline has them.
fn cubic_knots(m: usize) -> GeopResult<Vec<Scalar>> {
    let spans = m - 3;
    let mut knots = vec![Scalar::ZERO; 4];
    for i in 1..spans {
        knots.push(Scalar::from_ratio(i as i64, spans as i64)?);
    }
    knots.extend([Scalar::ONE; 4]);
    Ok(knots)
}

/// `x` with `rows · x = targets`, by elimination with partial pivoting.
fn solve_dense(mut rows: Vec<Vec<f64>>, mut targets: Vec<P2>) -> GeopResult<Vec<P2>> {
    let m = rows.len();
    for c in 0..m {
        let pivot = (c..m)
            .max_by(|&a, &b| rows[a][c].abs().total_cmp(&rows[b][c].abs()))
            .expect("a row");
        if rows[pivot][c] == 0.0 {
            return Err(GeopError::new("the fit's collocation system is singular"));
        }
        rows.swap(c, pivot);
        targets.swap(c, pivot);
        for r in 0..m {
            if r != c && rows[r][c] != 0.0 {
                let f = rows[r][c] / rows[c][c];
                let pivot_row = rows[c].clone();
                for (x, p) in rows[r][c..].iter_mut().zip(&pivot_row[c..]) {
                    *x -= f * p;
                }
                targets[r] = sub(targets[r], scale(targets[c], f));
            }
        }
    }
    Ok((0..m)
        .map(|c| scale(targets[c], 1.0 / rows[c][c]))
        .collect())
}

/// Per chain of `chains`, the distance its offset goes to its own left:
/// the first's `d`, and each other's into the same area as the first's —
/// of two closed chains one inside the other (an outline and a hole) one
/// inward and the other outward, as they run the same way round or not;
/// of two side by side both inward or both outward. An open chain goes to
/// its own left as the first does.
fn sides(sketch: &Sketch, chains: &[Chain], d: f64) -> GeopResult<Vec<f64>> {
    // Each closed chain as a polygon of its pieces as drawn.
    let mut polygons: Vec<Option<Vec<P2>>> = Vec::new();
    for chain in chains {
        if !chain.closed {
            polygons.push(None);
            continue;
        }
        let mut points = Vec::new();
        for e in &chain.edges {
            let piece = oriented(sketch, e.curve, e.reversed, 0.0)?;
            for k in 0..32 {
                points.push(piece.at(k as f64 / 32.0));
            }
        }
        polygons.push(Some(points));
    }
    let area = |q: &[P2]| {
        (0..q.len())
            .map(|k| cross(q[k], q[(k + 1) % q.len()]))
            .sum::<f64>()
    };
    let inside = |p: P2, q: &[P2]| {
        let mut odd = false;
        for k in 0..q.len() {
            let (a, b) = (q[k], q[(k + 1) % q.len()]);
            if (a[1] > p[1]) != (b[1] > p[1])
                && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
            {
                odd = !odd;
            }
        }
        odd
    };
    // How many of the other closed chains a closed one lies inside.
    let depth = |c: usize| -> usize {
        let Some(q) = &polygons[c] else {
            return 0;
        };
        (0..chains.len())
            .filter(|&o| o != c)
            .filter(|&o| polygons[o].as_ref().is_some_and(|r| inside(q[0], r)))
            .count()
    };
    let (Some(first), base) = (&polygons[0], depth(0)) else {
        return Ok(vec![d; chains.len()]);
    };
    let turn = area(first).signum();
    Ok((0..chains.len())
        .map(|c| match &polygons[c] {
            Some(q) if c > 0 => {
                let same = area(q).signum() == turn;
                let nested = (depth(c) + base) % 2 == 1;
                if same != nested { d } else { -d }
            }
            _ => d,
        })
        .collect())
}

impl Sketch {
    /// How far `p` is from the chain `chain`: positive to its left.
    pub fn chain_side(&self, chain: &Chain, p: P2) -> GeopResult<f64> {
        let mut best: Option<(f64, f64)> = None;
        for edge in &chain.edges {
            let piece = oriented(self, edge.curve, edge.reversed, 0.0)?;
            let t = piece.nearest(p);
            let q = piece.at(t);
            let d = distance(p, q);
            if best.is_none_or(|(b, _)| d < b) {
                let side = cross(piece.tangent(t), sub(p, q)).signum();
                best = Some((d, side * d));
            }
        }
        Ok(best.map_or(0.0, |(_, signed)| signed))
    }

    /// Offsets the chains `curves` (see [`Sketch::chains`]) by `distance`
    /// — the first to its left, to its right if negative, and every other
    /// one into the same area: away from the first where it lies inside or
    /// round it, as a hole's or an outline's — its corners as `corners`
    /// says (see the module docs). Returns the curves made, and the
    /// dimension of the distance.
    pub fn offset(
        &mut self,
        curves: &[CurveId],
        distance: Scalar,
        corners: Corners,
    ) -> GeopResult<Offsetted> {
        let chains = self.chains(curves)?;
        let d = distance.to_f64();
        if d == 0.0 || !d.is_finite() {
            return Err(GeopError::new(format!("cannot offset by {d}")));
        }
        let value = distance.abs();

        // A circle alone: one about the same center.
        if let [chain] = &chains[..]
            && let [edge] = &chain.edges[..]
            && let CurveKind::Circle { center, radius } = self.curves[&edge.curve].kind
        {
            let c = edge.curve;
            let r = radius.to_f64() - d;
            if r <= 0.0 {
                return Err(GeopError::new(format!(
                    "{c} cannot be offset by {d} towards its center: it is smaller than that"
                )));
            }
            let circle = self.add_circle(center, Scalar::from_f64(r));
            let distance = self.constrain(Constraint::Offset {
                a: c,
                b: circle,
                value,
            });
            return Ok(Offsetted {
                curves: vec![circle],
                distance,
            });
        }
        // Each chain's distance to its own left: into the area the first
        // one's offset goes into.
        let towards = sides(self, &chains, d)?;
        // Each run the way that has that area on the same side as the
        // first's: then the offsets, cut where they cross, run on from one
        // chain's into another's.
        let chains: Vec<Chain> = (chains.into_iter().zip(&towards))
            .map(|(chain, &side)| {
                if side.signum() == d.signum() {
                    return chain;
                }
                Chain {
                    edges: (chain.edges.into_iter().rev())
                        .map(|e| ProfileEdge {
                            curve: e.curve,
                            reversed: !e.reversed,
                        })
                        .collect(),
                    closed: chain.closed,
                }
            })
            .collect();
        let towards = vec![d; chains.len()];
        // Circles apart: whole, about their centers.
        let (rings, chains): (Vec<_>, Vec<_>) = chains.into_iter().zip(towards).partition(
            |(chain, _)| {
                matches!(&chain.edges[..], [edge] if matches!(self.curves[&edge.curve].kind, CurveKind::Circle { .. }))
            },
        );
        let mut pieces_of: Vec<(CurveId, bool)> = Vec::new();
        let mut along: Vec<f64> = Vec::new();
        let mut chain_of: Vec<(usize, usize, bool)> = Vec::new();
        for (chain, d) in &chains {
            let start = pieces_of.len();
            for e in &chain.edges {
                pieces_of.push((e.curve, e.reversed));
                along.push(*d);
                chain_of.push((start, chain.edges.len(), chain.closed));
            }
        }
        let n = pieces_of.len();
        // The piece after `i` in its chain, and before.
        let next_piece = |i: usize| -> Option<usize> {
            let (start, len, closed) = chain_of[i];
            if i + 1 < start + len {
                Some(i + 1)
            } else {
                closed.then_some(start)
            }
        };
        let previous_piece = |i: usize| -> Option<usize> {
            let (start, len, closed) = chain_of[i];
            if i > start {
                Some(i - 1)
            } else {
                closed.then_some(start + len - 1)
            }
        };
        // The finest the solver tells points apart, in this sketch: one
        // part in `RELATIVE_TOLERANCE` of its size.
        let size = {
            let size = bounds(self.points.keys().map(|&p| xy(self, p)))
                .map_or(0.0, |(lo, hi)| (hi[0] - lo[0]).hypot(hi[1] - lo[1]));
            // As the solver: one, where the sketch has no size at all.
            if size > 0.0 { size } else { 1.0 }
        };
        let resolution = size / RELATIVE_TOLERANCE as f64;
        let pieces: Vec<Shape> = pieces_of
            .iter()
            .map(|&(c, r)| oriented(self, c, r, 0.0))
            .collect::<GeopResult<_>>()?;
        let offsets: Vec<Shape> = (pieces_of.iter().zip(&along))
            .map(|(&(c, r), &d)| oriented(self, c, r, d))
            .collect::<GeopResult<_>>()?;
        // The circles and their offsets.
        let mut circles: Vec<(CurveId, Shape, Shape)> = Vec::new();
        for (chain, d) in &rings {
            // A circle runs counter-clockwise; run the other way, its left
            // is outward.
            let c = chain.edges[0].curve;
            let d = if chain.edges[0].reversed { -*d } else { *d };
            circles.push((
                c,
                oriented(self, c, false, 0.0)?,
                oriented(self, c, false, d)?,
            ));
        }
        let spline = |i: usize| matches!(pieces[i], Shape::Spline { .. });
        // Each joint: where the offsets of the pieces before and after it
        // end and start, and what joins them.
        enum Joint {
            /// They meet at one point, held there as `by` says.
            Shared { at: P2, by: Held },
            /// An arc round the corner, turning by `sweep`.
            Round { from: P2, to: P2, sweep: f64 },
        }
        /// What keeps offsets meeting at one point at one distance.
        #[derive(PartialEq)]
        enum Held {
            /// They run on smoothly: tangent there too.
            Tangent,
            /// A construction circle about the corner touching both.
            Circle,
            /// They run on straight: the point square to the corner, as an
            /// open end is.
            Square,
        }
        let mut joint_of: Vec<Option<Joint>> = Vec::new();
        for i in 0..n {
            let Some(j) = next_piece(i) else {
                joint_of.push(None);
                continue;
            };
            let d = along[i];
            let (ci, cj) = (pieces_of[i].0, pieces_of[j].0);
            let p = pieces[i].at(1.0);
            let (ti, tj) = (pieces[i].tangent(1.0), pieces[j].tangent(0.0));
            let normal = perp(ti);
            let smooth = self.tangent(ci, cj);
            if smooth && dot(ti, tj) < 0.0 {
                return Err(GeopError::new(format!(
                    "{ci} and {cj} turn back on each other where they meet: their offsets would cross"
                )));
            }
            let turn = cross(ti, tj);
            // How far apart the offsets end at the corner: no further than
            // the solver tells points apart, and there is no corner to
            // speak of — two lines of a mirrored outline meeting on the
            // mirror line, say.
            let (from, to) = (add(p, scale(normal, d)), add(p, scale(perp(tj), d)));
            let straight = plain::distance(from, to) <= resolution && dot(ti, tj) > 0.0;
            let lines = matches!(
                (&offsets[i], &offsets[j]),
                (
                    Shape::Plain(Plain::Line { .. }),
                    Shape::Plain(Plain::Line { .. })
                )
            );
            joint_of.push(Some(if smooth {
                Joint::Shared {
                    at: from,
                    by: Held::Tangent,
                }
            } else if straight {
                Joint::Shared {
                    at: from,
                    by: Held::Square,
                }
            } else if turn * d < 0.0 && corners == Corners::Round {
                Joint::Round {
                    from,
                    to,
                    sweep: angle_between(ti, tj),
                }
            } else if lines {
                // Where the offset lines, extended, meet: along the
                // corner's bisector, well defined however small the turn.
                if dot(ti, tj) <= -1.0 {
                    return Err(GeopError::new(format!(
                        "{ci} and {cj} fold back on each other where they meet: their offsets would cross"
                    )));
                }
                Joint::Shared {
                    at: add(p, scale(add(normal, perp(tj)), d / (1.0 + dot(ti, tj)))),
                    by: Held::Circle,
                }
            } else {
                // Where the offsets, extended, meet — the crossing nearest
                // the corner's own offset.
                let near = scale(add(from, to), 0.5);
                let crossings = offsets[i].full().crossings(&offsets[j].full());
                let Some(at) = crossings
                    .into_iter()
                    .min_by(|a, b| plain::distance(*a, near).total_cmp(&plain::distance(*b, near)))
                else {
                    return Err(GeopError::new(format!(
                        "the offsets of {ci} and {cj} by {d} do not meet: round their corner instead"
                    )));
                };
                Joint::Shared {
                    at,
                    by: Held::Circle,
                }
            }));
        }
        // Whether the offsets meet where they cross, cut back or carried on
        // to there, at the joint `k`: not at the offset of a corner.
        let crossed = |k: usize| {
            matches!(
                joint_of[k],
                Some(Joint::Shared {
                    by: Held::Circle,
                    ..
                })
            )
        };
        let start_crossed = |i: usize| previous_piece(i).is_some_and(crossed);
        let end_crossed = crossed;

        // Where each offset piece starts and ends.
        let start_of = |i: usize| -> P2 {
            match previous_piece(i).and_then(|k| joint_of[k].as_ref()) {
                None => offsets[i].at(0.0),
                Some(Joint::Shared { at, .. }) => *at,
                Some(Joint::Round { to, .. }) => *to,
            }
        };
        let end_of = |i: usize| -> P2 {
            match &joint_of[i] {
                None => offsets[i].at(1.0),
                Some(Joint::Shared { at, .. }) => *at,
                Some(Joint::Round { from, .. }) => *from,
            }
        };
        // The offset drawn round the chain, before it is trimmed: each
        // piece's offset between where it starts and ends — the wrong way
        // round where the piece is too short for its corners — and the arcs
        // round the corners between them, in order.
        let mut path: Vec<(Shape, Of)> = Vec::new();
        // The element after each along its chain's offset.
        let mut path_next: Vec<Option<usize>> = Vec::new();
        let mut chain_start = 0;
        for (i, offset) in offsets.iter().enumerate() {
            if chain_of[i].0 == i {
                chain_start = path.len();
            }
            let (s, e) = (start_of(i), end_of(i));
            let piece = match offset {
                Shape::Plain(Plain::Line { .. }) => {
                    (plain::distance(s, e) > resolution).then_some(Shape::Plain(Plain::Line {
                        a: s,
                        b: e,
                        infinite: false,
                    }))
                }
                Shape::Plain(
                    plain @ Plain::Round {
                        span: Some((_, sweep)),
                        ..
                    },
                ) => {
                    let (u, v) = (unwrapped(plain, s, true), unwrapped(plain, e, false));
                    Plain::arc(s, e, sweep * (v - u)).map(Shape::Plain)
                }
                Shape::Spline { of, .. } => {
                    // At the offset of the spline's own end, exactly that.
                    let from = if start_crossed(i) {
                        of.nearest(s, 0.0, 1.0)
                    } else {
                        0.0
                    };
                    let to = if end_crossed(i) {
                        of.nearest(e, 0.0, 1.0)
                    } else {
                        1.0
                    };
                    (plain::distance(s, e) > resolution).then(|| Shape::Spline {
                        of: of.clone(),
                        from,
                        to,
                    })
                }
                _ => None,
            };
            // A piece of no length between its corners leaves its
            // neighbours meeting there.
            if let Some(piece) = piece {
                path.push((piece, Of::Piece(i)));
            }
            if let Some(Joint::Round { from, to, sweep }) = &joint_of[i]
                && let Some(arc) = Plain::arc(*from, *to, *sweep)
            {
                path.push((Shape::Plain(arc), Of::Round(i)));
            }
            let (start, len, closed) = chain_of[i];
            if i + 1 == start + len {
                for k in chain_start..path.len() {
                    path_next.push(if k + 1 < path.len() {
                        Some(k + 1)
                    } else {
                        (closed && path.len() > chain_start).then_some(chain_start)
                    });
                }
            }
        }
        let mut path_previous: Vec<Option<usize>> = vec![None; path.len()];
        for (k, next) in path_next.iter().enumerate() {
            if let Some(next) = *next {
                path_previous[next] = Some(k);
            }
        }
        // A circle's offset is kept whole: one that another offset crosses
        // would be cut, which is not supported.
        for (k, (c, _, ring)) in circles.iter().enumerate() {
            let crossed = path
                .iter()
                .any(|(shape, _)| !shape.crossings(ring).is_empty())
                || (circles.iter().enumerate())
                    .any(|(j, (_, _, other))| j != k && !ring.crossings(other).is_empty());
            if crossed {
                return Err(GeopError::new(format!(
                    "the offset of circle {c} by {d} would cross another offset: cutting a circle's offset is not supported"
                )));
            }
        }

        // Cut where it crosses itself, and keep what is no nearer the chain
        // than the distance: what a piece too short for its corners, or
        // offsets running into each other, leave of the offset.
        let near_end = |shape: &Shape, q: P2| {
            plain::distance(shape.at(0.0), q) <= resolution
                || plain::distance(shape.at(1.0), q) <= resolution
        };
        let to_chain = |q: P2| {
            (pieces.iter().chain(circles.iter().map(|(_, c, _)| c)))
                .map(|p| plain::distance(q, p.at(p.nearest(q))))
                .fold(f64::INFINITY, f64::min)
        };
        let mut subs: Vec<Sub> = Vec::new();
        for (k, (shape, _)) in path.iter().enumerate() {
            let mut cuts: Vec<f64> = Vec::new();
            for (j, (other, _)) in path.iter().enumerate() {
                if j != k {
                    cuts.extend(
                        shape
                            .crossings(other)
                            .into_iter()
                            .filter(|&q| !near_end(shape, q))
                            .map(|q| shape.param(q))
                            .filter(|u| (0.0..=1.0).contains(u)),
                    );
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|a, b| plain::distance(shape.at(*a), shape.at(*b)) <= resolution);
            let ts: Vec<f64> = std::iter::once(0.0)
                .chain(cuts)
                .chain(std::iter::once(1.0))
                .collect();
            for w in ts.windows(2) {
                let part = shape.part(w[0], w[1]);
                if to_chain(part.at(0.5)) > d.abs() - resolution {
                    subs.push(Sub {
                        elem: k,
                        shape: part,
                        from_start: w[0] == 0.0,
                        to_end: w[1] == 1.0,
                    });
                }
            }
        }
        if subs.is_empty() && circles.is_empty() {
            return Err(GeopError::new(format!(
                "the offset by {d} leaves nothing: the chain is too short or narrow for it"
            )));
        }

        // Each part on to the one starting where it ends: the next of its
        // piece's own along the path where it reaches its end, else the one
        // starting at the crossing it was cut at.
        let mut next: Vec<Option<usize>> = vec![None; subs.len()];
        for a in 0..subs.len() {
            let end = subs[a].shape.at(1.0);
            let open_end = subs[a].to_end && path_next[subs[a].elem].is_none();
            let found: Vec<usize> = (0..subs.len())
                .filter(|&b| {
                    let joined = if subs[a].to_end {
                        subs[b].from_start && Some(subs[b].elem) == path_next[subs[a].elem]
                    } else {
                        !subs[b].from_start
                    };
                    b != a && joined && plain::distance(subs[b].shape.at(0.0), end) <= resolution
                })
                .collect();
            match (found.as_slice(), open_end) {
                (&[b], false) => next[a] = Some(b),
                ([], true) => {}
                _ => {
                    return Err(GeopError::new(format!(
                        "the offset by {d}, trimmed where it crosses itself, does not run on from {end:?} as one curve"
                    )));
                }
            }
        }
        let mut previous: Vec<Option<usize>> = vec![None; subs.len()];
        for (a, b) in next.iter().enumerate() {
            if let Some(b) = *b {
                if previous[b].is_some() {
                    return Err(GeopError::new(format!(
                        "the offset by {d}, trimmed where it crosses itself, runs into itself at {:?}",
                        subs[b].shape.at(0.0)
                    )));
                }
                previous[b] = Some(a);
            }
        }
        // What joins a part to the next across a corner of the chain, not
        // a crossing: the joint after the piece or arc it is of.
        let joint_after = |a: usize| -> Option<usize> {
            if !subs[a].to_end {
                return None;
            }
            match path[subs[a].elem].1 {
                Of::Piece(i) | Of::Round(i) => Some(i),
            }
        };
        // Whether the part `a` is of a spline's offset.
        let of_spline = |a: usize| matches!(path[subs[a].elem].1, Of::Piece(i) if spline(i));

        // Built: a point where each part ends, shared with the next; then
        // the parts, each tied to its source; then what joins them.
        let mut start_point: Vec<Option<PointId>> = vec![None; subs.len()];
        let mut end_point: Vec<Option<PointId>> = vec![None; subs.len()];
        for a in 0..subs.len() {
            let p = self.new_point_at(subs[a].shape.at(1.0));
            end_point[a] = Some(p);
            if let Some(b) = next[a] {
                start_point[b] = Some(p);
            }
        }
        for a in 0..subs.len() {
            if start_point[a].is_none() {
                start_point[a] = Some(self.new_point_at(subs[a].shape.at(0.0)));
            }
        }
        let corner_of = |sketch: &Sketch, i: usize| -> PointId {
            let (source, reversed) = pieces_of[i];
            let (s, e) = sketch.curves[&source].endpoints().expect("an open curve");
            if reversed { s } else { e }
        };
        let first = (0..subs.len()).find(|&a| matches!(path[subs[a].elem].1, Of::Piece(_)));
        if first.is_none() && !subs.is_empty() {
            return Err(GeopError::new(format!(
                "the offset by {d} leaves only arcs round corners: the chain is too short for it"
            )));
        }
        let first_piece = first.map(|first| match path[subs[first].elem].1 {
            Of::Piece(i) | Of::Round(i) => i,
        });
        // A construction circle about the first part's corner, as large as
        // the distance: what a spline's offset, and a part cut off from the
        // corners, is held at.
        let mut reach: Option<CurveId> = None;
        if let Some(first_piece) = first_piece
            && (0..subs.len()).any(of_spline)
        {
            let circle = self.add_circle(corner_of(self, first_piece), value);
            self.set_construction(circle, true);
            reach = Some(circle);
        }
        let mut curve_of: Vec<CurveId> = Vec::with_capacity(subs.len());
        let mut made = Vec::new();
        let mut distance = None;
        for a in 0..subs.len() {
            let (s, e) = (start_point[a].unwrap(), end_point[a].unwrap());
            let curve = match path[subs[a].elem].1 {
                Of::Piece(i) => {
                    let (source, reversed) = pieces_of[i];
                    // Each offset runs the way its source does.
                    let (s, e) = if reversed { (e, s) } else { (s, e) };
                    let curve = match (&self.curves[&source].kind, &subs[a].shape) {
                        (CurveKind::Line { .. }, _) => self.add_line(s, e),
                        (
                            CurveKind::Arc { .. },
                            Shape::Plain(Plain::Round {
                                span: Some((_, sweep)),
                                ..
                            }),
                        ) => self.add_arc(
                            s,
                            e,
                            Scalar::from_f64(if reversed { -sweep } else { *sweep }),
                        ),
                        (CurveKind::Spline { .. }, Shape::Spline { of, from, to }) => {
                            // Run along the spline, from `ta` to `tb` of it,
                            // its ends held across it only where they are
                            // at crossings.
                            let (ta, tb) = if reversed {
                                (of.t(*to), of.t(*from))
                            } else {
                                (of.t(*from), of.t(*to))
                            };
                            let ends_pinned = [
                                subs[a].from_start && !start_crossed(i),
                                subs[a].to_end && !end_crossed(i),
                            ];
                            let ends_pinned = if reversed {
                                [ends_pinned[1], ends_pinned[0]]
                            } else {
                                ends_pinned
                            };
                            let dist = if reversed { -along[i] } else { along[i] };
                            let tolerance = FIT_TOLERANCE * size;
                            let (control, at) = fitted(
                                &of.spline,
                                dist,
                                (ta, tb),
                                tolerance,
                                4 * of.spline.control_points.len(),
                            )?;
                            let last = control.len() - 1;
                            let mut points = vec![s];
                            for p in &control[1..last] {
                                points.push(self.new_point_at(*p));
                            }
                            points.push(e);
                            let fit = self.add_spline(points);
                            let by = reach.expect("made for a spline's offset");
                            for (k, &(on, at)) in at.iter().enumerate() {
                                let across =
                                    (k == 0 && !ends_pinned[0]) || (k == last && !ends_pinned[1]);
                                self.constrain(Constraint::OffsetAt {
                                    a: source,
                                    at: Scalar::from_f64(at),
                                    b: fit,
                                    on: Scalar::from_f64(on),
                                    by,
                                    across,
                                });
                            }
                            fit
                        }
                        (kind, _) => unreachable!("chained and trimmed as it is: {kind:?}"),
                    };
                    if distance.is_none() {
                        distance = Some(match self.curves[&source].kind {
                            CurveKind::Spline { .. } => self.constrain(Constraint::Radius {
                                curve: reach.expect("made for a spline's offset"),
                                value,
                            }),
                            _ => self.constrain(Constraint::Offset {
                                a: source,
                                b: curve,
                                value,
                            }),
                        });
                    } else {
                        match self.curves[&source].kind {
                            CurveKind::Line { .. } => {
                                self.constrain(Constraint::Parallel {
                                    a: source,
                                    b: curve,
                                });
                            }
                            CurveKind::Spline { .. } => {}
                            _ => {
                                self.constrain(Constraint::Concentric {
                                    a: source,
                                    b: curve,
                                });
                            }
                        }
                    }
                    curve
                }
                Of::Round(i) => {
                    let Shape::Plain(Plain::Round {
                        span: Some((_, sweep)),
                        ..
                    }) = subs[a].shape
                    else {
                        unreachable!("an arc round a corner");
                    };
                    let arc = self.add_arc(s, e, Scalar::from_f64(sweep));
                    let corner = corner_of(self, i);
                    self.constrain(Constraint::Center {
                        point: corner,
                        curve: arc,
                    });
                    arc
                }
            };
            curve_of.push(curve);
            made.push(curve);
        }
        // The circle the splines' offsets are held at, as large as the
        // first part is from its source.
        if let (Some(circle), Some(first)) = (reach, first)
            && !of_spline(first)
        {
            self.constrain(Constraint::Tangent {
                a: curve_of[first],
                b: circle,
            });
        }
        // Across the chain's corners, as the offsets meet there; a part
        // joined so to one whose distance is held holds it too. A spline's
        // offset ends where its own points hold it: at the offset of the
        // corner, or across it where offsets cross, and the other part's
        // own distance places it there.
        let mut held: Vec<bool> = (0..subs.len()).map(of_spline).collect();
        if let Some(first) = first {
            held[first] = true;
        }
        let mut joined: Vec<(usize, usize)> = Vec::new();
        for a in 0..subs.len() {
            let (Some(b), Some(i)) = (next[a], joint_after(a)) else {
                continue;
            };
            if !subs[b].from_start {
                continue;
            }
            joined.push((a, b));
            let (oa, ob) = (curve_of[a], curve_of[b]);
            let splines = [of_spline(a), of_spline(b)];
            let joint = joint_of[i]
                .as_ref()
                .expect("a joint after a piece with one next");
            match (path[subs[a].elem].1, path[subs[b].elem].1, joint) {
                (Of::Piece(_), Of::Piece(_), Joint::Shared { by, .. }) => match by {
                    _ if splines == [true, true] => {}
                    Held::Circle if splines[0] || splines[1] => {
                        let other = if splines[0] { ob } else { oa };
                        let corner = corner_of(self, i);
                        let circle = self.add_circle(corner, value);
                        self.set_construction(circle, true);
                        self.constrain(Constraint::Tangent {
                            a: other,
                            b: circle,
                        });
                        self.constrain(Constraint::Equal {
                            a: circle,
                            b: reach.expect("made for a spline's offset"),
                        });
                    }
                    _ if splines[0] || splines[1] => {}
                    Held::Tangent => {
                        self.constrain(Constraint::Tangent { a: oa, b: ob });
                    }
                    Held::Square => {
                        let corner = corner_of(self, i);
                        self.square_end(pieces_of[i].0, corner, end_point[a].unwrap());
                    }
                    Held::Circle => {
                        let corner = corner_of(self, i);
                        let circle = self.add_circle(corner, value);
                        self.set_construction(circle, true);
                        self.constrain(Constraint::Tangent { a: oa, b: circle });
                        self.constrain(Constraint::Tangent { a: ob, b: circle });
                    }
                },
                _ if splines[0] || splines[1] => {}
                _ => {
                    self.constrain(Constraint::Tangent { a: oa, b: ob });
                }
            }
        }
        loop {
            let mut grew = false;
            for &(a, b) in &joined {
                if held[a] != held[b] {
                    held[a] = true;
                    held[b] = true;
                    grew = true;
                }
            }
            if !grew {
                break;
            }
        }
        // A part cut off from them at crossings holds the distance by a
        // circle about its source's corner, touching it, as large as one
        // about the first's: one dimension still says how far all are.
        for a in 0..subs.len() {
            if held[a] {
                continue;
            }
            let (Some(first), Some(first_piece)) = (first, first_piece) else {
                unreachable!("a part of a piece is kept");
            };
            let reference = *reach.get_or_insert_with(|| {
                let circle = self.add_circle(corner_of(self, first_piece), value);
                self.set_construction(circle, true);
                self.constrain(Constraint::Tangent {
                    a: curve_of[first],
                    b: circle,
                });
                circle
            });
            match path[subs[a].elem].1 {
                Of::Piece(i) => {
                    let circle = self.add_circle(corner_of(self, i), value);
                    self.set_construction(circle, true);
                    self.constrain(Constraint::Tangent {
                        a: curve_of[a],
                        b: circle,
                    });
                    self.constrain(Constraint::Equal {
                        a: circle,
                        b: reference,
                    });
                }
                Of::Round(_) => {
                    self.constrain(Constraint::Equal {
                        a: curve_of[a],
                        b: reference,
                    });
                }
            }
        }
        // An open chain's offset ends square to the chain's ends; a
        // spline's, held at its end's offset, does already.
        {
            for a in 0..subs.len() {
                let k = subs[a].elem;
                let at_start =
                    if subs[a].from_start && path_previous[k].is_none() && previous[a].is_none() {
                        true
                    } else if subs[a].to_end && path_next[k].is_none() && next[a].is_none() {
                        false
                    } else {
                        continue;
                    };
                let Of::Piece(i) = path[k].1 else {
                    continue;
                };
                if spline(i) {
                    continue;
                }
                let (source, reversed) = pieces_of[i];
                let (s, e) = self.curves[&source].endpoints().expect("a line or arc");
                let end = if at_start != reversed { s } else { e };
                let image = if at_start {
                    start_point[a].unwrap()
                } else {
                    end_point[a].unwrap()
                };
                self.square_end(source, end, image);
            }
        }
        // Each circle's offset about its center, as far from it as the
        // distance: the first dimension, or a construction circle on it,
        // level with its center, as large as the one the rest is held at,
        // touching the offset.
        for (c, _, ring) in &circles {
            let Shape::Plain(Plain::Round { center, radius, .. }) = ring else {
                unreachable!("a circle's offset is a circle");
            };
            let CurveKind::Circle {
                center: middle,
                radius: source,
            } = self.curves[c].kind
            else {
                unreachable!("a circle");
            };
            let circle = self.add_circle(middle, Scalar::from_f64(*radius));
            made.push(circle);
            if distance.is_none() {
                distance = Some(self.constrain(Constraint::Offset {
                    a: *c,
                    b: circle,
                    value,
                }));
                continue;
            }
            let on = self.new_point_at(add(*center, [source.to_f64(), 0.0]));
            self.constrain(Constraint::PointOnCurve {
                point: on,
                curve: *c,
            });
            let level = self.add_line(middle, on);
            self.set_construction(level, true);
            self.constrain(Constraint::Horizontal { line: level });
            let touching = self.add_circle(on, value);
            self.set_construction(touching, true);
            self.constrain(Constraint::Tangent {
                a: circle,
                b: touching,
            });
            let reference = match reach {
                Some(reach) => reach,
                None => {
                    let first = first.expect("a part kept, its distance given");
                    let first_piece = first_piece.expect("a part kept");
                    let circle = self.add_circle(corner_of(self, first_piece), value);
                    self.set_construction(circle, true);
                    self.constrain(Constraint::Tangent {
                        a: curve_of[first],
                        b: circle,
                    });
                    reach = Some(circle);
                    circle
                }
            };
            self.constrain(Constraint::Equal {
                a: touching,
                b: reference,
            });
        }
        Ok(Offsetted {
            curves: made,
            distance: distance.expect("a part of a piece is kept"),
        })
    }

    /// The curves `curves` as chains (see [`Sketch::chain`]): each made of
    /// those joined to each other end to end.
    pub fn chains(&self, curves: &[CurveId]) -> GeopResult<Vec<Chain>> {
        let class = self.point_classes();
        let ends = |c: CurveId| -> GeopResult<Vec<PointId>> {
            Ok(match self.curve(c)?.endpoints() {
                Some((s, e)) => vec![class[&s], class[&e]],
                None => Vec::new(),
            })
        };
        let mut left: Vec<CurveId> = Vec::new();
        for &c in curves {
            if !left.contains(&c) {
                left.push(c);
            }
        }
        let mut chains = Vec::new();
        while let Some(first) = left.first().copied() {
            let mut group = vec![first];
            let mut points = ends(first)?;
            left.remove(0);
            loop {
                let mut joined = None;
                for (k, &c) in left.iter().enumerate() {
                    if ends(c)?.iter().any(|p| points.contains(p)) {
                        joined = Some(k);
                        break;
                    }
                }
                let Some(k) = joined else {
                    break;
                };
                let c = left.remove(k);
                points.extend(ends(c)?);
                group.push(c);
            }
            chains.push(self.chain(&group)?);
        }
        if chains.is_empty() {
            return Err(GeopError::new("no curves to make a chain of"));
        }
        Ok(chains)
    }

    /// Keeps `image`, an end of the offset of `source`, square to `source`
    /// at its end `end`: on the line through `end` across a line, or
    /// through an arc's center.
    fn square_end(&mut self, source: CurveId, end: PointId, image: PointId) {
        match self.curves[&source].kind {
            CurveKind::Line { .. } => {
                let across = self.add_line(end, image);
                self.set_construction(across, true);
                self.constrain(Constraint::Perpendicular {
                    a: source,
                    b: across,
                });
            }
            _ => {
                let Some(Plain::Round { center, .. }) = Plain::of(self, source, false) else {
                    return;
                };
                let m = self.new_point_at(center);
                self.constrain(Constraint::Center {
                    point: m,
                    curve: source,
                });
                let radial = self.add_line(m, end);
                self.set_construction(radial, true);
                self.constrain(Constraint::PointOnCurve {
                    point: image,
                    curve: radial,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn n(x: f64) -> Scalar {
        Scalar::from_f64(x)
    }

    /// A `w` x `h` rectangle at the origin, counter-clockwise, fully
    /// constrained; its lines, and the dimension of its width.
    fn rectangle(s: &mut Sketch, w: f64, h: f64) -> (Vec<CurveId>, ConstraintId) {
        let p = [[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Horizontal { line: l[0] });
        s.constrain(Constraint::Vertical { line: l[1] });
        s.constrain(Constraint::Horizontal { line: l[2] });
        s.constrain(Constraint::Vertical { line: l[3] });
        let width = s.constrain(Constraint::Length {
            curve: l[0],
            value: n(w),
        });
        s.constrain(Constraint::Length {
            curve: l[1],
            value: n(h),
        });
        (l, width)
    }

    /// The bounding box of the points of `curves`, as solved.
    fn bounds(s: &Sketch, curves: &[CurveId]) -> [f64; 4] {
        let points = curves.iter().flat_map(|c| s.curves[c].points());
        let (lo, hi) = plain::bounds(points.map(|p| xy(s, p))).unwrap();
        [lo[0], lo[1], hi[0], hi[1]]
    }

    fn near(a: [f64; 4], b: [f64; 4]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-7)
    }

    fn set(s: &mut Sketch, k: ConstraintId, v: f64) {
        match s.constraints.get_mut(&k) {
            Some(Constraint::Offset { value, .. } | Constraint::Length { value, .. }) => {
                *value = n(v)
            }
            other => panic!("{other:?}"),
        }
    }

    fn lines(s: &Sketch, curves: &[CurveId]) -> Vec<CurveId> {
        curves
            .iter()
            .copied()
            .filter(|c| matches!(s.curves[c].kind, CurveKind::Line { .. }))
            .collect()
    }

    /// Outwards, a rectangle's offset rounds its corners: four lines and
    /// four arcs about the corners, one distance for all of them, which
    /// the rectangle and the distance both drive.
    #[test]
    fn a_rectangle_offset_outwards_rounds_its_corners() {
        let mut s = Sketch::new();
        let (rect, width) = rectangle(&mut s, 2.0, 1.0);
        // The rectangle runs counter-clockwise: outwards is to its right.
        let made = s.offset(&rect, n(-0.25), Corners::Round).unwrap();
        assert_eq!(made.curves.len(), 8);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        // The rectangle and its offset nest: one ring between them.
        assert_eq!(s.regions().unwrap().len(), 1);
        let sides = lines(&s, &made.curves);
        assert!(
            near(bounds(&s, &sides), [-0.25, -0.25, 2.25, 1.25]),
            "{:?}",
            bounds(&s, &sides)
        );
        set(&mut s, made.distance, 0.5);
        set(&mut s, width, 3.0);
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        assert!(
            near(bounds(&s, &sides), [-0.5, -0.5, 3.5, 1.5]),
            "{:?}",
            bounds(&s, &sides)
        );
        s.enclose().unwrap();
    }

    /// Inwards, the offsets meet at the corners, held at one distance by
    /// construction circles about them; extended outwards, the same.
    #[test]
    fn offsets_meet_inwards_and_extended() {
        for (d, corners, inner, then) in [
            (
                0.25,
                Corners::Round,
                [0.25, 0.25, 1.75, 0.75],
                [0.125, 0.125, 1.875, 0.875],
            ),
            (
                -0.25,
                Corners::Extend,
                [-0.25, -0.25, 2.25, 1.25],
                [-0.125, -0.125, 2.125, 1.125],
            ),
        ] {
            let mut s = Sketch::new();
            let (rect, _) = rectangle(&mut s, 2.0, 1.0);
            let made = s.offset(&rect, n(d), corners).unwrap();
            assert_eq!(made.curves.len(), 4);
            let report = s.solve().unwrap();
            assert!(report.converged && report.dof == 0, "{d}: {report:?}");
            assert!(
                near(bounds(&s, &made.curves), inner),
                "{:?}",
                bounds(&s, &made.curves)
            );
            set(&mut s, made.distance, 0.125);
            let report = s.solve().unwrap();
            assert!(report.converged, "{report:?}");
            assert!(
                near(bounds(&s, &made.curves), then),
                "{:?}",
                bounds(&s, &made.curves)
            );
            // The rectangle and its offset nest: one ring between them.
            assert_eq!(s.regions().unwrap().len(), 1);
            s.enclose().unwrap();
        }
    }

    /// A slot runs on smoothly all round: its offset is a slot about it.
    #[test]
    fn a_slot_offsets_into_a_slot() {
        let mut s = Sketch::new();
        let p =
            [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let bottom = s.add_line(p[0], p[1]);
        let right = s.add_arc(p[1], p[2], n(PI));
        let top = s.add_line(p[2], p[3]);
        let left = s.add_arc(p[3], p[0], n(PI));
        for (a, b) in [(bottom, right), (right, top), (top, left), (left, bottom)] {
            s.constrain(Constraint::Tangent { a, b });
        }
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        s.constrain(Constraint::Horizontal { line: bottom });
        s.constrain(Constraint::Length {
            curve: bottom,
            value: n(2.0),
        });
        s.constrain(Constraint::Radius {
            curve: right,
            value: n(0.5),
        });
        s.constrain(Constraint::Equal { a: left, b: right });
        assert!(s.solve().unwrap().converged);
        let made = s
            .offset(&[top, left, bottom, right], n(-0.25), Corners::Round)
            .unwrap();
        assert_eq!(made.curves.len(), 4);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert!(
            near(bounds(&s, &made.curves), [0.0, -0.25, 2.0, 1.25]),
            "{:?}",
            bounds(&s, &made.curves)
        );
        s.enclose().unwrap();
    }

    /// An open chain's offset ends square to it.
    #[test]
    fn open_chains_end_square() {
        let mut s = Sketch::new();
        let p = [[0.0, 0.0], [2.0, 0.0], [3.0, 1.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let a = s.add_line(p[0], p[1]);
        let b = s.add_arc(p[1], p[2], n(PI / 2.0));
        s.constrain(Constraint::Tangent { a, b });
        for (q, at) in [(p[0], [0.0, 0.0]), (p[1], [2.0, 0.0]), (p[2], [3.0, 1.0])] {
            s.constrain(Constraint::Fix {
                point: q,
                x: n(at[0]),
                y: n(at[1]),
            });
        }
        // Inside the bend: to the left.
        let made = s.offset(&[b, a], n(0.5), Corners::Round).unwrap();
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert!(
            near(bounds(&s, &made.curves), [0.0, 0.5, 2.5, 1.0]),
            "{:?}",
            bounds(&s, &made.curves)
        );
        s.enclose().unwrap();
    }

    /// A circle offsets into one about the same center.
    #[test]
    fn circles_offset_about_their_center() {
        let mut s = Sketch::new();
        let c = s.add_point(n(1.0), n(1.0));
        let circle = s.add_circle(c, n(1.0));
        let made = s.offset(&[circle], n(-0.5), Corners::Round).unwrap();
        assert!(matches!(
            s.curves[&made.curves[0]].kind,
            CurveKind::Circle { center, radius } if center == c && radius.to_f64() == 1.5
        ));
        assert!(s.solve().unwrap().converged);
    }

    /// How far the points of the spline `fit` are from the spline
    /// `source`, at most and at least: each sampled point's distance to
    /// the nearest of a fine sampling of the source, refined.
    fn spread(s: &Sketch, source: CurveId, fit: CurveId) -> (f64, f64) {
        let shape = |c: CurveId| oriented(s, c, false, 0.0).unwrap();
        let (source, fit) = (shape(source), shape(fit));
        let mut range = (f64::INFINITY, 0.0f64);
        for k in 0..=200 {
            let p = fit.at(k as f64 / 200.0);
            let q = source.at(source.nearest(p));
            let d = plain::distance(p, q);
            range = (range.0.min(d), range.1.max(d));
        }
        range
    }

    /// An open spline offset: a fitted spline held to it at the distance
    /// of a construction circle's radius, the dimension, which it follows,
    /// as it follows the spline; it adds no freedom.
    #[test]
    fn a_spline_offsets_into_a_fitted_spline() {
        let mut s = Sketch::new();
        let q = [[0.0, 0.0], [1.0, 0.6], [2.0, -0.3], [3.0, 0.4], [4.0, 0.0]]
            .map(|q| s.add_point(n(q[0]), n(q[1])));
        let spline = s.add_spline(q.to_vec());
        let free = s.solve().unwrap().dof;
        let made = s.offset(&[spline], n(0.2), Corners::Round).unwrap();
        let [fit] = made.curves[..] else {
            panic!("{made:?}");
        };
        assert!(matches!(s.curves[&fit].kind, CurveKind::Spline { .. }));
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == free, "{report:?}");
        let (lo, hi) = spread(&s, spline, fit);
        assert!(lo > 0.2 - 1e-3 && hi < 0.2 + 1e-3, "{lo} {hi}");
        // The distance changed, and the spline's end dragged.
        match s.constraints.get_mut(&made.distance) {
            Some(Constraint::Radius { value, .. }) => *value = n(0.3),
            other => panic!("{other:?}"),
        }
        s.constrain(Constraint::Fix {
            point: q[4],
            x: n(4.0),
            y: n(0.4),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let (lo, hi) = spread(&s, spline, fit);
        assert!(lo > 0.3 - 1e-3 && hi < 0.3 + 1e-3, "{lo} {hi}");
        s.enclose().unwrap();
    }

    /// A closed outline of a line and a spline arching over it, offset in
    /// and out: in, the offsets cross at the corners and are cut back to
    /// where they do, whichever way corners are asked to close; out,
    /// rounded. Each time one ring. Out, extended, the line's offset does
    /// not reach the spline's, which ends at the spline's end: refused.
    #[test]
    fn a_line_and_a_spline_offset_in_and_out() {
        for (d, corners) in [
            (0.1, Corners::Round),
            (0.1, Corners::Extend),
            (-0.1, Corners::Round),
        ] {
            let mut s = Sketch::new();
            let p = [[0.0, 0.0], [2.0, 0.0], [2.5, 1.0], [1.0, 2.0], [-0.5, 1.0]]
                .map(|q| s.add_point(n(q[0]), n(q[1])));
            let base = s.add_line(p[0], p[1]);
            let arch = s.add_spline(vec![p[1], p[2], p[3], p[4], p[0]]);
            let free = s.solve().unwrap().dof;
            let made = s
                .offset(&[base, arch], n(d), corners)
                .unwrap_or_else(|e| panic!("{d} {corners:?}: {e:?}"));
            let report = s.solve().unwrap();
            assert!(
                report.converged && report.dof == free,
                "{d} {corners:?}: {report:?}"
            );
            assert_eq!(s.regions().unwrap().len(), 1, "{d} {corners:?}");
            let fit = *made
                .curves
                .iter()
                .find(|c| matches!(s.curves[c].kind, CurveKind::Spline { .. }))
                .unwrap();
            let (lo, hi) = spread(&s, arch, fit);
            assert!(
                lo > 0.1 - 1e-4 && hi < 0.1 + 1e-4,
                "{d} {corners:?}: {lo} {hi}"
            );
        }
        let mut s = Sketch::new();
        let p = [[0.0, 0.0], [2.0, 0.0], [2.5, 1.0], [1.0, 2.0], [-0.5, 1.0]]
            .map(|q| s.add_point(n(q[0]), n(q[1])));
        let base = s.add_line(p[0], p[1]);
        let arch = s.add_spline(vec![p[1], p[2], p[3], p[4], p[0]]);
        let e = s
            .offset(&[base, arch], n(-0.1), Corners::Extend)
            .unwrap_err();
        assert!(e.root_message().contains("do not meet"), "{e}");
    }

    /// What would cross itself is refused, naming what does.
    #[test]
    fn offsets_that_would_cross_themselves_are_refused() {
        let mut s = Sketch::new();
        let (rect, _) = rectangle(&mut s, 2.0, 1.0);
        let e = s.offset(&rect, n(0.6), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("too short"), "{e}");
        let c = s.add_point(n(5.0), n(0.0));
        let circle = s.add_circle(c, n(0.5));
        let e = s.offset(&[circle], n(0.5), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("smaller"), "{e}");
        let q = [[0.0, 3.0], [1.0, 4.0], [2.0, 3.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        // A parabola bending right by a radius of 1 at its top, offset 1.5
        // into the bend.
        let spline = s.add_spline(q.to_vec());
        let e = s.offset(&[spline], n(-1.5), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("bends tighter"), "{e}");
    }

    /// A closed square of side `side` at `corner`, counter-clockwise, its
    /// corners fixed: its sides.
    fn fixed_square(s: &mut Sketch, corner: [f64; 2], side: f64) -> Vec<CurveId> {
        let q = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
            .map(|q| [corner[0] + side * q[0], corner[1] + side * q[1]]);
        let p = q.map(|q| s.add_point(n(q[0]), n(q[1])));
        for (&point, q) in p.iter().zip(q) {
            s.constrain(Constraint::Fix {
                point,
                x: n(q[0]),
                y: n(q[1]),
            });
        }
        (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect()
    }

    /// An outline and a hole in it offset at once, both into the material
    /// between them though both run counter-clockwise: the outline in, the
    /// hole out — a square and a circle hole too. One dimension for all.
    #[test]
    fn an_outline_and_its_holes_offset_into_the_material() {
        let mut s = Sketch::new();
        let outline = fixed_square(&mut s, [0.0, 0.0], 4.0);
        let hole = fixed_square(&mut s, [1.0, 1.0], 1.0);
        let middle = s.add_point(n(3.0), n(3.0));
        s.constrain(Constraint::Fix {
            point: middle,
            x: n(3.0),
            y: n(3.0),
        });
        let round = s.add_circle(middle, n(0.4));
        s.constrain(Constraint::Radius {
            curve: round,
            value: n(0.4),
        });
        let all: Vec<CurveId> = outline
            .iter()
            .chain(&hole)
            .copied()
            .chain([round])
            .collect();
        let made = s.offset(&all, n(0.2), Corners::Round).unwrap();
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        let sides: Vec<CurveId> = lines(&s, &made.curves);
        assert_eq!(sides.len(), 8, "{made:?}");
        let (inner, outer): (Vec<CurveId>, Vec<CurveId>) = sides
            .iter()
            .partition(|&&c| bounds(&s, &[c])[0] > 0.5 && bounds(&s, &[c])[2] < 3.5);
        assert!(
            near(bounds(&s, &outer), [0.2, 0.2, 3.8, 3.8]),
            "{:?}",
            bounds(&s, &outer)
        );
        assert!(
            near(bounds(&s, &inner), [0.8, 0.8, 2.2, 2.2]),
            "{:?}",
            bounds(&s, &inner)
        );
        let circle = made
            .curves
            .iter()
            .find(|c| {
                matches!(s.curves[c].kind, CurveKind::Circle { .. }) && !s.curves[c].construction
            })
            .unwrap();
        let CurveKind::Circle { radius, .. } = s.curves[circle].kind else {
            unreachable!()
        };
        assert!((radius.to_f64() - 0.6).abs() < 1e-7, "{radius:?}");
        set(&mut s, made.distance, 0.1);
        assert!(s.solve().unwrap().converged);
        assert!(
            near(bounds(&s, &outer), [0.1, 0.1, 3.9, 3.9]),
            "{:?}",
            bounds(&s, &outer)
        );
        s.enclose().unwrap();
    }

    /// A hole closer to the outline than twice the distance: the two
    /// offsets run into each other and are cut where they cross, the
    /// outline's offset and the hole's joined into one curve round both.
    #[test]
    fn a_hole_near_the_outline_offsets_into_one_curve() {
        let mut s = Sketch::new();
        let outline = fixed_square(&mut s, [0.0, 0.0], 4.0);
        let hole = fixed_square(&mut s, [2.8, 1.5], 1.0);
        let all: Vec<CurveId> = outline.iter().chain(&hole).copied().collect();
        let made = s.offset(&all, n(0.3), Corners::Round).unwrap();
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        // What is left between the offsets: one area, round the hole and
        // the offsets' meeting.
        let loops = s.chain(
            &made
                .curves
                .iter()
                .copied()
                .filter(|c| !s.curves[c].construction)
                .collect::<Vec<_>>(),
        );
        assert!(loops.is_ok_and(|chain| chain.closed), "{made:?}");
        s.enclose().unwrap();
    }

    /// The chain through a curve runs on as long as it does not branch.
    #[test]
    fn chains_through_a_curve() {
        let mut s = Sketch::new();
        let (rect, _) = rectangle(&mut s, 2.0, 1.0);
        let mut chain = s.chain_through(rect[2]).unwrap();
        chain.sort();
        assert_eq!(chain, rect);
        let chain = s.chain(&rect).unwrap();
        assert!(chain.closed);
        // Inside a counter-clockwise loop is to its left.
        assert!((s.chain_side(&chain, [1.0, 0.25]).unwrap() - 0.25).abs() < 1e-12);
        assert!((s.chain_side(&chain, [1.0, -0.5]).unwrap() + 0.5).abs() < 1e-12);
    }

    /// Two lines running on in one line — the halves of a mirrored outline
    /// meeting on the mirror line — have no corner to round: their offsets
    /// meet where they part.
    #[test]
    fn lines_in_line_offset_straight_on() {
        let mut s = Sketch::new();
        let p = [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let a = s.add_line(p[0], p[1]);
        let b = s.add_line(p[1], p[2]);
        for (q, at) in [(p[0], 0.0), (p[1], 1.0), (p[2], 2.0)] {
            s.constrain(Constraint::Fix {
                point: q,
                x: n(at),
                y: n(0.0),
            });
        }
        for corners in [Corners::Round, Corners::Extend] {
            for d in [0.25, -0.25] {
                let mut s = s.clone();
                let made = s.offset(&[a, b], n(d), corners).unwrap();
                assert_eq!(made.curves.len(), 2, "{corners:?} {d}");
                let report = s.solve().unwrap();
                assert!(report.converged && report.dof == 0, "{report:?}");
                assert!(
                    near(bounds(&s, &made.curves), [0.0, d, 2.0, d]),
                    "{:?}",
                    bounds(&s, &made.curves)
                );
                s.enclose().unwrap();
            }
        }
    }

    /// A rounded rectangle — a `w` x `h` one, its corners filleted by `r` —
    /// fully constrained, counter-clockwise: its curves.
    fn rounded(s: &mut Sketch, w: f64, h: f64, r: f64) -> Vec<CurveId> {
        let q = |x: f64, y: f64| [x, y];
        let corners = [
            (q(r, 0.0), q(w - r, 0.0)),
            (q(w, r), q(w, h - r)),
            (q(w - r, h), q(r, h)),
            (q(0.0, h - r), q(0.0, r)),
        ];
        let mut points = Vec::new();
        for (a, b) in corners {
            points.push(s.add_point(n(a[0]), n(a[1])));
            points.push(s.add_point(n(b[0]), n(b[1])));
        }
        let mut curves = Vec::new();
        for k in 0..4 {
            let line = s.add_line(points[2 * k], points[2 * k + 1]);
            let arc = s.add_arc(points[2 * k + 1], points[(2 * k + 2) % 8], n(PI / 2.0));
            curves.extend([line, arc]);
        }
        for k in 0..8 {
            s.constrain(Constraint::Tangent {
                a: curves[k],
                b: curves[(k + 1) % 8],
            });
        }
        for (k, at) in points.iter().zip([
            q(r, 0.0),
            q(w - r, 0.0),
            q(w, r),
            q(w, h - r),
            q(w - r, h),
            q(r, h),
            q(0.0, h - r),
            q(0.0, r),
        ]) {
            s.constrain(Constraint::Fix {
                point: *k,
                x: n(at[0]),
                y: n(at[1]),
            });
        }
        curves
    }

    /// Offsets of closed chains, in and out, rounded and extended, of
    /// several sizes: each solved, fully constrained, and its solution
    /// proven.
    #[test]
    fn closed_offsets_are_proven() {
        let mut failures = Vec::new();
        for (w, h) in [(2.0, 1.0), (1.0, 1.0), (3.0, 0.5), (1.3, 0.7)] {
            for d in [0.1, 0.2, -0.1, -0.3] {
                for corners in [Corners::Round, Corners::Extend] {
                    for shape in ["rectangle", "rounded"] {
                        let mut s = Sketch::new();
                        let curves = match shape {
                            "rectangle" => rectangle(&mut s, w, h).0,
                            _ => rounded(&mut s, w, h, 0.2),
                        };
                        assert!(s.solve().unwrap().converged);
                        let case = format!("{shape} {w}x{h} by {d}, {corners:?}");
                        let Ok(_) = s.offset(&curves, n(d), corners) else {
                            continue;
                        };
                        let report = s.solve().unwrap();
                        if !report.converged || report.dof != 0 {
                            failures.push(format!("{case}: {report:?}"));
                        } else if let Err(e) = s.enclose() {
                            failures.push(format!("{case}: {}", e.root_message()));
                        }
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    /// A closed offset says one thing twice: going round the loop, every
    /// offset is at the distance from its source, and each joint holds it
    /// there again, so the last joint's tie says what the others already
    /// do. Round a crooked pentagon — no side level, no corner square, so
    /// no rounding is exactly zero — in and out, rounded and extended, the
    /// redundancy holds where the rest does and the solution is proven.
    #[test]
    fn closed_offsets_of_a_crooked_outline_are_proven() {
        let corners = [[0.0, 0.0], [2.1, 0.3], [2.6, 1.7], [1.2, 2.4], [-0.3, 1.3]];
        let mut failures = Vec::new();
        for d in [0.15, -0.15, 0.4] {
            for corners_as in [Corners::Round, Corners::Extend] {
                let mut s = Sketch::new();
                let p = corners.map(|q| s.add_point(n(q[0]), n(q[1])));
                let sides: Vec<CurveId> =
                    (0..5).map(|i| s.add_line(p[i], p[(i + 1) % 5])).collect();
                for (&point, q) in p.iter().zip(corners) {
                    s.constrain(Constraint::Fix {
                        point,
                        x: n(q[0]),
                        y: n(q[1]),
                    });
                }
                let case = format!("by {d}, {corners_as:?}");
                if let Err(e) = s.offset(&sides, n(d), corners_as) {
                    failures.push(format!("{case}: {}", e.root_message()));
                    continue;
                }
                let report = s.solve().unwrap();
                if !report.converged || report.dof != 0 {
                    failures.push(format!("{case}: {report:?}"));
                } else if let Err(e) = s.enclose() {
                    failures.push(format!("{case}: {}", e.root_message()));
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }

    /// A 4 x 4 square with a notch 1 wide and 2 deep cut down from the
    /// middle of its top, offset outward by 0.6: into the notch, its walls'
    /// offsets cross and its floor's turns back. What is left is trimmed
    /// where the arcs round the notch's top corners cross, over the notch's
    /// middle; the rest stays tied to the outline.
    #[test]
    fn an_offset_over_a_notch_narrower_than_twice_it_is_trimmed() {
        let corners = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 4.0],
            [2.5, 4.0],
            [2.5, 2.0],
            [1.5, 2.0],
            [1.5, 4.0],
            [0.0, 4.0],
        ];
        let mut s = Sketch::new();
        let p = corners.map(|q| s.add_point(n(q[0]), n(q[1])));
        let sides: Vec<CurveId> = (0..8).map(|i| s.add_line(p[i], p[(i + 1) % 8])).collect();
        for (&point, q) in p.iter().zip(corners) {
            s.constrain(Constraint::Fix {
                point,
                x: n(q[0]),
                y: n(q[1]),
            });
        }
        let made = s.offset(&sides, n(-0.6), Corners::Round).unwrap();
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert!(
            near(bounds(&s, &made.curves), [-0.6, -0.6, 4.6, 4.6]),
            "{:?}",
            bounds(&s, &made.curves)
        );
        // The notch's walls and floor leave nothing; the arcs round its top
        // corners meet over its middle.
        let dip = |s: &Sketch, r: f64| {
            let q = [2.0, 4.0 + (r * r - 0.25f64).sqrt()];
            made.curves
                .iter()
                .flat_map(|c| s.curves[c].points())
                .any(|p| plain::distance(xy(s, p), q) < 1e-7)
        };
        assert!(dip(&s, 0.6));
        assert_eq!(lines(&s, &made.curves).len(), 5);
        assert_eq!(made.curves.len(), 5 + 4 + 2);
        s.enclose().unwrap();
    }

    /// The notched square's trimmed offset following its dimension, from
    /// 0.6 to 0.7: the arcs round the notch's corners still cross over its
    /// middle, higher up. The solve once stopped flat at its first step:
    /// its step was solved through normal equations, which squared how
    /// ill-conditioned the two arcs held to one crossing are, and met none
    /// of the rows it was solved for.
    #[test]
    fn a_trimmed_offset_follows_its_dimension() {
        let mut s = Sketch::new();
        let corners = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 4.0],
            [2.5, 4.0],
            [2.5, 2.0],
            [1.5, 2.0],
            [1.5, 4.0],
            [0.0, 4.0],
        ];
        let p = corners.map(|q| s.add_point(n(q[0]), n(q[1])));
        let sides: Vec<CurveId> = (0..8).map(|i| s.add_line(p[i], p[(i + 1) % 8])).collect();
        for (&point, q) in p.iter().zip(corners) {
            s.constrain(Constraint::Fix {
                point,
                x: n(q[0]),
                y: n(q[1]),
            });
        }
        let made = s.offset(&sides, n(-0.6), Corners::Round).unwrap();
        s.solve().unwrap();
        set(&mut s, made.distance, 0.7);
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        assert!(
            near(bounds(&s, &made.curves), [-0.7, -0.7, 4.7, 4.7]),
            "{:?}",
            bounds(&s, &made.curves)
        );
    }
}
