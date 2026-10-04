//! Offsetting a chain of lines and arcs — or a circle — by a distance, the
//! offset tied to the chain by constraints so it follows it.
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
//!   cut back to where they meet.
//!
//! An arc round a corner has the corner as its center, so its radius is the
//! distance of both offsets it touches. Where offsets meet, a construction
//! circle about the corner, touching both, says the same. An open chain's
//! offset ends square to the chain's ends.
//!
//! Splines are refused — their offset is no spline — and so is an offset
//! that would cross itself: an arc offset past its center, a curve too
//! short for the corners at its ends to fit, pieces of the offset running
//! into each other.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use serde::{Deserialize, Serialize};

use crate::{
    plain::{P2, Plain, add, angle_between, cross, dist, dot, perp, scale, sub, unit, wrap, xy},
    profile::Chain,
    sketch::{Constraint, ConstraintId, CurveId, CurveKind, PointId, Sketch},
};

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

/// A piece of a chain, along the chain's direction, in plain numbers.
fn oriented<S: Scalar>(sketch: &Sketch<S>, curve: CurveId, reversed: bool) -> GeopResult<Plain> {
    let plain = match sketch.curve(curve)?.kind {
        CurveKind::Line { start, end } => {
            let (a, b) = (xy(sketch, start), xy(sketch, end));
            let (a, b) = if reversed { (b, a) } else { (a, b) };
            Plain::Line {
                a,
                b,
                infinite: false,
            }
        }
        CurveKind::Arc { start, end, sweep } => {
            let (s, e, sweep) = (xy(sketch, start), xy(sketch, end), sweep.to_f64());
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
            center: xy(sketch, center),
            radius: radius.to_f64(),
            span: None,
        },
        CurveKind::Spline { .. } => {
            return Err(GeopError::new(format!(
                "spline {curve} cannot be offset: the offset of a spline is no spline"
            )));
        }
    };
    Ok(plain)
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

impl<S: Scalar> Sketch<S> {
    /// How far `p` is from the chain `chain`: positive to its left.
    pub fn chain_side(&self, chain: &Chain, p: P2) -> GeopResult<f64> {
        let mut best: Option<(f64, f64)> = None;
        for edge in &chain.edges {
            let piece = oriented(self, edge.curve, edge.reversed)?;
            let t = piece.nearest(p);
            let q = piece.at(t);
            let d = dist(p, q);
            if best.is_none_or(|(b, _)| d < b) {
                let side = cross(piece.tangent(t), sub(p, q)).signum();
                best = Some((d, side * d));
            }
        }
        Ok(best.map_or(0.0, |(_, signed)| signed))
    }

    /// Offsets the chain `curves` (see [`Sketch::chain`]) by `distance` —
    /// to its left, to its right if negative — its corners as `corners`
    /// says (see the module docs). Returns the curves made, and the
    /// dimension of the distance.
    pub fn offset(&mut self, curves: &[CurveId], distance: S, corners: Corners) -> GeopResult<Offsetted> {
        let chain = self.chain(curves)?;
        let pieces_of: Vec<(CurveId, bool)> =
            chain.edges.iter().map(|e| (e.curve, e.reversed)).collect();
        let d = distance.to_f64();
        if d == 0.0 || !d.is_finite() {
            return Err(GeopError::new(format!("cannot offset by {d}")));
        }
        let value = distance.abs();
        let too_far = |c: CurveId| {
            GeopError::new(format!(
                "{c} cannot be offset by {d} towards its center: it is smaller than that"
            ))
        };

        // A circle: one about the same center.
        if let [(c, _)] = pieces_of[..]
            && let CurveKind::Circle { center, radius } = self.curves[&c].kind
        {
            let r = radius.to_f64() - d;
            if r <= 0.0 {
                return Err(too_far(c));
            }
            let circle = self.add_circle(center, S::from_f64(r));
            let distance = self.constrain(Constraint::Offset { a: c, b: circle, value });
            return Ok(Offsetted {
                curves: vec![circle],
                distance,
            });
        }

        let n = pieces_of.len();
        let pieces: Vec<Plain> = pieces_of
            .iter()
            .map(|&(c, r)| oriented(self, c, r))
            .collect::<GeopResult<_>>()?;
        let offsets: Vec<Plain> = pieces
            .iter()
            .zip(&pieces_of)
            .map(|(&p, &(c, _))| offset_plain(p, d).ok_or_else(|| too_far(c)))
            .collect::<GeopResult<_>>()?;
        let tangent = |a: CurveId, b: CurveId| {
            self.constraints.values().any(|k| {
                matches!(*k, Constraint::Tangent { a: x, b: y } if (x, y) == (a, b) || (x, y) == (b, a))
            })
        };

        // Each joint: where the offsets of the pieces before and after it
        // end and start, and what joins them.
        enum Joint {
            /// They meet at one point — and the corner keeps them at one
            /// distance by a construction circle, unless they run on
            /// smoothly.
            Shared { at: P2, circle: bool },
            /// An arc round the corner, turning by `sweep`.
            Round { from: P2, to: P2, sweep: f64 },
        }
        let joints = if chain.closed { n } else { n - 1 };
        let mut joint_of = Vec::new();
        for i in 0..joints {
            let j = (i + 1) % n;
            let (ci, cj) = (pieces_of[i].0, pieces_of[j].0);
            let p = pieces[i].at(1.0);
            let (ti, tj) = (pieces[i].tangent(1.0), pieces[j].tangent(0.0));
            let normal = perp(ti);
            let smooth = tangent(ci, cj);
            if smooth && dot(ti, tj) < 0.0 {
                return Err(GeopError::new(format!(
                    "{ci} and {cj} turn back on each other where they meet: their offsets would cross"
                )));
            }
            let turn = cross(ti, tj);
            joint_of.push(if smooth {
                Joint::Shared {
                    at: add(p, scale(normal, d)),
                    circle: false,
                }
            } else if turn * d < 0.0 && corners == Corners::Round {
                Joint::Round {
                    from: add(p, scale(normal, d)),
                    to: add(p, scale(perp(tj), d)),
                    sweep: angle_between(ti, tj),
                }
            } else {
                // Where the offsets, extended, meet — the crossing nearest
                // the corner's own offset.
                let near = add(p, scale(add(normal, perp(tj)), d / 2.0));
                let crossings = offsets[i].full().crossings(&offsets[j].full(), false);
                let at = match crossings
                    .into_iter()
                    .min_by(|a, b| dist(*a, near).total_cmp(&dist(*b, near)))
                {
                    Some(at) => at,
                    // Running on straight: they meet where they part.
                    None if turn == 0.0 && dot(ti, tj) > 0.0 => add(p, scale(normal, d)),
                    None => {
                        return Err(GeopError::new(format!(
                            "the offsets of {ci} and {cj} by {d} do not meet: round their corner instead"
                        )));
                    }
                };
                Joint::Shared { at, circle: true }
            });
        }

        // Where each offset piece starts and ends, and that it still runs
        // the way its piece does between them.
        let start_of = |i: usize| -> P2 {
            if i == 0 && !chain.closed {
                return offsets[0].at(0.0);
            }
            match &joint_of[(i + n - 1) % n] {
                Joint::Shared { at, .. } => *at,
                Joint::Round { to, .. } => *to,
            }
        };
        let end_of = |i: usize| -> P2 {
            if i == n - 1 && !chain.closed {
                return offsets[n - 1].at(1.0);
            }
            match &joint_of[i] {
                Joint::Shared { at, .. } => *at,
                Joint::Round { from, .. } => *from,
            }
        };
        let mut trimmed: Vec<Plain> = Vec::new();
        for i in 0..n {
            let (s, e) = (start_of(i), end_of(i));
            let (u, v) = (
                unwrapped(&offsets[i], s, true),
                unwrapped(&offsets[i], e, false),
            );
            let piece = match offsets[i] {
                Plain::Line { .. } if v > u => Some(Plain::Line {
                    a: s,
                    b: e,
                    infinite: false,
                }),
                Plain::Round {
                    span: Some((_, sweep)),
                    ..
                } if v > u => Plain::arc(s, e, sweep * (v - u)),
                _ => None,
            };
            trimmed.push(piece.ok_or_else(|| {
                GeopError::new(format!(
                    "{} is too short for an offset of {d}: its offset would turn back on itself",
                    pieces_of[i].0
                ))
            })?);
        }

        // The offset in order, joining arcs among its pieces: nothing in it
        // may cross anything but its neighbours, where they meet.
        let mut path: Vec<(Plain, String)> = Vec::new();
        for i in 0..n {
            path.push((trimmed[i], format!("the offset of {}", pieces_of[i].0)));
            if let Some(Joint::Round { from, to, sweep }) = joint_of.get(i)
                && let Some(arc) = Plain::arc(*from, *to, *sweep)
            {
                path.push((arc, format!("the arc round the corner after {}", pieces_of[i].0)));
            }
        }
        let m = path.len();
        for a in 0..m {
            for b in a + 2..m {
                if chain.closed && a == 0 && b == m - 1 {
                    continue;
                }
                if !path[a].0.crossings(&path[b].0, false).is_empty() {
                    return Err(GeopError::new(format!(
                        "the offset by {d} would cross itself: {} runs into {}",
                        path[a].1, path[b].1
                    )));
                }
            }
        }

        // Built: the points first — one where pieces share it — then the
        // pieces, then what joins them.
        let mut starts = Vec::new();
        let mut ends = Vec::new();
        let mut shared: Vec<Option<PointId>> = Vec::new();
        for joint in &joint_of {
            shared.push(match joint {
                Joint::Shared { at, .. } => Some(self.add_point(S::from_f64(at[0]), S::from_f64(at[1]))),
                Joint::Round { .. } => None,
            });
        }
        for i in 0..n {
            let s = match (i, chain.closed) {
                (0, false) => None,
                _ => shared[(i + n - 1) % n],
            };
            let e = match (i == n - 1, chain.closed) {
                (true, false) => None,
                _ => shared[i],
            };
            let at = |q: P2| (S::from_f64(q[0]), S::from_f64(q[1]));
            let (sx, sy) = at(trimmed[i].at(0.0));
            let (ex, ey) = at(trimmed[i].at(1.0));
            starts.push(s.unwrap_or_else(|| self.add_point(sx, sy)));
            ends.push(e.unwrap_or_else(|| self.add_point(ex, ey)));
        }
        let mut made = Vec::new();
        let mut offset_curves = Vec::new();
        let mut distance = None;
        for i in 0..n {
            let (source, reversed) = pieces_of[i];
            let (s, e) = (starts[i], ends[i]);
            // Each offset runs the way its source does.
            let (s, e) = if reversed { (e, s) } else { (s, e) };
            let curve = match (self.curves[&source].kind.clone(), trimmed[i]) {
                (CurveKind::Line { .. }, _) => self.add_line(s, e),
                (CurveKind::Arc { .. }, Plain::Round { span: Some((_, sweep)), .. }) => {
                    self.add_arc(s, e, S::from_f64(if reversed { -sweep } else { sweep }))
                }
                (kind, _) => unreachable!("chained and trimmed as a line or arc: {kind:?}"),
            };
            made.push(curve);
            offset_curves.push(curve);
            if i == 0 {
                distance = Some(self.constrain(Constraint::Offset {
                    a: source,
                    b: curve,
                    value,
                }));
            } else if let CurveKind::Line { .. } = self.curves[&source].kind {
                self.constrain(Constraint::Parallel { a: source, b: curve });
            } else {
                self.constrain(Constraint::Concentric { a: source, b: curve });
            }
        }
        for (i, joint) in joint_of.iter().enumerate() {
            let j = (i + 1) % n;
            let (oi, oj) = (offset_curves[i], offset_curves[j]);
            let corner = {
                let (source, reversed) = pieces_of[i];
                let (s, e) = self.curves[&source].endpoints().expect("a line or arc");
                if reversed { s } else { e }
            };
            match *joint {
                Joint::Shared { circle: false, .. } => {
                    self.constrain(Constraint::Tangent { a: oi, b: oj });
                }
                Joint::Shared { circle: true, .. } => {
                    let circle = self.add_circle(corner, value);
                    self.set_construction(circle, true);
                    self.constrain(Constraint::Tangent { a: oi, b: circle });
                    self.constrain(Constraint::Tangent { a: oj, b: circle });
                }
                Joint::Round { sweep, .. } => {
                    let arc = self.add_arc(ends[i], starts[j], S::from_f64(sweep));
                    made.push(arc);
                    self.constrain(Constraint::Center {
                        point: corner,
                        curve: arc,
                    });
                    self.constrain(Constraint::Tangent { a: oi, b: arc });
                    self.constrain(Constraint::Tangent { a: arc, b: oj });
                }
            }
        }
        // An open chain's offset ends square to the chain's ends.
        if !chain.closed {
            for (i, at_start) in [(0, true), (n - 1, false)] {
                let (source, reversed) = pieces_of[i];
                let (s, e) = self.curves[&source].endpoints().expect("a line or arc");
                let end = if at_start != reversed { s } else { e };
                let image = if at_start { starts[i] } else { ends[i] };
                self.square_end(source, end, image);
            }
        }
        Ok(Offsetted {
            curves: made,
            distance: distance.expect("a chain has a first piece"),
        })
    }

    /// Keeps `image`, an end of the offset of `source`, square to `source`
    /// at its end `end`: on the line through `end` across a line, or
    /// through an arc's center.
    fn square_end(&mut self, source: CurveId, end: PointId, image: PointId) {
        match self.curves[&source].kind {
            CurveKind::Line { .. } => {
                let across = self.add_line(end, image);
                self.set_construction(across, true);
                self.constrain(Constraint::Perpendicular { a: source, b: across });
            }
            _ => {
                let Some(Plain::Round { center, .. }) = Plain::of(self, source, false) else {
                    return;
                };
                let m = self.add_point(S::from_f64(center[0]), S::from_f64(center[1]));
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
    use geop_core_math::scalars::ScalInF64;
    use std::f64::consts::PI;

    type T = ScalInF64;

    fn n(x: f64) -> T {
        T::from_f64(x)
    }

    /// A `w` x `h` rectangle at the origin, counter-clockwise, fully
    /// constrained; its lines, and the dimension of its width.
    fn rectangle(s: &mut Sketch<T>, w: f64, h: f64) -> (Vec<CurveId>, ConstraintId) {
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
    fn bounds(s: &Sketch<T>, curves: &[CurveId]) -> [f64; 4] {
        let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for c in curves {
            for p in s.curves[c].points() {
                let q = xy(s, p);
                b = [b[0].min(q[0]), b[1].min(q[1]), b[2].max(q[0]), b[3].max(q[1])];
            }
        }
        b
    }

    fn near(a: [f64; 4], b: [f64; 4]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-7)
    }

    fn set(s: &mut Sketch<T>, k: ConstraintId, v: f64) {
        match s.constraints.get_mut(&k) {
            Some(Constraint::Offset { value, .. } | Constraint::Length { value, .. }) => {
                *value = n(v)
            }
            other => panic!("{other:?}"),
        }
    }

    fn lines(s: &Sketch<T>, curves: &[CurveId]) -> Vec<CurveId> {
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
        let mut s = Sketch::<T>::new();
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
        s.enclose::<T>().unwrap();
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
            let mut s = Sketch::<T>::new();
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
            s.enclose::<T>().unwrap();
        }
    }

    /// A slot runs on smoothly all round: its offset is a slot about it.
    #[test]
    fn a_slot_offsets_into_a_slot() {
        let mut s = Sketch::<T>::new();
        let p = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]
            .map(|q| s.add_point(n(q[0]), n(q[1])));
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
        s.enclose::<T>().unwrap();
    }

    /// An open chain's offset ends square to it.
    #[test]
    fn open_chains_end_square() {
        let mut s = Sketch::<T>::new();
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
        s.enclose::<T>().unwrap();
    }

    /// A circle offsets into one about the same center.
    #[test]
    fn circles_offset_about_their_center() {
        let mut s = Sketch::<T>::new();
        let c = s.add_point(n(1.0), n(1.0));
        let circle = s.add_circle(c, n(1.0));
        let made = s.offset(&[circle], n(-0.5), Corners::Round).unwrap();
        assert!(matches!(
            s.curves[&made.curves[0]].kind,
            CurveKind::Circle { center, radius } if center == c && radius.to_f64() == 1.5
        ));
        assert!(s.solve().unwrap().converged);
    }

    /// What would cross itself is refused, naming what does.
    #[test]
    fn offsets_that_would_cross_themselves_are_refused() {
        let mut s = Sketch::<T>::new();
        let (rect, _) = rectangle(&mut s, 2.0, 1.0);
        let e = s.offset(&rect, n(0.6), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("too short"), "{e}");
        let c = s.add_point(n(5.0), n(0.0));
        let circle = s.add_circle(c, n(0.5));
        let e = s.offset(&[circle], n(0.5), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("smaller"), "{e}");
        let q = [[0.0, 3.0], [1.0, 4.0], [2.0, 3.0]].map(|q| s.add_point(n(q[0]), n(q[1])));
        let spline = s.add_spline(q.to_vec());
        let e = s.offset(&[spline], n(0.1), Corners::Round).unwrap_err();
        assert!(e.root_message().contains("spline"), "{e}");
        let e = s
            .offset(&[rect[0], rect[2]], n(0.1), Corners::Round)
            .unwrap_err();
        assert!(e.root_message().contains("separate chains"), "{e}");
    }

    /// The chain through a curve runs on as long as it does not branch.
    #[test]
    fn chains_through_a_curve() {
        let mut s = Sketch::<T>::new();
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
}
