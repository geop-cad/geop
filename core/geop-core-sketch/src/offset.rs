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
//! Splines are refused — their offset is no spline — and so is an arc
//! offset past its center, and an offset that leaves nothing.

use geop_core_math::solvers::system::RELATIVE_TOLERANCE;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use serde::{Deserialize, Serialize};

use crate::{
    plain::{
        self, P2, Plain, add, angle_between, bounds, cross, distance, dot, perp, scale, sub, unit,
        wrap, xy,
    },
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
#[derive(Clone, Copy, Debug)]
struct Sub {
    elem: usize,
    plain: Plain,
    from_start: bool,
    to_end: bool,
}

/// A piece of a chain, along the chain's direction, in plain numbers.
fn oriented(sketch: &Sketch, curve: CurveId, reversed: bool) -> GeopResult<Plain> {
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

impl Sketch {
    /// How far `p` is from the chain `chain`: positive to its left.
    pub fn chain_side(&self, chain: &Chain, p: P2) -> GeopResult<f64> {
        let mut best: Option<(f64, f64)> = None;
        for edge in &chain.edges {
            let piece = oriented(self, edge.curve, edge.reversed)?;
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

    /// Offsets the chain `curves` (see [`Sketch::chain`]) by `distance` —
    /// to its left, to its right if negative — its corners as `corners`
    /// says (see the module docs). Returns the curves made, and the
    /// dimension of the distance.
    pub fn offset(
        &mut self,
        curves: &[CurveId],
        distance: Scalar,
        corners: Corners,
    ) -> GeopResult<Offsetted> {
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

        let n = pieces_of.len();
        // The finest the solver tells points apart, in this sketch: one
        // part in `RELATIVE_TOLERANCE` of its size.
        let resolution = {
            let size = bounds(self.points.keys().map(|&p| xy(self, p)))
                .map_or(0.0, |(lo, hi)| (hi[0] - lo[0]).hypot(hi[1] - lo[1]));
            // As the solver: one, where the sketch has no size at all.
            let size = if size > 0.0 { size } else { 1.0 };
            size / RELATIVE_TOLERANCE as f64
        };
        let pieces: Vec<Plain> = pieces_of
            .iter()
            .map(|&(c, r)| oriented(self, c, r))
            .collect::<GeopResult<_>>()?;
        let offsets: Vec<Plain> = pieces
            .iter()
            .zip(&pieces_of)
            .map(|(&p, &(c, _))| offset_plain(p, d).ok_or_else(|| too_far(c)))
            .collect::<GeopResult<_>>()?;
        // Each joint: where the offsets of the pieces before and after it
        // end and start, and what joins them.
        enum Joint {
            /// They meet at one point, held there as `by` says.
            Shared { at: P2, by: Held },
            /// An arc round the corner, turning by `sweep`.
            Round { from: P2, to: P2, sweep: f64 },
        }
        /// What keeps offsets meeting at one point at one distance.
        enum Held {
            /// They run on smoothly: tangent there too.
            Tangent,
            /// A construction circle about the corner touching both.
            Circle,
            /// They run on straight: the point square to the corner, as an
            /// open end is.
            Square,
        }
        let joints = if chain.closed { n } else { n - 1 };
        let mut joint_of = Vec::new();
        for i in 0..joints {
            let j = (i + 1) % n;
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
                (offsets[i], offsets[j]),
                (Plain::Line { .. }, Plain::Line { .. })
            );
            joint_of.push(if smooth {
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
                let crossings = offsets[i].full().crossings(&offsets[j].full(), false);
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
            });
        }

        // Where each offset piece starts and ends.
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
        // The offset drawn round the chain, before it is trimmed: each
        // piece's offset between where it starts and ends — the wrong way
        // round where the piece is too short for its corners — and the arcs
        // round the corners between them, in order.
        let mut path: Vec<(Plain, Of)> = Vec::new();
        for (i, offset) in offsets.iter().enumerate() {
            let (s, e) = (start_of(i), end_of(i));
            let piece = match *offset {
                Plain::Line { .. } => (plain::distance(s, e) > resolution).then_some(Plain::Line {
                    a: s,
                    b: e,
                    infinite: false,
                }),
                Plain::Round {
                    span: Some((_, sweep)),
                    ..
                } => {
                    let (u, v) = (unwrapped(offset, s, true), unwrapped(offset, e, false));
                    Plain::arc(s, e, sweep * (v - u))
                }
                _ => None,
            };
            // A piece of no length between its corners leaves its
            // neighbours meeting there.
            if let Some(piece) = piece {
                path.push((piece, Of::Piece(i)));
            }
            if let Some(Joint::Round { from, to, sweep }) = joint_of.get(i)
                && let Some(arc) = Plain::arc(*from, *to, *sweep)
            {
                path.push((arc, Of::Round(i)));
            }
        }
        let m = path.len();

        // Cut where it crosses itself, and keep what is no nearer the chain
        // than the distance: what a piece too short for its corners, or
        // offsets running into each other, leave of the offset.
        let near_end = |plain: &Plain, q: P2| {
            plain::distance(plain.at(0.0), q) <= resolution
                || plain::distance(plain.at(1.0), q) <= resolution
        };
        let to_chain = |q: P2| {
            pieces
                .iter()
                .map(|p| plain::distance(q, p.at(p.nearest(q))))
                .fold(f64::INFINITY, f64::min)
        };
        let mut subs: Vec<Sub> = Vec::new();
        for (k, (plain, _)) in path.iter().enumerate() {
            let mut cuts: Vec<f64> = Vec::new();
            for (j, (other, _)) in path.iter().enumerate() {
                if j != k {
                    cuts.extend(
                        plain
                            .crossings(other, false)
                            .into_iter()
                            .filter(|&q| !near_end(plain, q))
                            .map(|q| plain.param(q)),
                    );
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|a, b| plain::distance(plain.at(*a), plain.at(*b)) <= resolution);
            let ts: Vec<f64> = std::iter::once(0.0)
                .chain(cuts)
                .chain(std::iter::once(1.0))
                .collect();
            for w in ts.windows(2) {
                let part = plain.part(w[0], w[1]);
                if to_chain(part.at(0.5)) > d.abs() - resolution {
                    subs.push(Sub {
                        elem: k,
                        plain: part,
                        from_start: w[0] == 0.0,
                        to_end: w[1] == 1.0,
                    });
                }
            }
        }
        if subs.is_empty() {
            return Err(GeopError::new(format!(
                "the offset by {d} leaves nothing: the chain is too short or narrow for it"
            )));
        }

        // Each part on to the one starting where it ends: the next of its
        // piece's own along the path where it reaches its end, else the one
        // starting at the crossing it was cut at.
        let mut next: Vec<Option<usize>> = vec![None; subs.len()];
        for a in 0..subs.len() {
            let end = subs[a].plain.at(1.0);
            let open_end = subs[a].to_end && subs[a].elem == m - 1 && !chain.closed;
            let found: Vec<usize> = (0..subs.len())
                .filter(|&b| {
                    let joined = if subs[a].to_end {
                        subs[b].from_start && subs[b].elem == (subs[a].elem + 1) % m
                    } else {
                        !subs[b].from_start
                    };
                    b != a && joined && plain::distance(subs[b].plain.at(0.0), end) <= resolution
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
                        subs[b].plain.at(0.0)
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

        // Built: a point where each part ends, shared with the next; then
        // the parts, each tied to its source; then what joins them.
        let mut start_point: Vec<Option<PointId>> = vec![None; subs.len()];
        let mut end_point: Vec<Option<PointId>> = vec![None; subs.len()];
        for a in 0..subs.len() {
            let p = self.new_point_at(subs[a].plain.at(1.0));
            end_point[a] = Some(p);
            if let Some(b) = next[a] {
                start_point[b] = Some(p);
            }
        }
        for a in 0..subs.len() {
            if start_point[a].is_none() {
                start_point[a] = Some(self.new_point_at(subs[a].plain.at(0.0)));
            }
        }
        let corner_of = |sketch: &Sketch, i: usize| -> PointId {
            let (source, reversed) = pieces_of[i];
            let (s, e) = sketch.curves[&source].endpoints().expect("a line or arc");
            if reversed { s } else { e }
        };
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
                    let curve = match (&self.curves[&source].kind, subs[a].plain) {
                        (CurveKind::Line { .. }, _) => self.add_line(s, e),
                        (
                            CurveKind::Arc { .. },
                            Plain::Round {
                                span: Some((_, sweep)),
                                ..
                            },
                        ) => self.add_arc(
                            s,
                            e,
                            Scalar::from_f64(if reversed { -sweep } else { sweep }),
                        ),
                        (kind, _) => unreachable!("chained and trimmed as a line or arc: {kind:?}"),
                    };
                    if distance.is_none() {
                        distance = Some(self.constrain(Constraint::Offset {
                            a: source,
                            b: curve,
                            value,
                        }));
                    } else if let CurveKind::Line { .. } = self.curves[&source].kind {
                        self.constrain(Constraint::Parallel {
                            a: source,
                            b: curve,
                        });
                    } else {
                        self.constrain(Constraint::Concentric {
                            a: source,
                            b: curve,
                        });
                    }
                    curve
                }
                Of::Round(i) => {
                    let Plain::Round {
                        span: Some((_, sweep)),
                        ..
                    } = subs[a].plain
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
        // Across the chain's corners, as the offsets meet there; a part
        // joined so to one whose distance is held holds it too.
        let mut held = vec![false; subs.len()];
        let first = (0..subs.len())
            .find(|&a| matches!(path[subs[a].elem].1, Of::Piece(_)))
            .ok_or_else(|| {
                GeopError::new(format!(
                    "the offset by {d} leaves only arcs round corners: the chain is too short for it"
                ))
            })?;
        held[first] = true;
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
            match (path[subs[a].elem].1, path[subs[b].elem].1, &joint_of[i]) {
                (Of::Piece(_), Of::Piece(_), Joint::Shared { by, .. }) => match by {
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
        let mut reference: Option<CurveId> = None;
        for a in 0..subs.len() {
            if held[a] {
                continue;
            }
            let reference = *reference.get_or_insert_with(|| {
                let first_piece = match path[subs[first].elem].1 {
                    Of::Piece(i) | Of::Round(i) => i,
                };
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
        // An open chain's offset ends square to the chain's ends.
        if !chain.closed {
            for a in 0..subs.len() {
                let (k, at_start) = match (subs[a].from_start, subs[a].to_end, subs[a].elem) {
                    (true, _, 0) if previous[a].is_none() => (0, true),
                    (_, true, k) if k == m - 1 && next[a].is_none() => (k, false),
                    _ => continue,
                };
                let Of::Piece(i) = path[k].1 else {
                    continue;
                };
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
        Ok(Offsetted {
            curves: made,
            distance: distance.expect("a part of a piece is kept"),
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
    /// middle, higher up.
    #[test]
    #[ignore = "bug: the solve stalls at its first step (Stop::Flat with 40 independent rows of 40, the dimension's residual 0.1 left) although the offset at 0.6 is enclosed (Krawczyk) and the configuration at 0.7 is the same one: the two arcs round the notch's corners share their crossing point, each held by its center and one tangency. Not yet traced into the least-squares step."]
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
