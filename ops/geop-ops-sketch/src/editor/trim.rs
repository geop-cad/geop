//! Trimming: the trim tool removes the piece of a curve between the nearest
//! places either side of where it is clicked — or crossed by a stroke — at
//! which the sketch meets it: a point constrained onto it, or another curve
//! crossing it. A curve met nowhere goes as a whole, as does a spline,
//! which is never cut.
//!
//! What is left keeps what it was. The first piece left of a curve is the
//! curve itself, with its id and its constraints; every further piece is
//! held on the same line or circle by constraint — collinear, or concentric
//! and of equal radius — and what is left of a circle is an arc about the
//! circle's center. A piece ends at a point of the sketch, or at a new point
//! constrained onto the curve that crosses it there. A constraint that no
//! longer means what it did goes with what was removed: a length, an equal
//! length, a midpoint, a tangency at an end that is gone, a point on a piece
//! that is gone. So does an end of a removed piece that nothing holds on to
//! any more.
//!
//! Where curves cross is found in plain numbers (see [`Plain`]): only a
//! suggestion of where a piece ends, which the constraint onto the curve
//! crossing there then holds — the solve after trimming puts it there.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use super::*;
use crate::geometry::Plain;

/// What ends a piece of a curve: a point of the sketch, or the plan's
/// crossing of that index, which gets a point of its own only where a
/// piece left ends there.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Mark {
    Point(PointId),
    Crossing(usize),
}

/// Where two curves cross that no point of the sketch stands for.
struct Crossing {
    curves: [CurveId; 2],
    at: P2,
}

/// How a curve is cut: its plain shape — none for a spline, which is never
/// cut — and what cuts it, in order along it (see [`Plain::param`]):
/// strictly between its ends, for an open curve.
struct Cuts {
    plain: Option<Plain>,
    marks: Vec<(f64, Mark)>,
}

impl Cuts {
    fn closed(&self) -> bool {
        self.plain.as_ref().is_some_and(Plain::closed)
    }

    /// How many pieces the marks cut it into: an open curve one more than
    /// there are marks, a closed one as many — and one while it has none.
    fn pieces(&self) -> usize {
        if self.closed() {
            self.marks.len().max(1)
        } else {
            self.marks.len() + 1
        }
    }

    /// The piece `t` along it lies in. An open curve's piece `i` runs from
    /// mark `i - 1` — its start for the first — to mark `i`, its end for
    /// the last; a closed one's from mark `i` to the next, round.
    fn piece_at(&self, t: f64) -> usize {
        let before = self.marks.iter().filter(|(m, _)| *m < t).count();
        if self.closed() {
            let n = self.pieces();
            (before + n - 1) % n
        } else {
            before
        }
    }

    /// Where along it piece `i` runs, from and to — past 1 for the piece
    /// of a closed curve that runs round through its start.
    fn range(&self, i: usize) -> (f64, f64) {
        let n = self.marks.len();
        if self.closed() {
            if n == 0 {
                return (0.0, 1.0);
            }
            let to = self.marks[(i + 1) % n].0;
            (self.marks[i].0, if i + 1 == n { to + 1.0 } else { to })
        } else {
            let from = if i == 0 { 0.0 } else { self.marks[i - 1].0 };
            (from, if i == n { 1.0 } else { self.marks[i].0 })
        }
    }
}

/// Where a mark is on what is left of a curve.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Place {
    /// An end of a piece left.
    End,
    /// Inside the piece left that is now the curve of this id.
    On(CurveId),
    /// Between two pieces removed.
    Gone,
    /// Not between the curve's ends at all: on the line through it, beyond.
    Off,
}

/// What is left of a curve trimmed: what ends its pieces, in order along it
/// — an open curve's ends among them — and which of its pieces are left,
/// joined into runs from one of those to another.
struct Layout {
    closed: bool,
    bounds: Vec<(f64, Mark)>,
    /// Of every piece, the run it is part of — none for a piece removed.
    run_of: Vec<Option<usize>>,
    /// Every run, from bound to bound.
    runs: Vec<(usize, usize)>,
}

impl Layout {
    /// What is left of `curve`, cut by `cuts`, with the pieces `removed`.
    fn new(sketch: &Sketch, curve: CurveId, cuts: &Cuts, removed: &BTreeSet<usize>) -> Self {
        let closed = cuts.closed();
        let mut bounds = cuts.marks.clone();
        if !closed && let Some((start, end)) = sketch.curves[&curve].endpoints() {
            bounds.insert(0, (0.0, Mark::Point(start)));
            bounds.push((1.0, Mark::Point(end)));
        }
        let n = cuts.pieces();
        let kept = |i: usize| !removed.contains(&i);
        let mut run_of = vec![None; n];
        let mut runs = Vec::new();
        // A closed curve with a piece removed is gone if that is its only
        // piece; otherwise its runs are read starting after a piece removed,
        // so none of them is split where it runs round through its start.
        let first = match (closed, (0..n).find(|&i| !kept(i))) {
            (true, Some(_)) if n < 2 => return Self::gone(closed, bounds, n),
            (true, Some(r)) => r + 1,
            _ => 0,
        };
        for k in 0..n {
            let i = (first + k) % n;
            if !kept(i) {
                continue;
            }
            let starts = k == 0 || !kept((i + n - 1) % n);
            if starts {
                // A closed curve's piece `i` runs from bound `i`, an open
                // one's from bound `i` too: its start is bound 0.
                runs.push((i, i));
            }
            let run = runs.len() - 1;
            run_of[i] = Some(run);
            runs[run].1 = if closed { (i + 1) % n } else { i + 1 };
        }
        Layout {
            closed,
            bounds,
            run_of,
            runs,
        }
    }

    fn gone(closed: bool, bounds: Vec<(f64, Mark)>, n: usize) -> Self {
        Layout {
            closed,
            bounds,
            run_of: vec![None; n],
            runs: Vec::new(),
        }
    }

    /// Where `mark` is on what is left, the runs of it the curves `ids`.
    fn place(&self, mark: Mark, same: impl Fn(Mark, Mark) -> bool, ids: &[CurveId]) -> Place {
        let Some(j) = self.bounds.iter().position(|&(_, b)| same(b, mark)) else {
            return Place::Off;
        };
        let n = self.run_of.len();
        let (before, after) = if self.closed {
            (Some((j + n - 1) % n), Some(j))
        } else {
            (j.checked_sub(1), (j < n).then_some(j))
        };
        let run = |piece: Option<usize>| piece.and_then(|i| self.run_of[i]);
        match (run(before), run(after)) {
            (Some(a), Some(b)) if a == b => Place::On(ids[a]),
            (None, None) => Place::Gone,
            _ => Place::End,
        }
    }

    /// The run that ends at `mark`, if one does.
    fn run_ending_at(&self, mark: Mark, same: impl Fn(Mark, Mark) -> bool) -> Option<usize> {
        self.runs
            .iter()
            .position(|&(a, b)| same(self.bounds[a].1, mark) || same(self.bounds[b].1, mark))
    }
}

/// How the trim tool would cut every curve of a sketch it can trim — all
/// but the sketch's own axes, which only cut what crosses them.
pub(super) struct Plan<'a> {
    sketch: &'a Sketch,
    cuts: BTreeMap<CurveId, Cuts>,
    crossings: Vec<Crossing>,
}

impl<'a> Plan<'a> {
    /// How `sketch` is cut, the points `axes` the far ends of its own axes.
    pub(super) fn new(sketch: &'a Sketch, axes: &[PointId]) -> Self {
        let class = sketch.point_classes();
        let curves: Vec<CurveId> = sketch
            .curves
            .iter()
            .filter(|(_, c)| !c.points().iter().any(|p| axes.contains(p)))
            .map(|(&id, _)| id)
            .collect();
        // What lies on each curve by construction, by the class of the
        // point: its ends, and the points constrained onto it.
        let mut on: BTreeMap<CurveId, BTreeMap<PointId, PointId>> = BTreeMap::new();
        for &id in &curves {
            let mine = on.entry(id).or_default();
            if let Some((start, end)) = sketch.curves[&id].endpoints() {
                mine.insert(class[&start], start);
                mine.insert(class[&end], end);
            }
        }
        for c in sketch.constraints.values() {
            if let Constraint::PointOnCurve { point, curve } | Constraint::Midpoint { point, curve } =
                *c
                && let Some(mine) = on.get_mut(&curve)
            {
                mine.entry(class[&point]).or_insert(point);
            }
        }
        let mut cuts: BTreeMap<CurveId, Cuts> = curves
            .iter()
            .map(|&id| {
                let plain = Plain::of(sketch, id, false);
                (
                    id,
                    Cuts {
                        plain,
                        marks: Vec::new(),
                    },
                )
            })
            .collect();
        let add = |cuts: &mut BTreeMap<CurveId, Cuts>, curve: CurveId, at: P2, mark: Mark| {
            let c = cuts.get_mut(&curve).expect("every curve cut has its cuts");
            if let Some(plain) = &c.plain {
                let t = plain.param(at);
                if plain.closed() || (0.0 < t && t < 1.0) {
                    c.marks.push((t, mark));
                }
            }
        };
        // The points on a curve between its ends.
        for &id in &curves {
            let ends: Vec<PointId> = sketch.curves[&id]
                .endpoints()
                .map_or(Vec::new(), |(s, e)| vec![class[&s], class[&e]]);
            for (rep, &p) in &on[&id] {
                if !ends.contains(rep) {
                    add(&mut cuts, id, xy(sketch, p), Mark::Point(p));
                }
            }
        }
        // Where curves cross. Where two meet at a point of the sketch
        // already, that point is one of their crossings — the one nearest
        // it, of those found — and it cuts them as a point.
        let tangent = |a: CurveId, b: CurveId| {
            sketch.constraints.values().any(|c| {
                matches!(*c, Constraint::Tangent { a: x, b: y } if (x, y) == (a, b) || (x, y) == (b, a))
            })
        };
        let mut crossings = Vec::new();
        for (i, &a) in curves.iter().enumerate() {
            for &b in &curves[i + 1..] {
                let (Some(pa), Some(pb)) = (&cuts[&a].plain, &cuts[&b].plain) else {
                    continue;
                };
                let mut found = pa.crossings(pb, tangent(a, b));
                for (rep, &p) in &on[&a] {
                    if !on[&b].contains_key(rep) {
                        continue;
                    }
                    let at = xy(sketch, p);
                    if let Some(k) = (0..found.len())
                        .min_by(|&i, &j| dist(found[i], at).total_cmp(&dist(found[j], at)))
                    {
                        found.remove(k);
                    }
                }
                for at in found {
                    let mark = Mark::Crossing(crossings.len());
                    crossings.push(Crossing { curves: [a, b], at });
                    add(&mut cuts, a, at, mark);
                    add(&mut cuts, b, at, mark);
                }
            }
        }
        for c in cuts.values_mut() {
            c.marks.sort_by(|x, y| x.0.total_cmp(&y.0));
        }
        Plan {
            sketch,
            cuts,
            crossings,
        }
    }

    /// The piece of `curve` the point `q` near it is in — none for a curve
    /// that cannot be trimmed: one given from outside.
    pub(super) fn piece(&self, curve: CurveId, q: P2) -> Option<(CurveId, usize)> {
        if self.sketch.curves.get(&curve)?.fixed {
            return None;
        }
        let cuts = self.cuts.get(&curve)?;
        let piece = match &cuts.plain {
            Some(plain) => cuts.piece_at(plain.nearest(q)),
            None => 0,
        };
        Some((curve, piece))
    }

    /// The polyline the piece `piece` of `curve` is drawn as.
    pub(super) fn polyline(&self, curve: CurveId, piece: usize) -> Vec<P2> {
        match self.cuts.get(&curve) {
            Some(Cuts {
                plain: Some(plain), ..
            }) => {
                let (t0, t1) = self.cuts[&curve].range(piece);
                plain.polyline(t0, t1)
            }
            _ => polyline(self.sketch, curve),
        }
    }

    /// The sketch with the pieces `removed` — each a curve and which of its
    /// pieces (see [`Plan::piece`]) — removed, and what is left of their
    /// curves constrained as it was (see the module docs).
    pub(super) fn apply(&self, removed: &BTreeSet<(CurveId, usize)>) -> Sketch {
        let original = self.sketch;
        let class = original.point_classes();
        let same = |a: Mark, b: Mark| match (a, b) {
            (Mark::Point(p), Mark::Point(q)) => class[&p] == class[&q],
            (Mark::Crossing(i), Mark::Crossing(j)) => i == j,
            _ => false,
        };
        let mut pieces: BTreeMap<CurveId, BTreeSet<usize>> = BTreeMap::new();
        for &(curve, piece) in removed {
            pieces.entry(curve).or_default().insert(piece);
        }
        let layouts: BTreeMap<CurveId, Layout> = pieces
            .iter()
            .map(|(&c, removed)| (c, Layout::new(original, c, &self.cuts[&c], removed)))
            .collect();
        let mut next = original.clone();

        // A crossing a piece left ends at becomes a point.
        let mut crossing_points: BTreeMap<usize, PointId> = BTreeMap::new();
        for (i, crossing) in self.crossings.iter().enumerate() {
            let ends_here = crossing.curves.iter().any(|c| {
                layouts.get(c).is_some_and(|l| {
                    let ids = vec![*c; l.runs.len()];
                    l.place(Mark::Crossing(i), same, &ids) == Place::End
                })
            });
            if ends_here {
                let at = crossing.at;
                crossing_points.insert(
                    i,
                    next.add_point(Design::from_f64(at[0]), Design::from_f64(at[1])),
                );
            }
        }
        let point_of = |mark: Mark| match mark {
            Mark::Point(p) => p,
            Mark::Crossing(i) => crossing_points[&i],
        };

        // What is left of each curve: its first run the curve itself, every
        // further run a new curve of the same kind.
        let mut ids: BTreeMap<CurveId, Vec<CurveId>> = BTreeMap::new();
        let mut kinds: Vec<(CurveId, CurveKind)> = Vec::new();
        for (&c, layout) in &layouts {
            let curve = &original.curves[&c];
            let mut mine = Vec::new();
            for (k, &(a, b)) in layout.runs.iter().enumerate() {
                let (ta, start) = layout.bounds[a];
                let (tb, end) = layout.bounds[b];
                let (start, end) = (point_of(start), point_of(end));
                let span = if tb > ta { tb - ta } else { tb + 1.0 - ta };
                let kind = match curve.kind {
                    CurveKind::Line { .. } => CurveKind::Line { start, end },
                    CurveKind::Arc { sweep, .. } => CurveKind::Arc {
                        start,
                        end,
                        sweep: Design::from_f64(sweep.to_f64() * span),
                    },
                    CurveKind::Circle { .. } => CurveKind::Arc {
                        start,
                        end,
                        sweep: Design::from_f64(TAU * span),
                    },
                    CurveKind::Spline { .. } => unreachable!("a spline is never cut"),
                };
                let id = if k == 0 {
                    c
                } else {
                    let id = next.add_curve(kind.clone());
                    next.set_construction(id, curve.construction);
                    id
                };
                kinds.push((id, kind));
                mine.push(id);
            }
            ids.insert(c, mine);
        }
        let gone = |c: CurveId| ids.get(&c).is_some_and(|ids| ids.is_empty());
        let place = |c: CurveId, mark: Mark| match (layouts.get(&c), ids.get(&c)) {
            (Some(layout), Some(ids)) => layout.place(mark, same, ids),
            _ => Place::On(c),
        };

        // The constraints, following what is left of what they are on.
        let is_line = |c: CurveId| matches!(original.curves[&c].kind, CurveKind::Line { .. });
        let mut constraints = BTreeMap::new();
        for (&id, c) in &original.constraints {
            let curves = c.curves();
            if curves.iter().any(|&k| gone(k)) {
                continue;
            }
            let trimmed = |k: CurveId| layouts.contains_key(&k);
            let kept = match *c {
                Constraint::Length { curve, .. } if trimmed(curve) => None,
                Constraint::Equal { a, b } if (trimmed(a) || trimmed(b)) && is_line(a) => None,
                // Cut, an arc turns less, and a curve copies are moved by
                // moves them differently. (An offset holds for any piece:
                // it is about the line or circle.)
                Constraint::EqualSweep { a, b } if trimmed(a) || trimmed(b) => None,
                Constraint::Moved { by, .. } if trimmed(by) => None,
                Constraint::PointOnCurve { point, curve }
                | Constraint::Midpoint { point, curve }
                    if trimmed(curve) =>
                {
                    match place(curve, Mark::Point(point)) {
                        Place::On(curve) => Some(Constraint::PointOnCurve { point, curve }),
                        Place::Off if matches!(c, Constraint::PointOnCurve { .. }) => {
                            Some(c.clone())
                        }
                        _ => None,
                    }
                }
                Constraint::Tangent { a, b } if trimmed(a) || trimmed(b) => {
                    match original.shared_endpoint(a, b).ok().flatten() {
                        // Tangent at the end they share: on the runs that
                        // still end there, if they both do.
                        Some((a_at_end, _)) => {
                            let (s, e) = original.curves[&a].endpoints().expect("open, sharing");
                            let shared = Mark::Point(if a_at_end { e } else { s });
                            let run = |k: CurveId| match layouts.get(&k) {
                                Some(layout) => {
                                    layout.run_ending_at(shared, same).map(|run| ids[&k][run])
                                }
                                None => Some(k),
                            };
                            run(a)
                                .zip(run(b))
                                .map(|(a, b)| Constraint::Tangent { a, b })
                        }
                        None => Some(c.clone()),
                    }
                }
                _ => Some(c.clone()),
            };
            if let Some(kept) = kept {
                constraints.insert(id, kept);
            }
        }
        next.constraints = constraints;

        // What is left of a curve stays on its line or circle.
        for (&c, mine) in &ids {
            let curve = &original.curves[&c];
            if let (CurveKind::Circle { center, .. }, false) = (&curve.kind, mine.is_empty()) {
                next.constrain(Constraint::Center {
                    point: *center,
                    curve: c,
                });
            }
            for &other in mine.iter().skip(1) {
                if is_line(c) {
                    next.constrain(Constraint::Collinear { a: c, b: other });
                } else {
                    next.constrain(Constraint::Concentric { a: c, b: other });
                    next.constrain(Constraint::Equal { a: c, b: other });
                }
            }
        }
        // A new point lies on both curves crossing there, where it does not
        // end what is left of them.
        for (&i, &point) in &crossing_points {
            for curve in self.crossings[i].curves {
                match place(curve, Mark::Crossing(i)) {
                    Place::On(curve) => {
                        next.constrain(Constraint::PointOnCurve { point, curve });
                    }
                    Place::Off if !gone(curve) => {
                        next.constrain(Constraint::PointOnCurve { point, curve });
                    }
                    _ => {}
                }
            }
        }
        for (id, kind) in kinds {
            next.curves.get_mut(&id).expect("added or kept").kind = kind;
        }

        // The curves removed whole, and every point of a curve trimmed that
        // nothing is drawn through any more, nor centered on: with whatever
        // constraints still hold on to them.
        let deleted: Vec<CurveId> = ids
            .iter()
            .filter(|(_, ids)| ids.is_empty())
            .map(|(&c, _)| c)
            .collect();
        let alive = |p: PointId| {
            next.curves
                .iter()
                .filter(|(id, _)| !deleted.contains(id))
                .any(|(_, c)| c.points().contains(&p))
                || next.constraints.values().any(|k| {
                    matches!(*k, Constraint::Center { point, curve } if point == p && !deleted.contains(&curve))
                })
        };
        let orphans: Vec<PointId> = layouts
            .keys()
            .flat_map(|c| original.curves[c].points())
            .filter(|&p| !original.points[&p].fixed && !alive(p))
            .collect();
        next.remove(&orphans, &deleted, &[]);
        next
    }

    /// The pieces the points `met` — each near a curve — are in.
    pub(super) fn pieces(&self, met: &[(CurveId, P2)]) -> BTreeSet<(CurveId, usize)> {
        met.iter().filter_map(|&(c, q)| self.piece(c, q)).collect()
    }
}
