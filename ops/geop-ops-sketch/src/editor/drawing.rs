//! What each drawing tool builds from the points placed for it: the curves,
//! and the constraints that keep them what the tool drew — a rectangle's
//! sides square, a slot's ends tangent, a polygon's sides equal — so that
//! dragging or dimensioning it later keeps its shape.
//!
//! A line or arc drawn on from the end of another, running on smoothly
//! from it as drawn, is made tangent to it.
//!
//! The same construction serves the preview and the click: previewed, it
//! runs on a copy of the sketch with the pointer as the last point.

use std::f64::consts::{PI, TAU};

use super::*;
use crate::geometry::{angle_between, cross, dot, perp, rotate, scale, sweep_through, unit, wrap};

/// Lines drawn within this slope of horizontal or vertical get that
/// constraint.
const AUTO_HV_SLOPE: f64 = 0.034_920_769_491_747_23; // tan(2°)

/// A line or arc drawn on from another's end within this angle of running
/// on smoothly from it is made tangent to it.
const AUTO_TANGENT_ANGLE: f64 = 0.087_266_462_599_716_48; // 5°

/// What a construction needs beyond the points placed.
pub(super) struct Hints {
    /// A polygon's sides.
    pub sides: usize,
    /// Whether a polygon's sides touch its circle, rather than its corners
    /// lying on it.
    pub circumscribed: bool,
    /// A chain of lines: the curve it last added — which a tangent arc
    /// continues, and a line continues tangentially if it is an arc.
    pub tangent_to: Option<CurveId>,
    /// A center-point arc: how far it has turned, followed round.
    pub sweep: f64,
    /// The smallest size worth drawing, in the plane: what is smaller is a
    /// click on the same spot.
    pub min_size: f64,
}

/// What a construction built.
#[derive(Default)]
pub(super) struct Built {
    /// The point a chain of lines continues from.
    pub end: Option<PointId>,
    /// The curve it built last, which a tangent arc would continue.
    pub last: Option<CurveId>,
    /// A dimension it added whose value to ask for.
    pub prompt: Option<ConstraintId>,
}

impl Placed {
    /// The point it is in `sketch`: the point it snapped to, or a new one,
    /// constrained to what it snapped to.
    pub(super) fn materialize(&self, sketch: &mut Sketch) -> PointId {
        if let Some(Snap::Point(p)) = self.snap {
            return p;
        }
        let id = sketch.add_point(Design::from_f64(self.at[0]), Design::from_f64(self.at[1]));
        if let Some(snap) = self.snap {
            snap.constrain(sketch, id);
        }
        id
    }

    /// The existing point it snapped to, if any.
    fn existing(&self) -> Option<PointId> {
        match self.snap {
            Some(Snap::Point(p)) => Some(p),
            _ => None,
        }
    }
}

fn new_point(sketch: &mut Sketch, at: P2) -> PointId {
    sketch.add_point(Design::from_f64(at[0]), Design::from_f64(at[1]))
}

/// A line from `a` to `b`, horizontal or vertical by constraint if it is
/// drawn nearly so.
fn line(sketch: &mut Sketch, a: PointId, b: PointId) -> CurveId {
    let d = sub(pt(sketch, b), pt(sketch, a));
    let line = sketch.add_line(a, b);
    if d[1].abs() <= AUTO_HV_SLOPE * d[0].abs() {
        sketch.constrain(Constraint::Horizontal { line });
    } else if d[0].abs() <= AUTO_HV_SLOPE * d[1].abs() {
        sketch.constrain(Constraint::Vertical { line });
    }
    line
}

/// The direction in which a curve continuing `curve` from its end point
/// `p` leaves tangentially: on along a line, round along an arc.
pub(super) fn leaving_tangent(sketch: &Sketch, curve: CurveId, p: PointId) -> Option<P2> {
    let class = sketch.point_classes();
    let c = sketch.curves.get(&curve)?;
    let (start, end) = c.endpoints()?;
    let at_end = class[&end] == class[&p];
    if !at_end && class[&start] != class[&p] {
        return None;
    }
    let tangent = match &c.kind {
        CurveKind::Line { start, end } => sub(pt(sketch, *end), pt(sketch, *start)),
        CurveKind::Arc { start, end, sweep } => {
            let chord = sub(pt(sketch, *end), pt(sketch, *start));
            let half = sweep.to_f64() / 2.0;
            rotate(chord, if at_end { half } else { -half })
        }
        CurveKind::Spline { control_points, .. } => {
            let n = control_points.len();
            sub(
                pt(sketch, control_points[n - 1]),
                pt(sketch, control_points[n - 2]),
            )
        }
        CurveKind::Circle { .. } => return None,
    };
    // Leaving from the start runs against the curve's own direction.
    unit(if at_end {
        tangent
    } else {
        scale(tangent, -1.0)
    })
}

/// The newest curve with an end at `p`: what a tangent arc from `p`
/// continues.
pub(super) fn curve_ending_at(sketch: &Sketch, p: PointId) -> Option<CurveId> {
    let class = sketch.point_classes();
    sketch
        .curves
        .iter()
        .rev()
        .find(|(_, c)| {
            !c.construction
                && c.endpoints()
                    .is_some_and(|(s, e)| class[&s] == class[&p] || class[&e] == class[&p])
        })
        .map(|(&id, _)| id)
}

/// An arc from `start` to the point `to`, leaving `start` tangent to
/// `curve`, and so constrained.
fn tangent_arc(sketch: &mut Sketch, curve: CurveId, start: PointId, to: &Placed) -> Option<Built> {
    let t = leaving_tangent(sketch, curve, start)?;
    let chord = sub(to.at, pt(sketch, start));
    let half = angle_between(t, chord);
    if chord == [0.0, 0.0] || half.abs() >= PI {
        return None;
    }
    let end = to.materialize(sketch);
    let arc = sketch.add_arc(start, end, Design::from_f64(2.0 * half));
    sketch.constrain(Constraint::Tangent { a: curve, b: arc });
    Some(Built {
        end: Some(end),
        last: Some(arc),
        prompt: None,
    })
}

/// Makes `curve`, just drawn, tangent to every curve it runs on smoothly
/// from at an end the two share — drawn within [`AUTO_TANGENT_ANGLE`] of
/// that. Two lines are never tangent: one runs on along the other.
fn auto_tangent(sketch: &mut Sketch, curve: CurveId) {
    let Some((start, end)) = sketch.curves.get(&curve).and_then(|c| c.endpoints()) else {
        return;
    };
    let is_line =
        |sketch: &Sketch, c: CurveId| matches!(sketch.curves[&c].kind, CurveKind::Line { .. });
    for p in [start, end] {
        let Some(on) = leaving_tangent(sketch, curve, p) else {
            continue;
        };
        let others: Vec<CurveId> = sketch
            .curves
            .iter()
            .filter(|&(&id, c)| id != curve && c.construction == sketch.curves[&curve].construction)
            .map(|(&id, _)| id)
            .collect();
        for other in others {
            let tangent = sketch.constraints.values().any(|k| {
                matches!(*k, Constraint::Tangent { a, b } if (a, b) == (other, curve) || (a, b) == (curve, other))
            });
            if tangent || is_line(sketch, other) && is_line(sketch, curve) {
                continue;
            }
            // Smooth: `other` goes on beyond `p` the way `curve` comes
            // into it.
            if let Some(beyond) = leaving_tangent(sketch, other, p)
                && angle_between(beyond, scale(on, -1.0)).abs() <= AUTO_TANGENT_ANGLE
            {
                sketch.constrain(Constraint::Tangent { a: other, b: curve });
            }
        }
    }
}

/// The center of the circle through `a`, `b` and `c`; none for points on
/// one line.
fn circumcenter(a: P2, b: P2, c: P2) -> Option<P2> {
    let (b, c) = (sub(b, a), sub(c, a));
    let d = 2.0 * cross(b, c);
    if d == 0.0 {
        return None;
    }
    let (bb, cc) = (dot(b, b), dot(c, c));
    let center = [(c[1] * bb - b[1] * cc) / d, (b[0] * cc - c[0] * bb) / d];
    center.iter().all(|v| v.is_finite()).then(|| add(a, center))
}

/// The four corners `corners` joined into a loop of lines; returns the
/// lines.
fn closed(sketch: &mut Sketch, corners: &[PointId]) -> Vec<CurveId> {
    (0..corners.len())
        .map(|i| sketch.add_line(corners[i], corners[(i + 1) % corners.len()]))
        .collect()
}

/// Lines `lines` of a rectangle with sides along the axes, so constrained.
fn axis_aligned(sketch: &mut Sketch, lines: &[CurveId]) {
    for (i, &line) in lines.iter().enumerate() {
        sketch.constrain(if i % 2 == 0 {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
}

/// What `tool` builds into `sketch` from `placed` — all the points it
/// needs — or `None` where they make nothing: two clicks on one spot, three
/// on one line. `arc`: a chain of lines draws a tangent arc.
pub(super) fn construct(
    tool: DrawTool,
    arc: bool,
    sketch: &mut Sketch,
    placed: &[Placed],
    hints: &Hints,
) -> Option<Built> {
    let built = build(tool, arc, sketch, placed, hints)?;
    if matches!(
        tool,
        DrawTool::Line | DrawTool::Arc | DrawTool::CenterArc | DrawTool::TangentArc
    ) && let Some(last) = built.last
    {
        auto_tangent(sketch, last);
    }
    Some(built)
}

/// What [`construct`] builds, before it is made tangent where it runs on
/// smoothly.
fn build(
    tool: DrawTool,
    arc: bool,
    sketch: &mut Sketch,
    placed: &[Placed],
    hints: &Hints,
) -> Option<Built> {
    use DrawTool::*;
    let far = |a: P2, b: P2| dist(a, b) > hints.min_size;
    match (tool, placed) {
        (Point, [p]) => {
            let end = p.materialize(sketch);
            Some(Built {
                end: Some(end),
                ..Built::default()
            })
        }
        (Line, [a, b]) if arc => {
            let start = a.existing()?;
            tangent_arc(sketch, hints.tangent_to?, start, b)
        }
        (TangentArc, [a, b]) => {
            let start = a.existing()?;
            tangent_arc(sketch, curve_ending_at(sketch, start)?, start, b)
        }
        (Line, [a, b]) if far(a.at, b.at) => {
            let (start, end) = (a.materialize(sketch), b.materialize(sketch));
            if start == end {
                return None;
            }
            let l = line(sketch, start, end);
            // Out of a tangent arc of the chain, on tangentially — along
            // with whatever horizontal or vertical it was drawn near: the
            // arc gives way to both.
            if let Some(arc) = hints.tangent_to
                && matches!(sketch.curves[&arc].kind, CurveKind::Arc { .. })
            {
                sketch.constrain(Constraint::Tangent { a: arc, b: l });
            }
            Some(Built {
                end: Some(end),
                last: Some(l),
                prompt: None,
            })
        }
        (Rectangle, [a, c]) => {
            let (p, q) = (a.at, c.at);
            if (p[0] - q[0]).abs() <= hints.min_size || (p[1] - q[1]).abs() <= hints.min_size {
                return None;
            }
            // Two opposite corners, and the two they imply. The sides are
            // horizontal and vertical by constraint, so it stays a
            // rectangle whatever is dragged later.
            let first = a.materialize(sketch);
            let opposite = c.materialize(sketch);
            let second = new_point(sketch, [q[0], p[1]]);
            let fourth = new_point(sketch, [p[0], q[1]]);
            let lines = closed(sketch, &[first, second, opposite, fourth]);
            axis_aligned(sketch, &lines);
            Some(Built::default())
        }
        (CenterRectangle, [m, c]) => {
            let (center, q) = (m.at, c.at);
            let d = sub(q, center);
            if d[0].abs() <= hints.min_size || d[1].abs() <= hints.min_size {
                return None;
            }
            let corner = c.materialize(sketch);
            let others = [[-d[0], d[1]], [-d[0], -d[1]], [d[0], -d[1]]]
                .map(|o| new_point(sketch, add(center, o)));
            let corners = [corner, others[0], others[1], others[2]];
            let lines = closed(sketch, &corners);
            axis_aligned(sketch, &lines);
            // Its center is the middle of a diagonal.
            let diagonal = sketch.add_line(corners[0], corners[2]);
            sketch.set_construction(diagonal, true);
            let mid = m.materialize(sketch);
            sketch.constrain(Constraint::Midpoint {
                point: mid,
                curve: diagonal,
            });
            Some(Built::default())
        }
        (ThreePointRectangle, [a, b, c]) => {
            let side = sub(b.at, a.at);
            let n = perp(unit(side)?);
            let h = dot(sub(c.at, a.at), n);
            if !far(a.at, b.at) || h.abs() <= hints.min_size {
                return None;
            }
            let (p0, p1) = (a.materialize(sketch), b.materialize(sketch));
            let p2 = new_point(sketch, add(b.at, scale(n, h)));
            let p3 = new_point(sketch, add(a.at, scale(n, h)));
            let lines = closed(sketch, &[p0, p1, p2, p3]);
            sketch.constrain(Constraint::Perpendicular {
                a: lines[0],
                b: lines[1],
            });
            sketch.constrain(Constraint::Parallel {
                a: lines[0],
                b: lines[2],
            });
            sketch.constrain(Constraint::Parallel {
                a: lines[1],
                b: lines[3],
            });
            Some(Built::default())
        }
        (Circle, [m, r]) => {
            let radius = dist(m.at, r.at);
            if radius <= hints.min_size {
                return None;
            }
            let center = m.materialize(sketch);
            let circle = sketch.add_circle(center, Design::from_f64(radius));
            if let Some(p) = r.existing() {
                sketch.constrain(Constraint::PointOnCurve {
                    point: p,
                    curve: circle,
                });
            }
            Some(Built::default())
        }
        (ThreePointCircle, [a, b, c]) => {
            let center = circumcenter(a.at, b.at, c.at)?;
            let radius = dist(center, a.at);
            if !far(a.at, b.at) || !far(b.at, c.at) || !far(a.at, c.at) {
                return None;
            }
            let m = new_point(sketch, center);
            let circle = sketch.add_circle(m, Design::from_f64(radius));
            // Only what it was drawn through stays on it: the points it
            // snapped to.
            for p in [a, b, c].into_iter().filter_map(Placed::existing) {
                sketch.constrain(Constraint::PointOnCurve {
                    point: p,
                    curve: circle,
                });
            }
            Some(Built::default())
        }
        (Arc, [a, b, c]) => {
            let sweep = sweep_through(a.at, b.at, c.at);
            if !sweep.is_finite() || !far(a.at, b.at) {
                return None;
            }
            let (start, end) = (a.materialize(sketch), b.materialize(sketch));
            if start == end {
                return None;
            }
            let arc = sketch.add_arc(start, end, Design::from_f64(sweep));
            Some(Built {
                end: Some(end),
                last: Some(arc),
                prompt: None,
            })
        }
        (CenterArc, [m, s, e]) => {
            let (from, to) = (sub(s.at, m.at), sub(e.at, m.at));
            let radius = from[0].hypot(from[1]);
            if radius <= hints.min_size || to == [0.0, 0.0] {
                return None;
            }
            let raw = angle_between(from, to);
            let sweep = if hints.sweep == 0.0 {
                raw
            } else {
                hints.sweep + wrap(raw - hints.sweep)
            };
            if sweep.abs() >= TAU || sweep.abs() * radius <= hints.min_size {
                return None;
            }
            let start = s.materialize(sketch);
            let end = match e.existing() {
                Some(p) => p,
                None => new_point(sketch, add(m.at, rotate(from, sweep))),
            };
            if start == end {
                return None;
            }
            let arc = sketch.add_arc(start, end, Design::from_f64(sweep));
            let center = m.materialize(sketch);
            sketch.constrain(Constraint::Center {
                point: center,
                curve: arc,
            });
            Some(Built {
                end: Some(end),
                last: Some(arc),
                prompt: None,
            })
        }
        (Polygon, [m, v]) => {
            let n = hints.sides.max(3);
            let to = sub(v.at, m.at);
            let size = to[0].hypot(to[1]);
            if size <= hints.min_size {
                return None;
            }
            // Clicked: a corner, or — circumscribed — the middle of a side,
            // half a side's turn from its corners and further out.
            let half_turn = PI / n as f64;
            let first = if hints.circumscribed {
                scale(rotate(to, half_turn), 1.0 / half_turn.cos())
            } else {
                to
            };
            // Corners on a circle, sides of one length: regular.
            let center = m.materialize(sketch);
            let circle = sketch.add_circle(center, Design::from_f64(dist(first, [0.0, 0.0])));
            sketch.set_construction(circle, true);
            let mut corners = vec![if hints.circumscribed {
                new_point(sketch, add(m.at, first))
            } else {
                v.materialize(sketch)
            }];
            for k in 1..n {
                let at = add(m.at, rotate(first, TAU * k as f64 / n as f64));
                corners.push(new_point(sketch, at));
            }
            for &point in &corners {
                sketch.constrain(Constraint::PointOnCurve {
                    point,
                    curve: circle,
                });
            }
            let sides = closed(sketch, &corners);
            for &b in &sides[1..] {
                sketch.constrain(Constraint::Equal { a: sides[0], b });
            }
            // Circumscribed, the circle the sides touch: its diameter is
            // the width across flats, to give.
            if hints.circumscribed {
                let inner = sketch.add_circle(center, Design::from_f64(size));
                sketch.set_construction(inner, true);
                sketch.constrain(Constraint::Tangent {
                    a: sides[n - 1],
                    b: inner,
                });
            }
            Some(Built::default())
        }
        (Slot, [a, b, w]) => {
            let axis = unit(sub(b.at, a.at))?;
            let n = perp(axis);
            let radius = dot(sub(w.at, a.at), n).abs();
            if !far(a.at, b.at) || radius <= hints.min_size {
                return None;
            }
            let (ca, cb) = (a.materialize(sketch), b.materialize(sketch));
            let centerline = sketch.add_line(ca, cb);
            sketch.set_construction(centerline, true);
            let offset = scale(n, radius);
            // Counter-clockwise: along the right side, round `b`, back
            // along the left side, round `a`.
            let p = [
                new_point(sketch, sub(a.at, offset)),
                new_point(sketch, sub(b.at, offset)),
                new_point(sketch, add(b.at, offset)),
                new_point(sketch, add(a.at, offset)),
            ];
            let right = sketch.add_line(p[0], p[1]);
            let round_b = sketch.add_arc(p[1], p[2], Design::from_f64(PI));
            let left = sketch.add_line(p[2], p[3]);
            let round_a = sketch.add_arc(p[3], p[0], Design::from_f64(PI));
            for (x, y) in [
                (right, round_b),
                (round_b, left),
                (left, round_a),
                (round_a, right),
            ] {
                sketch.constrain(Constraint::Tangent { a: x, b: y });
            }
            sketch.constrain(Constraint::Center {
                point: cb,
                curve: round_b,
            });
            sketch.constrain(Constraint::Center {
                point: ca,
                curve: round_a,
            });
            sketch.constrain(Constraint::Equal {
                a: round_a,
                b: round_b,
            });
            Some(Built::default())
        }
        (ArcSlot, [m, s, e, w]) => {
            let (from, to) = (sub(s.at, m.at), sub(e.at, m.at));
            let radius = from[0].hypot(from[1]);
            if radius <= hints.min_size || to == [0.0, 0.0] {
                return None;
            }
            let raw = angle_between(from, to);
            let sweep = if hints.sweep == 0.0 {
                raw
            } else {
                hints.sweep + wrap(raw - hints.sweep)
            };
            let width = (dist(w.at, m.at) - radius).abs();
            if sweep.abs() >= TAU
                || sweep.abs() * radius <= hints.min_size
                || width <= hints.min_size
                || width >= radius
            {
                return None;
            }
            let center = m.materialize(sketch);
            let a = s.materialize(sketch);
            let b_at = add(m.at, rotate(from, sweep));
            let b = match e.existing() {
                Some(p) => p,
                None => new_point(sketch, b_at),
            };
            if a == b {
                return None;
            }
            // Its centerline, about the center.
            let centerline = sketch.add_arc(a, b, Design::from_f64(sweep));
            sketch.set_construction(centerline, true);
            sketch.constrain(Constraint::Center {
                point: center,
                curve: centerline,
            });
            // Round: out along the outer side, round `b`, back along the
            // inner side, round `a` — each end a half circle about its
            // center of the centerline.
            let (ua, ub) = (unit(from)?, unit(sub(b_at, m.at))?);
            let p = [
                add(s.at, scale(ua, width)),
                add(b_at, scale(ub, width)),
                sub(b_at, scale(ub, width)),
                sub(s.at, scale(ua, width)),
            ]
            .map(|q| new_point(sketch, q));
            let turn = sweep.signum() * PI;
            let outer = sketch.add_arc(p[0], p[1], Design::from_f64(sweep));
            let round_b = sketch.add_arc(p[1], p[2], Design::from_f64(turn));
            // The inner side turns the way the centerline does, from `a`'s
            // end to `b`'s.
            let inner = sketch.add_arc(p[3], p[2], Design::from_f64(sweep));
            let round_a = sketch.add_arc(p[3], p[0], Design::from_f64(turn));
            for (x, y) in [
                (outer, round_b),
                (round_b, inner),
                (inner, round_a),
                (round_a, outer),
            ] {
                sketch.constrain(Constraint::Tangent { a: x, b: y });
            }
            // Both sides about the center. With the ends tangent to both,
            // the inner side's center follows from the rest, so one of
            // these says what the others do: true on the solution, which
            // is all `System::enclose` asks of a redundant constraint.
            for side in [outer, inner] {
                sketch.constrain(Constraint::Concentric {
                    a: centerline,
                    b: side,
                });
            }
            sketch.constrain(Constraint::Center {
                point: b,
                curve: round_b,
            });
            sketch.constrain(Constraint::Center {
                point: a,
                curve: round_a,
            });
            Some(Built::default())
        }
        (Spline, placed) if placed.len() >= 2 => {
            let points: Vec<PointId> = placed.iter().map(|p| p.materialize(sketch)).collect();
            let spline = sketch.add_spline(points);
            Some(Built {
                last: Some(spline),
                ..Built::default()
            })
        }
        (Fillet, [corner]) => fillet(sketch, corner.existing()?),
        (Chamfer, [corner]) => chamfer(sketch, corner.existing()?),
        _ => None,
    }
}

/// The two lines meeting at the corner `p`, each with its end there and its
/// other end — `None` where `p` is no corner of exactly two lines.
type Corner = [(CurveId, PointId, PointId); 2];

fn corner_lines(sketch: &Sketch, p: PointId) -> Option<Corner> {
    let class = sketch.point_classes();
    let at_p = |q: PointId| class[&q] == class[&p];
    let lines: Vec<(CurveId, PointId, PointId)> = sketch
        .curves
        .iter()
        .filter(|(_, c)| !c.construction && !c.fixed)
        .filter_map(|(&id, c)| match c.kind {
            CurveKind::Line { start, end } if at_p(start) => Some((id, start, end)),
            CurveKind::Line { start, end } if at_p(end) => Some((id, end, start)),
            _ => None,
        })
        .collect();
    lines.try_into().ok()
}

/// Cuts both lines of `corner` back to `to`: each line's end at the corner
/// moved to its point of `to`.
fn cut_back(sketch: &mut Sketch, corner: &Corner, to: [PointId; 2]) -> Option<()> {
    for (&(line, old, _), new) in corner.iter().zip(to) {
        if let CurveKind::Line { start, end } = &mut sketch.curves.get_mut(&line)?.kind {
            if *start == old {
                *start = new;
            } else {
                *end = new;
            }
        }
    }
    Some(())
}

/// Bevels the corner where two lines meet at `p`: both lines cut back by
/// the same distance, a line between them, and that distance a dimension
/// to give. The corner stays, as the point both cut-back pieces — kept as
/// construction lines — run from, on both lines by constraint. `None`
/// where `p` is no corner of exactly two lines.
fn chamfer(sketch: &mut Sketch, p: PointId) -> Option<Built> {
    let corner = corner_lines(sketch, p)?;
    let [(l1, _, q1), (l2, _, q2)] = corner;
    let at = pt(sketch, p);
    let (d1, d2) = (sub(pt(sketch, q1), at), sub(pt(sketch, q2), at));
    let (u1, u2) = (unit(d1)?, unit(d2)?);
    let theta = angle_between(u1, u2).abs();
    if theta <= 0.0 || theta >= PI {
        return None;
    }
    let setback = d1[0].hypot(d1[1]).min(d2[0].hypot(d2[1])) / 3.0;
    let t1 = new_point(sketch, add(at, scale(u1, setback)));
    let t2 = new_point(sketch, add(at, scale(u2, setback)));
    cut_back(sketch, &corner, [t1, t2])?;
    sketch.add_line(t1, t2);
    let pieces = [t1, t2].map(|t| {
        let piece = sketch.add_line(p, t);
        sketch.set_construction(piece, true);
        piece
    });
    for line in [l1, l2] {
        sketch.constrain(Constraint::PointOnCurve {
            point: p,
            curve: line,
        });
    }
    sketch.constrain(Constraint::Equal {
        a: pieces[0],
        b: pieces[1],
    });
    let prompt = sketch.constrain(Constraint::Length {
        curve: pieces[0],
        value: Design::from_f64(setback),
    });
    Some(Built {
        prompt: Some(prompt),
        ..Built::default()
    })
}

/// Rounds the corner where two lines meet at `p`: both lines cut back, an
/// arc tangent to both between them, and its radius a dimension to give.
/// `None` where `p` is no corner of exactly two lines.
fn fillet(sketch: &mut Sketch, p: PointId) -> Option<Built> {
    let corner = corner_lines(sketch, p)?;
    let [(l1, _, q1), (l2, _, q2)] = corner;
    let at = pt(sketch, p);
    let (d1, d2) = (sub(pt(sketch, q1), at), sub(pt(sketch, q2), at));
    let (u1, u2) = (unit(d1)?, unit(d2)?);
    let theta = angle_between(u1, u2).abs();
    if theta <= 0.0 || theta >= PI {
        return None;
    }
    // Cut back a third of the shorter line, and round with the radius
    // that makes the arc tangent there.
    let setback = d1[0].hypot(d1[1]).min(d2[0].hypot(d2[1])) / 3.0;
    let radius = setback * (theta / 2.0).tan();
    let t1 = new_point(sketch, add(at, scale(u1, setback)));
    let t2 = new_point(sketch, add(at, scale(u2, setback)));
    cut_back(sketch, &corner, [t1, t2])?;
    // Turning from coming in along the first line to leaving along the
    // second.
    let turn = cross(scale(u1, -1.0), u2).signum() * (PI - theta);
    let arc = sketch.add_arc(t1, t2, Design::from_f64(turn));
    sketch.constrain(Constraint::Tangent { a: l1, b: arc });
    sketch.constrain(Constraint::Tangent { a: l2, b: arc });
    let prompt = sketch.constrain(Constraint::Radius {
        curve: arc,
        value: Design::from_f64(radius),
    });
    // The corner itself is gone, unless something else holds on to it.
    let used = sketch.curves.values().any(|c| c.points().contains(&p))
        || sketch.constraints.values().any(|k| k.points().contains(&p));
    if !used && !sketch.points[&p].fixed {
        sketch.remove(&[p], &[], &[]);
    }
    Some(Built {
        prompt: Some(prompt),
        ..Built::default()
    })
}
