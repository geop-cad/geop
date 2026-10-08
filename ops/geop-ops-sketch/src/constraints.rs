//! Constraints as the sketch editor offers and shows them: one
//! [`ConstraintTool`] per kind of constraint, which of them a selection can
//! still become ([`ConstraintTool::fit`]), the constraints a complete one
//! adds ([`ConstraintTool::build`]), what each is called ([`name`]) and the
//! glyph that marks it in the sketch ([`glyph`]).
//!
//! A tool takes its operands in any order: what matters is what kind of
//! entity each is — a point, a line, an arc, a circle, a spline — and each
//! tool lists the combinations it accepts ([`Pattern`]). A selection that
//! is one of them is complete; one that can still grow into one is
//! partial; anything else rules the tool out.

use geop_core_math::{
    scalars::{Field, Ring, Scalar},
    vector::Vector2,
};
use geop_core_sketch::{CurveId, PointId, dimension::Measure, geometry::Arc};
use geop_ops::Design;

use crate::{
    Constraint, CurveKind, Sketch,
    geometry::{P2, add, cross, dot, polyline, polyline_mid, scale, sub, unit, xy},
};

/// A point or a curve of the sketch, as selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pick {
    Point(PointId),
    Curve(CurveId),
}

/// What kind of entity a pick is, as a bit of a [`Pattern`]'s slot.
const POINT: u8 = 1;
const LINE: u8 = 2;
const ARC: u8 = 4;
const CIRCLE: u8 = 8;
const SPLINE: u8 = 16;
const ROUND: u8 = ARC | CIRCLE;
const CURVE: u8 = LINE | ARC | CIRCLE | SPLINE;

fn kind_bit(sketch: &Sketch, pick: Pick) -> u8 {
    match pick {
        Pick::Point(_) => POINT,
        Pick::Curve(c) => match sketch.curves[&c].kind {
            CurveKind::Line { .. } => LINE,
            CurveKind::Arc { .. } => ARC,
            CurveKind::Circle { .. } => CIRCLE,
            CurveKind::Spline { .. } => SPLINE,
        },
    }
}

/// A combination of operands a tool accepts.
enum Pattern {
    /// One operand per slot, each of a kind its slot allows.
    Exact(&'static [u8]),
    /// Any number of operands, at least `.1`, each of a kind `.0` allows:
    /// the constraint is added to each.
    Each(u8, usize),
}

/// How a selection stands towards a tool.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fit {
    /// It is all the tool needs: it can be applied.
    pub complete: bool,
    /// It can still grow into what the tool needs.
    pub partial: bool,
    /// Complete, it could still grow into another combination the tool
    /// takes — a line's length, or the distance between it and a point.
    pub extendable: bool,
}

impl Fit {
    /// Whether the tool can still be used with the selection.
    pub fn usable(&self) -> bool {
        self.complete || self.partial
    }
}

/// Whether the kinds `kinds` can each take a slot of `slots` of their own:
/// a matching of the operands to the slots.
fn assignable(kinds: &[u8], slots: &[u8]) -> bool {
    fn go(kinds: &[u8], slots: &[u8], used: &mut Vec<bool>) -> bool {
        let Some((&first, rest)) = kinds.split_first() else {
            return true;
        };
        for (i, &slot) in slots.iter().enumerate() {
            if !used[i] && slot & first != 0 {
                used[i] = true;
                if go(rest, slots, used) {
                    return true;
                }
                used[i] = false;
            }
        }
        false
    }
    kinds.len() <= slots.len() && go(kinds, slots, &mut vec![false; slots.len()])
}

/// A constraint tool: one per kind of constraint the editor offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintTool {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Collinear,
    Equal,
    Concentric,
    Midpoint,
    Symmetric,
    Fix,
    Distance,
    DistanceX,
    DistanceY,
    Radius,
    Diameter,
    Angle,
}

/// What the palette shows of a constraint tool.
pub struct ToolInfo {
    pub tool: ConstraintTool,
    /// Its name in events and the icon it is shown as.
    pub name: &'static str,
    pub label: &'static str,
    /// What it takes and does.
    pub doc: &'static str,
    pub shortcut: Option<&'static str>,
}

impl ConstraintTool {
    pub const ALL: [ToolInfo; 18] = {
        use ConstraintTool::*;
        const fn t(
            tool: ConstraintTool,
            name: &'static str,
            label: &'static str,
            doc: &'static str,
            shortcut: Option<&'static str>,
        ) -> ToolInfo {
            ToolInfo {
                tool,
                name,
                label,
                doc,
                shortcut,
            }
        }
        [
            t(
                Coincident,
                "coincident",
                "Coincident",
                "Two points become one, or a point lies on a curve",
                Some("i"),
            ),
            t(
                Horizontal,
                "horizontal",
                "Horizontal",
                "Lines run horizontally, or two points line up horizontally",
                Some("h"),
            ),
            t(
                Vertical,
                "vertical",
                "Vertical",
                "Lines run vertically, or two points line up vertically",
                Some("v"),
            ),
            t(Parallel, "parallel", "Parallel", "Lines run parallel", None),
            t(
                Perpendicular,
                "perpendicular",
                "Perpendicular",
                "Two lines stand square",
                None,
            ),
            t(
                Tangent,
                "tangent",
                "Tangent",
                "Two curves meet tangentially",
                None,
            ),
            t(
                Collinear,
                "collinear",
                "Collinear",
                "Lines lie on one line",
                None,
            ),
            t(
                Equal,
                "equal",
                "Equal",
                "Lines of equal length, or circles and arcs of equal radius",
                Some("e"),
            ),
            t(
                Concentric,
                "concentric",
                "Concentric",
                "Circles and arcs share a center, or a point is one's center",
                None,
            ),
            t(
                Midpoint,
                "midpoint",
                "Midpoint",
                "A point is the middle of a line or arc",
                None,
            ),
            t(
                Symmetric,
                "symmetric",
                "Symmetric",
                "Two points mirror each other across a line",
                None,
            ),
            t(
                Fix,
                "fix",
                "Fix",
                "Points and curves stay where they are",
                None,
            ),
            t(
                Distance,
                "distance",
                "Distance",
                "Distance between two points, a point and a line, or two lines; a line's or arc's length",
                Some("d"),
            ),
            t(
                DistanceX,
                "distance_x",
                "Horizontal distance",
                "Horizontal distance between two points, or a line's width",
                None,
            ),
            t(
                DistanceY,
                "distance_y",
                "Vertical distance",
                "Vertical distance between two points, or a line's height",
                None,
            ),
            t(
                Radius,
                "radius",
                "Radius",
                "Radius of circles and arcs",
                None,
            ),
            t(
                Diameter,
                "diameter",
                "Diameter",
                "Diameter of circles and arcs",
                None,
            ),
            t(
                Angle,
                "angle",
                "Angle",
                "Angle from one line to another",
                None,
            ),
        ]
    };

    pub fn info(self) -> &'static ToolInfo {
        Self::ALL
            .iter()
            .find(|i| i.tool == self)
            .expect("every tool is listed")
    }

    pub fn by_name(name: &str) -> Option<ConstraintTool> {
        Self::ALL.iter().find(|i| i.name == name).map(|i| i.tool)
    }

    /// Whether it adds a dimension: a value to give.
    pub fn is_dimension(self) -> bool {
        use ConstraintTool::*;
        matches!(
            self,
            Distance | DistanceX | DistanceY | Radius | Diameter | Angle
        )
    }

    fn patterns(self) -> &'static [Pattern] {
        use ConstraintTool::*;
        use Pattern::*;
        match self {
            Coincident => &[Exact(&[POINT, POINT]), Exact(&[POINT, LINE | ROUND])],
            Horizontal | Vertical => &[Each(LINE, 1), Exact(&[POINT, POINT])],
            Parallel | Collinear => &[Each(LINE, 2)],
            Perpendicular | Angle => &[Exact(&[LINE, LINE])],
            Tangent => &[Exact(&[CURVE, CURVE])],
            Equal => &[Each(LINE, 2), Each(ROUND, 2)],
            Concentric => &[Each(ROUND, 2), Exact(&[POINT, ROUND])],
            Midpoint => &[Exact(&[POINT, LINE | ARC])],
            Symmetric => &[Exact(&[POINT, POINT, LINE])],
            Fix => &[Each(POINT | CURVE, 1)],
            Distance => &[
                Exact(&[POINT, POINT]),
                Exact(&[POINT, LINE]),
                Exact(&[LINE, LINE]),
                Exact(&[LINE | ARC]),
            ],
            DistanceX | DistanceY => &[Exact(&[POINT, POINT]), Exact(&[LINE])],
            Radius | Diameter => &[Each(ROUND, 1)],
        }
    }

    /// How `picks` stand towards the tool. No selection at all is where
    /// every tool starts: partial.
    pub fn fit(self, sketch: &Sketch, picks: &[Pick]) -> Fit {
        if picks.is_empty() {
            return Fit {
                partial: true,
                ..Fit::default()
            };
        }
        let kinds: Vec<u8> = picks.iter().map(|&p| kind_bit(sketch, p)).collect();
        let n = kinds.len();
        let mut fit = Fit::default();
        for pattern in self.patterns() {
            match *pattern {
                Pattern::Exact(slots) if assignable(&kinds, slots) => {
                    if n == slots.len() {
                        fit.complete = true;
                    } else {
                        fit.partial = true;
                        fit.extendable = true;
                    }
                }
                Pattern::Each(mask, min) if kinds.iter().all(|k| k & mask != 0) => {
                    if n >= min {
                        fit.complete = true;
                    } else {
                        fit.partial = true;
                    }
                }
                _ => {}
            }
        }
        // A combination of the right kinds may still not be one the
        // constraint can hold: two lines meeting are no tangency.
        if fit.complete && self.build(sketch, picks).is_none() {
            fit.complete = false;
        }
        fit.extendable &= fit.complete;
        fit
    }

    /// The constraints the tool adds for `picks`, measured from the
    /// geometry as it is, so adding them moves nothing — `None` if the
    /// picks are not all it needs, or not something it can constrain.
    pub fn build(self, sketch: &Sketch, picks: &[Pick]) -> Option<Vec<Constraint>> {
        use ConstraintTool as T;
        use geop_core_sketch::Constraint::*;
        let points: Vec<PointId> = picks
            .iter()
            .filter_map(|p| match p {
                Pick::Point(p) => Some(*p),
                _ => None,
            })
            .collect();
        let curves: Vec<CurveId> = picks
            .iter()
            .filter_map(|p| match p {
                Pick::Curve(c) => Some(*c),
                _ => None,
            })
            .collect();
        let kind = |c: CurveId| &sketch.curves[&c].kind;
        let is_line = |c: CurveId| matches!(kind(c), CurveKind::Line { .. });
        let is_round = |c: CurveId| is_round(kind(c));
        let all = |f: &dyn Fn(CurveId) -> bool| !curves.is_empty() && curves.iter().all(|&c| f(c));
        let pair = |f: &dyn Fn(CurveId, CurveId) -> Constraint| -> Vec<Constraint> {
            curves[1..].iter().map(|&b| f(curves[0], b)).collect()
        };
        // Measured values are proposals, free choices: sharp.
        let p = |q: PointId| pt(sketch, q);
        let out = match (self, points.as_slice(), curves.as_slice()) {
            (T::Coincident, &[a, b], []) => vec![Coincident { a, b }],
            (T::Coincident, &[point], &[curve])
                if !matches!(kind(curve), CurveKind::Spline { .. }) =>
            {
                vec![PointOnCurve { point, curve }]
            }
            (T::Horizontal, [], _) if all(&is_line) => {
                curves.iter().map(|&line| Horizontal { line }).collect()
            }
            (T::Vertical, [], _) if all(&is_line) => {
                curves.iter().map(|&line| Vertical { line }).collect()
            }
            (T::Horizontal, &[a, b], []) => vec![DistanceY {
                a,
                b,
                value: Design::ZERO,
            }],
            (T::Vertical, &[a, b], []) => vec![DistanceX {
                a,
                b,
                value: Design::ZERO,
            }],
            (T::Parallel, [], _) if curves.len() >= 2 && all(&is_line) => {
                pair(&|a, b| Parallel { a, b })
            }
            (T::Collinear, [], _) if curves.len() >= 2 && all(&is_line) => {
                pair(&|a, b| Collinear { a, b })
            }
            (T::Perpendicular, [], &[a, b]) if is_line(a) && is_line(b) => {
                vec![Perpendicular { a, b }]
            }
            (T::Tangent, [], &[a, b]) => {
                let (la, lb) = (is_line(a), is_line(b));
                let shared = sketch.shared_endpoint(a, b).ok().flatten().is_some();
                let circle = |c| matches!(kind(c), CurveKind::Circle { .. });
                let ok = shared && !(la && lb) && !circle(a) && !circle(b)
                    || !shared && (is_round(a) && (is_round(b) || lb) || la && is_round(b));
                if !ok {
                    return None;
                }
                vec![Tangent { a, b }]
            }
            (T::Equal, [], _) if curves.len() >= 2 && (all(&is_line) || all(&is_round)) => {
                pair(&|a, b| Equal { a, b })
            }
            (T::Concentric, [], _) if curves.len() >= 2 && all(&is_round) => {
                pair(&|a, b| Concentric { a, b })
            }
            (T::Concentric, &[point], &[curve]) if is_round(curve) => vec![Center { point, curve }],
            (T::Midpoint, &[point], &[curve])
                if matches!(kind(curve), CurveKind::Line { .. } | CurveKind::Arc { .. }) =>
            {
                vec![Midpoint { point, curve }]
            }
            (T::Symmetric, &[a, b], &[line]) if is_line(line) => vec![Symmetric { a, b, line }],
            (T::Fix, _, _) if !picks.is_empty() => {
                let mut fixed: Vec<PointId> = points.clone();
                let mut out = Vec::new();
                for &c in &curves {
                    fixed.extend(sketch.curves[&c].points());
                    if let CurveKind::Circle { radius, .. } = *kind(c) {
                        out.push(Radius {
                            curve: c,
                            value: radius,
                        });
                    }
                }
                fixed.dedup();
                out.extend(fixed.into_iter().map(|point| {
                    let q = p(point);
                    Fix {
                        point,
                        x: q[0],
                        y: q[1],
                    }
                }));
                out
            }
            (T::Distance, &[a, b], []) => vec![Distance {
                a,
                b,
                value: p(b).sub(&p(a)).norm().sharpen(),
            }],
            (T::Distance, &[point], &[line]) if is_line(line) => {
                vec![PointLineDistance {
                    point,
                    line,
                    value: point_line_distance(sketch, point, line)?,
                }]
            }
            (T::Distance, [], &[a, b]) if is_line(a) && is_line(b) => {
                let (start, _) = sketch.curves[&b].endpoints()?;
                vec![PointLineDistance {
                    point: start,
                    line: a,
                    value: point_line_distance(sketch, start, a)?,
                }]
            }
            (T::Distance, [], &[curve]) => vec![Length {
                curve,
                value: length(sketch, curve)?,
            }],
            (T::DistanceX | T::DistanceY, _, _) => {
                let (a, b) = match (points.as_slice(), curves.as_slice()) {
                    (&[a, b], []) => (a, b),
                    ([], &[line]) if is_line(line) => sketch.curves[&line].endpoints()?,
                    _ => return None,
                };
                let k = if self == T::DistanceX { 0 } else { 1 };
                // Measured from the lower to the higher, so it reads as a
                // positive distance.
                let (a, b) = if p(b)[k].to_f64() < p(a)[k].to_f64() {
                    (b, a)
                } else {
                    (a, b)
                };
                let value = p(b)[k].sub(p(a)[k]).sharpen();
                vec![if k == 0 {
                    DistanceX { a, b, value }
                } else {
                    DistanceY { a, b, value }
                }]
            }
            (T::Radius, [], _) if all(&is_round) => curves
                .iter()
                .map(|&curve| {
                    Some(Radius {
                        curve,
                        value: radius(sketch, kind(curve))?,
                    })
                })
                .collect::<Option<_>>()?,
            (T::Diameter, [], _) if all(&is_round) => curves
                .iter()
                .map(|&curve| {
                    Some(Diameter {
                        curve,
                        value: radius(sketch, kind(curve))?.mul(Design::TWO),
                    })
                })
                .collect::<Option<_>>()?,
            (T::Angle, [], &[a, b]) if is_line(a) && is_line(b) => {
                let (a0, a1) = line_points(sketch, kind(a))?;
                let (b0, b1) = line_points(sketch, kind(b))?;
                // The angle as drawn, measured on screen: a proposal the
                // designer accepts or edits, so any nearby value would do.
                let plain = |v: Vector2<Design>| [v[0].to_f64(), v[1].to_f64()];
                let (da, db) = (plain(a1.sub(&a0)), plain(b1.sub(&b0)));
                vec![Angle {
                    a,
                    b,
                    value: Design::from_f64(cross(da, db).atan2(dot(da, db))),
                }]
            }
            _ => return None,
        };
        (!out.is_empty()).then_some(out)
    }
}

fn pt(sketch: &Sketch, p: PointId) -> Vector2<Design> {
    sketch.points[&p].xy()
}

fn is_round(kind: &CurveKind) -> bool {
    matches!(kind, CurveKind::Arc { .. } | CurveKind::Circle { .. })
}

/// `(start, end)` of a line.
fn line_points(sketch: &Sketch, kind: &CurveKind) -> Option<(Vector2<Design>, Vector2<Design>)> {
    match kind {
        CurveKind::Line { start, end } => Some((pt(sketch, *start), pt(sketch, *end))),
        _ => None,
    }
}

/// An arc, as the sketch's own geometry has it.
fn arc(sketch: &Sketch, start: PointId, end: PointId, sweep: Design) -> Option<Arc<Design>> {
    Some(Arc {
        s: pt(sketch, start),
        e: pt(sketch, end),
        half: sweep.div(Design::TWO).ok()?,
    })
}

/// The radius of a circle or arc — a value proposed for a constraint, so a
/// free choice: sharp.
fn radius(sketch: &Sketch, kind: &CurveKind) -> Option<Design> {
    match *kind {
        CurveKind::Circle { radius, .. } => Some(radius),
        CurveKind::Arc { start, end, sweep } => {
            Some(arc(sketch, start, end, sweep)?.radius().ok()?.sharpen())
        }
        _ => None,
    }
}

/// The length of a line or arc, as proposed for a constraint: sharp.
fn length(sketch: &Sketch, curve: CurveId) -> Option<Design> {
    match sketch.curves[&curve].kind {
        CurveKind::Line { start, end } => {
            Some(pt(sketch, end).sub(&pt(sketch, start)).norm().sharpen())
        }
        CurveKind::Arc { start, end, sweep } => {
            Some(arc(sketch, start, end, sweep)?.length().ok()?.sharpen())
        }
        _ => None,
    }
}

/// How far `point` is from `line`'s infinite extension, as proposed for a
/// constraint: sharp.
fn point_line_distance(sketch: &Sketch, point: PointId, line: CurveId) -> Option<Design> {
    let (a, b) = line_points(sketch, &sketch.curves[&line].kind)?;
    geop_core_sketch::geometry::line_distance(a, b, pt(sketch, point))
        .ok()
        .map(|d| d.abs().sharpen())
}

/// The value of a dimensional constraint, as it is given: an angle in
/// degrees.
pub fn value(c: &Constraint) -> Option<f64> {
    use geop_core_sketch::Constraint::*;
    match *c {
        Distance { value, .. }
        | DistanceX { value, .. }
        | DistanceY { value, .. }
        | PointLineDistance { value, .. }
        | Length { value, .. }
        | Radius { value, .. }
        | Diameter { value, .. }
        | Offset { value, .. } => Some(value.to_f64()),
        Angle { value, .. } => Some(value.to_f64().to_degrees()),
        _ => None,
    }
}

/// `c` with its value set to `v` — in the constraint's own units, an angle
/// in radians — if it has one.
pub fn set_value(c: &mut Constraint, v: f64) {
    use geop_core_sketch::Constraint::*;
    if let Distance { value, .. }
    | DistanceX { value, .. }
    | DistanceY { value, .. }
    | PointLineDistance { value, .. }
    | Length { value, .. }
    | Radius { value, .. }
    | Diameter { value, .. }
    | Offset { value, .. }
    | Angle { value, .. } = c
    {
        *value = Design::from_f64(v);
    }
}

/// A dimension's value, as it is shown.
pub fn format_value(v: f64) -> String {
    if v.abs() >= 100.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

/// The glyph that marks `c` in the sketch, and where: `None` for a
/// constraint the drawing shows by itself (coincident points are one).
/// A dimension given by a formula is marked so.
pub fn glyph(sketch: &Sketch, c: &Constraint, formula: bool) -> Option<(String, P2)> {
    use geop_core_sketch::Constraint::*;
    let mid = |curve: CurveId| polyline_mid(&polyline(sketch, curve));
    let p = |point: PointId| xy(sketch, point);
    let between = |a: PointId, b: PointId| scale(add(p(a), p(b)), 0.5);
    let shown = |prefix: &str, v: Design| {
        let f = if formula { "ƒ " } else { "" };
        format!("{f}{prefix}{}", format_value(v.to_f64()))
    };
    Some(match *c {
        Coincident { .. } => return None,
        PointOnCurve { point, .. } => ("◦".into(), p(point)),
        Horizontal { line } => ("H".into(), mid(line)),
        Vertical { line } => ("V".into(), mid(line)),
        Parallel { a, .. } => ("∥".into(), mid(a)),
        Perpendicular { a, .. } => ("⊥".into(), mid(a)),
        Collinear { a, .. } => ("≡".into(), mid(a)),
        Tangent { a, .. } => ("T".into(), mid(a)),
        Equal { a, .. } => ("=".into(), mid(a)),
        Concentric { a, .. } => ("◎".into(), mid(a)),
        Center { point, .. } => ("◎".into(), p(point)),
        Midpoint { point, .. } => ("M".into(), p(point)),
        Symmetric { a, b, .. } => ("⇆".into(), between(a, b)),
        Fix { point, .. } => ("⚓".into(), p(point)),
        Distance { a, b, value } => (shown("", value), between(a, b)),
        DistanceX { a, b, value } => (
            if !formula && value.is_sharp() && value.could_be_equal(Design::ZERO) {
                "|".into()
            } else {
                shown("↔ ", value)
            },
            between(a, b),
        ),
        DistanceY { a, b, value } => (
            if !formula && value.is_sharp() && value.could_be_equal(Design::ZERO) {
                "—".into()
            } else {
                shown("↕ ", value)
            },
            between(a, b),
        ),
        PointLineDistance { point, value, .. } => (shown("", value), p(point)),
        Length { curve, value } => (shown("", value), mid(curve)),
        Radius { curve, value } => (shown("R", value), mid(curve)),
        Diameter { curve, value } => (shown("⌀", value), mid(curve)),
        Angle { b, value, .. } => {
            let f = if formula { "ƒ " } else { "" };
            (format!("{f}{:.1}°", value.to_f64().to_degrees()), mid(b))
        }
        // A pattern's copies show they are copies.
        Moved { .. } => return None,
        EqualSweep { b, .. } => ("⌒=".into(), mid(b)),
        Offset { b, value, .. } => (shown("⇥ ", value), mid(b)),
    })
}

/// The lines a dimension `c` is drawn with when its value is shown at
/// `label`, in the plane: for a distance or a length, extension lines from
/// what it measures out to a dimension line through the label, along what
/// it measures; for a radius or a diameter, a leader from the center across
/// the circle to the label; for an angle, an arc about where the lines
/// meet, through the label. None for a constraint that is no dimension.
pub fn dimension_lines(sketch: &Sketch, c: &Constraint, label: P2) -> Vec<Vec<P2>> {
    use geop_core_sketch::Constraint::*;
    let p = |point: PointId| xy(sketch, point);
    let line_ends = |curve: CurveId| match sketch.curves[&curve].kind {
        CurveKind::Line { start, end } => Some((p(start), p(end))),
        _ => None,
    };
    // A circle's or an arc's center and radius, as drawn.
    let round = |curve: CurveId| -> Option<(P2, f64)> {
        match sketch.curves[&curve].kind {
            CurveKind::Circle { center, radius } => Some((p(center), radius.to_f64())),
            CurveKind::Arc { start, end, sweep } => {
                let a = arc(sketch, start, end, sweep)?;
                let c = a.center().ok()?;
                Some(([c[0].to_f64(), c[1].to_f64()], a.radius().ok()?.to_f64()))
            }
            _ => None,
        }
    };
    let linear = |a: P2, b: P2, along: P2| Measure::Linear { a, b, along }.lines(label);
    match *c {
        Distance { a, b, .. } => linear(p(a), p(b), sub(p(b), p(a))),
        DistanceX { a, b, .. } => linear(p(a), p(b), [1.0, 0.0]),
        DistanceY { a, b, .. } => linear(p(a), p(b), [0.0, 1.0]),
        Length { curve, .. } => match line_ends(curve) {
            Some((a, b)) => linear(a, b, sub(b, a)),
            None => vec![vec![polyline_mid(&polyline(sketch, curve)), label]],
        },
        Offset { a, b, .. } if line_ends(a).is_none() => {
            // Across from one rim to the other, towards the label.
            let (Some((center, ra)), Some((_, rb))) = (round(a), round(b)) else {
                return Vec::new();
            };
            let Some(d) = unit(sub(label, center)) else {
                return Vec::new();
            };
            let (inner, outer) = (ra.min(rb), ra.max(rb));
            let to = if dot(sub(label, center), d) > outer {
                label
            } else {
                add(center, scale(d, outer))
            };
            vec![vec![add(center, scale(d, inner)), to]]
        }
        PointLineDistance { .. } | Offset { .. } => {
            // From a point — the offset line's start — square to the line.
            let (q, line) = match *c {
                PointLineDistance { point, line, .. } => (Some(p(point)), line),
                Offset { a, b, .. } => (line_ends(b).map(|(start, _)| start), a),
                _ => unreachable!("matched above"),
            };
            let (Some(q), Some((a, b))) = (q, line_ends(line)) else {
                return Vec::new();
            };
            let Some(u) = unit(sub(b, a)) else {
                return Vec::new();
            };
            let foot = add(a, scale(u, dot(sub(q, a), u)));
            let mut lines = linear(q, foot, sub(foot, q));
            // The line itself, on to where the dimension meets it.
            lines.push(vec![a, foot]);
            lines
        }
        Radius { curve, .. } | Diameter { curve, .. } => {
            let Some((center, radius)) = round(curve) else {
                return Vec::new();
            };
            Measure::Radial {
                center,
                radius,
                diameter: matches!(c, Diameter { .. }),
            }
            .lines(label)
        }
        Angle { a, b, value } => {
            let (Some((a0, a1)), Some((b0, b1))) = (line_ends(a), line_ends(b)) else {
                return Vec::new();
            };
            let (da, db) = (sub(a1, a0), sub(b1, b0));
            let denom = cross(da, db);
            if denom == 0.0 {
                return Vec::new();
            }
            // Where the lines meet, and an arc about it through the label,
            // turning from the first line's direction by the angle.
            let s = cross(sub(b0, a0), db) / denom;
            Measure::Angular {
                vertex: add(a0, scale(da, s)),
                from: da,
                to: db,
                sweep: value.to_f64(),
            }
            .lines(label)
        }
        _ => Vec::new(),
    }
}

/// What `c` is called in the list of constraints.
pub fn name(c: &Constraint) -> String {
    use geop_core_sketch::Constraint::*;
    match c {
        Coincident { a, b } => format!("Coincident {a}, {b}"),
        PointOnCurve { point, curve } => format!("{point} on {curve}"),
        Horizontal { line } => format!("Horizontal {line}"),
        Vertical { line } => format!("Vertical {line}"),
        Parallel { a, b } => format!("Parallel {a}, {b}"),
        Perpendicular { a, b } => format!("Perpendicular {a}, {b}"),
        Collinear { a, b } => format!("Collinear {a}, {b}"),
        Tangent { a, b } => format!("Tangent {a}, {b}"),
        Equal { a, b } => format!("Equal {a}, {b}"),
        Concentric { a, b } => format!("Concentric {a}, {b}"),
        Center { point, curve } => format!("{point} center of {curve}"),
        Midpoint { point, curve } => format!("{point} midpoint of {curve}"),
        Symmetric { a, b, line } => format!("Symmetric {a}, {b} about {line}"),
        Fix { point, .. } => format!("Fix {point}"),
        Distance { a, b, .. } => format!("Distance {a}–{b}"),
        DistanceX { a, b, .. } => format!("Horizontal {a}–{b}"),
        DistanceY { a, b, .. } => format!("Vertical {a}–{b}"),
        PointLineDistance { point, line, .. } => format!("Distance {point}–{line}"),
        Length { curve, .. } => format!("Length {curve}"),
        Radius { curve, .. } => format!("Radius {curve}"),
        Diameter { curve, .. } => format!("Diameter {curve}"),
        Angle { a, b, .. } => format!("Angle {a}→{b} (°)"),
        Moved { a, b, by } => format!("{b} is {a} moved by {by}"),
        EqualSweep { a, b } => format!("Equal sweep {a}, {b}"),
        Offset { a, b, .. } => format!("Offset {a}–{b}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn n(x: f64) -> Design {
        Design::from_f64(x)
    }

    /// Nothing selected leaves every tool open; a selection rules out the
    /// tools it can never become, keeps those it still can, and completes
    /// those it is all of.
    #[test]
    fn selections_rule_tools_in_and_out() {
        let mut s = Sketch::new();
        let a = s.add_point(n(0.0), n(0.0));
        let b = s.add_point(n(1.0), n(0.1));
        let line = s.add_line(a, b);
        let c = s.add_point(n(3.0), n(3.0));
        let circle = s.add_circle(c, n(1.0));
        let fit = |tool: ConstraintTool, picks: &[Pick]| tool.fit(&s, picks);
        for info in &ConstraintTool::ALL {
            assert!(fit(info.tool, &[]).usable(), "{}", info.name);
        }
        let l = [Pick::Curve(line)];
        assert!(fit(ConstraintTool::Horizontal, &l).complete);
        assert!(fit(ConstraintTool::Parallel, &l).partial);
        assert!(!fit(ConstraintTool::Radius, &l).usable());
        assert!(!fit(ConstraintTool::Concentric, &l).usable());
        // A line's length, or the distance to a point or another line.
        let distance = fit(ConstraintTool::Distance, &l);
        assert!(distance.complete && distance.extendable);
        let o = [Pick::Curve(circle)];
        assert!(fit(ConstraintTool::Diameter, &o).complete);
        assert!(!fit(ConstraintTool::Horizontal, &o).usable());
        let pc = [Pick::Point(a), Pick::Curve(circle)];
        assert!(fit(ConstraintTool::Coincident, &pc).complete);
        assert!(fit(ConstraintTool::Concentric, &pc).complete);
        assert!(!fit(ConstraintTool::Midpoint, &pc).usable());
        // A line and a circle touch; a line and its own end point do not.
        let lc = [Pick::Curve(line), Pick::Curve(circle)];
        assert!(fit(ConstraintTool::Tangent, &lc).complete);
        let pp = [Pick::Point(a), Pick::Point(b)];
        assert!(fit(ConstraintTool::Symmetric, &pp).partial);
        let built = ConstraintTool::DistanceX.build(&s, &pp).unwrap();
        assert_eq!(built.len(), 1);
        assert!((value(&built[0]).unwrap() - 1.0).abs() < 1e-12);
    }
}
