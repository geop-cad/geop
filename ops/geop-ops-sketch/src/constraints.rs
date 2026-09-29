//! Constraints as the sketch editor offers and shows them: which fit a
//! selection ([`options`]), what each is called ([`name`]) and the glyph
//! that marks it in the sketch ([`glyph`]).

use geop_core_sketch::{Constraint, CurveId, CurveKind, PointId, Sketch, profile::curve_polyline};
use serde::{Deserialize, Serialize};

use crate::geometry::{P2, add, cross, dist, dot, polyline_mid, scale, sub};

/// The points and curves selected in the sketch, in the order they were
/// picked.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Selection {
    pub points: Vec<PointId>,
    pub curves: Vec<CurveId>,
}

impl Selection {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.curves.is_empty()
    }
}

/// A constraint that fits the selection, measured from the geometry as it
/// is, so adding it moves nothing.
pub struct ConstraintOption {
    pub label: &'static str,
    pub title: &'static str,
    pub constraint: Constraint,
}

fn pt(sketch: &Sketch, p: PointId) -> P2 {
    sketch.points[&p].xy()
}

fn is_round(kind: &CurveKind) -> bool {
    matches!(kind, CurveKind::Arc { .. } | CurveKind::Circle { .. })
}

/// `(start, end)` of a line.
fn line_points(sketch: &Sketch, kind: &CurveKind) -> Option<(P2, P2)> {
    match kind {
        CurveKind::Line { start, end } => Some((pt(sketch, *start), pt(sketch, *end))),
        _ => None,
    }
}

/// The radius of a circle or arc.
fn radius(sketch: &Sketch, kind: &CurveKind) -> f64 {
    match kind {
        CurveKind::Circle { radius, .. } => *radius,
        CurveKind::Arc { start, end, sweep } => {
            dist(pt(sketch, *start), pt(sketch, *end)) / (2.0 * (sweep / 2.0).sin().abs())
        }
        _ => 0.0,
    }
}

/// The length along an arc.
fn arc_length(sketch: &Sketch, start: PointId, end: PointId, sweep: f64) -> f64 {
    let chord = dist(pt(sketch, start), pt(sketch, end));
    let half = sweep / 2.0;
    if half == 0.0 {
        chord
    } else {
        chord * half / half.sin()
    }
}

/// Every constraint that fits `sel` in `sketch`.
pub fn options(sketch: &Sketch, sel: &Selection) -> Vec<ConstraintOption> {
    use Constraint::*;
    let mut out = Vec::new();
    let mut add = |label, title, constraint| {
        out.push(ConstraintOption {
            label,
            title,
            constraint,
        })
    };
    let (points, curves) = (sel.points.as_slice(), sel.curves.as_slice());
    let kind = |c: CurveId| &sketch.curves[&c].kind;

    match (points, curves) {
        (&[point], &[]) => {
            let [x, y] = pt(sketch, point);
            add("Fix", "Fix the point where it is", Fix { point, x, y });
        }
        (&[a, b], &[]) => {
            let (pa, pb) = (pt(sketch, a), pt(sketch, b));
            add("Coincident", "Make the two points one", Coincident { a, b });
            add("Horizontal", "Same y", DistanceY { a, b, value: 0.0 });
            add("Vertical", "Same x", DistanceX { a, b, value: 0.0 });
            add(
                "Distance",
                "Distance between the points",
                Distance {
                    a,
                    b,
                    value: dist(pa, pb),
                },
            );
            add(
                "Δx",
                "Horizontal distance",
                DistanceX {
                    a,
                    b,
                    value: pb[0] - pa[0],
                },
            );
            add(
                "Δy",
                "Vertical distance",
                DistanceY {
                    a,
                    b,
                    value: pb[1] - pa[1],
                },
            );
        }
        (&[], &[curve]) => {
            let k = kind(curve);
            if let Some((a, b)) = line_points(sketch, k) {
                add("Horizontal", "Horizontal line", Horizontal { line: curve });
                add("Vertical", "Vertical line", Vertical { line: curve });
                add(
                    "Length",
                    "Line length",
                    Length {
                        curve,
                        value: dist(a, b),
                    },
                );
            }
            if is_round(k) {
                add(
                    "Radius",
                    "Radius",
                    Radius {
                        curve,
                        value: radius(sketch, k),
                    },
                );
            }
            if let CurveKind::Arc { start, end, sweep } = *k {
                add(
                    "Arc length",
                    "Length along the arc",
                    Length {
                        curve,
                        value: arc_length(sketch, start, end, sweep),
                    },
                );
            }
        }
        (&[], &[a, b]) => {
            let (ka, kb) = (kind(a), kind(b));
            let lines = (line_points(sketch, ka), line_points(sketch, kb));
            if let (Some((a0, a1)), Some((b0, b1))) = lines {
                add("Parallel", "Parallel lines", Parallel { a, b });
                add(
                    "Perpendicular",
                    "Perpendicular lines",
                    Perpendicular { a, b },
                );
                add("Collinear", "On one line", Collinear { a, b });
                add("Equal", "Equal length", Equal { a, b });
                let (da, db) = (sub(a1, a0), sub(b1, b0));
                add(
                    "Angle",
                    "Angle from the first line to the second",
                    Angle {
                        a,
                        b,
                        value: cross(da, db).atan2(dot(da, db)),
                    },
                );
            }
            let both_lines = lines.0.is_some() && lines.1.is_some();
            let line_round = lines.0.is_some() && is_round(kb) || is_round(ka) && lines.1.is_some();
            let shared = !both_lines && sketch.shared_endpoint(a, b).ok().flatten().is_some();
            if shared || line_round || is_round(ka) && is_round(kb) {
                add("Tangent", "Tangent curves", Tangent { a, b });
            }
            if is_round(ka) && is_round(kb) {
                add("Concentric", "Same center", Concentric { a, b });
                add("Equal", "Equal radius", Equal { a, b });
            }
        }
        (&[point], &[curve]) => {
            let k = kind(curve);
            if !matches!(k, CurveKind::Spline { .. }) {
                add(
                    "On curve",
                    "Point lies on the curve",
                    PointOnCurve { point, curve },
                );
            }
            if matches!(k, CurveKind::Line { .. } | CurveKind::Arc { .. }) {
                add(
                    "Midpoint",
                    "Point is the curve's midpoint",
                    Midpoint { point, curve },
                );
            }
            if let Some((a, b)) = line_points(sketch, k) {
                let d = sub(b, a);
                let value = (cross(d, sub(pt(sketch, point), a)) / d[0].hypot(d[1])).abs();
                add(
                    "Distance",
                    "Distance from the line",
                    PointLineDistance {
                        point,
                        line: curve,
                        value,
                    },
                );
            }
        }
        (&[a, b], &[line]) if matches!(kind(line), CurveKind::Line { .. }) => {
            add(
                "Symmetric",
                "Mirror images across the line",
                Symmetric { a, b, line },
            );
        }
        _ => {}
    }
    out
}

/// The value of a dimensional constraint.
pub fn value(c: &Constraint) -> Option<f64> {
    use Constraint::*;
    match *c {
        Distance { value, .. }
        | DistanceX { value, .. }
        | DistanceY { value, .. }
        | PointLineDistance { value, .. }
        | Length { value, .. }
        | Radius { value, .. }
        | Angle { value, .. } => Some(value),
        _ => None,
    }
}

/// `c` with its value set to `v`, if it has one.
pub fn set_value(c: &mut Constraint, v: f64) {
    use Constraint::*;
    if let Distance { value, .. }
    | DistanceX { value, .. }
    | DistanceY { value, .. }
    | PointLineDistance { value, .. }
    | Length { value, .. }
    | Radius { value, .. }
    | Angle { value, .. } = c
    {
        *value = v;
    }
}

/// The glyph that marks `c` in the sketch, and where: `None` for a
/// constraint the drawing shows by itself (coincident points are one).
pub fn glyph(sketch: &Sketch, c: &Constraint) -> Option<(String, P2)> {
    use Constraint::*;
    let positions = sketch.positions();
    let mid = |curve: CurveId| polyline_mid(&curve_polyline(sketch, &positions, curve));
    let p = |point: PointId| pt(sketch, point);
    let between = |a: PointId, b: PointId| scale(add(p(a), p(b)), 0.5);
    let fmt = |v: f64| {
        if v.abs() >= 100.0 {
            format!("{v:.1}")
        } else {
            format!("{v:.2}")
        }
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
        Midpoint { point, .. } => ("M".into(), p(point)),
        Symmetric { a, b, .. } => ("⇆".into(), between(a, b)),
        Fix { point, .. } => ("⚓".into(), p(point)),
        Distance { a, b, value } => (fmt(value), between(a, b)),
        DistanceX { a, b, value } => (
            if value == 0.0 {
                "|".into()
            } else {
                format!("Δx {}", fmt(value))
            },
            between(a, b),
        ),
        DistanceY { a, b, value } => (
            if value == 0.0 {
                "—".into()
            } else {
                format!("Δy {}", fmt(value))
            },
            between(a, b),
        ),
        PointLineDistance { point, value, .. } => (fmt(value), p(point)),
        Length { curve, value } => (fmt(value), mid(curve)),
        Radius { curve, value } => (format!("R{}", fmt(value)), mid(curve)),
        Angle { b, value, .. } => (format!("{:.1}°", value.to_degrees()), mid(b)),
    })
}

/// What `c` is called in the list of constraints.
pub fn name(c: &Constraint) -> String {
    use Constraint::*;
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
        Midpoint { point, curve } => format!("{point} midpoint of {curve}"),
        Symmetric { a, b, line } => format!("Symmetric {a}, {b} about {line}"),
        Fix { point, .. } => format!("Fix {point}"),
        Distance { a, b, .. } => format!("Distance {a}–{b}"),
        DistanceX { a, b, .. } => format!("Δx {a}–{b}"),
        DistanceY { a, b, .. } => format!("Δy {a}–{b}"),
        PointLineDistance { point, line, .. } => format!("Distance {point}–{line}"),
        Length { curve, .. } => format!("Length {curve}"),
        Radius { curve, .. } => format!("Radius {curve}"),
        Angle { a, b, .. } => format!("Angle {a}→{b} (°)"),
    }
}
