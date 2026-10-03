//! Where the pointer snaps to while drawing and dragging: an existing
//! point — the origin among them — where two curves cross, the middle of a
//! line or arc, or a curve.
//! A snap is a suggestion of a constraint (see [`Snap::constrain`]), never
//! a move of geometry; holding shift turns it off.

use super::*;
use crate::references::{Source, X_AXIS, Y_AXIS};

/// What the pointer snaps to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Snap {
    /// An existing point: what is placed there is that point.
    Point(PointId),
    /// Where two lines, arcs or circles cross — the sketch's axes counted
    /// as the infinite lines they stand for.
    Intersection(CurveId, CurveId),
    /// The middle of a line or arc.
    Midpoint(CurveId),
    /// Somewhere on a curve.
    OnCurve(CurveId),
}

/// The key of the visual a curve's middle is snapped to by.
fn mid_key(curve: CurveId) -> String {
    format!("m{}", curve.0)
}

/// The middle of a line or arc, as drawn.
pub(super) fn curve_mid(sketch: &Sketch, curve: CurveId) -> Option<P2> {
    match sketch.curves.get(&curve)?.kind {
        CurveKind::Line { start, end } => Some(crate::geometry::scale(
            add(pt(sketch, start), pt(sketch, end)),
            0.5,
        )),
        CurveKind::Arc { start, end, sweep } => {
            use geop_core_sketch::geometry::{Arc, V};
            let arc = Arc {
                s: V::of(&sketch.points[&start].xy()),
                e: V::of(&sketch.points[&end].xy()),
                half: Design::from_f64(sweep.to_f64() / 2.0),
            };
            let m = arc.arc_mid().ok()?;
            Some([m.x.to_f64(), m.y.to_f64()])
        }
        _ => None,
    }
}

/// A curve as the pointer meets it, in plain numbers: what intersections
/// are found on. Only a suggestion of where to place a point — the
/// constraints placing it there are what holds it.
enum Plain {
    /// From `a` to `b`, or — `infinite` — the whole line through them.
    Line { a: P2, b: P2, infinite: bool },
    /// About `center`; an arc's part of it from the angle `start`,
    /// counter-clockwise by `sweep` (clockwise if negative).
    Round {
        center: P2,
        radius: f64,
        span: Option<(f64, f64)>,
    },
}

impl Plain {
    /// `curve` of `sketch`, `infinite` if a line; none for a spline.
    fn of(sketch: &Sketch, curve: CurveId, infinite: bool) -> Option<Plain> {
        let p = |q: PointId| pt(sketch, q);
        Some(match sketch.curves.get(&curve)?.kind {
            CurveKind::Line { start, end } => Plain::Line {
                a: p(start),
                b: p(end),
                infinite,
            },
            CurveKind::Circle { center, radius } => Plain::Round {
                center: p(center),
                radius: radius.to_f64(),
                span: None,
            },
            CurveKind::Arc { start, end, sweep } => {
                use geop_core_sketch::geometry::{Arc, V};
                let arc = Arc {
                    s: V::of(&sketch.points[&start].xy()),
                    e: V::of(&sketch.points[&end].xy()),
                    half: Design::from_f64(sweep.to_f64() / 2.0),
                };
                let c = arc.center().ok()?;
                let center = [c.x.to_f64(), c.y.to_f64()];
                let from = sub(p(start), center);
                Plain::Round {
                    center,
                    radius: arc.radius().ok()?.to_f64(),
                    span: Some((from[1].atan2(from[0]), sweep.to_f64())),
                }
            }
            CurveKind::Spline { .. } => return None,
        })
    }

    /// Whether `q`, a point of its full line or circle, lies on it.
    fn holds(&self, q: P2) -> bool {
        match *self {
            Plain::Line { a, b, infinite } => {
                let d = sub(b, a);
                let t = crate::geometry::dot(sub(q, a), d) / crate::geometry::dot(d, d);
                infinite || (0.0..=1.0).contains(&t)
            }
            Plain::Round { span: None, .. } => true,
            Plain::Round {
                center,
                span: Some((start, sweep)),
                ..
            } => {
                let r = sub(q, center);
                let turned = (r[1].atan2(r[0]) - start) * sweep.signum();
                turned.rem_euclid(std::f64::consts::TAU) <= sweep.abs()
            }
        }
    }

    /// Where the full lines and circles of `self` and `other` cross, kept
    /// where both actually run.
    fn crossings(&self, other: &Plain) -> Vec<P2> {
        use crate::geometry::{cross, dot, scale};
        let found = match (self, other) {
            (Plain::Line { a, b, .. }, Plain::Line { a: c, b: d, .. }) => {
                let (u, v) = (sub(*b, *a), sub(*d, *c));
                let denom = cross(u, v);
                if denom == 0.0 {
                    Vec::new()
                } else {
                    vec![add(*a, scale(u, cross(sub(*c, *a), v) / denom))]
                }
            }
            (Plain::Line { a, b, .. }, Plain::Round { center, radius, .. })
            | (Plain::Round { center, radius, .. }, Plain::Line { a, b, .. }) => {
                // `a + t u` at `radius` from `center`.
                let u = sub(*b, *a);
                let f = sub(*a, *center);
                let (qa, qb, qc) = (dot(u, u), 2.0 * dot(f, u), dot(f, f) - radius * radius);
                let disc = qb * qb - 4.0 * qa * qc;
                if disc < 0.0 || qa == 0.0 {
                    Vec::new()
                } else {
                    let root = disc.sqrt();
                    [-1.0, 1.0]
                        .map(|sign| add(*a, scale(u, (-qb + sign * root) / (2.0 * qa))))
                        .to_vec()
                }
            }
            (
                Plain::Round {
                    center: c0,
                    radius: r0,
                    ..
                },
                Plain::Round {
                    center: c1,
                    radius: r1,
                    ..
                },
            ) => {
                let d = sub(*c1, *c0);
                let dist = d[0].hypot(d[1]);
                if dist == 0.0 || dist > r0 + r1 || dist < (r0 - r1).abs() {
                    Vec::new()
                } else {
                    let along = (r0 * r0 - r1 * r1 + dist * dist) / (2.0 * dist);
                    let off = (r0 * r0 - along * along).max(0.0).sqrt();
                    let base = add(*c0, scale(d, along / dist));
                    let n = [-d[1] / dist, d[0] / dist];
                    [-1.0, 1.0]
                        .map(|sign| add(base, scale(n, sign * off)))
                        .to_vec()
                }
            }
        };
        found
            .into_iter()
            .filter(|&q| self.holds(q) && other.holds(q))
            .collect()
    }
}

impl Snap {
    /// `point`, placed where it snapped, constrained to what it snapped to:
    /// onto the curve, to its middle — nothing for a point it is already
    /// (see [`super::drawing`]).
    pub fn constrain(self, sketch: &mut Sketch, point: PointId) {
        match self {
            Snap::Point(other) if other != point => {
                let class = sketch.point_classes();
                if class[&other] != class[&point] {
                    sketch.constrain(Constraint::Coincident { a: other, b: point });
                }
            }
            Snap::Point(_) => {}
            Snap::Intersection(a, b) => {
                for curve in [a, b] {
                    sketch.constrain(Constraint::PointOnCurve { point, curve });
                }
            }
            Snap::Midpoint(curve) => {
                sketch.constrain(Constraint::Midpoint { point, curve });
            }
            Snap::OnCurve(curve) => {
                if !matches!(sketch.curves[&curve].kind, CurveKind::Spline { .. }) {
                    sketch.constrain(Constraint::PointOnCurve { point, curve });
                }
            }
        }
    }
}

impl<S: Scalar> Editing<'_, S> {
    /// The points never snapped to: the far ends of the sketch's axes,
    /// which are only there to give the axes a direction.
    fn hidden_points(&self) -> Vec<PointId> {
        self.args
            .references
            .iter()
            .filter(|r| r.source == Source::Frame)
            .flat_map(|r| [X_AXIS, Y_AXIS].map(|k| r.points.get(k).copied()))
            .flatten()
            .collect()
    }

    /// Where `pointer` — at `p` in the plane — snaps to, and what to: a
    /// point, where two curves cross, a line's or arc's middle, or,
    /// `onto_curves`, any curve. The
    /// points `exclude` are left out, and every curve on one of them, so
    /// nothing snaps onto itself. Nothing with `shift` held.
    pub(super) fn snap(
        &self,
        pointer: &Pointer<S>,
        p: P2,
        shift: bool,
        exclude: &[PointId],
        onto_curves: bool,
    ) -> (P2, Option<Snap>) {
        if shift {
            return (p, None);
        }
        let sketch = self.sketch();
        let world = |q: P2| to_world(&self.frame, q);
        let hidden = self.hidden_points();
        let class = sketch.point_classes();
        let excluded = |q: &PointId| exclude.iter().any(|e| class[e] == class[q]);
        let mut candidates = Vec::new();
        for &id in sketch.points.keys() {
            if !excluded(&id) && !hidden.contains(&id) {
                candidates.push(Visual::new(
                    id.to_string(),
                    Shape::Point {
                        at: world(pt(sketch, id)),
                    },
                    Style::Free,
                ));
            }
        }
        for (&id, curve) in &sketch.curves {
            if curve.points().iter().any(excluded) {
                continue;
            }
            let axis = curve.points().iter().any(|q| hidden.contains(q));
            if let Some(mid) = curve_mid(sketch, id).filter(|_| !axis) {
                candidates.push(Visual::new(
                    mid_key(id),
                    Shape::Point { at: world(mid) },
                    Style::Free,
                ));
            }
            if onto_curves {
                candidates.push(Visual::new(
                    id.to_string(),
                    Shape::Polyline {
                        points: visuals::drawn(self.args, id)
                            .into_iter()
                            .map(world)
                            .collect(),
                    },
                    Style::Free,
                ));
            }
        }
        // Where curves cross. Where they already meet at a point, that point
        // is what is snapped to: points are tried first.
        let crossable: Vec<(CurveId, Plain)> = sketch
            .curves
            .iter()
            .filter(|(_, c)| !c.points().iter().any(excluded))
            .filter_map(|(&id, c)| {
                let axis = c.points().iter().any(|q| hidden.contains(q));
                Some((id, Plain::of(sketch, id, axis)?))
            })
            .collect();
        let mut crossings = Vec::new();
        for (i, (a, pa)) in crossable.iter().enumerate() {
            for (b, pb) in &crossable[i + 1..] {
                for at in pa.crossings(pb) {
                    candidates.push(Visual::new(
                        format!("x{}", crossings.len()),
                        Shape::Point { at: world(at) },
                        Style::Free,
                    ));
                    crossings.push((Snap::Intersection(*a, *b), at));
                }
            }
        }
        let is_mid = |key: &str| key_id(key, 'm').is_some();
        let is_crossing = |key: &str| key_id(key, 'x').is_some();
        let hit = hit_key(
            &candidates,
            pointer,
            &[&is_point, &is_crossing, &is_mid, &is_curve],
        );
        match hit.as_deref() {
            Some(key) if is_point(key) => {
                let id = point_key(key).expect("a point's key");
                (pt(sketch, id), Some(Snap::Point(id)))
            }
            Some(key) if is_crossing(key) => {
                let (snap, at) = crossings[key_id(key, 'x').expect("a crossing's key") as usize];
                (at, Some(snap))
            }
            Some(key) if is_mid(key) => {
                let id = CurveId(key_id(key, 'm').expect("a middle's key"));
                (curve_mid(sketch, id).unwrap_or(p), Some(Snap::Midpoint(id)))
            }
            Some(key) => (p, curve_key(key).map(Snap::OnCurve)),
            None => (p, None),
        }
    }
}
