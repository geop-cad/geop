//! Where the pointer snaps to while drawing and dragging: an existing
//! point — the origin among them — where two curves cross, the middle of a
//! line or arc, or a curve.
//! A snap is a suggestion of a constraint (see [`Snap::constrain`]), never
//! a move of geometry; holding shift turns it off.

use super::*;
use crate::geometry::Plain;

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
            use geop_core_sketch::geometry::Arc;
            let arc = Arc {
                s: sketch.points[&start].xy(),
                e: sketch.points[&end].xy(),
                half: Design::from_f64(sweep.to_f64() / 2.0),
            };
            let m = arc.arc_mid().ok()?;
            Some([m[0].to_f64(), m[1].to_f64()])
        }
        _ => None,
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
        let hidden = visuals::axis_ends(self.args);
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
                for at in pa.crossings(pb, false) {
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
