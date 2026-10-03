//! What the user does in the drawing: clicks with a tool, drags of what the
//! editor lets be dragged, dialog fields and keys — each answered with a new
//! sketch.

use super::*;

impl<S: Scalar> Editing<'_, S> {
    /// The point to use where `pointer` is, at `p` in the plane: the point
    /// drawn there, or a new one — fixed if placed on the origin,
    /// constrained onto a curve it is placed on.
    fn place_point(&self, sketch: &mut Sketch, pointer: &Pointer<S>, p: P2) -> PointId {
        let visuals = visuals(sketch, self.s, &self.frame);
        let hit = hit_key(&visuals, pointer, &[&is_point, &is_origin, &is_curve]);
        if let Some(existing) = hit.as_deref().and_then(point_key) {
            return existing;
        }
        if hit.as_deref() == Some("origin") {
            let id = sketch.add_point(design(0.0), design(0.0));
            sketch.constrain(Constraint::Fix {
                point: id,
                x: Design::ZERO,
                y: Design::ZERO,
            });
            return id;
        }
        let id = sketch.add_point(design(p[0]), design(p[1]));
        if let Some(curve) = hit.as_deref().and_then(curve_key)
            && !matches!(sketch.curves[&curve].kind, CurveKind::Spline { .. })
        {
            sketch.constrain(Constraint::PointOnCurve { point: id, curve });
        }
        id
    }

    /// Ends the curve being drawn: a spline with at least two points is
    /// added, anything else dropped.
    fn finish_draft(&mut self) {
        if let Some(Draft::Spline { points }) = self.s.draft.take()
            && points.len() >= 2
        {
            let mut next = self.sketch().clone();
            next.add_spline(points);
            self.commit(next);
        }
    }

    /// A click at `p` in the plane, `t` along the pointer's ray — one that
    /// selected nothing.
    fn click(&mut self, pointer: &Pointer<S>, p: P2, t: S) {
        let mut next = self.sketch().clone();
        match (self.s.tool, self.s.draft.clone()) {
            (Tool::Select, _) => self.origin_at(pointer),
            (Tool::Point, _) => {
                self.place_point(&mut next, pointer, p);
                self.commit(next);
            }
            (Tool::Line, Some(Draft::Line { start })) => {
                let end = self.place_point(&mut next, pointer, p);
                if end == start {
                    return;
                }
                let (a, b) = (pt(&next, start), pt(&next, end));
                let line = next.add_line(start, end);
                let d = sub(b, a);
                if d[1].abs() <= AUTO_HV_SLOPE * d[0].abs() {
                    next.constrain(Constraint::Horizontal { line });
                } else if d[0].abs() <= AUTO_HV_SLOPE * d[1].abs() {
                    next.constrain(Constraint::Vertical { line });
                }
                self.commit(next);
                // Lines chain: the next one starts where this one ended.
                self.s.draft = Some(Draft::Line { start: end });
            }
            (Tool::Line, _) => {
                let start = self.place_point(&mut next, pointer, p);
                self.commit(next);
                self.s.draft = Some(Draft::Line { start });
            }
            (Tool::Rectangle, Some(Draft::Rectangle { corner })) => {
                let first = pt(&next, corner);
                let tolerance = pointer.reach_at(1.0, t).to_f64();
                if (p[0] - first[0]).abs() <= tolerance || (p[1] - first[1]).abs() <= tolerance {
                    return;
                }
                // Two opposite corners, and the two they imply. The sides
                // are horizontal and vertical by constraint, so it stays a
                // rectangle whatever is dragged later.
                let opposite = self.place_point(&mut next, pointer, p);
                let second = next.add_point(design(p[0]), design(first[1]));
                let fourth = next.add_point(design(first[0]), design(p[1]));
                let corners = [corner, second, opposite, fourth];
                for i in 0..4 {
                    let line = next.add_line(corners[i], corners[(i + 1) % 4]);
                    next.constrain(if i % 2 == 0 {
                        Constraint::Horizontal { line }
                    } else {
                        Constraint::Vertical { line }
                    });
                }
                self.commit(next);
                self.s.draft = None;
            }
            (Tool::Rectangle, _) => {
                let corner = self.place_point(&mut next, pointer, p);
                self.commit(next);
                self.s.draft = Some(Draft::Rectangle { corner });
            }
            (Tool::Arc, Some(Draft::Arc { start, end: None })) => {
                let end = self.place_point(&mut next, pointer, p);
                if end == start {
                    return;
                }
                self.commit(next);
                self.s.draft = Some(Draft::Arc {
                    start,
                    end: Some(end),
                });
            }
            (
                Tool::Arc,
                Some(Draft::Arc {
                    start,
                    end: Some(end),
                }),
            ) => {
                let sweep = sweep_through(pt(&next, start), pt(&next, end), p);
                if sweep.is_finite() {
                    next.add_arc(start, end, design(sweep));
                    self.commit(next);
                }
                self.s.draft = None;
            }
            (Tool::Arc, _) => {
                let start = self.place_point(&mut next, pointer, p);
                self.commit(next);
                self.s.draft = Some(Draft::Arc { start, end: None });
            }
            (Tool::Circle, Some(Draft::Circle { center })) => {
                let radius = dist(pt(&next, center), p);
                if radius > 0.0 {
                    next.add_circle(center, design(radius));
                    self.commit(next);
                }
                self.s.draft = None;
            }
            (Tool::Circle, _) => {
                let center = self.place_point(&mut next, pointer, p);
                self.commit(next);
                self.s.draft = Some(Draft::Circle { center });
            }
            (Tool::Spline, draft) => {
                let mut points = match draft {
                    Some(Draft::Spline { points }) => points,
                    _ => Vec::new(),
                };
                let at = self.place_point(&mut next, pointer, p);
                if points.last() != Some(&at) {
                    points.push(at);
                }
                self.commit(next);
                self.s.draft = Some(Draft::Spline { points });
            }
        }
    }

    /// A click with the select tool on the origin: a point fixed there,
    /// selected, to constrain others to.
    fn origin_at(&mut self, pointer: &Pointer<S>) {
        let visuals = visuals(self.sketch(), self.s, &self.frame);
        if hit_key(&visuals, pointer, &[&is_origin]).is_none() {
            return;
        }
        let mut next = self.sketch().clone();
        let id = self.place_point(&mut next, pointer, [0.0, 0.0]);
        self.commit(next);
        self.selection.push(id.to_string());
    }

    /// The visual `key` dragged from `from` to `to` in the plane: what it
    /// stands for follows the pointer, as far as the constraints let it — a
    /// point, a line's or a spline's points, a circle's radius, an arc's
    /// sweep.
    fn drag(&mut self, key: &str, from: P2, to: P2, done: bool) {
        if self.s.drag.is_none() {
            let sketch = self.sketch();
            let points = |points: Vec<PointId>| Drag::Points {
                origins: points.iter().map(|&q| pt(sketch, q)).collect(),
                points,
                grab: from,
            };
            self.s.drag = match (point_key(key), curve_key(key)) {
                (Some(point), _) if sketch.points.contains_key(&point) => Some(points(vec![point])),
                (_, Some(curve)) => sketch.curves.get(&curve).map(|c| match &c.kind {
                    CurveKind::Circle { .. } => Drag::Circle { curve },
                    CurveKind::Arc { .. } => Drag::Arc { curve },
                    CurveKind::Line { .. } | CurveKind::Spline { .. } => points(c.points()),
                }),
                _ => None,
            };
        }
        let Some(drag) = self.s.drag.clone() else {
            return;
        };
        if done {
            self.s.drag = None;
        }
        let p = to;
        let mut next = self.sketch().clone();
        let mut drags = Vec::new();
        match drag {
            Drag::Points {
                points,
                origins,
                grab,
            } => {
                let delta = sub(p, grab);
                drags = points
                    .iter()
                    .zip(&origins)
                    .map(|(&q, &o)| (q, add(o, delta)))
                    .collect();
            }
            Drag::Circle { curve } => {
                let center = match next.curves[&curve].kind {
                    CurveKind::Circle { center, .. } => center,
                    _ => return,
                };
                let r = dist(pt(&next, center), p);
                if let Some(c) = next.curves.get_mut(&curve)
                    && let CurveKind::Circle { radius, .. } = &mut c.kind
                {
                    *radius = design(r);
                }
            }
            Drag::Arc { curve } => {
                let CurveKind::Arc { start, end, .. } = next.curves[&curve].kind else {
                    return;
                };
                let through = sweep_through(pt(&next, start), pt(&next, end), p);
                if !through.is_finite() {
                    return;
                }
                if let Some(c) = next.curves.get_mut(&curve)
                    && let CurveKind::Arc { sweep, .. } = &mut c.kind
                {
                    *sweep = design(through);
                }
            }
        }
        solve(&mut next, self.s, &drags);
        self.args.sketch = next;
    }

    /// Removes what is selected: points, curves and constraints.
    pub(super) fn delete_selection(&mut self) {
        let (selection, constraints) = selected(self.sketch(), self.selection);
        if selection.is_empty() && constraints.is_empty() {
            return;
        }
        let mut next = self.sketch().clone();
        next.remove(&selection.points, &selection.curves, &constraints);
        self.selection.clear();
        self.commit(next);
    }

    /// Puts down what is being drawn, and takes up `tool`.
    pub(super) fn take(&mut self, tool: Tool) {
        self.finish_draft();
        self.s.tool = tool;
    }

    /// Adds the constraint labelled `label` among those that fit the
    /// selection, and clears the selection.
    pub(super) fn constrain(&mut self, label: &str) {
        let (selection, _) = selected(self.sketch(), self.selection);
        let options = constraints::options(self.sketch(), &selection);
        if let Some(option) = options.into_iter().find(|o| o.label == label) {
            let mut next = self.sketch().clone();
            next.constrain(option.constraint);
            self.commit(next);
            self.selection.clear();
        }
    }

    /// Makes the selected curves construction geometry — or, if they all
    /// are, profile geometry again.
    pub(super) fn toggle_construction(&mut self) {
        let (selection, _) = selected(self.sketch(), self.selection);
        let mut next = self.sketch().clone();
        let all = selection
            .curves
            .iter()
            .all(|c| next.curves.get(c).is_some_and(|c| c.construction));
        for &c in &selection.curves {
            next.set_construction(c, !all);
        }
        self.commit(next);
    }

    /// The constraint `id`'s entry in the list used: pressed, it is
    /// selected; removed; given a new value.
    pub(super) fn constraint(&mut self, id: ConstraintId, value: &Value) {
        let mut next = self.sketch().clone();
        match value {
            Value::Press => *self.selection = vec![id.to_string()],
            Value::Remove => {
                next.remove(&[], &[], &[id]);
                self.selection.retain(|k| glyph_key(k) != Some(id));
                self.commit(next);
            }
            Value::Number(v) => {
                if let Some(c) = next.constraints.get_mut(&id) {
                    let v = if matches!(c, Constraint::Angle { .. }) {
                        v.to_radians()
                    } else {
                        *v
                    };
                    constraints::set_value(c, v);
                    self.commit(next);
                }
            }
            _ => {}
        }
    }

    fn key(&mut self, key: &str) {
        match key {
            "Escape" => {
                if self.s.draft.is_some() {
                    self.finish_draft();
                } else {
                    self.s.tool = Tool::Select;
                }
            }
            "Enter" => self.finish_draft(),
            "Delete" | "Backspace" => self.delete_selection(),
            other => {
                if let Some(tool) = Tool::by_shortcut(other) {
                    self.take(tool);
                }
            }
        }
    }

    pub(super) fn event(&mut self, event: &CanvasEvent<S>) {
        match event {
            CanvasEvent::Key { key } => self.key(key),
            CanvasEvent::Hover { pointer } => {
                self.s.cursor = self.in_plane(pointer).map(|(p, _)| p)
            }
            CanvasEvent::Leave => self.s.cursor = None,
            CanvasEvent::Click {
                pointer,
                button,
                double,
                ..
            } => match button {
                Button::Secondary => self.finish_draft(),
                Button::Primary => {
                    if let Some((p, t)) = self.in_plane(pointer) {
                        self.s.cursor = Some(p);
                        if !*double {
                            self.click(pointer, p, t);
                        }
                    }
                    if *double {
                        self.finish_draft();
                    }
                }
            },
            CanvasEvent::Move {
                key,
                from,
                to,
                done,
            } => {
                let (from, to) = (self.to_sketch(from), self.to_sketch(to));
                self.drag(key, from, to, *done);
            }
        }
    }
}
