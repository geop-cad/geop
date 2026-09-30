//! What the user does in the drawing: clicks with a tool, selections,
//! drags, dialog fields and keys — each answered with a new sketch.

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
            let id = sketch.add_point(0.0, 0.0);
            sketch.constrain(Constraint::Fix {
                point: id,
                x: 0.0,
                y: 0.0,
            });
            return id;
        }
        let id = sketch.add_point(p[0], p[1]);
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

    /// A click at `p` in the plane, `t` along the pointer's ray.
    fn click(&mut self, pointer: &Pointer<S>, p: P2, t: S, shift: bool) {
        let mut next = self.sketch().clone();
        match (self.s.tool, self.s.draft.clone()) {
            (Tool::Select, _) => self.select_at(pointer, shift),
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
                let second = next.add_point(p[0], first[1]);
                let fourth = next.add_point(first[0], p[1]);
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
                    next.add_arc_with_sweep(start, end, sweep);
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
                    next.add_circle(center, radius);
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

    /// A click with the select tool: a constraint's glyph selects the
    /// constraint; a point or a curve joins the selection, or leaves it; the
    /// origin gets a point fixed there, selected, to constrain others to.
    fn select_at(&mut self, pointer: &Pointer<S>, shift: bool) {
        let visuals = self.visuals();
        let hit = hit_key(
            &visuals,
            pointer,
            &[&is_glyph, &is_point, &is_origin, &is_curve],
        );
        let Some(key) = hit else {
            if !shift {
                self.s.selection = Selection::default();
            }
            self.s.selected_constraint = None;
            return;
        };
        if let Some(constraint) = glyph_key(&key) {
            self.s.selected_constraint = Some(constraint);
            self.s.selection = Selection::default();
            return;
        }
        self.s.selected_constraint = None;
        fn toggle<T: PartialEq>(xs: &mut Vec<T>, x: T) {
            match xs.iter().position(|y| *y == x) {
                Some(i) => {
                    xs.remove(i);
                }
                None => xs.push(x),
            }
        }
        if let Some(point) = point_key(&key) {
            toggle(&mut self.s.selection.points, point);
        } else if let Some(curve) = curve_key(&key) {
            toggle(&mut self.s.selection.curves, curve);
        } else {
            let mut next = self.sketch().clone();
            let id = self.place_point(&mut next, pointer, [0.0, 0.0]);
            self.commit(next);
            if !shift {
                self.s.selection = Selection::default();
            }
            self.s.selection.points.push(id);
        }
    }

    /// A drag with the select tool, grabbed at `from`: what was grabbed
    /// follows the pointer, as far as the constraints let it.
    fn drag(&mut self, from: &Pointer<S>, to: &Pointer<S>, done: bool) {
        let (Some((grab, _)), Some((p, _))) = (self.in_plane(from), self.in_plane(to)) else {
            return;
        };
        if self.s.drag.is_none() {
            let visuals = self.visuals();
            let key = hit_key(&visuals, from, &[&is_point, &is_curve]);
            let sketch = self.sketch();
            let points = |points: Vec<PointId>| Drag::Points {
                origins: points.iter().map(|&q| pt(sketch, q)).collect(),
                points,
                grab,
            };
            self.s.drag = match key {
                Some(key) => match (point_key(&key), curve_key(&key)) {
                    (Some(point), _) => Some(points(vec![point])),
                    (_, Some(curve)) => Some(match &sketch.curves[&curve].kind {
                        CurveKind::Circle { .. } => Drag::Circle { curve },
                        CurveKind::Arc { .. } => Drag::Arc { curve },
                        CurveKind::Line { .. } | CurveKind::Spline { .. } => {
                            points(sketch.curves[&curve].points())
                        }
                    }),
                    _ => None,
                },
                None => None,
            };
        }
        let Some(drag) = self.s.drag.clone() else {
            return;
        };
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
                    *radius = r;
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
                    *sweep = through;
                }
            }
        }
        solve(&mut next, self.s, &drags);
        self.args.sketch = next;
        if done {
            self.s.drag = None;
        }
    }

    fn delete_selection(&mut self) {
        let mut next = self.sketch().clone();
        if let Some(constraint) = self.s.selected_constraint.take() {
            next.remove(&[], &[], &[constraint]);
        } else if !self.s.selection.is_empty() {
            next.remove(&self.s.selection.points, &self.s.selection.curves, &[]);
            self.s.selection = Selection::default();
        } else {
            return;
        }
        self.commit(next);
    }

    pub(super) fn dialog(&mut self, key: &str, value: &Value) {
        if let Some(tool) = key.strip_prefix("tool:").and_then(Tool::by_key) {
            self.finish_draft();
            self.s.tool = tool;
            return;
        }
        if let Some(i) = key
            .strip_prefix("constrain:")
            .and_then(|i| i.parse::<usize>().ok())
        {
            let options = constraints::options(self.sketch(), &self.s.selection);
            if let Some(option) = options.into_iter().nth(i) {
                let mut next = self.sketch().clone();
                next.constrain(option.constraint);
                self.commit(next);
                self.s.selection = Selection::default();
            }
            return;
        }
        if let Some(id) = key
            .strip_prefix("constraint:")
            .and_then(|i| i.parse().ok())
            .map(ConstraintId)
        {
            let mut next = self.sketch().clone();
            match value {
                Value::Press => {
                    self.s.selected_constraint = Some(id);
                    self.s.selection = Selection::default();
                }
                Value::Remove => {
                    next.remove(&[], &[], &[id]);
                    if self.s.selected_constraint == Some(id) {
                        self.s.selected_constraint = None;
                    }
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
            return;
        }
        match key {
            "construction" => {
                let mut next = self.sketch().clone();
                let all = self
                    .s
                    .selection
                    .curves
                    .iter()
                    .all(|c| next.curves.get(c).is_some_and(|c| c.construction));
                for &c in &self.s.selection.curves {
                    next.set_construction(c, !all);
                }
                self.commit(next);
            }
            "delete" => self.delete_selection(),
            _ => {}
        }
    }

    fn key(&mut self, key: &str) {
        match key {
            "Escape" => {
                if self.s.draft.is_some() {
                    self.finish_draft();
                } else {
                    self.s.selection = Selection::default();
                    self.s.selected_constraint = None;
                    self.s.tool = Tool::Select;
                }
            }
            "Enter" => self.finish_draft(),
            "Delete" | "Backspace" => self.delete_selection(),
            other => {
                if let Some(tool) = Tool::by_shortcut(other) {
                    self.finish_draft();
                    self.s.tool = tool;
                }
            }
        }
    }

    /// Where the pointer is over: what a click there would take.
    fn hover(&mut self, pointer: &Pointer<S>) {
        self.s.cursor = self.in_plane(pointer).map(|(p, _)| p);
        let visuals = self.visuals();
        self.s.hover = if self.s.tool == Tool::Select {
            hit_key(
                &visuals,
                pointer,
                &[&is_glyph, &is_point, &is_origin, &is_curve],
            )
        } else {
            hit_key(&visuals, pointer, &[&is_point, &is_origin, &is_curve])
        };
    }

    pub(super) fn event(&mut self, event: &StepEditEvent<S>) {
        match event {
            StepEditEvent::Dialog { .. } => {}
            StepEditEvent::Key { key } => self.key(key),
            StepEditEvent::Hover { pointer } => self.hover(pointer),
            StepEditEvent::Leave => {
                self.s.cursor = None;
                self.s.hover = None;
            }
            StepEditEvent::Click {
                pointer,
                button,
                double,
                shift,
            } => match button {
                Button::Secondary => self.finish_draft(),
                Button::Primary => {
                    if let Some((p, t)) = self.in_plane(pointer) {
                        self.s.cursor = Some(p);
                        self.click(pointer, p, t, *shift);
                    }
                    if *double {
                        self.finish_draft();
                    }
                    self.hover(pointer);
                }
            },
            StepEditEvent::Drag { from, to, done } => {
                if self.s.tool == Tool::Select {
                    self.drag(from, to, *done);
                }
            }
        }
    }
}
