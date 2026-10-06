//! What the user does in the drawing: clicks with a tool, drags of what the
//! editor lets be dragged, dialog fields and keys — each answered with a new
//! sketch.

use geop_ops::{
    EntityRef,
    parameters::{evaluate, is_formula, number},
};

use super::drawing::{Built, Hints, construct, curve_ending_at};
use super::trim::Plan;
use super::*;
use crate::geometry::{angle_between, segments_cross, wrap};
use crate::references::{Reference, Source};

impl<S: Scalar> Editing<'_, S> {
    /// What a construction needs beyond its points, the pointer reaching
    /// `pointer`, `t` along its ray.
    fn hints(&self, pointer: &Pointer<S>, t: S) -> Hints {
        Hints {
            sides: self.s.sides,
            circumscribed: self.s.circumscribed,
            tangent_to: self.s.draft.previous,
            sweep: self.s.draft.sweep,
            min_size: pointer.reach_at(1.0, t).to_f64(),
        }
    }

    /// The point placed where `pointer` is, at `p` in the plane: snapped,
    /// unless `shift` is held.
    fn placed(&self, pointer: &Pointer<S>, p: P2, shift: bool) -> Placed {
        let (at, snap) = self.snap(pointer, p, shift, &[], true);
        Placed { at, snap }
    }

    /// The pointer moved, `shift` held or not: while drawing, where it
    /// snaps to — and, for a chain of lines, whether it came back onto the
    /// chain's end, which switches between a line and a tangent arc.
    fn hover(&mut self, pointer: &Pointer<S>, shift: bool) {
        let Some((p, _)) = self.in_plane(pointer) else {
            self.s.cursor = None;
            self.s.snap = None;
            return;
        };
        let Tool::Draw(tool) = self.s.tool else {
            self.s.cursor = Some(p);
            self.s.snap = None;
            if self.s.tool == Tool::Trim {
                self.s.stroke = Stroke {
                    path: Vec::new(),
                    met: self.meets(pointer).into_iter().collect(),
                };
            }
            return;
        };
        let placed = self.placed(pointer, p, shift);
        self.s.cursor = Some(placed.at);
        self.s.snap = placed.snap;
        let draft = &self.s.draft;
        match (tool, draft.placed.as_slice()) {
            (
                DrawTool::Line,
                &[
                    Placed {
                        snap: Some(Snap::Point(end)),
                        ..
                    },
                ],
            ) if draft.previous.is_some() => {
                let at = to_world(&self.frame, pt(self.sketch(), end));
                let near = hit_visuals(
                    &[Visual::new("end", Shape::Point { at }, Style::Free)],
                    pointer,
                    None,
                    |_| true,
                )
                .is_some();
                let draft = &mut self.s.draft;
                if near && !draft.at_end {
                    draft.arc = !draft.arc;
                }
                draft.at_end = near;
            }
            (DrawTool::CenterArc | DrawTool::ArcSlot, [m, s]) => {
                let raw = angle_between(sub(s.at, m.at), sub(placed.at, m.at));
                let previous = draft.sweep;
                self.s.draft.sweep = if previous == 0.0 {
                    raw
                } else {
                    previous + wrap(raw - previous)
                };
            }
            _ => {}
        }
    }

    /// A primary click, not a double one, at `p` in the plane, `t` along
    /// the pointer's ray, with a drawing tool in hand.
    fn place(&mut self, tool: DrawTool, pointer: &Pointer<S>, p: P2, t: S, shift: bool) {
        let placed = self.placed(pointer, p, shift);
        self.s.cursor = Some(placed.at);
        // A tangent arc starts at the end of a curve.
        if tool == DrawTool::TangentArc
            && self.s.draft.placed.is_empty()
            && !matches!(placed.snap, Some(Snap::Point(q)) if curve_ending_at(self.sketch(), q).is_some())
        {
            return;
        }
        self.s.draft.placed.push(placed);
        if tool.needs().is_some_and(|n| self.s.draft.placed.len() >= n) {
            let hints = self.hints(pointer, t);
            if self.build(tool, &hints).is_none() {
                // Nothing to build there: the click is not taken.
                self.s.draft.placed.pop();
            }
        }
    }

    /// Builds what `tool` draws from the points placed, and adds it to the
    /// sketch: as construction geometry, if that is what is drawn. A chain
    /// of lines goes on from where it ended; anything else starts afresh.
    fn build(&mut self, tool: DrawTool, hints: &Hints) -> Option<()> {
        let mut next = self.sketch().clone();
        let first_new = next.next_id;
        let Built { end, last, prompt } = construct(
            tool,
            self.s.draft.arc,
            &mut next,
            &self.s.draft.placed,
            hints,
        )?;
        if self.s.construction {
            let new: Vec<CurveId> = next
                .curves
                .range(CurveId(first_new)..)
                .map(|(&c, _)| c)
                .collect();
            for c in new {
                next.set_construction(c, true);
            }
        }
        self.commit(next);
        self.s.prompt = prompt;
        self.s.draft = match (tool, end) {
            (DrawTool::Line, Some(end)) => Draft {
                placed: vec![Placed {
                    at: pt(self.sketch(), end),
                    snap: Some(Snap::Point(end)),
                }],
                previous: last,
                arc: false,
                at_end: true,
                sweep: 0.0,
            },
            _ => Draft::default(),
        };
        Some(())
    }

    /// Ends what is being drawn: a spline with at least two points is
    /// added, anything else — a chain of lines, points placed for a shape
    /// not finished — dropped.
    fn finish_draft(&mut self) {
        if self.s.tool == Tool::Draw(DrawTool::Spline) && self.s.draft.placed.len() >= 2 {
            let hints = Hints {
                sides: self.s.sides,
                circumscribed: self.s.circumscribed,
                tangent_to: None,
                sweep: 0.0,
                min_size: 0.0,
            };
            let _ = self.build(DrawTool::Spline, &hints);
        }
        self.s.draft = Draft::default();
    }

    /// A click with a constraint tool in hand: on a point or a curve, it is
    /// selected for the constraint — or taken out again — and once the
    /// selection is all the constraint needs, and can become nothing else,
    /// it is added. On nothing, a selection that is all it needs is taken
    /// as it is: a line's length rather than the distance between it and
    /// something else. A dimension is only ever added that way: the click
    /// that takes it is where its value goes.
    fn pick_for(&mut self, tool: ConstraintTool, pointer: &Pointer<S>) {
        let visuals = visuals(self.args, self.s, self.selection, &self.frame);
        let hit = hit_key(&visuals, pointer, &[&is_point, &is_curve]);
        let Some(key) = hit else {
            let (picks, _) = selected(self.sketch(), self.selection);
            if tool.fit(self.sketch(), &picks).complete {
                let at = self.in_plane(pointer).map(|(p, _)| p);
                self.apply_constraint(tool, at);
            }
            return;
        };
        match self.selection.iter().position(|k| *k == key) {
            Some(i) => {
                self.selection.remove(i);
            }
            None => self.selection.push(key),
        }
        let (picks, _) = selected(self.sketch(), self.selection);
        let fit = tool.fit(self.sketch(), &picks);
        if fit.complete && !fit.extendable && !tool.is_dimension() {
            self.apply_constraint(tool, None);
        } else if !fit.usable() {
            // What was clicked cannot be part of it: it starts the
            // selection afresh.
            let last = self.selection.pop();
            self.selection.clear();
            self.selection.extend(last);
            let (picks, _) = selected(self.sketch(), self.selection);
            if !tool.fit(self.sketch(), &picks).usable() {
                self.selection.clear();
            }
        }
    }

    /// Adds what `tool` makes of the selection, and clears it — a
    /// dimension's value then shown at `at`, if given, and asked for in
    /// place.
    fn apply_constraint(&mut self, tool: ConstraintTool, at: Option<P2>) {
        let (picks, _) = selected(self.sketch(), self.selection);
        let Some(constraints) = tool.build(self.sketch(), &picks) else {
            return;
        };
        let mut next = self.sketch().clone();
        let ids: Vec<ConstraintId> = constraints.into_iter().map(|c| next.constrain(c)).collect();
        self.commit(next);
        self.selection.clear();
        if tool.is_dimension() {
            let first = ids.first().copied();
            if let (Some(id), Some(at)) = (first, at)
                && let Some((_, anchor)) =
                    constraints::glyph(self.sketch(), &self.sketch().constraints[&id], false)
            {
                self.args.labels.insert(id, sub(at, anchor));
            }
            self.s.prompt = first;
        }
    }

    /// A constraint tool's button pressed: with a selection that is all it
    /// needs, the constraint is added — a dimension once a click has said
    /// where its value goes; otherwise the tool is taken up — or, in hand
    /// already, put down.
    pub(super) fn press_constraint(&mut self, tool: ConstraintTool) {
        let (picks, _) = selected(self.sketch(), self.selection);
        if self.s.tool == Tool::Constrain(tool) {
            self.s.tool = Tool::Select;
        } else if !picks.is_empty()
            && tool.fit(self.sketch(), &picks).complete
            && !tool.is_dimension()
        {
            self.apply_constraint(tool, None);
            self.s.tool = Tool::Select;
        } else {
            self.finish_draft();
            self.s.tool = Tool::Constrain(tool);
        }
    }

    /// The visual `key` dragged from `from` to `to` in the plane: what it
    /// stands for follows the pointer, as far as the constraints let it — a
    /// point, a line's or a spline's points, a circle's radius, an arc's
    /// sweep. A single point dragged snaps onto points and middles — unless
    /// `shift` is held — and is constrained there when let go.
    fn drag(&mut self, key: &str, from: P2, to: P2, pointer: &Pointer<S>, done: bool, shift: bool) {
        if let Some(id) = glyph_key(key) {
            self.drag_label(id, from, to, done);
            return;
        }
        if self.s.drag.is_none() {
            let sketch = self.sketch();
            let points = |points: Vec<PointId>| Drag::Points {
                origins: points.iter().map(|&q| pt(sketch, q)).collect(),
                points,
                grab: from,
            };
            self.s.drag = match (point_key(key), curve_key(key)) {
                (Some(point), _) if sketch.points.get(&point).is_some_and(|p| !p.fixed) => {
                    Some(points(vec![point]))
                }
                (_, Some(curve)) => {
                    sketch
                        .curves
                        .get(&curve)
                        .filter(|c| !c.fixed)
                        .map(|c| match &c.kind {
                            CurveKind::Circle { .. } => Drag::Circle { curve },
                            CurveKind::Arc { .. } => Drag::Arc { curve },
                            CurveKind::Line { .. } | CurveKind::Spline { .. } => points(c.points()),
                        })
                }
                _ => None,
            };
        }
        let Some(drag) = self.s.drag.clone() else {
            return;
        };
        if done {
            self.s.drag = None;
        }
        let mut next = self.sketch().clone();
        let mut drags = Vec::new();
        self.s.snap = None;
        match drag {
            // A dimension's value moves nothing to solve (see `drag_label`).
            Drag::Label { .. } => return,
            Drag::Points {
                points,
                origins,
                grab,
            } => {
                let delta = sub(to, grab);
                drags = points
                    .iter()
                    .zip(&origins)
                    .map(|(&q, &o)| (q, add(o, delta)))
                    .collect();
                if let [(point, target)] = drags[..] {
                    let (at, snap) = self.snap(pointer, target, shift, &[point], false);
                    drags = vec![(point, at)];
                    match snap {
                        Some(snap) if done => snap.constrain(&mut next, point),
                        snap => self.s.snap = snap,
                    }
                }
            }
            Drag::Circle { curve } => {
                let CurveKind::Circle { center, .. } = next.curves[&curve].kind else {
                    return;
                };
                let r = dist(pt(&next, center), to);
                if let Some(c) = next.curves.get_mut(&curve)
                    && let CurveKind::Circle { radius, .. } = &mut c.kind
                {
                    *radius = Design::from_f64(r);
                }
            }
            Drag::Arc { curve } => {
                let CurveKind::Arc { start, end, .. } = next.curves[&curve].kind else {
                    return;
                };
                let through = crate::geometry::sweep_through(pt(&next, start), pt(&next, end), to);
                if !through.is_finite() {
                    return;
                }
                if let Some(c) = next.curves.get_mut(&curve)
                    && let CurveKind::Arc { sweep, .. } = &mut c.kind
                {
                    *sweep = Design::from_f64(through);
                }
            }
        }
        solve(&mut next, self.s, &drags);
        self.args.sketch = next;
    }

    /// The value of the dimension `id` dragged from `from` to `to`: it goes
    /// as far as the pointer went, from where it was shown when grabbed.
    fn drag_label(&mut self, id: ConstraintId, from: P2, to: P2, done: bool) {
        let Some((_, anchor)) = self
            .sketch()
            .constraints
            .get(&id)
            .and_then(|c| constraints::glyph(self.sketch(), c, false))
        else {
            return;
        };
        let grabbed = match &self.s.drag {
            Some(Drag::Label { offset, grab, .. }) => (*offset, *grab),
            _ => {
                // Not placed yet: grabbed where the pointer is on it.
                let offset = self
                    .args
                    .labels
                    .get(&id)
                    .copied()
                    .unwrap_or(sub(from, anchor));
                self.s.drag = Some(Drag::Label {
                    id,
                    offset,
                    grab: from,
                });
                (offset, from)
            }
        };
        self.args
            .labels
            .insert(id, add(grabbed.0, sub(to, grabbed.1)));
        if done {
            self.s.drag = None;
        }
    }

    /// Removes what is selected: points, curves and constraints. What a
    /// projection gave goes with all the projection gave — it would only
    /// come back with the next update — and the sketch's own origin and
    /// axes stay.
    pub(super) fn delete_selection(&mut self) {
        let (picks, constraints) = selected(self.sketch(), self.selection);
        if picks.is_empty() && constraints.is_empty() {
            return;
        }
        let mut next = self.sketch().clone();
        let mut gone_references = Vec::new();
        let (mut points, mut curves) = (Vec::new(), Vec::new());
        for &pick in &picks {
            match self.args.reference_of(pick) {
                Some(i) if self.args.references[i].source == Source::Frame => {}
                Some(i) => gone_references.push(i),
                None => match pick {
                    Pick::Point(p) => points.push(p),
                    Pick::Curve(c) => curves.push(c),
                },
            }
        }
        gone_references.sort_unstable();
        gone_references.dedup();
        for &i in gone_references.iter().rev() {
            self.args.references.remove(i).remove_from(&mut next);
        }
        next.remove(&points, &curves, &constraints);
        self.selection.clear();
        self.commit(next);
    }

    /// The curve the trim tool, at `pointer`, is over — one it can trim —
    /// and where in the plane.
    fn meets(&self, pointer: &Pointer<S>) -> Option<(CurveId, P2)> {
        let (p, _) = self.in_plane(pointer)?;
        let sketch = self.sketch();
        let trimmable = |key: &str| {
            curve_key(key).is_some_and(|c| sketch.curves.get(&c).is_some_and(|c| !c.fixed))
        };
        let visuals = visuals(self.args, self.s, self.selection, &self.frame);
        let curve = hit_key(&visuals, pointer, &[&trimmable])
            .as_deref()
            .and_then(curve_key)?;
        Some((curve, p))
    }

    /// The trim tool dragged from `from` to `to`, released if `done`: what
    /// it meets along the way — under the pointer, and every curve the way
    /// it went crosses — is removed when it is let go.
    fn stroke(&mut self, from: &Pointer<S>, to: &Pointer<S>, done: bool) {
        if self.s.stroke.path.is_empty() {
            let Some((start, _)) = self.in_plane(from) else {
                return;
            };
            self.s.stroke = Stroke {
                path: vec![start],
                met: self.meets(from).into_iter().collect(),
            };
        }
        if let Some((p, _)) = self.in_plane(to) {
            let last = *self.s.stroke.path.last().expect("started above");
            let mut met: Vec<(CurveId, P2)> = self.meets(to).into_iter().collect();
            for (&id, curve) in &self.sketch().curves {
                if curve.fixed {
                    continue;
                }
                let drawn = visuals::drawn(self.args, id);
                met.extend(
                    drawn
                        .windows(2)
                        .filter_map(|w| segments_cross(last, p, w[0], w[1]))
                        .map(|q| (id, q)),
                );
            }
            self.s.stroke.met.extend(met);
            self.s.stroke.path.push(p);
            self.s.cursor = Some(p);
        }
        if done {
            let stroke = std::mem::take(&mut self.s.stroke);
            self.trim(&stroke.met);
        }
    }

    /// Removes the pieces of curves the points `met` are on, up to where
    /// the sketch meets them (see [`super::trim`]).
    fn trim(&mut self, met: &[(CurveId, P2)]) {
        let axes = visuals::axis_ends(self.args);
        let plan = Plan::new(self.sketch(), &axes);
        let removed = plan.pieces(met);
        if removed.is_empty() {
            return;
        }
        let next = plan.apply(&removed);
        self.commit(next);
    }

    /// Puts down what is being drawn, and takes up `tool`.
    pub(super) fn take(&mut self, tool: Tool) {
        self.finish_draft();
        self.s.stroke = Stroke::default();
        self.s.error = None;
        self.s.modify.mirror_line = None;
        if tool == Tool::Modify(ModifyTool::LinearPattern) {
            self.s.modify.spacing = self.default_spacing();
        }
        self.s.tool = if self.s.tool == tool {
            Tool::Select
        } else {
            tool
        };
    }

    /// Makes the selected curves construction geometry — or, if they all
    /// are, profile geometry again; reference geometry stays as it is. With no curve selected, switches
    /// whether what is drawn next is construction geometry.
    pub(super) fn toggle_construction(&mut self) {
        let curves: Vec<CurveId> = selected_curves(self.sketch(), self.selection)
            .into_iter()
            // Reference geometry is construction geometry, always.
            .filter(|&c| self.args.reference_of(Pick::Curve(c)).is_none())
            .collect();
        if curves.is_empty() {
            self.s.construction = !self.s.construction;
            return;
        }
        let mut next = self.sketch().clone();
        let all = curves
            .iter()
            .all(|c| next.curves.get(c).is_some_and(|c| c.construction));
        for &c in &curves {
            next.set_construction(c, !all);
        }
        self.commit(next);
    }

    /// The dimension `id` given `text`: a plain number — an angle in
    /// degrees — or a formula of the part's parameters, which it then
    /// follows. A formula that does not evaluate is refused, saying why.
    pub(super) fn set_value(&mut self, id: ConstraintId, text: &str) {
        let text = text.trim();
        let Some(c) = self.sketch().constraints.get(&id) else {
            return;
        };
        if text.is_empty() || constraints::value(c).is_none() {
            self.s.prompt = None;
            return;
        }
        let inputs = self.before.inputs();
        let value = match evaluate(text, |name| number(inputs, name)) {
            Ok(v) => v,
            Err(e) => {
                self.s.error = Some(e.root_message().to_string());
                return;
            }
        };
        if is_formula(text) {
            self.args.formulas.insert(id, text.to_string());
        } else {
            self.args.formulas.remove(&id);
        }
        let mut next = self.sketch().clone();
        let c = next.constraints.get_mut(&id).expect("checked above");
        let value = if matches!(c, Constraint::Angle { .. }) {
            value.to_radians()
        } else {
            value
        };
        constraints::set_value(c, value);
        self.commit(next);
        self.s.prompt = None;
        self.s.error = None;
    }

    /// The constraint `id`'s entry in the list used: pressed, it is
    /// selected; removed; given a new value.
    pub(super) fn constraint(&mut self, id: ConstraintId, value: &Value) {
        match value {
            Value::Press => *self.selection = vec![id.to_string()],
            Value::Remove => {
                let mut next = self.sketch().clone();
                next.remove(&[], &[], &[id]);
                self.selection.retain(|k| glyph_key(k) != Some(id));
                self.commit(next);
            }
            Value::Text(text) => self.set_value(id, text),
            Value::Number(v) => self.set_value(id, &v.to_string()),
            _ => {}
        }
    }

    /// What the sketch projects is now `entities`: projections of those it
    /// no longer holds are removed, and those new to it projected — those
    /// that are neither a vertex, nor an edge, nor a face skipped.
    pub(super) fn project(&mut self, entities: Vec<EntityRef>) {
        let projectable = |e: &EntityRef| {
            let inner = e.split_instance().map_or(e.clone(), |(_, inner)| inner);
            matches!(
                inner,
                EntityRef::Vertex { .. } | EntityRef::Edge { .. } | EntityRef::Face { .. }
            )
        };
        let mut next = self.sketch().clone();
        let mut kept = Vec::new();
        for reference in std::mem::take(&mut self.args.references) {
            match &reference.source {
                Source::Projection { entity } if !entities.contains(entity) => {
                    reference.remove_from(&mut next);
                }
                _ => kept.push(reference),
            }
        }
        self.s.error = None;
        for entity in entities.into_iter().filter(projectable) {
            let source = Source::Projection { entity };
            if kept.iter().any(|r| r.source == source) {
                continue;
            }
            let mut reference = Reference::new(source);
            match reference.update(&mut next, self.before, &self.frame) {
                Ok(()) => kept.push(reference),
                Err(e) => {
                    reference.remove_from(&mut next);
                    self.s.error = Some(e.root_message().to_string());
                }
            }
        }
        self.args.references = kept;
        self.commit(next);
    }

    /// Escape: close the prompt, else finish the draft, else put the tool
    /// down.
    fn cancel(&mut self) {
        if self.s.prompt.take().is_some() {
            self.s.error = None;
        } else if !self.s.draft.placed.is_empty() {
            self.finish_draft();
        } else {
            self.s.tool = Tool::Select;
            self.s.stroke = Stroke::default();
        }
    }

    /// Enter and a secondary click: apply the constraint tool in hand, if
    /// what it needs is picked, else finish the draft.
    fn confirm(&mut self) {
        match self.s.tool {
            Tool::Constrain(tool) => {
                let (picks, _) = selected(self.sketch(), self.selection);
                if tool.fit(self.sketch(), &picks).complete {
                    self.apply_constraint(tool, None);
                }
            }
            _ => self.finish_draft(),
        }
    }

    /// A key that is a tool's shortcut, lowercased.
    fn key(&mut self, key: &str) {
        let shortcut = |s: Option<&str>| s == Some(key);
        if key == "x" {
            self.toggle_construction();
        } else if shortcut(Some(TRIM_SHORTCUT)) {
            self.take(Tool::Trim);
        } else if let Some(info) = DrawTool::ALL.iter().find(|i| shortcut(i.shortcut)) {
            self.take(Tool::Draw(info.tool));
        } else if let Some(info) = ModifyTool::ALL.iter().find(|i| shortcut(i.shortcut)) {
            self.take(Tool::Modify(info.tool));
        } else if let Some(info) = ConstraintTool::ALL.iter().find(|i| shortcut(i.shortcut)) {
            self.press_constraint(info.tool);
        }
    }

    /// A double click with nothing in hand: on a dimension, its value is
    /// asked for in place.
    fn double_click(&mut self, pointer: &Pointer<S>) {
        let visuals = visuals(self.args, self.s, self.selection, &self.frame);
        if let Some(id) = hit_key(&visuals, pointer, &[&is_glyph])
            .as_deref()
            .and_then(glyph_key)
            && self
                .sketch()
                .constraints
                .get(&id)
                .is_some_and(|c| constraints::value(c).is_some())
        {
            self.s.prompt = Some(id);
            self.s.error = None;
        }
    }

    pub(super) fn event(&mut self, event: &CanvasEvent<S>) {
        match event {
            CanvasEvent::Key { key } => self.key(key),
            CanvasEvent::Cancel => self.cancel(),
            CanvasEvent::Confirm => self.confirm(),
            CanvasEvent::Delete => self.delete_selection(),
            CanvasEvent::Hover { pointer, shift } => self.hover(pointer, *shift),
            CanvasEvent::Leave => {
                self.s.cursor = None;
                self.s.snap = None;
                if self.s.stroke.path.is_empty() {
                    self.s.stroke = Stroke::default();
                }
            }
            CanvasEvent::Click {
                pointer,
                double,
                shift,
            } => match self.s.tool {
                Tool::Select if *double => self.double_click(pointer),
                Tool::Select => {}
                Tool::Constrain(tool) => self.pick_for(tool, pointer),
                Tool::Modify(tool) => self.modify_click(tool, pointer),
                Tool::Trim => {
                    let met: Vec<_> = self.meets(pointer).into_iter().collect();
                    self.trim(&met);
                    // What is under the pointer now.
                    self.hover(pointer, *shift);
                }
                Tool::Draw(tool) => {
                    if *double {
                        self.finish_draft();
                    } else if let Some((p, t)) = self.in_plane(pointer) {
                        self.place(tool, pointer, p, t, *shift);
                    }
                }
            },
            CanvasEvent::Move {
                key,
                from,
                to,
                pointer,
                done,
                shift,
            } => {
                let (from, to) = (self.to_sketch(from), self.to_sketch(to));
                self.drag(key, from, to, pointer, *done, *shift);
            }
            CanvasEvent::Stroke { from, to, done, .. } => {
                if self.s.tool == Tool::Trim {
                    self.stroke(from, to, *done);
                }
            }
            // A planar sketch asks for no gizmo: it is dragged in its plane.
            CanvasEvent::Gizmo { .. } => {}
        }
    }
}
