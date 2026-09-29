//! Editing a sketch step: choosing its plane, then drawing in it with tools,
//! constraining what is drawn, and dragging it — every gesture decided here
//! and answered with a new sketch, which is solved before it is returned.
//!
//! Drawing snaps the way constraints say, never by moving geometry onto
//! what it is near: a point placed on an existing point *is* that point, one
//! placed on a curve is constrained onto it ([`Constraint::PointOnCurve`]),
//! one placed on the sketch's origin is fixed there, and a line drawn nearly
//! horizontal or vertical gets that constraint. Proximity only suggests;
//! the constraint is what the sketch then holds.

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{
    Constraint, ConstraintId, CurveId, CurveKind, PointId, Sketch, SolveReport,
    profile::curve_polyline,
};
use geop_ops::{
    EditContext, Edited, EntityRef,
    operation::resolve_plane,
    ui::{
        Button, ButtonItem, DialogValue, Event, ListItem, Picked, Picking, Pointer, Presentation,
        Shape, Style, Target, Tone, Visual, hit::hit_visuals,
    },
};
use serde::{Deserialize, Serialize};

use crate::{
    AddSketchArgs,
    constraints::{self, Selection},
    geometry::{P2, add, dist, sub, sweep_through},
};

/// What a sketch's plane can be picked from.
const PLANE_TARGETS: &[Target] = &[
    Target::Face,
    Target::Datum(geop_core_math::primitives::DatumKind::Plane),
];

/// Lines drawn within this slope of horizontal or vertical get that
/// constraint.
const AUTO_HV_SLOPE: f64 = 0.034_920_769_491_747_23; // tan(2°)

/// A drawing tool, or selecting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Tool {
    #[default]
    Select,
    Line,
    Rectangle,
    Arc,
    Circle,
    Spline,
    Point,
}

impl Tool {
    /// Every tool, as the palette offers it: `(tool, key, label, shortcut)`.
    const ALL: [(Tool, &'static str, &'static str, &'static str); 7] = [
        (Tool::Select, "select", "Select", "Escape"),
        (Tool::Line, "line", "Line", "l"),
        (Tool::Rectangle, "rectangle", "Rectangle", "r"),
        (Tool::Arc, "arc", "Arc", "a"),
        (Tool::Circle, "circle", "Circle", "c"),
        (Tool::Spline, "spline", "Spline", "s"),
        (Tool::Point, "point", "Point", "p"),
    ];

    fn by_key(key: &str) -> Option<Tool> {
        Tool::ALL.iter().find(|t| t.1 == key).map(|t| t.0)
    }

    fn by_shortcut(shortcut: &str) -> Option<Tool> {
        Tool::ALL
            .iter()
            .find(|t| t.3 != "Escape" && t.3 == shortcut.to_lowercase())
            .map(|t| t.0)
    }
}

/// A curve being drawn: the points placed for it so far.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "tool", rename_all = "snake_case")]
enum Draft {
    Line {
        start: PointId,
    },
    Rectangle {
        corner: PointId,
    },
    Arc {
        start: PointId,
        end: Option<PointId>,
    },
    Circle {
        center: PointId,
    },
    Spline {
        points: Vec<PointId>,
    },
}

/// What a drag moves, as it was grabbed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Drag {
    /// Points, pulled by how far the pointer moved from `grab`.
    Points {
        points: Vec<PointId>,
        origins: Vec<P2>,
        grab: P2,
    },
    /// A circle's radius.
    Circle { curve: CurveId },
    /// An arc's sweep.
    Arc { curve: CurveId },
}

/// How the last solve went.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
enum Solved {
    Solved {
        report: SolveReport,
    },
    /// The sketch could not be solved at all — it is kept as drawn.
    Failed {
        error: String,
    },
}

/// Whether the plane is being chosen or the sketch drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Mode {
    Setup,
    Draw,
}

/// The temporary state of editing a sketch.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SketchSession {
    /// Decided on the first edit: a new, empty sketch starts by choosing
    /// its plane, an existing one straight in the drawing.
    mode: Option<Mode>,
    pick: Picking,
    tool: Tool,
    draft: Option<Draft>,
    selection: Selection,
    selected_constraint: Option<ConstraintId>,
    /// The pointer, in the plane.
    cursor: Option<P2>,
    /// The key of the visual the pointer is over.
    hover: Option<String>,
    drag: Option<Drag>,
    /// Solved once when the edit starts, and after every change.
    solved: Option<Solved>,
}

/// The id in a visual key: `p3` is point 3.
fn key_id(key: &str, prefix: char) -> Option<u64> {
    key.strip_prefix(prefix)?.parse().ok()
}

fn point_key(key: &str) -> Option<PointId> {
    key_id(key, 'p').map(PointId)
}

fn curve_key(key: &str) -> Option<CurveId> {
    key_id(key, 'c').map(CurveId)
}

/// The key of a constraint's glyph.
fn glyph_key(key: &str) -> Option<ConstraintId> {
    key_id(key, 'k').map(ConstraintId)
}

/// The key of the visual among `visuals` the pointer is over, trying the
/// kinds `stages` accept in order: a point drawn on the origin is that
/// point, not the origin.
fn hit_key<S: Scalar>(
    visuals: &[Visual<S>],
    pointer: &Pointer<S>,
    stages: &[&dyn Fn(&str) -> bool],
) -> Option<String> {
    stages.iter().find_map(|accept| {
        hit_visuals(visuals, pointer, |v| accept(&v.key)).map(|h| h.visual.key.clone())
    })
}

fn is_point(key: &str) -> bool {
    point_key(key).is_some()
}

fn is_curve(key: &str) -> bool {
    curve_key(key).is_some()
}

fn is_glyph(key: &str) -> bool {
    glyph_key(key).is_some()
}

fn is_origin(key: &str) -> bool {
    key == "origin"
}

/// The point `id` of `sketch`.
fn pt(sketch: &Sketch, id: PointId) -> P2 {
    sketch.points[&id].xy()
}

/// The edit in progress: what it works on, and where the plane is.
struct Editing<'a, S: Scalar> {
    args: &'a mut AddSketchArgs,
    s: &'a mut SketchSession,
    frame: CoordinateSystem<S>,
}

/// `p` of the sketch, in `frame`'s plane.
fn to_world<S: Scalar>(frame: &CoordinateSystem<S>, p: P2) -> Vector3<S> {
    frame.uv_to_xyz(&Vector2::from_array(p.map(S::from_f64)))
}

/// Solves `sketch`, pulling `drags` towards their targets, and records how
/// that went. A sketch the solver rejects outright stays as drawn.
fn solve(sketch: &mut Sketch, s: &mut SketchSession, drags: &[(PointId, P2)]) {
    s.solved = Some(match sketch.solve_with_drag(drags) {
        Ok(report) => Solved::Solved { report },
        Err(e) => Solved::Failed {
            error: e.root_message().to_string(),
        },
    });
}

impl<S: Scalar> Editing<'_, S> {
    fn sketch(&self) -> &Sketch {
        &self.args.sketch
    }

    /// Where `pointer`'s ray meets the plane, in the sketch's coordinates,
    /// and how far along the ray.
    fn in_plane(&self, pointer: &Pointer<S>) -> Option<(P2, S)> {
        let (t, p) = pointer.ray.intersect_uv_plane(&self.frame)?;
        Some(([p[0].to_f64(), p[1].to_f64()], t))
    }

    /// Makes `next` the sketch, solved.
    fn commit(&mut self, mut next: Sketch) {
        solve(&mut next, self.s, &[]);
        self.args.sketch = next;
    }

    /// What is drawn now: hit tests use these.
    fn visuals(&self) -> Vec<Visual<S>> {
        visuals(self.sketch(), self.s, &self.frame)
    }

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

    fn dialog(&mut self, key: &str, value: &DialogValue) {
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
                DialogValue::Press => {
                    self.s.selected_constraint = Some(id);
                    self.s.selection = Selection::default();
                }
                DialogValue::Remove => {
                    next.remove(&[], &[], &[id]);
                    if self.s.selected_constraint == Some(id) {
                        self.s.selected_constraint = None;
                    }
                    self.commit(next);
                }
                DialogValue::Number(v) => {
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
            "setup" => {
                self.s.draft = None;
                self.s.mode = Some(Mode::Setup);
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

    fn event(&mut self, event: &Event<S>) {
        match event {
            Event::Dialog { key, value } => self.dialog(key, value),
            Event::Key { key } => self.key(key),
            Event::Hover { pointer } => self.hover(pointer),
            Event::Leave => {
                self.s.cursor = None;
                self.s.hover = None;
            }
            Event::Click {
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
            Event::Drag { from, to, done } => {
                if self.s.tool == Tool::Select {
                    self.drag(from, to, *done);
                }
            }
        }
    }
}

/// What the sketch looks like in its plane `frame`, with what is selected,
/// hovered and being drawn in `s`.
fn visuals<S: Scalar>(
    sketch: &Sketch,
    s: &SketchSession,
    frame: &CoordinateSystem<S>,
) -> Vec<Visual<S>> {
    let report = match &s.solved {
        Some(Solved::Solved { report }) => Some(report),
        _ => None,
    };
    let failed: Vec<&Constraint> = report
        .map(|r| {
            r.failed_constraints
                .iter()
                .filter_map(|k| sketch.constraints.get(k))
                .collect()
        })
        .unwrap_or_default();
    let failed_points: Vec<PointId> = failed.iter().flat_map(|c| c.points()).collect();
    let failed_curves: Vec<CurveId> = failed.iter().flat_map(|c| c.curves()).collect();
    let hovered = |key: &str| s.hover.as_deref() == Some(key);
    let world = |p: P2| to_world(frame, p);
    let positions = sketch.positions();
    let mut out = Vec::new();

    out.push(Visual::new(
        "origin",
        Shape::Point {
            at: world([0.0, 0.0]),
        },
        if hovered("origin") {
            Style::Hover
        } else {
            Style::Guide
        },
    ));
    if let Ok(regions) = sketch.regions() {
        for (i, region) in regions.iter().enumerate() {
            let uv = |polyline: Vec<P2>| -> Vec<Vector2<S>> {
                polyline
                    .into_iter()
                    .map(|p| Vector2::from_array(p.map(S::from_f64)))
                    .collect()
            };
            let outer = uv(region.outer.polyline(sketch, &positions));
            let holes: Vec<_> = region
                .holes
                .iter()
                .map(|h| uv(h.polyline(sketch, &positions)))
                .collect();
            out.push(Visual::new(
                format!("region{i}"),
                Shape::region(frame, &outer, &holes),
                Style::Region,
            ));
        }
    }
    for (&id, curve) in &sketch.curves {
        let key = id.to_string();
        let style = if s.selection.curves.contains(&id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if failed_curves.contains(&id) {
            Style::Failed
        } else if curve.construction {
            Style::Construction
        } else if report.is_none_or(|r| r.free_curves.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        out.push(Visual::new(
            key,
            Shape::Polyline {
                points: curve_polyline(sketch, &positions, id)
                    .into_iter()
                    .map(world)
                    .collect(),
            },
            style,
        ));
        if let CurveKind::Spline { control_points } = &curve.kind {
            out.push(Visual::new(
                format!("hull{}", id.0),
                Shape::Polyline {
                    points: control_points
                        .iter()
                        .map(|&p| world(pt(sketch, p)))
                        .collect(),
                },
                Style::Guide,
            ));
        }
    }
    if let (Some(draft), Some(cursor)) = (&s.draft, s.cursor) {
        let preview = draft_preview(sketch, draft, cursor);
        if preview.len() >= 2 {
            out.push(Visual::new(
                "draft",
                Shape::Polyline {
                    points: preview.into_iter().map(world).collect(),
                },
                Style::Draft,
            ));
        }
    }
    for (&id, p) in &sketch.points {
        let key = id.to_string();
        let style = if s.selection.points.contains(&id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if failed_points.contains(&id) {
            Style::Failed
        } else if report.is_none_or(|r| r.free_points.get(&id).copied().unwrap_or(true)) {
            Style::Free
        } else {
            Style::Fixed
        };
        out.push(Visual::new(key, Shape::Point { at: world(p.xy()) }, style));
    }
    // Glyphs of one spot stack sideways, so each can be read and clicked.
    let mut anchors: Vec<P2> = Vec::new();
    for (&id, c) in &sketch.constraints {
        let Some((text, at)) = constraints::glyph(sketch, c) else {
            continue;
        };
        let stack = anchors.iter().filter(|&&a| dist(a, at) < 1e-9).count();
        anchors.push(at);
        let key = id.to_string();
        let style = if s.selected_constraint == Some(id) {
            Style::Selected
        } else if hovered(&key) {
            Style::Hover
        } else if report.is_some_and(|r| r.failed_constraints.contains(&id)) {
            Style::Failed
        } else {
            Style::Fixed
        };
        out.push(Visual::new(
            key,
            Shape::Label {
                at: world(at),
                text,
                offset: frame
                    .u()
                    .prod_scalar(S::from_f64(1.5 + 2.5 * stack as f64))
                    .add(&frame.v().prod_scalar(S::from_f64(1.3))),
            },
            style,
        ));
    }
    out
}

/// What the curve being drawn would be with its next point at `cursor`.
fn draft_preview(sketch: &Sketch, draft: &Draft, cursor: P2) -> Vec<P2> {
    // Previewed as the sketch itself would draw the curve: in a copy, with
    // a point at the cursor.
    let mut temp = sketch.clone();
    let at = temp.add_point(cursor[0], cursor[1]);
    let curve = match draft {
        Draft::Line { start } => temp.add_line(*start, at),
        Draft::Arc { start, end: None } => temp.add_line(*start, at),
        Draft::Arc {
            start,
            end: Some(end),
        } => {
            let sweep = sweep_through(pt(&temp, *start), pt(&temp, *end), cursor);
            if !sweep.is_finite() {
                return Vec::new();
            }
            temp.add_arc_with_sweep(*start, *end, sweep)
        }
        Draft::Circle { center } => {
            let radius = dist(pt(&temp, *center), cursor);
            temp.add_circle(*center, radius)
        }
        Draft::Rectangle { corner } => {
            let [x0, y0] = pt(&temp, *corner);
            return vec![[x0, y0], [cursor[0], y0], cursor, [x0, cursor[1]], [x0, y0]];
        }
        Draft::Spline { points } => {
            let mut points = points.clone();
            points.push(at);
            temp.add_spline(points)
        }
    };
    curve_polyline(&temp, &temp.positions(), curve)
}

/// What each tool asks for next.
fn tool_hint(tool: Tool, draft: Option<&Draft>) -> &'static str {
    match (tool, draft) {
        (Tool::Select, _) => {
            "Click to select (adds to the selection) · drag points or curves · drag empty space to pan"
        }
        (Tool::Line, None) => "Click the start point",
        (Tool::Line, Some(_)) => "Click the next point · right-click / Esc ends the chain",
        (Tool::Rectangle, None) => "Click one corner",
        (Tool::Rectangle, Some(_)) => "Click the opposite corner",
        (Tool::Arc, None) => "Click the start point",
        (Tool::Arc, Some(Draft::Arc { end: None, .. })) => "Click the end point",
        (Tool::Arc, Some(_)) => "Click a point the arc passes through",
        (Tool::Circle, None) => "Click the center",
        (Tool::Circle, Some(_)) => "Click a point on the circle",
        (Tool::Spline, _) => "Click control points · double-click / Enter / right-click finishes",
        (Tool::Point, _) => "Click to place a point",
    }
}

/// The dialog while drawing.
fn draw_dialog(sketch: &Sketch, s: &SketchSession) -> geop_ops::ui::Dialog {
    let mut d = geop_ops::ui::Dialog::new();
    d.heading("draw_heading", "Draw");
    d.buttons(
        "tools",
        Tool::ALL
            .iter()
            .map(|&(tool, key, label, shortcut)| {
                ButtonItem::new(format!("tool:{key}"), label)
                    .title(format!("Shortcut: {shortcut}"))
                    .active(s.tool == tool)
            })
            .collect(),
    );
    d.text("tool_hint", tool_hint(s.tool, s.draft.as_ref()), Tone::Hint);

    d.heading("constrain_heading", "Constrain");
    let options = constraints::options(sketch, &s.selection);
    if options.is_empty() && s.selection.curves.is_empty() {
        d.text(
            "constrain_hint",
            "Select points and curves (Select tool) to see the constraints that apply.",
            Tone::Hint,
        );
    }
    if !options.is_empty() {
        d.buttons(
            "constrain",
            options
                .iter()
                .enumerate()
                .map(|(i, o)| ButtonItem::new(format!("constrain:{i}"), o.label).title(o.title))
                .collect(),
        );
    }
    let mut edits = Vec::new();
    if !s.selection.curves.is_empty() {
        edits.push(ButtonItem::new("construction", "Construction"));
    }
    if !s.selection.is_empty() || s.selected_constraint.is_some() {
        edits.push(ButtonItem::new("delete", "Delete"));
    }
    if !edits.is_empty() {
        d.buttons("edit", edits);
    }

    d.heading("status_heading", "Status");
    let (status, tone) = match &s.solved {
        Some(Solved::Failed { error }) => (error.clone(), Tone::Error),
        Some(Solved::Solved { report }) if !report.converged => (
            format!(
                "Over-constrained or conflicting: {} constraint(s) cannot be met",
                report.failed_constraints.len()
            ),
            Tone::Error,
        ),
        Some(Solved::Solved { report }) if report.dof == 0 => {
            ("Fully constrained".into(), Tone::Success)
        }
        Some(Solved::Solved { report }) => (
            format!(
                "{} degree{} of freedom",
                report.dof,
                if report.dof == 1 { "" } else { "s" }
            ),
            Tone::Hint,
        ),
        None => (String::new(), Tone::Hint),
    };
    d.text("status", status, tone);
    let curves = sketch.curves.values().filter(|c| !c.construction).count();
    let regions = match sketch.regions() {
        Ok(regions) => format!(
            "{} closed region{}",
            regions.len(),
            if regions.len() == 1 { "" } else { "s" }
        ),
        Err(e) => e.root_message().to_string(),
    };
    d.text(
        "counts",
        format!(
            "{curves} curve{} · {regions}",
            if curves == 1 { "" } else { "s" }
        ),
        Tone::Hint,
    );

    d.heading("constraints_heading", "Constraints");
    let failed = match &s.solved {
        Some(Solved::Solved { report }) => report.failed_constraints.clone(),
        _ => Vec::new(),
    };
    d.list(
        "constraints",
        sketch
            .constraints
            .iter()
            .map(|(&id, c)| {
                let mut item = ListItem::new(format!("constraint:{}", id.0), constraints::name(c));
                item.selected = s.selected_constraint == Some(id);
                item.removable = true;
                item.tone = if failed.contains(&id) {
                    Tone::Error
                } else {
                    Tone::Normal
                };
                item.value = constraints::value(c).map(|v| {
                    if matches!(c, Constraint::Angle { .. }) {
                        v.to_degrees()
                    } else {
                        v
                    }
                });
                item
            })
            .collect(),
        "None yet.",
    );
    if let Some([x, y]) = s.cursor {
        d.text("cursor", format!("x {x:.3} · y {y:.3}"), Tone::Hint);
    }
    d.button("setup", "Sketch setup…");
    d
}

/// Edits a sketch step: see the module docs.
pub(crate) fn edit<S: Scalar>(
    ctx: &EditContext<S>,
    mut args: AddSketchArgs,
    mut s: SketchSession,
    event: Option<&Event<S>>,
) -> Edited<AddSketchArgs, SketchSession, S> {
    if s.mode.is_none() {
        // A new sketch starts by picking its plane, an existing one in the
        // drawing.
        let new = args.sketch.curves.is_empty();
        s.mode = Some(if new { Mode::Setup } else { Mode::Draw });
        if new {
            s.pick.arm("plane");
        }
    }
    if s.solved.is_none() {
        solve(&mut args.sketch, &mut s, &[]);
    }
    let plane = resolve_plane(ctx.part, &args.plane);

    if let Some(event) = event {
        match (s.mode, plane) {
            (Some(Mode::Draw), Ok(frame)) => {
                Editing {
                    args: &mut args,
                    s: &mut s,
                    frame,
                }
                .event(event);
            }
            _ => setup_event(ctx, &mut args, &mut s, event),
        }
    }

    let plane = resolve_plane(ctx.part, &args.plane);
    let presentation = match (s.mode, plane) {
        (Some(Mode::Draw), Ok(frame)) => {
            let visuals = visuals(&args.sketch, &s, &frame);
            let grab = s.tool == Tool::Select
                && s.hover
                    .as_deref()
                    .is_some_and(|k| point_key(k).is_some() || curve_key(k).is_some());
            Presentation {
                dialog: draw_dialog(&args.sketch, &s),
                visuals,
                focus: Some(frame),
                grab,
                ..Presentation::default()
            }
        }
        (_, plane) => {
            setup_presentation(&args, &s, plane.err().map(|e| e.root_message().to_string()))
        }
    };
    Edited {
        args,
        session: s,
        presentation,
    }
}

/// An event while the plane is being chosen.
fn setup_event<S: Scalar>(
    ctx: &EditContext<S>,
    args: &mut AddSketchArgs,
    s: &mut SketchSession,
    event: &Event<S>,
) {
    match event.dialog() {
        Some(("plane", _)) => s.pick.toggle("plane"),
        Some(("draw", _)) if resolve_plane(ctx.part, &args.plane).is_ok() => {
            s.pick.disarm();
            s.mode = Some(Mode::Draw);
        }
        _ => {}
    }
    if let Event::Key { key } = event
        && key == "Escape"
    {
        s.pick.disarm();
    }
    if let Picked::Picked { entity, .. } = s.pick.handle(ctx.view, event, PLANE_TARGETS)
        && resolve_plane(ctx.part, &entity).is_ok()
    {
        // The plane is what the sketch needed: straight on to drawing it.
        args.plane = entity;
        s.pick.disarm();
        s.mode = Some(Mode::Draw);
    }
}

/// What is shown while the plane is being chosen.
fn setup_presentation<S: Scalar>(
    args: &AddSketchArgs,
    s: &SketchSession,
    plane_error: Option<String>,
) -> Presentation<S> {
    let mut d = geop_ops::ui::Dialog::new();
    let picking = s.pick.is("plane");
    d.pick_button("plane", format!("plane: {}", args.plane.label()), picking);
    if picking {
        d.text(
            "plane_hint",
            "Click a planar face, a datum plane or one of the origin's planes…",
            Tone::Hint,
        );
    }
    if let Some(error) = plane_error {
        d.text("plane_error", error, Tone::Error);
    }
    let n = args.sketch.curves.len();
    d.push(
        "draw",
        geop_ops::ui::Control::Button {
            label: if n == 0 {
                "Draw sketch…".into()
            } else {
                format!("Edit sketch ({n} curve{})…", if n == 1 { "" } else { "s" })
            },
            title: None,
            active: false,
            enabled: true,
            primary: true,
        },
    );
    let mut highlights: Vec<EntityRef> = s.pick.hover.clone().into_iter().collect();
    if !picking {
        highlights.push(args.plane.clone());
    }
    Presentation {
        dialog: d,
        highlights,
        pickable: if picking {
            PLANE_TARGETS.to_vec()
        } else {
            Vec::new()
        },
        ..Presentation::default()
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        primitives::{DatumComponent, FrameAxis, Ray},
        scalars::ScalInF64 as S,
    };
    use geop_ops::{
        ORIGIN, Part,
        operation::Operation,
        ui::{Control, PartView, Reach},
    };

    use super::*;
    use crate::AddSketch;

    /// A sketch step being edited on an empty part, as an editor drives it.
    struct Editor {
        part: Part<S>,
        view: PartView<S>,
        args: AddSketchArgs,
        session: SketchSession,
        presentation: Presentation<S>,
    }

    impl Editor {
        fn new() -> Self {
            let part = Part::new();
            let view = PartView::of(&part).unwrap();
            let args = AddSketch.new_args(&part);
            let mut editor = Editor {
                part,
                view,
                args,
                session: SketchSession::default(),
                presentation: Presentation::default(),
            };
            editor.send(None);
            editor
        }

        fn send(&mut self, event: Option<Event<S>>) {
            let ctx = EditContext {
                part: &self.part,
                view: &self.view,
            };
            let edited = AddSketch.edit(
                &ctx,
                self.args.clone(),
                self.session.clone(),
                event.as_ref(),
            );
            self.args = edited.args;
            self.session = edited.session;
            self.presentation = edited.presentation;
        }

        fn press(&mut self, key: &str) {
            self.send(Some(Event::Dialog {
                key: key.into(),
                value: DialogValue::Press,
            }));
        }

        fn click(&mut self, x: f64, y: f64) {
            self.send(Some(Event::Click {
                pointer: down(x, y),
                button: Button::Primary,
                double: false,
                shift: false,
            }));
        }

        fn key(&mut self, key: &str) {
            self.send(Some(Event::Key { key: key.into() }));
        }

        fn sketch(&self) -> &Sketch {
            &self.args.sketch
        }

        /// The labels of the constraint buttons the dialog offers.
        fn constrain_options(&self) -> Vec<String> {
            match self.presentation.dialog.get("constrain") {
                Some(Control::Buttons { buttons }) => {
                    buttons.iter().map(|b| b.label.clone()).collect()
                }
                _ => Vec::new(),
            }
        }
    }

    fn v(p: [f64; 3]) -> Vector3<S> {
        Vector3::from_array(p.map(S::from_f64))
    }

    /// A pointer from `origin` along `dir`, reaching 0.009 units.
    fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
        Pointer {
            ray: Ray::try_new(v(origin), v(dir)).unwrap(),
            reach: Reach::Tube {
                radius: S::from_f64(0.009),
            },
        }
    }

    /// Straight down onto the `xy` plane at `(x, y)`.
    fn down(x: f64, y: f64) -> Pointer<S> {
        pointer([x, y, 10.0], [0.0, 0.0, -1.0])
    }

    /// Straight into drawing on the default plane.
    fn drawing() -> Editor {
        let mut e = Editor::new();
        e.press("draw");
        e
    }

    fn close(a: P2, b: P2) -> bool {
        dist(a, b) < 1e-6
    }

    /// A new sketch starts by picking its plane; picking one goes straight
    /// on to drawing on it, head on.
    #[test]
    fn a_new_sketch_picks_its_plane_first() {
        let mut e = Editor::new();
        assert!(e.session.pick.is("plane"));
        assert!(e.presentation.focus.is_none());
        assert!(
            e.presentation
                .pickable
                .contains(&Target::Datum(geop_core_math::primitives::DatumKind::Plane))
        );
        // The origin's zx plane's square, ten reaches of 0.009 from the
        // origin: seen from the front, at (0.04, 0, 0.04).
        let front = pointer([0.04, -10.0, 0.04], [0.0, 1.0, 0.0]);
        e.send(Some(Event::Click {
            pointer: front,
            button: Button::Primary,
            double: false,
            shift: false,
        }));
        assert_eq!(
            e.args.plane,
            EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y))
        );
        let focus = e.presentation.focus.expect("drawing faces the plane");
        assert_eq!(*focus.w(), v([0.0, 1.0, 0.0]));
    }

    /// Lines chain; one drawn nearly horizontal is constrained horizontal,
    /// one started on the origin starts at a point fixed there, and one
    /// ended on an existing point ends at that very point.
    #[test]
    fn lines_snap_by_constraint() {
        let mut e = drawing();
        e.press("tool:line");
        e.click(0.001, 0.0);
        e.click(1.0, 0.02);
        e.click(1.0, 1.0);
        e.click(0.003, -0.002);
        e.key("Escape");
        let s = e.sketch();
        assert_eq!(s.curves.len(), 3);
        assert_eq!(s.points.len(), 3, "the chain closed on its first point");
        let has = |pred: &dyn Fn(&Constraint) -> bool| s.constraints.values().any(pred);
        assert!(has(&|c| matches!(
            c,
            Constraint::Fix { x: 0.0, y: 0.0, .. }
        )));
        assert!(has(&|c| matches!(c, Constraint::Horizontal { .. })));
        assert!(has(&|c| matches!(c, Constraint::Vertical { .. })));
        // Solved: horizontal and vertical hold exactly.
        let corner = s
            .points
            .values()
            .map(|p| p.xy())
            .find(|p| p[0] > 0.5 && p[1] < 0.5)
            .unwrap();
        assert!(corner[1].abs() < 1e-6, "{corner:?}");
        assert!(e.session.draft.is_none());
    }

    /// A point placed on a curve is constrained onto it, not moved.
    #[test]
    fn points_on_curves_are_constrained_onto_them() {
        let mut e = drawing();
        e.press("tool:line");
        e.click(-1.0, -1.0);
        e.click(1.0, 1.0);
        e.key("Escape");
        e.press("tool:point");
        e.click(0.302, 0.3);
        let s = e.sketch();
        assert_eq!(s.points.len(), 3);
        assert!(
            s.constraints
                .values()
                .any(|c| matches!(c, Constraint::PointOnCurve { .. }))
        );
    }

    /// Two clicks make a rectangle: four lines, horizontal and vertical by
    /// constraint, bounding one region — shown shaded.
    #[test]
    fn rectangles() {
        let mut e = drawing();
        e.key("r");
        e.click(0.2, 0.2);
        e.send(Some(Event::Hover {
            pointer: down(0.8, 0.6),
        }));
        assert!(
            e.presentation.visuals.iter().any(|v| v.key == "draft"),
            "the rectangle is previewed"
        );
        e.click(1.2, 0.7);
        let s = e.sketch();
        assert_eq!(s.curves.len(), 4);
        assert_eq!(s.constraints.len(), 4);
        assert_eq!(s.regions().unwrap().len(), 1);
        assert!(
            e.presentation
                .visuals
                .iter()
                .any(|v| v.style == Style::Region)
        );
    }

    /// Selecting two lines offers what fits two lines; choosing one adds it.
    /// The Delete key removes what is selected.
    #[test]
    fn selections_offer_constraints() {
        let mut e = drawing();
        e.press("tool:line");
        e.click(0.0, 0.5);
        e.click(1.0, 0.7);
        e.key("Escape");
        e.click(0.0, 1.5);
        e.click(1.0, 2.0);
        e.key("Escape");
        e.key("Escape");
        assert_eq!(e.session.tool, Tool::Select);
        e.click(0.5, 0.6);
        e.click(0.5, 1.75);
        assert_eq!(e.session.selection.curves.len(), 2);
        let options = e.constrain_options();
        let parallel = options.iter().position(|o| o == "Parallel").unwrap();
        e.press(&format!("constrain:{parallel}"));
        assert!(
            e.sketch()
                .constraints
                .values()
                .any(|c| matches!(c, Constraint::Parallel { .. }))
        );
        assert!(e.session.selection.is_empty());

        e.click(0.5, 0.6);
        e.key("Delete");
        assert_eq!(e.sketch().curves.len(), 1);
        assert!(
            e.sketch().constraints.is_empty(),
            "the parallel went with it"
        );
    }

    /// Hovering a point offers a grab; dragging it moves it, as far as the
    /// constraints let it.
    #[test]
    fn dragging_points() {
        let mut e = drawing();
        e.press("tool:line");
        e.click(0.0, 0.0);
        e.click(1.0, 0.0);
        e.key("Escape");
        e.key("Escape");
        e.send(Some(Event::Hover {
            pointer: down(1.0, 0.0),
        }));
        assert!(e.presentation.grab);
        e.send(Some(Event::Drag {
            from: down(1.0, 0.0),
            to: down(2.0, 0.4),
            done: true,
        }));
        let s = e.sketch();
        let ends: Vec<P2> = s.points.values().map(|p| p.xy()).collect();
        // The line starts fixed at the origin and is horizontal by
        // constraint: the end follows the pointer along it, not up.
        assert!(ends.iter().any(|&p| close(p, [0.0, 0.0])), "{ends:?}");
        assert!(ends.iter().any(|&p| close(p, [2.0, 0.0])), "{ends:?}");
        assert!(e.session.drag.is_none());
    }

    /// A constraint's value is edited in the list, an angle in degrees.
    #[test]
    fn constraint_values_are_edited_in_the_list() {
        let mut e = drawing();
        e.press("tool:line");
        e.click(0.0, 0.5);
        e.click(1.0, 0.8);
        e.key("Escape");
        e.key("Escape");
        e.click(0.5, 0.65);
        let options = e.constrain_options();
        let length = options.iter().position(|o| o == "Length").unwrap();
        e.press(&format!("constrain:{length}"));
        let (&id, _) = e.sketch().constraints.iter().next().unwrap();
        e.send(Some(Event::Dialog {
            key: format!("constraint:{}", id.0),
            value: DialogValue::Number(2.0),
        }));
        let s = e.sketch();
        let ends: Vec<P2> = s.points.values().map(|p| p.xy()).collect();
        assert!((dist(ends[0], ends[1]) - 2.0).abs() < 1e-6, "{ends:?}");
    }

    /// Sketch setup goes back to choosing the plane, drawing kept.
    #[test]
    fn setup_and_back() {
        let mut e = drawing();
        e.key("c");
        e.click(0.0, 0.0);
        e.click(0.5, 0.0);
        assert_eq!(e.sketch().curves.len(), 1);
        e.press("setup");
        assert!(e.presentation.focus.is_none());
        assert!(matches!(
            e.presentation.dialog.get("draw"),
            Some(Control::Button { label, .. }) if label.contains("1 curve")
        ));
        e.press("draw");
        assert!(e.presentation.focus.is_some());
    }
}
