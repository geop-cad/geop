//! Editing a 3-D sketch step: drawing points, lines, arcs and splines in
//! space, constraining them, dragging them — every gesture answered with a
//! new sketch, solved before it is returned.
//!
//! A click places a point where the pointer hits: on a point the sketch
//! has, it *is* that point; on a vertex of the part or a datum point, it is
//! a fixed point there, kept there as the part changes; on an edge, it is
//! constrained onto the edge; on a face or a datum plane, it is where the
//! face is hit. Anywhere else it lies in the plane through the last point
//! placed, facing the eye. Coordinates can be typed instead.
//!
//! The lines a line tool draws chain from point to point, and so do the
//! arcs of the arc tool: from where the last ended, through a point, to
//! another. A spline takes points until it is finished — a double click,
//! Enter — and goes on smoothly from a curve it starts at, or into one it
//! ends at (see [`geop_core_sketch::space::Sketch3d::adopts`]).

use geop_core_math::{
    scalars::{Field, Scalar},
    vector::Vector3,
};
use geop_core_sketch::{
    ConstraintId, CurveId, PointId,
    space::{Constraint3d, Coordinate, CurveKind3d, End, Solve3dReport},
};
use geop_ops::{
    Context, Design, Part,
    operation::{Aspects, EntityRef, Role},
    ui::{
        Action, Button, CanvasEvent, Edit, Form, Gizmo, InHand, ListItem, Number, Pointer, Shape,
        Style, Tone, Unit, Value, Visual, hit::hit_visuals,
    },
};

use crate::{AddSketch3dArgs, Reference3d, Target};

type V3 = Vector3<Design>;

/// What a click in the viewport does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    /// Selecting and dragging.
    Select,
    Point,
    Line,
    Arc,
    Spline,
}

impl Tool {
    /// Every tool: how events name it, its label and its key.
    const ALL: [(Tool, &'static str, &'static str, &'static str); 5] = [
        (Tool::Select, "select", "Select", "Escape"),
        (Tool::Point, "point", "Point", "p"),
        (Tool::Line, "line", "Line", "l"),
        (Tool::Arc, "arc", "3-point arc", "a"),
        (Tool::Spline, "spline", "Spline", "s"),
    ];

    fn draws(self) -> bool {
        self != Tool::Select
    }
}

/// The state of editing a 3-D sketch beyond its arguments.
#[derive(Debug, Default)]
pub struct Sketch3dSession {
    /// The tool in hand, once one is chosen: until then, a line for an
    /// empty sketch, selecting for one with curves.
    tool: Option<Tool>,
    /// The points placed for what is being drawn: a chain's last point, an
    /// arc's start and the point it passes through, a spline's points.
    placed: Vec<PointId>,
    /// The points placed that were new: taken out again if what is drawn
    /// ends without using them.
    fresh: Vec<PointId>,
    /// Where a click would place a point now.
    hover: Option<V3>,
    /// The coordinates typed for the next point.
    typed: [f64; 3],
}

impl Sketch3dSession {
    /// The tool in hand.
    pub fn tool(&self, args: &AddSketch3dArgs) -> Tool {
        self.tool.unwrap_or_else(|| {
            if args.sketch.curves.values().any(|c| c.is_drawn()) {
                Tool::Select
            } else {
                Tool::Line
            }
        })
    }
}

/// What the selection holds of the sketch.
#[derive(Default)]
struct Picks {
    points: Vec<PointId>,
    curves: Vec<CurveId>,
}

/// The points the gizmo moves: those selected and those of the curves
/// selected, each once — but none fixed, which do not move.
fn moved(args: &AddSketch3dArgs, picks: &Picks) -> Vec<PointId> {
    let sketch = &args.sketch;
    let mut points: Vec<PointId> = picks.points.clone();
    for c in &picks.curves {
        points.extend(sketch.curves[c].points());
    }
    let mut seen = std::collections::BTreeSet::new();
    points.retain(|p| seen.insert(*p) && !sketch.points[p].fixed);
    points
}

fn picked(args: &AddSketch3dArgs, selection: &[String]) -> Picks {
    let sketch = &args.sketch;
    let mut picks = Picks::default();
    for key in selection {
        if let Some(&p) = sketch.points.keys().find(|p| p.to_string() == *key) {
            picks.points.push(p);
        } else if let Some(&c) = sketch.curves.keys().find(|c| c.to_string() == *key) {
            picks.curves.push(c);
        }
    }
    picks
}

/// `args` with what the part `before` gives it brought up to date, and how
/// its constraints stand — or why that cannot be said.
fn checked<S: Scalar>(
    before: &Part<S>,
    args: &AddSketch3dArgs,
) -> Result<(AddSketch3dArgs, Solve3dReport<Design>), String> {
    let mut args = args.clone();
    args.resolve(before)
        .map_err(|e| e.root_message().to_string())?;
    let report = args
        .sketch
        .check()
        .map_err(|e| e.root_message().to_string())?;
    Ok((args, report))
}

/// Solves `args` on `before`, pulling `drags` — what the part gives
/// brought up to date first. What cannot be solved is left as it is, and
/// the form says why.
fn solve<S: Scalar>(before: &Part<S>, args: &mut AddSketch3dArgs, drags: &[(PointId, V3)]) {
    args.tidy();
    if args.resolve(before).is_ok() {
        let _ = args.sketch.solve_with_drag(drags);
    }
}

// ── where a click goes ──────────────────────────────────────────────────────

/// Where a click places a point (see the module docs).
#[derive(Clone, Debug)]
enum Located {
    /// A point the sketch has.
    Point(PointId),
    /// A point of the part — a vertex, a datum point: a fixed point there.
    Fixed(EntityRef, V3),
    /// A point of an edge of the part: on it.
    OnEdge(EntityRef, V3),
    /// Anywhere else.
    Free(V3),
}

impl Located {
    fn at(&self, args: &AddSketch3dArgs) -> V3 {
        match self {
            Located::Point(p) => args.sketch.points[p].at,
            Located::Fixed(_, at) | Located::OnEdge(_, at) | Located::Free(at) => *at,
        }
    }
}

/// Where a click with `pointer` places a point.
fn locate<S: Scalar>(
    context: &Context<'_, S>,
    args: &AddSketch3dArgs,
    s: &Sketch3dSession,
    pointer: &Pointer<S>,
) -> Option<Located> {
    let design = |v: &Vector3<S>| v.map(|c| c.cast::<Design>());
    let points: Vec<Visual<S>> = args
        .sketch
        .points
        .iter()
        .map(|(id, p)| {
            Visual::new(
                id.to_string(),
                Shape::Point {
                    at: p.at.map(|c| c.cast()),
                },
                Style::Free,
            )
        })
        .collect();
    if let Some(hit) = hit_visuals(&points, pointer, None, |_| true) {
        let id = args
            .sketch
            .points
            .keys()
            .find(|p| p.to_string() == hit.visual.key)?;
        return Some(Located::Point(*id));
    }
    if let Some(view) = context.view
        && let Some(hit) = view.pick(
            pointer,
            &[Role::Point, Role::Edge, Role::Face, Role::Plane],
            None,
        )
    {
        let aspects = Aspects::of(&hit.entity, context.before).ok()?;
        return Some(if let Some(at) = aspects.point {
            Located::Fixed(hit.entity, design(&at))
        } else if aspects.curve.is_some() {
            Located::OnEdge(hit.entity, design(&hit.point))
        } else {
            Located::Free(design(&hit.point))
        });
    }
    // In the plane through the last point placed — or the origin — facing
    // the eye.
    let anchor = s
        .placed
        .last()
        .map(|p| args.sketch.points[p].at.map(|c| c.cast::<S>()))
        .unwrap_or_else(Vector3::zero);
    let (_, at) = pointer.ray.intersect_plane(&anchor, pointer.ray.dir())?;
    Some(Located::Free(design(&at)))
}

/// The point a click at `located` places: the sketch's own, or a new one —
/// fixed at a point of the part, or constrained onto an edge of it.
fn place(args: &mut AddSketch3dArgs, s: &mut Sketch3dSession, located: Located) -> PointId {
    let sketch = &mut args.sketch;
    match located {
        Located::Point(p) => p,
        Located::Fixed(entity, at) => {
            let known = args.references.iter().find_map(|r| match r.target {
                Target::Point { point } if r.entity == entity => Some(point),
                _ => None,
            });
            known.unwrap_or_else(|| {
                let point = sketch.add_fixed_point(at);
                args.references.push(Reference3d {
                    entity,
                    target: Target::Point { point },
                });
                s.fresh.push(point);
                point
            })
        }
        Located::OnEdge(entity, at) => {
            let known = args.references.iter().find_map(|r| match r.target {
                Target::Curve { curve } if r.entity == entity => Some(curve),
                _ => None,
            });
            let curve = known.unwrap_or_else(|| {
                let curve = sketch.add_curve(CurveKind3d::Reference);
                args.references.push(Reference3d {
                    entity,
                    target: Target::Curve { curve },
                });
                curve
            });
            let point = sketch.add_point(at);
            sketch.constrain(Constraint3d::OnCurve { point, curve });
            s.fresh.push(point);
            point
        }
        Located::Free(at) => {
            let point = sketch.add_point(at);
            s.fresh.push(point);
            point
        }
    }
}

/// Whether `a` and `b` are one point of the sketch.
fn same(args: &AddSketch3dArgs, a: PointId, b: PointId) -> bool {
    let class = args.sketch.point_classes();
    class[&a] == class[&b]
}

/// Makes the spline ends of `curve`, and the spline ends it meets, go on
/// smoothly where it meets one other curve — unless that end already has a
/// direction.
fn smooth_joints(args: &mut AddSketch3dArgs, curve: CurveId) {
    let sketch = &args.sketch;
    let is_spline = |c: CurveId| matches!(sketch.curves[&c].kind, CurveKind3d::Spline { .. });
    let others: Vec<CurveId> = sketch
        .curves
        .iter()
        .filter(|&(&c, k)| c != curve && k.is_drawn() && !k.construction)
        .map(|(&c, _)| c)
        .collect();
    let mut tangents = Vec::new();
    for &other in &others {
        if !(is_spline(curve) || is_spline(other)) {
            continue;
        }
        let Ok(Some((end, _))) = sketch.shared_end(curve, other) else {
            continue;
        };
        // Only where the two meet alone.
        let meeting = others
            .iter()
            .filter(|&&c| {
                sketch
                    .shared_end(curve, c)
                    .ok()
                    .flatten()
                    .is_some_and(|(e, _)| e == end)
            })
            .count();
        if meeting == 1 {
            tangents.push(Constraint3d::Tangent { a: other, b: curve });
        }
    }
    for tangent in tangents {
        let id = args.sketch.constrain(tangent);
        if args.sketch.validate().is_err() {
            args.sketch.remove(&[], &[], &[id]);
        }
    }
}

/// A click of the tool `tool` at `located`.
fn click<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketch3dArgs,
    s: &mut Sketch3dSession,
    tool: Tool,
    located: Located,
) {
    let p = place(args, s, located);
    match tool {
        Tool::Select => {}
        Tool::Point => {
            s.fresh.clear();
        }
        Tool::Line => {
            if let Some(&last) = s.placed.last()
                && !same(args, last, p)
            {
                let line = args.sketch.add_line(last, p);
                smooth_joints(args, line);
                s.fresh.clear();
            }
            s.placed = vec![p];
        }
        Tool::Arc => {
            if s.placed.last().is_none_or(|&last| !same(args, last, p)) {
                s.placed.push(p);
            }
            if let [start, through, end] = s.placed[..] {
                if !same(args, start, end) {
                    let arc = args.sketch.add_arc(start, through, end);
                    smooth_joints(args, arc);
                    s.fresh.clear();
                }
                s.placed = vec![end];
            }
        }
        Tool::Spline => {
            if s.placed.last().is_none_or(|&last| !same(args, last, p)) {
                s.placed.push(p);
            }
        }
    }
    solve(before, args, &[]);
}

/// Ends what is being drawn: a spline is added through the points placed,
/// and points placed for nothing go again.
fn finish<S: Scalar>(before: &Part<S>, args: &mut AddSketch3dArgs, s: &mut Sketch3dSession) {
    if s.tool(args) == Tool::Spline && s.placed.len() >= 2 {
        let spline = args.sketch.add_spline(std::mem::take(&mut s.placed));
        smooth_joints(args, spline);
        s.fresh.clear();
    }
    s.placed.clear();
    let used = |p: &PointId| {
        args.sketch.curves.values().any(|c| c.points().contains(p))
            || args
                .sketch
                .constraints
                .values()
                .any(|k| !matches!(k, Constraint3d::OnCurve { .. }) && k.points().contains(p))
    };
    let unused: Vec<PointId> = s.fresh.iter().copied().filter(|p| !used(p)).collect();
    s.fresh.clear();
    if !unused.is_empty() {
        args.sketch.remove(&unused, &[], &[]);
    }
    solve(before, args, &[]);
}

/// Takes up `tool`, ending what was being drawn.
fn take_up<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketch3dArgs,
    s: &mut Sketch3dSession,
    tool: Tool,
) {
    finish(before, args, s);
    s.tool = Some(tool);
    s.hover = None;
}

/// Removes what is selected, and everything on it.
fn remove_selected<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketch3dArgs,
    selection: &mut Vec<String>,
) {
    let picks = picked(args, selection);
    args.sketch.remove(&picks.points, &picks.curves, &[]);
    selection.clear();
    solve(before, args, &[]);
}

/// A pointer or key event passed on (see the module docs).
pub(crate) fn event<S: Scalar>(
    context: Context<'_, S>,
    edit: Edit<'_, AddSketch3dArgs, Sketch3dSession>,
    event: &CanvasEvent<S>,
) {
    let Edit {
        args,
        session: s,
        selection,
        ..
    } = edit;
    let before = context.before;
    let tool = s.tool(args);
    s.tool = Some(tool);
    match event {
        CanvasEvent::Hover { pointer, .. } => {
            s.hover = tool
                .draws()
                .then(|| locate(&context, args, s, pointer).map(|l| l.at(args)))
                .flatten();
        }
        CanvasEvent::Leave => s.hover = None,
        CanvasEvent::Click {
            button: Button::Primary,
            double: true,
            ..
        } if tool.draws() => finish(before, args, s),
        CanvasEvent::Click {
            pointer,
            button: Button::Primary,
            ..
        } if tool.draws() => {
            if let Some(located) = locate(&context, args, s, pointer) {
                click(before, args, s, tool, located);
            }
        }
        CanvasEvent::Click {
            button: Button::Secondary,
            ..
        } => finish(before, args, s),
        CanvasEvent::Move { key, to, .. } => {
            let point = args
                .sketch
                .points
                .iter()
                .find(|(id, p)| id.to_string() == *key && !p.fixed)
                .map(|(&id, _)| id);
            if let Some(p) = point {
                solve(before, args, &[(p, to.map(|c| c.cast()))]);
            }
        }
        // The selection moved by the gizmo, from where it was when the drag
        // started: what the arguments are again for each event of it.
        CanvasEvent::Gizmo { drag, .. } => {
            let points = moved(args, &picked(args, selection));
            let drags: Vec<(PointId, V3)> = points
                .into_iter()
                .map(|p| {
                    let at = args.sketch.points[&p].at.map(|c| c.cast::<S>());
                    (p, drag.apply(&at).map(|c| c.cast()))
                })
                .collect();
            solve(before, args, &drags);
        }
        CanvasEvent::Key { key } => match key.as_str() {
            "Escape" if !s.placed.is_empty() => finish(before, args, s),
            "Escape" => take_up(before, args, s, Tool::Select),
            "Enter" => finish(before, args, s),
            "Delete" | "Backspace" => remove_selected(before, args, selection),
            key => {
                if let Some(&(tool, ..)) = Tool::ALL.iter().find(|t| t.3 == key) {
                    take_up(before, args, s, tool);
                }
            }
        },
        _ => {}
    }
}

// ── what the form shows ─────────────────────────────────────────────────────

/// What the tool in hand asks for next.
fn hint(tool: Tool, placed: usize) -> &'static str {
    match (tool, placed) {
        (Tool::Select, _) => {
            "Select points and curves to constrain them or move them with the gizmo, or drag points"
        }
        (Tool::Point, _) => "Click to place a point: on the part, or in space",
        (Tool::Line, 0) => "Click where the line starts",
        (Tool::Line, _) => "Click the next point · Esc ends the chain",
        (Tool::Arc, 0) => "Click where the arc starts",
        (Tool::Arc, 1) => "Click a point it passes through",
        (Tool::Arc, _) => "Click where it ends",
        (Tool::Spline, _) => "Click the points it passes through · double-click or Enter finishes",
    }
}

/// The world axis `direction` is exactly, if it is one.
fn axis_of(direction: &V3) -> Option<Coordinate> {
    Coordinate::ALL.into_iter().find(|axis| {
        (0..3).all(|k| {
            let want = if k == axis.index() { 1.0 } else { 0.0 };
            direction[k].to_f64() == want
        })
    })
}

/// How the constraint `id` reads in a list.
fn describe(args: &AddSketch3dArgs, id: ConstraintId, c: &Constraint3d<Design>) -> String {
    use Constraint3d::*;
    let direction = |direction: &V3| match args.reference_of(Target::Direction { constraint: id }) {
        Some(r) => r.entity.label(),
        None => match axis_of(direction) {
            Some(axis) => axis.name().to_string(),
            None => "a direction".to_string(),
        },
    };
    let end = |e: End| match e {
        End::Start => "start",
        End::End => "end",
    };
    match c {
        Coincident { a, b } => format!("Coincident {a} {b}"),
        Coordinate { point, axis, .. } => format!("{} of {point}", axis.name()),
        Distance { a, b, .. } => format!("Distance {a} {b}"),
        Length { line, .. } => format!("Length {line}"),
        Radius { arc, .. } => format!("Radius {arc}"),
        Parallel { a, b } => format!("Parallel {a} {b}"),
        ParallelTo { line, direction: d } => format!("{line} along {}", direction(d)),
        TangentTo {
            curve,
            end: e,
            direction: d,
        } => format!("{curve} {} along {}", end(*e), direction(d)),
        OnCurve { point, curve } => match args.reference_of(Target::Curve { curve: *curve }) {
            Some(r) => format!("{point} on {}", r.entity.label()),
            None => format!("{point} on {curve}"),
        },
        Tangent { a, b } => format!("Tangent {a} {b}"),
    }
}

/// The constraints the selection `picks` can take, each to add: its name,
/// label and what it adds.
fn constraints_for(
    args: &AddSketch3dArgs,
    picks: &Picks,
) -> Vec<(&'static str, &'static str, Vec<Constraint3d<Design>>)> {
    let sketch = &args.sketch;
    let kind = |c: &CurveId| &sketch.curves[c].kind;
    let lines: Vec<CurveId> = picks
        .curves
        .iter()
        .copied()
        .filter(|c| matches!(kind(c), CurveKind3d::Line { .. }))
        .collect();
    let at = |p: &PointId| sketch.points[p].at;
    let dist = |a: &V3, b: &V3| b.sub(a).norm();
    let (np, nc) = (picks.points.len(), picks.curves.len());
    let mut out = Vec::new();
    if np == 2 && nc == 0 {
        let (a, b) = (picks.points[0], picks.points[1]);
        out.push((
            "coincident",
            "Coincident",
            vec![Constraint3d::Coincident { a, b }],
        ));
        out.push((
            "distance",
            "Distance",
            vec![Constraint3d::Distance {
                a,
                b,
                value: dist(&at(&a), &at(&b)),
            }],
        ));
    }
    if np >= 1 && nc == 0 {
        let fixes = picks
            .points
            .iter()
            .filter(|p| !sketch.points[p].fixed)
            .flat_map(|&point| {
                Coordinate::ALL.map(|axis| Constraint3d::Coordinate {
                    point,
                    axis,
                    value: at(&point)[axis.index()],
                })
            })
            .collect::<Vec<_>>();
        if !fixes.is_empty() {
            out.push(("fix", "Fix", fixes));
        }
    }
    if np == 0 && nc == 1 {
        let c = picks.curves[0];
        match kind(&c) {
            CurveKind3d::Line { start, end } => out.push((
                "length",
                "Length",
                vec![Constraint3d::Length {
                    line: c,
                    value: dist(&at(start), &at(end)),
                }],
            )),
            CurveKind3d::Arc {
                start,
                through,
                end,
            } => {
                let arc = geop_core_sketch::space::Arc3 {
                    s: at(start),
                    m: at(through),
                    e: at(end),
                };
                if let Ok(value) = arc.radius() {
                    out.push((
                        "radius",
                        "Radius",
                        vec![Constraint3d::Radius { arc: c, value }],
                    ));
                }
            }
            _ => {}
        }
    }
    if np == 0 && lines.len() == 2 {
        out.push((
            "parallel",
            "Parallel",
            vec![Constraint3d::Parallel {
                a: lines[0],
                b: lines[1],
            }],
        ));
    }
    if np == 0 && nc == 2 {
        let (a, b) = (picks.curves[0], picks.curves[1]);
        if sketch.shared_end(a, b).ok().flatten().is_some() {
            out.push(("tangent", "Tangent", vec![Constraint3d::Tangent { a, b }]));
        }
    }
    if np == 1
        && nc == 1
        && matches!(
            kind(&picks.curves[0]),
            CurveKind3d::Line { .. } | CurveKind3d::Arc { .. }
        )
    {
        out.push((
            "on_curve",
            "On curve",
            vec![Constraint3d::OnCurve {
                point: picks.points[0],
                curve: picks.curves[0],
            }],
        ));
    }
    for (name, label, axis) in [
        ("along_x", "Along x", Coordinate::X),
        ("along_y", "Along y", Coordinate::Y),
        ("along_z", "Along z", Coordinate::Z),
    ] {
        let mut direction = V3::zero();
        direction[axis.index()] = Design::ONE;
        if let Some(c) = along(args, picks, direction) {
            out.push((name, label, vec![c]));
        }
    }
    out
}

/// What makes the selection `picks` run along `direction`, if it can: a
/// line parallel to it, or an arc or a spline with one of its end points
/// leaving or arriving along it.
fn along(args: &AddSketch3dArgs, picks: &Picks, direction: V3) -> Option<Constraint3d<Design>> {
    let sketch = &args.sketch;
    let [curve] = picks.curves[..] else {
        return None;
    };
    match (&sketch.curves[&curve].kind, &picks.points[..]) {
        (CurveKind3d::Line { .. }, []) => Some(Constraint3d::ParallelTo {
            line: curve,
            direction,
        }),
        (CurveKind3d::Arc { .. } | CurveKind3d::Spline { .. }, [p]) => {
            let (start, end) = sketch.curves[&curve].endpoints()?;
            let end = if same(args, *p, start) {
                End::Start
            } else if same(args, *p, end) {
                End::End
            } else {
                return None;
            };
            Some(Constraint3d::TangentTo {
                curve,
                end,
                direction,
            })
        }
        _ => None,
    }
}

/// How the sketch stands, in a line.
fn status(checked: &Result<(AddSketch3dArgs, Solve3dReport<Design>), String>) -> (String, Tone) {
    let (args, report) = match checked {
        Ok(checked) => checked,
        Err(e) => return (e.clone(), Tone::Error),
    };
    let chains = match args.sketch.chains() {
        Ok(chains) if chains.is_empty() => "nothing drawn yet".to_string(),
        Ok(chains) => format!(
            "{} chain{}",
            chains.len(),
            if chains.len() == 1 { "" } else { "s" }
        ),
        Err(e) => e.root_message().to_string(),
    };
    let plural = |n: usize| if n == 1 { "" } else { "s" };
    if !report.converged {
        let n = report.failed_constraints.len();
        (
            format!("{n} constraint{} conflict · {chains}", plural(n)),
            Tone::Error,
        )
    } else if report.dof == 0 {
        (format!("Fully constrained · {chains}"), Tone::Success)
    } else {
        (
            format!(
                "{} degree{} of freedom · {chains}",
                report.dof,
                plural(report.dof)
            ),
            Tone::Hint,
        )
    }
}

/// What the sketch draws: its curves and points, styled by how free they
/// are, and what the tool in hand would draw next.
fn visuals<S: Scalar>(
    args: &AddSketch3dArgs,
    s: &Sketch3dSession,
    report: Option<&Solve3dReport<Design>>,
) -> Vec<Visual<S>> {
    let sketch = &args.sketch;
    let cast = |v: &V3| v.map(|c| c.cast::<S>());
    let failed: Vec<&Constraint3d<Design>> = report
        .map(|r| {
            r.failed_constraints
                .iter()
                .filter_map(|k| sketch.constraints.get(k))
                .collect()
        })
        .unwrap_or_default();
    let free = |p: &PointId| report.is_none_or(|r| r.free_points.get(p).copied().unwrap_or(true));
    let mut out = Vec::new();
    for (&id, curve) in sketch.curves.iter().filter(|(_, c)| c.is_drawn()) {
        let Ok(polyline) = sketch.curve_polyline(id) else {
            continue;
        };
        let style = if failed.iter().any(|k| k.curves().contains(&id)) {
            Style::Failed
        } else if curve.construction {
            Style::Construction
        } else if curve.points().iter().any(free) {
            Style::Free
        } else {
            Style::Fixed
        };
        out.push(
            Visual::new(
                id.to_string(),
                Shape::Polyline {
                    points: polyline.iter().map(cast).collect(),
                },
                style,
            )
            .selectable(),
        );
    }
    for (&id, p) in &sketch.points {
        let style = if p.fixed {
            Style::Reference
        } else if failed.iter().any(|k| k.points().contains(&id)) {
            Style::Failed
        } else if free(&id) {
            Style::Free
        } else {
            Style::Fixed
        };
        let visual =
            Visual::new(id.to_string(), Shape::Point { at: cast(&p.at) }, style).selectable();
        out.push(if p.fixed { visual } else { visual.draggable() });
    }
    if let Some(hover) = &s.hover {
        let mut draft: Vec<V3> = match s.tool(args) {
            Tool::Spline | Tool::Arc => s.placed.iter().map(|p| sketch.points[p].at).collect(),
            _ => s
                .placed
                .last()
                .map(|p| sketch.points[p].at)
                .into_iter()
                .collect(),
        };
        draft.push(*hover);
        if draft.len() > 1 {
            out.push(Visual::new(
                "draft",
                Shape::Polyline {
                    points: draft.iter().map(cast).collect(),
                },
                Style::Draft,
            ));
        }
        out.push(Visual::new(
            "snap",
            Shape::Point { at: cast(hover) },
            Style::Snap,
        ));
    }
    out
}

/// The form of a 3-D sketch step on `before` (see the module docs).
pub(crate) fn form<'a, S: Scalar>(
    before: &'a Part<S>,
    args: &AddSketch3dArgs,
    s: &Sketch3dSession,
    selection: &[String],
) -> Form<'a, S, AddSketch3dArgs, Sketch3dSession> {
    let mut f = Form::new();
    let tool = s.tool(args);
    f.tool = if tool.draws() {
        InHand::Clicks
    } else {
        InHand::Nothing
    };
    let mut tools: Vec<Action> = Tool::ALL
        .iter()
        .map(|&(t, name, label, _)| {
            // The planar sketch's icons; selecting has none, and shows
            // its label.
            let action = Action::new(name, label).group("Draw").active(t == tool);
            if t == Tool::Select {
                action
            } else {
                action.icon(name)
            }
        })
        .collect();
    let finishing = if tool == Tool::Spline {
        "Finish the spline"
    } else {
        "End what is drawn"
    };
    tools.push(
        Action::new("finish", "Finish")
            .title(finishing)
            .group("Draw"),
    );
    f.actions("tools", tools, move |edit, value| {
        let Edit { args, session, .. } = edit;
        match value {
            "finish" => finish(before, args, session),
            name => {
                if let Some(&(t, ..)) = Tool::ALL.iter().find(|t| t.1 == name) {
                    take_up(before, args, session, t);
                }
            }
        }
    });
    f.text("hint", hint(tool, s.placed.len()), Tone::Hint);

    // The coordinates of the one point selected, to move it; else, with a
    // tool in hand, of the next point, to place it.
    let picks = picked(args, selection);
    match (&picks.points[..], &picks.curves[..]) {
        ([p], []) if !args.sketch.points[p].fixed => {
            let at = args.sketch.points[p].at;
            let p = *p;
            for axis in Coordinate::ALL {
                f.number(
                    axis.name(),
                    Number::new(axis.name(), at[axis.index()].to_f64(), Unit::Length),
                    move |args, v| {
                        let mut to = args.sketch.points[&p].at;
                        to[axis.index()] = Design::from_f64(v);
                        solve(before, args, &[(p, to)]);
                    },
                );
            }
        }
        _ if tool.draws() => {
            for axis in Coordinate::ALL {
                let number = Number::new(axis.name(), s.typed[axis.index()], Unit::Length);
                f.dialog
                    .push(axis.name(), geop_ops::ui::Control::Number(number));
                f.on(axis.name(), move |edit, value| {
                    if let Value::Number(v) = value {
                        edit.session.typed[axis.index()] = v;
                    }
                });
            }
            f.actions(
                "place",
                vec![
                    Action::new("place", "Place point")
                        .title("Place a point at the coordinates typed"),
                ],
                move |edit, _| {
                    let Edit { args, session, .. } = edit;
                    let at = Vector3::from_array(session.typed.map(Design::from_f64));
                    let tool = session.tool(args);
                    click(before, args, session, tool, Located::Free(at));
                },
            );
        }
        _ => {}
    }

    // What the selection can be constrained by.
    let available = constraints_for(args, &picks);
    if !available.is_empty() {
        let actions = available
            .iter()
            .map(|(name, label, _)| Action::new(*name, *label).group("Constrain"))
            .collect();
        f.actions("constrain", actions, move |edit, value| {
            let Edit {
                args, selection, ..
            } = edit;
            let picks = picked(args, selection);
            if let Some((_, _, constraints)) = constraints_for(args, &picks)
                .into_iter()
                .find(|(name, ..)| *name == value)
            {
                for c in constraints {
                    args.sketch.constrain(c);
                }
                selection.clear();
                solve(before, args, &[]);
            }
        });
    }
    if along(args, &picks, V3::from_array([Design::ONE; 3])).is_some() {
        f.reference(
            "along",
            "Along an axis or edge",
            Vec::new(),
            &[Role::Line],
            None,
            false,
            move |edit, entities| {
                let Edit {
                    args, selection, ..
                } = edit;
                let [entity] = &entities[..] else {
                    return;
                };
                let Some(line) = Aspects::of(entity, before).ok().and_then(|a| a.line) else {
                    return;
                };
                let picks = picked(args, selection);
                let direction = line.direction.map(|c| c.cast());
                if let Some(c) = along(args, &picks, direction) {
                    let constraint = args.sketch.constrain(c);
                    args.references.push(Reference3d {
                        entity: entity.clone(),
                        target: Target::Direction { constraint },
                    });
                    selection.clear();
                    solve(before, args, &[]);
                }
            },
        );
        f.optional("along");
    }

    let checked = checked(before, args);
    let (text, tone) = status(&checked);
    f.text("status", text, tone);

    let items = args
        .sketch
        .constraints
        .iter()
        .map(|(&id, c)| {
            let mut item = ListItem::new(id.to_string(), describe(args, id, c));
            item.removable = true;
            item.value = c.value().map(|v| v.to_f64());
            if let Ok((_, report)) = &checked
                && report.failed_constraints.contains(&id)
            {
                item.tone = Tone::Error;
            }
            item
        })
        .collect();
    f.list("constraints", items, "No constraints yet");
    for &id in args.sketch.constraints.keys() {
        f.on(id.to_string(), move |edit, value| {
            let args = edit.args;
            match value {
                Value::Remove => {
                    args.sketch.remove(&[], &[], &[id]);
                    solve(before, args, &[]);
                }
                Value::Number(v) => {
                    if let Some(c) = args.sketch.constraints.get_mut(&id) {
                        c.set_value(Design::from_f64(v));
                        solve(before, args, &[]);
                    }
                }
                _ => {}
            }
        });
    }
    f.visuals = visuals(args, s, checked.as_ref().ok().map(|(_, r)| r));
    // What is selected is moved by a gizmo at its middle: along an axis,
    // in a plane, or freely.
    let moving = moved(args, &picks);
    if tool == Tool::Select && !moving.is_empty() {
        let sum = moving
            .iter()
            .fold(V3::zero(), |sum, p| sum.add(&args.sketch.points[p].at));
        let n = Design::from_f64(moving.len() as f64);
        if let Ok(inverse) = Design::ONE.div(n) {
            let middle = sum.prod_scalar(inverse);
            f.gizmo = Some(Gizmo::new(middle.map(|c| c.cast())).translate());
        }
    }
    f
}
