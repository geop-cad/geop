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

mod dialog;
mod gestures;
#[cfg(test)]
mod tests;
mod visuals;

use dialog::draw_dialog;
use visuals::visuals;

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{
    Constraint, ConstraintId, CurveId, CurveKind, PointId, Sketch, SolveReport,
    point::{P2, add, dist, sub},
    profile::curve_polyline,
};
use geop_ops::{
    Part,
    ui::{
        Button, ButtonItem, Dialog, Event, Form, ListItem, Pointer, Shape, Style, Target, Tone,
        Value, Visual, hit::hit_visuals,
    },
};

use crate::{
    AddSketchArgs,
    constraints::{self, Selection},
    geometry::sweep_through,
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
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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
#[derive(Clone, Debug, PartialEq)]
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
#[derive(Clone, Debug, PartialEq)]
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
#[derive(Clone, Debug, PartialEq)]
enum Solved {
    Solved {
        report: SolveReport,
    },
    /// The sketch could not be solved at all — it is kept as drawn.
    Failed {
        error: String,
    },
}

/// The temporary state of editing a sketch.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SketchSession {
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
}

/// What a sketch step shows: the plane to pick and, once it is one, the
/// sketch drawn in it with the tools, constraints and state of the drawing.
pub(crate) fn form<S: Scalar>(
    before: &Part<S>,
    args: &AddSketchArgs,
    s: &SketchSession,
) -> Form<S> {
    let mut s = s.clone();
    if s.solved.is_none() {
        solve(&mut args.sketch.clone(), &mut s, &[]);
    }
    let mut d = Dialog::new();
    d.pick(
        "plane",
        "plane",
        vec![args.plane.clone()],
        PLANE_TARGETS,
        false,
    );
    let frame = match args.plane.resolve_plane(before) {
        Ok(frame) => frame,
        Err(e) => {
            d.text("plane_error", e.root_message(), Tone::Error);
            return Form::dialog(d);
        }
    };
    draw_dialog(&mut d, &args.sketch, &s);
    let grab = s.tool == Tool::Select
        && s.hover
            .as_deref()
            .is_some_and(|k| point_key(k).is_some() || curve_key(k).is_some());
    Form {
        dialog: d,
        visuals: visuals(&args.sketch, &s, &frame),
        focus: Some(frame),
        grab,
    }
}

/// The field `key` set to `value`: a plane picked — only a plane is taken —
/// or a field of the drawing.
pub(crate) fn set<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketchArgs,
    s: &mut SketchSession,
    key: &str,
    value: Value,
) {
    match (key, value) {
        ("plane", Value::Entity(plane)) => {
            if plane.resolve_plane(before).is_ok() {
                args.plane = plane;
                s.draft = None;
            }
        }
        (key, value) => editing(before, args, s, |e| e.dialog(key, &value)),
    }
}

/// A pointer or key event in the drawing.
pub(crate) fn event<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketchArgs,
    s: &mut SketchSession,
    event: &Event<S>,
) {
    editing(before, args, s, |e| e.event(event));
}

/// `f` applied to the drawing, solved first if it is not yet. Nothing, while
/// the plane is none.
fn editing<S: Scalar>(
    before: &Part<S>,
    args: &mut AddSketchArgs,
    s: &mut SketchSession,
    f: impl FnOnce(&mut Editing<S>),
) {
    let Ok(frame) = args.plane.resolve_plane(before) else {
        return;
    };
    if s.solved.is_none() {
        solve(&mut args.sketch, s, &[]);
    }
    f(&mut Editing { args, s, frame });
}
