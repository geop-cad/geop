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
use geop_core_sketch::{ConstraintId, CurveId, PointId};
use geop_ops::{
    Design, Part,
    operation::Role,
    ui::{
        Action, Button, CanvasEvent, Edit, Form, ListItem, Pointer, Shape, Style, Tone, Value,
        Visual, hit::hit_visuals,
    },
};

use crate::{
    AddSketchArgs, Constraint, CurveKind, Sketch,
    constraints::{self, Selection},
    geometry::{P2, add, design, dist, loop_polyline, polyline, sub, sweep_through, xy},
};

type SolveReport = geop_core_sketch::SolveReport<Design>;

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
    /// Every tool, as the palette offers it: `(tool, name, label, shortcut)`.
    const ALL: [(Tool, &'static str, &'static str, &'static str); 7] = [
        (Tool::Select, "select", "Select", "Escape"),
        (Tool::Line, "line", "Line", "l"),
        (Tool::Rectangle, "rectangle", "Rectangle", "r"),
        (Tool::Arc, "arc", "Arc", "a"),
        (Tool::Circle, "circle", "Circle", "c"),
        (Tool::Spline, "spline", "Spline", "s"),
        (Tool::Point, "point", "Point", "p"),
    ];

    fn by_name(name: &str) -> Option<Tool> {
        Tool::ALL.iter().find(|t| t.1 == name).map(|t| t.0)
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

/// The temporary state of editing a sketch. What is selected is the
/// editor's: the keys of the visuals of the points, curves and constraints
/// selected.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SketchSession {
    tool: Tool,
    draft: Option<Draft>,
    /// The pointer, in the plane.
    cursor: Option<P2>,
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

/// What the selection `keys` hold that `sketch` has: its points and curves,
/// and its constraints.
fn selected(sketch: &Sketch, keys: &[String]) -> (Selection, Vec<ConstraintId>) {
    let mut selection = Selection::default();
    let mut constraints = Vec::new();
    for key in keys {
        if let Some(p) = point_key(key).filter(|p| sketch.points.contains_key(p)) {
            selection.points.push(p);
        } else if let Some(c) = curve_key(key).filter(|c| sketch.curves.contains_key(c)) {
            selection.curves.push(c);
        } else if let Some(k) = glyph_key(key).filter(|k| sketch.constraints.contains_key(k)) {
            constraints.push(k);
        }
    }
    (selection, constraints)
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
        hit_visuals(visuals, pointer, None, |v| accept(&v.key)).map(|h| h.visual.key.clone())
    })
}

fn is_point(key: &str) -> bool {
    point_key(key).is_some()
}

fn is_curve(key: &str) -> bool {
    curve_key(key).is_some()
}

fn is_origin(key: &str) -> bool {
    key == "origin"
}

/// The point `id` of `sketch`, as drawn.
fn pt(sketch: &Sketch, id: PointId) -> P2 {
    xy(sketch, id)
}

/// The edit in progress: what it works on, what is selected, and where the
/// plane is.
struct Editing<'a, S: Scalar> {
    args: &'a mut AddSketchArgs,
    s: &'a mut SketchSession,
    selection: &'a mut Vec<String>,
    frame: CoordinateSystem<S>,
}

/// `p` of the sketch, in `frame`'s plane.
fn to_world<S: Scalar>(frame: &CoordinateSystem<S>, p: P2) -> Vector3<S> {
    frame.uv_to_xyz(&Vector2::from_array(p.map(S::from_f64)))
}

/// Solves `sketch`, pulling `drags` towards their targets, and records how
/// that went. A sketch the solver rejects outright stays as drawn.
fn solve(sketch: &mut Sketch, s: &mut SketchSession, drags: &[(PointId, P2)]) {
    let drags: Vec<_> = drags
        .iter()
        .map(|&(p, at)| (p, Vector2::from_array(at.map(design))))
        .collect();
    s.solved = Some(match sketch.solve_with_drag(&drags) {
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

    /// `p`, a point of the plane, in the sketch's coordinates.
    fn to_sketch(&self, p: &Vector3<S>) -> P2 {
        let uvw = self.frame.to_uvw(p);
        [uvw[0].to_f64(), uvw[1].to_f64()]
    }
}

/// What a sketch step shows: the plane to pick and, once it is one, the
/// sketch drawn in it with the tools, constraints and state of the drawing.
pub(crate) fn form<'a, S: Scalar>(
    before: &'a Part<S>,
    args: &AddSketchArgs,
    s: &SketchSession,
    selection: &[String],
) -> Form<'a, S, AddSketchArgs, SketchSession> {
    let mut s = s.clone();
    if s.solved.is_none() {
        solve(&mut args.sketch.clone(), &mut s, &[]);
    }
    let mut f = Form::<S, AddSketchArgs, SketchSession>::new();
    // Only a plane is taken; another one moves the drawing onto it.
    f.reference(
        "plane",
        "plane",
        args.plane.iter().cloned().collect(),
        &[Role::Plane],
        None,
        false,
        move |edit, picked| {
            if let [plane] = picked.as_slice()
                && plane.resolve_plane(before).is_ok()
            {
                edit.args.plane = Some(plane.clone());
                edit.session.draft = None;
            }
        },
    );
    let Some(plane) = &args.plane else {
        return f;
    };
    let frame = match plane.resolve_plane(before) {
        Ok(frame) => frame,
        Err(e) => {
            f.text("plane_error", e.root_message(), Tone::Error);
            return f;
        }
    };
    draw_dialog(&mut f, before, &args.sketch, &s, selection);
    f.visuals = visuals(&args.sketch, &s, &frame);
    f.focus = Some(frame);
    f.tool = s.tool != Tool::Select;
    f
}

/// A pointer or key event in the drawing.
pub(crate) fn event<S: Scalar>(
    before: &Part<S>,
    edit: Edit<'_, AddSketchArgs, SketchSession>,
    event: &CanvasEvent<S>,
) {
    editing(before, edit, |e| e.event(event));
}

/// `f` applied to the drawing, solved first if it is not yet. Nothing, while
/// the plane is none.
fn editing<S: Scalar>(
    before: &Part<S>,
    edit: Edit<'_, AddSketchArgs, SketchSession>,
    f: impl FnOnce(&mut Editing<S>),
) {
    let Edit {
        args,
        session: s,
        selection,
        ..
    } = edit;
    let Some(Ok(frame)) = args.plane.as_ref().map(|p| p.resolve_plane(before)) else {
        return;
    };
    if s.solved.is_none() {
        solve(&mut args.sketch, s, &[]);
    }
    f(&mut Editing {
        args,
        s,
        selection,
        frame,
    });
}
