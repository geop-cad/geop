//! Editing a sketch step: choosing its plane, then drawing in it with tools,
//! constraining what is drawn, and dragging it — every gesture decided here
//! and answered with a new sketch, which is solved before it is returned.
//!
//! Drawing snaps the way constraints say, never by moving geometry onto
//! what it is near: a point placed on an existing point — the sketch's
//! origin among them — *is* that point, one placed on a line's or arc's
//! middle is constrained there ([`Constraint::Midpoint`]), one placed on a
//! curve is constrained onto it ([`Constraint::PointOnCurve`]), and a line
//! drawn nearly horizontal or vertical gets that constraint. Dragging a
//! point snaps it onto points and middles the same way. Proximity only
//! suggests; the constraint is what the sketch then holds — and holding
//! shift turns the suggestions off.
//!
//! Every kind of constraint is a tool of its own (see
//! [`crate::constraints`]): pressed with a selection that is all it needs,
//! it is added at once; otherwise the tool is taken up, and what is clicked
//! next is selected for it until it has all it needs.
//!
//! The trim tool removes what it is clicked on or dragged across, up to
//! where the sketch meets it (see [`trim`]). Mirror, patterns and offset
//! make geometry from what is selected, tied to it by constraints (see
//! [`modify`]).

mod dialog;
mod drawing;
mod gestures;
mod modify;
mod snap;
#[cfg(test)]
mod tests;
mod trim;
mod visuals;

use dialog::draw_dialog;
use snap::Snap;
use visuals::visuals;

use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{ConstraintId, CurveId, PointId, offset::Corners};
use geop_ops::{
    Design, Part,
    operation::Role,
    ui::{
        Action, Button, CanvasEvent, Edit, Form, InHand, ListItem, Pointer, Shape, Style, Tone,
        Value, Visual, hit::hit_visuals,
    },
};

use crate::{
    AddSketchArgs, Constraint, CurveKind, Sketch,
    constraints::{self, ConstraintTool, Pick},
    geometry::{P2, add, dist, loop_polyline, polyline, sub, xy},
};

type SolveReport = geop_core_sketch::SolveReport<Design>;

/// A drawing tool: what clicks in the plane make.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DrawTool {
    Line,
    Rectangle,
    CenterRectangle,
    ThreePointRectangle,
    Circle,
    ThreePointCircle,
    Arc,
    CenterArc,
    TangentArc,
    Polygon,
    Slot,
    ArcSlot,
    Spline,
    Point,
    Fillet,
    Chamfer,
}

/// What the palette shows of a tool.
pub struct DrawInfo<T = DrawTool> {
    pub tool: T,
    /// Its name in events and the icon it is shown as.
    pub name: &'static str,
    pub label: &'static str,
    pub shortcut: Option<&'static str>,
}

impl DrawTool {
    pub const ALL: [DrawInfo; 16] = {
        use DrawTool::*;
        const fn t(
            tool: DrawTool,
            name: &'static str,
            label: &'static str,
            shortcut: Option<&'static str>,
        ) -> DrawInfo {
            DrawInfo {
                tool,
                name,
                label,
                shortcut,
            }
        }
        [
            t(
                Line,
                "line",
                "Line — move back onto its end for a tangent arc",
                Some("l"),
            ),
            t(Rectangle, "rectangle", "Corner rectangle", Some("r")),
            t(
                CenterRectangle,
                "center_rectangle",
                "Center rectangle",
                None,
            ),
            t(
                ThreePointRectangle,
                "three_point_rectangle",
                "3-point rectangle",
                None,
            ),
            t(Circle, "circle", "Center circle", Some("c")),
            t(
                ThreePointCircle,
                "three_point_circle",
                "3-point circle",
                None,
            ),
            t(Arc, "arc", "3-point arc", Some("a")),
            t(CenterArc, "center_arc", "Center-point arc", None),
            t(TangentArc, "tangent_arc", "Tangent arc", Some("t")),
            t(Polygon, "polygon", "Polygon", Some("g")),
            t(Slot, "slot", "Slot", None),
            t(ArcSlot, "arc_slot", "Arc slot", None),
            t(Spline, "spline", "Spline", Some("s")),
            t(Point, "point", "Point", Some("p")),
            t(Fillet, "fillet", "Fillet a corner", Some("f")),
            t(Chamfer, "chamfer", "Chamfer a corner", None),
        ]
    };

    pub fn info(self) -> &'static DrawInfo {
        Self::ALL
            .iter()
            .find(|i| i.tool == self)
            .expect("every tool is listed")
    }

    fn by_name(name: &str) -> Option<DrawTool> {
        Self::ALL.iter().find(|i| i.name == name).map(|i| i.tool)
    }

    /// How many points it takes: `None` for a spline, which takes points
    /// until it is finished.
    fn needs(self) -> Option<usize> {
        use DrawTool::*;
        match self {
            Point | Fillet | Chamfer => Some(1),
            Line | Rectangle | CenterRectangle | Circle | TangentArc | Polygon => Some(2),
            ThreePointRectangle | ThreePointCircle | Arc | CenterArc | Slot => Some(3),
            ArcSlot => Some(4),
            Spline => None,
        }
    }

    /// Whether it changes what is drawn rather than drawing anew.
    fn modifies(self) -> bool {
        matches!(self, DrawTool::Fillet | DrawTool::Chamfer)
    }
}

/// A tool that makes geometry from what is selected, tied to it by
/// constraints (see [`modify`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModifyTool {
    Offset,
    Mirror,
    LinearPattern,
    CircularPattern,
}

impl ModifyTool {
    pub const ALL: [DrawInfo<ModifyTool>; 4] = {
        use ModifyTool::*;
        [
            DrawInfo {
                tool: Offset,
                name: "offset",
                label: "Offset — click curves for their chains, then where the offset goes",
                shortcut: Some("o"),
            },
            DrawInfo {
                tool: Mirror,
                name: "mirror",
                label: "Mirror — click the line to mirror across",
                shortcut: None,
            },
            DrawInfo {
                tool: LinearPattern,
                name: "linear_pattern",
                label: "Linear pattern of what is selected — click a line for its direction",
                shortcut: None,
            },
            DrawInfo {
                tool: CircularPattern,
                name: "circular_pattern",
                label: "Circular pattern of what is selected — click its center",
                shortcut: None,
            },
        ]
    };

    fn by_name(name: &str) -> Option<ModifyTool> {
        Self::ALL.iter().find(|i| i.name == name).map(|i| i.tool)
    }

    /// Whether it starts from a selection made beforehand.
    fn needs_selection(self) -> bool {
        matches!(
            self,
            ModifyTool::LinearPattern | ModifyTool::CircularPattern
        )
    }
}

/// What a click in the plane does.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tool {
    /// Selecting and dragging.
    #[default]
    Select,
    Draw(DrawTool),
    /// Picking what a constraint is added to.
    Constrain(ConstraintTool),
    /// Removing what is clicked or dragged across, up to where the sketch
    /// meets it.
    Trim,
    /// Making geometry from what is selected.
    Modify(ModifyTool),
}

/// The trim tool's shortcut.
const TRIM_SHORTCUT: &str = "m";

/// Where the trim tool meets the sketch: with no button held, under the
/// pointer; dragged, along the way the pointer went.
#[derive(Clone, Debug, Default, PartialEq)]
struct Stroke {
    /// Where the pointer went, in the plane, while dragged.
    path: Vec<P2>,
    /// Every curve met, and a point of the plane where it was met.
    met: Vec<(CurveId, P2)>,
}

/// A point placed for what is being drawn: where, and what it snapped to.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Placed {
    at: P2,
    snap: Option<Snap>,
}

/// What is being drawn: the points placed for it so far, and what the
/// tools that need more remember.
#[derive(Clone, Debug, Default, PartialEq)]
struct Draft {
    placed: Vec<Placed>,
    /// A chain of lines: the curve it last added, which a tangent arc
    /// continues.
    previous: Option<CurveId>,
    /// A chain of lines: draws a tangent arc next rather than a line.
    arc: bool,
    /// A chain of lines: whether the pointer is over its end — coming back
    /// there switches between a line and a tangent arc.
    at_end: bool,
    /// A center-point arc: how far it has turned so far, followed round as
    /// the pointer goes, so it can turn past half a circle.
    sweep: f64,
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
    /// A dimension's value: where it was shown when grabbed, as an offset
    /// from what it measures, and where it was grabbed.
    Label {
        id: ConstraintId,
        offset: P2,
        grab: P2,
    },
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
#[derive(Clone, Debug, PartialEq)]
pub struct SketchSession {
    tool: Tool,
    draft: Draft,
    /// The pointer, in the plane — where it snaps to, while drawing.
    cursor: Option<P2>,
    /// What the pointer snaps to, drawing or dragging.
    snap: Option<Snap>,
    drag: Option<Drag>,
    /// Solved once when the edit starts, and after every change.
    solved: Option<Solved>,
    /// Whether what is drawn is construction geometry.
    construction: bool,
    /// How many sides a polygon gets.
    sides: usize,
    /// The dimension whose value is being typed, in place.
    prompt: Option<ConstraintId>,
    /// Why the last value typed was refused.
    error: Option<String>,
    /// What the trim tool meets.
    stroke: Stroke,
    /// Whether a polygon's sides touch its circle rather than its corners
    /// lying on it.
    circumscribed: bool,
    /// The options of the tools that make geometry from geometry.
    modify: ModifyOptions,
}

/// What mirror, patterns and offset are set to make.
#[derive(Clone, Debug, PartialEq)]
struct ModifyOptions {
    /// The line mirrored across, once clicked.
    mirror_line: Option<CurveId>,
    /// How many copies a pattern has, the original among them.
    count: usize,
    /// A linear pattern's spacing: when the tool is taken up, as far as
    /// what is selected reaches, and half as far again.
    spacing: f64,
    /// A circular pattern's angle between copies, in degrees; none to
    /// spread them over the whole circle.
    pitch: Option<f64>,
    /// An offset to both sides.
    both: bool,
    corners: Corners,
}

impl Default for ModifyOptions {
    fn default() -> Self {
        Self {
            mirror_line: None,
            count: 3,
            spacing: 1.0,
            pitch: None,
            both: false,
            corners: Corners::Round,
        }
    }
}

impl Default for SketchSession {
    fn default() -> Self {
        Self {
            tool: Tool::Select,
            draft: Draft::default(),
            cursor: None,
            snap: None,
            drag: None,
            solved: None,
            construction: false,
            sides: 6,
            prompt: None,
            error: None,
            stroke: Stroke::default(),
            circumscribed: false,
            modify: ModifyOptions::default(),
        }
    }
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
/// in the order they were picked, and its constraints.
fn selected(sketch: &Sketch, keys: &[String]) -> (Vec<Pick>, Vec<ConstraintId>) {
    let mut picks = Vec::new();
    let mut constraints = Vec::new();
    for key in keys {
        if let Some(p) = point_key(key).filter(|p| sketch.points.contains_key(p)) {
            picks.push(Pick::Point(p));
        } else if let Some(c) = curve_key(key).filter(|c| sketch.curves.contains_key(c)) {
            picks.push(Pick::Curve(c));
        } else if let Some(k) = glyph_key(key).filter(|k| sketch.constraints.contains_key(k)) {
            constraints.push(k);
        }
    }
    (picks, constraints)
}

/// The curves among the selection `keys` that `sketch` has, in the order
/// they were picked.
fn selected_curves(sketch: &Sketch, keys: &[String]) -> Vec<CurveId> {
    selected(sketch, keys)
        .0
        .into_iter()
        .filter_map(|p| match p {
            Pick::Curve(c) => Some(c),
            Pick::Point(_) => None,
        })
        .collect()
}

/// The key of the visual among `visuals` the pointer is over, trying the
/// kinds `stages` accept in order: a point drawn on a curve is that point,
/// not the curve.
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

fn is_glyph(key: &str) -> bool {
    glyph_key(key).is_some()
}

/// The point `id` of `sketch`, as drawn.
fn pt(sketch: &Sketch, id: PointId) -> P2 {
    xy(sketch, id)
}

/// The edit in progress: what it works on, what is selected, and where the
/// plane is.
struct Editing<'a, S: Scalar> {
    before: &'a Part<S>,
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
        .map(|&(p, at)| (p, Vector2::from_array(at.map(Design::from_f64))))
        .collect();
    s.solved = Some(match sketch.solve_with_drag(&drags) {
        Ok(report) => Solved::Solved { report },
        Err(e) => Solved::Failed {
            error: e.root_message().to_string(),
        },
    });
}

/// `args` brought up to date with the part `before`, in `frame`: its own
/// origin and axes there, what it projects where that is now, its formulas
/// evaluated — and solved again if that changed it, or if it was not
/// solved yet, recording how that went in `s`. What cannot be brought up
/// to date stays as it was: the step says why when it is built.
fn refresh<S: Scalar>(
    args: &mut AddSketchArgs,
    s: &mut SketchSession,
    before: &Part<S>,
    frame: &CoordinateSystem<S>,
) {
    let drawn = args.sketch.clone();
    args.ensure_frame();
    let _ = args.update_references(before, frame);
    let inputs = before.inputs();
    let _ = args.apply_formulas(|formula| {
        geop_ops::parameters::evaluate(formula, |name| geop_ops::parameters::number(inputs, name))
    });
    if s.solved.is_none() || args.sketch != drawn {
        solve(&mut args.sketch, s, &[]);
    }
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
                edit.session.draft = Draft::default();
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
    let mut args = args.clone();
    let mut s = s.clone();
    refresh(&mut args, &mut s, before, &frame);
    draw_dialog(&mut f, before, &args, &s, selection, &frame);
    f.visuals = visuals(&args, &s, selection, &frame);
    f.focus = Some(frame);
    f.tool = match s.tool {
        Tool::Select => InHand::Nothing,
        Tool::Trim => InHand::Strokes,
        Tool::Draw(_) | Tool::Constrain(_) | Tool::Modify(_) => InHand::Clicks,
    };
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

/// `f` applied to the drawing, brought up to date with the part — and so
/// solved — first. Nothing, while the plane is none.
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
    refresh(args, s, before, &frame);
    f(&mut Editing {
        before,
        args,
        s,
        selection,
        frame,
    });
}
