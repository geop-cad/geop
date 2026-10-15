//! Sketches drawn as an editor draws them: every gesture a command to the
//! [`crate::Editor`], as rays and keys.
//!
//! What the sketch tool keeps while it is edited is its own; these tests
//! read what the editor shows of it instead — the tool active among the
//! dialog's actions, the hint, the snap marker, the constraints' status —
//! and how the sketch the step holds solves.

use geop_core_editor::editing::{
    Button, Control, Pointer, Presentation, Shape, StepEditEvent, Style, Tone, Value,
};
use geop_core_editor::numeric_input::NumericInput;
use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{
    CurveId, PointId,
    geometry::Arc,
    plain::{P2, add, bounds as point_bounds, cross, distance, scale, sub, xy},
};
use geop_ops_parameters::{Parameter, Parameters};
use geop_ops_sketch::{
    AddSketchArgs, Constraint, CurveKind, Sketch, SketchPlane, editor::SketchSession, references,
};

use super::testing::{Stepping, origin_plane, pointer};
use crate::Program;

type SolveReport = geop_core_sketch::SolveReport<geop_ops_model::Design>;
type SketchEditing = geop_core_editor::edit_form::FormState<SketchSession>;

/// A sketch step being drawn as a front end draws it, with the step's
/// arguments and what the editor last showed of it.
struct Editor {
    step: Stepping,
    args: AddSketchArgs,
    presentation: Presentation<S>,
}

impl Editor {
    /// A new sketch step, in a program with the parameters `parameters` —
    /// or, not `new`, one on the origin's `xy` plane being edited again.
    fn on(parameters: Parameters, new: bool) -> Self {
        let mut program = Program::new();
        program.set_parameters(parameters);
        let step = match new {
            true => Stepping::new(program, "add_sketch"),
            false => {
                let plane = SketchPlane::Datum(origin_plane(FrameAxis::Z));
                program.push("sketch", AddSketchArgs::new(Some(plane)));
                Stepping::open(program, "sketch")
            }
        };
        let args = step.args();
        let presentation = step.presentation().clone();
        Editor {
            step,
            args,
            presentation,
        }
    }

    fn new(new: bool) -> Self {
        Self::on(Parameters::default(), new)
    }

    fn send(&mut self, event: StepEditEvent<S>) {
        self.step.send(event);
        self.args = self.step.args();
        self.presentation = self.step.presentation().clone();
    }

    fn editing(&self) -> &SketchEditing {
        self.step.editing()
    }

    fn press(&mut self, key: &str) {
        self.dialog(key, Value::Press);
    }

    fn dialog(&mut self, key: &str, value: Value) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value,
        });
    }

    /// Presses the action `value` of the field `key`.
    fn act(&mut self, key: &str, value: &str) {
        self.dialog(key, Value::Choice(value.into()));
    }

    /// The keys of the visuals selected.
    fn selection(&self) -> &[String] {
        &self.editing().selection
    }

    fn click_with(&mut self, x: f64, y: f64, double: bool, shift: bool) {
        self.send(StepEditEvent::Click {
            pointer: down(x, y),
            button: Button::Primary,
            double,
            shift,
        });
    }

    fn click(&mut self, x: f64, y: f64) {
        self.click_with(x, y, false, false);
    }

    fn hover(&mut self, x: f64, y: f64) {
        self.send(StepEditEvent::Hover {
            pointer: down(x, y),
            shift: false,
        });
    }

    fn drag(&mut self, from: P2, to: P2, shift: bool) {
        self.hover(from[0], from[1]);
        self.send(StepEditEvent::Drag {
            from: down(from[0], from[1]),
            to: down(to[0], to[1]),
            done: true,
            shift,
        });
    }

    fn key(&mut self, key: &str) {
        self.send(StepEditEvent::Key { key: key.into() });
    }

    /// A drag through `points`, as the viewer sends it: from where it went
    /// down to each point in turn, released at the last.
    fn stroke(&mut self, points: &[P2]) {
        let [first, rest @ ..] = points else {
            return;
        };
        self.hover(first[0], first[1]);
        for (i, p) in rest.iter().enumerate() {
            self.send(StepEditEvent::Drag {
                from: down(first[0], first[1]),
                to: down(p[0], p[1]),
                done: i + 1 == rest.len(),
                shift: false,
            });
        }
    }

    /// How many visuals are drawn as what the tool in hand would remove.
    fn removed(&self) -> usize {
        self.presentation
            .visuals
            .iter()
            .filter(|v| v.style == Style::Removed)
            .count()
    }

    /// Whether a visual `key` is drawn.
    fn shows(&self, key: &str) -> bool {
        self.presentation.visuals.iter().any(|v| v.key == key)
    }

    /// The tool in hand, by the name of its action in the dialog; `None`
    /// for selecting.
    fn tool(&self) -> Option<String> {
        let dialog = &self.presentation.dialog;
        let active = |key: &str| match dialog.get(key) {
            Some(Control::Actions { actions }) => (actions.iter())
                .find(|a| a.active && a.value != "construction")
                .map(|a| a.value.clone()),
            _ => None,
        };
        active("tool").or_else(|| active("constrain"))
    }

    /// Whether what is drawn next is construction geometry.
    fn construction(&self) -> bool {
        match self.presentation.dialog.get("tool") {
            Some(Control::Actions { actions }) => {
                (actions.iter()).any(|a| a.value == "construction" && a.active)
            }
            _ => false,
        }
    }

    /// The line on what to do next, and its tone: an error, for a value
    /// refused.
    fn hint(&self) -> (String, Tone) {
        match self.presentation.dialog.get("hint") {
            Some(Control::Text { text, tone }) => (text.clone(), *tone),
            other => panic!("no hint: {other:?}"),
        }
    }

    /// The line drawn from `a` to `b`, either way round.
    fn line_between(&self, a: P2, b: P2) -> Option<CurveId> {
        let s = self.sketch();
        self.drawn_curves()
            .into_iter()
            .find_map(|(id, k)| match *k {
                CurveKind::Line { start, end } => {
                    let (p, q) = (xy(s, start), xy(s, end));
                    ((close(p, a) && close(q, b)) || (close(p, b) && close(q, a))).then_some(id)
                }
                _ => None,
            })
    }

    /// Draws a polyline through `points` with the line tool, and puts the
    /// tool down.
    fn lines(&mut self, points: &[P2]) {
        self.act("tool", "line");
        for p in points {
            self.click(p[0], p[1]);
        }
        self.key("Escape");
        self.key("Escape");
    }

    fn sketch(&self) -> &Sketch {
        &self.args.sketch
    }

    /// The curves drawn — not given, like the sketch's own axes.
    fn drawn_curves(&self) -> Vec<(CurveId, &CurveKind)> {
        self.sketch()
            .curves
            .iter()
            .filter(|(_, c)| !c.fixed)
            .map(|(&id, c)| (id, &c.kind))
            .collect()
    }

    /// The points drawn — not given, like the sketch's origin.
    fn drawn_points(&self) -> Vec<PointId> {
        self.sketch()
            .points
            .iter()
            .filter(|(_, p)| !p.fixed)
            .map(|(&id, _)| id)
            .collect()
    }

    fn has(&self, pred: impl Fn(&Constraint) -> bool) -> bool {
        self.sketch().constraints.values().any(pred)
    }

    fn count(&self, pred: impl Fn(&Constraint) -> bool) -> usize {
        self.sketch()
            .constraints
            .values()
            .filter(|c| pred(c))
            .count()
    }

    /// The constraint tools the dialog offers, and whether each can be
    /// used now.
    fn constraint_tools(&self) -> Vec<(String, bool)> {
        match self.presentation.dialog.get("constrain") {
            Some(Control::Actions { actions }) => actions
                .iter()
                .map(|a| (a.value.clone(), a.enabled))
                .collect(),
            _ => Vec::new(),
        }
    }

    fn enabled(&self, tool: &str) -> bool {
        self.constraint_tools()
            .iter()
            .find(|(name, _)| name == tool)
            .unwrap_or_else(|| panic!("no tool {tool}"))
            .1
    }

    /// How the sketch the step holds solves, once the editor's own solve
    /// was no failure: the status it shows is no error.
    fn report(&self) -> SolveReport {
        let status = self.presentation.dialog.get("status");
        let Some(Control::Text { text, tone }) = status else {
            panic!("no status: {status:?}")
        };
        assert_ne!(*tone, Tone::Error, "{text}");
        self.sketch().clone().solve().unwrap()
    }

    /// The sketch's origin point.
    fn origin(&self) -> PointId {
        self.args.references[0].points[references::ORIGIN]
    }
}

fn v(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// Straight down onto the `xy` plane at `(x, y)`.
fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// Drawing on the default plane.
fn drawing() -> Editor {
    Editor::new(false)
}

fn close(a: P2, b: P2) -> bool {
    distance(a, b) < 1e-6
}

/// The arc `arc` of `s`, as solved.
fn arc_of(s: &Sketch, arc: CurveId) -> Arc<S> {
    let CurveKind::Arc { start, end, sweep } = s.curves[&arc].kind else {
        panic!("{arc} is no arc")
    };
    let at = |p: PointId| Vector2::from_array(xy(s, p).map(S::from_f64));
    Arc {
        s: at(start),
        e: at(end),
        half: S::from_f64(sweep.to_f64() / 2.0),
    }
}

fn plain(v: Vector2<S>) -> P2 {
    [v[0].to_f64(), v[1].to_f64()]
}

/// A new sketch has no plane, and starts by waiting for one to be picked;
/// picking one goes straight on to drawing on it, head on — with the
/// sketch's own origin and axes there to draw from.
#[test]
fn a_new_sketch_picks_its_plane_first() {
    let mut e = Editor::new(true);
    assert_eq!(e.args.plane, None);
    assert!(matches!(
        e.presentation.dialog.get("plane"),
        Some(Control::Reference(r)) if r.armed
    ));
    assert!(e.presentation.focus.is_none());
    assert!(
        e.presentation.pickable.iter().any(|kind| kind == "datum"),
        "{:?}",
        e.presentation.pickable
    );
    // The origin's zx plane's square, ten reaches of 0.009 from the
    // origin: seen from the front, at (0.04, 0, 0.04).
    let front = pointer([0.04, -10.0, 0.04], [0.0, 1.0, 0.0]);
    e.send(StepEditEvent::Click {
        pointer: front,
        button: Button::Primary,
        double: false,
        shift: false,
    });
    assert_eq!(
        e.args.plane,
        Some(SketchPlane::Datum(origin_plane(FrameAxis::Y)))
    );
    let focus = e
        .presentation
        .focus
        .clone()
        .expect("drawing faces the plane");
    assert_eq!(*focus.w(), v([0.0, 1.0, 0.0]));
    // The origin and both axes, given: fixed, and drawn to be selected.
    let s = e.sketch();
    assert_eq!(s.points.values().filter(|p| p.fixed).count(), 3);
    assert_eq!(
        s.curves
            .values()
            .filter(|c| c.fixed && c.construction)
            .count(),
        2
    );
    let origin = e.origin().to_string();
    assert!(e.presentation.visuals.iter().any(|v| v.key == origin));
}

/// Lines chain; one drawn nearly horizontal is constrained horizontal, one
/// started on the origin starts *at* the origin, and one ended on an
/// existing point ends at that very point.
#[test]
fn lines_snap_by_constraint() {
    let mut e = drawing();
    e.lines(&[[0.001, 0.0], [1.0, 0.02], [1.0, 1.0], [0.003, -0.002]]);
    assert_eq!(e.drawn_curves().len(), 3);
    assert_eq!(e.drawn_points().len(), 2, "the chain closed on the origin");
    let origin = e.origin();
    assert!(
        e.drawn_curves()
            .iter()
            .filter(|(_, k)| matches!(k, CurveKind::Line { start, end } if *start == origin || *end == origin))
            .count()
            == 2
    );
    assert!(e.has(|c| matches!(c, Constraint::Horizontal { .. })));
    assert!(e.has(|c| matches!(c, Constraint::Vertical { .. })));
    // Solved: horizontal and vertical hold exactly.
    let s = e.sketch();
    let corner = e
        .drawn_points()
        .iter()
        .map(|&id| xy(s, id))
        .find(|p| p[0] > 0.5 && p[1] < 0.5)
        .unwrap();
    assert!(corner[1].abs() < 1e-6, "{corner:?}");
    assert_eq!(e.tool(), None, "the drawing ended");
}

/// A point placed on a curve is constrained onto it, one on a line's
/// middle is its midpoint — and with shift held, nothing snaps.
#[test]
fn points_snap_onto_curves_and_middles() {
    let mut e = drawing();
    e.lines(&[[-0.5, 0.0], [1.5, 2.0]]);
    e.act("tool", "point");
    e.click(0.302, 0.8);
    assert!(e.has(|c| matches!(c, Constraint::PointOnCurve { .. })));
    e.click(0.504, 1.003);
    assert!(e.has(|c| matches!(c, Constraint::Midpoint { .. })));
    let before = e.sketch().constraints.len();
    e.click_with(1.0, 1.499, false, true);
    assert_eq!(e.sketch().constraints.len(), before, "shift: no snapping");
    assert_eq!(e.drawn_points().len(), 5);
}

/// Hovering where a click would snap shows where.
#[test]
fn snapping_is_shown() {
    let mut e = drawing();
    e.lines(&[[-0.5, 0.5], [1.5, 0.5]]);
    e.act("tool", "line");
    e.hover(0.503, 0.502);
    // On the line's middle.
    let snap = e.presentation.visuals.iter().find(|v| v.key == "snap");
    assert!(
        matches!(snap, Some(v) if v.style == Style::Snap
            && matches!(v.shape, Shape::Point { at } if close([at[0].to_f64(), at[1].to_f64()], [0.5, 0.5]))),
        "{snap:?}"
    );
    e.send(StepEditEvent::Hover {
        pointer: down(0.503, 0.502),
        shift: true,
    });
    assert!(!e.shows("snap"));
}

/// Two clicks make a rectangle: four lines, horizontal and vertical by
/// constraint, bounding one region — shown shaded.
#[test]
fn rectangles() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.hover(0.8, 0.6);
    assert_eq!(
        e.presentation
            .visuals
            .iter()
            .filter(|v| v.style == Style::Draft)
            .count(),
        4,
        "the rectangle is previewed"
    );
    e.click(1.2, 0.7);
    assert_eq!(e.drawn_curves().len(), 4);
    assert_eq!(e.sketch().constraints.len(), 4);
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
    assert!(
        e.presentation
            .visuals
            .iter()
            .any(|v| v.style == Style::Region)
    );
}

/// Every shape a tool draws keeps its shape by constraint, and closes into
/// a region: the solver finds nothing to correct, and leaves only the
/// freedoms the shape has.
#[test]
fn every_shape_tool_draws_a_closed_constrained_shape() {
    // (tool, clicks, curves drawn, degrees of freedom left)
    let shapes: [(&str, &[P2], usize, usize); 7] = [
        ("center_rectangle", &[[1.0, 1.0], [1.5, 1.3]], 5, 4),
        (
            "three_point_rectangle",
            &[[0.2, 0.2], [1.2, 0.5], [1.0, 1.0]],
            4,
            5,
        ),
        (
            "three_point_circle",
            &[[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]],
            1,
            3,
        ),
        ("polygon", &[[2.0, 2.0], [2.5, 2.0]], 7, 4),
        ("slot", &[[0.5, 0.5], [1.5, 0.5], [1.0, 0.8]], 5, 5),
        ("circle", &[[3.0, 3.0], [3.4, 3.0]], 1, 3),
        ("rectangle", &[[0.2, 0.2], [1.0, 1.0]], 4, 4),
    ];
    for (tool, clicks, curves, dof) in shapes {
        let mut e = drawing();
        e.act("tool", tool);
        for c in clicks {
            e.click(c[0], c[1]);
        }
        assert_eq!(e.drawn_curves().len(), curves, "{tool}");
        let report = e.report();
        assert!(report.converged, "{tool}: {report:?}");
        assert_eq!(report.dof, dof, "{tool}: {report:?}");
        assert_eq!(e.sketch().regions().unwrap().len(), 1, "{tool}");
    }
}

/// A center-point arc turns the way the pointer went round, past half a
/// circle if it went that far, and keeps its center by constraint.
#[test]
fn center_point_arcs_follow_the_pointer_round() {
    let mut e = drawing();
    e.act("tool", "center_arc");
    e.click(1.0, 1.0);
    e.click(2.0, 1.0);
    // Round counter-clockwise, past the top and the far side.
    for a in [0.5f64, 1.5, 2.5, 3.5, 4.0] {
        e.hover(1.0 + a.cos(), 1.0 + a.sin());
    }
    e.click(1.0 + 4.0f64.cos(), 1.0 + 4.0f64.sin());
    let arcs: Vec<f64> = e
        .drawn_curves()
        .iter()
        .filter_map(|(_, k)| match k {
            CurveKind::Arc { sweep, .. } => Some(sweep.to_f64()),
            _ => None,
        })
        .collect();
    assert_eq!(arcs.len(), 1);
    assert!((arcs[0] - 4.0).abs() < 1e-9, "{arcs:?}");
    assert!(e.has(|c| matches!(c, Constraint::Center { .. })));
    assert!(e.report().converged);
}

/// Drawing a chain of lines and moving back onto its end switches to a
/// tangent arc; after the arc, the chain goes on with lines — lines and
/// arcs in one fluent motion.
#[test]
fn line_chains_switch_to_tangent_arcs_and_back() {
    let mut e = drawing();
    e.act("tool", "line");
    e.click(0.0, 0.0);
    e.click(1.0, 0.0);
    // Away from the end, and back onto it: the next segment is an arc.
    e.hover(1.3, 0.2);
    assert!(!e.hint().0.contains("tangent arc ends"), "a line");
    e.hover(1.002, 0.001);
    e.hover(1.4, 0.5);
    assert!(e.hint().0.contains("tangent arc ends"), "an arc");
    e.click(1.0, 1.0);
    // Then a line again.
    assert!(!e.hint().0.contains("tangent arc ends"), "a line");
    e.click(0.0, 1.0);
    e.key("Escape");
    let kinds: Vec<&str> = e
        .drawn_curves()
        .iter()
        .map(|(_, k)| match k {
            CurveKind::Line { .. } => "line",
            CurveKind::Arc { .. } => "arc",
            _ => "other",
        })
        .collect();
    assert_eq!(kinds, ["line", "arc", "line"]);
    // Into the arc and out of it again, tangent both times — and both
    // lines, drawn level, still horizontal: the return line's tangency and
    // its horizontal say the same, and the sketch solves all the same.
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 2);
    assert_eq!(e.count(|c| matches!(c, Constraint::Horizontal { .. })), 2);
    let report = e.report();
    assert!(report.converged, "{report:?}");
    // A half circle to the left, tangent to the first line.
    let arc = e
        .drawn_curves()
        .iter()
        .find_map(|(_, k)| match k {
            CurveKind::Arc { sweep, .. } => Some(sweep.to_f64()),
            _ => None,
        })
        .unwrap();
    assert!((arc - std::f64::consts::PI).abs() < 1e-6, "{arc}");
}

/// A line drawn on from a tangent arc leaves it tangentially, by
/// constraint, whichever way it was drawn — and gets no horizontal or
/// vertical it was not drawn near.
#[test]
fn lines_leave_tangent_arcs_tangentially() {
    let mut e = drawing();
    e.act("tool", "line");
    e.click(0.5, 0.5);
    e.click(1.5, 0.5);
    e.hover(1.8, 0.7);
    e.hover(1.502, 0.501);
    e.hover(1.9, 0.9);
    assert!(e.hint().0.contains("tangent arc ends"), "an arc");
    // A quarter turn up-left, so the arc ends heading up and left.
    e.click(2.0, 1.2);
    e.click(2.3, 2.0);
    e.key("Escape");
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 2);
    assert_eq!(e.count(|c| matches!(c, Constraint::Horizontal { .. })), 1);
    assert_eq!(e.count(|c| matches!(c, Constraint::Vertical { .. })), 0);
    let report = e.report();
    assert!(report.converged, "{report:?}");
    // The return line runs on the way the arc ends.
    let s = e.sketch();
    let (arc, line) = {
        let curves = e.drawn_curves();
        (curves[1].0, curves[2].0)
    };
    let CurveKind::Line { start, end } = s.curves[&line].kind else {
        panic!("a line")
    };
    let CurveKind::Arc { end: arc_end, .. } = s.curves[&arc].kind else {
        panic!("an arc")
    };
    assert_eq!(arc_end, start, "the line starts where the arc ends");
    let t = plain(arc_of(s, arc).tangent_end().unwrap());
    let d = sub(xy(s, end), xy(s, start));
    assert!(
        cross(t, d).abs() < 1e-6 * distance(xy(s, end), xy(s, start)),
        "{t:?} vs {d:?}"
    );
}

/// The tangent-arc tool continues a curve from its end.
#[test]
fn tangent_arcs_continue_curves() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.0], [1.0, 0.0]]);
    e.act("tool", "tangent_arc");
    // Not from nowhere.
    e.click(0.5, 0.7);
    assert!(
        e.hint().0.starts_with("Click the end of a curve"),
        "{:?}",
        e.hint()
    );
    e.click(1.0, 0.0);
    e.click(1.5, 0.5);
    assert!(e.has(|c| matches!(c, Constraint::Tangent { .. })));
    assert!(e.report().converged);
}

/// With nothing selected every constraint tool is there to take; a
/// selection greys out those it can never become.
#[test]
fn constraint_tools_follow_the_selection() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.5], [1.0, 0.7]]);
    e.lines(&[[0.0, 1.5], [1.0, 2.0]]);
    assert!(
        ["horizontal", "parallel", "distance", "radius", "angle"]
            .iter()
            .all(|tool| e.constraint_tools().iter().any(|(name, _)| name == tool)),
        "a button for every constraint"
    );
    assert!(e.constraint_tools().iter().all(|(_, enabled)| *enabled));
    e.click(0.5, 0.6);
    assert_eq!(e.selection().len(), 1);
    assert!(e.enabled("horizontal") && e.enabled("parallel") && e.enabled("distance"));
    assert!(!e.enabled("radius") && !e.enabled("concentric") && !e.enabled("diameter"));
    e.click(0.5, 1.75);
    assert!(e.enabled("parallel") && e.enabled("angle"));
    assert!(!e.enabled("midpoint") && !e.enabled("symmetric"));
    // Complete: pressed, it is added at once.
    e.act("constrain", "parallel");
    assert!(e.has(|c| matches!(c, Constraint::Parallel { .. })));
    assert!(e.selection().is_empty());

    e.click(0.5, 0.6);
    e.key("Delete");
    assert_eq!(e.drawn_curves().len(), 1);
    assert!(
        !e.has(|c| matches!(c, Constraint::Parallel { .. })),
        "the parallel went with it"
    );
}

/// A constraint tool taken up with nothing selected picks what is clicked
/// next, and adds the constraint once it has all it needs — staying in
/// hand for the next.
#[test]
fn constraint_tools_pick_what_is_clicked() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.5], [1.0, 0.7]]);
    e.lines(&[[0.0, 1.5], [1.0, 2.0]]);
    e.act("constrain", "perpendicular");
    assert_eq!(e.tool().as_deref(), Some("perpendicular"));
    e.click(0.5, 0.6);
    assert_eq!(e.selection().len(), 1);
    e.click(0.5, 1.75);
    assert!(e.has(|c| matches!(c, Constraint::Perpendicular { .. })));
    assert!(e.selection().is_empty());
    assert_eq!(e.tool().as_deref(), Some("perpendicular"));
    e.key("Escape");
    assert_eq!(e.tool(), None);
}

/// The sketch's origin is a point, and its axes are lines, to constrain
/// against: a line made parallel to the `x` axis turns horizontal.
#[test]
fn the_origin_and_axes_are_selectable() {
    let mut e = drawing();
    e.lines(&[[0.5, 0.5], [1.5, 0.9]]);
    // The x axis, far out along it.
    e.click(3.0, 0.0);
    e.click(1.0, 0.7);
    assert_eq!(e.selection().len(), 2);
    e.act("constrain", "parallel");
    assert!(e.has(|c| matches!(c, Constraint::Parallel { .. })));
    let s = e.sketch();
    let ends: Vec<P2> = e.drawn_points().iter().map(|&p| xy(s, p)).collect();
    assert!((ends[0][1] - ends[1][1]).abs() < 1e-6, "{ends:?}");
    // The origin, and the line's start: one point.
    let origin = e.origin();
    e.click(0.0, 0.0);
    assert_eq!(e.selection(), [origin.to_string()]);
    e.click(0.5, ends[0][1]);
    e.act("constrain", "coincident");
    assert!(e.report().converged);
    let s = e.sketch();
    assert!(
        e.drawn_points()
            .iter()
            .any(|&p| close(xy(s, p), [0.0, 0.0]))
    );
    // Neither can be deleted.
    e.click(0.0, 0.0);
    e.key("Delete");
    assert!(e.sketch().points.contains_key(&origin));
}

/// A dimension's value is asked for in place as it is added, and can be a
/// formula of the part's parameters, which it then follows.
#[test]
fn dimensions_take_values_and_formulas() {
    let parameters = Parameters {
        values: vec![Parameter::number("width", "4", 0.0, 100.0)],
        ..Parameters::default()
    };
    let mut e = Editor::on(parameters, false);
    e.lines(&[[0.0, 0.5], [1.0, 0.8]]);
    e.click(0.5, 0.65);
    e.act("constrain", "distance");
    // Nothing yet: the dimension follows the pointer, to be placed.
    assert!(!e.has(|c| matches!(c, Constraint::Length { .. })));
    e.hover(0.6, 1.5);
    assert!(e.presentation.visuals.iter().any(|v| v.key == "placing"));
    let middle = |e: &Editor| {
        let s = e.sketch();
        let ends: Vec<P2> = e.drawn_points().iter().map(|&p| xy(s, p)).collect();
        scale(add(ends[0], ends[1]), 0.5)
    };
    let placed = sub([0.6, 1.5], middle(&e));
    e.click(0.6, 1.5);
    let (&id, _) = e
        .sketch()
        .constraints
        .iter()
        .find(|(_, c)| matches!(c, Constraint::Length { .. }))
        .expect("a line's length");
    let prompt = e
        .presentation
        .prompt
        .clone()
        .expect("its value is asked for");
    e.dialog(&prompt.key, Value::Text("width / 2".into()));
    assert!(e.presentation.prompt.is_none());
    assert_eq!(e.args.formulas[&id], "width / 2".into());
    let length = |e: &Editor| {
        let s = e.sketch();
        let ends: Vec<P2> = e.drawn_points().iter().map(|&p| xy(s, p)).collect();
        distance(ends[0], ends[1])
    };
    assert!((length(&e) - 2.0).abs() < 1e-6);
    // Its value is where it was placed, with its dimension lines; it can
    // be dragged on from there.
    let label_at = |e: &Editor| {
        let label = e
            .presentation
            .visuals
            .iter()
            .find(|v| v.key == id.to_string())
            .unwrap()
            .clone();
        let Shape::Label { at, .. } = label.shape else {
            panic!("a label")
        };
        [at[0].to_f64(), at[1].to_f64()]
    };
    // Placed off the line's middle, it stays there as the line grows.
    assert!(
        distance(label_at(&e), add(middle(&e), placed)) < 1e-9,
        "{:?}",
        label_at(&e)
    );
    assert!(
        e.presentation
            .visuals
            .iter()
            .any(|v| v.key.starts_with(&format!("{id}/")))
    );
    e.key("Escape");
    assert_eq!(e.tool(), None);
    let from = label_at(&e);
    e.drag(from, [from[0] + 0.5, from[1] + 0.3], false);
    let to = label_at(&e);
    assert!(
        distance(to, [from[0] + 0.5, from[1] + 0.3]) < 1e-9,
        "{to:?}"
    );
    assert!(
        (length(&e) - 2.0).abs() < 1e-6,
        "moving the value moves nothing"
    );
    // A formula that does not evaluate is refused, saying why.
    let [x, y] = to;
    e.click(x, y);
    e.click_with(x, y, true, false);
    assert!(e.presentation.prompt.is_some(), "double click opens it");
    e.dialog("prompt", Value::Text("height * 2".into()));
    assert!(e.presentation.prompt.is_some());
    assert!(matches!(
        e.presentation.dialog.get("hint"),
        Some(Control::Text {
            tone: Tone::Error,
            ..
        })
    ));
    // A plain number is a value again.
    e.dialog("prompt", Value::Text("3".into()));
    assert!(!e.args.formulas.contains_key(&id));
    assert!((length(&e) - 3.0).abs() < 1e-6);
    // And in the list too.
    e.dialog(&format!("constraint:{}", id.0), Value::Text("width".into()));
    assert!((length(&e) - 4.0).abs() < 1e-6);
}

/// Hovering a point offers a grab; dragging it moves it, as far as the
/// constraints let it.
#[test]
fn dragging_points() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.0], [1.0, 0.0]]);
    e.hover(1.0, 0.0);
    assert!(e.presentation.grab);
    e.drag([1.0, 0.0], [2.0, 0.4], false);
    let s = e.sketch();
    let ends: Vec<P2> = s.points.keys().map(|&id| xy(s, id)).collect();
    // The line starts at the origin and is horizontal by constraint: the
    // end follows the pointer along it, not up.
    assert!(ends.iter().any(|&p| close(p, [0.0, 0.0])), "{ends:?}");
    assert!(ends.iter().any(|&p| close(p, [2.0, 0.0])), "{ends:?}");
}

/// A point dragged onto another point, or the origin, snaps there and is
/// made one with it when let go — unless shift is held.
#[test]
fn dragged_points_snap_onto_points() {
    let mut e = drawing();
    e.lines(&[[0.5, 0.3], [1.0, 1.2]]);
    e.drag([0.5, 0.3], [0.003, 0.002], true);
    assert!(!e.has(|c| matches!(c, Constraint::Coincident { .. })));
    e.drag([0.003, 0.002], [0.002, -0.001], false);
    assert!(e.has(|c| matches!(c, Constraint::Coincident { .. })));
    let s = e.sketch();
    assert!(
        e.drawn_points()
            .iter()
            .any(|&p| close(xy(s, p), [0.0, 0.0]))
    );
    // Onto the middle of another line.
    e.lines(&[[2.0, 0.0], [2.0, 2.0]]);
    let free = e
        .drawn_points()
        .into_iter()
        .find(|&p| close(xy(e.sketch(), p), [1.0, 1.2]))
        .unwrap();
    let _ = free;
    e.drag([1.0, 1.2], [2.002, 1.0], false);
    assert!(e.has(|c| matches!(c, Constraint::Midpoint { .. })));
}

/// A corner of two lines is rounded: both cut back, an arc tangent to
/// each between them, its radius a dimension asked for.
#[test]
fn fillets_round_corners() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(1.2, 1.2);
    e.act("tool", "fillet");
    e.click(1.2, 1.2);
    assert_eq!(e.drawn_curves().len(), 5);
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 2);
    assert!(e.presentation.prompt.is_some());
    assert!(e.report().converged, "{:?}", e.report());
    e.dialog("prompt", Value::Text("0.1".into()));
    assert!(e.report().converged, "{:?}", e.report());
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
}

/// With construction switched on, what is drawn is construction geometry;
/// switched on with curves selected, it is those that change.
#[test]
fn construction_geometry() {
    let mut e = drawing();
    e.act("tool", "construction");
    assert!(e.construction());
    e.lines(&[[0.5, 0.5], [1.0, 1.0]]);
    assert!(e.sketch().curves[&e.drawn_curves()[0].0].construction);
    e.act("tool", "construction");
    e.click(0.75, 0.75);
    e.act("tool", "construction");
    assert!(!e.sketch().curves[&e.drawn_curves()[0].0].construction);
}

/// Picking another plane moves the drawing onto it, kept as drawn.
#[test]
fn another_plane_keeps_the_drawing() {
    let mut e = drawing();
    e.key("c");
    e.click(0.5, 0.5);
    e.click(1.0, 0.5);
    assert_eq!(e.drawn_curves().len(), 1);
    e.press("plane");
    assert!(e.presentation.focus.is_none());
    e.send(StepEditEvent::Click {
        pointer: pointer([0.04, -10.0, 0.04], [0.0, 1.0, 0.0]),
        button: Button::Primary,
        double: false,
        shift: false,
    });
    assert_eq!(e.drawn_curves().len(), 1);
    let focus = e.presentation.focus.expect("drawing faces the new plane");
    assert_eq!(*focus.w(), v([0.0, 1.0, 0.0]));
}

/// A polygon's sides are set in the dialog while its tool is in hand.
#[test]
fn polygons_have_as_many_sides_as_asked() {
    let mut e = drawing();
    e.act("tool", "polygon");
    assert!(e.presentation.dialog.get("sides").is_some());
    e.dialog("sides", Value::Number(5.0));
    e.click(1.0, 1.0);
    e.click(1.5, 1.0);
    let lines = e
        .drawn_curves()
        .iter()
        .filter(|(_, k)| matches!(k, CurveKind::Line { .. }))
        .count();
    assert_eq!(lines, 5);
}

/// A line drawn along the sketch's `x` axis is that line where it is
/// clicked, not the axis under it.
#[test]
fn drawn_lines_win_over_the_axes_they_lie_on() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.0], [1.0, 0.0]]);
    let line = e.drawn_curves()[0].0;
    e.click(0.5, 0.0);
    assert_eq!(e.selection(), [line.to_string()]);
    // Beyond it, the axis.
    e.key("Escape");
    e.click(2.0, 0.0);
    assert_ne!(e.selection(), [line.to_string()]);
    assert_eq!(e.selection().len(), 1);
}

/// A region is filled exactly: a rectangle with two round holes is shaded
/// as the rectangle minus the holes, no triangle overlapping another or
/// spanning a hole.
#[test]
fn regions_with_holes_are_filled_exactly() {
    let mut e = drawing();
    e.key("r");
    e.click(-2.9, -1.4);
    e.click(2.9, 1.4);
    e.key("c");
    e.click(-1.75, 0.0);
    e.click(-1.12, 0.0);
    e.click(1.75, 0.0);
    e.click(2.25, 0.0);
    let triangles: Vec<[Vector3<S>; 3]> = e
        .presentation
        .visuals
        .iter()
        .filter(|v| v.style == Style::Region)
        .flat_map(|v| match &v.shape {
            Shape::Triangles { triangles } => triangles.clone(),
            _ => Vec::new(),
        })
        .collect();
    let area: f64 = triangles
        .iter()
        .map(|[a, b, c]| {
            let (u, w) = (b.sub(a), c.sub(a));
            (u[0].to_f64() * w[1].to_f64() - u[1].to_f64() * w[0].to_f64()).abs() / 2.0
        })
        .sum();
    let pi = std::f64::consts::PI;
    let expected = 5.8 * 2.8 - pi * 0.63 * 0.63 - pi * 0.5 * 0.5;
    // The holes are drawn as polylines: a little less round than circles.
    assert!((area - expected).abs() < 1e-2, "{area} vs {expected}");
}

/// An angle is placed by the click that takes it, drawn as an arc about
/// where its lines meet through its value, and its value dragged on.
#[test]
fn angles_are_placed_and_dragged() {
    let mut e = drawing();
    e.lines(&[[0.5, 0.5], [2.0, 0.6]]);
    e.lines(&[[0.5, 0.7], [1.5, 1.8]]);
    e.act("constrain", "angle");
    e.click(1.25, 0.55);
    e.click(1.0, 1.25);
    assert!(!e.has(|c| matches!(c, Constraint::Angle { .. })));
    e.click(1.3, 0.95);
    let (&id, _) = e
        .sketch()
        .constraints
        .iter()
        .find(|(_, c)| matches!(c, Constraint::Angle { .. }))
        .expect("the angle, placed");
    assert!(e.args.labels.contains_key(&id));
    // Its arc: every point of it as far from where the lines meet.
    let arc = e
        .presentation
        .visuals
        .iter()
        .find(|v| v.key == format!("{id}/0"))
        .expect("its arc");
    let Shape::Polyline { points } = &arc.shape else {
        panic!("a polyline")
    };
    assert!(points.len() > 10);
    e.dialog("prompt", Value::Text("40".into()));
    assert!(e.report().converged, "{:?}", e.report());
    e.key("Escape");
    let before = e.args.labels[&id];
    let label = |e: &Editor| match &e
        .presentation
        .visuals
        .iter()
        .find(|v| v.key == id.to_string())
        .unwrap()
        .shape
    {
        Shape::Label { at, .. } => [at[0].to_f64(), at[1].to_f64()],
        _ => panic!("a label"),
    };
    let from = label(&e);
    e.drag(from, [from[0] + 0.2, from[1] + 0.2], false);
    let after = e.args.labels[&id];
    assert!(
        distance(sub(after, before), [0.2, 0.2]) < 1e-9,
        "{before:?} -> {after:?}"
    );
}

/// A point placed where two curves cross snaps there and is constrained
/// onto both: two lines, a line and a circle, a line and an axis.
#[test]
fn points_snap_to_intersections() {
    let mut e = drawing();
    e.lines(&[[0.5, 0.5], [2.5, 1.5]]);
    e.lines(&[[0.5, 1.5], [2.5, 0.5]]);
    e.act("tool", "circle");
    e.click(3.5, 1.0);
    e.click(3.5, 1.5);
    e.lines(&[[2.8, 1.003], [4.7, 0.997]]);
    e.lines(&[[5.0, -0.5], [5.5, 0.5]]);
    e.act("tool", "point");
    let on = |e: &Editor, point: PointId| {
        e.sketch()
            .constraints
            .values()
            .filter(|c| matches!(c, Constraint::PointOnCurve { point: p, .. } if *p == point))
            .count()
    };
    for (near, at) in [
        ([1.503, 1.004], [1.5, 1.0]),
        ([3.004, 1.002], [3.0, 1.0]),
        ([5.252, 0.003], [5.25, 0.0]),
    ] {
        e.hover(near[0], near[1]);
        assert!(e.shows("snap"), "{near:?}");
        e.click(near[0], near[1]);
        let p = *e.drawn_points().last().unwrap();
        assert_eq!(on(&e, p), 2, "{near:?}");
        assert!(
            distance(xy(e.sketch(), p), at) < 1e-6,
            "{:?}",
            xy(e.sketch(), p)
        );
    }
    assert!(e.report().converged, "{:?}", e.report());
    // With shift, no snapping: a point anywhere.
    e.click_with(1.503, 1.004, false, true);
    assert_eq!(on(&e, *e.drawn_points().last().unwrap()), 0);
}

/// A curve the sketch meets nowhere goes as a whole, with its constraints
/// and its points; hovering it shows that first. The trim tool takes a
/// press anywhere, to drag across what it removes.
#[test]
fn trimming_removes_a_curve_met_nowhere() {
    let mut e = drawing();
    e.lines(&[[0.5, 0.5], [1.5, 0.5]]);
    assert!(e.has(|c| matches!(c, Constraint::Horizontal { .. })));
    e.key("m");
    assert_eq!(e.tool().as_deref(), Some("trim"));
    e.hover(0.2, 1.4);
    assert!(e.presentation.grab, "a press anywhere strokes");
    assert_eq!(e.removed(), 0);
    e.hover(1.0, 0.5);
    assert_eq!(e.removed(), 1);
    e.click(1.0, 0.5);
    assert!(e.drawn_curves().is_empty());
    assert!(e.drawn_points().is_empty());
    assert!(!e.has(|c| matches!(c, Constraint::Horizontal { .. })));
    assert_eq!(e.tool().as_deref(), Some("trim"), "the tool stays in hand");
}

/// A curve crossed is cut back to the crossing: it keeps its id and its
/// constraints, and ends at a new point on the curve crossing it. Cut back
/// to that point in turn, the other curve ends there too: a corner.
#[test]
fn trimming_cuts_back_to_where_curves_cross() {
    let mut e = drawing();
    e.lines(&[[0.2, 0.5], [1.8, 0.5]]);
    e.lines(&[[1.0, 0.1], [1.0, 0.9]]);
    let across = e.line_between([0.2, 0.5], [1.8, 0.5]).unwrap();
    let up = e.line_between([1.0, 0.1], [1.0, 0.9]).unwrap();
    e.key("m");
    e.click(0.5, 0.5);
    assert_eq!(e.line_between([1.0, 0.5], [1.8, 0.5]), Some(across));
    assert!(e.has(|c| matches!(c, Constraint::Horizontal { line } if *line == across)));
    assert!(e.has(|c| matches!(c, Constraint::PointOnCurve { curve, .. } if *curve == up)));
    assert_eq!(
        e.drawn_points().len(),
        4,
        "the cut end went: {:?}",
        e.sketch()
    );
    assert!(e.report().converged, "{:?}", e.report());

    e.click(1.0, 0.3);
    assert_eq!(e.line_between([1.0, 0.5], [1.0, 0.9]), Some(up));
    assert_eq!(e.drawn_curves().len(), 2);
    assert_eq!(e.drawn_points().len(), 3);
    let s = e.sketch();
    let (a, b) = (s.curves[&across].points(), s.curves[&up].points());
    assert!(a.iter().any(|p| b.contains(p)), "a corner: {s:?}");
    assert!(!e.has(|c| matches!(c, Constraint::PointOnCurve { .. })));
    assert!(e.report().converged, "{:?}", e.report());
}

/// The middle of a curve crossed twice goes: what is left either side
/// stays on one line by constraint, the first part keeping the curve's own
/// constraints — but not its length, which no longer holds.
#[test]
fn trimming_the_middle_splits_a_curve() {
    let mut e = drawing();
    e.lines(&[[0.2, 0.5], [2.2, 0.5]]);
    e.lines(&[[0.8, 0.1], [0.8, 0.9]]);
    e.lines(&[[1.6, 0.1], [1.6, 0.9]]);
    let across = e.line_between([0.2, 0.5], [2.2, 0.5]).unwrap();
    e.click(1.2, 0.5);
    e.act("constrain", "distance");
    e.click(1.2, 1.2);
    e.key("Escape");
    assert!(e.has(|c| matches!(c, Constraint::Length { .. })));

    e.key("m");
    e.hover(1.2, 0.5);
    assert_eq!(e.removed(), 1);
    e.click(1.2, 0.5);
    assert_eq!(e.drawn_curves().len(), 4);
    assert_eq!(e.line_between([0.2, 0.5], [0.8, 0.5]), Some(across));
    let rest = e
        .line_between([1.6, 0.5], [2.2, 0.5])
        .expect("the far part");
    assert!(e.has(|c| matches!(*c, Constraint::Collinear { a, b } if (a, b) == (across, rest))));
    assert_eq!(e.count(|c| matches!(c, Constraint::Horizontal { .. })), 1);
    assert!(!e.has(|c| matches!(c, Constraint::Length { .. })));
    assert!(e.report().converged, "{:?}", e.report());
}

/// A circle crossed twice keeps the arc not clicked, about its center.
#[test]
fn trimming_a_circle_leaves_an_arc() {
    let mut e = drawing();
    e.key("c");
    e.click(1.0, 1.0);
    e.click(1.5, 1.0);
    e.lines(&[[0.2, 1.0], [1.8, 1.0]]);
    e.key("m");
    e.click(1.0, 1.5);
    let arcs: Vec<f64> = e
        .drawn_curves()
        .iter()
        .filter_map(|(_, k)| match k {
            CurveKind::Arc { sweep, .. } => Some(sweep.to_f64()),
            _ => None,
        })
        .collect();
    assert_eq!(arcs.len(), 1, "{:?}", e.sketch());
    assert!((arcs[0] - std::f64::consts::PI).abs() < 1e-6, "{arcs:?}");
    assert!(
        !e.drawn_curves()
            .iter()
            .any(|(_, k)| matches!(k, CurveKind::Circle { .. }))
    );
    assert!(e.has(|c| matches!(c, Constraint::Center { .. })));
    // The arc's ends lie on the line.
    assert_eq!(e.count(|c| matches!(c, Constraint::PointOnCurve { .. })), 2);
    assert!(e.report().converged, "{:?}", e.report());
    // Cut back to the arc, the line closes a region with it.
    e.click(0.3, 1.0);
    e.click(1.7, 1.0);
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
    assert!(e.report().converged, "{:?}", e.report());
}

/// Dragged across, the trim tool shows what it would remove along the way,
/// and removes every curve the way it went crosses when let go.
#[test]
fn dragging_trims_every_curve_crossed() {
    let mut e = drawing();
    for x in [0.5, 1.0, 1.5] {
        e.lines(&[[x, 0.2], [x, 0.8]]);
    }
    e.lines(&[[0.5, 1.5], [1.5, 1.5]]);
    e.key("m");
    e.hover(0.3, 0.5);
    e.send(StepEditEvent::Drag {
        from: down(0.3, 0.5),
        to: down(1.2, 0.52),
        done: false,
        shift: false,
    });
    assert_eq!(e.removed(), 2, "crossed so far");
    assert!(e.presentation.visuals.iter().any(|v| v.key == "stroke"));
    e.send(StepEditEvent::Drag {
        from: down(0.3, 0.5),
        to: down(1.7, 0.55),
        done: true,
        shift: false,
    });
    assert_eq!(e.drawn_curves().len(), 1, "{:?}", e.sketch());
    assert!(e.line_between([0.5, 1.5], [1.5, 1.5]).is_some());
    assert!(!e.shows("stroke"));
    assert_eq!(e.removed(), 0);
    // A stroke that winds through several pieces of one curve takes each.
    e.lines(&[[0.2, 1.0], [2.0, 1.0]]);
    e.lines(&[[0.8, 0.6], [0.8, 1.4]]);
    e.lines(&[[1.4, 0.6], [1.4, 1.4]]);
    e.key("m");
    e.stroke(&[[0.5, 0.9], [0.5, 1.1], [1.7, 1.1], [1.7, 0.9]]);
    assert!(
        e.line_between([0.8, 1.0], [1.4, 1.0]).is_some(),
        "{:?}",
        e.sketch()
    );
    assert!(e.line_between([0.2, 1.0], [0.8, 1.0]).is_none());
    assert!(e.line_between([1.4, 1.0], [2.0, 1.0]).is_none());
    assert!(e.report().converged, "{:?}", e.report());
}

/// A fillet's arc meets its lines only where it is tangent to them, at its
/// ends: trimmed, it goes whole, with its tangencies and its radius.
#[test]
fn trimming_a_fillet_removes_its_constraints() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(1.2, 1.2);
    e.act("tool", "fillet");
    e.click(1.2, 1.2);
    e.key("Escape");
    e.key("Escape");
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 2);
    let arc = e
        .drawn_curves()
        .iter()
        .find_map(|(id, k)| matches!(k, CurveKind::Arc { .. }).then_some(*id))
        .unwrap();
    let mid = plain(arc_of(e.sketch(), arc).arc_mid().unwrap());
    e.key("m");
    e.click(mid[0], mid[1]);
    assert_eq!(e.drawn_curves().len(), 4, "{:?}", e.sketch());
    assert!(!e.has(|c| matches!(c, Constraint::Tangent { .. } | Constraint::Radius { .. })));
    assert!(e.report().converged, "{:?}", e.report());
}

/// A point on a curve by constraint cuts it like a crossing: a line
/// started on another's middle cuts that one there. Trimmed back to it,
/// the point ends the line instead of being its middle; the line started
/// there, met nowhere else, goes whole — and the point stays, still the
/// other's end.
#[test]
fn points_on_a_curve_cut_it() {
    let mut e = drawing();
    e.lines(&[[0.2, 0.5], [1.8, 0.5]]);
    e.lines(&[[1.002, 0.5], [1.0, 1.2]]);
    assert!(e.has(|c| matches!(c, Constraint::Midpoint { .. })));
    e.key("m");
    e.click(0.5, 0.5);
    assert!(
        e.line_between([1.0, 0.5], [1.8, 0.5]).is_some(),
        "{:?}",
        e.sketch()
    );
    assert!(!e.has(|c| matches!(c, Constraint::Midpoint { .. })));
    assert_eq!(e.drawn_points().len(), 3, "{:?}", e.sketch());
    assert!(e.report().converged, "{:?}", e.report());
    e.click(1.0, 0.9);
    assert_eq!(e.drawn_curves().len(), 1);
    assert!(e.line_between([1.0, 0.5], [1.8, 0.5]).is_some());
    assert_eq!(e.drawn_points().len(), 2);
}

/// The sketch's own axes are no curves to trim back to, and cannot be
/// trimmed: a rectangle's side across one goes whole, and a click on the
/// axis itself does nothing.
#[test]
fn the_axes_neither_cut_nor_are_trimmed() {
    let mut e = drawing();
    e.key("r");
    e.click(-0.5, 0.4);
    e.click(0.6, 1.2);
    let curves = e.sketch().curves.len();
    e.key("m");
    e.hover(0.0, 0.8);
    assert_eq!(e.removed(), 0, "the axis is no curve to trim");
    e.click(0.0, 0.8);
    assert_eq!(e.sketch().curves.len(), curves);
    e.click(0.3, 0.4);
    assert_eq!(e.drawn_curves().len(), 3, "{:?}", e.sketch());
    assert!(e.report().converged, "{:?}", e.report());
}

/// The bounding box of the points of `curves`, as solved.
fn bounds(s: &Sketch, curves: &[CurveId]) -> [f64; 4] {
    let points = curves.iter().flat_map(|c| s.curves[c].points());
    let (lo, hi) = point_bounds(points.map(|p| xy(s, p))).unwrap();
    [lo[0], lo[1], hi[0], hi[1]]
}

fn near(a: [f64; 4], b: [f64; 4]) -> bool {
    a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-6)
}

impl Editor {
    /// The drawn curves of the kind `kind` names: `"line"`, `"arc"`,
    /// `"circle"`.
    fn curves_of(&self, kind: &str) -> Vec<CurveId> {
        self.drawn_curves()
            .into_iter()
            .filter(|(_, k)| {
                matches!(
                    (kind, k),
                    ("line", CurveKind::Line { .. })
                        | ("arc", CurveKind::Arc { .. })
                        | ("circle", CurveKind::Circle { .. })
                )
            })
            .map(|(id, _)| id)
            .collect()
    }

    /// The profile curves drawn: not construction.
    fn profile_curves(&self) -> Vec<CurveId> {
        self.drawn_curves()
            .into_iter()
            .filter(|(id, _)| !self.sketch().curves[id].construction)
            .map(|(id, _)| id)
            .collect()
    }

    /// The centers of the circles drawn.
    fn circle_centers(&self) -> Vec<P2> {
        let s = self.sketch();
        self.drawn_curves()
            .into_iter()
            .filter_map(|(_, k)| match k {
                CurveKind::Circle { center, .. } => Some(xy(s, *center)),
                _ => None,
            })
            .collect()
    }
}

/// Offset: a click on a side takes the rectangle's whole chain, the
/// pointer beside it shows the offset there, and a click makes it — rounded
/// round the corners, tied to the rectangle by one distance, which is
/// asked for and which the offset follows, as it follows the rectangle.
#[test]
fn offsets_follow_their_chain() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(1.2, 0.7);
    let rectangle: Vec<CurveId> = e.drawn_curves().iter().map(|(id, _)| *id).collect();
    e.key("o");
    assert_eq!(e.tool().as_deref(), Some("offset"));
    e.click(0.7, 0.2);
    assert_eq!(e.selection().len(), 4, "the side's chain: all four");
    e.hover(0.7, -0.1);
    assert!(
        e.presentation
            .visuals
            .iter()
            .any(|v| v.key.starts_with("offset") && v.style == Style::Draft),
        "the offset is previewed"
    );
    e.click(0.7, -0.05);
    assert_eq!(e.drawn_curves().len(), rectangle.len() + 8);
    assert_eq!(e.curves_of("arc").len(), 4, "a round corner each");
    let offset = e.count(
        |c| matches!(c, Constraint::Offset { value, .. } if (value.to_f64() - 0.25).abs() < 1e-9),
    );
    assert_eq!(offset, 1, "{:?}", e.sketch().constraints);
    assert!(e.presentation.prompt.is_some(), "the distance is asked for");
    e.dialog("prompt", Value::Text("0.1".into()));
    let report = e.report();
    assert!(report.converged, "{report:?}");
    // The rectangle keeps its four freedoms; the offset adds none.
    assert_eq!(report.dof, 4, "{report:?}");
    let lines: Vec<CurveId> = e
        .curves_of("line")
        .into_iter()
        .filter(|c| !rectangle.contains(c) && !e.sketch().curves[c].construction)
        .collect();
    // Wherever the solve left the rectangle, its offset is 0.1 round it.
    let around = |e: &Editor| {
        let r = bounds(e.sketch(), &rectangle);
        near(
            bounds(e.sketch(), &lines),
            [r[0] - 0.1, r[1] - 0.1, r[2] + 0.1, r[3] + 0.1],
        )
    };
    assert!(around(&e), "{:?}", e.sketch());
    assert_eq!(e.sketch().regions().unwrap().len(), 1, "a ring");
    // The rectangle's corner dragged out: the offset goes with it.
    let corner = bounds(e.sketch(), &rectangle);
    e.drag([corner[2], corner[3]], [1.5, 0.9], true);
    assert!(e.report().converged);
    assert!(around(&e), "{:?}", e.sketch());
    let r = bounds(e.sketch(), &rectangle);
    assert!(close([r[2], r[3]], [1.5, 0.9]), "{r:?}");
}

/// Offset to both sides, corners extended: two offsets, each with its own
/// distance, meeting at their corners; an inward offset too deep for the
/// shape is refused, saying why.
#[test]
fn offsets_to_both_sides_and_refusals() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(1.2, 0.7);
    e.key("o");
    e.dialog("both", Value::Bool(true));
    e.dialog("corners", Value::Choice("extend".into()));
    e.click(0.7, 0.2);
    e.click(0.7, 0.3);
    assert_eq!(e.count(|c| matches!(c, Constraint::Offset { .. })), 2);
    assert_eq!(e.profile_curves().len(), 12);
    assert!(e.report().converged, "{:?}", e.report());
    // An arc offset past its center, from beyond it: refused.
    e.act("tool", "arc");
    e.click(3.0, 1.0);
    e.click(5.0, 1.0);
    e.click(4.0, 2.0);
    e.key("Escape");
    let curves = e.sketch().curves.len();
    e.key("o");
    e.click(4.0, 2.0);
    assert_eq!(e.selection().len(), 1);
    e.click(4.0, -0.5);
    assert_eq!(e.sketch().curves.len(), curves, "nothing made");
    let (why, tone) = e.hint();
    assert!(tone == Tone::Error && why.contains("smaller"), "{why:?}");
}

/// Mirror: what is selected is mirrored across the line clicked, a half
/// outline ending on the sketch's `y` axis closing into a whole one; with
/// nothing selected, the line comes first and each curve clicked after is
/// mirrored.
#[test]
fn mirrors_close_half_outlines() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.3], [0.6, 0.3], [0.6, 1.0], [0.0, 1.0]]);
    assert_eq!(e.count(|c| matches!(c, Constraint::PointOnCurve { .. })), 2);
    for p in [[0.3, 0.3], [0.6, 0.65], [0.3, 1.0]] {
        e.click(p[0], p[1]);
    }
    assert_eq!(e.selection().len(), 3);
    e.act("tool", "mirror");
    e.click(0.0, 2.0);
    assert_eq!(e.drawn_curves().len(), 6);
    assert_eq!(e.count(|c| matches!(c, Constraint::Symmetric { .. })), 2);
    assert_eq!(e.sketch().regions().unwrap().len(), 1, "one closed outline");
    assert!(e.report().converged, "{:?}", e.report());
    assert_eq!(e.tool(), None);
    // The original dragged wider: the image follows.
    e.drag([0.6, 0.3], [0.8, 0.3], true);
    assert!(e.report().converged);
    let s = e.sketch();
    assert!(
        e.drawn_points()
            .iter()
            .any(|&p| close(xy(s, p), [-0.8, 0.3])),
        "{:?}",
        e.drawn_points()
            .iter()
            .map(|&p| xy(s, p))
            .collect::<Vec<_>>()
    );

    // The line first, then what to mirror.
    e.key("c");
    e.click(2.0, 2.0);
    e.click(2.2, 2.0);
    e.act("tool", "mirror");
    e.click(3.0, 0.0);
    e.click(2.2, 2.0);
    let centers = e.circle_centers();
    assert!(
        centers.iter().any(|&c| close(c, [2.0, -2.0])),
        "{centers:?}"
    );
    assert!(e.has(|c| matches!(c, Constraint::Equal { .. })));
    assert_eq!(
        e.tool().as_deref(),
        Some("mirror"),
        "still in hand for the next"
    );
}

/// A linear pattern of a circle along the `x` axis: as many copies as
/// set, as far apart as set; then its construction line is selected, to
/// give its count, and its spacing asked for.
#[test]
fn linear_patterns_repeat_along_a_line() {
    let mut e = drawing();
    e.key("c");
    e.click(0.5, 0.5);
    e.click(0.7, 0.5);
    e.key("Escape");
    e.click(0.7, 0.5);
    assert_eq!(e.selection().len(), 1);
    e.act("tool", "linear_pattern");
    e.dialog("count", Value::Number(4.0));
    e.dialog("spacing", Value::Number(1.0));
    e.click(3.0, 0.0);
    let mut xs: Vec<f64> = e.circle_centers().iter().map(|c| c[0]).collect();
    xs.sort_by(f64::total_cmp);
    assert_eq!(xs.len(), 4, "{xs:?}");
    assert!((xs[3] - 3.5).abs() < 1e-6, "{xs:?}");
    assert!(e.report().converged, "{:?}", e.report());
    assert!(e.presentation.prompt.is_some(), "the spacing is asked for");
    assert!(e.presentation.dialog.get("pattern_count").is_some());
    e.dialog("prompt", Value::Text("0.75".into()));
    e.dialog("pattern_count", Value::Number(2.0));
    let mut xs: Vec<f64> = e.circle_centers().iter().map(|c| c[0]).collect();
    xs.sort_by(f64::total_cmp);
    assert_eq!(xs.len(), 2, "{xs:?}");
    assert!((xs[1] - xs[0] - 0.75).abs() < 1e-6, "{xs:?}");
    assert!(e.report().converged, "{:?}", e.report());
    // Without a selection, there is nothing to pattern.
    e.click(5.0, 5.0);
    assert!(e.selection().is_empty());
    assert!(matches!(
        e.presentation.dialog.get("tool"),
        Some(Control::Actions { actions }) if actions.iter().any(|a| a.value == "linear_pattern" && !a.enabled)
    ));
}

/// A circular pattern of a hole round the origin, spread over the whole
/// circle: its angle follows its count.
#[test]
fn circular_patterns_spread_round_the_circle() {
    let mut e = drawing();
    e.key("c");
    e.click(0.8, 0.6);
    e.click(0.9, 0.6);
    e.key("Escape");
    e.click(0.9, 0.6);
    e.act("tool", "circular_pattern");
    e.dialog("count", Value::Number(6.0));
    e.click(0.0, 0.0);
    let centers = e.circle_centers();
    assert_eq!(centers.len(), 6);
    let start = 0.6f64.atan2(0.8);
    for k in 0..6 {
        let a = start + std::f64::consts::PI / 3.0 * k as f64;
        assert!(
            centers.iter().any(|&c| close(c, [a.cos(), a.sin()])),
            "{k}: {centers:?}"
        );
    }
    assert!(
        e.args
            .formulas
            .values()
            .any(|f| *f == NumericInput::from("360/6"))
    );
    e.dialog("pattern_count", Value::Number(8.0));
    assert_eq!(e.circle_centers().len(), 8);
    assert!(
        e.args
            .formulas
            .values()
            .any(|f| *f == NumericInput::from("360/8"))
    );
    let report = e.report();
    assert!(report.converged, "{report:?}");
    // Wherever the solve turned the original to, the copies are an eighth
    // of a turn apart all round.
    let mut angles: Vec<f64> = e
        .circle_centers()
        .iter()
        .map(|c| c[1].atan2(c[0]))
        .collect();
    angles.sort_by(f64::total_cmp);
    for w in angles.windows(2) {
        assert!(
            (w[1] - w[0] - std::f64::consts::FRAC_PI_4).abs() < 1e-6,
            "{angles:?}"
        );
    }
}

/// An arc slot: four clicks — the center of its arc, where the arc starts
/// and ends, and its width — make a closed slot, round at both ends, that
/// keeps its shape by constraint.
#[test]
fn arc_slots_bend_round_their_center() {
    let mut e = drawing();
    e.act("tool", "arc_slot");
    for c in [[2.0, 2.0], [3.0, 2.0]] {
        e.click(c[0], c[1]);
    }
    e.hover(2.7, 2.7);
    e.click(2.0, 3.0);
    e.click(3.2, 2.0);
    assert_eq!(e.profile_curves().len(), 4);
    assert_eq!(e.curves_of("arc").len(), 5, "its centerline among them");
    let report = e.report();
    assert!(report.converged, "{report:?}");
    // Center, radius, start, sweep, width.
    assert_eq!(report.dof, 6, "{report:?}");
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
    e.sketch().enclose::<S>().unwrap();
    // Half round: its ends' centers and its center on one line.
    let mut e = drawing();
    e.act("tool", "arc_slot");
    e.click(1.2, 0.6);
    e.click(1.6, 0.6);
    e.hover(1.2, 1.0);
    e.click(0.8, 0.6);
    e.click(1.7, 0.6);
    let report = e.report();
    assert!(report.converged && report.dof == 6, "{report:?}");
    e.sketch().enclose::<S>().unwrap();
}

/// Circumscribed, a polygon's sides touch its circle: the second click is
/// a side's middle, and the circle's diameter is the width across flats.
#[test]
fn circumscribed_polygons_touch_their_circle() {
    let mut e = drawing();
    e.act("tool", "polygon");
    e.dialog("circumscribed", Value::Bool(true));
    e.click(1.0, 1.0);
    e.click(1.5, 1.0);
    assert_eq!(e.curves_of("line").len(), 6);
    let report = e.report();
    assert!(report.converged && report.dof == 4, "{report:?}");
    let s = e.sketch();
    let corners: Vec<f64> = e
        .drawn_points()
        .iter()
        .map(|&p| distance(xy(s, p), [1.0, 1.0]))
        .filter(|&r| r > 0.1)
        .collect();
    let across_corners = 0.5 / (std::f64::consts::PI / 6.0).cos();
    assert!(
        corners.iter().all(|r| (r - across_corners).abs() < 1e-6),
        "{corners:?}"
    );
    assert!(e.has(|c| matches!(c, Constraint::Tangent { .. })));
    e.sketch().enclose::<S>().unwrap();
}

/// A chamfer bevels a corner: both sides cut back by one distance, given
/// in place.
#[test]
fn chamfers_bevel_corners() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(1.2, 1.2);
    e.act("tool", "chamfer");
    e.click(1.2, 1.2);
    assert_eq!(e.profile_curves().len(), 5);
    assert!(e.presentation.prompt.is_some());
    e.dialog("prompt", Value::Text("0.1".into()));
    assert!(e.report().converged, "{:?}", e.report());
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
    // Wherever the solve left the corner, the bevel cuts 0.1 off each side.
    let s = e.sketch();
    let bevel = e
        .profile_curves()
        .into_iter()
        .find_map(|c| match s.curves[&c].kind {
            CurveKind::Line { start, end } if s.constraints.values().all(|k| {
                !matches!(k, Constraint::Horizontal { line } | Constraint::Vertical { line } if *line == c)
            }) => Some((xy(s, start), xy(s, end))),
            _ => None,
        })
        .expect("the bevel");
    assert!(
        (distance(bevel.0, bevel.1) - 0.1 * 2f64.sqrt()).abs() < 1e-6,
        "{bevel:?}"
    );
    e.sketch().enclose::<S>().unwrap();
}

/// An arc drawn on from a line's end, running on smoothly from it, is made
/// tangent to it; one drawn off at a corner is not.
#[test]
fn arcs_from_a_line_run_on_tangentially() {
    let mut e = drawing();
    e.lines(&[[0.0, 0.5], [1.0, 0.5]]);
    e.act("tool", "arc");
    e.click(1.0, 0.5);
    e.click(1.0, 1.5);
    e.click(1.49, 1.0);
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 1);
    assert!(e.report().converged, "{:?}", e.report());
    // At a corner: no tangency.
    e.lines(&[[3.0, 0.5], [4.0, 0.5]]);
    e.act("tool", "arc");
    e.click(4.0, 0.5);
    e.click(4.0, 1.5);
    e.click(3.6, 1.0);
    assert_eq!(e.count(|c| matches!(c, Constraint::Tangent { .. })), 1);
}

/// A plate drawn with several tools — a rectangle with a chamfered corner,
/// an arc slot and a circumscribed hexagon in it — is one region whose
/// solution is proven, as extruding it needs.
#[test]
fn a_plate_of_several_tools_is_proven() {
    let mut e = drawing();
    e.key("r");
    e.click(0.2, 0.2);
    e.click(2.6, 1.8);
    e.act("tool", "chamfer");
    e.click(2.6, 1.8);
    e.dialog("prompt", Value::Text("0.2".into()));
    e.sketch().enclose::<S>().unwrap();
    e.act("tool", "arc_slot");
    e.click(1.2, 0.6);
    e.click(1.6, 0.6);
    e.hover(1.2, 1.0);
    e.click(0.8, 0.6);
    e.click(1.7, 0.6);
    // The rectangle's four freedoms, the chamfer's none, the slot's six.
    assert_eq!(e.report().dof, 10, "{:?}", e.report());
    e.sketch().enclose::<S>().unwrap();
    e.act("tool", "polygon");
    e.dialog("circumscribed", Value::Bool(true));
    e.click(2.1, 0.7);
    e.click(2.25, 0.7);
    assert!(e.report().converged, "{:?}", e.report());
    assert_eq!(e.sketch().regions().unwrap().len(), 1);
    e.sketch().enclose::<S>().unwrap();
}

/// A mirrored arc cut back by trimming turns less than its original: it
/// keeps its ends' symmetry where they are left, but not its sweep.
#[test]
fn trimming_a_mirror_image_frees_its_sweep() {
    let mut e = drawing();
    e.act("tool", "arc");
    e.click(0.5, 0.5);
    e.click(1.5, 0.5);
    e.click(1.0, 1.0);
    e.key("Escape");
    e.act("tool", "mirror");
    e.click(0.0, 2.0);
    e.click(1.0, 1.0);
    assert!(e.has(|c| matches!(c, Constraint::EqualSweep { .. })));
    e.key("Escape");
    e.lines(&[[-1.0, 0.2], [-1.0, 1.5]]);
    e.key("m");
    e.click(-0.646, 0.854);
    assert!(!e.has(|c| matches!(c, Constraint::EqualSweep { .. })));
    assert_eq!(e.curves_of("arc").len(), 2, "{:?}", e.sketch());
    assert!(e.report().converged, "{:?}", e.report());
}
