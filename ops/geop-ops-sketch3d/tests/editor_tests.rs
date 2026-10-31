//! 3-D sketches drawn as an editor draws them — every gesture a command
//! to the [`geop_core_editor::editor::Editor`], as rays and keys — and
//! built on a part: a given block (`geop_lib_fixtures`). Its points are
//! dragged by the gizmo, and its tools taken and put down by the keys
//! every tool shares.

use geop_core_editor::editing::{
    Axis, GizmoPart, Pointer, Reach, Shape, StepEditEvent, Style, Value,
};
use geop_core_math::{scalars::Scalar, vector::Vector3};
use geop_core_program::{element::ElementReference, program::Program, testing::args};
use geop_core_sketch::space::{Constraint3d, CurveKind3d};
use geop_lib_fixtures::{PrimitiveArgs, drilled_box};
use geop_ops_model::sketch3d::{PlacedSketch3d, PointSource, Sketch3d, sketch3d};
use geop_ops_model::{NamedModel, read_model};
use geop_ops_sketch3d::{
    AddSketch3dArgs, Reference3d, Sketch3dEditing, Sketch3dSession, Target, Tool,
};

use geop_core_editor::{
    editor::Command,
    registry::Registry,
    testing::{
        Gesture, Stepping, click_at, drag_gizmo, editor_with, event, from_above, handled,
        pointer as reaching,
    },
};
use geop_core_math::primitives::Ray;
use std::collections::BTreeMap;

/// What a 3-D sketch is drawn among: the model's and the 3-D sketches'
/// operations and drawers, and the given solids it is drawn on.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch3d::register)
        .and_then(geop_lib_fixtures::register)
        .unwrap()
}

/// An editor of [`registry`], with no other files and nothing loaded.
fn editor() -> geop_core_editor::editor::Editor {
    geop_core_editor::editor::Editor::new(registry(), BTreeMap::new())
}

/// An editor of [`registry`] on `program`, which loads without an error.
fn loaded(program: Program) -> geop_core_editor::editor::Editor {
    editor_with(registry(), program)
}

fn close(a: &Vector3, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// A pointer from `origin` along `dir`, reaching a hundredth of a unit.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer {
    Pointer {
        cycle: 0,
        reach: Reach::Tube {
            radius: Scalar::from_f64(0.01),
        },
        ..reaching(origin, dir)
    }
}

/// Looking straight down at `(x, y)`.
fn down(x: f64, y: f64) -> Pointer {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// A 3-D sketch step being drawn as a front end draws it, with what the
/// editor last showed of it.
struct Editor {
    step: Stepping,
    presentation: geop_core_editor::editing::Presentation,
}

impl Editor {
    /// A new 3-D sketch — after the box `[1, 2] x [1, 2] x [0, 1]` if
    /// `on_box`, what it is drawn on.
    fn on(on_box: bool) -> Self {
        let mut program = Program::new();
        if on_box {
            program.push(
                "box",
                PrimitiveArgs::block([1.0, 1.0, 0.0], [2.0, 2.0, 1.0]),
            );
        }
        let step = Stepping::new(loaded(program), "add_sketch3d");
        let presentation = step.presentation().clone();
        Editor { step, presentation }
    }

    fn send(&mut self, event: StepEditEvent) {
        self.step.send(event);
        self.presentation = self.step.presentation().clone();
    }

    fn args(&self) -> AddSketch3dArgs {
        self.step.args()
    }

    fn sketch(&self) -> Sketch3d {
        self.args().sketch
    }

    fn editing(&self) -> &Sketch3dEditing {
        self.step.editing()
    }

    fn session(&self) -> &Sketch3dSession {
        &self.editing().session
    }

    fn click(&mut self, pointer: Pointer) {
        self.send(StepEditEvent::Click {
            pointer,
            double: false,
            shift: false,
        });
    }

    fn key(&mut self, key: &str) {
        self.send(StepEditEvent::Key { key: key.into() });
    }

    fn dialog(&mut self, key: &str, value: Value) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value,
        });
    }

    fn act(&mut self, key: &str, value: &str) {
        self.dialog(key, Value::Choice(value.into()));
    }

    /// Where the point `id` of the sketch is.
    fn at(&self, id: geop_core_sketch::PointId) -> Vector3 {
        self.sketch().points[&id].at
    }

    /// The sketch's points, oldest first.
    fn points(&self) -> Vec<geop_core_sketch::PointId> {
        self.sketch().points.keys().copied().collect()
    }

    /// The step's id, which its sketch is named by.
    fn id(&self) -> String {
        self.step
            .editor
            .editing()
            .expect("a step is edited")
            .id
            .clone()
    }

    /// The 3-D sketch the step built, and the model with its wire.
    fn built(&self) -> (Sketch3d, NamedModel) {
        let part = self.step.editor.part();
        let whole = ElementReference::<_, PlacedSketch3d>::whole(sketch3d(&self.id()));
        let sketch = part.retrieve_element(&whole).unwrap().sketch.clone();
        (sketch, read_model(&part).unwrap().into_owned())
    }
}

/// A click places a point where the pointer hits the part: at a vertex, a
/// point fixed there; on an edge, a point constrained onto it; on a face,
/// where the face is hit; off the part, in the plane through the last
/// point facing the eye. A new sketch starts with the line tool, which
/// chains the points with lines. Built, the sketch is one chain, and a
/// wire of the part whose edges and vertices are named after it.
#[test]
fn clicks_place_points_on_the_part() {
    let mut e = Editor::on(true);
    assert_eq!(e.session().tool(&e.args()), Tool::Line);
    e.click(down(2.0, 2.0));
    e.click(pointer([1.4, 10.0, 1.0], [0.0, -1.0, 0.0]));
    e.click(down(1.6, 1.5));
    e.click(down(4.0, 5.0));
    e.key("Escape");
    let sketch = e.sketch();
    let p = e.points();
    assert_eq!(p.len(), 4);
    // At the corner, fixed there by the vertex.
    assert!(sketch.points[&p[0]].fixed);
    assert!(close(&e.at(p[0]), [2.0, 2.0, 1.0]));
    let args = e.args();
    assert!(matches!(
        args.reference_of(Target::Point { point: p[0] }),
        Some(Reference3d::Point {
            from: PointSource::Vertex(_),
            ..
        })
    ));
    // On the top's back edge, `y = 2, z = 1`, where it was clicked.
    assert!(close(&e.at(p[1]), [1.4, 2.0, 1.0]), "{:?}", e.at(p[1]));
    assert!(sketch.constraints.values().any(|c| matches!(
        c,
        Constraint3d::OnCurve { point, .. } if *point == p[1]
    )));
    // On the top, and then off the box in the top's plane.
    assert!(close(&e.at(p[2]), [1.6, 1.5, 1.0]));
    assert!(close(&e.at(p[3]), [4.0, 5.0, 1.0]));
    let lines = sketch
        .curves
        .values()
        .filter(|c| matches!(c.kind, CurveKind3d::Line { .. }))
        .count();
    assert_eq!(lines, 3);
    assert_eq!(e.session().tool(&e.args()), Tool::Line);
    e.key("Escape");
    assert_eq!(e.session().tool(&e.args()), Tool::Select);

    let (built, model) = e.built();
    assert_eq!(built.chains().unwrap().len(), 1);
    model.check_names().unwrap();
    let line = sketch
        .curves
        .iter()
        .find(|(_, c)| c.is_drawn())
        .map(|(id, _)| *id)
        .unwrap();
    assert!(
        model
            .edge_id(&format!("sketch3d({},{line})", e.id()))
            .is_ok()
    );
    assert!(
        model
            .vertex_id(&format!("sketch3d({},{})", e.id(), p[3]))
            .is_ok()
    );
}

/// A spline started at the end of a line goes on smoothly from it, and
/// typed coordinates place points as clicks do.
#[test]
fn spline_goes_on_smoothly_from_a_line() {
    let mut e = Editor::on(false);
    e.click(pointer([1.0, -10.0, 0.0], [0.0, 1.0, 0.0]));
    e.click(pointer([2.0, -10.0, 0.0], [0.0, 1.0, 0.0]));
    e.key("s");
    // On the line's end, the spline starts at it.
    e.click(pointer([2.0, -10.0, 0.0], [0.0, 1.0, 0.0]));
    for (key, value) in [("x", 3.0), ("y", 1.0), ("z", 1.0)] {
        e.dialog(key, Value::Number(value));
    }
    e.act("place", "place");
    e.click(pointer([4.0, -10.0, 2.0], [0.0, 1.0, 0.0]));
    e.key("Enter");
    let sketch = e.sketch();
    let spline = sketch
        .curves
        .iter()
        .find(|(_, c)| matches!(c.kind, CurveKind3d::Spline { .. }))
        .map(|(&id, _)| id)
        .expect("a spline");
    let CurveKind3d::Spline { points } = &sketch.curves[&spline].kind else {
        unreachable!()
    };
    assert_eq!(points.len(), 3);
    assert!(close(&e.at(points[1]), [3.0, 1.0, 1.0]));
    assert!(
        close(&e.at(points[2]), [4.0, 1.0, 2.0]),
        "{:?}",
        e.at(points[2])
    );
    assert!(
        sketch
            .constraints
            .values()
            .any(|c| matches!(c, Constraint3d::Tangent { .. }))
    );
    let geometry = sketch.enclose().unwrap();
    let t = sketch.curve_nurbs(spline, &geometry).unwrap()[0]
        .tangent(Scalar::ZERO)
        .unwrap();
    assert!(
        t[0].to_f64() > 0.0 && t[1].abs().to_f64() < 1e-9 && t[2].abs().to_f64() < 1e-9,
        "{t:?}"
    );
}

/// A line selected is made to run along an axis; a point is dragged, and
/// follows as far as the constraints let it.
#[test]
fn selections_are_constrained_and_points_dragged() {
    let mut e = Editor::on(false);
    e.click(pointer([1.0, -10.0, 0.0], [0.0, 1.0, 0.0]));
    e.click(pointer([1.5, -10.0, 2.0], [0.0, 1.0, 0.0]));
    e.key("Escape");
    e.key("Escape");
    // Click the line between its ends: selected.
    e.click(pointer([1.25, -10.0, 1.0], [0.0, 1.0, 0.0]));
    let line = *e.sketch().curves.keys().next().unwrap();
    assert_eq!(e.editing().selection, [line.to_string()]);
    e.act("constrain", "along_z");
    let p = e.points();
    let (a, b) = (e.at(p[0]), e.at(p[1]));
    assert!(
        (a[0].to_f64() - b[0].to_f64()).abs() < 1e-9
            && (a[1].to_f64() - b[1].to_f64()).abs() < 1e-9,
        "{a:?} {b:?}"
    );
    // Dragged sideways, the top end takes the line with it, upright.
    let grab = e.at(p[1]).to_array().map(|c| c.to_f64());
    let from = pointer([grab[0], -10.0, grab[2]], [0.0, 1.0, 0.0]);
    e.send(StepEditEvent::Hover {
        pointer: from,
        shift: false,
    });
    assert!(e.presentation.grab);
    e.send(StepEditEvent::Drag {
        from,
        to: pointer([grab[0] + 1.0, -10.0, grab[2] + 1.0], [0.0, 1.0, 0.0]),
        done: true,
        shift: false,
    });
    let (a, b) = (e.at(p[0]), e.at(p[1]));
    assert!((b[2].to_f64() - grab[2] - 1.0).abs() < 1e-6, "{b:?}");
    assert!((a[0].to_f64() - b[0].to_f64()).abs() < 1e-9, "{a:?} {b:?}");
}

/// The arc tool chains arcs, each through three clicks.
#[test]
fn arcs_take_three_clicks() {
    let mut e = Editor::on(false);
    e.key("a");
    let along_y = |x: f64, z: f64| pointer([x, -10.0, z], [0.0, 1.0, 0.0]);
    e.click(along_y(0.0, 0.0));
    e.click(along_y(
        1.0 - std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
    ));
    e.click(along_y(1.0, 1.0));
    let sketch = e.sketch();
    let arcs: Vec<_> = sketch
        .curves
        .iter()
        .filter(|(_, c)| matches!(c.kind, CurveKind3d::Arc { .. }))
        .collect();
    assert_eq!(arcs.len(), 1);
    let (&arc, _) = arcs[0];
    let pieces = sketch
        .curve_nurbs(
            arc,
            &geop_core_sketch::space::Enclosure3d::as_drawn(&sketch),
        )
        .unwrap();
    let mid = pieces[0].evaluate(Scalar::from_f64(0.5)).unwrap();
    // A quarter circle about (1, 0, 0) in the y = 0 plane.
    let r = (mid[0].to_f64() - 1.0).hypot(mid[2].to_f64());
    assert!((r - 1.0).abs() < 1e-9, "{mid:?}");
    // The next arc starts where this one ended.
    assert_eq!(e.session().tool(&e.args()), Tool::Arc);
}

/// Drawing a line, the pointer passing near the line along `x` through the
/// line's start snaps the end onto it: shown as the axis, labelled, with
/// the arrows of `x`, `y` and `z` at the start — and, clicked, the line is
/// made parallel to `x`. With shift held nothing snaps. Back on the first
/// point, the chain closes on it.
#[test]
fn lines_snap_along_axes_and_onto_points() {
    let mut e = Editor::on(false);
    let side = |x: f64, z: f64| pointer([x, -10.0, z], [0.0, 1.0, 0.0]);
    e.click(side(1.0, 1.0));
    // Half a reach above the line along `x` through (1, 0, 1).
    e.send(StepEditEvent::Hover {
        pointer: side(3.0, 1.005),
        shift: false,
    });
    let visuals = &e.presentation.visuals;
    let shown = |key: &str| visuals.iter().find(|v| v.key == key);
    assert!(
        matches!(shown("snap_label"), Some(v) if matches!(&v.shape, Shape::Label { text, .. } if text == "x")),
        "{visuals:?}"
    );
    assert!(matches!(shown("snap"), Some(v) if v.style == Style::Snap));
    assert!(
        matches!(shown("triad"), Some(v) if matches!(v.shape, Shape::Triad { at, .. } if close(&at, [1.0, 0.0, 1.0])))
    );
    e.click(side(3.0, 1.005));
    let p = e.points();
    assert!(close(&e.at(p[1]), [3.0, 0.0, 1.0]), "{:?}", e.at(p[1]));
    let along = |sketch: &Sketch3d, axis: usize| {
        sketch.constraints.values().any(|c| {
            matches!(c, Constraint3d::ParallelTo { direction, .. } if direction[axis].to_f64() == 1.0)
        })
    };
    assert!(along(&e.sketch(), 0));

    // Up along `z` — with shift, it stays where it is clicked, unaligned.
    e.send(StepEditEvent::Click {
        pointer: side(3.004, 2.0),
        double: false,
        shift: true,
    });
    let p = e.points();
    assert!(close(&e.at(p[2]), [3.004, 0.0, 2.0]), "{:?}", e.at(p[2]));
    assert!(!along(&e.sketch(), 2));

    // Back to the first point: the chain ends on it, a loop.
    e.send(StepEditEvent::Hover {
        pointer: side(1.004, 1.0),
        shift: false,
    });
    assert!(
        e.presentation
            .visuals
            .iter()
            .any(|v| v.key == "snap" && v.style == Style::Snap)
    );
    e.click(side(1.004, 1.0));
    e.key("Escape");
    let sketch = e.sketch();
    assert_eq!(sketch.points.len(), 3);
    let chains = sketch.chains().unwrap();
    assert!(chains.len() == 1 && chains[0].closed, "{chains:?}");
}

/// A point of a 3-D sketch, selected, is moved by its gizmo along `x`
/// only: the `x` arrow dragged up and off to the side moves it along
/// `x`, as far as the pointer went along it.
#[test]
fn a_3d_sketch_point_is_dragged_along_an_axis() {
    let mut editor = editor();
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let click = |editor: &mut geop_core_editor::editor::Editor, origin: [f64; 3], dir: [f64; 3]| {
        event(
            editor,
            StepEditEvent::Click {
                pointer: reaching(origin, dir),
                double: false,
                shift: false,
            },
        )
    };
    // A line from the origin to (2, 0, 2), seen from the front.
    click(&mut editor, [0.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    click(&mut editor, [2.0, -10.0, 2.0], [0.0, 1.0, 0.0]);
    let key = |editor: &mut geop_core_editor::editor::Editor| {
        event(
            editor,
            StepEditEvent::Key {
                key: "Escape".into(),
            },
        )
    };
    key(&mut editor);
    let step = key(&mut editor);
    assert!(step.presentation.gizmo.is_none(), "nothing selected");
    let step = click(&mut editor, [2.0, -10.0, 2.0], [0.0, 1.0, 0.0]);
    let gizmo = step.presentation.gizmo.expect("the point has a gizmo");
    assert!(gizmo.modes.translate && !gizmo.modes.rotate && !gizmo.modes.scale);

    // Its `x` arrow, grabbed from the front, dragged by 0.7 along `x` —
    // and up, which an arrow does not follow.
    let front = |x: f64, z: f64| reaching([x, -10.0, z], [0.0, 1.0, 0.0]);
    let step = drag_gizmo(
        &mut editor,
        GizmoPart::Move(Axis::X),
        front(2.05, 2.0),
        front(2.75, 2.4),
        front(2.4, 2.2),
        false,
    );
    assert_eq!(step.error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    let mut points: Vec<[f64; 3]> = args
        .sketch
        .points
        .values()
        .map(|p| [0, 1, 2].map(|k| p.at[k].to_f64()))
        .collect();
    points.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let moved = points.last().unwrap();
    assert!(
        (moved[0] - 2.7).abs() < 1e-9 && moved[1].abs() < 1e-9 && (moved[2] - 2.0).abs() < 1e-9,
        "{points:?}"
    );
    assert!(points[0].iter().all(|c| c.abs() < 1e-9), "{points:?}");
}

/// A 3-D sketch drawn by clicks from the side: the second point, clicked
/// near the line along `x` through the first, snaps onto it; the third,
/// near the line along `z` through the second, onto that; the fourth, near
/// the first point, is the first point. Committed, the sketch is one loop
/// with two lines parallel to the axes they snapped to.
#[test]
fn a_3d_sketch_snaps_along_axes_and_onto_points() {
    let mut editor = loaded(Program::new());
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let side = |x: f64, z: f64| reaching([x, -10.0, z], [0.0, 1.0, 0.0]);
    editor.handle(click_at(side(1.0, 1.0)));
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(4.0, 1.006),
            shift: false,
        },
    });
    let visuals = update.step.unwrap().presentation.visuals;
    assert!(
        visuals.iter().any(|v| matches!(
            &v.shape,
            Shape::Label { text, .. } if text == "x"
        )),
        "the snap to x is shown: {visuals:?}"
    );
    editor.handle(click_at(side(4.0, 1.006)));
    editor.handle(click_at(side(4.007, 3.0)));
    editor.handle(click_at(side(1.005, 1.0)));
    editor.handle(Command::Event {
        event: StepEditEvent::Key {
            key: "Escape".into(),
        },
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    let sketch = &args.sketch;
    assert_eq!(sketch.points.len(), 3);
    let parallel: Vec<usize> = sketch
        .constraints
        .values()
        .filter_map(|c| match c {
            Constraint3d::ParallelTo { direction, .. } => {
                (0..3).find(|&k| direction[k].to_f64() == 1.0)
            }
            _ => None,
        })
        .collect();
    assert_eq!(parallel, [0, 2]);
    let chains = sketch.chains().unwrap();
    assert!(chains.len() == 1 && chains[0].closed, "{chains:?}");
}

/// Drawing a 3-D sketch on a part picks what the pointer is over in the
/// part as the editor drew it when the step was opened: no hover draws
/// the part again, however many there are, and a click only as the editor
/// shows the sketch it changed — once.
#[test]
fn a_3d_sketch_picks_without_drawing_the_part_again() {
    let mut editor = loaded(drilled_box());
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let drawn = geop_core_editor::view::parts_drawn();
    for k in 0..5 {
        let at = from_above(0.2 + 0.1 * k as f64, 0.3);
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Hover {
                pointer: at,
                shift: false,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    assert_eq!(geop_core_editor::view::parts_drawn() - drawn, 0);
    let update = editor.handle(click_at(from_above(0.5, 0.5)));
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(geop_core_editor::view::parts_drawn() - drawn, 1);
    // The click was on the box's top: the point is there.
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let steps = &editor.program().steps;
    let step = steps.iter().find(|s| s.operation == "add_sketch3d");
    let args: AddSketch3dArgs = args(step.unwrap());
    let points: Vec<[f64; 3]> = (args.sketch.points.values())
        .map(|p| p.at.to_array().map(|c| c.to_f64()))
        .collect();
    // Where the click meets the face as drawn: its mesh meets the hole's
    // wall along the rim's samples, points of the rim's curve, which lie
    // in the top's plane to the last bits of their enclosures.
    assert_eq!(points.len(), 1);
    let off = (0..3).map(|k| (points[0][k] - [0.5, 0.5, 1.0][k]).abs());
    assert!(off.fold(0.0, f64::max) < 1e-12, "{points:?}");
}

/// The default view of an empty part, as the browser sends it — 1920 x
/// 1080, the camera at `(66, 44, 88)` looking at the origin: a line's
/// first click on the origin, its second 160 px right of and 90 px above
/// it. Both place a point, and the line between them is drawn. (Reported
/// as placing nothing: in a 1400 x 900 window that spot lies under the
/// step's dialog, and the viewer sends no click there at all.)
#[test]
fn a_3d_line_is_drawn_from_the_origin_in_the_default_view() {
    let mut editor = loaded(Program::new());
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let eye = |dir: [f64; 3]| {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
        Pointer {
            cycle: 0,
            ray: Ray::try_new(
                v([65.90889047678681, 43.93926031785788, 87.87852063571576]),
                v(dir),
            )
            .unwrap(),
            reach: Reach::Cone {
                slope: Scalar::from_f64(0.00838515269409588),
            },
        }
    };
    for dir in [
        [
            -0.5570860145310995,
            -0.3713906763541206,
            -0.7427813527082412,
        ],
        [
            -0.4499813577124772,
            -0.2893350678973743,
            -0.8448680347817981,
        ],
    ] {
        let hover = editor.handle(Command::Event {
            event: StepEditEvent::Hover {
                pointer: eye(dir),
                shift: false,
            },
        });
        let visuals = hover.step.unwrap().presentation.visuals;
        assert!(
            visuals.iter().any(|v| v.key == "snap"),
            "where the click goes is shown: {visuals:?}"
        );
        editor.handle(click_at(eye(dir)));
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.sketch.curves.len(), 1, "{:?}", args.sketch);
}

/// The curves of the 3-D sketch drawn, committed, each as it reads.
fn drawn_curves_3d(editor: &mut geop_core_editor::editor::Editor) -> Vec<String> {
    handled(editor, Command::Commit);
    let sketch: AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    (sketch.sketch.curves.values())
        .filter(|c| c.is_drawn() && !c.construction)
        .map(|c| format!("{c:?}"))
        .collect()
}

/// The 3-D sketch ends its chain as the planar one does
/// ([`every_drawing_tool_ends_its_chain_the_same_way`]), its line tool
/// taken by its key in either case.
#[test]
fn every_3d_drawing_tool_ends_its_chain_the_same_way() {
    for gesture in [Gesture::Enter, Gesture::Escape] {
        for key in ["l", "L"] {
            let mut editor = editor();
            editor.handle(Command::New {
                kind: "add_sketch3d".into(),
            });
            event(&mut editor, StepEditEvent::Key { key: key.into() });
            let click = |editor: &mut geop_core_editor::editor::Editor, x: f64, y: f64| {
                editor.handle(click_at(from_above(x, y)));
            };
            click(&mut editor, 0.0, 0.0);
            click(&mut editor, 1.0, 0.0);
            gesture.send(&mut editor);
            click(&mut editor, 0.0, 1.0);
            click(&mut editor, 1.0, 1.0);
            let curves = drawn_curves_3d(&mut editor);
            assert_eq!(curves.len(), 2, "3-D, {gesture:?}, {key:?}: {curves:#?}");
        }
    }
}

/// A tool's key is the same typed with shift or caps lock: in the 3-D
/// sketch, `A` takes the arc tool as `a` does.
#[test]
fn a_tools_key_is_the_same_typed_in_capitals() {
    for key in ["a", "A"] {
        let mut editor = editor();
        editor.handle(Command::New {
            kind: "add_sketch3d".into(),
        });
        event(&mut editor, StepEditEvent::Key { key: key.into() });
        for (x, y) in [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)] {
            editor.handle(click_at(from_above(x, y)));
        }
        let curves = drawn_curves_3d(&mut editor);
        assert!(
            curves.len() == 1 && curves[0].contains("Arc"),
            "{key:?}: {curves:#?}"
        );
    }
}
