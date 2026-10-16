//! 3-D sketches drawn as an editor draws them — every gesture a command
//! to the [`geop_core_editor::editor::Editor`], as rays and keys — and
//! built on a part: an extruded box, so an integration test, as the
//! extrusion depends on this crate.

use geop_core_editor::editing::{Button, Pointer, Reach, Shape, StepEditEvent, Style, Value};
use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::Program, values::Design};
use geop_core_sketch::space::{Constraint3d, CurveKind3d};
use geop_ops_extrude_revolve::Extents;
use geop_ops_model::Combine;
use geop_ops_model::{NamedModel, read_model};
use geop_ops_sketch3d::{
    AddSketch3dArgs, PlacedSketch3d, PointSource, Reference3d, Sketch3d, Sketch3dEditing,
    Sketch3dSession, Target, Tool, sketch3d,
};

use geop_core_editor::{
    registry::Registry,
    testing::{Stepping, editor_with, pointer as reaching},
};
use geop_ops_extrude_revolve::testing::{extrude_sketch, on_origin, rectangle};

/// What a 3-D sketch is drawn among: the model's, the sketches', the
/// datums' and the extrusions' operations and drawers.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(geop_ops_sketch3d::register)
        .and_then(geop_ops_extrude_revolve::register)
        .unwrap()
}

/// An editor of [`registry`] on `program`, which loads without an error.
fn loaded(program: Program) -> geop_core_editor::editor::Editor {
    editor_with(registry(), program)
}

fn close(a: &Vector3<S>, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// A pointer from `origin` along `dir`, reaching a hundredth of a unit.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    Pointer {
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
        ..reaching(origin, dir)
    }
}

/// Looking straight down at `(x, y)`.
fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// A 3-D sketch step being drawn as a front end draws it, with what the
/// editor last showed of it.
struct Editor {
    step: Stepping,
    presentation: geop_core_editor::editing::Presentation<S>,
}

impl Editor {
    /// A new 3-D sketch — after the box `[1, 2] x [1, 2] x [0, 1]` if
    /// `on_box`, what it is drawn on.
    fn on(on_box: bool) -> Self {
        let mut program = Program::new();
        if on_box {
            let square = rectangle(1.0, 1.0, 2.0, 2.0);
            let (extent, combine) = (Extents::blind(1.0), Combine::NewBody);
            extrude_sketch(
                &mut program,
                "box",
                on_origin(FrameAxis::Z),
                square,
                extent,
                combine,
            );
        }
        let step = Stepping::new(loaded(program), "add_sketch3d");
        let presentation = step.presentation().clone();
        Editor { step, presentation }
    }

    fn send(&mut self, event: StepEditEvent<S>) {
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

    fn click(&mut self, pointer: Pointer<S>) {
        self.send(StepEditEvent::Click {
            pointer,
            button: Button::Primary,
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
    fn at(&self, id: geop_core_sketch::PointId) -> Vector3<S> {
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
    fn built(&self) -> (Sketch3d, NamedModel<Design>) {
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
    let geometry = sketch.enclose::<S>().unwrap();
    let t = sketch.curve_nurbs(spline, &geometry).unwrap()[0]
        .tangent(S::ZERO)
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
            &geop_core_sketch::space::Enclosure3d::<S>::as_drawn(&sketch),
        )
        .unwrap();
    let mid = pieces[0].evaluate(S::from_f64(0.5)).unwrap();
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
        button: Button::Primary,
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
