//! 3-D sketches drawn as an editor draws them — every gesture through a
//! [`StepEditor`], as rays and keys — and built on a part.

use geop_core_math::{
    primitives::{CoordinateSystem, Datum, DatumKind, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_sketch::space::{Constraint3d, CurveKind3d};
use geop_ops::{
    Context, EntityRef, NoFiles, Operation, Operations, Part,
    ui::{Button, PartView, Pointer, Presentation, Reach, StepEditEvent, StepEditor, Value},
};
use serde::{Deserialize, Serialize};

use crate::{AddSketch3d, AddSketch3dArgs, Reference3d, Sketch3d, Sketch3dSession, Target};

/// The one operation, as a set an editor edits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
enum Ops {
    #[operation(group = Sketch)]
    AddSketch3d(AddSketch3dArgs),
}

fn v(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

fn close(a: &Vector3<S>, b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k].to_f64() - b[k]).abs() < 1e-7)
}

/// A pointer from `origin` along `dir`, reaching a hundredth of a unit.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
    }
}

/// Looking straight down at `(x, y)`.
fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// The part with the box `[1, 2] x [1, 2] x [0, 1]`, `cube(b)`.
fn boxed() -> Part<S> {
    let mut part = Part::new();
    geop_ops_extrude_revolve::shapes::cube::cube_solid(
        &mut part,
        "b",
        v([1.0, 1.0, 0.0]),
        v([2.0, 2.0, 1.0]),
    )
    .unwrap();
    part
}

/// A 3-D sketch step being edited on a part, as an editor drives it.
struct Editor {
    part: Part<S>,
    view: PartView<S>,
    editor: StepEditor<Ops, S>,
    presentation: Presentation<S>,
}

impl Editor {
    fn on(part: Part<S>) -> Self {
        let view = PartView::of(&part).unwrap();
        let step = Ops::new_step("add_sketch3d", &part).unwrap();
        let context = Context::new(&part, "route", &NoFiles);
        let editor = StepEditor::new(step, context, true);
        let presentation = editor.presentation(context, &part, &view);
        Editor {
            part,
            view,
            editor,
            presentation,
        }
    }

    fn args(&self) -> AddSketch3dArgs {
        let Ops::AddSketch3d(args) = self.editor.step().clone();
        args
    }

    fn sketch(&self) -> Sketch3d {
        self.args().sketch
    }

    fn send(&mut self, event: StepEditEvent<S>) {
        let context = Context::new(&self.part, "route", &NoFiles);
        self.editor.handle(context, &self.view, &event);
        self.presentation = self.editor.presentation(context, &self.part, &self.view);
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

    fn session(&self) -> &Sketch3dSession {
        self.editor.session().downcast_ref().unwrap()
    }

    /// Where the point `id` of the sketch is.
    fn at(&self, id: geop_core_sketch::PointId) -> Vector3<S> {
        self.sketch().points[&id].at.map(|c| c.cast())
    }

    /// The sketch's points, oldest first.
    fn points(&self) -> Vec<geop_core_sketch::PointId> {
        self.sketch().points.keys().copied().collect()
    }

    /// The step built on the part.
    fn built(&self) -> Part<S> {
        AddSketch3d
            .apply(self.part.clone(), "route", &self.args(), &NoFiles)
            .unwrap()
    }
}

/// A click places a point where the pointer hits the part: at a vertex, a
/// point fixed there; on an edge, a point constrained onto it; on a face,
/// where the face is hit; off the part, in the plane through the last
/// point facing the eye. A new sketch starts with the line tool, which
/// chains the points with lines.
#[test]
fn clicks_place_points_on_the_part() {
    let mut e = Editor::on(boxed());
    assert_eq!(e.session().tool(&e.args()), crate::Tool::Line);
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
        Some(Reference3d {
            entity: EntityRef::Vertex { .. },
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
    assert_eq!(e.session().tool(&e.args()), crate::Tool::Line);
    e.key("Escape");
    assert_eq!(e.session().tool(&e.args()), crate::Tool::Select);

    let built = e.built();
    let id = built.sketch3d_id("route").unwrap();
    assert_eq!(built.sketch3d(id).unwrap().chains().unwrap().len(), 1);
    built.check_names().unwrap();
}

/// A spline started at the end of a line goes on smoothly from it, and
/// typed coordinates place points as clicks do.
#[test]
fn spline_goes_on_smoothly_from_a_line() {
    let mut e = Editor::on(Part::new());
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
    let mut e = Editor::on(Part::new());
    e.click(pointer([1.0, -10.0, 0.0], [0.0, 1.0, 0.0]));
    e.click(pointer([1.5, -10.0, 2.0], [0.0, 1.0, 0.0]));
    e.key("Escape");
    e.key("Escape");
    // Click the line between its ends: selected.
    e.click(pointer([1.25, -10.0, 1.0], [0.0, 1.0, 0.0]));
    let line = *e.sketch().curves.keys().next().unwrap();
    assert_eq!(e.editor.selection(), [line.to_string()]);
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

/// A point fixed at a datum point follows it when the datum moves, and a
/// line along a datum axis turns with it.
#[test]
fn references_follow_the_part() {
    let with_datums = |at: [f64; 3], axis: [f64; 3]| {
        let mut part = Part::<S>::new();
        let point = Datum {
            kind: DatumKind::Point,
            frame: CoordinateSystem::world_at(v(at)),
        };
        part.add_datum(point, "d").unwrap();
        let w = v(axis).normalize().unwrap();
        let frame = geop_ops::operation::frame_along(Vector3::zero(), &w).unwrap();
        part.add_datum(
            Datum {
                kind: DatumKind::Axis,
                frame,
            },
            "a",
        )
        .unwrap();
        part
    };
    let mut sketch = Sketch3d::new();
    let n = |x: f64| geop_ops::Design::from_f64(x);
    let p = sketch.add_fixed_point(Vector3::from_array([1.0, 0.0, 0.0].map(n)));
    let q = sketch.add_point(Vector3::from_array([1.0, 1.0, 0.1].map(n)));
    let line = sketch.add_line(p, q);
    let along = sketch.constrain(Constraint3d::ParallelTo {
        line,
        direction: Vector3::from_array([0.0, 1.0, 0.0].map(n)),
    });
    let args = AddSketch3dArgs {
        sketch,
        references: vec![
            Reference3d {
                entity: EntityRef::datum("d"),
                target: Target::Point { point: p },
            },
            Reference3d {
                entity: EntityRef::datum("a"),
                target: Target::Direction { constraint: along },
            },
        ],
    };
    let at = |part: Part<S>| {
        let built = AddSketch3d.apply(part, "route", &args, &NoFiles).unwrap();
        let s = built.sketch3d(built.sketch3d_id("route").unwrap()).unwrap();
        (
            s.points[&p].at.map(|c| c.cast::<S>()),
            s.points[&q].at.map(|c| c.cast::<S>()),
        )
    };
    let (a, b) = at(with_datums([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]));
    assert!(close(&a, [1.0, 0.0, 0.0]));
    // Not along the axis as drawn: solved onto it.
    assert!(close(&b, [1.0, 1.0, 0.0]), "{b:?}");
    let (a, b) = at(with_datums([2.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    assert!(close(&a, [2.0, 0.0, 0.0]));
    assert!(
        (b[0].to_f64() - 2.0).abs() < 1e-9 && b[1].abs().to_f64() < 1e-9,
        "{b:?}"
    );
}

/// The arc tool chains arcs, each through three clicks.
#[test]
fn arcs_take_three_clicks() {
    let mut e = Editor::on(Part::new());
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
    assert_eq!(e.session().tool(&e.args()), crate::Tool::Arc);
}
