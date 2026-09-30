//! Sketches drawn as an editor draws them: every gesture through a
//! [`StepEditor`], as rays and keys.

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Ray},
    scalars::ScalInF64 as S,
};
use geop_ops::{
    EntityRef, ORIGIN, Operations, Part,
    ui::{Control, PartView, Presentation, Reach, StepEditor},
};
use serde::{Deserialize, Serialize};

use super::*;
use crate::AddSketch;

/// The one operation, as a set an editor edits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Operations)]
#[serde(tag = "operation", content = "args", rename_all = "snake_case")]
enum Ops {
    AddSketch(AddSketchArgs),
}

/// A sketch step being edited on an empty part, as an editor drives it.
struct Editor {
    part: Part<S>,
    view: PartView<S>,
    editor: StepEditor<Ops>,
    args: AddSketchArgs,
    presentation: Presentation<S>,
}

impl Editor {
    /// A new sketch step — or, not `new`, one being edited again.
    fn new(new: bool) -> Self {
        let part = Part::new();
        let view = PartView::of(&part).unwrap();
        let step = Ops::new_step("add_sketch", &part).unwrap();
        let editor = StepEditor::new(step, &part, new);
        let presentation = editor.presentation(&part);
        let Ops::AddSketch(args) = editor.step().clone();
        Editor {
            part,
            view,
            editor,
            args,
            presentation,
        }
    }

    fn send(&mut self, event: StepEditEvent<S>) {
        self.editor.handle(&self.part, &self.view, &event);
        let Ops::AddSketch(args) = self.editor.step().clone();
        self.args = args;
        self.presentation = self.editor.presentation(&self.part);
    }

    fn session(&self) -> &SketchSession {
        self.editor.session().downcast_ref().unwrap()
    }

    fn press(&mut self, key: &str) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value: Value::Press,
        });
    }

    fn click(&mut self, x: f64, y: f64) {
        self.send(StepEditEvent::Click {
            pointer: down(x, y),
            button: Button::Primary,
            double: false,
            shift: false,
        });
    }

    fn key(&mut self, key: &str) {
        self.send(StepEditEvent::Key { key: key.into() });
    }

    fn sketch(&self) -> &Sketch {
        &self.args.sketch
    }

    /// The labels of the constraint buttons the dialog offers.
    fn constrain_options(&self) -> Vec<String> {
        match self.presentation.dialog.get("constrain") {
            Some(Control::Buttons { buttons }) => buttons.iter().map(|b| b.label.clone()).collect(),
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

/// Drawing on the default plane.
fn drawing() -> Editor {
    Editor::new(false)
}

fn close(a: P2, b: P2) -> bool {
    dist(a, b) < 1e-6
}

/// A new sketch starts by picking its plane; picking one goes straight
/// on to drawing on it, head on.
#[test]
fn a_new_sketch_picks_its_plane_first() {
    let mut e = Editor::new(true);
    assert!(matches!(
        e.presentation.dialog.get("plane"),
        Some(Control::Pick { armed: true, .. })
    ));
    assert!(e.presentation.focus.is_none());
    assert!(
        e.presentation
            .pickable
            .contains(&Target::Datum(geop_core_math::primitives::DatumKind::Plane))
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
    assert!(e.session().draft.is_none());
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
    e.send(StepEditEvent::Hover {
        pointer: down(0.8, 0.6),
    });
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
    assert_eq!(e.session().tool, Tool::Select);
    e.click(0.5, 0.6);
    e.click(0.5, 1.75);
    assert_eq!(e.session().selection.curves.len(), 2);
    let options = e.constrain_options();
    let parallel = options.iter().position(|o| o == "Parallel").unwrap();
    e.press(&format!("constrain:{parallel}"));
    assert!(
        e.sketch()
            .constraints
            .values()
            .any(|c| matches!(c, Constraint::Parallel { .. }))
    );
    assert!(e.session().selection.is_empty());

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
    e.send(StepEditEvent::Hover {
        pointer: down(1.0, 0.0),
    });
    assert!(e.presentation.grab);
    e.send(StepEditEvent::Drag {
        from: down(1.0, 0.0),
        to: down(2.0, 0.4),
        done: true,
    });
    let s = e.sketch();
    let ends: Vec<P2> = s.points.values().map(|p| p.xy()).collect();
    // The line starts fixed at the origin and is horizontal by
    // constraint: the end follows the pointer along it, not up.
    assert!(ends.iter().any(|&p| close(p, [0.0, 0.0])), "{ends:?}");
    assert!(ends.iter().any(|&p| close(p, [2.0, 0.0])), "{ends:?}");
    assert!(e.session().drag.is_none());
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
    e.send(StepEditEvent::Dialog {
        key: format!("constraint:{}", id.0),
        value: Value::Number(2.0),
    });
    let s = e.sketch();
    let ends: Vec<P2> = s.points.values().map(|p| p.xy()).collect();
    assert!((dist(ends[0], ends[1]) - 2.0).abs() < 1e-6, "{ends:?}");
}

/// Picking another plane moves the drawing onto it, kept as drawn.
#[test]
fn another_plane_keeps_the_drawing() {
    let mut e = drawing();
    e.key("c");
    e.click(0.0, 0.0);
    e.click(0.5, 0.0);
    assert_eq!(e.sketch().curves.len(), 1);
    e.press("plane");
    assert!(e.presentation.focus.is_none());
    e.send(StepEditEvent::Click {
        pointer: pointer([0.04, -10.0, 0.04], [0.0, 1.0, 0.0]),
        button: Button::Primary,
        double: false,
        shift: false,
    });
    assert_eq!(e.sketch().curves.len(), 1);
    let focus = e.presentation.focus.expect("drawing faces the new plane");
    assert_eq!(*focus.w(), v([0.0, 1.0, 0.0]));
}
