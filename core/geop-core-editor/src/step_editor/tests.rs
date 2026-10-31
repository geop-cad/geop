//! The step editor driving an operation as an editor does: picks,
//! handles, selection and keys turned into the operation's events.

use crate::{
    editable::{Context, Editable, Editors, OperationGroup, OperationInfo, OperationTier},
    editing::{
        CanvasEvent, Control, Event, Form, Number, Picked, Pointer, Presentation, Preview, Reach,
        Shape, StepEditEvent, Style, Track, Unit, Value, Visual,
        hit::{Cycle, ElementHit, Scene},
    },
};
use geop_core_math::{geop_error::GeopResult, primitives::Ray, scalars::Scalar, vector::Vector3};
use geop_core_program::{
    element::PickedElement,
    numeric::NumericInput,
    operation::Operation,
    part::{Part, StepContext},
    program::{Program, ProgramStep},
    runner::Runner,
};
use serde::{Deserialize, Serialize};

use super::{Editing, StepEditor};

/// What a test operation is to an editor: only its name matters.
const INFO: OperationInfo = OperationInfo::new("", "", OperationGroup::Solid, OperationTier::Menu);

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct MarkArgs {
    on: Vec<PickedElement>,
    distance: NumericInput,
    /// The solid marked, as a face of it picks it.
    #[serde(default)]
    body: Vec<PickedElement>,
}

/// What a mark keeps while edited: the selection, and the canvas events it
/// was sent, by name.
#[derive(Default)]
struct MarkEditing {
    selection: Vec<String>,
    events: Vec<&'static str>,
}

/// Marks a face at a distance: a reference field, a number with a handle
/// along `x`, a selectable dot at `(5, 0, 0)`, and a field of the solid a
/// face is of, to pick if asked. Builds nothing.
struct MarkOperation;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_array([x, y, z].map(Scalar::from_f64))
}

impl Operation for MarkOperation {
    const KIND: &'static str = "mark";
    type Args = MarkArgs;

    fn run(_: &mut StepContext, _: &MarkArgs) -> GeopResult<()> {
        Ok(())
    }
}

impl Editable for MarkOperation {
    const INFO: OperationInfo = INFO;
    type EditingState = MarkEditing;

    fn new_args(_: &Context) -> MarkArgs {
        MarkArgs::default()
    }

    fn presentation(_: &Context, args: &MarkArgs, editing: &MarkEditing) -> Form {
        let mut form = Form::default();
        let handle = Track {
            at: v(0.0, 0.0, 0.0),
            direction: v(1.0, 0.0, 0.0),
        };
        let value = args.distance.plain().unwrap_or_default();
        let distance = Number::new("Distance", value, Unit::Length).handle(Some(handle));
        (form.dialog)
            .reference(
                "on",
                "On",
                picked(&args.on),
                &["face"],
                false,
                Preview(|_, hit| Some(Picked::new(hit.clone()))),
            )
            .number("distance", distance)
            .reference(
                "body",
                "Body",
                picked(&args.body),
                &["face"],
                false,
                Preview(|_, _| Some(Picked::new(solid()))),
            )
            .optional("body");
        let dot = Shape::Point {
            at: v(5.0, 0.0, 0.0),
        };
        form.visuals = vec![Visual::new("dot", dot, Style::Free).selectable()];
        form.selection = editing.selection.clone();
        form
    }

    fn process_user_event(
        _: &Context,
        args: &MarkArgs,
        editing: &MarkEditing,
        event: Event,
    ) -> (MarkArgs, MarkEditing) {
        let mut args = args.clone();
        let mut selection = editing.selection.clone();
        let mut events = editing.events.clone();
        match event {
            Event::Field {
                key,
                value: Value::Elements(on),
            } if key == "on" => args.on = on,
            Event::Field {
                key,
                value: Value::Elements(body),
            } if key == "body" => args.body = body,
            Event::Field { key, value } if key == "distance" => {
                if let Some(distance) = value.numeric_input() {
                    args.distance = distance;
                }
            }
            Event::Field { .. } => {}
            Event::Select(keys) => selection = keys,
            Event::Canvas(event) => events.push(match event {
                CanvasEvent::Click { .. } => "click",
                CanvasEvent::Cancel => "cancel",
                CanvasEvent::Hover { .. } => "hover",
                _ => "other",
            }),
        }
        (args, MarkEditing { selection, events })
    }
}

/// A part with two faces of one solid under every pointer: `top`, and
/// behind it `bottom`.
struct TwoFaces;

impl Scene for TwoFaces {
    fn under(&self, p: &Pointer, accepts: &[String]) -> Vec<ElementHit> {
        let hit = |name: &str, t: f64| ElementHit {
            element: face(name),
            point: p.ray.at(Scalar::from_f64(t)),
            t: Scalar::from_f64(t),
        };
        match accepts.iter().any(|k| k == "face") {
            true => vec![hit("top", 0.0), hit("bottom", 1.0)],
            false => Vec::new(),
        }
    }

    fn hit_instance(&self, _: &str, _: &Pointer) -> Option<Scalar> {
        None
    }
}

/// The face `name` of the part, as a pick names it.
fn face(name: &str) -> PickedElement {
    PickedElement {
        target: geop_core_program::placed::Located::new("model"),
        kind: "face".into(),
        name: name.into(),
    }
}

/// The solid both faces are of, as a pick names it.
fn solid() -> PickedElement {
    PickedElement {
        target: geop_core_program::placed::Located::new("model"),
        kind: "solid".into(),
        name: "block".into(),
    }
}

/// `elements`, as a reference field holds them.
fn picked(elements: &[PickedElement]) -> Vec<Picked> {
    (elements.iter().cloned()).map(Picked::new).collect()
}

/// Looking straight down `-z` at `(x, y)`, reaching a tenth of a unit.
fn down(x: f64, y: f64) -> Pointer {
    Pointer {
        cycle: 0,
        ray: Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
        reach: Reach::Tube {
            radius: Scalar::from_f64(0.1),
        },
    }
}

/// A mark step, `mark`, being edited, as an editor drives it.
struct Editor {
    editors: Editors,
    runner: Runner,
    part: Part,
    program: Program,
    editor: StepEditor,
}

impl Editor {
    /// A mark with `args` — `new`, as just inserted, or opened again.
    fn new(args: MarkArgs, new: bool) -> Self {
        let editors = Editors::default().with(MarkOperation).unwrap();
        let runner = Runner::new(editors.operations().unwrap());
        let mut part = Part::default();
        let program = Program::from_steps(vec![
            ProgramStep::new::<MarkOperation>("mark", &args).unwrap(),
        ]);
        runner.run(&mut part, &program, None);
        let editor = StepEditor::new(&editors, &runner, &part, &program, "mark", new).unwrap();
        Self {
            editors,
            runner,
            part,
            program,
            editor,
        }
    }

    fn send(&mut self, event: StepEditEvent) {
        let editing = Editing {
            editors: &self.editors,
            runner: &self.runner,
            part: (&self.part).into(),
            program: &mut self.program,
            scene: &TwoFaces,
        };
        self.editor.handle(editing, &event).unwrap();
        (self.runner).run(&mut self.part, &self.program, None);
    }

    fn click(&mut self, x: f64, y: f64) {
        self.click_with(down(x, y));
    }

    fn click_with(&mut self, pointer: Pointer) {
        self.send(StepEditEvent::Click {
            pointer,
            double: false,
            shift: false,
        });
    }

    fn hover(&mut self, pointer: Pointer) {
        self.send(StepEditEvent::Hover {
            pointer,
            shift: false,
        });
    }

    fn presentation(&self) -> Presentation {
        (self.editor)
            .presentation(
                &self.editors,
                &self.runner,
                &self.part,
                &self.program,
                &TwoFaces,
            )
            .unwrap()
    }

    fn args(&self) -> MarkArgs {
        serde_json::from_value(self.program.steps[0].args.clone()).unwrap()
    }

    fn editing(&self) -> &MarkEditing {
        self.editor.editing().downcast_ref().unwrap()
    }

    /// Whether the field `on` waits for a click.
    fn armed(&self) -> bool {
        let presentation = self.presentation();
        matches!(presentation.dialog.get("on"), Some(Control::Reference(r)) if r.armed)
    }
}

/// A new step waits for what it is built on: its empty reference field is
/// armed, and a click picks for it and is done.
#[test]
fn a_new_step_picks_for_its_empty_reference() {
    let mut editor = Editor::new(MarkArgs::default(), true);
    assert!(editor.armed());
    editor.click(0.0, 0.0);
    assert_eq!(editor.args().on, vec![face("top")]);
    assert!(!editor.armed());
    assert_eq!(editor.editing().events, Vec::<&str>::new());
}

/// A step opened again waits for nothing.
#[test]
fn an_opened_step_picks_nothing_until_asked() {
    let mut editor = Editor::new(MarkArgs::default(), false);
    assert!(!editor.armed());
    editor.send(StepEditEvent::Dialog {
        key: "on".into(),
        value: Value::Press,
    });
    assert!(editor.armed());
}

/// A number's handle is dragged along its track, the value snapped.
#[test]
fn a_handle_drags_its_number() {
    let args = MarkArgs {
        on: vec![face("top")],
        distance: NumericInput::Plain(1.0),
        body: Vec::new(),
    };
    let mut editor = Editor::new(args, false);
    editor.send(StepEditEvent::Drag {
        from: down(0.0, 0.0),
        to: down(1.234, 0.0),
        done: true,
        shift: false,
    });
    assert_eq!(editor.args().distance, NumericInput::Plain(2.23));
}

/// A click on a selectable visual selects it, and the operation is sent
/// the selection; Escape clears it and cancels; a click on nothing is the
/// operation's.
#[test]
fn clicks_select_and_escape_clears() {
    let args = MarkArgs {
        on: vec![face("top")],
        distance: NumericInput::Plain(1.0),
        body: Vec::new(),
    };
    let mut editor = Editor::new(args, false);
    editor.click(5.0, 0.0);
    assert_eq!(editor.editing().selection, vec!["dot".to_string()]);
    let presentation = (editor.editor)
        .presentation(
            &editor.editors,
            &editor.runner,
            &editor.part,
            &editor.program,
            &TwoFaces,
        )
        .unwrap();
    let dot = presentation
        .visuals
        .iter()
        .find(|v| v.key == "dot")
        .unwrap();
    assert_eq!(dot.style, Style::Selected);
    editor.send(StepEditEvent::Key {
        key: "Escape".into(),
    });
    assert_eq!(editor.editing().selection, Vec::<String>::new());
    assert_eq!(editor.editing().events, vec!["cancel"]);
    editor.click(20.0, 20.0);
    assert_eq!(editor.editing().events, vec!["cancel", "click"]);
}

/// A pointer stepped behind lights the next of what lies under it, and a
/// click takes the one lit: the face behind the first, and round again to
/// the first. The presentation says which, of how many.
#[test]
fn a_pointer_stepped_behind_picks_what_is_behind() {
    let mut editor = Editor::new(MarkArgs::default(), true);
    editor.hover(down(0.0, 0.0));
    let shown = editor.presentation();
    assert_eq!(shown.highlights, vec![face("top")]);
    assert_eq!(shown.cycle, Some(Cycle { at: 0, of: 2 }));
    editor.hover(down(0.0, 0.0).cycled(1));
    let shown = editor.presentation();
    assert_eq!(shown.highlights, vec![face("bottom")]);
    assert_eq!(shown.cycle, Some(Cycle { at: 1, of: 2 }));
    assert_eq!(
        editor.presentation().cycle,
        Some(Cycle { at: 1, of: 2 }),
        "a presentation asked again says the same"
    );
    editor.click_with(down(0.0, 0.0).cycled(1));
    assert_eq!(editor.args().on, vec![face("bottom")]);

    let mut editor = Editor::new(MarkArgs::default(), true);
    editor.click_with(down(0.0, 0.0).cycled(2));
    assert_eq!(
        editor.args().on,
        vec![face("top")],
        "stepped past the last, the first"
    );
}

/// What a field would hold of each element under the pointer is counted
/// once: both faces pick the one solid, so there is nothing behind it to
/// step to.
#[test]
fn what_a_click_would_hold_alike_is_one_candidate() {
    let args = MarkArgs {
        on: vec![face("top")],
        distance: NumericInput::Plain(1.0),
        body: Vec::new(),
    };
    let mut editor = Editor::new(args, false);
    editor.send(StepEditEvent::Dialog {
        key: "body".into(),
        value: Value::Press,
    });
    editor.hover(down(0.0, 0.0).cycled(1));
    assert_eq!(editor.presentation().cycle, Some(Cycle { at: 0, of: 1 }));
    editor.click_with(down(0.0, 0.0).cycled(1));
    assert_eq!(
        editor.args().body,
        vec![face("top")],
        "the first face, the solid's"
    );
}
