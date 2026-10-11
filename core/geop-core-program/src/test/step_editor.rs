//! The [`StepEditor`] driving an operation as an editor does: picks,
//! handles, selection and keys turned into the operation's events.

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{CoordinateSystem, Ray},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{
    editing::{
        CanvasEvent, Control, EntityRef, Event, Form, Number, Pointer, Reach, Role, Shape,
        StepEditEvent, Style, Track, Unit, Value, Visual, hit::EntityHit,
    },
    operation::{Context, Operation, OperationInfo},
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    step_editor::{Editing, Scene, StepEditor},
    test::{Pose, Sc, info},
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct MarkArgs {
    on: Vec<EntityRef>,
    distance: f64,
}

/// What a mark keeps while edited: the selection, and the canvas events it
/// was sent, by name.
#[derive(Default)]
struct MarkEditing {
    selection: Vec<String>,
    events: Vec<&'static str>,
}

/// Marks a face at a distance: a reference field, a number with a handle
/// along `x`, and a selectable dot at `(5, 0, 0)`. Builds nothing.
struct MarkOperation;

fn v(x: f64, y: f64, z: f64) -> Vector3<Sc> {
    Vector3::from_array([x, y, z].map(Sc::from_f64))
}

impl Operation for MarkOperation {
    const INFO: OperationInfo = info("mark");
    type Args = MarkArgs;
    type EditingState = MarkEditing;
    type Input = Sc;
    type State = Pose;

    fn new_args(_: &Context<Sc, Pose>) -> MarkArgs {
        MarkArgs::default()
    }

    fn run(_: &str, _: &mut Part<Sc, Pose>, _: &MarkArgs) -> GeopResult<()> {
        Ok(())
    }

    fn presentation(_: &Context<Sc, Pose>, args: &MarkArgs, editing: &MarkEditing) -> Form<Sc> {
        let mut form = Form::default();
        let handle = Track {
            at: v(0.0, 0.0, 0.0),
            direction: v(1.0, 0.0, 0.0),
        };
        let distance = Number::new("Distance", args.distance, Unit::Length).handle(Some(handle));
        (form.dialog)
            .reference("on", "On", args.on.clone(), &[Role::Face], None, false)
            .number("distance", distance);
        let dot = Shape::Point {
            at: v(5.0, 0.0, 0.0),
        };
        form.visuals = vec![Visual::new("dot", dot, Style::Free).selectable()];
        form.selection = editing.selection.clone();
        form
    }

    fn process_user_event(
        _: &Context<Sc, Pose>,
        args: &MarkArgs,
        editing: &MarkEditing,
        event: Event<Sc>,
    ) -> (MarkArgs, MarkEditing) {
        let mut args = args.clone();
        let mut selection = editing.selection.clone();
        let mut events = editing.events.clone();
        match event {
            Event::Field {
                key,
                value: Value::Entities(on),
            } if key == "on" => args.on = on,
            Event::Field {
                key,
                value: Value::Number(distance),
            } if key == "distance" => args.distance = distance,
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

/// A part with one face, `top`, under every pointer.
struct OneFace;

impl Scene<Sc> for OneFace {
    fn pick(
        &self,
        p: &Pointer<Sc>,
        roles: &[Role],
        _: Option<&EntityRef>,
    ) -> Option<EntityHit<Sc>> {
        roles.contains(&Role::Face).then(|| EntityHit {
            entity: EntityRef::face("top"),
            point: *p.ray.origin(),
            t: Sc::from_f64(0.0),
        })
    }

    fn hit_instance(&self, _: &str, _: &Pointer<Sc>) -> Option<Sc> {
        None
    }

    fn roles(&self, _: &EntityRef) -> GeopResult<Vec<Role>> {
        Ok(vec![Role::Face])
    }

    fn lies_in(&self, _: &EntityRef, _: &EntityRef) -> bool {
        true
    }

    fn frame(&self, _: &EntityRef) -> Option<CoordinateSystem<Sc>> {
        None
    }
}

/// Looking straight down `-z` at `(x, y)`, reaching a tenth of a unit.
fn down(x: f64, y: f64) -> Pointer<Sc> {
    Pointer {
        ray: Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
        reach: Reach::Tube {
            radius: Sc::from_f64(0.1),
        },
    }
}

/// A mark step, `mark`, being edited, as an editor drives it.
struct Editor {
    runner: ProgramRunner<Sc, Pose>,
    program: Program,
    editor: StepEditor<Sc>,
}

impl Editor {
    /// A mark with `args` — `new`, as just inserted, or opened again.
    fn new(args: MarkArgs, new: bool) -> Self {
        let registry = OperationRegistry::default().with(MarkOperation).unwrap();
        let mut runner = ProgramRunner::new(registry);
        let program = Program::new(vec![
            ProgramStep::new::<MarkOperation>("mark", &args).unwrap(),
        ]);
        runner.run(&program, &Default::default(), &Default::default());
        let editor = StepEditor::new(&runner, &program, "mark", new).unwrap();
        Self {
            runner,
            program,
            editor,
        }
    }

    fn send(&mut self, event: StepEditEvent<Sc>) {
        let editing = Editing {
            runner: &self.runner,
            program: &mut self.program,
            scene: &OneFace,
        };
        self.editor.handle(editing, &event).unwrap();
        (self.runner).run(&self.program, &Default::default(), &Default::default());
    }

    fn click(&mut self, x: f64, y: f64) {
        self.send(StepEditEvent::Click {
            pointer: down(x, y),
            button: Default::default(),
            double: false,
            shift: false,
        });
    }

    fn args(&self) -> MarkArgs {
        serde_json::from_value(self.program.steps[0].args.clone()).unwrap()
    }

    fn editing(&self) -> &MarkEditing {
        self.editor.editing().downcast_ref().unwrap()
    }

    /// Whether the field `on` waits for a click.
    fn armed(&self) -> bool {
        let presentation = (self.editor)
            .presentation(&self.runner, &self.program, &OneFace)
            .unwrap();
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
    assert_eq!(editor.args().on, vec![EntityRef::face("top")]);
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
        on: vec![EntityRef::face("top")],
        distance: 1.0,
    };
    let mut editor = Editor::new(args, false);
    editor.send(StepEditEvent::Drag {
        from: down(0.0, 0.0),
        to: down(1.234, 0.0),
        done: true,
        shift: false,
    });
    assert_eq!(editor.args().distance, 2.23);
}

/// A click on a selectable visual selects it, and the operation is sent
/// the selection; Escape clears it and cancels; a click on nothing is the
/// operation's.
#[test]
fn clicks_select_and_escape_clears() {
    let args = MarkArgs {
        on: vec![EntityRef::face("top")],
        distance: 1.0,
    };
    let mut editor = Editor::new(args, false);
    editor.click(5.0, 0.0);
    assert_eq!(editor.editing().selection, vec!["dot".to_string()]);
    let presentation = (editor.editor)
        .presentation(&editor.runner, &editor.program, &OneFace)
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
