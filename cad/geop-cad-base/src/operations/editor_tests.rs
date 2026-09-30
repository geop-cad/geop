//! The editor engine: a program edited by commands, and only what changed
//! sent back.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::{
    primitives::Ray,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    EntityRef, ORIGIN,
    operation::Role,
    ui::{Button, Control, Pointer, Reach, StepEditEvent, Tone, Value},
};
use geop_ops_booleans::Combine;

use crate::{Command, Editor, PartOperation, Program, Update, examples};

fn editor() -> (Editor<S>, Update<S>) {
    let mut editor = Editor::new();
    let update = editor.handle(Command::LoadExample {
        name: "box_with_drill_hole".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    (editor, update)
}

fn dialog(key: &str, value: Value) -> Command<S> {
    Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    }
}

/// A pointer from `origin` along `dir`, reaching 0.009 units.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

fn distance(editor: &Editor<S>, id: &str) -> f64 {
    let program = editor.program();
    match &program.steps[program.index_of(id).unwrap()].operation {
        PartOperation::Extrude(args) => args.distance,
        other => panic!("{other:?}"),
    }
}

/// A new step goes in where the program runs to, once it builds; an
/// existing one is edited in place, moved and removed by id — and every
/// change can be undone and redone.
#[test]
fn commands_edit_the_program() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();

    let update = editor.handle(Command::New {
        kind: "extrude".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Extrude");
    assert_eq!(step.id, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!(update.step.is_none());
    let steps = update.program.expect("the program changed").steps;
    assert_eq!(steps.last().unwrap().id, "extrude1");

    editor.handle(Command::Open {
        id: "extrude1".into(),
    });
    editor.handle(dialog("distance", Value::Number(2.0)));
    // Not in the program until committed.
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    editor.handle(Command::Commit);
    assert_eq!(distance(&editor, "extrude1"), 2.0);

    editor.handle(Command::Move {
        id: "extrude1".into(),
        index: 0,
    });
    assert_eq!(editor.program().steps[0].id, "extrude1");
    editor.handle(Command::Remove {
        id: "extrude1".into(),
    });
    assert_eq!(*editor.program(), before);

    for _ in 0..3 {
        editor.handle(Command::Undo);
    }
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    let update = editor.handle(Command::Redo);
    assert!(update.program.unwrap().can_redo);
    assert_eq!(distance(&editor, "extrude1"), 2.0);
}

/// A command that would break the program is refused, saying why, and
/// changes nothing.
#[test]
fn bad_commands_are_refused() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let mut twice = before.clone();
    twice.steps.push(twice.steps[0].clone());
    for bad in [
        Command::Remove { id: "nope".into() },
        Command::Move {
            id: "box".into(),
            index: before.steps.len(),
        },
        Command::Load { program: twice },
        Command::Commit,
        Command::New {
            kind: "fillet".into(),
        },
    ] {
        let update = editor.handle(bad);
        assert!(update.error.is_some());
        assert_eq!(*editor.program(), before);
    }

    // A step that does not build cannot go in: without the box, the hole
    // has no face to be sketched on.
    let update = editor.handle(Command::Remove { id: "box".into() });
    assert!(update.error.is_none());
    let update = editor.handle(Command::Open {
        id: "hole_sketch".into(),
    });
    assert!(update.step.unwrap().error.is_some());
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_some());
}

/// The part is sent again only when what is drawn changed, the program only
/// when it or its run did — a hover sends neither.
#[test]
fn only_what_changed_is_sent() {
    let (mut editor, update) = editor();
    assert!(update.program.is_some() && update.scene.is_some());
    let update = editor.handle(Command::Open { id: "hole".into() });
    assert!(update.program.is_some(), "the step is marked as edited");
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: pointer([5.0, 5.0, 5.0], [0.0, 0.0, -1.0]),
        },
    });
    assert!(update.program.is_none() && update.scene.is_none());
    assert!(update.step.is_some());
    let update = editor.handle(Command::Preview { preview: false });
    assert!(update.scene.is_some(), "the part before the step is drawn");
    let update = editor.handle(Command::Show);
    assert!(update.program.is_some() && update.scene.is_some());
}

/// A handle dragged along its track sets its field, snapped; the combine
/// mode follows the distance's sign.
#[test]
fn handles_are_dragged() {
    let (mut editor, _) = editor();
    editor.handle(Command::Open { id: "hole".into() });
    // Seen from the side: the hole's end cap is half a unit into the box,
    // at (1, 1, 0.5); dragged up past its sketch plane, it builds up.
    let side = |z: f64| pointer([1.0, -10.0, z], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover { pointer: side(0.5) },
    });
    assert!(update.step.unwrap().presentation.grab);
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.5),
            to: side(1.253),
            done: true,
        },
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!((distance(&editor, "hole") - 0.25).abs() < 1e-9);
    let program = editor.program();
    let PartOperation::Extrude(hole) = &program.steps[program.index_of("hole").unwrap()].operation
    else {
        panic!("an extrude");
    };
    assert!(matches!(hole.combine, Combine::Union { .. }));
}

/// A new extrude's distance handle, dragged as a viewer drags it: straight
/// away — the newest sketch it starts with needs no click — seen in
/// perspective, the pointer moving in several steps before it is released.
#[test]
fn new_steps_handles_are_dragged_in_steps() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    // From the side, in perspective: the hole sketch's extrude ends a unit
    // above the box's top, at (1, 1, 2).
    let side = |z: f64| {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
        Pointer {
            ray: Ray::try_new(v([1.0, -10.0, 2.0]), v([0.0, 11.0, z - 2.0])).unwrap(),
            reach: Reach::Cone {
                slope: S::from_f64(0.002),
            },
        }
    };
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover { pointer: side(2.0) },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the handle offers a grab"
    );
    for (z, done) in [(2.1, false), (2.2, false), (2.3, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: side(2.0),
                to: side(z),
                done,
            },
        });
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let PartOperation::Extrude(extrude) = &program.steps.last().unwrap().operation else {
        panic!("an extrude");
    };
    assert!(
        (extrude.distance - 1.3).abs() < 1e-9,
        "{}",
        extrude.distance
    );
}

/// A new step whose first input is still empty starts by picking it — a
/// new sketch waits for its plane, nothing preselected, and says so rather
/// than failing. One that already
/// holds its first input — the newest sketch, for an extrude — waits for no
/// click; pressing the field picks another, and while it does, the sketches
/// the program used — hidden otherwise — are shown to pick from.
#[test]
fn new_steps_pick_their_first_input() {
    let (mut editor, update) = editor();
    let hidden = update.scene.unwrap().hidden;
    assert!(hidden.contains(&"outline".to_string()));
    assert!(hidden.contains(&"hole_sketch".to_string()));

    let update = editor.handle(Command::New {
        kind: "add_sketch".into(),
    });
    let step = update.step.unwrap();
    // Waiting for its plane is no error; it cannot be committed yet.
    assert_eq!(step.missing, ["plane"]);
    assert_eq!(step.error, None);
    let presentation = step.presentation;
    assert_eq!(presentation.pickable, [Role::Plane]);
    let Some(Control::Reference(plane)) = presentation.dialog.get("plane") else {
        panic!("the plane is picked");
    };
    assert!(plane.armed && plane.value.is_empty(), "{plane:?}");
    let refused = editor.handle(Command::Commit);
    let refused = refused.error.expect("the commit is refused");
    assert!(refused.contains("pick the plane first"), "{refused}");
    editor.handle(Command::Cancel);

    let update = editor.handle(Command::New {
        kind: "extrude".into(),
    });
    assert!(update.step.unwrap().presentation.pickable.is_empty());
    let update = editor.handle(dialog("sketch", Value::Press));
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Sketch]);
    assert!(update.scene.unwrap().hidden.is_empty());
}

/// Choosing to cut sticks while the distance changes on the same side of
/// the sketch plane; crossing it turns a cut into a join, and back.
#[test]
fn a_chosen_cut_stays_a_cut() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    let combine = |editor: &mut Editor<S>| {
        editor.handle(Command::Commit);
        let program = editor.program();
        let PartOperation::Extrude(extrude) = &program.steps.last().unwrap().operation else {
            panic!("an extrude");
        };
        let combine = extrude.combine.clone();
        editor.handle(Command::Undo);
        combine
    };
    editor.handle(dialog("combine", Value::Choice("difference".into())));
    editor.handle(dialog("distance", Value::Number(2.0)));
    assert!(matches!(combine(&mut editor), Combine::Difference { .. }));
}

/// A revolve picks its axis in the viewport, like anything else it builds
/// on: any line in the sketch's plane — here the origin's y axis.
#[test]
fn revolve_axes_are_picked() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "revolve".into(),
    });
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Reference(axis)) = presentation.dialog.get("axis") else {
        panic!("the axis is picked");
    };
    assert!(
        axis.value.is_empty(),
        "the newest sketch, a circle, has no line to default to"
    );
    // Kept apart from the box: joining the two is a boolean of its own.
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    // From below: the outline's square, under the box.
    let below = |x: f64, y: f64| pointer([x, y, -10.0], [0.0, 0.0, 1.0]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    editor.handle(click(below(1.0, 0.5)));
    editor.handle(dialog("axis", Value::Press));
    // The origin's y axis, ten reaches of 0.009 long, from below.
    let update = editor.handle(click(below(0.0, 0.06)));
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Reference(axis)) = presentation.dialog.get("axis") else {
        panic!("the axis is picked");
    };
    assert_eq!(
        axis.entities().collect::<Vec<_>>(),
        [&EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Axis(FrameAxis::Y)
        )]
    );
    assert_eq!(axis.value[0].tone, Tone::Normal);
    assert!(!axis.armed, "one axis is picked, and done");
}

/// Seeking runs only the steps before the marker, and new steps go there.
#[test]
fn new_steps_go_where_the_program_runs_to() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::Seek { marker: Some(2) });
    let state = update.program.unwrap();
    assert_eq!(state.marker, 2);
    assert!(state.steps[1].runs && !state.steps[2].runs);
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    editor.handle(Command::Commit);
    assert_eq!(editor.program().steps[2].id, "extrude1");
    let update = editor.handle(Command::Show);
    assert_eq!(update.program.unwrap().marker, 3);
}

/// Every example loads by name, and builds.
#[test]
fn examples_load_by_name() {
    let mut editor = Editor::<S>::new();
    let names = editor.handle(Command::Show).program.unwrap().examples;
    assert_eq!(names.len(), examples::all().len());
    let update = editor.handle(Command::LoadExample {
        name: names[0].into(),
    });
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    assert_ne!(*editor.program(), Program::new());
}
