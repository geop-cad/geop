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
        PartOperation::Extrude(args) => match &args.extent.side1 {
            geop_ops_extrude_revolve::Extent::Blind(d) => d.plain().expect("a plain distance"),
            other => panic!("{other:?} is no distance"),
        },
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
        Command::Load {
            program: twice,
            path: None,
        },
        Command::Commit,
        Command::New {
            kind: "no_such_operation".into(),
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
            shift: false,
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
        event: StepEditEvent::Hover {
            pointer: side(0.5),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab);
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.5),
            to: side(1.253),
            done: true,
            shift: false,
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
        event: StepEditEvent::Hover {
            pointer: side(2.0),
            shift: false,
        },
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
                shift: false,
            },
        });
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let PartOperation::Extrude(extrude) = &program.steps.last().unwrap().operation else {
        panic!("an extrude");
    };
    assert_eq!(
        extrude.extent.side1,
        geop_ops_extrude_revolve::Extent::blind(1.3)
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

/// Seeking back before a fillet and forward again runs the steps up to the
/// marker each time, the fillet's part shown only once it runs.
#[test]
fn seeking_back_before_a_fillet() {
    let (mut editor, _) = editor();
    let before = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    let rim = before
        .topology()
        .edges
        .iter()
        .find(|(_, e)| e.curve.as_arc().unwrap().is_some())
        .map(|(&id, _)| before.name_of(id).unwrap().to_string())
        .unwrap();
    let mut program = editor.program().clone();
    program.push(
        "round",
        geop_ops_fillet::FilletArgs::constant(vec![rim], 0.1),
    );
    let steps = program.steps.len();
    let update = editor.handle(Command::Load {
        program,
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    for marker in [Some(steps - 1), Some(1), None, Some(steps - 1)] {
        let update = editor.handle(Command::Seek { marker });
        assert!(update.error.is_none(), "{marker:?}: {:?}", update.error);
        let state = update.program.expect("the program is sent");
        assert_eq!(state.marker, marker.unwrap_or(steps), "{marker:?}");
        for (i, step) in state.steps.iter().enumerate() {
            assert_eq!(step.runs, i < state.marker, "{marker:?}: step {i}");
            assert!(
                step.error.is_none(),
                "{marker:?}: step {i}: {:?}",
                step.error
            );
        }
        let scene = update.scene.expect("the scene is sent anew");
        let rounded = scene.part.solids.iter().any(|s| s.starts_with("fillet("));
        assert_eq!(
            rounded,
            state.marker == steps,
            "{marker:?}: {:?}",
            scene.part.solids
        );
    }
}

/// A new shell picks its solid, then the faces to open — faces of that
/// solid, clicked on it.
#[test]
fn new_shell_picks_its_faces_on_the_solid() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "shell".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // From above, onto the box's top beside the hole.
    let above = |x: f64, y: f64| pointer([x, y, 10.0], [0.0, 0.0, -1.0]);
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("faces", Value::Press));
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the shell is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    match &program.steps.last().unwrap().operation {
        PartOperation::Shell(args) => {
            assert_eq!(args.faces, ["extrude(box,end)"], "{args:?}");
        }
        other => panic!("{other:?}"),
    }
}

/// An offset plane's handle, on the plane, drags its distance: grabbed
/// from the side and pulled up by 0.3, the plane half a unit above the
/// box's top goes to 0.8.
#[test]
fn offset_plane_dragged_by_its_handle() {
    let (mut editor, _) = editor();
    let mut program = editor.program().clone();
    program.push(
        "plane",
        geop_ops_datums::AddDatumArgs {
            selection: vec![EntityRef::Face {
                name: "extrude(box,end)".into(),
            }],
            construction: geop_ops_datums::Construction::Offset {
                distance: 0.5.into(),
            },
        },
    );
    editor.handle(Command::Load {
        program,
        path: None,
    });
    let update = editor.handle(Command::Open { id: "plane".into() });
    let step = update.step.expect("the plane is edited");
    let (at, direction) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            geop_ops::ui::Shape::Handle { at, direction } if v.key == "distance" => {
                Some((at, direction))
            }
            _ => None,
        })
        .expect("the distance has a handle");
    assert!(direction[2].abs().could_be_equal(S::ONE), "{direction:?}");
    assert!(at[2].could_be_equal(S::from_f64(1.5)), "{at:?}");
    // From the side, square to the track, through the handle.
    let [x, y, z] = [0, 1, 2].map(|k| at[k].to_f64());
    let side = |dz: f64| pointer([x, y - 10.0, z + dz], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(0.0),
            shift: false,
        },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the handle is not grabbed"
    );
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.0),
            to: side(0.3),
            done: true,
            shift: false,
        },
    });
    if let Some(e) = &editor.handle(Command::Commit).error {
        panic!("{e}")
    }
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::AddDatum(args) => assert_eq!(
            args.construction,
            geop_ops_datums::Construction::Offset {
                distance: 0.8.into()
            }
        ),
        other => panic!("{other:?}"),
    }
}

/// A new offset plane, made from the box's top, is dragged by its handle
/// while it is still being made: picked, offset, grabbed, pulled up 0.3.
#[test]
fn new_offset_plane_dragged_by_its_handle() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "add_datum".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    editor.handle(click(pointer([0.3, 0.3, 10.0], [0.0, 0.0, -1.0])));
    let update = editor.handle(dialog("construction", Value::Choice("offset".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the plane is edited");
    let (at, _) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            geop_ops::ui::Shape::Handle { at, direction } if v.key == "distance" => {
                Some((at, direction))
            }
            _ => None,
        })
        .expect("the distance has a handle");
    let [x, y, z] = [0, 1, 2].map(|k| at[k].to_f64());
    let side = |dz: f64| pointer([x, y - 10.0, z + dz], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(0.0),
            shift: false,
        },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the handle is not grabbed"
    );
    let before = match &step.presentation.dialog.get("distance") {
        Some(Control::Number(n)) => n.value,
        other => panic!("{other:?}"),
    };
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.0),
            to: side(0.3),
            done: true,
            shift: false,
        },
    });
    let after = match update.step.unwrap().presentation.dialog.get("distance") {
        Some(Control::Number(n)) => n.value,
        other => panic!("{other:?}"),
    };
    assert!((after - before - 0.3).abs() < 1e-9, "{before} -> {after}");
}

/// A new linear pattern picks nothing by itself: it waits for the body or
/// the feature to copy, here the drilled box, clicked. Along `x`, its
/// spacing's handle, grabbed from above and pulled one further along `x`,
/// spreads the copies two apart; a fourth is asked for in the dialog, and
/// committed, the scene lists the drilled box and its three copies, the
/// copies named after the step.
#[test]
fn new_linear_pattern_dragged_by_its_spacing_handle() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "linear_pattern".into(),
    });
    let step = update.step.expect("the pattern is edited");
    assert_eq!(step.missing, ["bodies"], "nothing is picked by itself");
    // The box's top, beside its hole.
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer([0.3, 0.3, 10.0], [0.0, 0.0, -1.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let (at, direction) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            geop_ops::ui::Shape::Handle { at, direction } if v.key == "spacing" => {
                Some((at, direction))
            }
            _ => None,
        })
        .expect("the spacing has a handle");
    assert!(direction[0].could_be_equal(S::ONE), "{direction:?}");
    let [x, y, z] = [0, 1, 2].map(|k| at[k].to_f64());
    let above = |dx: f64| pointer([x + dx, y, z + 10.0], [0.0, 0.0, -1.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: above(0.0),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab, "no grab");
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: above(0.0),
            to: above(1.0),
            done: true,
            shift: false,
        },
    });
    let update = editor.handle(dialog("count", Value::Number(4.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::LinearPattern(args) => {
            assert_eq!(args.first.count, 4.0.into());
            match &args.first.spacing {
                geop_ops_pattern::Spacing::Step(step) => {
                    let step = step.plain().expect("a plain spacing");
                    assert!((step - 2.0).abs() < 1e-9, "{step}")
                }
                other => panic!("{other:?}"),
            }
        }
        other => panic!("{other:?}"),
    }
    let solids: Vec<String> = update
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == crate::editor::StructureKind::Solid)
        .map(|item| item.name)
        .collect();
    assert_eq!(solids.len(), 4, "{solids:?}");
    let id = editor.program().steps.last().unwrap().id.clone();
    assert!(
        solids.contains(&format!("linear_pattern({id},3,extrude(hole))")),
        "{solids:?}"
    );
}

/// A new route picks what it runs through by clicks, in order: down
/// through the box's hole — its rim on top, its rim at the bottom, both
/// clips — and out to the box's far bottom corner. Too tight for the
/// default hookup wire, it is refused, the bend drawn as failed; a thinner
/// wire, chosen in the dialog, fits, and once built the dialog reports the
/// route's length and every wire's cut length.
#[test]
fn new_route_is_picked_and_its_wires_chosen() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "route".into(),
    });
    let step = update.step.expect("the route is edited");
    assert_eq!(step.presentation.pickable, [Role::Point, Role::Circle]);
    assert_eq!(step.missing, ["through"]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // The hole, 0.4 round `(1, 1)` and 0.5 deep, seen at a slant: its rim
    // on the box's top, then its rim on its floor, through the hole — both
    // halfway between the vertices that split each circle.
    let rim = 1.0 + 0.4 * std::f64::consts::FRAC_1_SQRT_2;
    editor.handle(click(pointer(
        [rim + 4.0, rim + 4.0, 1.0 + 5.0],
        [-4.0, -4.0, -5.0],
    )));
    editor.handle(click(pointer(
        [rim - 0.3, rim - 0.3, 1.5],
        [0.3, 0.3, -1.0],
    )));
    // The box's bottom corner at `(2, 2, 0)`, from below.
    let update = editor.handle(click(pointer([2.0, 2.0, -10.0], [0.0, 0.0, 1.0])));
    let step = update.step.expect("the route is edited");
    let Some(Control::Reference(through)) = step.presentation.dialog.get("through") else {
        panic!("what the route runs through is picked");
    };
    let picked: Vec<&EntityRef> = through.entities().collect();
    assert!(
        matches!(
            picked.as_slice(),
            [
                EntityRef::Edge { .. },
                EntityRef::Edge { .. },
                EntityRef::Vertex { .. }
            ]
        ),
        "{picked:?}"
    );
    // The default wire bundles 1.4 across, which may bend no tighter than
    // 7.2: the turn out to the corner is far tighter.
    let error = step.error.expect("too tight a bend is refused");
    assert!(
        error.contains("may bend no tighter than radius 7.18"),
        "{error}"
    );
    assert!(error.contains("between point 2"), "{error}");
    assert!(
        step.presentation
            .visuals
            .iter()
            .any(|v| v.key.starts_with("route:") && v.style == geop_ops::ui::Style::Failed),
        "the bend too tight is drawn as failed"
    );

    // The wire, selected, given a diameter of its own: 0.1 across.
    editor.handle(dialog("wire:0", Value::Press));
    editor.handle(dialog("wire_size", Value::Choice("diameter".into())));
    let update = editor.handle(dialog("wire_diameter", Value::Number(0.1)));
    let step = update.step.expect("the route is edited");
    assert_eq!(step.error, None);
    let update = editor.handle(dialog("service_loop", Value::Number(0.25)));
    assert_eq!(update.step.expect("the route is edited").error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!(
        update
            .scene
            .unwrap()
            .part
            .solids
            .contains(&"route(route1)".into()),
        "the bundle is drawn"
    );

    let cable = editor.part().cable("route(route1)").unwrap().clone();
    // Straight down the hole, 0.5, then a single arc out to the corner.
    assert!(cable.length.to_f64() > 0.5 + 1.5, "{:?}", cable.length);
    let cut = cable.wires[0].cut_length.to_f64() - cable.length.to_f64();
    assert!((cut - 0.5).abs() < 1e-9, "{cut}");
    let update = editor.handle(Command::Open {
        id: "route1".into(),
    });
    let presentation = update.step.expect("the route is edited").presentation;
    let Some(Control::Text { text, tone }) = presentation.dialog.get("report") else {
        panic!("what was built is reported: {:?}", presentation.dialog);
    };
    assert_eq!(*tone, Tone::Success);
    assert!(
        text.starts_with(&format!("Route {:.1} long", cable.length.to_f64())),
        "{text}"
    );
    let Some(Control::List { items, .. }) = presentation.dialog.get("wires") else {
        panic!("the wires are listed");
    };
    let detail = items[0].detail.as_deref().unwrap();
    let cut_length = cable.wires[0].cut_length.to_f64();
    assert!(
        detail.ends_with(&format!("cut {cut_length:.1}")),
        "{detail}"
    );
}

/// A sweep made in the editor: the horn's sketches picked, its rail
/// dropped for a twist and taper, the profile kept facing one way —
/// the dialog showing the twist and scale only once there is no rail.
///
/// It is made a new body first: joined to the horn, every change in the
/// dialog would run a boolean of the sweep with it, which is not what is
/// tested here.
#[test]
fn sweep_with_rails_twist_and_orientation() {
    let mut editor = Editor::new();
    let update = editor.handle(Command::LoadExample {
        name: "horn".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let solids = |update: &Update<S>| {
        let scene = update.scene.as_ref().expect("a scene");
        scene
            .structure
            .iter()
            .filter(|i| i.kind == crate::editor::StructureKind::Solid)
            .count()
    };
    assert_eq!(solids(&update), 1);

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    let sketch = |name: &str| Value::Entities(vec![EntityRef::Sketch { name: name.into() }]);
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    let update = editor.handle(dialog("rails", Value::Press));
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Path]);
    let update = editor.handle(dialog("rails", sketch("flare")));
    let dialog_shown = update.step.unwrap().presentation.dialog;
    assert!(
        dialog_shown.get("twist").is_none(),
        "the rail decides the twist"
    );

    editor.handle(dialog("rails", Value::Entities(Vec::new())));
    editor.handle(dialog("orientation", Value::Choice("fixed_normal".into())));
    editor.handle(dialog("twist", Value::Number(90.0)));
    let update = editor.handle(dialog("end_scale", Value::Number(0.5)));
    let step = update.step.unwrap();
    assert_eq!(step.error, None);
    let Some(Control::Number(twist)) = step.presentation.dialog.get("twist") else {
        panic!("the twist is shown without rails");
    };
    assert_eq!(twist.value, 90.0);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(solids(&update), 2);
    let program = editor.program();
    let PartOperation::Sweep(args) = &program.steps.last().unwrap().operation else {
        panic!("a sweep");
    };
    assert_eq!(
        (
            args.twist,
            args.end_scale,
            args.orientation,
            args.rails.len()
        ),
        (
            90.0,
            0.5,
            geop_ops_extrude_revolve::Orientation::FixedNormal,
            0
        )
    );
}

/// A new hole, the way the front end makes one: the plate's top clicked
/// as the face, the sketch's point clicked as the centre, a counterbored
/// M5 through all chosen in the dialog — which says what that comes to —
/// committed, and the plate drilled; made tapped, its thread drawn.
#[test]
fn new_hole_from_the_dialog() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program: super::hole_tests::plate(&[[20.0, 20.0]]),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "hole".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let above = |x: f64, y: f64| pointer([x, y, 50.0], [0.0, 0.0, -1.0]);
    let update = editor.handle(click(above(40.0, 30.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("points", Value::Press));
    let update = editor.handle(click(above(20.0, 20.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("kind", Value::Choice("counterbore".into())));
    editor.handle(dialog("size", Value::Choice("M5".into())));
    let update = editor.handle(dialog("end", Value::Choice("through_all".into())));
    let step = update.step.expect("the hole is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let shown = &step.presentation.dialog;
    match shown.get("dimensions") {
        Some(Control::Text { text, .. }) => {
            assert_eq!(text, "Ø5.5, counterbore Ø10 × 5 deep")
        }
        other => panic!("{other:?}"),
    }
    assert!(shown.get("depth").is_none(), "through all has no depth");
    // The hole and its counterbore, drawn on the face.
    let circles = step
        .presentation
        .visuals
        .iter()
        .filter(|v| v.key.starts_with("hole0"))
        .count();
    assert_eq!(circles, 2);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::Hole(args) => {
            assert_eq!(args.face, "extrude(plate,end)");
            assert_eq!(
                args.points,
                [EntityRef::SketchPoint {
                    sketch: "centres".into(),
                    point: geop_core_sketch::PointId(0),
                }]
            );
            assert_eq!(args.kind, geop_ops_hole::HoleKind::Counterbore);
            assert_eq!(args.end, geop_ops_extrude_revolve::Extent::ThroughAll);
        }
        other => panic!("{other:?}"),
    }
    let scene = update.scene.expect("the scene changed");
    assert_eq!(scene.part.solids, ["hole(hole1)"]);

    // Tapped instead: the scene draws its cosmetic thread.
    editor.handle(Command::Open { id: "hole1".into() });
    editor.handle(dialog("kind", Value::Choice("tapped".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let scene = update.scene.expect("the scene changed");
    assert_eq!(scene.part.threads.len(), 1);
    assert_eq!(scene.part.threads[0].designation, "M5x0.8");
}

/// A boundary surface picks the box's four top edges, clicked from above
/// and outside, and fills them; a new thicken then starts from that sheet
/// and makes it a slab.
#[test]
fn new_boundary_surface_picks_edges_then_thickens() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "boundary_surface".into(),
    });
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Curve]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // Each down onto the box's top rim at 45 degrees, from outside it.
    for (origin, dir) in [
        ([0.5, -5.0, 6.0], [0.0, 1.0, -1.0]),
        ([7.0, 0.5, 6.0], [-1.0, 0.0, -1.0]),
        ([1.5, 7.0, 6.0], [0.0, -1.0, -1.0]),
        ([-5.0, 1.5, 6.0], [1.0, 0.0, -1.0]),
    ] {
        let update = editor.handle(click(pointer(origin, dir)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let step = program.steps.last().unwrap();
    match &step.operation {
        PartOperation::BoundarySurface(args) => assert_eq!(args.edges.len(), 4, "{args:?}"),
        other => panic!("{other:?}"),
    }
    let sheet = format!("boundary({})", step.id);
    let scene = update.scene.expect("the scene is sent anew");
    let faces = |scene: &crate::editor::SceneState<S>| -> Vec<String> {
        scene.part.faces.iter().map(|f| f.name.clone()).collect()
    };
    assert!(faces(&scene).contains(&sheet), "{:?}", faces(&scene));

    let update = editor.handle(Command::New {
        kind: "thicken".into(),
    });
    let step = update.step.expect("the thicken is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    editor.handle(dialog("thickness", Value::Number(0.2)));
    let update = editor.handle(dialog("side", Value::Choice("both".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let step = program.steps.last().unwrap();
    match &step.operation {
        PartOperation::Thicken(args) => {
            assert_eq!(args.face, sheet);
            assert_eq!(args.thickness, 0.2);
        }
        other => panic!("{other:?}"),
    }
    let scene = update.scene.expect("the scene is sent anew");
    let slab = format!("thicken({})", step.id);
    assert!(scene.part.solids.contains(&slab), "{:?}", scene.part.solids);
    assert!(!faces(&scene).contains(&sheet));
}

/// A 3-D sketch drawn the way the front end draws it: a new step, clicks
/// that place points — at the origin, fixed there, and then in space —
/// chained by lines, Enter to end; committed, a sweep along it pipes the
/// section drawn before into a valid solid.
#[test]
fn a_3d_sketch_is_drawn_and_swept_along() {
    let mut editor = Editor::<S>::new();
    let mut program = examples::pipe();
    program.steps.truncate(1);
    let update = editor.handle(Command::Load {
        program,
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    assert_eq!(update.step.unwrap().label, "3-D sketch");
    let click = |editor: &mut Editor<S>, origin: [f64; 3], dir: [f64; 3]| {
        editor.handle(Command::Event {
            event: StepEditEvent::Click {
                pointer: pointer(origin, dir),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        })
    };
    // On the origin's ball: a point fixed at the origin.
    click(&mut editor, [0.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    // Then in the plane through the last point, facing the eye.
    click(&mut editor, [2.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    click(&mut editor, [2.0, -10.0, 2.0], [0.0, 1.0, 0.0]);
    let update = click(&mut editor, [-10.0, 2.0, 2.0], [1.0, 0.0, 0.0]);
    let presentation = update.step.unwrap().presentation;
    let points = presentation
        .visuals
        .iter()
        .filter(|v| matches!(v.shape, geop_ops::ui::Shape::Point { .. }))
        .count();
    assert!(points >= 4, "{:?}", presentation.visuals);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Key {
            key: "Enter".into(),
        },
    });
    let step = update.step.unwrap();
    assert!(step.error.is_none(), "{:?}", step.error);
    match step.presentation.dialog.get("status") {
        Some(Control::Text { text, .. }) => assert!(text.contains("1 chain"), "{text}"),
        other => panic!("{other:?}"),
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let route = editor.program().steps.last().unwrap().clone();
    let PartOperation::AddSketch3d(args) = &route.operation else {
        panic!("{route:?}");
    };
    assert_eq!(args.sketch.curves.len(), 3);
    assert_eq!(args.references.len(), 1, "the origin, as a fixed point");

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::Sweep(sweep) = &editor.program().steps.last().unwrap().operation else {
        panic!("a sweep");
    };
    assert_eq!(
        (sweep.profile.as_str(), sweep.path.as_ref()),
        ("section", Some(&EntityRef::sketch3d(route.id.as_str())))
    );
    let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// The number a program's state gives `name`.
fn state_number(editor: &Editor<S>, name: &str) -> f64 {
    match editor.program().state.get(name) {
        Some(geop_ops::part::ParamValue::Number(v)) => v.to_f64(),
        other => panic!("{name} is no number: {other:?}"),
    }
}

/// How far the placed part `instance` is turned about `z`, in degrees.
fn turn_of(editor: &Editor<S>, instance: &str) -> f64 {
    match editor
        .program()
        .state
        .get(&geop_ops::part::pose_parameter(instance))
    {
        Some(geop_ops::part::ParamValue::Pose(pose)) => pose.euler_degrees()[2],
        other => panic!("{instance} has no pose: {other:?}"),
    }
}

/// The arm's forearm turns on a revolute joint limited to ±150°: dragged
/// round past the limit by a point of it, it turns about the joint's axis
/// and stops at 150° — the joint's angle, in the program's state, exactly
/// the limit, and shown in the step's dialog. Set in the dialog, the joint
/// turns the forearm there; set in the program's panel, the wrist turns the
/// hand.
#[test]
fn a_revolute_joint_is_dragged_up_to_its_limit() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let elbow = "add_part(fore,m1).angle";
    assert_eq!(state_number(&editor, elbow), 30.0);
    let program = update.program.unwrap();
    assert_eq!(program.joints.len(), 2, "{:?}", program.joints);
    let freedom = program.freedom.unwrap();
    assert_eq!(freedom.parts["upper"], 0);
    assert_eq!(freedom.parts["fore"], 1);
    assert_eq!(freedom.parts["hand"], 2);

    let update = editor.handle(Command::Open { id: "fore".into() });
    assert!(update.error.is_none(), "{:?}", update.error);
    // Grabbed halfway along, where nothing lies on it, and dragged round
    // the elbow at (3, 0) to 170°, looking down.
    let down = |[x, y]: [f64; 2]| pointer([x, y, 10.0], [0.0, 0.0, -1.0]);
    let round = |degrees: f64| {
        let a = degrees.to_radians();
        [3.0 + 1.5 * a.cos(), 1.5 * a.sin()]
    };
    let mut done = false;
    for (k, angle) in [60.0, 100.0, 140.0, 170.0].into_iter().enumerate() {
        done = k == 3;
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: down(round(30.0)),
                to: down(round(angle)),
                done,
                shift: false,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    assert!(done);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(state_number(&editor, elbow), 150.0);
    assert!((turn_of(&editor, "fore") - 150.0).abs() < 1e-9);
    editor.handle(Command::Open { id: "fore".into() });
    let step = editor.handle(dialog("mate:m1", Value::Press)).step.unwrap();
    let Some(Control::Number(angle)) = step.presentation.dialog.get("mate:m1:angle") else {
        panic!("the joint's angle is shown");
    };
    assert_eq!(angle.value, 150.0);
    assert!(step.presentation.dialog.get("mate:m1:max").is_some());

    // Set in the dialog: the forearm turns there.
    let update = editor.handle(dialog("mate:m1:angle", Value::Number(-60.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(state_number(&editor, elbow), -60.0);
    let fore = turn_of(&editor, "fore");
    assert!(
        (fore + 60.0).abs() < 1e-9,
        "{fore} {:?}",
        editor.program().state
    );

    // Set in the program's panel: the hand turns, the forearm stays.
    let wrist = "add_part(hand,m1).angle";
    let update = editor.handle(Command::Joint {
        parameter: wrist.into(),
        value: 90.0,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(state_number(&editor, wrist), 90.0);
    assert!((turn_of(&editor, "fore") + 60.0).abs() < 1e-9);
    assert!((turn_of(&editor, "hand") - 30.0).abs() < 1e-9);
    let joints = update.program.unwrap().joints;
    assert_eq!(joints[1].values[0].value, 90.0);
    assert_eq!(joints[1].values[0].max, Some(120.0));
}

/// A new draft picks the faces to tilt by clicking them, then the neutral
/// plane: the enclosure's +x wall from the side, its bottom from below.
#[test]
fn new_draft_picks_its_faces_and_neutral_plane() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program: super::plastic_tests::enclosure(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "draft".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let update = editor.handle(click(pointer([10.0, 1.0, 0.5], [-1.0, 0.0, 0.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("neutral", Value::Press));
    let update = editor.handle(click(pointer([1.0, 1.0, -10.0], [0.0, 0.0, 1.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("angle", Value::Number(5.0)));
    let step = editor
        .handle(Command::Commit)
        .program
        .expect("the program changed");
    assert!(step.steps.iter().all(|s| s.error.is_none()));
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::Draft(args) => {
            assert_eq!(args.faces, [super::plastic_tests::wall("c5")], "{args:?}");
            assert_eq!(
                args.neutral,
                Some(EntityRef::Face {
                    name: "extrude(box,start)".into()
                })
            );
            assert_eq!(args.angle, 5.0);
        }
        other => panic!("{other:?}"),
    }
    let scene = editor
        .handle(Command::Seek { marker: None })
        .scene
        .expect("the scene is sent");
    assert!(
        scene.part.solids.iter().any(|s| s.starts_with("draft(")),
        "{:?}",
        scene.part.solids
    );
}

/// A new rib starts from the newest sketch and solid; set to grow down,
/// parallel to its sketch, it builds.
#[test]
fn new_rib_grows_from_the_newest_sketch() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program: super::plastic_tests::enclosure_with_rib_sketch(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New { kind: "rib".into() });
    let step = update.step.expect("a step is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    editor.handle(dialog("direction", Value::Choice("parallel".into())));
    let update = editor.handle(dialog("flipped", Value::Bool(true)));
    let step = update.step.expect("the rib is edited");
    assert_eq!(step.error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::Rib(args) => {
            assert_eq!(args.sketch, "rib_sketch");
            assert_eq!(args.solid, "shell(s)");
            assert!(args.flipped);
        }
        other => panic!("{other:?}"),
    }
    let scene = update.scene.expect("the scene is sent anew");
    assert!(
        scene.part.solids.iter().any(|s| s.starts_with("rib(")),
        "{:?}",
        scene.part.solids
    );
}

/// A standard screw is placed as any part: offered among the files, its
/// size picked from its table, and mated by its datums — its axis on the
/// hole's, the underside of its head on the plate.
#[test]
fn a_standard_screw_is_placed_in_a_plate() {
    use geop_ops::part::{ParamValue, pose_parameter};

    let screw = "std:iso4762_socket_head_cap_screw.geop";
    let (plate, hole) = (examples::metric_plate(), examples::metric_plate_hole());
    let mut editor = Editor::<S>::new();
    let files = [("plate.geop".to_string(), Some(plate.to_json().unwrap()))].into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("assembly.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("plate.geop".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::Select { options, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen from a list");
    };
    assert!(options.iter().any(|o| o.value == screw), "{options:?}");
    editor.handle(dialog("file", Value::Choice(screw.into())));
    let update = editor.handle(dialog("parameter:size", Value::Choice("M4x12".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let face = |name: &str| EntityRef::Face { name: name.into() };
    let mates = [
        (
            "m1",
            "concentric",
            vec![
                EntityRef::datum("part2/axis"),
                face(&format!("part1/{hole}")),
            ],
        ),
        (
            "m2",
            "coincident",
            vec![
                EntityRef::datum("part2/seat"),
                face("part1/extrude(plate,end)"),
            ],
        ),
    ];
    for (id, kind, entities) in mates {
        editor.handle(dialog("add_mate", Value::Choice(kind.into())));
        let update = editor.handle(dialog(
            &format!("mate:{id}:entities"),
            Value::Entities(entities),
        ));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let PartOperation::AddPart(args) = &program.steps[1].operation else {
        panic!("the screw is placed");
    };
    assert_eq!(
        args.parameters.get("size"),
        Some(&ParamValue::Text("M4x12".into()))
    );
    assert!(editor.part().check_mates(|_| true).unwrap().converged);
    let Some(ParamValue::Pose(pose)) = program.state.get(&pose_parameter("part2")) else {
        panic!("the screw has a pose");
    };
    // Its head on the plate's top, its tip 12 below, down the hole.
    let v = |p: [f64; 3]| Vector3::from_array(p.map(geop_ops::Design::from_f64));
    for (local, want) in [
        ([0.0, 0.0, 0.0], [20.0, 20.0, 5.0]),
        ([0.0, 0.0, -12.0], [20.0, 20.0, -7.0]),
    ] {
        let got = pose.apply(&v(local));
        let got = [0, 1, 2].map(|k| got[k].to_f64());
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-6),
            "{local:?} is at {got:?}, not {want:?}"
        );
    }
}

/// A linear guide is placed from the standard parts as the front end does:
/// the rail, its size and length picked in its dialog, then a carriage of
/// the same size on it by a slider joint between their `axis` datums —
/// which leaves it the one motion along the rail, and puts it on the rail.
#[test]
fn a_carriage_slides_on_a_standard_rail() {
    use geop_ops::part::{ParamValue, pose_parameter};

    let (rail, carriage) = ("std:linear_rail.geop", "std:linear_carriage.geop");
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("guide.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    let update = editor.handle(Command::New {
        kind: "add_part".into(),
    });
    let step = update.step.unwrap();
    let Some(Control::Select { options, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen from a list");
    };
    for file in [rail, carriage, "std:spur_gear.geop", "std:servo_sg90.geop"] {
        assert!(
            options.iter().any(|o| o.value == file),
            "{file}: {options:?}"
        );
    }
    editor.handle(dialog("file", Value::Choice(rail.into())));
    editor.handle(dialog("fixed", Value::Bool(true)));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9".into())));
    let update = editor.handle(dialog("parameter:length", Value::Number(300.0)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice(carriage.into())));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9H".into())));
    editor.handle(dialog("add_mate", Value::Choice("slider".into())));
    let update = editor.handle(dialog(
        "mate:m1:entities",
        Value::Entities(vec![
            EntityRef::datum("part1/axis"),
            EntityRef::datum("part2/axis"),
        ]),
    ));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let PartOperation::AddPart(args) = &program.steps[0].operation else {
        panic!("the rail is placed");
    };
    assert_eq!(
        args.parameters.get("length"),
        Some(&ParamValue::Number(geop_ops::Design::from_f64(300.0)))
    );
    let part = editor.part();
    assert!(part.check_mates(|_| true).unwrap().converged);
    let joints = part.joints().unwrap();
    assert_eq!(joints.len(), 1, "{joints:?}");
    assert_eq!(joints[0].kind, "Slider");
    // On the rail: its axis on the rail's — the carriage drawn sitting on a
    // rail at the origin, so its pose turns nothing and moves it only along
    // the rail.
    let Some(ParamValue::Pose(pose)) = program.state.get(&pose_parameter("part2")) else {
        panic!("the carriage has a pose");
    };
    let origin = pose.apply(&Vector3::zero());
    let origin = [0, 1, 2].map(|k| origin[k].to_f64());
    assert!(
        origin[0].abs() < 1e-6 && origin[1].abs() < 1e-6,
        "{origin:?}"
    );
}

/// A robot of two boards and 20 screws, driven as the front end does: the
/// drag tool takes the screw under the pointer, and dragging it sends where
/// that screw went — not the hundred other placed parts, which stay where
/// they are, nor any part's looks again.
#[test]
fn dragging_one_of_many_placed_parts_sends_only_it() {
    use super::assembly_scale_tests::{down_onto, robot_editor};
    let (mut editor, _) = robot_editor(2, 20);
    let before = editor.program().state.clone();
    editor.handle(Command::DragTool { on: true });
    // The screws stand in a row along x at y = 0.25, from x = 0.25 on.
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down_onto(5.25, 0.25),
            shift: false,
        },
    });
    let tool = update.tool.expect("the drag tool is in hand");
    assert!(tool.grab, "a press over a screw grabs it");
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: down_onto(5.25, 0.25),
            to: down_onto(5.35, 0.3),
            done: true,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let scene = update.scene.expect("the screw moved");
    assert!(scene.components.is_empty() && scene.removed.is_empty() && !scene.all);
    let after = &editor.program().state;
    let moved: Vec<&String> = after
        .keys()
        .filter(|name| before.get(*name) != after.get(*name))
        .collect();
    assert_eq!(moved, ["screw10.pose"]);
    let sent: Vec<&str> = scene.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(sent, ["screw10"]);
}

/// A new SubD step starts from a box cage, its vertices, edges and faces
/// drawn to click. Clicked from above, the cage's top face is selected —
/// not the bottom one behind it — and its centre's coordinates shown;
/// Extrude pulls it out; dragged by its gizmo's `z` arrow and then by the
/// face itself, it moves; and the limit body the editor shows is the solid the
/// step builds.
#[test]
fn subd_cage_is_shaped_in_the_viewport() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::New {
        kind: "subd".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the subd is edited");
    for key in ["f9", "e4-5", "v6"] {
        assert!(
            step.presentation.visuals.iter().any(|v| v.key == key),
            "{key} is not drawn"
        );
    }
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let number =
        |step: &crate::editor::StepState<S>, key: &str| match step.presentation.dialog.get(key) {
            Some(Control::Number(n)) => n.value,
            other => panic!("{key}: {other:?}"),
        };
    let style = |step: &crate::editor::StepState<S>, key: &str| {
        step.presentation
            .visuals
            .iter()
            .find(|v| v.key == key)
            .map(|v| v.style)
    };
    let update = editor.handle(click(pointer([0.3, 0.2, 10.0], [0.0, 0.0, -1.0])));
    let step = update.step.expect("the subd is edited");
    assert_eq!(style(&step, "f9"), Some(geop_ops::ui::Style::Selected));
    assert_eq!(style(&step, "f8"), Some(geop_ops::ui::Style::Region));
    assert_eq!(number(&step, "z"), 1.0);

    let update = editor.handle(dialog("edit", Value::Choice("extrude".into())));
    let step = update.step.expect("the subd is edited");
    assert_eq!(step.error, None);
    assert_eq!(number(&step, "z"), 1.5);

    // The gizmo's `z` arrow, seen from the side, pulled up by a quarter —
    // shift held, so not snapped.
    let side = |dz: f64| pointer([0.0, -10.0, 1.55 + dz], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(0.0),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.0),
            to: side(0.25),
            done: true,
            shift: true,
        },
    });
    let step = update.step.expect("the subd is edited");
    assert!((number(&step, "z") - 1.75).abs() < 1e-9);

    // The face itself, grabbed looking down at 45 degrees and dragged in
    // the plane facing the eye: moving the eye up by `d` moves it by `d / 2`
    // up and `d / 2` back.
    let slanted = |dz: f64| pointer([0.3, -10.0, 11.75 + dz], [0.0, 1.0, -1.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: slanted(0.0),
            shift: false,
        },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the face offers a grab"
    );
    for (dz, done) in [(0.2, false), (0.5, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: slanted(0.0),
                to: slanted(dz),
                done,
                shift: false,
            },
        });
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let last = program.steps.last().unwrap();
    let PartOperation::Subd(args) = &last.operation else {
        panic!("{:?}", last.operation);
    };
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 2.0).abs() < 1e-9, "{at:?}");
    }
    assert_eq!(args.cage.faces.len(), 10);
    let scene = update.scene.expect("the part changed");
    assert_eq!(scene.part.solids, [format!("subd({})", last.id)]);
}

/// The measure tool, used as the viewer uses it: taken in hand, the box's
/// top hovered and clicked — its area, its plane — then its bottom clicked
/// from below: one unit apart, parallel, the least distance drawn between
/// them. Then the mass properties and the interference of the part asked
/// for. None of it changes the program, and the tool put down shows nothing.
#[test]
fn measure_tool_measures_what_is_clicked() {
    use crate::inspect::{Inspection, Query};
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let update = editor.handle(Command::MeasureTool { on: true });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert!(update.program.unwrap().measure_tool);
    let top = EntityRef::Face {
        name: "extrude(box,end)".into(),
    };
    let bottom = EntityRef::Face {
        name: "extrude(box,start)".into(),
    };
    let from_above = pointer([0.5, 0.5, 10.0], [0.0, 0.0, -1.0]);
    let from_below = pointer([0.5, 0.5, -10.0], [0.0, 0.0, 1.0]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let measured = |update: &Update<S>| match &update.inspection {
        Some(Inspection::Measure(m)) => m.clone(),
        other => panic!("no measurement: {other:?}"),
    };
    let value = |m: &geop_ops_inspect::Measurement<S>, label: &str| {
        m.values
            .iter()
            .find(|v| v.label == label)
            .unwrap_or_else(|| panic!("no {label}: {m:?}"))
            .value
    };

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above,
            shift: false,
        },
    });
    assert_eq!(update.tool.unwrap().highlights, vec![top.clone()]);

    let update = editor.handle(click(from_above));
    assert!(update.program.is_none(), "measuring changes no program");
    let m = measured(&update);
    assert_eq!(m.entities, vec![top.clone()]);
    let hole = std::f64::consts::PI * 0.4 * 0.4;
    assert!(value(&m, "Area").contains(4.0 - hole), "{m:?}");
    assert!(m.plane.is_some());

    let update = editor.handle(click(from_below));
    let m = measured(&update);
    assert_eq!(m.entities, vec![top, bottom]);
    assert!(value(&m, "Distance").contains(1.0), "{m:?}");
    assert!(value(&m, "Angle").contains(0.0), "{m:?}");
    let tool = update.tool.unwrap();
    assert!(tool.visuals.iter().any(|v| v.key == "distance"));

    let update = editor.handle(Command::Inspect {
        query: Query::MassProperties,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let Some(Inspection::MassProperties(report)) = update.inspection else {
        panic!("no mass properties: {:?}", update.inspection);
    };
    assert_eq!(report.bodies.len(), 1);
    let volume = report.total.unwrap().volume;
    assert!(volume.contains(4.0 - hole * 0.5), "{volume:?}");

    let update = editor.handle(Command::Inspect {
        query: Query::Interference,
    });
    let Some(Inspection::Interference(report)) = update.inspection else {
        panic!("no interference: {:?}", update.inspection);
    };
    assert_eq!(report.solids, 1);
    assert!(report.found.is_empty());

    let update = editor.handle(Command::MeasureTool { on: false });
    assert!(update.tool.is_none() && update.inspection.is_none());
    assert_eq!(*editor.program(), before);
}

/// Where the vertices of the drawing being edited are on its sheet, in the
/// view `view`, as the front end shows them: by name.
fn sheet_vertices(
    editor: &Editor<S>,
    view: geop_ops_drawing::DrawnView,
) -> Vec<(String, [f64; 2])> {
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing is edited");
    };
    let layout = geop_ops_drawing::layout(editor.part(), args, "", &[]).unwrap();
    layout
        .candidates
        .into_iter()
        .filter(|c| c.view == view)
        .filter_map(|c| match c.what {
            geop_ops_drawing::Pickable::Point {
                target: geop_ops_drawing::Target::Vertex { name },
                at,
            } => Some((name, at)),
            _ => None,
        })
        .collect()
}

/// The drilled hole's diameter dimensioned on the drawing's sheet by
/// clicking its rim from above, and a centre mark put on it; the diameter,
/// selected in the list, removed with Delete.
#[test]
fn a_hole_is_dimensioned_on_a_drawing_and_a_dimension_removed() {
    use geop_ops_drawing::{Annotation, DrawnView, EdgeShape, Pickable, ViewKind};
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "drawing".into(),
    });
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing is edited");
    };
    let layout = geop_ops_drawing::layout(editor.part(), args, "", &[]).unwrap();
    let rim = layout
        .candidates
        .iter()
        .filter(|c| c.view == DrawnView::View(ViewKind::Top))
        .find_map(|c| match &c.what {
            Pickable::Edge {
                points,
                shape: EdgeShape::Circle { .. },
                ..
            } => Some(points[points.len() / 2]),
            _ => None,
        })
        .expect("the hole's rim, seen round from above");
    let click = |p: [f64; 2]| Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer([p[0], p[1], 10.0], [0.0, 0.0, -1.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let key = |key: &str| Command::Event {
        event: StepEditEvent::Key { key: key.into() },
    };
    editor.handle(dialog("tools", Value::Choice("diameter".into())));
    editor.handle(click(rim));
    let placed = editor.handle(click([rim[0] + 15.0, rim[1] + 15.0]));
    let step = placed.step.unwrap();
    let Some(Control::List { items, .. }) = step.presentation.dialog.get("annotations") else {
        panic!("the annotations are listed");
    };
    let detail = items[0].detail.clone().unwrap_or_default();
    assert!(
        detail.starts_with('⌀') && detail.ends_with("top"),
        "{items:?}"
    );
    editor.handle(dialog("tools", Value::Choice("center_mark".into())));
    editor.handle(click(rim));
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing");
    };
    assert!(matches!(
        args.annotations.as_slice(),
        [
            Annotation::Radius { diameter: true, .. },
            Annotation::CenterMark { .. }
        ]
    ));

    editor.handle(key("Escape"));
    editor.handle(dialog("annotation:0", Value::Press));
    let update = editor.handle(key("Delete"));
    assert!(update.error.is_none(), "{:?}", update.error);
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing");
    };
    assert!(
        matches!(args.annotations.as_slice(), [Annotation::CenterMark { .. }]),
        "{:?}",
        args.annotations
    );
}

/// A drawing step, the way the front end makes one: started — its sheet
/// shown head on, no part drawn — its views chosen and the title block's
/// name typed; a diagonal of the box's top dimensioned by clicking two of
/// its corners in the top view and then where the value goes, the value
/// dragged further out, a note added pointing at a corner; then
/// downloaded from the dialog as SVG and DXF, and committed.
#[test]
fn a_drawing_is_annotated_on_its_sheet_and_downloaded() {
    use geop_ops_drawing::{Annotation, DrawnView, Format, ViewKind};
    let (mut editor, _) = editor();
    let started = editor.handle(Command::New {
        kind: "drawing".into(),
    });
    assert!(started.error.is_none(), "{:?}", started.error);
    let step = started.step.unwrap();
    // The sheet, in the world's xy plane, and nothing of the part.
    let sheet = step.presentation.sheet.expect("the sheet is shown");
    assert!(step.presentation.focus.is_some());
    assert!(sheet.size.to_f64() > 400.0, "an A3 sheet");
    let scene = started.scene.expect("a scene");
    assert!(scene.part.faces.is_empty() && scene.part.datums.is_empty());
    assert!(matches!(
        step.presentation.dialog.get("download"),
        Some(Control::Download { formats }) if formats.len() == 2
    ));
    editor.handle(dialog("view:iso", Value::Bool(false)));
    editor.handle(dialog("title:name", Value::Text("Drilled box".into())));

    // Two opposite corners of the box's top, seen from above.
    let top = DrawnView::View(ViewKind::Top);
    let corners = sheet_vertices(&editor, top);
    let (low, high) = {
        let by = |f: fn(f64, f64) -> bool| {
            corners
                .iter()
                .cloned()
                .reduce(|a, b| {
                    if f(b.1[0] + b.1[1], a.1[0] + a.1[1]) {
                        b
                    } else {
                        a
                    }
                })
                .unwrap()
        };
        (by(|a, b| a < b), by(|a, b| a > b))
    };
    let at = |p: [f64; 2]| pointer([p[0], p[1], 10.0], [0.0, 0.0, -1.0]);
    let event = |event| Command::Event { event };
    let click = |p: [f64; 2]| {
        event(StepEditEvent::Click {
            pointer: at(p),
            button: Button::Primary,
            double: false,
            shift: false,
        })
    };
    let hover = |p: [f64; 2]| {
        event(StepEditEvent::Hover {
            pointer: at(p),
            shift: false,
        })
    };
    editor.handle(dialog("tools", Value::Choice("distance".into())));
    let hovered = editor.handle(hover(low.1)).step.unwrap();
    assert!(
        hovered
            .presentation
            .visuals
            .iter()
            .any(|v| v.key == "hover"),
        "the corner under the pointer is lit"
    );
    editor.handle(click(low.1));
    editor.handle(click(high.1));
    // The dimension follows the pointer, and goes down where clicked.
    let middle = [(low.1[0] + high.1[0]) / 2.0, (low.1[1] + high.1[1]) / 2.0];
    let out = [middle[0] - 10.0, middle[1] + 10.0];
    let placing = editor.handle(hover(out)).step.unwrap();
    assert!(
        placing
            .presentation
            .visuals
            .iter()
            .any(|v| v.key == "placing"),
        "the dimension being placed is shown"
    );
    editor.handle(click(out));
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing");
    };
    let [
        Annotation::Distance {
            from, to, label, ..
        },
    ] = args.annotations.as_slice()
    else {
        panic!("one distance: {:?}", args.annotations);
    };
    assert_eq!(
        (from.to_string(), to.to_string()),
        (low.0.clone(), high.0.clone())
    );
    assert!(
        (label[0] + 10.0).abs() < 1e-9 && (label[1] - 10.0).abs() < 1e-9,
        "{label:?}"
    );

    // Put down, the tool lets its value be dragged.
    editor.handle(event(StepEditEvent::Key {
        key: "Escape".into(),
    }));
    let update = editor.handle(event(StepEditEvent::Key {
        key: "Escape".into(),
    }));
    let step = update.step.unwrap();
    let Some(Control::List { items, .. }) = step.presentation.dialog.get("annotations") else {
        panic!("the annotations are listed");
    };
    assert_eq!(items[0].detail.as_deref(), Some("2.83 · top"), "{items:?}");
    editor.handle(hover(out));
    let dragged = editor.handle(event(StepEditEvent::Drag {
        from: at(out),
        to: at([out[0] - 5.0, out[1] + 5.0]),
        done: true,
        shift: false,
    }));
    assert!(dragged.error.is_none(), "{:?}", dragged.error);
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing");
    };
    let label = match &args.annotations[0] {
        Annotation::Distance { label, .. } => *label,
        other => panic!("{other:?}"),
    };
    assert!(
        (label[0] + 15.0).abs() < 1e-9 && (label[1] - 15.0).abs() < 1e-9,
        "{label:?}"
    );

    // A note, pointing at a corner.
    editor.handle(dialog("tools", Value::Choice("note".into())));
    editor.handle(dialog("note_text", Value::Text("DEBURR".into())));
    editor.handle(click(low.1));
    editor.handle(click([low.1[0] - 20.0, low.1[1] - 20.0]));
    let Some(PartOperation::Drawing(args)) = editor.editing() else {
        panic!("a drawing");
    };
    assert!(
        matches!(&args.annotations[1], Annotation::Note { text, leader: Some(_), .. } if text == "DEBURR"),
        "{:?}",
        args.annotations
    );

    // Downloaded from the dialog, as it now is.
    let exported = editor.handle(Command::ExportDrawing {
        id: None,
        format: Format::Svg,
        date: "2026-10-04".into(),
    });
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let file = exported.export.unwrap();
    assert_eq!(file.name, "drawing.svg");
    let svg = file.text().unwrap();
    assert!(svg.starts_with("<svg"));
    for text in ["Drilled box", ">2.83<", ">DEBURR<"] {
        assert!(svg.contains(text), "{text} is not on the sheet");
    }
    // The blind hole, seen from the front, is hidden.
    let hidden = &svg[svg.find(r#"<g class="HIDDEN""#).unwrap()..];
    assert!(hidden[..hidden.find("</g>").unwrap()].contains("<line"));
    let dxf = editor.handle(Command::ExportDrawing {
        id: None,
        format: Format::Dxf,
        date: String::new(),
    });
    let dxf = dxf.export.unwrap();
    assert!(dxf.text().unwrap().ends_with("EOF\n") && dxf.text().unwrap().contains("DEBURR"));

    let committed = editor.handle(Command::Commit);
    assert!(committed.error.is_none(), "{:?}", committed.error);
    // Back to the part: the scene draws its faces again.
    assert!(!committed.scene.expect("a scene").part.faces.is_empty());
    let PartOperation::Drawing(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a drawing step");
    };
    assert_eq!((args.views.len(), args.annotations.len()), (3, 2));
    assert_eq!(args.name, "Drilled box");
}

/// The arm, exported as a URDF robot the way the front end asks — the
/// command and the update as JSON: a ZIP archive named after the file, its
/// bytes in base64, holding `robot.urdf` with the arm's joints. Refused
/// while a step is edited, and for the four-bar, whose bars only mates
/// hold: the error names them.
#[test]
fn an_assembly_is_exported_as_urdf() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let json = editor.handle_json(r#"{"command": "export_urdf"}"#).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    let export = &update["export"];
    assert_eq!(export["name"], "arm.zip");
    assert!(export["text"].is_null());
    // "PK\x03\x04", a ZIP archive's first local header, in base64.
    let bytes = export["bytes"].as_str().unwrap();
    assert!(bytes.starts_with("UEsDBA"), "{}", &bytes[..16]);
    assert_eq!(bytes.len() % 4, 0);

    let crate::editor::Content::Bytes(zip) =
        editor.handle(Command::ExportUrdf).export.unwrap().content
    else {
        panic!("a binary file");
    };
    let text = String::from_utf8_lossy(&zip);
    assert!(text.contains("robot.urdf") && text.contains("meshes/link.stl"));
    assert!(text.contains(r#"<joint name="add_part(fore,m1)" type="revolute">"#));
    assert!(text.contains(r#"<joint name="add_part(hand,m1)" type="revolute">"#));

    editor.handle(Command::Open { id: "fore".into() });
    let refused = editor.handle(Command::ExportUrdf);
    assert!(refused.export.is_none());
    assert!(refused.error.unwrap().contains("finish editing"));
    editor.handle(Command::Cancel);

    editor.handle(Command::LoadWorkspaceExample {
        name: "four_bar".into(),
        folder: None,
    });
    let refused = editor.handle(Command::ExportUrdf);
    assert!(refused.export.is_none());
    let error = refused.error.unwrap();
    assert!(
        error.contains("free to move") && error.contains("coupler"),
        "{error}"
    );
}

/// "Download STEP" writes the part shown; stored next to a new program in a
/// folder — as the front end stores a file the user chooses in the file
/// field — the file is offered by a new "Import STEP" step by its path
/// relative to the program, and brings its solid back, named after the
/// step.
#[test]
fn a_part_exported_as_step_is_imported_back() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::ExportStep);
    assert!(update.error.is_none(), "{:?}", update.error);
    let export = update.export.expect("a file is written");
    assert!(export.name.ends_with(".step"), "{}", export.name);
    assert!(export.text().unwrap().contains("MANIFOLD_SOLID_BREP"));

    let files = std::collections::BTreeMap::from([(
        "parts/box.step".to_string(),
        Some(export.text().unwrap().to_string()),
    )]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("parts/imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Import STEP");
    let Some(Control::File {
        options, accept, ..
    }) = step.presentation.dialog.get("file")
    else {
        panic!("the file is a file field");
    };
    assert!(options.iter().any(|o| o.value == "box.step"), "{options:?}");
    assert_eq!(accept, &["step", "stp"]);
    let update = editor.handle(dialog("file", Value::Choice("box.step".into())));
    assert!(update.step.unwrap().error.is_none());
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let id = editor.program().steps.last().unwrap().id.clone();
    let solids: Vec<String> = update
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == crate::editor::StructureKind::Solid)
        .map(|item| item.name)
        .collect();
    assert_eq!(solids, vec![format!("import({id},s0)")]);
}

/// A STEP file whose corner lies 3e-4 above where its three faces meet —
/// further than the kernel can carry as one point — imports with the corner
/// put back where they meet, and the dialog says what was rebuilt, and how
/// far it moved.
#[test]
fn an_import_that_heals_the_file_says_what_it_rebuilt() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::ExportStep);
    let text = update
        .export
        .expect("a file is written")
        .text()
        .unwrap()
        .to_string();
    // The first vertex's point, raised.
    let at = text.find("=VERTEX_POINT(").expect("a vertex");
    let id = &text[at..];
    let id = &id[id.find("',#").unwrap() + 3..id.find(");").unwrap()];
    let line = format!("#{id}=CARTESIAN_POINT('',(");
    let start = text.find(&line).expect("its point") + line.len();
    let end = start + text[start..].find("))").unwrap();
    let mut xyz: Vec<f64> = text[start..end]
        .split(',')
        .map(|c| c.parse().unwrap())
        .collect();
    xyz[2] += 3e-4;
    let raised = format!(
        "{}{:?},{:?},{:?}{}",
        &text[..start],
        xyz[0],
        xyz[1],
        xyz[2],
        &text[end..]
    );

    let files = std::collections::BTreeMap::from([("box.step".to_string(), Some(raised))]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let update = editor.handle(dialog("file", Value::Choice("box.step".into())));
    let step = update.step.expect("a step is edited");
    assert!(step.error.is_none(), "{:?}", step.error);
    let Some(Control::Text { text, .. }) = step.presentation.dialog.get("healed_0") else {
        panic!(
            "the dialog does not say what was rebuilt: {:?}",
            step.presentation.dialog
        );
    };
    assert!(
        text.contains("by up to 3.0e-4 mm") && text.contains("1 vertices and"),
        "{text}"
    );
    assert!(editor.handle(Command::Commit).error.is_none());
}

/// "Download STL" writes every solid of the part shown as a binary STL
/// mesh, named after the program's file — the placed parts' solids too, so
/// the pin in the plate adds to the plate's triangles.
#[test]
fn a_part_is_exported_as_stl_with_its_placed_parts() {
    /// The triangles of a binary STL file, checked against its length.
    fn triangles(update: Update<S>) -> u32 {
        assert!(update.error.is_none(), "{:?}", update.error);
        let export = update.export.expect("a file is written");
        let crate::editor::Content::Bytes(bytes) = export.content else {
            panic!("a binary file");
        };
        assert!(!bytes.starts_with(b"solid"), "a binary header");
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
        assert_eq!(bytes.len(), 84 + 50 * count as usize);
        count
    }
    let (mut editor, _) = editor();
    let plate = triangles(editor.handle(Command::ExportStl));
    assert!(plate > 0);

    editor.handle(Command::LoadWorkspaceExample {
        name: "pin_in_plate".into(),
        folder: None,
    });
    let json = editor.handle_json(r#"{"command": "export_stl"}"#).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(update["export"]["name"], "assembly.stl");
    let assembly = triangles(editor.handle(Command::ExportStl));
    assert!(
        assembly > plate,
        "{assembly} triangles, the plate alone {plate}"
    );
}

/// A fillet whose radius varies, set up as the front end does: the box's
/// upright edge at the origin picked, "variable radius" ticked — which
/// brings up the end radius, starting at the radius — the end radius set,
/// then the corner on top picked as a vertex to give a radius of its own,
/// which brings up a number for it. The step builds, rounded.
#[test]
fn variable_fillet_set_up_in_the_dialog() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "fillet".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // Diagonally onto the upright edge at the origin, halfway up.
    let update = editor.handle(click(pointer([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(dialog("variable", Value::Bool(true)));
    let step = update.step.expect("the fillet is edited");
    let number =
        |step: &crate::editor::StepState<S>, key: &str| match step.presentation.dialog.get(key) {
            Some(Control::Number(n)) => n.value,
            other => panic!("{key}: {other:?}"),
        };
    assert_eq!(number(&step, "end_radius"), number(&step, "radius"));
    editor.handle(dialog("end_radius", Value::Number(0.2)));
    // The corner on top of that edge, picked from above and outside.
    editor.handle(dialog("radius_vertices", Value::Press));
    let update = editor.handle(click(pointer([-5.0, -5.0, 6.0], [1.0, 1.0, -1.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the fillet is edited");
    assert_eq!(number(&step, "vertex_radius_0"), number(&step, "radius"));
    let update = editor.handle(dialog("vertex_radius_0", Value::Number(0.15)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let last = program.steps.last().unwrap();
    match &last.operation {
        PartOperation::Fillet(args) => {
            assert_eq!(args.edges.len(), 1, "{args:?}");
            assert_eq!(args.end_radius, Some(0.2.into()), "{args:?}");
            assert_eq!(args.vertex_radii.len(), 1, "{args:?}");
            assert_eq!(args.vertex_radii[0].radius, 0.15, "{args:?}");
        }
        other => panic!("{other:?}"),
    }
    let part = program.build::<S>(&geop_ops::NoFiles).unwrap();
    assert!(
        part.solid_names().iter().any(|s| s.starts_with("fillet(")),
        "{:?}",
        part.solid_names()
    );
}

/// The three edges at the box's corner `(0, 0, 1)` picked one after the
/// other in a new fillet, as the front end does: the edges field holds all
/// three, and the step builds the box with that corner rounded by a ball's
/// piece, named after the corner's vertex.
#[test]
fn fillet_rounds_a_corner_picked_edge_by_edge() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "fillet".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // The upright edge, diagonally halfway up; the top's edges along x and
    // y, from above and outside.
    for (origin, dir) in [
        ([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0]),
        ([0.5, -5.0, 6.0], [0.0, 1.0, -1.0]),
        ([-5.0, 0.5, 6.0], [1.0, 0.0, -1.0]),
    ] {
        let update = editor.handle(click(pointer(origin, dir)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(dialog("radius", Value::Number(0.2)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let PartOperation::Fillet(args) = &program.steps.last().unwrap().operation else {
        panic!("not a fillet");
    };
    assert_eq!(args.edges.len(), 3, "{args:?}");
    let part = program.build::<S>(&geop_ops::NoFiles).unwrap();
    let corners: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|name| name.ends_with(",corner)"))
        .collect();
    assert_eq!(corners.len(), 1, "{corners:?}");
    assert!(super::regression_tests::check_valid(&part).is_ok());
}

/// The plate's thickness, typed into its extrude's dialog as a formula of
/// the parameters, as one types it in the parameters panel: the field
/// shows the formula and what it comes to, and drops its slider and its
/// handle — a value that follows a formula is not dragged. A formula that
/// does not evaluate is kept as typed, the field and the step saying why;
/// a number typed makes it plain, and draggable, again.
#[test]
fn formulas_are_typed_into_dialog_fields() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadExample {
        name: "parametric_plate".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let field = |step: &crate::editor::StepState<S>| match step.presentation.dialog.get("distance")
    {
        Some(Control::Number(n)) => n.clone(),
        other => panic!("{other:?} is no number field"),
    };
    let handle = |step: &crate::editor::StepState<S>| {
        step.presentation
            .visuals
            .iter()
            .any(|v| v.key == "distance" && matches!(v.shape, geop_ops::ui::Shape::Handle { .. }))
    };
    let thickness = |editor: &Editor<S>| {
        let program = editor.program();
        match &program.steps[program.index_of("plate").unwrap()].operation {
            PartOperation::Extrude(args) => args.extent.side1.clone(),
            other => panic!("{other:?}"),
        }
    };
    let top = |editor: &Editor<S>| {
        editor
            .part()
            .topology()
            .vertices
            .values()
            .map(|v| v.point[2].to_f64())
            .fold(f64::NEG_INFINITY, f64::max)
    };

    let update = editor.handle(Command::Open { id: "plate".into() });
    let step = update.step.expect("the plate is edited");
    // The example's own formula.
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("thickness"));
    assert_eq!((n.value, n.range, n.error), (0.5, None, None));
    assert!(!handle(&step), "a formula's value has a handle");

    let update = editor.handle(dialog("distance", Value::Text("3 * thickness".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the plate is edited");
    assert_eq!(step.error, None);
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("3 * thickness"));
    assert!((n.value - 1.5).abs() < 1e-12, "{}", n.value);
    assert!(n.range.is_none() && !handle(&step));

    let update = editor.handle(dialog("distance", Value::Text("3 * thicknes".into())));
    let step = update.step.expect("the plate is edited");
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("3 * thicknes"));
    assert_eq!(
        n.error.as_deref(),
        Some(r#"there is no parameter "thicknes""#)
    );
    let error = step.error.expect("the step fails");
    assert!(error.contains(r#""3 * thicknes""#), "{error}");

    editor.handle(dialog("distance", Value::Text("3 * thickness".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(
        thickness(&editor),
        geop_ops_extrude_revolve::Extent::blind("3 * thickness")
    );
    assert!((top(&editor) - 1.5).abs() < 1e-9, "{}", top(&editor));

    // A number typed is plain again: slid and dragged.
    editor.handle(Command::Open { id: "plate".into() });
    let update = editor.handle(dialog("distance", Value::Text(" 0.8 ".into())));
    let step = update.step.expect("the plate is edited");
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("0.8"));
    assert!(n.range.is_some() && handle(&step));
    editor.handle(Command::Commit);
    assert_eq!(
        thickness(&editor),
        geop_ops_extrude_revolve::Extent::blind(0.8)
    );
}

/// The bill of materials, asked for the way the inspect panel asks — the
/// command and the update as JSON: the bolted plate's three parts, the
/// screw and nut designated by their norms. Of the part shown: seeking
/// back before the nut drops it. Exported as CSV, indented, named after
/// the file.
#[test]
fn the_bill_of_materials_is_asked_for_and_exported() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let ask = r#"{"command": "inspect", "query": {"bom": {"structure": "flat"}}}"#;
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    let bom = &update["inspection"];
    assert_eq!(bom["kind"], "bom");
    assert_eq!(bom["structure"], "flat");
    let designations: Vec<&str> = bom["lines"]
        .as_array()
        .unwrap()
        .iter()
        .map(|l| l["designation"].as_str().unwrap_or("-"))
        .collect();
    assert_eq!(designations, ["-", "ISO 4762 M4x12", "ISO 4032 M4"]);
    let screw = &bom["lines"][1];
    assert_eq!(
        (&screw["kind"], &screw["quantity"]),
        (&"part".into(), &1.into())
    );
    assert_eq!(screw["material"], "Steel");
    assert!(screw["unit_mass"]["value"].as_f64().unwrap() > 0.0);
    assert!(bom["total_mass"]["value"].as_f64().unwrap() > 0.0);

    // Up to the screw: no nut yet.
    editor.handle(Command::Seek { marker: Some(2) });
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(update["inspection"]["lines"].as_array().unwrap().len(), 2);
    editor.handle(Command::Seek { marker: None });

    let json = editor
        .handle_json(r#"{"command": "export_bom", "structure": "indented"}"#)
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["export"]["name"], "bolted_plate_bom.csv");
    let csv = update["export"]["text"].as_str().unwrap();
    let rows: Vec<&str> = csv.lines().collect();
    assert_eq!(rows.len(), 1 + 3 + 1, "{csv}");
    assert!(
        rows[2].starts_with("2,1,1,ISO 4762 socket head cap screw,ISO 4762 M4x12,"),
        "{csv}"
    );
    assert!(rows[4].contains("Total"), "{csv}");
}

/// The arm, exported as STEP the way the front end asks: one product for
/// the arm and one for the link it places three times, each placement an
/// occurrence named after its step — and read back, the three links where
/// the arm has them.
#[test]
fn an_assembly_is_exported_as_step_products() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let json = editor.handle_json(r#"{"command": "export_step"}"#).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert!(
        update["export"]["name"]
            .as_str()
            .unwrap()
            .ends_with(".step")
    );
    let text = update["export"]["text"].as_str().unwrap();
    assert_eq!(
        text.matches("=PRODUCT('").count(),
        2,
        "the arm and the link"
    );
    assert!(text.contains("PRODUCT('link','link'"));
    for occurrence in ["upper", "fore", "hand"] {
        assert!(
            text.contains(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{occurrence}'")),
            "{occurrence} is placed"
        );
    }
    let bodies = geop_ops_step::read_step::<S>(text).unwrap();
    assert_eq!(bodies.len(), 3, "the link, once where each step places it");
}

/// Sheet metal the way the front end does it, on the bracket: a sketch on
/// the back flange cut through it by a new sheet-metal cut that takes the
/// newest sketch and picks the flange by a click; a new hem picked on the
/// plate's right edge and opened in the dialog; and the flat pattern
/// exported for laser cutting, as the File menu asks for it.
#[test]
fn sheet_metal_cut_hem_and_cutting_export() {
    use geop_ops_sheetmetal::HemKind;
    use geop_ops_sketch::AddSketchArgs;

    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadExample {
        name: "sheet_metal_bracket".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    // A hole drawn on the back flange's outside, 0.3 up: the face's sketch
    // `x` runs along the world's, its `y` down.
    let mut program = editor.program().clone();
    let mut hole = geop_ops_sketch::Sketch::new();
    let c = hole.add_point(1.0.into(), (-0.3).into());
    hole.add_circle(c, 0.05.into());
    program.push(
        "flange_hole",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "edge_flange(back,flange,a)".into(),
            }),
            sketch: hole,
            ..Default::default()
        },
    );
    let update = editor.handle(Command::Load {
        program,
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);

    editor.handle(Command::New {
        kind: "sheet_cut".into(),
    });
    // The face is the one the sketch lies on unless picked: pressed, it
    // takes the click.
    editor.handle(dialog("face", Value::Press));
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer([1.0, 5.0, 0.3], [0.0, -1.0, 0.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = editor.program().steps.last().unwrap().clone();
    match &step.operation {
        PartOperation::SheetCut(args) => {
            assert_eq!(args.sketch, "flange_hole");
            assert_eq!(args.face, "edge_flange(back,flange,a)");
        }
        other => panic!("{other:?}"),
    }
    let scene = update.scene.expect("the scene is sent anew");
    assert_eq!(scene.part.solids, [format!("sheet_cut({})", step.id)]);

    editor.handle(Command::New { kind: "hem".into() });
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer([2.0, 0.4, 10.0], [0.0, 0.0, -1.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("kind", Value::Choice("open".into())));
    editor.handle(dialog("gap", Value::Number(0.06)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = editor.program().steps.last().unwrap().clone();
    match &step.operation {
        PartOperation::Hem(args) => {
            assert_eq!(args.edge, "base_flange(plate,outline,c5,b)");
            assert_eq!((args.kind, args.gap), (HemKind::Open, 0.06));
        }
        other => panic!("{other:?}"),
    }
    let scene = update.scene.expect("the scene is sent anew");
    assert_eq!(scene.part.solids, [format!("hem({})", step.id)]);

    let json = editor
        .handle_json(r#"{"command": "export_flat_pattern"}"#)
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["export"]["name"], "part_flat.dxf");
    let dxf = update["export"]["text"].as_str().unwrap();
    assert_eq!(dxf.matches("\nUP 90%%d R0.08\n").count(), 2, "{dxf}");
    assert_eq!(dxf.matches("\nUP 180%%d R0.03\n").count(), 1, "{dxf}");
    // Three holes, each one circle.
    assert_eq!(dxf.matches("\nCIRCLE\n8\nCUT\n").count(), 3, "{dxf}");
}

/// The sketch tools the way the front end uses them: a plate drawn as a
/// rectangle with a chamfered corner, an arc slot and a circumscribed hex
/// hole in it, and round it all an offset rim — a second sketch offset into
/// a ring by clicks. Each extrudes into a valid solid.
#[test]
fn sketch_tools_draw_profiles_that_extrude() {
    let mut editor = Editor::<S>::new();
    let event = |editor: &mut Editor<S>, event: StepEditEvent<S>| {
        let update = editor.handle(Command::Event { event });
        assert!(update.error.is_none(), "{:?}", update.error);
        update
    };
    let click = |editor: &mut Editor<S>, x: f64, y: f64| {
        event(
            editor,
            StepEditEvent::Click {
                pointer: pointer([x, y, 10.0], [0.0, 0.0, -1.0]),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        )
    };
    let hover = |editor: &mut Editor<S>, x: f64, y: f64| {
        event(
            editor,
            StepEditEvent::Hover {
                pointer: pointer([x, y, 10.0], [0.0, 0.0, -1.0]),
                shift: false,
            },
        )
    };
    let key =
        |editor: &mut Editor<S>, key: &str| event(editor, StepEditEvent::Key { key: key.into() });
    let tool = |editor: &mut Editor<S>, name: &str| {
        editor.handle(dialog("tool", Value::Choice(name.into())))
    };
    let extruded = |editor: &mut Editor<S>| {
        let update = editor.handle(Command::Commit);
        assert!(update.error.is_none(), "{:?}", update.error);
        editor.handle(Command::New {
            kind: "extrude".into(),
        });
        editor.handle(dialog("distance", Value::Number(0.3)));
        let update = editor.handle(Command::Commit);
        assert!(update.error.is_none(), "{:?}", update.error);
        let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
        if let Err(e) = super::regression_tests::check_valid(&part) {
            panic!("{e}");
        }
        part
    };
    // A new sketch on the origin's xy plane, seen from above.
    let new_sketch = |editor: &mut Editor<S>| {
        editor.handle(Command::New {
            kind: "add_sketch".into(),
        });
        click(editor, 0.04, 0.04);
    };

    new_sketch(&mut editor);
    tool(&mut editor, "rectangle");
    click(&mut editor, 0.2, 0.2);
    click(&mut editor, 2.6, 1.8);
    tool(&mut editor, "chamfer");
    let update = click(&mut editor, 2.6, 1.8);
    let prompt = update.step.unwrap().presentation.prompt;
    assert!(prompt.is_some(), "the chamfer's size is asked for");
    editor.handle(dialog("prompt", Value::Text("0.2".into())));
    // An arc slot about (1.2, 0.6), its arc from (1.6, 0.6) half round.
    tool(&mut editor, "arc_slot");
    click(&mut editor, 1.2, 0.6);
    click(&mut editor, 1.6, 0.6);
    hover(&mut editor, 1.2, 1.0);
    click(&mut editor, 0.8, 0.6);
    click(&mut editor, 1.7, 0.6);
    // A hexagon 0.3 across its flats.
    tool(&mut editor, "polygon");
    editor.handle(dialog("circumscribed", Value::Bool(true)));
    click(&mut editor, 2.1, 0.7);
    let update = click(&mut editor, 2.25, 0.7);
    let step = update.step.unwrap();
    match step.presentation.dialog.get("status") {
        Some(Control::Text { text, tone }) => {
            assert!(text.contains("1 region"), "{text}");
            assert_ne!(*tone, Tone::Error, "{text}");
        }
        other => panic!("{other:?}"),
    }
    key(&mut editor, "Escape");
    let part = extruded(&mut editor);
    assert_eq!(part.solid_names().len(), 1);

    // A rim round a 1 x 1 square, offset outwards by clicks.
    new_sketch(&mut editor);
    tool(&mut editor, "rectangle");
    click(&mut editor, 4.0, 0.2);
    click(&mut editor, 5.0, 1.2);
    key(&mut editor, "o");
    click(&mut editor, 4.5, 0.2);
    hover(&mut editor, 4.5, 0.05);
    click(&mut editor, 4.5, 0.05);
    editor.handle(dialog("prompt", Value::Text("0.1".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::AddSketch(rim) = &editor.program().steps.last().unwrap().operation else {
        panic!("a sketch");
    };
    let regions = rim.sketch.regions().unwrap();
    assert_eq!(regions.len(), 1, "a ring");
    assert_eq!(regions[0].holes.len(), 1, "round the square");
    let id = editor.program().steps.last().unwrap().id.clone();
    editor.handle(Command::Open { id });
    extruded(&mut editor);
}

/// What reads each parameter is sent with the program, for the parameters
/// panel to warn before one is removed; and renaming one, as its dialog's
/// name field does, renames it in every formula — the other parameters',
/// a sketch's dimensions, an extrude's length, a table's columns — so
/// nothing fails and the part is the same. A name taken, and a rename
/// while a step is edited, are refused; undo takes a rename back.
#[test]
fn renaming_a_parameter_renames_what_reads_it() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadExample {
        name: "parametric_plate".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let uses = update.program.expect("the program is sent").parameter_uses;
    let readers = |name: &str| uses.get(name).cloned().unwrap_or_default();
    assert_eq!(readers("width"), ["depth", "outline"]);
    assert_eq!(readers("depth"), ["outline"]);
    assert_eq!(readers("thickness"), ["plate"]);
    assert_eq!(readers("screw"), ["hole_sketch"]);
    let volume = |editor: &Editor<S>| {
        let part = editor.part();
        let [name] = part.solid_names().try_into().expect("one solid");
        geop_ops_inspect::bodies::PlacedSolid {
            solid: part.solid_id(&name).unwrap(),
            name,
            part,
            component: None,
            pose: None,
        }
        .mass_properties()
        .unwrap()
        .volume
        .to_f64()
    };
    let before = volume(&editor);

    for (from, to) in [
        ("width", "plate_width"),
        ("screw", "bolt"),
        ("thickness", "t"),
    ] {
        let update = editor.handle(Command::RenameParameter {
            from: from.into(),
            to: to.into(),
        });
        assert!(update.error.is_none(), "{from} -> {to}: {:?}", update.error);
        let program = update.program.expect("the program is sent");
        assert!(
            program.steps.iter().all(|s| s.error.is_none()),
            "{:?}",
            program.steps
        );
        assert!(
            program.parameters.errors.is_empty(),
            "{:?}",
            program.parameters.errors
        );
        assert!(
            !program.parameter_uses.contains_key(from),
            "{from} is still read"
        );
    }
    let program = editor.program();
    let json = program.to_json().unwrap();
    for (old, new) in [
        ("\"width / 2\"", "\"plate_width / 2\""),
        ("\"screw.clearance\"", "\"bolt.clearance\""),
        ("{\"blind\":\"thickness\"}", "{\"blind\":\"t\"}"),
    ] {
        let compact: String = json.split_whitespace().collect();
        assert!(
            !compact.contains(&old.replace(' ', "")),
            "{old} is left in {json}"
        );
        assert!(
            compact.contains(&new.replace(' ', "")),
            "{new} is not in {json}"
        );
    }
    assert_eq!(volume(&editor), before);

    let taken = editor.handle(Command::RenameParameter {
        from: "depth".into(),
        to: "t".into(),
    });
    let error = taken.error.expect("a name taken is refused");
    assert!(
        error.contains(r#"there is a parameter "t" already"#),
        "{error}"
    );
    editor.handle(Command::Open { id: "plate".into() });
    let editing = editor.handle(Command::RenameParameter {
        from: "depth".into(),
        to: "plate_depth".into(),
    });
    assert!(editing.error.is_some(), "renamed while a step is edited");
    editor.handle(Command::Cancel);

    editor.handle(Command::Undo);
    assert!(editor.program().parameters.get("thickness").is_some());
    assert!(editor.program().parameters.get("t").is_none());
}

/// The bolted plate's drawing with its bill of materials, as the front end
/// makes it: a new drawing step, "Bill of materials" ticked, committed and
/// exported. The sheet draws the parts placed, where they are placed, and
/// lists them: the plate, and the standard screw and nut by the titles and
/// designations their own programs carry — which name their products in
/// a STEP file too — each line ballooned once with its item number.
#[test]
fn an_assembly_drawing_lists_its_parts() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "drawing".into(),
    });
    let step = update.step.expect("a drawing is edited");
    assert!(matches!(
        step.presentation.dialog.get("bom"),
        Some(Control::Checkbox { value: false, .. })
    ));
    let ticked = editor.handle(dialog("bom", Value::Bool(true)));
    // The sheet shown is the one downloaded: its parts list, given by the
    // editor, and the balloons pointing at the parts.
    let shown: Vec<String> = ticked
        .step
        .expect("a drawing is edited")
        .presentation
        .visuals
        .into_iter()
        .filter_map(|v| match v.shape {
            geop_ops::ui::Shape::Label { text, .. } => Some(text),
            _ => None,
        })
        .collect();
    for text in ["ITEM", "ISO 4762 M4x12", "ISO 4032 M4"] {
        assert!(
            shown.iter().any(|t| t == text),
            "{text} is not shown: {shown:?}"
        );
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let exported = editor.handle(Command::ExportDrawing {
        id: None,
        format: geop_ops_drawing::Format::Svg,
        date: "2026-10-04".into(),
    });
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let svg = exported.export.unwrap().text().unwrap().to_string();
    for text in [
        ">ITEM<",
        ">DESIGNATION<",
        ">ISO 4762 M4x12<",
        ">ISO 4032 M4<",
        ">plate<",
        ">Steel<",
    ] {
        assert!(svg.contains(text), "{text} is not on the sheet");
    }
    // The four default views draw the parts, with their hidden lines; three
    // balloons, each with its item number, stand on the dimension layer.
    let layer = |name: &str| {
        let start = svg.find(&format!("<g class=\"{name}\"")).unwrap();
        let end = start + svg[start..].find("</g>").unwrap();
        svg[start..end].to_string()
    };
    assert!(layer("VISIBLE").contains("<line"), "no view line");
    assert!(layer("HIDDEN").contains("<line"), "no hidden line");
    let dimensions = layer("DIMENSIONS");
    assert_eq!(dimensions.matches("r=\"4\"/>").count(), 3, "{dimensions}");
    for item in ["1", "2", "3"] {
        assert!(
            dimensions.contains(&format!(">{item}</text>")),
            "balloon {item}"
        );
    }
    // A STEP file names them so too: one product per size.
    let step = editor.handle(Command::ExportStep);
    let text = step
        .export
        .expect("a STEP file")
        .text()
        .unwrap()
        .to_string();
    assert!(
        text.contains("PRODUCT('ISO 4762 M4x12'"),
        "the screw's product"
    );
    assert!(text.contains("PRODUCT('plate'"), "the plate's product");
}

/// A new linear pattern picks the hole cut through a plate as a feature by
/// a click on the hole's wall, through the hole's mouth from above: the
/// dialog lists the feature and no body any more, hovering lights the
/// hole, and committed with four instances two apart, the scene holds the
/// plate alone, drilled four times.
#[test]
fn new_linear_pattern_picks_a_feature_by_its_wall() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program: super::feature_pattern_tests::plate_with_hole(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "linear_pattern".into(),
    });
    editor.handle(dialog("features", Value::Press));
    // From above the hole's middle down onto its wall at 45°, 0.1 above
    // the plate's bottom — clear of the rim and of the wall's seams.
    let r = 0.3 * std::f64::consts::FRAC_1_SQRT_2;
    let wall = pointer([1.0, 1.0, 3.0], [r, r, 0.1 - 3.0]);
    let hole = EntityRef::Feature {
        name: "hole".into(),
    };
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: wall,
            shift: false,
        },
    });
    let step = update.step.expect("the pattern is edited");
    assert!(
        step.presentation.highlights.contains(&hole),
        "{:?}",
        step.presentation.highlights
    );
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Click {
            pointer: wall,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let picked = |key: &str| match step.presentation.dialog.get(key) {
        Some(Control::Reference(r)) => r.entities().cloned().collect::<Vec<_>>(),
        other => panic!("{key}: {other:?}"),
    };
    assert_eq!(picked("features"), std::slice::from_ref(&hole));
    assert!(picked("bodies").is_empty());
    // A feature combines as it did: there is no combining to choose.
    assert!(step.presentation.dialog.get("combine").is_none());

    editor.handle(dialog("count", Value::Number(4.0)));
    editor.handle(dialog("spacing", Value::Number(2.0)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::LinearPattern(args) => {
            assert_eq!(args.features, [hole]);
            assert!(args.bodies.is_empty());
        }
        other => panic!("{other:?}"),
    }
    let scene = editor
        .handle(Command::Seek { marker: None })
        .scene
        .expect("the scene is sent");
    let id = editor.program().steps.last().unwrap().id.clone();
    assert_eq!(scene.part.solids, [format!("linear_pattern({id})")]);
    let copied = scene
        .part
        .faces
        .iter()
        .filter(|f| f.feature.as_deref() == Some(id.as_str()))
        .count();
    assert!(copied >= 3, "the copies' faces are the pattern's: {copied}");
}

/// What the step being edited shows after `event`, as the front end sends
/// it.
fn event(editor: &mut Editor<S>, event: StepEditEvent<S>) -> crate::editor::StepState<S> {
    let update = editor.handle(Command::Event { event });
    assert!(update.error.is_none(), "{:?}", update.error);
    update.step.expect("a step is edited")
}

/// Hovers `at`, which must be over `part` of the gizmo — a press there
/// grabs — and drags it to `to`: once on the way, half way, saying what
/// the drag has done so far, then released there.
fn drag_gizmo(
    editor: &mut Editor<S>,
    part: geop_ops::ui::GizmoPart,
    at: Pointer<S>,
    to: Pointer<S>,
    halfway: Pointer<S>,
    shift: bool,
) -> crate::editor::StepState<S> {
    let step = event(
        editor,
        StepEditEvent::Hover {
            pointer: at,
            shift: false,
        },
    );
    let gizmo = step.presentation.gizmo.expect("a gizmo");
    assert_eq!(gizmo.hover, Some(part), "{gizmo:?}");
    assert!(step.presentation.grab, "a press on the gizmo grabs");
    let step = event(
        editor,
        StepEditEvent::Drag {
            from: at,
            to: halfway,
            done: false,
            shift,
        },
    );
    let gizmo = step.presentation.gizmo.expect("a gizmo while dragged");
    assert_eq!(gizmo.active, Some(part));
    assert!(gizmo.readout.is_some(), "{gizmo:?}");
    event(
        editor,
        StepEditEvent::Drag {
            from: at,
            to,
            done: true,
            shift,
        },
    )
}

/// The subd cage's top face, selected, moved up by its gizmo's `z` arrow,
/// turned by its ring about `z` and stretched along `x` by the cube beyond
/// the `x` arrow — about the face's centre, each snapped: to the grid, to
/// 15 degrees, to a tenth.
#[test]
fn subd_faces_are_moved_turned_and_scaled_by_the_gizmo() {
    use geop_ops::ui::GizmoPart;

    let mut editor = Editor::<S>::new();
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    let step = event(
        &mut editor,
        StepEditEvent::Click {
            pointer: pointer([0.3, 0.2, 10.0], [0.0, 0.0, -1.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    );
    let gizmo = step.presentation.gizmo.expect("the selection has a gizmo");
    assert!(gizmo.modes.translate && gizmo.modes.rotate && gizmo.modes.scale);
    assert_eq!(
        [0, 1, 2].map(|k| gizmo.at[k].to_f64()),
        [0.0, 0.0, 1.0],
        "at the face's centre"
    );
    // A face has an orientation of its own; the world's is chosen.
    assert!(
        step.presentation
            .dialog
            .get(geop_ops::ui::GIZMO_ORIENTATION)
            .is_some()
    );
    editor.handle(dialog(
        geop_ops::ui::GIZMO_ORIENTATION,
        Value::Choice("world".into()),
    ));
    // No handles any more: the gizmo is what drags.
    assert!(
        !step
            .presentation
            .visuals
            .iter()
            .any(|v| matches!(v.shape, geop_ops::ui::Shape::Handle { .. }))
    );

    // Seen from the side, the `z` arrow pulled up by 0.31: a reach of
    // 0.009 snaps to fiftieths.
    let side = |z: f64| pointer([0.0, -10.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(2),
        side(1.05),
        side(1.355),
        side(1.2),
        false,
    );
    // From above, the ring about `z` turned from 45 to 90 degrees round.
    let reach = 0.009;
    let round = |degrees: f64| {
        let (s, c) = degrees.to_radians().sin_cos();
        let r = 7.0 * reach;
        pointer([r * c, r * s, 10.0], [0.0, 0.0, -1.0])
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(2),
        round(45.0),
        round(92.0),
        round(60.0),
        false,
    );
    // The cube beyond the `x` arrow pulled out to twice as far.
    let along = |x: f64| pointer([x, 0.0, 10.0], [0.0, 0.0, -1.0]);
    let step = drag_gizmo(
        &mut editor,
        GizmoPart::Stretch(0),
        along(12.5 * reach),
        along(25.0 * reach),
        along(20.0 * reach),
        false,
    );
    assert_eq!(step.error, None);

    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let last = editor.program().steps.last().unwrap().clone();
    let PartOperation::Subd(args) = &last.operation else {
        panic!("{:?}", last.operation);
    };
    // The top's corners `(±1, ±1)`, turned by 45 degrees, lie on the axes
    // √2 out; stretched along `x`, twice that along it.
    let r = 2f64.sqrt();
    let mut corners: Vec<[f64; 3]> = args
        .cage
        .face(9)
        .unwrap()
        .vertices
        .iter()
        .map(|&v| args.cage.vertex(v).unwrap().at)
        .collect();
    corners.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    let want = [
        [-2.0 * r, 0.0, 1.3],
        [0.0, -r, 1.3],
        [0.0, r, 1.3],
        [2.0 * r, 0.0, 1.3],
    ];
    for (got, want) in corners.iter().zip(want) {
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-9),
            "{corners:?}"
        );
    }
}

/// A point of a 3-D sketch, selected, is moved by its gizmo along `x`
/// only: the `x` arrow dragged up and off to the side moves it along
/// `x`, as far as the pointer went along it.
#[test]
fn a_3d_sketch_point_is_dragged_along_an_axis() {
    use geop_ops::ui::GizmoPart;

    let mut editor = Editor::<S>::new();
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let click = |editor: &mut Editor<S>, origin: [f64; 3], dir: [f64; 3]| {
        event(
            editor,
            StepEditEvent::Click {
                pointer: pointer(origin, dir),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        )
    };
    // A line from the origin to (2, 0, 2), seen from the front.
    click(&mut editor, [0.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    click(&mut editor, [2.0, -10.0, 2.0], [0.0, 1.0, 0.0]);
    let key = |editor: &mut Editor<S>| {
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
    let front = |x: f64, z: f64| pointer([x, -10.0, z], [0.0, 1.0, 0.0]);
    let step = drag_gizmo(
        &mut editor,
        GizmoPart::Move(0),
        front(2.05, 2.0),
        front(2.75, 2.4),
        front(2.4, 2.2),
        false,
    );
    assert_eq!(step.error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::AddSketch3d(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a 3-D sketch");
    };
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

/// Move body shifts the body by a gizmo where its middle goes: its `y`
/// arrow dragged, then the square of the `x`–`z` plane.
#[test]
fn a_body_is_moved_by_its_gizmo() {
    use geop_ops::ui::GizmoPart;

    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "move_body".into(),
    });
    let step = update.step.expect("move body is edited");
    let gizmo = step.presentation.gizmo.expect("a gizmo");
    assert!(gizmo.modes.translate && !gizmo.modes.rotate);
    let at = [0, 1, 2].map(|k| gizmo.at[k].to_f64());
    // Looking down `x`, the `y` arrow dragged 0.3 along.
    let side = |y: f64, z: f64| pointer([at[0] - 10.0, y, z], [1.0, 0.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(1),
        side(at[1] + 0.05, at[2]),
        side(at[1] + 0.35, at[2]),
        side(at[1] + 0.2, at[2]),
        false,
    );
    // Looking down `y` at the moved gizmo, its `x`–`z` square — the one
    // normal to `y` — dragged by (0.2, -0.4).
    let at = [at[0], at[1] + 0.3, at[2]];
    let front = |x: f64, z: f64| pointer([x, at[1] - 10.0, z], [0.0, 1.0, 0.0]);
    let corner = 0.03;
    drag_gizmo(
        &mut editor,
        GizmoPart::Plane(1),
        front(at[0] + corner, at[2] + corner),
        front(at[0] + corner + 0.2, at[2] + corner - 0.4),
        front(at[0] + corner + 0.1, at[2] + corner),
        false,
    );
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::MoveBody(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a move");
    };
    let want = [0.2, 0.3, -0.4];
    assert!(
        (0..3).all(|k| (args.translation[k] - want[k]).abs() < 1e-9),
        "{:?}",
        args.translation
    );
}

/// Placed parts are moved by a gizmo at their origin: a fixed rail turned
/// a quarter about `z` by its ring, and a carriage on it slid along the
/// rail by its `z` arrow — the slider joint solved as it is dragged.
#[test]
fn placed_parts_are_moved_and_turned_by_the_gizmo() {
    use geop_ops::part::{ParamValue, pose_parameter};
    use geop_ops::ui::GizmoPart;

    let (rail, carriage) = ("std:linear_rail.geop", "std:linear_carriage.geop");
    let mut editor = Editor::<S>::new();
    editor.handle(Command::Load {
        program: Program::new(),
        path: Some("guide.geop".into()),
    });
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice(rail.into())));
    editor.handle(dialog("fixed", Value::Bool(true)));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9".into())));
    assert!(editor.handle(Command::Commit).error.is_none());
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice(carriage.into())));
    editor.handle(dialog("parameter:size", Value::Choice("MGN9H".into())));
    editor.handle(dialog("add_mate", Value::Choice("slider".into())));
    editor.handle(dialog(
        "mate:m1:entities",
        Value::Entities(vec![
            EntityRef::datum("part1/axis"),
            EntityRef::datum("part2/axis"),
        ]),
    ));
    assert!(editor.handle(Command::Commit).error.is_none());
    let pose = |editor: &Editor<S>, id: &str| match editor.program().state.get(&pose_parameter(id))
    {
        Some(ParamValue::Pose(pose)) => *pose,
        other => panic!("{id}: {other:?}"),
    };
    let apply = |pose: &geop_core_math::primitives::Pose<geop_ops::Design>, p: [f64; 3]| {
        let at = pose.apply(&Vector3::from_array(p.map(geop_ops::Design::from_f64)));
        [0, 1, 2].map(|k| at[k].to_f64())
    };
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);

    // The carriage, slid 10 along the rail by its `z` arrow, seen from the
    // side.
    let start = apply(&pose(&editor, "part2"), [0.0; 3]);
    let update = editor.handle(Command::Open { id: "part2".into() });
    let step = update.step.expect("the carriage is edited");
    let gizmo = step.presentation.gizmo.expect("a placed part has a gizmo");
    assert!(gizmo.modes.translate && gizmo.modes.rotate);
    assert!(
        step.presentation
            .dialog
            .get(geop_ops::ui::GIZMO_ORIENTATION)
            .is_some()
    );
    let side = |z: f64| pointer([start[0], start[1] - 100.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(2),
        side(start[2] + 0.05),
        side(start[2] + 10.055),
        side(start[2] + 5.0),
        false,
    );
    assert!(editor.handle(Command::Commit).error.is_none());
    let slid = apply(&pose(&editor, "part2"), [0.0; 3]);
    assert!(
        close(slid, [start[0], start[1], start[2] + 10.0]),
        "{start:?} slid to {slid:?}"
    );

    // The rail, fixed, turned a quarter about `z` by its ring, seen from
    // above: it goes where it is turned, and the carriage turns with it.
    editor.handle(Command::Open { id: "part1".into() });
    let reach = 0.009;
    let round = |degrees: f64| {
        let (s, c) = degrees.to_radians().sin_cos();
        let r = 7.0 * reach;
        pointer([r * c, r * s, 100.0], [0.0, 0.0, -1.0])
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(2),
        round(45.0),
        round(137.0),
        round(90.0),
        false,
    );
    assert!(editor.handle(Command::Commit).error.is_none());
    let turned = pose(&editor, "part1");
    assert!(
        close(apply(&turned, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]),
        "{turned:?}"
    );
    assert!(editor.part().check_mates(|_| true).unwrap().converged);
}

/// The toolbar's sections are the editor's: it sends the operations group
/// by group, in the groups' order, every operation in one, and within a
/// group the few used most (shown big) first, then the common ones (small),
/// then those only in the group's menu.
#[test]
fn operations_are_offered_by_group() {
    use geop_ops::{OperationTier, Operations};
    let (_, update) = editor();
    let operations = update.program.expect("the program is sent").operations;
    assert_eq!(operations.len(), PartOperation::infos().len());
    let order: Vec<_> = operations.iter().map(|o| (o.group, o.tier)).collect();
    assert!(
        order.is_sorted(),
        "not group by group, the most used first: {order:?}"
    );
    let big: Vec<&str> = operations
        .iter()
        .filter(|o| o.tier == OperationTier::Big)
        .map(|o| o.kind)
        .collect();
    assert_eq!(
        big,
        [
            "add_sketch",
            "extrude",
            "revolve",
            "hole",
            "fillet",
            "boolean",
            "linear_pattern",
            "subd",
            "base_flange",
            "add_part",
            "drawing",
        ]
    );
    let json = serde_json::to_value(&operations[0]).unwrap();
    assert_eq!(json["group"], "Sketch");
    assert_eq!(json["tier"], "Big");
    let hem = operations.iter().find(|o| o.kind == "hem").unwrap();
    assert_eq!(serde_json::to_value(hem).unwrap()["group"], "Sheet metal");
    let mirror = operations.iter().find(|o| o.kind == "mirror").unwrap();
    assert_eq!(
        serde_json::to_value(mirror).unwrap()["group"],
        "Pattern & bodies"
    );
}

/// A click of the primary button with `pointer`.
fn click_at(pointer: Pointer<S>) -> Command<S> {
    Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    }
}

/// Looking straight down at `(x, y)`.
fn from_above(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// The 3-D sketch `route`: lines from `(1, 1, 1)` to `(3, 1, 1)` and on to
/// `(3, 2, 2)` — its points `p0`, `p1` and `p3`, its lines `c2` and `c4`.
fn route() -> geop_ops_sketch3d::AddSketch3dArgs {
    let mut s = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(examples::n));
    let a = s.add_point(at([1.0, 1.0, 1.0]));
    let b = s.add_point(at([3.0, 1.0, 1.0]));
    s.add_line(a, b);
    let c = s.add_point(at([3.0, 2.0, 2.0]));
    s.add_line(b, c);
    geop_ops_sketch3d::AddSketch3dArgs {
        sketch: s,
        references: Vec::new(),
    }
}

/// An editor on `program`, which builds.
fn editing(program: Program) -> Editor<S> {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::Load {
        program,
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// A 3-D sketch drawn by clicks from the side: the second point, clicked
/// near the line along `x` through the first, snaps onto it; the third,
/// near the line along `z` through the second, onto that; the fourth, near
/// the first point, is the first point. Committed, the sketch is one loop
/// with two lines parallel to the axes they snapped to.
#[test]
fn a_3d_sketch_snaps_along_axes_and_onto_points() {
    let mut editor = editing(Program::new());
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let side = |x: f64, z: f64| pointer([x, -10.0, z], [0.0, 1.0, 0.0]);
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
            geop_ops::ui::Shape::Label { text, .. } if text == "x"
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
    let PartOperation::AddSketch3d(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a 3-D sketch");
    };
    let sketch = &args.sketch;
    assert_eq!(sketch.points.len(), 3);
    let parallel: Vec<usize> = sketch
        .constraints
        .values()
        .filter_map(|c| match c {
            geop_core_sketch::space::Constraint3d::ParallelTo { direction, .. } => {
                (0..3).find(|&k| direction[k].to_f64() == 1.0)
            }
            _ => None,
        })
        .collect();
    assert_eq!(parallel, [0, 2]);
    let chains = sketch.chains().unwrap();
    assert!(chains.len() == 1 && chains[0].closed, "{chains:?}");
}

/// What a 3-D sketch builds is picked like any edge or vertex — before
/// they were edges and vertices, nothing of a 3-D sketch could be picked
/// but the whole sketch, as a path. A datum's selection takes the sketch's
/// corner, lit when hovered, and then its line; the datum builds on them.
#[test]
fn a_3d_sketch_point_and_line_are_picked() {
    let mut program = Program::new();
    program.push("route", route());
    let mut editor = editing(program);
    editor.handle(Command::New {
        kind: "add_datum".into(),
    });
    let corner = EntityRef::Vertex {
        name: "sketch3d(route,p1)".into(),
    };
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(3.004, 1.0),
            shift: false,
        },
    });
    let highlights = update.step.unwrap().presentation.highlights;
    assert!(highlights.contains(&corner), "{highlights:?}");
    editor.handle(click_at(from_above(3.004, 1.0)));
    editor.handle(click_at(from_above(2.0, 1.005)));
    editor.handle(dialog("construction", Value::Choice("parallel".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::AddDatum(datum) = &editor.program().steps.last().unwrap().operation else {
        panic!("a datum");
    };
    assert_eq!(
        datum.selection,
        [
            corner,
            EntityRef::Edge {
                name: "sketch3d(route,c2)".into()
            }
        ]
    );
    let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    part.check_names().unwrap();
}

/// A boundary surface between a planar sketch's line and a 3-D sketch's
/// line, both clicked: the ruled face between them, a valid sheet.
#[test]
fn a_boundary_surface_spans_sketch_lines() {
    let mut base = geop_ops_sketch::Sketch::new();
    let (a, b) = (
        base.add_point(examples::n(0.0), examples::n(0.0)),
        base.add_point(examples::n(2.0), examples::n(0.0)),
    );
    let line = base.add_line(a, b);
    let mut program = Program::new();
    program.push(
        "base",
        geop_ops_sketch::AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: base,
            ..Default::default()
        },
    );
    program.push("route", route());
    let mut editor = editing(program);
    editor.handle(Command::New {
        kind: "boundary_surface".into(),
    });
    editor.handle(click_at(from_above(1.0, 0.004)));
    editor.handle(click_at(from_above(2.0, 1.004)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::BoundarySurface(args) = &editor.program().steps.last().unwrap().operation
    else {
        panic!("a boundary surface");
    };
    assert_eq!(
        args.edges,
        [
            EntityRef::SketchCurve {
                sketch: "base".into(),
                curve: line,
            },
            EntityRef::Edge {
                name: "sketch3d(route,c2)".into()
            }
        ]
    );
    let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    assert!(part.face_id("boundary(boundary_surface1)").is_ok());
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// The horn swept with a 3-D sketch as its rail, clicked where it runs:
/// the rail is the whole sketch, and the horn builds as from the planar
/// one.
#[test]
fn a_sweep_takes_a_3d_sketch_as_its_rail() {
    let mut program = examples::horn();
    program.steps.truncate(2);
    let mut flare = geop_ops_sketch3d::Sketch3d::new();
    let points = [
        [0.0, 0.5, 0.0],
        [1.5, 0.4, 0.0],
        [3.0, 0.9, 0.0],
        [4.0, 1.6, 0.0],
    ]
    .map(|p| flare.add_point(Vector3::from_array(p.map(examples::n))));
    flare.add_spline(points.to_vec());
    program.push(
        "flare",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: flare,
            references: Vec::new(),
        },
    );
    let mut editor = editing(program);
    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    let sketch = |name: &str| Value::Entities(vec![EntityRef::Sketch { name: name.into() }]);
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    editor.handle(dialog("rails", Value::Press));
    editor.handle(click_at(from_above(1.5, 0.4)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::Sweep(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a sweep");
    };
    assert_eq!(args.rails, [EntityRef::sketch3d("flare")]);
    let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// Two lines of a 3-D sketch, `(x, y, z) = (s, 0.5, 0)` and `(s, 1.5, 1)`
/// for `s` from -0.5 to 2.5 — or, with `across`, lines
/// `(0, s, s - 0.5)` and `(2, s, s - 0.5)` crossing them, run downwards.
fn network_lines(across: bool) -> geop_ops_sketch3d::AddSketch3dArgs {
    let mut s = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(examples::n));
    let lines: [[[f64; 3]; 2]; 2] = if across {
        [
            [[0.0, 2.5, 2.0], [0.0, -0.5, -1.0]],
            [[2.0, 2.5, 2.0], [2.0, -0.5, -1.0]],
        ]
    } else {
        [
            [[-0.5, 0.5, 0.0], [2.5, 0.5, 0.0]],
            [[-0.5, 1.5, 1.0], [2.5, 1.5, 1.0]],
        ]
    };
    for [a, b] in lines {
        let (a, b) = (s.add_point(at(a)), s.add_point(at(b)));
        s.add_line(a, b);
    }
    geop_ops_sketch3d::AddSketch3dArgs {
        sketch: s,
        references: Vec::new(),
    }
}

/// A UV surface through two 3-D sketches' lines, crossing each other: the
/// u curves clicked first, then the v field pressed and the v curves
/// clicked. The sheet spans the grid between them, a valid face.
#[test]
fn a_uv_surface_picks_its_curves_by_clicks() {
    let mut program = Program::new();
    program.push("along", network_lines(false));
    program.push("across", network_lines(true));
    let mut editor = editing(program);
    let update = editor.handle(Command::New {
        kind: "network_surface".into(),
    });
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Curve]);
    for (x, y) in [(1.0, 1.504), (1.0, 0.504)] {
        let update = editor.handle(click_at(from_above(x, y)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    editor.handle(dialog("v_curves", Value::Press));
    for (x, y) in [(2.004, 1.0), (0.004, 1.0)] {
        let update = editor.handle(click_at(from_above(x, y)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let PartOperation::NetworkSurface(args) = &editor.program().steps.last().unwrap().operation
    else {
        panic!("a UV surface");
    };
    let edge = |name: &str| EntityRef::Edge { name: name.into() };
    assert_eq!(
        args.u_curves,
        [edge("sketch3d(along,c5)"), edge("sketch3d(along,c2)")]
    );
    assert_eq!(
        args.v_curves,
        [edge("sketch3d(across,c5)"), edge("sketch3d(across,c2)")]
    );
    let scene = update.scene.expect("the scene is sent anew");
    let faces: Vec<&str> = scene.part.faces.iter().map(|f| f.name.as_str()).collect();
    assert!(faces.contains(&"network(uv_surface1)"), "{faces:?}");
    let part = editor.program().build::<S>(&geop_ops::NoFiles).unwrap();
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
    // Its corner where the first u curve picked meets the first v curve.
    let corner = part
        .vertex_id("network(uv_surface1,sketch3d(along,c5),sketch3d(across,c5))")
        .unwrap();
    let p = part.topology().get_vertex(corner).unwrap().point;
    assert!(p.could_be_equal(&Vector3::from_array([2.0, 1.5, 1.0].map(S::from_f64))));
}

/// The default view of an empty part, as the browser sends it — 1920 x
/// 1080, the camera at `(66, 44, 88)` looking at the origin: a line's
/// first click on the origin, its second 160 px right of and 90 px above
/// it. Both place a point, and the line between them is drawn. (Reported
/// as placing nothing: in a 1400 x 900 window that spot lies under the
/// step's dialog, and the viewer sends no click there at all.)
#[test]
fn a_3d_line_is_drawn_from_the_origin_in_the_default_view() {
    let mut editor = editing(Program::new());
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let eye = |dir: [f64; 3]| {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
        Pointer {
            ray: Ray::try_new(
                v([65.90889047678681, 43.93926031785788, 87.87852063571576]),
                v(dir),
            )
            .unwrap(),
            reach: Reach::Cone {
                slope: S::from_f64(0.00838515269409588),
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
    let PartOperation::AddSketch3d(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a 3-D sketch");
    };
    assert_eq!(args.sketch.curves.len(), 1, "{:?}", args.sketch);
}

/// Editing the first sketch of a program builds only up to it, however
/// many steps follow: a point of the handle's outline dragged rebuilds the
/// sketch, not the sweep, hole and boolean after it, on every move of the
/// pointer. Put away, the whole program is built again.
#[test]
fn editing_an_early_sketch_builds_only_up_to_it() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadExample {
        name: "handle_with_hole".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let steps = editor.program().steps.len();
    assert!(steps >= 3, "{steps}");
    let update = editor.handle(Command::Open {
        id: "outline".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let crate::PartOperation::AddSketch(args) = editor.editing().unwrap().clone() else {
        panic!("a sketch")
    };
    let corner = args
        .sketch
        .points
        .values()
        .find(|p| !p.fixed)
        .expect("a point of the outline")
        .xy();
    let corner = [corner[0].to_f64(), corner[1].to_f64()];
    let down = |[x, y]: [f64; 2]| pointer([x, y, 10.0], [0.0, 0.0, -1.0]);
    editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down(corner),
            shift: false,
        },
    });
    for (k, d) in [0.05, 0.1, 0.15].into_iter().enumerate() {
        let before = editor.runner.steps_built();
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: down(corner),
                to: down([corner[0] + d, corner[1] + d]),
                done: k == 2,
                shift: true,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
        // Counted, not timed: the sketch at most, never what follows it.
        let built = editor.runner.steps_built() - before;
        assert!(built <= 1, "a drag in the first sketch built {built} steps");
        assert_eq!(editor.runner.results().len(), 1);
    }
    let crate::PartOperation::AddSketch(dragged) = editor.editing().unwrap() else {
        panic!("a sketch")
    };
    assert_ne!(dragged.sketch, args.sketch, "the drag changed nothing");
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(editor.runner.results().len(), steps);
    assert!(editor.runner.results().iter().all(|r| r.error.is_none()));
}

/// A file left and opened again takes up its steps as they were built:
/// the drilled box, then an assembly placing it, then the box again,
/// builds the box's steps once, not twice. Edited in between, it is built
/// again from the step that changed.
#[test]
fn a_file_opened_again_is_not_built_again() {
    let part = examples::box_with_drill_hole();
    let mut assembly = Program::new();
    assembly.push(
        "box",
        geop_ops_assembly::AddPartArgs {
            file: "part.geop".into(),
            fixed: true,
            ..Default::default()
        },
    );
    let mut editor = Editor::<S>::new();
    let files = [
        ("part.geop".to_string(), Some(part.to_json().unwrap())),
        (
            "assembly.geop".to_string(),
            Some(assembly.to_json().unwrap()),
        ),
    ]
    .into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let load = |editor: &mut Editor<S>, program: &Program, path: &str| {
        let update = editor.handle(Command::Load {
            program: program.clone(),
            path: Some(path.into()),
        });
        assert!(update.error.is_none(), "{:?}", update.error);
        assert!(
            editor.runner.results().iter().all(|r| r.error.is_none()),
            "{:?}",
            editor.runner.results()
        );
    };
    load(&mut editor, &part, "part.geop");
    let built = editor.runner.part().revision();
    load(&mut editor, &assembly, "assembly.geop");
    load(&mut editor, &part, "part.geop");
    assert_eq!(
        editor.runner.part().revision(),
        built,
        "the box opened again was built again"
    );

    // Its hole drilled deeper: from that step on.
    let mut deeper = part.clone();
    let last = deeper.steps.len() - 1;
    let PartOperation::Extrude(hole) = &mut deeper.steps[last].operation else {
        panic!("the hole is an extrusion")
    };
    hole.extent = geop_ops_extrude_revolve::Extents::blind(-0.8);
    assert_ne!(deeper.steps[last], part.steps[last], "the hole changed");
    let before = editor.runner.steps_built();
    load(&mut editor, &deeper, "part.geop");
    assert_eq!(editor.runner.steps_built() - before, 1);
}

/// A sketch is copied as the front end copies it — the update's export —
/// and pasted where the program runs to, under a fresh id, building as the
/// original does; pasted into a program without one of its id, it keeps
/// its own.
#[test]
fn a_sketch_is_copied_and_pasted() {
    let (mut editor, _) = editor();
    let sketch = editor
        .program()
        .steps
        .iter()
        .find(|s| geop_ops::Operations::kind(&s.operation) == "add_sketch")
        .expect("the example has a sketch")
        .clone();

    let copied = editor.handle(Command::Copy {
        id: sketch.id.clone(),
    });
    assert!(copied.error.is_none(), "{:?}", copied.error);
    let text = copied.export.unwrap().text().unwrap().to_string();

    editor.handle(Command::Seek { marker: Some(1) });
    let update = editor.handle(Command::Paste { text: text.clone() });
    assert!(update.error.is_none(), "{:?}", update.error);
    let state = update.program.unwrap();
    assert_eq!(state.marker, 2, "the program runs to the pasted step");
    let pasted = &editor.program().steps[1];
    assert_ne!(pasted.id, sketch.id);
    assert_eq!(pasted.operation, sketch.operation);
    editor.handle(Command::Seek { marker: None });
    let update = editor.handle(Command::Show);
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");

    // Another file: its own id is free there.
    editor.handle(Command::Load {
        program: Program::new(),
        path: Some("other.geop".into()),
    });
    let update = editor.handle(Command::Paste { text });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(editor.program().steps[0].id, sketch.id);

    let refused = editor.handle(Command::Paste {
        text: "not steps".into(),
    });
    assert!(refused.error.unwrap().contains("pasting"));
}
