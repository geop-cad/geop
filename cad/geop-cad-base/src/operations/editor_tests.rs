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
    match &editor.handle(Command::Commit).error {
        Some(e) => panic!("{e}"),
        None => {}
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

/// A new linear pattern starts on the newest solid, along `x`: its
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
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Sketch]);
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
    assert_eq!(update.step.unwrap().presentation.pickable, [Role::Edge]);
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
        (sweep.profile.as_str(), sweep.path.as_str()),
        ("section", route.id.as_str())
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
/// Extrude pulls it out; dragged by its `z` handle and then by the face
/// itself, it moves; and the limit body the editor shows is the solid the
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

    // The `z` handle, seen from the side, pulled up by a quarter.
    let at = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            geop_ops::ui::Shape::Handle { at, .. } if v.key == "z" => Some(at),
            _ => None,
        })
        .expect("z has a handle");
    let [x, y, z] = [0, 1, 2].map(|k| at[k].to_f64());
    let side = |dz: f64| pointer([x, y - 10.0, z + dz], [0.0, 1.0, 0.0]);
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
            shift: false,
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
            pointer: from_above.clone(),
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

/// A drawing step, the way the front end makes one: started, its views
/// chosen, a distance dimensioned by clicking two corners of the box, the
/// title block's name typed, committed — then exported as SVG and DXF.
#[test]
fn a_drawing_is_made_and_exported() {
    let (mut editor, _) = editor();
    let started = editor.handle(Command::New {
        kind: "drawing".into(),
    });
    assert!(started.error.is_none(), "{:?}", started.error);
    editor.handle(dialog("view:iso", Value::Bool(false)));
    editor.handle(dialog("title:name", Value::Text("Drilled box".into())));
    // The box's two top right corners, from above.
    editor.handle(dialog("distance", Value::Press));
    let above = |x: f64, y: f64| pointer([x, y, 10.0], [0.0, 0.0, -1.0]);
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let first = editor.handle(click(above(2.0, 2.0))).step.unwrap();
    let Some(Control::Reference(picked)) = first.presentation.dialog.get("distance") else {
        panic!("the distance field");
    };
    assert_eq!(picked.value.len(), 1, "{picked:?}");
    let update = editor.handle(click(above(2.0, 0.0)));
    let step = update.step.unwrap();
    let Some(Control::List { items, .. }) = step.presentation.dialog.get("dimensions") else {
        panic!("the dimensions are listed");
    };
    assert_eq!(items.len(), 1, "{items:?}");
    assert!(items[0].label.starts_with("Distance"), "{}", items[0].label);
    let committed = editor.handle(Command::Commit);
    assert!(committed.error.is_none(), "{:?}", committed.error);
    let PartOperation::Drawing(args) = &editor.program().steps.last().unwrap().operation else {
        panic!("a drawing step");
    };
    assert_eq!(args.views.len(), 3);
    assert_eq!(args.name, "Drilled box");

    let exported = editor.handle(Command::ExportDrawing {
        id: None,
        format: geop_ops_drawing::Format::Svg,
        date: "2026-10-04".into(),
    });
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let file = exported.export.unwrap();
    assert_eq!(file.name, "drawing.svg");
    let svg = file.text().unwrap();
    assert!(svg.starts_with("<svg"));
    assert!(svg.contains("Drilled box") && svg.contains(">2<"));
    // The blind hole, seen from the front, is hidden.
    assert!(svg.contains(r#"<g class="HIDDEN""#));
    let hidden = &svg[svg.find(r#"<g class="HIDDEN""#).unwrap()..];
    assert!(hidden[..hidden.find("</g>").unwrap()].contains("<line"));

    let dxf = editor.handle(Command::ExportDrawing {
        id: Some("drawing1".into()),
        format: geop_ops_drawing::Format::Dxf,
        date: String::new(),
    });
    assert!(dxf.export.unwrap().text().unwrap().ends_with("EOF\n"));
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

/// "Download STEP" writes the part shown; dropped next to a new program,
/// the file is offered by a new "Import STEP" step, which brings its solid
/// back, named after the step.
#[test]
fn a_part_exported_as_step_is_imported_back() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::ExportStep);
    assert!(update.error.is_none(), "{:?}", update.error);
    let export = update.export.expect("a file is written");
    assert!(export.name.ends_with(".step"), "{}", export.name);
    assert!(export.text().unwrap().contains("MANIFOLD_SOLID_BREP"));

    let files = std::collections::BTreeMap::from([(
        "box.step".to_string(),
        Some(export.text().unwrap().to_string()),
    )]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Import STEP");
    let Some(Control::Select { options, .. }) = step.presentation.dialog.get("file") else {
        panic!("the file is chosen from a list");
    };
    assert!(options.iter().any(|o| o.value == "box.step"), "{options:?}");
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
