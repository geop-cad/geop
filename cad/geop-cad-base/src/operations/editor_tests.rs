//! The editor engine: a program edited by commands, and only what changed
//! sent back.

use geop_core_editor::output::{Bounded, Content, Inspector, InspectorInfo, Output as Made, Picks};
use geop_core_editor::view::drawn;
use geop_core_editor::{
    editing::{
        Button, Control, GizmoPart, Pointer, Reach, Shape, StepEditEvent, Style, Tone, Value,
        Visual,
    },
    selectable::Selectable,
};
use geop_core_editor::{registry::Registry, testing::editor_with};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{FrameAxis, Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{
    element::PickedElement,
    program::{Program, ProgramStep},
    values::{Design, Setting, StateValue, pose_state},
};
use geop_ops_assembly::MateEntity;
use geop_ops_datums::datum;
use geop_ops_extrude_revolve::{ExtrudeArgs, Output};
use geop_ops_model::Combine;
use geop_ops_model::{BooleanOp, Face, NamedModel, Vertex, element, read_model};
use geop_ops_parameters::parameters_of;
use geop_ops_reference_geometry::Selected;

use crate::{
    Command, Editor, Update,
    registry::testing::{
        Built, args, click_at, dialog, example_editor, from_above, handled, joints_panel, loaded,
        origin_axis, parameters_panel, pointer, rename_parameter, set_joint,
    },
};

/// The model the program builds as far as the editor runs it.
fn model(editor: &Editor) -> NamedModel<Design> {
    read_model(&editor.part()).unwrap().into_owned()
}

/// The face `name` of the model, as a pick names it.
fn face(name: &str) -> PickedElement {
    element::<Face>(name).picked()
}

/// The datum `name` of the part placed as `part`, as a pick names it.
fn placed_datum(part: &str, name: &str) -> PickedElement {
    MateEntity::of(part, Selected::Datum(datum(name))).picked()
}

/// The face `name` of the part placed as `part`, as a pick names it.
fn placed_face(part: &str, name: &str) -> PickedElement {
    MateEntity::of(part, Selected::Face(element(name))).picked()
}

/// `value` as it is saved: what arguments holding typed references are
/// compared by.
fn saved(value: impl serde::Serialize) -> serde_json::Value {
    serde_json::to_value(value).unwrap()
}

fn editor() -> (Editor, Update) {
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadExample {
            name: "box_with_drill_hole".into(),
        },
    );
    (editor, update)
}

fn distance(editor: &Editor, id: &str) -> f64 {
    let program = editor.program();
    let args: ExtrudeArgs = args(&program.steps[program.index_of(id).unwrap()]);
    match &args.extent.side1 {
        geop_ops_extrude_revolve::Extent::Blind(d) => d.plain().expect("a plain distance"),
        other => panic!("{other:?} is no distance"),
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
    let update = handled(&mut editor, Command::Commit);
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
    handled(&mut editor, Command::Commit);
    assert!((distance(&editor, "hole") - 0.25).abs() < 1e-9);
    let program = editor.program();
    let hole: ExtrudeArgs = args(&program.steps[program.index_of("hole").unwrap()]);
    assert!(matches!(
        hole.output,
        Output::Solid(Combine::With {
            op: BooleanOp::Union,
            ..
        })
    ));
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
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let extrude: ExtrudeArgs = args(program.steps.last().unwrap());
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
    assert_eq!(
        presentation.pickable,
        [drawn::FACE, drawn::DATUM, drawn::SKETCH]
    );
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
    assert_eq!(update.step.unwrap().presentation.pickable, [drawn::SKETCH]);
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
    let combine = |editor: &mut Editor| {
        editor.handle(Command::Commit);
        let program = editor.program();
        let extrude: ExtrudeArgs = args(program.steps.last().unwrap());
        let Output::Solid(combine) = extrude.output else {
            panic!("a solid");
        };
        editor.handle(Command::Undo);
        combine
    };
    editor.handle(dialog("combine", Value::Choice("difference".into())));
    editor.handle(dialog("distance", Value::Number(2.0)));
    assert!(matches!(
        combine(&mut editor),
        Combine::With {
            op: BooleanOp::Difference,
            ..
        }
    ));
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
    // Left apart from the box: joining the two is a boolean of its own.
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    // From below: the outline's square, under the box.
    let below = |x: f64, y: f64| pointer([x, y, -10.0], [0.0, 0.0, 1.0]);
    editor.handle(click_at(below(1.0, 0.5)));
    editor.handle(dialog("axis", Value::Press));
    // The origin's y axis, ten reaches of 0.009 long, from below.
    let update = editor.handle(click_at(below(0.0, 0.06)));
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Reference(axis)) = presentation.dialog.get("axis") else {
        panic!("the axis is picked");
    };
    assert_eq!(
        axis.elements().cloned().collect::<Vec<_>>(),
        [origin_axis(FrameAxis::Y).picked()]
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

/// Parameters given to a program without any, while a step is edited, put
/// their step first: the step edited, and the marker, stay at the steps they
/// were at, and the commit keeps both the parameters and the step's edit.
#[test]
fn the_first_parameter_given_while_a_step_is_edited() {
    use geop_core_program::operation::Operation;
    use geop_ops_parameters::{DefineParameters, Parameter, Parameters};
    let (mut editor, _) = editor();
    assert!(parameters_of(editor.program()).is_empty());
    editor.handle(Command::Seek { marker: Some(2) });
    handled(
        &mut editor,
        Command::New {
            kind: "extrude".into(),
        },
    );
    editor.handle(dialog("distance", Value::Number(2.0)));
    let parameters = Parameters {
        values: vec![Parameter::number("depth", "3", 0.0, 100.0)],
        ..Parameters::default()
    };
    handled(
        &mut editor,
        crate::registry::testing::set_parameters(parameters),
    );
    handled(&mut editor, Command::Commit);
    let steps = &editor.program().steps;
    assert_eq!(steps[0].operation, DefineParameters::KIND);
    assert_eq!(steps[3].id, "extrude1");
    assert_eq!(distance(&editor, "extrude1"), 2.0);
    let update = editor.handle(Command::Show);
    assert_eq!(update.program.unwrap().marker, 4);
}

/// Every example loads by name, and builds.
#[test]
fn examples_load_by_name() {
    let mut editor = crate::editor();
    let names = editor.handle(Command::Show).program.unwrap().examples;
    assert_eq!(
        names.len(),
        crate::registry().unwrap().example_programs().len()
    );
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
    let before = editor.program().build_model().unwrap();
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
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
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
    let above = |x: f64, y: f64| from_above(x, y);
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("faces", Value::Press));
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the shell is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: geop_ops_shell::ShellArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(saved(&args.faces), saved(["extrude(box,end)"]), "{args:?}");
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
        geop_ops_reference_geometry::AddDatumArgs {
            selection: vec![Selected::Face(element("extrude(box,end)"))],
            construction: geop_ops_reference_geometry::Construction::Offset {
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
            Shape::Handle { at, direction } if v.key == "distance" => Some((at, direction)),
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
    let args: geop_ops_reference_geometry::AddDatumArgs =
        args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.construction,
        geop_ops_reference_geometry::Construction::Offset {
            distance: 0.8.into()
        }
    );
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
    editor.handle(click(from_above(0.3, 0.3)));
    let update = editor.handle(dialog("construction", Value::Choice("offset".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the plane is edited");
    let (at, _) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            Shape::Handle { at, direction } if v.key == "distance" => Some((at, direction)),
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
    let update = handled(&mut editor, click_at(from_above(0.3, 0.3)));
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let (at, direction) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            Shape::Handle { at, direction } if v.key == "spacing" => Some((at, direction)),
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
    handled(&mut editor, dialog("count", Value::Number(4.0)));
    let update = handled(&mut editor, Command::Commit);
    let args = args::<geop_ops_pattern::LinearPatternArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.first.count, 4.0.into());
    match &args.first.spacing {
        geop_ops_pattern::Spacing::Step(step) => {
            let step = step.plain().expect("a plain spacing");
            assert!((step - 2.0).abs() < 1e-9, "{step}")
        }
        other => panic!("{other:?}"),
    }
    let solids: Vec<String> = update
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == "solid")
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
    let pickable = &step.presentation.pickable;
    assert!(
        [drawn::VERTEX, drawn::EDGE]
            .iter()
            .all(|k| pickable.contains(&k.to_string())),
        "{pickable:?}"
    );
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
    // The rims as edges, the corner as its vertex: a point, which leaves
    // the route free to choose its way.
    let picked: Vec<Selected> = (through.elements())
        .map(|e| Selected::from_picked(e).expect("an element of the part"))
        .collect();
    assert!(
        matches!(
            picked.as_slice(),
            [Selected::Edge(_), Selected::Edge(_), Selected::Vertex(_)]
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
            .any(|v| v.key.starts_with("route:") && v.style == Style::Failed),
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

    let cable = geop_ops_harness::cable(&editor.part(), "route(route1)")
        .expect("the route's cable")
        .clone();
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
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadExample {
            name: "horn".into(),
        },
    );
    let solids = |update: &Update| {
        let scene = update.scene.as_ref().expect("a scene");
        scene.structure.iter().filter(|i| i.kind == "solid").count()
    };
    assert_eq!(solids(&update), 1);

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    let sketch = |name: &str| Value::Elements(vec![geop_ops_sketch::whole_sketch(name).picked()]);
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    let update = editor.handle(dialog("rails", Value::Press));
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::SKETCH, drawn::EDGE]
    );
    let update = editor.handle(dialog("rails", sketch("flare")));
    let dialog_shown = update.step.unwrap().presentation.dialog;
    assert!(
        dialog_shown.get("twist").is_none(),
        "the rail decides the twist"
    );

    editor.handle(dialog("rails", Value::Elements(Vec::new())));
    editor.handle(dialog("orientation", Value::Choice("fixed_normal".into())));
    editor.handle(dialog("twist", Value::Number(90.0)));
    let update = editor.handle(dialog("end_scale", Value::Number(0.5)));
    let step = update.step.unwrap();
    assert_eq!(step.error, None);
    let Some(Control::Number(twist)) = step.presentation.dialog.get("twist") else {
        panic!("the twist is shown without rails");
    };
    assert_eq!(twist.value, 90.0);
    let update = handled(&mut editor, Command::Commit);
    assert_eq!(solids(&update), 2);
    let program = editor.program();
    let args: geop_ops_extrude_revolve::SweepArgs = args(program.steps.last().unwrap());
    assert_eq!(
        (
            args.twist,
            args.end_scale,
            args.orientation,
            args.rails.len()
        ),
        (90.0, 0.5, geop_lib_sweep::Orientation::FixedNormal, 0)
    );
}

/// A new hole, the way the front end makes one: the plate's top clicked
/// as the face, the sketch's point clicked as the centre, a counterbored
/// M5 through all chosen in the dialog — which says what that comes to —
/// committed, and the plate drilled; made tapped, its thread drawn.
#[test]
fn new_hole_from_the_dialog() {
    let mut editor = loaded(geop_ops_hole::testing::plate(&[[20.0, 20.0]]));
    editor.handle(Command::New {
        kind: "hole".into(),
    });
    let above = |x: f64, y: f64| pointer([x, y, 50.0], [0.0, 0.0, -1.0]);
    handled(&mut editor, click_at(above(40.0, 30.0)));
    editor.handle(dialog("points", Value::Press));
    handled(&mut editor, click_at(above(20.0, 20.0)));
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
    let update = handled(&mut editor, Command::Commit);
    let args = args::<geop_ops_hole::HoleArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.face, Some(element("extrude(plate,end)")));
    assert_eq!(
        args.points,
        [geop_ops_hole::HoleCentre::SketchPoint(
            geop_ops_sketch::sketch_element("centres", "p0")
        )]
    );
    assert_eq!(args.kind, geop_ops_hole::HoleKind::Counterbore);
    assert_eq!(args.end, geop_ops_extrude_revolve::Extent::ThroughAll);
    let scene = update.scene.expect("the scene changed");
    assert_eq!(scene.part.solids, ["hole(hole1)"]);

    // Tapped instead: the scene draws its cosmetic thread.
    editor.handle(Command::Open { id: "hole1".into() });
    editor.handle(dialog("kind", Value::Choice("tapped".into())));
    let update = handled(&mut editor, Command::Commit);
    let scene = update.scene.expect("the scene changed");
    assert_eq!(scene.part.annotations.len(), 1);
    assert_eq!(scene.part.annotations[0].label, "M5x0.8");
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
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::EDGE, drawn::SKETCH_CURVE]
    );
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
    let step = editor.program().steps.last().unwrap().clone();
    let boundary: geop_ops_surface::BoundarySurfaceArgs = args(&step);
    assert_eq!(boundary.edges.len(), 4, "{boundary:?}");
    let sheet = format!("boundary({})", step.id);
    let scene = update.scene.expect("the scene is sent anew");
    let faces = |scene: &geop_core_editor::editor::SceneState| -> Vec<String> {
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
    let step = editor.program().steps.last().unwrap().clone();
    let thicken: geop_ops_surface::ThickenArgs = args(&step);
    assert_eq!(thicken.face.name(), sheet);
    assert_eq!(thicken.thickness, 0.2);
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
    let mut editor = crate::editor();
    let mut program = geop_ops_extrude_revolve::examples::pipe();
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
    let click = |editor: &mut Editor, origin: [f64; 3], dir: [f64; 3]| {
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
        .filter(|v| matches!(v.shape, Shape::Point { .. }))
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
    let drawn: geop_ops_sketch3d::AddSketch3dArgs = args(&route);
    assert_eq!(drawn.sketch.curves.len(), 3);
    assert_eq!(drawn.references.len(), 1, "the origin, as a fixed point");

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let sweep: geop_ops_extrude_revolve::SweepArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        (sweep.profile, sweep.path),
        (
            geop_ops_sketch::whole_sketch("section"),
            Some(geop_ops_extrude_revolve::PathRef::Sketch3d(
                geop_ops_sketch3d::sketch3d_path(&route.id)
            ))
        )
    );
    let part = editor.program().build_model().unwrap();
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// The number a program's state gives `name`.
fn state_number(editor: &Editor, name: &str) -> f64 {
    match editor.program().state.get(name) {
        Some(StateValue::Number(v)) => v.to_f64(),
        other => panic!("{name} is no number: {other:?}"),
    }
}

/// How far the placed part `instance` is turned about `z`, in degrees.
fn turn_of(editor: &Editor, instance: &str) -> f64 {
    match editor.program().state.get(&pose_state(instance)) {
        Some(StateValue::Pose(pose)) => pose.euler_degrees()[2],
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
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "arm".into(),
            folder: None,
        },
    );
    let elbow = "add_part(fore,m1).angle";
    assert_eq!(state_number(&editor, elbow), 30.0);
    let program = update.program.unwrap();
    let joints = joints_panel(&program);
    assert_eq!(joints["joints"].as_array().unwrap().len(), 2, "{joints:?}");
    let freedom = &joints["freedom"];
    assert_eq!(freedom["parts"]["upper"], 0);
    assert_eq!(freedom["parts"]["fore"], 1);
    assert_eq!(freedom["parts"]["hand"], 2);

    handled(&mut editor, Command::Open { id: "fore".into() });
    // Grabbed halfway along, where nothing lies on it, and dragged round
    // the elbow at (3, 0) to 170°, looking down.
    let down = |[x, y]: [f64; 2]| from_above(x, y);
    let round = |degrees: f64| {
        let a = degrees.to_radians();
        [3.0 + 1.5 * a.cos(), 1.5 * a.sin()]
    };
    let mut done = false;
    for (k, angle) in [60.0, 100.0, 140.0, 170.0].into_iter().enumerate() {
        done = k == 3;
        handled(
            &mut editor,
            Command::Event {
                event: StepEditEvent::Drag {
                    from: down(round(30.0)),
                    to: down(round(angle)),
                    done,
                    shift: false,
                },
            },
        );
    }
    assert!(done);
    handled(&mut editor, Command::Commit);
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
    handled(&mut editor, dialog("mate:m1:angle", Value::Number(-60.0)));
    handled(&mut editor, Command::Commit);
    assert_eq!(state_number(&editor, elbow), -60.0);
    let fore = turn_of(&editor, "fore");
    assert!(
        (fore + 60.0).abs() < 1e-9,
        "{fore} {:?}",
        editor.program().state
    );

    // Set in the program's panel: the hand turns, the forearm stays.
    let wrist = "add_part(hand,m1).angle";
    let update = handled(&mut editor, set_joint(wrist, 90.0));
    assert_eq!(state_number(&editor, wrist), 90.0);
    assert!((turn_of(&editor, "fore") + 60.0).abs() < 1e-9);
    assert!((turn_of(&editor, "hand") - 30.0).abs() < 1e-9);
    let program = update.program.unwrap();
    let joints = &joints_panel(&program)["joints"];
    assert_eq!(joints[1]["values"][0]["value"], 90.0);
    assert_eq!(joints[1]["values"][0]["max"], 120.0);
}

/// A new draft picks the faces to tilt by clicking them, then the neutral
/// plane: the enclosure's +x wall from the side, its bottom from below.
#[test]
fn new_draft_picks_its_faces_and_neutral_plane() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::Load {
        program: geop_ops_plastic::testing::enclosure(),
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
    let args: geop_ops_plastic::DraftArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.faces,
        [element(geop_ops_plastic::testing::wall("c5"))],
        "{args:?}"
    );
    assert_eq!(
        args.neutral,
        Some(geop_ops_sketch::SketchPlane::Face(element(
            "extrude(box,start)"
        )))
    );
    assert_eq!(args.angle, 5.0);
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
    let mut editor = crate::editor();
    let update = editor.handle(Command::Load {
        program: geop_ops_plastic::testing::enclosure_with_rib_sketch(),
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
    let args: geop_ops_plastic::RibArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.sketch, geop_ops_sketch::whole_sketch("rib_sketch"));
    assert_eq!(args.solid.name(), "shell(s)");
    assert!(args.flipped);
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
    let screw = "std:iso4762_socket_head_cap_screw.geop";
    let (plate, hole) = (
        geop_ops_assembly::examples::metric_plate(),
        geop_ops_assembly::examples::metric_plate_hole(),
    );
    let mut editor = crate::editor();
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
    let mates = [
        (
            "m1",
            "concentric",
            vec![placed_datum("part2", "axis"), placed_face("part1", &hole)],
        ),
        (
            "m2",
            "coincident",
            vec![
                placed_datum("part2", "seat"),
                placed_face("part1", "extrude(plate,end)"),
            ],
        ),
    ];
    for (id, kind, entities) in mates {
        editor.handle(dialog("add_mate", Value::Choice(kind.into())));
        let update = editor.handle(dialog(
            &format!("mate:{id}:entities"),
            Value::Elements(entities),
        ));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let args: geop_ops_assembly::AddPartArgs = args(&program.steps[1]);
    assert_eq!(
        args.parameters.get("size"),
        Some(&Setting::Text("M4x12".into()))
    );
    assert!(
        geop_ops_assembly::solving::check(&editor.part())
            .unwrap()
            .converged
    );
    let Some(StateValue::Pose(pose)) = program.state.get(&pose_state("part2")) else {
        panic!("the screw has a pose");
    };
    // Its head on the plate's top, its tip 12 below, down the hole.
    let v = |p: [f64; 3]| Vector3::from_array(p.map(Design::from_f64));
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
    let (rail, carriage) = ("std:linear_rail.geop", "std:linear_carriage.geop");
    let mut editor = crate::editor();
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
        Value::Elements(vec![
            placed_datum("part1", "axis"),
            placed_datum("part2", "axis"),
        ]),
    ));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let program = editor.program();
    let args: geop_ops_assembly::AddPartArgs = args(&program.steps[0]);
    assert_eq!(args.parameters.get("length"), Some(&Setting::Number(300.0)));
    let part = editor.part();
    assert!(geop_ops_assembly::solving::check(&part).unwrap().converged);
    let joints = geop_ops_assembly::solving::settings(&part);
    assert_eq!(joints.len(), 1, "{joints:?}");
    assert_eq!(joints[0].kind, "Slider");
    // On the rail: its axis on the rail's — the carriage drawn sitting on a
    // rail at the origin, so its pose turns nothing and moves it only along
    // the rail.
    // A part the solve left where it was drawn has no pose in the state.
    let pose = match program.state.get(&pose_state("part2")) {
        Some(StateValue::Pose(pose)) => *pose,
        None => Pose::identity(),
        other => panic!("the carriage's pose is {other:?}"),
    };
    let origin = pose.apply(&Vector3::zero());
    let origin = [0, 1, 2].map(|k| origin[k].to_f64());
    assert!(
        origin[0].abs() < 1e-6 && origin[1].abs() < 1e-6,
        "{origin:?}"
    );
}

/// A new SubD step starts from a box cage, its vertices, edges and faces
/// drawn to click. Clicked from above, the cage's top face is selected —
/// not the bottom one behind it — and its centre's coordinates shown;
/// Extrude pulls it out; dragged by its gizmo's `z` arrow and then by the
/// face itself, it moves; and the limit body the editor shows is the solid the
/// step builds.
#[test]
fn subd_cage_is_shaped_in_the_viewport() {
    let mut editor = crate::editor();
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
    let number = |step: &geop_core_editor::editor::StepState, key: &str| match step
        .presentation
        .dialog
        .get(key)
    {
        Some(Control::Number(n)) => n.value,
        other => panic!("{key}: {other:?}"),
    };
    let style = |step: &geop_core_editor::editor::StepState, key: &str| {
        step.presentation
            .visuals
            .iter()
            .find(|v| v.key == key)
            .map(|v| v.style)
    };
    let update = editor.handle(click(from_above(0.3, 0.2)));
    let step = update.step.expect("the subd is edited");
    assert_eq!(style(&step, "f9"), Some(Style::Selected));
    assert_eq!(style(&step, "f8"), Some(Style::Region));
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
    let args: geop_ops_subd::SubdArgs = args(last);
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 2.0).abs() < 1e-9, "{at:?}");
    }
    assert_eq!(args.cage.faces.len(), 10);
    let scene = update.scene.expect("the part changed");
    assert_eq!(scene.part.solids, [format!("subd({})", last.id)]);
}

/// A drag of the subd cage's top face whose release never arrives, and
/// then another drag of it, started elsewhere: the second is a new drag,
/// and moves the face on from where the first left it — it neither carries
/// on the first, nor starts from the cage as it was before the first.
#[test]
fn subd_drag_without_a_release_is_not_undone_by_the_next() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::New {
        kind: "subd".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    // Looking down at 45 degrees, at `x` across: moving the eye up by `d`
    // moves the face by `d / 2` up and `d / 2` back.
    let slanted = |x: f64, dz: f64| pointer([x, -10.0, 11.75 + dz], [0.0, 1.0, -1.0]);
    for (x, done) in [(0.3, false), (-0.3, true)] {
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Hover {
                pointer: slanted(x, 0.0),
                shift: false,
            },
        });
        assert!(
            update.step.unwrap().presentation.grab,
            "the face offers a grab"
        );
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: slanted(x, 0.0),
                to: slanted(x, 0.2),
                done,
                shift: false,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let args: geop_ops_subd::SubdArgs = args(program.steps.last().unwrap());
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 1.2).abs() < 1e-9, "{at:?}");
    }
}

/// The measure tool, used as the viewer uses it: taken in hand, the box's
/// top hovered and clicked — its area, its plane — then its bottom clicked
/// from below: one unit apart, parallel, the least distance drawn between
/// them. Then the mass properties and the interference of the part asked
/// for. None of it changes the program, and the tool put down shows nothing.
#[test]
fn measure_tool_measures_what_is_clicked() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let update = editor.handle(Command::Tool {
        key: Some("pick:measure".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(
        update.program.unwrap().tool.as_deref(),
        Some("pick:measure")
    );
    let top = face("extrude(box,end)");
    let bottom = face("extrude(box,start)");
    let from_below = pointer([0.5, 0.5, -10.0], [0.0, 0.0, 1.0]);
    let measured = |update: &Update| match &update.inspection {
        Some(inspection) if inspection.kind == "measure" => inspection.report.clone(),
        other => panic!("no measurement: {other:?}"),
    };
    let value = |m: &serde_json::Value, label: &str| {
        let found = (m["values"].as_array().unwrap().iter())
            .find(|v| v["label"] == label)
            .unwrap_or_else(|| panic!("no {label}: {m}"));
        Bounded {
            value: found["value"].as_f64().unwrap(),
            error: found["error"].as_f64().unwrap(),
        }
    };

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(0.5, 0.5),
            shift: false,
        },
    });
    assert_eq!(update.tool.unwrap().highlights, vec![top.clone()]);

    let update = editor.handle(click_at(from_above(0.5, 0.5)));
    assert!(update.program.is_none(), "measuring changes no program");
    let m = measured(&update);
    assert_eq!(m["entities"], serde_json::json!([top]));
    let hole = std::f64::consts::PI * 0.4 * 0.4;
    assert!(value(&m, "Area").contains(4.0 - hole), "{m}");
    assert!(!m["plane"].is_null());

    let update = editor.handle(click_at(from_below));
    let m = measured(&update);
    assert_eq!(m["entities"], serde_json::json!([top, bottom]));
    assert!(value(&m, "Distance").contains(1.0), "{m}");
    assert!(value(&m, "Angle").contains(0.0), "{m}");
    let tool = update.tool.unwrap();
    assert!(tool.visuals.iter().any(|v| v.key == "distance"));

    let update = editor.handle(Command::Inspect {
        inspector: "mass_properties".into(),
        options: serde_json::Value::Null,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let report = update.inspection.unwrap().report;
    assert_eq!(report["bodies"].as_array().unwrap().len(), 1);
    let volume = &report["total"]["volume"];
    let volume = Bounded {
        value: volume["value"].as_f64().unwrap(),
        error: volume["error"].as_f64().unwrap(),
    };
    assert!(volume.contains(4.0 - hole * 0.5), "{volume:?}");

    let update = editor.handle(Command::Inspect {
        inspector: "interference".into(),
        options: serde_json::Value::Null,
    });
    let report = update.inspection.unwrap().report;
    assert_eq!(report["solids"], 1);
    assert_eq!(report["found"], serde_json::json!([]));

    let update = editor.handle(Command::Tool { key: None });
    assert!(update.tool.is_none() && update.inspection.is_none());
    assert_eq!(*editor.program(), before);
}

/// The arguments of the drawing being edited.
fn drawing_args(editor: &Editor) -> geop_ops_drawing::DrawingArgs {
    args(editor.editing().expect("a drawing is edited"))
}

/// The drawing `editor` shows, downloaded as the dialog's buttons do, in
/// `format`, dated `date`.
fn drawing_file(editor: &mut Editor, format: &str, date: &str) -> Update {
    editor.handle(Command::Export {
        exporter: "drawing".into(),
        format: Some(format.into()),
        options: serde_json::Value::Null,
        date: date.into(),
    })
}

/// Where the vertices of the drawing being edited are on its sheet, in the
/// view `view`, as the front end shows them: by name.
fn sheet_vertices(editor: &Editor, view: geop_ops_drawing::DrawnView) -> Vec<(String, [f64; 2])> {
    let args = drawing_args(editor);
    let layout = geop_ops_drawing::layout(&editor.part(), &args, "", &[]).unwrap();
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
    let args = drawing_args(&editor);
    let layout = geop_ops_drawing::layout(&editor.part(), &args, "", &[]).unwrap();
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
    let args = drawing_args(&editor);
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
    let args = drawing_args(&editor);
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
    use geop_ops_drawing::{Annotation, DrawnView, ViewKind};
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
        Some(Control::Download { formats, .. }) if formats.len() == 2
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
    let args = drawing_args(&editor);
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
    let args = drawing_args(&editor);
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
    let args = drawing_args(&editor);
    assert!(
        matches!(&args.annotations[1], Annotation::Note { text, leader: Some(_), .. } if text == "DEBURR"),
        "{:?}",
        args.annotations
    );

    // Downloaded from the dialog, as it now is.
    let exported = drawing_file(&mut editor, "svg", "2026-10-04");
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let file = exported.export.unwrap();
    assert_eq!(file.name, "part.svg");
    let svg = file.text().unwrap();
    assert!(svg.starts_with("<svg"));
    for text in ["Drilled box", ">2.83<", ">DEBURR<"] {
        assert!(svg.contains(text), "{text} is not on the sheet");
    }
    // The blind hole, seen from the front, is hidden.
    let hidden = &svg[svg.find(r#"<g class="HIDDEN""#).unwrap()..];
    assert!(hidden[..hidden.find("</g>").unwrap()].contains("<line"));
    let dxf = drawing_file(&mut editor, "dxf", "");
    let dxf = dxf.export.unwrap();
    assert!(dxf.text().unwrap().ends_with("EOF\n") && dxf.text().unwrap().contains("DEBURR"));

    let committed = editor.handle(Command::Commit);
    assert!(committed.error.is_none(), "{:?}", committed.error);
    // Back to the part: the scene draws its faces again.
    assert!(!committed.scene.expect("a scene").part.faces.is_empty());
    let step = editor.program().steps.last().unwrap();
    assert_eq!(step.operation, "drawing");
    let args: geop_ops_drawing::DrawingArgs = crate::registry::testing::args(step);
    assert_eq!((args.views.len(), args.annotations.len()), (3, 2));
    assert_eq!(args.name, "Drilled box");
}

/// The arm, exported as a URDF robot the way the front end asks — the
/// command and the update as JSON: a ZIP archive named after the file, its
/// bytes in base64, holding `robot.urdf` with the arm's joints. Refused for
/// the four-bar, whose bars only mates hold: the error names them.
#[test]
fn an_assembly_is_exported_as_urdf() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let json = editor
        .handle_json(r#"{"command": "export", "exporter": "urdf"}"#)
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    let export = &update["export"];
    assert_eq!(export["name"], "arm.zip");
    assert!(export["text"].is_null());
    // "PK\x03\x04", a ZIP archive's first local header, in base64.
    let bytes = export["bytes"].as_str().unwrap();
    assert!(bytes.starts_with("UEsDBA"), "{}", &bytes[..16]);
    assert_eq!(bytes.len() % 4, 0);

    let urdf = |editor: &mut Editor| {
        editor.handle(Command::Export {
            exporter: "urdf".into(),
            format: None,
            options: serde_json::Value::Null,
            date: String::new(),
        })
    };
    let zip = urdf(&mut editor).export.unwrap();
    let text = String::from_utf8_lossy(zip.bytes());
    assert!(text.contains("robot.urdf") && text.contains("meshes/link.stl"));
    assert!(text.contains(r#"<joint name="add_part(fore,m1)" type="revolute">"#));
    assert!(text.contains(r#"<joint name="add_part(hand,m1)" type="revolute">"#));

    editor.handle(Command::LoadWorkspaceExample {
        name: "four_bar".into(),
        folder: None,
    });
    let refused = urdf(&mut editor);
    assert!(refused.export.is_none());
    let error = refused.error.unwrap();
    assert!(
        error.contains("free to move") && error.contains("coupler"),
        "{error}"
    );
}

/// The part shown, downloaded as STEP the way the front end asks for it.
fn step_file(editor: &mut Editor) -> String {
    let update = editor.handle(Command::Export {
        exporter: "step".into(),
        format: None,
        options: serde_json::Value::Null,
        date: String::new(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let export = update.export.expect("a file is written");
    assert!(export.name.ends_with(".step"), "{}", export.name);
    let text = export.text().expect("STEP is text").to_string();
    assert!(text.contains("MANIFOLD_SOLID_BREP"), "no solid in {text}");
    text
}

/// The part shown written as STEP, stored next to a new program in a
/// folder — as the front end stores a file the user chooses in the file
/// field — is offered by a new "Import STEP" step by its path relative to
/// the program, and brings its solid back, named after the step.
#[test]
fn a_part_exported_as_step_is_imported_back() {
    let (mut editor, _) = editor();
    let files = std::collections::BTreeMap::from([(
        "parts/box.step".to_string(),
        Some(step_file(&mut editor)),
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
        .filter(|item| item.kind == "solid")
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
    let text = step_file(&mut editor);
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

/// "Download STL", as the File menu offers it: the exporters the program
/// state lists, the one picked asked for by its kind. It writes every
/// solid of the part shown as a binary STL mesh, named after the program's
/// file — the placed parts' solids too, so the pin in the plate adds to the
/// plate's triangles. An exporter there is none of is refused, naming those
/// there are; so is a format the exporter does not write.
#[test]
fn a_part_is_exported_as_stl_with_its_placed_parts() {
    /// The triangles of a binary STL file, checked against its length.
    fn triangles(update: Update) -> u32 {
        assert!(update.error.is_none(), "{:?}", update.error);
        let export = update.export.expect("a file is written");
        assert_eq!(export.media_type, "model/stl");
        let Content::Bytes(bytes) = export.content else {
            panic!("a binary file");
        };
        assert!(!bytes.starts_with(b"solid"), "a binary header");
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
        assert_eq!(bytes.len(), 84 + 50 * count as usize);
        count
    }
    let export = |exporter: &str, format: Option<&str>, options| Command::Export {
        exporter: exporter.into(),
        format: format.map(str::to_string),
        options,
        date: String::new(),
    };
    let stl = || export("stl", None, serde_json::Value::Null);
    let (mut editor, update) = editor();
    let offered = update.program.unwrap().exporters;
    assert!(
        offered
            .iter()
            .any(|e| e.kind == "stl" && e.formats == ["stl"]),
        "{offered:?}"
    );
    let plate = triangles(editor.handle(stl()));
    assert!(plate > 0);

    editor.handle(Command::LoadWorkspaceExample {
        name: "pin_in_plate".into(),
        folder: None,
    });
    let json = (editor
        .handle_json(r#"{"command": "export", "exporter": "stl", "date": "2026-10-11"}"#))
    .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert_eq!(update["export"]["name"], "assembly.stl");
    assert!(update["export"]["bytes"].is_string(), "{update}");
    let assembly = triangles(editor.handle(stl()));
    assert!(
        assembly > plate,
        "{assembly} triangles, the plate alone {plate}"
    );
    // Only the pin, as the exporter's own options say.
    let options = serde_json::json!({ "solids": ["pin/extrude(pin)"] });
    let pin = triangles(editor.handle(export("stl", None, options)));
    assert!(pin < assembly, "{pin} of {assembly}");

    let update = editor.handle(export("gltf", None, serde_json::Value::Null));
    let error = update.error.expect("refused");
    assert!(
        error.contains("no exporter `gltf`") && error.contains("stl"),
        "{error}"
    );
    assert!(update.export.is_none());
    let update = editor.handle(export("stl", Some("obj"), serde_json::Value::Null));
    assert!(update.error.expect("refused").contains("writes no `obj`"));
}

/// An inspector of the tests that picks faces, two at most: what it says
/// is which are picked, and it draws a point where the first one is hit.
struct Picked;

impl Inspector for Picked {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "picked",
        label: "Picked",
        doc: "Which faces are picked.",
        icon: None,
        picks: Some(Picks {
            accepts: &[drawn::FACE],
            most: 2,
        }),
    };
    type Options = ();
    type Report = Vec<String>;

    fn inspect(
        _: &mut Made,
        picked: &[PickedElement],
        _: (),
    ) -> GeopResult<(Vec<String>, Vec<Visual<S>>)> {
        let names = picked.iter().map(|p| format!("{p:?}")).collect();
        let at = Vector3::from_array([0.5, 0.5, 1.0].map(S::from_f64));
        let visuals = (picked.first())
            .map(|_| Visual::new("first", Shape::Point { at }, Style::Selected))
            .into_iter()
            .collect();
        Ok((names, visuals))
    }
}

/// An inspector of the tests asked of the whole part: how many solids it
/// has, and the label it is told to give that.
struct Solids;

#[derive(Default, serde::Deserialize)]
struct SolidsOptions {
    label: String,
}

impl Inspector for Solids {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "solids",
        label: "Solids",
        doc: "How many solids the part has.",
        icon: None,
        picks: None,
    };
    type Options = SolidsOptions;
    type Report = (String, usize);

    fn inspect(
        output: &mut Made,
        _: &[PickedElement],
        options: SolidsOptions,
    ) -> GeopResult<((String, usize), Vec<Visual<S>>)> {
        let model = read_model(&output.part())?;
        Ok(((options.label, model.solid_names().len()), Vec::new()))
    }
}

/// Inspecting as the inspect panel does, with whatever inspectors the
/// editor offers: listed in the program state; one that picks taken in
/// hand, the box's top hovered and clicked, then its bottom clicked from
/// below — each pick asked about again, the answer's point drawn; one of
/// the whole part asked once, told its options. None of it changes the
/// program, and the tool put down shows nothing.
#[test]
fn inspectors_are_offered_picked_with_and_asked() {
    // An editor of what the box is built of, offering only these
    // inspectors.
    let registry = (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(|r| r.inspector(Picked))
        .and_then(|r| r.inspector(Solids))
        .unwrap();
    let mut editor = editor_with(
        registry,
        geop_ops_extrude_revolve::examples::box_with_drill_hole(),
    );
    let before = editor.program().clone();
    let update = handled(&mut editor, Command::Show);
    let program = update.program.unwrap();
    let kinds: Vec<&str> = program.inspectors.iter().map(|i| i.kind).collect();
    assert_eq!(kinds, ["picked", "solids"]);
    assert_eq!(program.tool, None);

    let update = handled(
        &mut editor,
        Command::Tool {
            key: Some("pick:picked".into()),
        },
    );
    assert_eq!(update.program.unwrap().tool.as_deref(), Some("pick:picked"));
    let top = face("extrude(box,end)");
    let bottom = face("extrude(box,start)");
    let report = |update: &Update| {
        let inspection = update.inspection.as_ref().expect("an answer");
        assert_eq!(inspection.kind, "picked");
        serde_json::from_value::<Vec<String>>(inspection.report.clone()).unwrap()
    };
    let update = handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Hover {
                pointer: from_above(0.5, 0.5),
                shift: false,
            },
        },
    );
    assert_eq!(update.tool.as_ref().unwrap().highlights, vec![top.clone()]);
    assert!(report(&update).is_empty());

    let update = handled(&mut editor, click_at(from_above(0.5, 0.5)));
    assert!(update.program.is_none(), "inspecting changes no program");
    assert_eq!(report(&update), [format!("{top:?}")]);
    assert!(
        update
            .tool
            .as_ref()
            .unwrap()
            .visuals
            .iter()
            .any(|v| v.key == "first")
    );
    let update = handled(
        &mut editor,
        click_at(pointer([0.5, 0.5, -10.0], [0.0, 0.0, 1.0])),
    );
    assert_eq!(report(&update), [format!("{top:?}"), format!("{bottom:?}")]);

    let json = editor
        .handle_json(
            r#"{"command": "inspect", "inspector": "solids", "options": {"label": "bodies"}}"#,
        )
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{update}");
    assert_eq!(update["inspection"]["kind"], "solids");
    assert_eq!(
        update["inspection"]["report"],
        serde_json::json!(["bodies", 1])
    );

    let update = editor.handle(Command::Tool {
        key: Some("pick:solids".into()),
    });
    assert!(update.error.expect("refused").contains("picks nothing"));
    let update = editor.handle(Command::Inspect {
        inspector: "mass".into(),
        options: serde_json::Value::Null,
    });
    assert!(
        update
            .error
            .expect("refused")
            .contains("there is no inspector `mass`")
    );

    let update = handled(&mut editor, Command::Tool { key: None });
    assert!(update.tool.is_none() && update.inspection.is_none());
    assert_eq!(*editor.program(), before);
}

/// The structure tree of an assembly, as the front end lists it: the
/// placed parts, and the mates their steps list — named, with the entities
/// they are between, and never drawn.
#[test]
fn an_assembly_lists_its_mates() {
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "pin_in_plate".into(),
            folder: None,
        },
    );
    let structure = update.scene.expect("a scene").structure;
    let of = |kind: &str| -> Vec<&geop_core_editor::registry::StructureItem> {
        structure.iter().filter(|i| i.kind == kind).collect()
    };
    let parts: Vec<&str> = of("part").iter().map(|i| i.name.as_str()).collect();
    assert!(parts.contains(&"pin"), "{structure:?}");
    let mates = of("mate");
    let names: Vec<&str> = mates.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "add_part(plate,fixed)",
            "add_part(pin,m1)",
            "add_part(pin,m2)"
        ],
        "{structure:?}"
    );
    assert!(mates.iter().all(|mate| mate.visible.is_none()));
    // The plate held by its own frame; the pin's faces against the hole's.
    let between: Vec<usize> = mates.iter().map(|mate| mate.elements.len()).collect();
    assert_eq!(between, [1, 2, 2], "{mates:?}");
    assert_eq!(
        mates[2].elements,
        [
            placed_face("pin", "extrude(pin,start)"),
            placed_face("plate", "extrude(hole,end)")
        ]
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
    // Diagonally onto the upright edge at the origin, halfway up.
    handled(
        &mut editor,
        click_at(pointer([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0])),
    );
    let update = editor.handle(dialog("variable", Value::Bool(true)));
    let step = update.step.expect("the fillet is edited");
    let number = |step: &geop_core_editor::editor::StepState, key: &str| match step
        .presentation
        .dialog
        .get(key)
    {
        Some(Control::Number(n)) => n.value,
        other => panic!("{key}: {other:?}"),
    };
    assert_eq!(number(&step, "end_radius"), number(&step, "radius"));
    editor.handle(dialog("end_radius", Value::Number(0.2)));
    // The corner on top of that edge, picked from above and outside.
    editor.handle(dialog("radius_vertices", Value::Press));
    let update = handled(
        &mut editor,
        click_at(pointer([-5.0, -5.0, 6.0], [1.0, 1.0, -1.0])),
    );
    let step = update.step.expect("the fillet is edited");
    assert_eq!(number(&step, "vertex_radius_0"), number(&step, "radius"));
    handled(&mut editor, dialog("vertex_radius_0", Value::Number(0.15)));
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let last = program.steps.last().unwrap();
    let args = args::<geop_ops_fillet::FilletArgs>(last);
    assert_eq!(args.edges.len(), 1, "{args:?}");
    assert_eq!(args.end_radius, Some(0.2.into()), "{args:?}");
    assert_eq!(args.vertex_radii.len(), 1, "{args:?}");
    assert_eq!(args.vertex_radii[0].radius, 0.15, "{args:?}");
    let part = program.build_model().unwrap();
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
    // The upright edge, diagonally halfway up; the top's edges along x and
    // y, from above and outside.
    for (origin, dir) in [
        ([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0]),
        ([0.5, -5.0, 6.0], [0.0, 1.0, -1.0]),
        ([-5.0, 0.5, 6.0], [1.0, 0.0, -1.0]),
    ] {
        handled(&mut editor, click_at(pointer(origin, dir)));
    }
    handled(&mut editor, dialog("radius", Value::Number(0.2)));
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let args: geop_ops_fillet::FilletArgs = args(program.steps.last().unwrap());
    assert_eq!(args.edges.len(), 3, "{args:?}");
    let part = program.build_model().unwrap();
    let corners: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|name| name.ends_with(",corner)"))
        .collect();
    assert_eq!(corners.len(), 1, "{corners:?}");
    assert!(crate::registry::testing::check_valid(&part).is_ok());
}

/// The plate's thickness, typed into its extrude's dialog as a formula of
/// the parameters, as one types it in the parameters panel: the field
/// shows the formula and what it comes to, and drops its slider and its
/// handle — a value that follows a formula is not dragged. A formula that
/// does not evaluate is kept as typed, the field and the step saying why;
/// a number typed makes it plain, and draggable, again.
#[test]
fn formulas_are_typed_into_dialog_fields() {
    let mut editor = example_editor("parametric_plate");
    let field =
        |step: &geop_core_editor::editor::StepState| match step.presentation.dialog.get("distance")
        {
            Some(Control::Number(n)) => n.clone(),
            other => panic!("{other:?} is no number field"),
        };
    let handle = |step: &geop_core_editor::editor::StepState| {
        step.presentation
            .visuals
            .iter()
            .any(|v| v.key == "distance" && matches!(v.shape, Shape::Handle { .. }))
    };
    let thickness = |editor: &Editor| {
        let program = editor.program();
        let args: ExtrudeArgs = args(&program.steps[program.index_of("plate").unwrap()]);
        args.extent.side1.clone()
    };
    let top = |editor: &Editor| {
        model(editor)
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

    let update = handled(
        &mut editor,
        dialog("distance", Value::Text("3 * thickness".into())),
    );
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
    handled(&mut editor, Command::Commit);
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
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let ask = r#"{"command": "inspect", "inspector": "bom", "options": {"structure": "flat"}}"#;
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["inspection"]["kind"], "bom");
    let bom = &update["inspection"]["report"];
    assert_eq!(bom["structure"], "flat");
    let designations: Vec<&str> = (bom["lines"].as_array().unwrap().iter())
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
    let lines = update["inspection"]["report"]["lines"].as_array().unwrap();
    assert_eq!(lines.len(), 2);
    editor.handle(Command::Seek { marker: None });

    let json = editor
        .handle_json(
            r#"{"command": "export", "exporter": "bom", "options": {"structure": "indented"}}"#,
        )
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
/// the arm and one for each of the three links it places, each placement an
/// occurrence named after its step — and read back, the three links where
/// the arm has them.
#[test]
fn an_assembly_is_exported_as_step_products() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let text = step_file(&mut editor);
    // TODO: one product of the link for all three, once the parts of no
    // state are shared between instances (see
    // `geop_core_program::part::Part`).
    assert_eq!(
        text.matches("=PRODUCT('").count(),
        4,
        "the arm and each link"
    );
    assert!(text.contains("PRODUCT('link','link'"));
    for occurrence in ["upper", "fore", "hand"] {
        assert!(
            text.contains(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{occurrence}'")),
            "{occurrence} is placed"
        );
    }
    let bodies = geop_ops_step::read_step::<S>(&text).unwrap();
    assert_eq!(bodies.len(), 3, "the link, once where each step places it");
}

/// The sketch tools the way the front end uses them: a plate drawn as a
/// rectangle with a chamfered corner, an arc slot and a circumscribed hex
/// hole in it, and round it all an offset rim — a second sketch offset into
/// a ring by clicks. Each extrudes into a valid solid.
#[test]
fn sketch_tools_draw_profiles_that_extrude() {
    let mut editor = crate::editor();
    let event =
        |editor: &mut Editor, event: StepEditEvent<S>| handled(editor, Command::Event { event });
    let click = |editor: &mut Editor, x: f64, y: f64| {
        event(
            editor,
            StepEditEvent::Click {
                pointer: from_above(x, y),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        )
    };
    let hover = |editor: &mut Editor, x: f64, y: f64| {
        event(
            editor,
            StepEditEvent::Hover {
                pointer: from_above(x, y),
                shift: false,
            },
        )
    };
    let key =
        |editor: &mut Editor, key: &str| event(editor, StepEditEvent::Key { key: key.into() });
    let tool =
        |editor: &mut Editor, name: &str| editor.handle(dialog("tool", Value::Choice(name.into())));
    let extruded = |editor: &mut Editor| {
        handled(editor, Command::Commit);
        editor.handle(Command::New {
            kind: "extrude".into(),
        });
        editor.handle(dialog("distance", Value::Number(0.3)));
        handled(editor, Command::Commit);
        let part = editor.program().build_model().unwrap();
        if let Err(e) = crate::registry::testing::check_valid(&part) {
            panic!("{e}");
        }
        part
    };
    // A new sketch on the origin's xy plane, seen from above.
    let new_sketch = |editor: &mut Editor| {
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
    handled(&mut editor, Command::Commit);
    let rim: geop_ops_sketch::AddSketchArgs = args(editor.program().steps.last().unwrap());
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
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadExample {
            name: "parametric_plate".into(),
        },
    );
    let uses = parameters_panel(&update.program.expect("the program is sent")).parameter_uses;
    let readers = |name: &str| uses.get(name).cloned().unwrap_or_default();
    assert_eq!(readers("width"), ["depth", "outline"]);
    assert_eq!(readers("depth"), ["outline"]);
    assert_eq!(readers("thickness"), ["plate"]);
    assert_eq!(readers("screw"), ["hole_sketch"]);
    let volume = |editor: &Editor| {
        let part = model(editor);
        let [name] = part.solid_names().try_into().expect("one solid");
        crate::registry::testing::solid_volume(&part, &name)
    };
    let before = volume(&editor);

    for (from, to) in [
        ("width", "plate_width"),
        ("screw", "bolt"),
        ("thickness", "t"),
    ] {
        let update = editor.handle(rename_parameter(from, to));
        assert!(update.error.is_none(), "{from} -> {to}: {:?}", update.error);
        let program = update.program.expect("the program is sent");
        assert!(
            program.steps.iter().all(|s| s.error.is_none()),
            "{:?}",
            program.steps
        );
        let panel = parameters_panel(&program);
        assert!(
            panel.parameters.errors.is_empty(),
            "{:?}",
            panel.parameters.errors
        );
        assert!(
            !panel.parameter_uses.contains_key(from),
            "{from} is still read"
        );
    }
    let program = editor.program();
    let json = program.to_json().unwrap();
    for (old, new) in [
        ("\"width / 2\"", "\"plate_width / 2\""),
        ("\"screw.clearance\"", "\"bolt.clearance\""),
        (
            "{\"blind\":{\"formula\":\"thickness\"}}",
            "{\"blind\":{\"formula\":\"t\"}}",
        ),
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

    let taken = editor.handle(rename_parameter("depth", "t"));
    let error = taken.error.expect("a name taken is refused");
    assert!(
        error.contains(r#"there is a parameter "t" already"#),
        "{error}"
    );
    editor.handle(Command::Open { id: "plate".into() });
    let editing = editor.handle(rename_parameter("depth", "plate_depth"));
    assert!(editing.error.is_some(), "renamed while a step is edited");
    editor.handle(Command::Cancel);

    editor.handle(Command::Undo);
    assert!(parameters_of(editor.program()).get("thickness").is_some());
    assert!(parameters_of(editor.program()).get("t").is_none());
}

/// The bolted plate's drawing with its bill of materials, as the front end
/// makes it: a new drawing step, "Bill of materials" ticked, committed and
/// exported. The sheet draws the parts placed, where they are placed, and
/// lists them: the plate, and the standard screw and nut by the titles and
/// designations their own programs carry — which name their products in
/// a STEP file too (see [`an_assembly_step_file_names_its_parts`]) — each
/// line ballooned once with its item number.
#[test]
fn an_assembly_drawing_lists_its_parts() {
    let mut editor = crate::editor();
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
            Shape::Label { text, .. } => Some(text),
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

    let exported = drawing_file(&mut editor, "svg", "2026-10-04");
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
}

/// The bolted plate written as STEP: the standard screw and nut are named
/// by the designations their own programs carry, one product per size.
#[test]
fn an_assembly_step_file_names_its_parts() {
    let mut editor = crate::editor();
    handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "bolted_plate".into(),
            folder: None,
        },
    );
    let text = step_file(&mut editor);
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
    let mut editor = loaded(geop_ops_hole::testing::plate_with_hole());
    editor.handle(Command::New {
        kind: "linear_pattern".into(),
    });
    editor.handle(dialog("features", Value::Press));
    // From above the hole's middle down onto its wall at 45°, 0.1 above
    // the plate's bottom — clear of the rim and of the wall's seams.
    let r = 0.3 * std::f64::consts::FRAC_1_SQRT_2;
    let wall = pointer([1.0, 1.0, 3.0], [r, r, 0.1 - 3.0]);
    let hole = geop_ops_pattern::feature("hole").picked();
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: wall,
            shift: false,
        },
    });
    let step = update.step.expect("the pattern is edited");
    // What is hovered is the wall's face, as the pointer hits it.
    let walls: Vec<PickedElement> = (step.presentation.highlights.iter())
        .filter(|e| e.kind == drawn::FACE)
        .cloned()
        .collect();
    assert_eq!(walls.len(), 1, "{:?}", step.presentation.highlights);
    let update = handled(&mut editor, click_at(wall));
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    // The hole picked is lit as its faces, the wall among them.
    assert!(
        step.presentation.highlights.contains(&walls[0]),
        "{:?}",
        step.presentation.highlights
    );
    let picked = |key: &str| match step.presentation.dialog.get(key) {
        Some(Control::Reference(r)) => r.elements().cloned().collect::<Vec<_>>(),
        other => panic!("{key}: {other:?}"),
    };
    assert_eq!(picked("features"), std::slice::from_ref(&hole));
    assert!(picked("bodies").is_empty());
    // A feature combines as it did: there is no combining to choose.
    assert!(step.presentation.dialog.get("combine").is_none());

    editor.handle(dialog("count", Value::Number(4.0)));
    editor.handle(dialog("spacing", Value::Number(2.0)));
    handled(&mut editor, Command::Commit);
    let args = args::<geop_ops_pattern::LinearPatternArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.seeds.features, [geop_ops_pattern::feature("hole")]);
    assert!(args.seeds.bodies.is_empty());
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
fn event(editor: &mut Editor, event: StepEditEvent<S>) -> geop_core_editor::editor::StepState {
    let update = handled(editor, Command::Event { event });
    update.step.expect("a step is edited")
}

/// Hovers `at`, which must be over `part` of the gizmo — a press there
/// grabs — and drags it to `to`: once on the way, half way, saying what
/// the drag has done so far, then released there.
fn drag_gizmo(
    editor: &mut Editor,
    part: GizmoPart,
    at: Pointer<S>,
    to: Pointer<S>,
    halfway: Pointer<S>,
    shift: bool,
) -> geop_core_editor::editor::StepState {
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
    let mut editor = crate::editor();
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    let step = event(
        &mut editor,
        StepEditEvent::Click {
            pointer: from_above(0.3, 0.2),
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
            .get(geop_core_editor::step_editor::GIZMO_ORIENTATION)
            .is_some()
    );
    editor.handle(dialog(
        geop_core_editor::step_editor::GIZMO_ORIENTATION,
        Value::Choice("world".into()),
    ));
    // No handles any more: the gizmo is what drags.
    assert!(
        !step
            .presentation
            .visuals
            .iter()
            .any(|v| matches!(v.shape, Shape::Handle { .. }))
    );

    // Seen from the side, the `z` arrow pulled up by 0.31: a reach of
    // 0.009 snaps to fiftieths.
    let side = |z: f64| pointer([0.0, -10.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(geop_core_editor::editing::Axis::Z),
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
        from_above(r * c, r * s)
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(geop_core_editor::editing::Axis::Z),
        round(45.0),
        round(92.0),
        round(60.0),
        false,
    );
    // The cube beyond the `x` arrow pulled out to twice as far.
    let along = |x: f64| from_above(x, 0.0);
    let step = drag_gizmo(
        &mut editor,
        GizmoPart::Stretch(geop_core_editor::editing::Axis::X),
        along(12.5 * reach),
        along(25.0 * reach),
        along(20.0 * reach),
        false,
    );
    assert_eq!(step.error, None);

    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let last = editor.program().steps.last().unwrap().clone();
    let args: geop_ops_subd::SubdArgs = args(&last);
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
    let mut editor = crate::editor();
    editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    let click = |editor: &mut Editor, origin: [f64; 3], dir: [f64; 3]| {
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
    let key = |editor: &mut Editor| {
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
        GizmoPart::Move(geop_core_editor::editing::Axis::X),
        front(2.05, 2.0),
        front(2.75, 2.4),
        front(2.4, 2.2),
        false,
    );
    assert_eq!(step.error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: geop_ops_sketch3d::AddSketch3dArgs = args(editor.program().steps.last().unwrap());
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
        GizmoPart::Move(geop_core_editor::editing::Axis::Y),
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
        GizmoPart::Plane(geop_core_editor::editing::Axis::Y),
        front(at[0] + corner, at[2] + corner),
        front(at[0] + corner + 0.2, at[2] + corner - 0.4),
        front(at[0] + corner + 0.1, at[2] + corner),
        false,
    );
    handled(&mut editor, Command::Commit);
    let args: geop_ops_pattern::MoveBodyArgs = args(editor.program().steps.last().unwrap());
    let want = [0.2, 0.3, -0.4];
    assert!(
        (0..3).all(|k| (args.translation[k] - want[k]).abs() < 1e-9),
        "{:?}",
        args.translation
    );
}

/// Placed parts are moved by a gizmo at their origin: a fixed block turned
/// a quarter about `z` by its ring, and a block on it slid along its `z`
/// axis by its `z` arrow — the slider joint between their frames solved as
/// it is dragged.
#[test]
fn placed_parts_are_moved_and_turned_by_the_gizmo() {
    let block = geop_ops_extrude_revolve::examples::box_with_drill_hole()
        .to_json()
        .unwrap();
    let mut editor = crate::editor();
    let files = [("block.geop".to_string(), Some(block))].into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    editor.handle(Command::Load {
        program: Program::new(),
        path: Some("guide.geop".into()),
    });
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("block.geop".into())));
    handled(&mut editor, Command::Commit);
    editor.handle(Command::New {
        kind: "add_part".into(),
    });
    editor.handle(dialog("file", Value::Choice("block.geop".into())));
    editor.handle(dialog("add_mate", Value::Choice("slider".into())));
    editor.handle(dialog(
        "mate:m1:entities",
        Value::Elements(vec![
            placed_datum("part1", "origin"),
            placed_datum("part2", "origin"),
        ]),
    ));
    handled(&mut editor, Command::Commit);
    // A part nothing solved for yet is at the origin.
    let pose = |editor: &Editor, id: &str| match editor.program().state.get(&pose_state(id)) {
        Some(StateValue::Pose(pose)) => *pose,
        None => Pose::identity(),
        other => panic!("{id}: {other:?}"),
    };
    let apply = |pose: &Pose<Design>, p: [f64; 3]| {
        let at = pose.apply(&Vector3::from_array(p.map(Design::from_f64)));
        [0, 1, 2].map(|k| at[k].to_f64())
    };
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);

    // The second block, slid 10 along the first's `z` axis by its `z`
    // arrow, seen from the side.
    let start = apply(&pose(&editor, "part2"), [0.0; 3]);
    let update = editor.handle(Command::Open { id: "part2".into() });
    let step = update.step.expect("the second block is edited");
    let gizmo = step.presentation.gizmo.expect("a placed part has a gizmo");
    assert!(gizmo.modes.translate && gizmo.modes.rotate);
    assert!(
        step.presentation
            .dialog
            .get(geop_core_editor::step_editor::GIZMO_ORIENTATION)
            .is_some()
    );
    let side = |z: f64| pointer([start[0], start[1] - 100.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(geop_core_editor::editing::Axis::Z),
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

    // The first block, fixed, turned a quarter about `z` by its ring, seen
    // from above: it goes where it is turned, and the second turns with it.
    editor.handle(Command::Open { id: "part1".into() });
    let reach = 0.009;
    let round = |degrees: f64| {
        let (s, c) = degrees.to_radians().sin_cos();
        let r = 7.0 * reach;
        pointer([r * c, r * s, 100.0], [0.0, 0.0, -1.0])
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(geop_core_editor::editing::Axis::Z),
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
    assert!(
        geop_ops_assembly::solving::check(&editor.part())
            .unwrap()
            .converged
    );
}

/// The toolbar's sections are the editor's: it sends the operations group
/// by group, in the groups' order, every operation in one, and within a
/// group the few used most (shown big) first, then the common ones (small),
/// then those only in the group's menu.
#[test]
fn operations_are_offered_by_group() {
    use geop_core_editor::editable::OperationTier;
    let (_, update) = editor();
    let operations = update.program.expect("the program is sent").operations;
    assert_eq!(operations.len(), crate::registry().unwrap().infos().len());
    let order: Vec<_> = operations
        .iter()
        .map(|o| (o.info.group, o.info.tier))
        .collect();
    assert!(
        order.is_sorted(),
        "not group by group, the most used first: {order:?}"
    );
    let big: Vec<&str> = operations
        .iter()
        .filter(|o| o.info.tier == OperationTier::Big)
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
    // The part's own parameters and properties come first, in its menu.
    let json = serde_json::to_value(&operations[0]).unwrap();
    assert_eq!(json["group"], "Part");
    assert_eq!(json["tier"], "Menu");
    let sketch = operations.iter().find(|o| o.kind == "add_sketch").unwrap();
    let json = serde_json::to_value(sketch).unwrap();
    assert_eq!(json["group"], "Sketch");
    assert_eq!(json["tier"], "Big");
    let mirror = operations.iter().find(|o| o.kind == "mirror").unwrap();
    assert_eq!(
        serde_json::to_value(mirror).unwrap()["group"],
        "Pattern & bodies"
    );
}

/// The 3-D sketch `route`: lines from `(1, 1, 1)` to `(3, 1, 1)` and on to
/// `(3, 2, 2)` — its points `p0`, `p1` and `p3`, its lines `c2` and `c4`.
fn route() -> geop_ops_sketch3d::AddSketch3dArgs {
    let mut s = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(geop_ops_extrude_revolve::testing::n));
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
fn editing(program: Program) -> Editor {
    let mut editor = crate::editor();
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
    let args: geop_ops_sketch3d::AddSketch3dArgs = args(editor.program().steps.last().unwrap());
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
    let corner = element::<Vertex>("sketch3d(route,p1)");
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(3.004, 1.0),
            shift: false,
        },
    });
    let highlights = update.step.unwrap().presentation.highlights;
    assert!(highlights.contains(&corner.picked()), "{highlights:?}");
    editor.handle(click_at(from_above(3.004, 1.0)));
    editor.handle(click_at(from_above(2.0, 1.005)));
    editor.handle(dialog("construction", Value::Choice("parallel".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let datum: geop_ops_reference_geometry::AddDatumArgs =
        args(editor.program().steps.last().unwrap());
    assert_eq!(
        datum.selection,
        [
            Selected::Vertex(corner),
            Selected::Edge(element("sketch3d(route,c2)"))
        ]
    );
    let part = editor.program().build_model().unwrap();
    part.check_names().unwrap();
}

/// A boundary surface between a planar sketch's line and a 3-D sketch's
/// line, both clicked: the ruled face between them, a valid sheet.
#[test]
fn a_boundary_surface_spans_sketch_lines() {
    let mut base = geop_ops_sketch::Sketch::new();
    let (a, b) = (
        base.add_point(
            geop_ops_extrude_revolve::testing::n(0.0),
            geop_ops_extrude_revolve::testing::n(0.0),
        ),
        base.add_point(
            geop_ops_extrude_revolve::testing::n(2.0),
            geop_ops_extrude_revolve::testing::n(0.0),
        ),
    );
    let line = base.add_line(a, b);
    let mut program = Program::new();
    program.push(
        "base",
        geop_ops_sketch::AddSketchArgs {
            plane: Some(geop_ops_sketch::SketchPlane::Datum(
                geop_ops_datums::origin_plane(FrameAxis::Z),
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
    let args: geop_ops_surface::BoundarySurfaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.edges,
        [
            geop_ops_sketch3d::CurveSource::Sketch(geop_ops_sketch::sketch_element("base", line)),
            geop_ops_sketch3d::CurveSource::Edge(element("sketch3d(route,c2)"))
        ]
    );
    let part = editor.program().build_model().unwrap();
    assert!(part.face_id("boundary(boundary_surface1)").is_ok());
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// The horn swept with a 3-D sketch as its rail, clicked where it runs:
/// the rail is the whole sketch, and the horn builds as from the planar
/// one.
#[test]
fn a_sweep_takes_a_3d_sketch_as_its_rail() {
    let mut program = geop_ops_extrude_revolve::examples::horn();
    program.steps.truncate(2);
    let mut flare = geop_ops_sketch3d::Sketch3d::new();
    let points = [
        [0.0, 0.5, 0.0],
        [1.5, 0.4, 0.0],
        [3.0, 0.9, 0.0],
        [4.0, 1.6, 0.0],
    ]
    .map(|p| {
        flare.add_point(Vector3::from_array(
            p.map(geop_ops_extrude_revolve::testing::n),
        ))
    });
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
    let sketch = |name: &str| Value::Elements(vec![geop_ops_sketch::whole_sketch(name).picked()]);
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    editor.handle(dialog("rails", Value::Press));
    editor.handle(click_at(from_above(1.5, 0.4)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: geop_ops_extrude_revolve::SweepArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.rails,
        [geop_ops_extrude_revolve::PathRef::Sketch3d(
            geop_ops_sketch3d::sketch3d_path("flare")
        )]
    );
    let part = editor.program().build_model().unwrap();
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// Two lines of a 3-D sketch, `(x, y, z) = (s, 0.5, 0)` and `(s, 1.5, 1)`
/// for `s` from -0.5 to 2.5 — or, with `across`, lines
/// `(0, s, s - 0.5)` and `(2, s, s - 0.5)` crossing them, run downwards.
fn network_lines(across: bool) -> geop_ops_sketch3d::AddSketch3dArgs {
    let mut s = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(geop_ops_extrude_revolve::testing::n));
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
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::EDGE, drawn::SKETCH_CURVE]
    );
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
    let args: geop_ops_surface::NetworkSurfaceArgs = args(editor.program().steps.last().unwrap());
    let edge = |name: &str| geop_ops_sketch3d::CurveSource::Edge(element(name));
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
    let part = editor.program().build_model().unwrap();
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
    // Its corner where the first u curve picked meets the first v curve.
    let corner = part
        .vertex_id("network(uv_surface1,sketch3d(along,c5),sketch3d(across,c5))")
        .unwrap();
    let p = part.topology().get_vertex(corner).unwrap().point;
    assert!(p.could_be_equal(&Vector3::from_array([2.0, 1.5, 1.0].map(S::from_f64))));
}

/// Drawing a 3-D sketch on a part picks what the pointer is over in the
/// part as the editor drew it when the step was opened: no hover draws
/// the part again, however many there are, and a click only as the editor
/// shows the sketch it changed — once.
#[test]
fn a_3d_sketch_picks_without_drawing_the_part_again() {
    let (mut editor, _) = editor();
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
    let args: geop_ops_sketch3d::AddSketch3dArgs = args(step.unwrap());
    let points: Vec<[f64; 3]> = (args.sketch.points.values())
        .map(|p| p.at.to_array().map(|c| c.to_f64()))
        .collect();
    assert_eq!(points, [[0.5, 0.5, 1.0]]);
}

/// Moving the pointer over an assembly draws no part: neither the
/// assembly, which was drawn when it was built, nor the parts it places,
/// each drawn once where it is first placed.
#[test]
fn hovering_over_an_assembly_draws_no_part() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let hover = |editor: &mut Editor| {
        let drawn = geop_core_editor::view::parts_drawn();
        for k in 0..5 {
            let update = editor.handle(Command::Event {
                event: StepEditEvent::Hover {
                    pointer: from_above(0.1 * k as f64, 0.0),
                    shift: false,
                },
            });
            assert!(update.error.is_none(), "{:?}", update.error);
        }
        geop_core_editor::view::parts_drawn() - drawn
    };
    assert_eq!(hover(&mut editor), 0);
    // Nor while a part is placed, which picks in what its step builds.
    let update = editor.handle(Command::Open { id: "fore".into() });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(hover(&mut editor), 0);
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
    let args: geop_ops_sketch3d::AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.sketch.curves.len(), 1, "{:?}", args.sketch);
}

/// Editing the first sketch of a program builds only up to it, however
/// many steps follow: a point of the handle's outline dragged rebuilds the
/// sketch, not the sweep, hole and boolean after it, on every move of the
/// pointer. Put away, the whole program is built again.
#[test]
fn editing_an_early_sketch_builds_only_up_to_it() {
    let mut editor = example_editor("handle_with_hole");
    let steps = editor.program().steps.len();
    assert!(steps >= 3, "{steps}");
    handled(
        &mut editor,
        Command::Open {
            id: "outline".into(),
        },
    );
    let args: geop_ops_sketch::AddSketchArgs = args(editor.editing().unwrap());
    let corner = args
        .sketch
        .points
        .values()
        .find(|p| !p.fixed)
        .expect("a point of the outline")
        .xy();
    let corner = [corner[0].to_f64(), corner[1].to_f64()];
    let down = |[x, y]: [f64; 2]| from_above(x, y);
    editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down(corner),
            shift: false,
        },
    });
    for (k, d) in [0.05, 0.1, 0.15].into_iter().enumerate() {
        let update = handled(
            &mut editor,
            Command::Event {
                event: StepEditEvent::Drag {
                    from: down(corner),
                    to: down([corner[0] + d, corner[1] + d]),
                    done: k == 2,
                    shift: true,
                },
            },
        );
        // What follows the sketch does not run while it is edited.
        if let Some(program) = update.program {
            assert!(program.steps.iter().all(|s| !s.runs), "{:?}", program.steps);
        }
    }
    let dragged: geop_ops_sketch::AddSketchArgs =
        crate::registry::testing::args(editor.editing().unwrap());
    assert_ne!(dragged.sketch, args.sketch, "the drag changed nothing");
    let update = handled(&mut editor, Command::Commit);
    let program = update.program.expect("the program changed");
    assert_eq!(program.steps.len(), steps);
    assert!(
        program.steps.iter().all(|s| s.runs && s.error.is_none()),
        "{:?}",
        program.steps
    );
}

/// A file left and opened again takes up its steps as they were built:
/// the drilled box, then an assembly placing it, then the box again,
/// builds the box's steps once, not twice. Edited in between, it is built
/// again from the step that changed.
///
/// Only that each load builds is checked: how many versions a run built
/// was counted by `Part::built`, which geop-core-program no longer has.
#[test]
fn a_file_opened_again_is_not_built_again() {
    let part = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    let mut assembly = Program::new();
    assembly.push(
        "box",
        geop_ops_assembly::AddPartArgs {
            file: "part.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let mut editor = crate::editor();
    let files = [
        ("part.geop".to_string(), Some(part.to_json().unwrap())),
        (
            "assembly.geop".to_string(),
            Some(assembly.to_json().unwrap()),
        ),
    ]
    .into();
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let load = |editor: &mut Editor, program: &Program, path: &str| {
        let update = handled(
            editor,
            Command::Load {
                program: program.clone(),
                path: Some(path.into()),
            },
        );
        let steps = update.program.expect("the program is sent").steps;
        assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    };
    load(&mut editor, &part, "part.geop");
    load(&mut editor, &assembly, "assembly.geop");
    load(&mut editor, &part, "part.geop");

    // Its hole drilled deeper: from that step on.
    let mut deeper = part.clone();
    let last = deeper.steps.len() - 1;
    crate::registry::testing::edit_args(&mut deeper.steps[last], |hole: &mut ExtrudeArgs| {
        hole.extent = geop_ops_extrude_revolve::Extents::blind(-0.8);
    });
    assert_ne!(deeper.steps[last], part.steps[last], "the hole changed");
    load(&mut editor, &deeper, "part.geop");
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
        .find(|s| s.operation == "add_sketch")
        .expect("the example has a sketch")
        .clone();

    let copied = editor.handle(Command::Copy {
        id: sketch.id.clone(),
    });
    assert!(copied.error.is_none(), "{:?}", copied.error);
    let text = copied.export.unwrap().text().unwrap().to_string();

    editor.handle(Command::Seek { marker: Some(1) });
    let update = handled(&mut editor, Command::Paste { text: text.clone() });
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
    handled(&mut editor, Command::Paste { text });
    assert_eq!(editor.program().steps[0].id, sketch.id);

    let refused = editor.handle(Command::Paste {
        text: "not steps".into(),
    });
    assert!(refused.error.unwrap().contains("pasting"));
}

/// A program the editor is given is checked against its operations before
/// it is taken: one loaded or pasted with a step of an operation there is
/// none of, or with arguments its operation does not read, is refused,
/// naming the step, and the program edited stays as it was.
#[test]
fn a_program_of_unknown_steps_is_refused() {
    let mut editor = crate::editor();
    let mut unknown = Program::new();
    unknown.steps.push(ProgramStep {
        id: "mystery".into(),
        operation: "no_such_operation".into(),
        args: serde_json::json!({}),
    });
    let refused = editor.handle(Command::Load {
        program: unknown.clone(),
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(
        error.contains("step `mystery`") && error.contains("no_such_operation"),
        "{error}"
    );
    assert!(editor.program().steps.is_empty());

    let refused = editor.handle(Command::Paste {
        text: unknown.to_json().unwrap(),
    });
    assert!(refused.error.unwrap().contains("no_such_operation"));
    assert!(editor.program().steps.is_empty());

    let mut unreadable = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    unreadable.steps[0].args = serde_json::json!("not arguments");
    let refused = editor.handle(Command::Load {
        program: unreadable,
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(error.contains("arguments of operation"), "{error}");
    assert!(editor.program().steps.is_empty());
}

/// A gesture of the key and pointer grammar every tool shares.
#[derive(Clone, Copy, Debug)]
enum Gesture {
    Enter,
    SecondaryClick,
    Escape,
}

impl Gesture {
    fn send(self, editor: &mut Editor) {
        let pointer = from_above(0.0, 0.0);
        let sent = match self {
            Gesture::Enter => StepEditEvent::Key {
                key: "Enter".into(),
            },
            Gesture::Escape => StepEditEvent::Key {
                key: "Escape".into(),
            },
            Gesture::SecondaryClick => StepEditEvent::Click {
                pointer,
                button: Button::Secondary,
                double: false,
                shift: false,
            },
        };
        event(editor, sent);
    }
}

/// The curves of the sketch the step being edited is, once committed, as
/// text: what was drawn, for a failed count to say — not the axes a new
/// sketch starts with.
fn drawn_curves(editor: &mut Editor) -> Vec<String> {
    handled(editor, Command::Commit);
    let sketch: geop_ops_sketch::AddSketchArgs = args(editor.program().steps.last().unwrap());
    sketch
        .sketch
        .curves
        .values()
        .filter(|c| !c.construction)
        .map(|c| format!("{c:?}"))
        .collect()
}

/// The curves of the 3-D sketch drawn, committed, each as it reads.
fn drawn_curves_3d(editor: &mut Editor) -> Vec<String> {
    handled(editor, Command::Commit);
    let sketch: geop_ops_sketch3d::AddSketch3dArgs = args(editor.program().steps.last().unwrap());
    (sketch.sketch.curves.values())
        .filter(|c| c.is_drawn() && !c.construction)
        .map(|c| format!("{c:?}"))
        .collect()
}

/// Enter, a secondary click and Escape all end the chain being drawn: two
/// lines drawn after one, not a chain of three.
#[test]
fn every_drawing_tool_ends_its_chain_the_same_way() {
    for gesture in [Gesture::Enter, Gesture::SecondaryClick, Gesture::Escape] {
        // On the origin's xy plane, seen from above.
        let mut editor = crate::editor();
        editor.handle(Command::New {
            kind: "add_sketch".into(),
        });
        let click = |editor: &mut Editor, x: f64, y: f64| {
            editor.handle(click_at(from_above(x, y)));
        };
        click(&mut editor, 0.04, 0.04);
        editor.handle(dialog("tool", Value::Choice("line".into())));
        click(&mut editor, 0.0, 0.0);
        click(&mut editor, 1.0, 0.0);
        gesture.send(&mut editor);
        click(&mut editor, 0.0, 1.0);
        click(&mut editor, 1.0, 1.0);
        let curves = drawn_curves(&mut editor);
        assert_eq!(curves.len(), 2, "planar, {gesture:?}: {curves:#?}");
    }
}

/// The 3-D sketch ends its chain as the planar one does
/// ([`every_drawing_tool_ends_its_chain_the_same_way`]), its line tool
/// taken by its key in either case.
#[test]
fn every_3d_drawing_tool_ends_its_chain_the_same_way() {
    for gesture in [Gesture::Enter, Gesture::SecondaryClick, Gesture::Escape] {
        for key in ["l", "L"] {
            let mut editor = crate::editor();
            editor.handle(Command::New {
                kind: "add_sketch3d".into(),
            });
            event(&mut editor, StepEditEvent::Key { key: key.into() });
            let click = |editor: &mut Editor, x: f64, y: f64| {
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

/// A second Escape puts the drawing tool down: the next click draws nothing.
#[test]
fn a_second_escape_puts_the_drawing_tool_down() {
    let mut editor = crate::editor();
    editor.handle(Command::New {
        kind: "add_sketch".into(),
    });
    let click = |editor: &mut Editor, x: f64, y: f64| {
        editor.handle(click_at(from_above(x, y)));
    };
    click(&mut editor, 0.04, 0.04);
    editor.handle(dialog("tool", Value::Choice("line".into())));
    click(&mut editor, 0.0, 0.0);
    click(&mut editor, 1.0, 0.0);
    Gesture::Escape.send(&mut editor);
    Gesture::Escape.send(&mut editor);
    click(&mut editor, 0.0, 1.0);
    click(&mut editor, 1.0, 1.0);
    let curves = drawn_curves(&mut editor);
    assert_eq!(curves.len(), 1, "{curves:#?}");
}

/// A tool's key is the same typed with shift or caps lock: in the 3-D
/// sketch, `A` takes the arc tool as `a` does.
#[test]
fn a_tools_key_is_the_same_typed_in_capitals() {
    for key in ["a", "A"] {
        let mut editor = crate::editor();
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

/// A new fillet waits for its edges to be picked, as edges.
#[test]
fn new_fillet_picks_edges() {
    let mut editor = example_editor("box_with_drill_hole");
    for kind in ["fillet", "chamfer"] {
        let update = editor.handle(Command::New { kind: kind.into() });
        let step = update.step.expect("a step is edited");
        assert_eq!(step.missing, ["edges"]);
        assert_eq!(step.presentation.pickable, [drawn::EDGE]);
        editor.handle(Command::Cancel);
    }
}
