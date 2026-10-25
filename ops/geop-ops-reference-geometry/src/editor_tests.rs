//! Datums added as an editor adds them: every gesture a command to the
//! [`geop_core_editor::editor::Editor`] — picks, the gizmo, handles — and
//! the step's dialog as it says what a selection fits; on the origin, on
//! the drilled box and on a 3-D sketch.

use geop_core_editor::{
    editing::{
        Axis, Button, Control, GizmoPart, Pointer, Presentation, Reach, Shape, StepEditEvent, Tone,
        Value,
    },
    editor::Command,
    registry::Registry,
    testing::{Stepping, built, click_at, dialog, editor_with, from_above, pointer},
};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::Program, testing::args};
use geop_ops_model::datums::{AnyDatum, Datums, ORIGIN, component_name, datums};
use geop_ops_extrude_revolve::{examples::box_with_drill_hole, testing::drilled_box};
use geop_ops_model::{Vertex, element, model, testing::model_of};
use geop_ops_sketch3d::testing::route;

use crate::{AddDatumArgs, CONSTRUCTIONS, Construction, Selected};

/// What these tests edit: sketches, planar and in space, the drilled box's
/// extrusions, and datums.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(crate::register)
        .and_then(geop_ops_sketch3d::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

fn datum(name: impl Into<String>) -> Selected {
    Selected::Datum(ElementReference::<Datums, AnyDatum>::new(datums(), name))
}

fn origin() -> Selected {
    datum(ORIGIN)
}

fn base(normal: FrameAxis) -> Selected {
    datum(component_name(ORIGIN, DatumComponent::Plane(normal)))
}

fn axis(along: FrameAxis) -> Selected {
    datum(component_name(ORIGIN, DatumComponent::Axis(along)))
}

fn v(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// A program of the datum step `d` alone, made by `args`.
fn program(args: &AddDatumArgs) -> Program {
    let mut program = Program::new();
    program.push("d", args.clone());
    program
}

/// The datum step `d` of `args` edited by `events` — a new one, starting
/// with a pick, if `new` — and what is shown after them.
fn edited(
    args: AddDatumArgs,
    new: bool,
    events: &[StepEditEvent<S>],
) -> (AddDatumArgs, Presentation<S>) {
    let mut step = match new {
        true => Stepping::new(editor_with(registry(), Program::new()), "add_datum"),
        false => Stepping::open(editor_with(registry(), program(&args)), "d"),
    };
    if new {
        assert_eq!(step.args::<AddDatumArgs>(), args, "a new step starts so");
    }
    for event in events {
        step.send(event.clone());
    }
    (step.args(), step.presentation().clone())
}

/// An offset point is moved by a gizmo at it, along the axes its
/// offsets are measured along: its `z` arrow dragged edits `z`.
#[test]
fn offset_points_are_dragged_by_a_gizmo() {
    let args = AddDatumArgs {
        selection: vec![origin()],
        construction: Construction::Point {
            x: 1.0.into(),
            y: 2.0.into(),
            z: 3.0.into(),
        },
    };
    let shown = edited(args.clone(), false, &[]).1;
    assert!(shown.visuals.is_empty(), "no handles: {:?}", shown.visuals);
    let gizmo = shown.gizmo.expect("a gizmo");
    assert!(
        gizmo.at.could_be_equal(&v([1.0, 2.0, 3.0])),
        "{:?}",
        gizmo.at
    );
    assert!(gizmo.modes.translate && !gizmo.modes.rotate);
    // Seen from the side, its `z` arrow dragged half a unit up — the grid
    // the pointer's reach snaps to being a tenth.
    let side = |z: f64| Pointer {
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
        ..pointer([1.0, -10.0, z], [0.0, 1.0, 0.0])
    };
    let hover = StepEditEvent::Hover {
        pointer: side(3.05),
        shift: false,
    };
    let hovered = edited(args.clone(), false, std::slice::from_ref(&hover)).1;
    assert_eq!(
        hovered.gizmo.and_then(|g| g.hover),
        Some(GizmoPart::Move(Axis::Z))
    );
    assert!(hovered.grab);
    let drag = StepEditEvent::Drag {
        from: side(3.05),
        to: side(3.57),
        done: true,
        shift: false,
    };
    let (dragged, _) = edited(args, false, &[drag]);
    assert_eq!(
        dragged.construction,
        Construction::Point {
            x: 1.0.into(),
            y: 2.0.into(),
            z: 3.5.into()
        }
    );
}

/// The dialog says what each selected element can be used as, and offers
/// only the constructions that fit the selection — for a step that does
/// not build too.
#[test]
fn dialogs_say_what_a_selection_fits() {
    let nowhere = Selected::Edge(ElementReference::new(model(), "nowhere"));
    let args = AddDatumArgs {
        selection: vec![origin(), nowhere],
        construction: Construction::Offset {
            distance: 1.0.into(),
        },
    };
    let step = Stepping::open(editor_with(registry(), program(&args)), "d");
    assert!(step.shown.error.is_some(), "the step does not build");
    let dialog = &step.presentation().dialog;
    let Some(Control::Reference(selection)) = dialog.get("selection") else {
        panic!("the selection is a reference field");
    };
    assert_eq!(selection.value[0].detail.as_deref(), Some("point"));
    assert_eq!(selection.value[1].tone, Tone::Error);
    assert!(dialog.get("construction_needs").is_some());

    let args = AddDatumArgs {
        selection: vec![base(FrameAxis::Z)],
        ..args
    };
    let dialog = edited(args, false, &[]).1.dialog;
    let Some(Control::Actions { actions }) = dialog.get("construction") else {
        panic!("the constructions are actions");
    };
    let enabled = |method: &str| actions.iter().find(|a| a.value == method).unwrap().enabled;
    assert!(enabled("offset"));
    assert!(!enabled("midpoint"));
}

/// Picking in the viewport adds to the selection, and again takes out;
/// the construction follows what fits.
#[test]
fn picks_build_the_selection() {
    let args = AddDatumArgs {
        selection: Vec::new(),
        construction: Construction::default_of(&CONSTRUCTIONS[0]),
    };
    // The origin's ball, from above.
    let click = StepEditEvent::Click {
        pointer: from_above(0.0, 0.0),
        button: Button::Primary,
        double: false,
        shift: false,
    };
    let (once, shown) = edited(args.clone(), true, std::slice::from_ref(&click));
    assert_eq!(once.selection, [origin()]);
    assert!(
        shown.pickable.iter().any(|kind| kind == "datum"),
        "{:?}",
        shown.pickable
    );
    let (twice, _) = edited(args, true, &[click.clone(), click]);
    assert!(twice.selection.is_empty());
}

/// Elements are taken out of the selection in its field, one or all — by
/// the editor, as for any reference field; the construction follows.
#[test]
fn selections_are_edited_in_their_field() {
    let args = AddDatumArgs {
        selection: vec![origin(), axis(FrameAxis::X)],
        construction: Construction::Perpendicular {},
    };
    let field = |value| StepEditEvent::Dialog {
        key: "selection".into(),
        value,
    };
    let (removed, _) = edited(args.clone(), false, &[field(Value::RemoveAt(0))]);
    assert_eq!(removed.selection, [axis(FrameAxis::X)]);
    assert_eq!(removed.construction.schema().method, "along_line");
    let (cleared, _) = edited(args, false, &[field(Value::Clear)]);
    assert!(cleared.selection.is_empty());
}

/// A datum a step adds is listed in the part's structure, after the
/// origin, to be shown or hidden like it.
#[test]
fn datums_added_are_listed_in_the_structure() {
    let mut program = drilled_box();
    let top = Selected::Face(ElementReference::new(model(), "extrude(box,end)"));
    let lifted = AddDatumArgs {
        selection: vec![top],
        construction: Construction::Offset {
            distance: 1.0.into(),
        },
    };
    program.push("lifted", lifted);
    let editor = editor_with(registry(), program);
    let part = built(&editor).unwrap();
    let structure = (editor.registry())
        .structure(&part.current(), &["lifted".into()])
        .unwrap();
    let datums: Vec<(&str, Option<bool>)> = (structure.iter())
        .filter(|item| item.kind == "datum")
        .map(|item| (item.name.as_str(), item.visible))
        .collect();
    assert_eq!(datums, [(ORIGIN, Some(true)), ("lifted", Some(false))]);
}

/// An offset plane's handle, on the plane, drags its distance: grabbed
/// from the side and pulled up by 0.3, the plane half a unit above the
/// box's top goes to 0.8.
#[test]
fn offset_plane_dragged_by_its_handle() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
    let mut program = editor.program().clone();
    program.push(
        "plane",
        crate::AddDatumArgs {
            selection: vec![Selected::Face(element("extrude(box,end)"))],
            construction: crate::Construction::Offset {
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
    let args: crate::AddDatumArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.construction,
        crate::Construction::Offset {
            distance: 0.8.into()
        }
    );
}

/// A new offset plane, made from the box's top, is dragged by its handle
/// while it is still being made: picked, offset, grabbed, pulled up 0.3.
#[test]
fn new_offset_plane_dragged_by_its_handle() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
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

/// What a 3-D sketch builds is picked like any edge or vertex — before
/// they were edges and vertices, nothing of a 3-D sketch could be picked
/// but the whole sketch, as a path. A datum's selection takes the sketch's
/// corner, lit when hovered, and then its line; the datum builds on them.
#[test]
fn a_3d_sketch_point_and_line_are_picked() {
    let mut program = Program::new();
    program.push("route", route());
    let mut editor = editor_with(registry(), program);
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
    let datum: crate::AddDatumArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        datum.selection,
        [
            Selected::Vertex(corner),
            Selected::Edge(element("sketch3d(route,c2)"))
        ]
    );
    let part = model_of(&built(&editor).unwrap());
    part.check_names().unwrap();
}
