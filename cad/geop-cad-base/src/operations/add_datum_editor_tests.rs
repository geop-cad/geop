//! Datums added as an editor adds them: every gesture a command to the
//! [`crate::Editor`], and the step's dialog as it says what a selection
//! fits.

use geop_core_editor::editing::{
    Axis, Button, Control, GizmoPart, Pointer, Presentation, Reach, StepEditEvent, Tone, Value,
};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::Program};
use geop_ops_add_datum::{AddDatumArgs, CONSTRUCTIONS, Construction, Selected};
use geop_ops_datums::{AnyDatum, Datums, ORIGIN, component_name, datums};
use geop_ops_extrude_revolve::testing::drilled_box;
use geop_ops_model::model;

use super::testing::{Built, Stepping, from_above, pointer};
use crate::part::structure;

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
        true => Stepping::new(Program::new(), "add_datum"),
        false => Stepping::open(program(&args), "d"),
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
    let step = Stepping::open(program(&args), "d");
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
    let part = program.built().unwrap();
    let structure = structure(&part.current(), &["lifted".into()]).unwrap();
    let datums: Vec<(&str, Option<bool>)> = (structure.iter())
        .filter(|item| item.kind == "datum")
        .map(|item| (item.name.as_str(), item.visible))
        .collect();
    assert_eq!(datums, [(ORIGIN, Some(true)), ("lifted", Some(false))]);
}
