//! Parts placed as an editor places them: the step placing one edited
//! through the [`crate::Editor`], which solves the mates after every edit
//! with what the step pulls.

use geop_core_program::{
    element::{ElementReference, PickedElement},
    program::Program,
    values::Design,
};
use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Control, Pointer, StepEditEvent, Tone, Value},
    selectable::Selectable,
};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops_add_datum::Selected;
use geop_ops_assembly::{AddPartArgs, Kind, Mate, MateEntity, MateKind, placed_parts};
use geop_ops_datums::{ORIGIN, component_name, datums};
use geop_ops_extrude_revolve::Extents;
use geop_ops_model::Combine;
use geop_ops_model::testing::assert_close;

use crate::Command;
use crate::registry::testing::{Stepping, extrude_sketch, handled, on_origin, pointer, rectangle};

/// The unit cube from the origin to `(1, 1, 1)`, as a program.
fn cube() -> Program {
    let mut program = Program::new();
    let square = rectangle(0.0, 0.0, 1.0, 1.0);
    let (extent, combine) = (Extents::blind(1.0), Combine::NewBody);
    extrude_sketch(
        &mut program,
        "c",
        on_origin(FrameAxis::Z),
        square,
        extent,
        combine,
    );
    program
}

/// A step placing `cube.geop`, held fixed if `fixed`, with the mates
/// `mates`.
fn args(fixed: bool, mates: &[(&str, MateKind, [MateEntity; 2])]) -> AddPartArgs {
    let args = AddPartArgs {
        file: "cube.geop".into(),
        mates: (mates.iter())
            .map(|(id, kind, entities)| {
                let mate = Mate {
                    kind: *kind,
                    entities: entities.to_vec(),
                    joints: Vec::new(),
                };
                (id.to_string(), mate)
            })
            .collect(),
        ..Default::default()
    };
    if fixed { args.fixed() } else { args }
}

/// The datum `name` of the part placed as `instance`.
fn datum(instance: &str, name: String) -> MateEntity {
    MateEntity::of(
        instance,
        Selected::Datum(ElementReference::new(datums(), name)),
    )
}

/// The frame of the part placed as `instance`.
fn origin(instance: &str) -> MateEntity {
    datum(instance, ORIGIN.to_string())
}

/// The `xy` plane of the frame of the part placed as `instance`.
fn base(instance: &str) -> MateEntity {
    datum(
        instance,
        component_name(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
    )
}

fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// Where `pose` puts the point `p`.
fn apply(pose: &Pose<Design>, p: [f64; 3]) -> [f64; 3] {
    let q = pose.apply(&Vector3::from_array(p.map(n)));
    [0, 1, 2].map(|k| q[k].to_f64())
}

/// Looking straight down at `(x, y, z)`.
fn down_at(x: f64, y: f64, z: f64) -> Pointer<S> {
    pointer([x, y, z + 10.0], [0.0, 0.0, -1.0])
}

/// The cube placed as `a`, fixed at the origin, and then as `b` by
/// `placed`, in a workspace holding `cube.geop`: the step placing `b`
/// opened.
fn placing(placed: AddPartArgs) -> Stepping {
    let mut editor = crate::editor();
    let files = BTreeMap::from([("cube.geop".to_string(), Some(cube().to_json().unwrap()))]);
    handled(&mut editor, Command::Files { files });
    let mut program = Program::new();
    program.push("a", args(true, &[]));
    program.push("b", placed);
    handled(
        &mut editor,
        Command::Load {
            program,
            path: Some("assembly.geop".into()),
        },
    );
    let shown = handled(&mut editor, Command::Open { id: "b".into() })
        .step
        .expect("the step is edited");
    Stepping { editor, shown }
}

/// Where the part placed as `instance` is, as the editor last ran the
/// program.
fn moved_to(step: &Stepping, instance: &str) -> Pose<Design> {
    let reader = step.editor.part();
    let placed = placed_parts(&reader).unwrap();
    let placed = (placed.iter()).find(|p| p.name == instance);
    placed
        .unwrap_or_else(|| panic!("no part {instance}"))
        .located
        .world
}

/// Adding a mate in the editor arms its entities to be picked; once it has
/// both, picking is done.
#[test]
fn adding_a_mate_arms_its_entities() {
    let mut step = placing(args(false, &[]));
    step.dialog("add_mate", Value::Choice("coincident".into()));
    let Some(Control::Reference(entities)) = step.presentation().dialog.get("mate:m1:entities")
    else {
        panic!("the new mate's entities are a reference field");
    };
    assert!(entities.armed, "a new mate waits for its entities");

    let pair = vec![origin("b"), origin("a")];
    let picked: Vec<PickedElement> = pair.iter().map(Selectable::picked).collect();
    step.dialog("mate:m1:entities", Value::Elements(picked));
    assert!(
        step.presentation().dialog.get("mate:m1:entities").is_none(),
        "done picking"
    );
    assert_eq!(step.args::<AddPartArgs>().mates["m1"].entities, pair);
}

/// Typing where the part goes pulls it there, turned as typed.
#[test]
fn the_placement_fields_pull_the_part_there() {
    let mut step = placing(args(false, &[]));
    step.dialog("x", Value::Number(2.0));
    step.dialog("rotation_z", Value::Number(90.0));
    let pose = moved_to(&step, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, 0.0, 0.0], 1e-6);
    assert_close(apply(&pose, [1.0, 0.0, 0.0]), [2.0, 1.0, 0.0], 1e-6);
}

/// A fixed part goes where it is dragged: no mate moves it, so the solve
/// moves it alone.
#[test]
fn a_fixed_part_goes_where_it_is_dragged() {
    let mut step = placing(args(true, &[]));
    step.send(StepEditEvent::Hover {
        pointer: down_at(0.5, 0.5, 1.0),
        shift: false,
    });
    step.send(StepEditEvent::Drag {
        from: down_at(0.5, 0.5, 1.0),
        to: down_at(2.5, 0.0, 1.0),
        done: true,
        shift: false,
    });
    let pose = moved_to(&step, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, -0.5, 0.0], 1e-6);
}

/// Mates that cannot all hold are listed as such.
#[test]
fn mates_that_cannot_hold_are_said_so() {
    let distance = |value| MateKind::Constraint(Kind::Distance { value: n(value) });
    let mates = args(
        false,
        &[
            ("m1", distance(1.0), [base("b"), base("a")]),
            ("m2", distance(2.0), [base("b"), base("a")]),
        ],
    );
    let step = placing(mates);
    let Some(Control::List { items, .. }) = step.presentation().dialog.get("mates") else {
        panic!("the mates are a list");
    };
    assert!(items.iter().any(|i| i.tone == Tone::Error), "{items:?}");
}
