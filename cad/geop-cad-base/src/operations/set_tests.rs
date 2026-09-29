//! The editor's operations as one set: offered, started and edited through
//! it, sessions and all as JSON.

use geop_core_math::{
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    EditContext, Operations,
    ui::{DialogValue, Event, PartView, Shape},
};
use geop_ops_booleans::Combine;

use crate::{PartOperation, ProgramRunner, examples};

/// Every operation is offered, under the kind its steps serialize with.
#[test]
fn every_operation_is_offered() {
    let infos = PartOperation::infos();
    let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
    assert_eq!(
        kinds,
        ["add_sketch", "extrude", "revolve", "boolean", "add_datum"]
    );
    assert_eq!(infos[0].label, "Sketch");
    assert!(infos[1].doc.starts_with("Sweep a sketch"));
    for (_, program) in examples::all() {
        for step in &program.steps {
            let json = serde_json::to_value(&step.operation).unwrap();
            assert_eq!(json["operation"], step.operation.kind());
            assert!(kinds.contains(&step.operation.kind()));
        }
    }
}

/// A new step starts from what the steps before it built.
#[test]
fn new_steps_start_from_what_was_built() {
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&examples::box_with_drill_hole(), None);
    let part = runner.part();
    let PartOperation::Extrude(extrude) = PartOperation::new_step("extrude", part).unwrap() else {
        panic!("an extrude");
    };
    assert_eq!(extrude.sketch, "hole_sketch");
    assert_eq!(
        extrude.combine,
        Combine::Union {
            target: "extrude(hole)".into()
        }
    );
    for info in PartOperation::infos() {
        let step = PartOperation::new_step(info.kind, part).unwrap();
        assert_eq!(step.kind(), info.kind);
    }
    assert!(PartOperation::new_step("fillet", part).is_err());
}

/// Editing a step through the set: its distance as a handle, and a new
/// value from the dialog — which cuts, being negative.
#[test]
fn steps_are_edited_through_the_set() {
    let program = examples::box_with_drill_hole();
    let index = program.index_of("hole").unwrap();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, Some(index));
    let part = runner.part();
    let view = PartView::of(part).unwrap();
    let ctx = EditContext { part, view: &view };
    let step = &program.steps[index].operation;

    let shown = step.edit(&ctx, serde_json::Value::Null, None);
    let handle = shown
        .presentation
        .visuals
        .iter()
        .find(|v| v.key == "distance")
        .expect("the distance is a handle");
    let Shape::Handle {
        at,
        direction: Some(direction),
    } = handle.shape
    else {
        panic!("{handle:?}");
    };
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    assert!(at.could_be_equal(&v([1.0, 1.0, 0.5])), "{at:?}");
    assert!(direction.could_be_equal(&v([0.0, 0.0, 1.0])));

    let edited = shown.args.edit(
        &ctx,
        shown.session,
        Some(&Event::Dialog {
            key: "distance".into(),
            value: DialogValue::Number(-0.25),
        }),
    );
    let PartOperation::Extrude(args) = edited.args else {
        panic!("still an extrude");
    };
    assert_eq!(args.distance, -0.25);
    assert!(matches!(args.combine, Combine::Difference { .. }));
}
