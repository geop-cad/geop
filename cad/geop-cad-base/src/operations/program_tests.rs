//! Programs of the editor's operations: edited, read back, and run.

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{Part, PartDescription, StepHandle};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::ExtrudeArgs;

use crate::{PartOperation, ProgramEdit, ProgramRunner, examples::box_with_drill_hole};

fn extrude(sketch: &str, distance: f64) -> PartOperation {
    ExtrudeArgs {
        sketch: sketch.into(),
        distance,
        symmetric: false,
        combine: Combine::NewBody,
    }
    .into()
}

/// Every kind of edit, addressed by id, and each rejected when it would
/// leave the program invalid — without changing anything.
#[test]
fn edits_change_the_program_by_id() {
    let mut program = box_with_drill_hole();
    let len = program.steps.len();

    let id = program
        .update(ProgramEdit::Insert {
            index: len,
            id: None,
            operation: extrude("outline", 2.0),
        })
        .unwrap();
    assert_eq!(id.as_deref(), Some("extrude1"));
    assert_eq!(program.steps[len].id, "extrude1");

    program
        .update(ProgramEdit::Update {
            id: "extrude1".into(),
            operation: extrude("outline", 3.0),
        })
        .unwrap();
    assert_eq!(program.steps[len].operation, extrude("outline", 3.0));

    program
        .update(ProgramEdit::Move {
            id: "extrude1".into(),
            index: 0,
        })
        .unwrap();
    assert_eq!(program.steps[0].id, "extrude1");

    program
        .update(ProgramEdit::Remove {
            id: "extrude1".into(),
        })
        .unwrap();
    assert_eq!(program, box_with_drill_hole());

    let before = program.clone();
    for bad in [
        ProgramEdit::Insert {
            index: len + 1,
            id: None,
            operation: extrude("outline", 1.0),
        },
        ProgramEdit::Insert {
            index: 0,
            id: Some("box".into()),
            operation: extrude("outline", 1.0),
        },
        ProgramEdit::Insert {
            index: 0,
            id: Some("not an id".into()),
            operation: extrude("outline", 1.0),
        },
        ProgramEdit::Remove { id: "nope".into() },
        ProgramEdit::Move {
            id: "box".into(),
            index: len,
        },
    ] {
        assert!(program.update(bad.clone()).is_err(), "{bad:?}");
        assert_eq!(program, before, "a rejected {bad:?} changed the program");
    }
}

/// An edit serializes as one flat JSON object, as an editor sends it.
#[test]
fn edits_read_from_json() {
    let edit: ProgramEdit = serde_json::from_str(
        r#"{"edit": "insert", "index": 0, "operation": "extrude",
            "args": {"sketch": "outline", "distance": 2.0}}"#,
    )
    .unwrap();
    assert_eq!(
        edit,
        ProgramEdit::Insert {
            index: 0,
            id: None,
            operation: extrude("outline", 2.0),
        }
    );
}

/// A runner builds what [`Program::apply`] builds; stopping early builds
/// just the steps before the stop; and a run after an edit starts from
/// the last unchanged step.
#[test]
fn runner_stops_early_and_reuses_the_unchanged_prefix() {
    let program = box_with_drill_hole();
    let describe = |part: &Part<S>| PartDescription::of(part).unwrap();
    let mut runner = ProgramRunner::<S>::new();

    runner.run(&program, None);
    assert!(runner.results().iter().all(|r| r.error.is_none()));
    assert_eq!(
        describe(runner.part()),
        describe(&program.apply(Part::new()).unwrap())
    );

    // Back in time: only the box.
    runner.run(&program, Some(2));
    assert_eq!(runner.results().len(), 2);
    let description = describe(runner.part());
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(box)"]
    );

    // An edit to the hole keeps the box's part: the first two steps are
    // served from the cache, which has to hold exactly what they built.
    let mut edited = program.clone();
    edited
        .update(ProgramEdit::Update {
            id: "hole".into(),
            operation: extrude("hole_sketch", -0.25),
        })
        .unwrap();
    runner.run(&edited, None);
    assert!(runner.results().iter().all(|r| r.error.is_none()));
    assert_eq!(
        describe(runner.part()),
        describe(&edited.apply(Part::new()).unwrap())
    );
}

/// Every step's handles, placed where the part as that step saw it puts
/// them: the box's distance at its top, the hole's at its bottom, and a
/// handle for every sketch point.
#[test]
fn runner_provides_every_handle() {
    use geop_ops::operation::{HandleGroup, HandleMotion};
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&box_with_drill_hole(), None);
    let handles = runner.handles().unwrap();
    let feature: Vec<&StepHandle> = handles
        .iter()
        .filter(|h| h.handle.group == HandleGroup::Feature)
        .collect();
    assert_eq!(feature.len(), 2);
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9);
    let (boxed, hole) = (feature[0], feature[1]);
    assert_eq!(boxed.step, "box");
    assert!(close(boxed.handle.position, [1.0, 1.0, 1.0]), "{boxed:?}");
    assert_eq!(hole.step, "hole");
    assert!(close(hole.handle.position, [1.0, 1.0, 0.5]), "{hole:?}");
    let HandleMotion::Linear {
        direction,
        arg,
        value,
        scale,
    } = &hole.handle.motion
    else {
        panic!("{hole:?}")
    };
    assert!(close(*direction, [0.0, 0.0, 1.0]));
    assert_eq!(arg, &["distance"]);
    assert_eq!((*value, *scale), (-0.5, 1.0));
    // 4 corners of the outline, the circle's center.
    let sketch = handles.len() - feature.len();
    assert_eq!(sketch, 5);
    let json = serde_json::to_value(&handles[0]).unwrap();
    assert_eq!(json["step"], "outline");
    assert_eq!(json["motion"], "planar");
}

/// A step that fails ends the run there, reported by id; the part is
/// what the steps before it built.
#[test]
fn runner_reports_the_failing_step() {
    let mut program = box_with_drill_hole();
    program
        .update(ProgramEdit::Update {
            id: "hole".into(),
            operation: ExtrudeArgs {
                sketch: "hole_sketch".into(),
                distance: -0.5,
                symmetric: false,
                combine: Combine::Difference {
                    target: "extrude(nothing)".into(),
                },
            }
            .into(),
        })
        .unwrap();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None);
    let last = runner.results().last().unwrap();
    assert_eq!(last.id, "hole");
    assert!(last.error.as_deref().unwrap().contains("extrude(nothing)"));
    assert!(runner.part().solid_id("extrude(box)").is_ok());
}

/// Every step the run covered has a dialog, made with the part before it —
/// the failing step's too, since a dialog is what helps fix it.
#[test]
fn runner_provides_every_dialog() {
    use geop_ops::operation::{ArgDialog, Role};
    let mut program = crate::examples::boss_on_reference_plane();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None);
    let dialogs = runner.dialogs();
    let steps: Vec<&str> = program.steps.iter().map(|s| s.id.as_str()).collect();
    let with_dialog: Vec<&str> = dialogs.iter().map(|d| d.step.as_str()).collect();
    assert_eq!(with_dialog, steps);
    let lifted = dialogs.iter().find(|d| d.step == "lifted").unwrap();
    assert_eq!(
        lifted.dialog.args["selection"],
        ArgDialog::Selection {
            roles: vec![vec![Role::Plane]]
        }
    );
    let ArgDialog::Options { fit } = &lifted.dialog.args["construction"] else {
        panic!("the constructions are options");
    };
    assert!(fit.contains(&"offset"));
    assert!(
        dialogs
            .iter()
            .all(|d| d.step == "lifted" || d.dialog.args.is_empty())
    );

    // Point the datum at a face that is not there: the step fails, and its
    // dialog says so.
    let index = program.index_of("lifted").unwrap();
    let PartOperation::AddDatum(args) = &mut program.steps[index].operation else {
        panic!("`lifted` adds a datum");
    };
    args.selection = vec![geop_ops::EntityRef::Face {
        name: "nowhere".into(),
    }];
    runner.run(&program, None);
    assert!(runner.results()[index].error.is_some());
    let dialogs = runner.dialogs();
    assert_eq!(dialogs.len(), index + 1);
    assert_eq!(
        dialogs[index].dialog.args["selection"],
        ArgDialog::Selection {
            roles: vec![vec![]]
        }
    );
}
