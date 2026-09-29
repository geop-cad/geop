//! Programs of the editor's operations: edited, read back, and run.

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{EntityRef, Part, PartDescription};
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

/// The runner says what the steps it built build on — what an editor
/// hides once it is used.
#[test]
fn runner_reports_what_steps_build_on() {
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&box_with_drill_hole(), None);
    let references = runner.references();
    for sketch in ["outline", "hole_sketch"] {
        assert!(references.contains(&EntityRef::Sketch {
            name: sketch.into()
        }));
    }
    assert!(references.contains(&EntityRef::Face {
        name: "extrude(box,end)".into()
    }));
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
