//! Programs of the editor's operations, run.

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{Part, PartDescription};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::ExtrudeArgs;

use crate::{PartOperation, ProgramRunner, examples::box_with_drill_hole};

fn extrude(sketch: &str, distance: f64) -> PartOperation {
    ExtrudeArgs {
        sketch: sketch.into(),
        distance,
        symmetric: false,
        combine: Combine::NewBody,
    }
    .into()
}

/// A runner builds what [`Program::build`] builds; stopping early builds
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
        describe(&program.build().unwrap())
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
    let hole = edited.index_of("hole").unwrap();
    edited.steps[hole].operation = extrude("hole_sketch", -0.25);
    runner.run(&edited, None);
    assert!(runner.results().iter().all(|r| r.error.is_none()));
    assert_eq!(
        describe(runner.part()),
        describe(&edited.build().unwrap())
    );
}

/// A step that fails ends the run there, reported by id; the part is
/// what the steps before it built.
#[test]
fn runner_reports_the_failing_step() {
    let mut program = box_with_drill_hole();
    let hole = program.index_of("hole").unwrap();
    program.steps[hole].operation = ExtrudeArgs {
        sketch: "hole_sketch".into(),
        distance: -0.5,
        symmetric: false,
        combine: Combine::Difference {
            target: "extrude(nothing)".into(),
        },
    }
    .into();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None);
    let last = runner.results().last().unwrap();
    assert_eq!(last.id, "hole");
    assert!(last.error.as_deref().unwrap().contains("extrude(nothing)"));
    assert!(runner.part().solid_id("extrude(box)").is_ok());
}
