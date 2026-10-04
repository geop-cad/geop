//! The editor's operations as one set: offered, and started from what the
//! steps before built.

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{NoFiles, Operations};
use geop_ops_booleans::Combine;

use crate::{PartOperation, ProgramRunner, examples};

/// Every operation is offered, under the kind its steps serialize with.
#[test]
fn every_operation_is_offered() {
    let infos = PartOperation::infos();
    let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
    assert_eq!(
        kinds,
        [
            "add_sketch",
            "extrude",
            "revolve",
            "sweep",
            "loft",
            "boolean",
            "split",
            "delete_body",
            "extract_face",
            "project_curve",
            "fillet",
            "chamfer",
            "shell",
            "add_datum",
            "add_part",
            "linear_pattern",
            "circular_pattern",
            "mirror",
            "move_body",
            "route",
            "hole",
            "thread",
            "boundary_surface",
            "offset_surface",
            "thicken",
            "knit",
            "trim_surface",
            "extend_surface",
            "add_sketch3d",
            "part_pattern"
        ]
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
    runner.run(&examples::box_with_drill_hole(), None, &NoFiles);
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
    assert!(PartOperation::new_step("no_such_operation", part).is_err());
}
