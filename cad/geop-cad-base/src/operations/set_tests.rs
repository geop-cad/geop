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
            "add_sketch3d",
            "add_datum",
            "extrude",
            "revolve",
            "sweep",
            "loft",
            "hole",
            "thread",
            "rib",
            "fillet",
            "chamfer",
            "shell",
            "draft",
            "lip",
            "groove",
            "boolean",
            "split",
            "linear_pattern",
            "circular_pattern",
            "mirror",
            "move_body",
            "delete_body",
            "boundary_surface",
            "offset_surface",
            "thicken",
            "knit",
            "trim_surface",
            "extend_surface",
            "extract_face",
            "project_curve",
            "base_flange",
            "edge_flange",
            "sheet_cut",
            "hem",
            "flat_pattern",
            "subd",
            "add_part",
            "part_pattern",
            "route",
            "drawing",
            "import_step"
        ]
    );
    assert_eq!(infos[0].label, "Sketch");
    assert!(
        infos[3].doc.starts_with("Sweep a sketch"),
        "{}",
        infos[3].doc
    );
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

/// Every operation's dialog opens, the way the toolbar opens it, on an
/// empty part and on a drilled box, and is cancelled again: none may fail
/// for want of something to work on — in the browser a panic leaves the
/// editor unusable — and each answers with a step to edit or says why not.
#[test]
fn every_operation_opens_on_any_part() {
    use crate::{Command, Editor};
    for program in [None, Some(examples::box_with_drill_hole())] {
        let mut editor = Editor::<S>::new();
        if let Some(program) = program {
            let update = editor.handle(Command::Load {
                program,
                path: None,
            });
            assert!(update.error.is_none(), "{:?}", update.error);
        }
        for info in PartOperation::infos() {
            let update = editor.handle(Command::New {
                kind: info.kind.to_string(),
            });
            assert!(
                update.step.is_some() || update.error.is_some(),
                "{}: neither a step nor a reason",
                info.kind
            );
            editor.handle(Command::Cancel);
        }
    }
}
