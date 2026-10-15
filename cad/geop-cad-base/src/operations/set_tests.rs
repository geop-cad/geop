//! The editor's operations as one set: offered, and started from what the
//! steps before built.

use geop_ops_extrude_revolve::{ExtrudeArgs, Output};
use std::collections::HashSet;

use geop_ops_booleans::Combine;
use geop_ops_sketch::whole_sketch;

use crate::{
    examples,
    registry::{
        registry, runner,
        testing::{Built, args, handled},
    },
};

/// Every operation is offered once, registered under the kind its steps
/// serialize with, and every step of every example is of one of them.
#[test]
fn every_operation_is_offered() {
    let registry = registry().unwrap();
    let infos = registry.infos();
    let operations = registry.operations();
    let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
    assert_eq!(kinds.iter().collect::<HashSet<_>>().len(), kinds.len());
    assert_eq!(kinds[0], "add_sketch");
    assert_eq!(infos[0].info.label, "Sketch");
    assert_eq!(kinds[1], "add_datum");
    let extrude = infos.iter().find(|i| i.kind == "extrude").unwrap();
    assert!(
        extrude.info.doc.starts_with("Sweep a sketch"),
        "{}",
        extrude.info.doc
    );
    for info in &infos {
        assert_eq!(operations.get(info.kind).unwrap().kind(), info.kind);
    }
    for (_, program) in examples::all() {
        for step in &program().steps {
            assert!(
                kinds.contains(&step.operation.as_str()),
                "{}",
                step.operation
            );
        }
    }
}

/// A new step starts from what the steps before it built.
#[test]
fn new_steps_start_from_what_was_built() {
    let program = examples::box_with_drill_hole();
    let part = program.built().unwrap();
    let registry = registry().unwrap();
    let editors = registry.editors();
    let runner = runner();
    let new_step = |kind: &str| editors.new_step(&runner, &part, &program, kind, "new", None);
    let step = new_step("extrude").unwrap();
    let extrude: ExtrudeArgs = args(&step);
    assert_eq!(extrude.sketch, whole_sketch("hole_sketch"));
    assert_eq!(
        extrude.output,
        Output::Solid(Combine::join("extrude(hole)"))
    );
    for info in registry.infos() {
        let step = new_step(info.kind).unwrap();
        assert_eq!(step.operation, info.kind);
    }
    assert!(new_step("no_such_operation").is_err());
}

/// Every operation's dialog opens, the way the toolbar opens it, on an
/// empty part and on a drilled box, and is cancelled again: none may fail
/// for want of something to work on — in the browser a panic leaves the
/// editor unusable — and each answers with a step to edit or says why not.
#[test]
fn every_operation_opens_on_any_part() {
    use crate::Command;
    for program in [None, Some(examples::box_with_drill_hole())] {
        let mut editor = crate::editor();
        if let Some(program) = program {
            handled(
                &mut editor,
                Command::Load {
                    program,
                    path: None,
                },
            );
        }
        for info in crate::registry().unwrap().infos() {
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
