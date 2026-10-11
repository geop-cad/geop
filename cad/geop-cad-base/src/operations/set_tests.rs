//! The editor's operations as one set: offered, and started from what the
//! steps before built.

use geop_ops_extrude_revolve::Output;
use std::collections::HashSet;

use geop_ops_extrude_revolve::ExtrudeArgs;

use crate::{
    examples,
    operations::{
        infos, registry,
        testing::{Built, args, handled, join},
    },
};

/// Every operation is offered once, registered under the kind its steps
/// serialize with, and every step of every example is of one of them.
#[test]
fn every_operation_is_offered() {
    let registry = registry().unwrap();
    let infos = infos();
    let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
    assert_eq!(kinds.iter().collect::<HashSet<_>>().len(), kinds.len());
    assert_eq!(kinds[0], "add_sketch");
    assert_eq!(infos[0].label, "Sketch");
    assert_eq!(kinds[1], "add_datum");
    let extrude = infos.iter().find(|i| i.kind == "extrude").unwrap();
    assert!(extrude.doc.starts_with("Sweep a sketch"), "{}", extrude.doc);
    for info in &infos {
        assert_eq!(registry.get(info.kind).unwrap().info(), *info);
    }
    for (_, program) in examples::all() {
        for step in &program().program.steps {
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
    let runner = examples::box_with_drill_hole().built().unwrap();
    let step = runner.new_step("extrude", "new", None).unwrap();
    let extrude: ExtrudeArgs = args(&step);
    assert_eq!(extrude.sketch, "hole_sketch");
    assert_eq!(extrude.output, Output::Solid(join("extrude(hole)")));
    for info in infos() {
        let step = runner.new_step(info.kind, "new", None).unwrap();
        assert_eq!(step.operation, info.kind);
    }
    assert!(runner.new_step("no_such_operation", "new", None).is_err());
}

/// Every operation's dialog opens, the way the toolbar opens it, on an
/// empty part and on a drilled box, and is cancelled again: none may fail
/// for want of something to work on — in the browser a panic leaves the
/// editor unusable — and each answers with a step to edit or says why not.
#[test]
fn every_operation_opens_on_any_part() {
    use crate::{Command, Editor};
    for program in [None, Some(examples::box_with_drill_hole())] {
        let mut editor = Editor::new();
        if let Some(program) = program {
            handled(
                &mut editor,
                Command::Load {
                    program,
                    path: None,
                },
            );
        }
        for info in infos() {
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
