//! The editor of everything, as the front end drives it: commands that
//! edit the program, and only what changed sent back; every operation
//! offered, group by group; every example loaded by name; a program of
//! steps no operation reads refused.

use geop_core_editor::editing::{StepEditEvent, Value};
use geop_core_program::program::{Program, ProgramStep};
use geop_ops_extrude_revolve::ExtrudeArgs;

use crate::{
    Command, Editor, Update,
    registry::testing::{args, dialog, handled, pointer},
};

fn editor() -> (Editor, Update) {
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadExample {
            name: "box_with_drill_hole".into(),
        },
    );
    (editor, update)
}

fn distance(editor: &Editor, id: &str) -> f64 {
    let program = editor.program();
    let args: ExtrudeArgs = args(&program.steps[program.index_of(id).unwrap()]);
    match &args.extent.side1 {
        geop_ops_extrude_revolve::Extent::Blind(d) => d.plain().expect("a plain distance"),
        other => panic!("{other:?} is no distance"),
    }
}

/// A new step goes in where the program runs to, once it builds; an
/// existing one is edited in place, moved and removed by id — and every
/// change can be undone and redone.
#[test]
fn commands_edit_the_program() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();

    let update = editor.handle(Command::New {
        kind: "extrude".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Extrude");
    assert_eq!(step.id, None);
    let update = handled(&mut editor, Command::Commit);
    assert!(update.step.is_none());
    let steps = update.program.expect("the program changed").steps;
    assert_eq!(steps.last().unwrap().id, "extrude1");

    editor.handle(Command::Open {
        id: "extrude1".into(),
    });
    editor.handle(dialog("distance", Value::Number(2.0)));
    // Not in the program until committed.
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    editor.handle(Command::Commit);
    assert_eq!(distance(&editor, "extrude1"), 2.0);

    editor.handle(Command::Move {
        id: "extrude1".into(),
        index: 0,
    });
    assert_eq!(editor.program().steps[0].id, "extrude1");
    editor.handle(Command::Remove {
        id: "extrude1".into(),
    });
    assert_eq!(*editor.program(), before);

    for _ in 0..3 {
        editor.handle(Command::Undo);
    }
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    let update = editor.handle(Command::Redo);
    assert!(update.program.unwrap().can_redo);
    assert_eq!(distance(&editor, "extrude1"), 2.0);
}

/// A command that would break the program is refused, saying why, and
/// changes nothing.
#[test]
fn bad_commands_are_refused() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let mut twice = before.clone();
    twice.steps.push(twice.steps[0].clone());
    for bad in [
        Command::Remove { id: "nope".into() },
        Command::Move {
            id: "box".into(),
            index: before.steps.len(),
        },
        Command::Load {
            program: twice,
            path: None,
        },
        Command::Commit,
        Command::New {
            kind: "no_such_operation".into(),
        },
    ] {
        let update = editor.handle(bad);
        assert!(update.error.is_some());
        assert_eq!(*editor.program(), before);
    }

    // A step that does not build cannot go in: without the box, the hole
    // has no face to be sketched on.
    let update = editor.handle(Command::Remove { id: "box".into() });
    assert!(update.error.is_none());
    let update = editor.handle(Command::Open {
        id: "hole_sketch".into(),
    });
    assert!(update.step.unwrap().error.is_some());
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_some());
}

/// The part is sent again only when what is drawn changed, the program only
/// when it or its run did — a hover sends neither.
#[test]
fn only_what_changed_is_sent() {
    let (mut editor, update) = editor();
    assert!(update.program.is_some() && update.scene.is_some());
    let update = editor.handle(Command::Open { id: "hole".into() });
    assert!(update.program.is_some(), "the step is marked as edited");
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: pointer([5.0, 5.0, 5.0], [0.0, 0.0, -1.0]),
            shift: false,
        },
    });
    assert!(update.program.is_none() && update.scene.is_none());
    assert!(update.step.is_some());
    let update = editor.handle(Command::Preview { preview: false });
    assert!(update.scene.is_some(), "the part before the step is drawn");
    let update = editor.handle(Command::Show);
    assert!(update.program.is_some() && update.scene.is_some());
}

/// Every example loads by name, and builds.
#[test]
fn examples_load_by_name() {
    let mut editor = crate::editor();
    let names = editor.handle(Command::Show).program.unwrap().examples;
    assert_eq!(
        names.len(),
        crate::registry().unwrap().example_programs().len()
    );
    let update = editor.handle(Command::LoadExample {
        name: names[0].into(),
    });
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    assert_ne!(*editor.program(), Program::new());
}

/// The toolbar's sections are the editor's: it sends the operations group
/// by group, in the groups' order, every operation in one, and within a
/// group the few used most (shown big) first, then the common ones (small),
/// then those only in the group's menu.
#[test]
fn operations_are_offered_by_group() {
    use geop_core_editor::editable::OperationTier;
    let (_, update) = editor();
    let operations = update.program.expect("the program is sent").operations;
    assert_eq!(operations.len(), crate::registry().unwrap().infos().len());
    let order: Vec<_> = operations
        .iter()
        .map(|o| (o.info.group, o.info.tier))
        .collect();
    assert!(
        order.is_sorted(),
        "not group by group, the most used first: {order:?}"
    );
    let big: Vec<&str> = operations
        .iter()
        .filter(|o| o.info.tier == OperationTier::Big)
        .map(|o| o.kind)
        .collect();
    assert_eq!(
        big,
        [
            "add_sketch",
            "extrude",
            "revolve",
            "hole",
            "fillet",
            "boolean",
            "linear_pattern",
            "subd",
            "base_flange",
            "add_part",
            "drawing",
        ]
    );
    // The part's own parameters and properties come first, in its menu.
    let json = serde_json::to_value(&operations[0]).unwrap();
    assert_eq!(json["group"], "Part");
    assert_eq!(json["tier"], "Menu");
    let sketch = operations.iter().find(|o| o.kind == "add_sketch").unwrap();
    let json = serde_json::to_value(sketch).unwrap();
    assert_eq!(json["group"], "Sketch");
    assert_eq!(json["tier"], "Big");
    let mirror = operations.iter().find(|o| o.kind == "mirror").unwrap();
    assert_eq!(
        serde_json::to_value(mirror).unwrap()["group"],
        "Pattern & bodies"
    );
}

/// A program the editor is given is checked against its operations before
/// it is taken: one loaded or pasted with a step of an operation there is
/// none of, or with arguments its operation does not read, is refused,
/// naming the step, and the program edited stays as it was.
#[test]
fn a_program_of_unknown_steps_is_refused() {
    let mut editor = crate::editor();
    let mut unknown = Program::new();
    unknown.steps.push(ProgramStep {
        id: "mystery".into(),
        operation: "no_such_operation".into(),
        args: serde_json::json!({}),
    });
    let refused = editor.handle(Command::Load {
        program: unknown.clone(),
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(
        error.contains("step `mystery`") && error.contains("no_such_operation"),
        "{error}"
    );
    assert!(editor.program().steps.is_empty());

    let refused = editor.handle(Command::Paste {
        text: unknown.to_json().unwrap(),
    });
    assert!(refused.error.unwrap().contains("no_such_operation"));
    assert!(editor.program().steps.is_empty());

    let mut unreadable = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    unreadable.steps[0].args = serde_json::json!("not arguments");
    let refused = editor.handle(Command::Load {
        program: unreadable,
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(error.contains("arguments of operation"), "{error}");
    assert!(editor.program().steps.is_empty());
}
