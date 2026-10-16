//! The steps of parameters edited as the front end edits them: a table's
//! dialog, its input, columns, rows and values set field by field.

use geop_core_editor::{
    editing::{Control, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{built, dialog, editor_with, handled},
};
use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    program::Program,
};

use crate::{LookupTableArgs, parameters::Row, steps::LOOKUP};

/// An editor of what parameters offer, on an empty program.
fn editor() -> Editor {
    let registry = crate::register(Registry::default()).expect("one name each");
    editor_with(registry, Program::new())
}

/// The keys and labels of the items of the list `key` of the step edited.
fn items(editor: &mut Editor, key: &str) -> Vec<(String, String)> {
    let step = editor.handle(Command::Show).step.expect("a step is edited");
    let Some(Control::List { items, .. }) = step.presentation.dialog.get(key) else {
        panic!("no list {key}: {:?}", step.presentation.dialog);
    };
    (items.iter())
        .map(|item| (item.key.clone(), item.label.clone()))
        .collect()
}

/// A table is made in its dialog as under Parameters: the input it is
/// looked up by named, a column added and named, a row added, named and
/// selected, each value set. Committed, the part is built in the row
/// selected, its columns read as `input.column`; the last row is not taken
/// out, and taking out the row selected selects the first left.
#[test]
fn a_table_is_made_in_its_dialog() {
    let mut editor = editor();
    handled(
        &mut editor,
        Command::New {
            kind: LOOKUP.into(),
        },
    );
    handled(&mut editor, dialog("input", Value::Text("size".into())));
    handled(&mut editor, dialog("column:0", Value::Text("d".into())));
    handled(&mut editor, dialog("add", Value::Choice("column".into())));
    handled(&mut editor, dialog("column:1", Value::Text("k".into())));
    handled(&mut editor, dialog("row:0", Value::Text("M4".into())));
    handled(&mut editor, dialog("add", Value::Choice("row".into())));
    handled(&mut editor, dialog("row:1", Value::Text("M5".into())));
    for (cell, value) in [("0:0", 4.0), ("0:1", 2.8), ("1:0", 5.0), ("1:1", 3.5)] {
        handled(
            &mut editor,
            dialog(&format!("cell:{cell}"), Value::Number(value)),
        );
    }
    handled(&mut editor, dialog("row:1", Value::Press));
    let cells = items(&mut editor, "values");
    assert_eq!(cells[3], ("cell:1:1".to_string(), "M5: k".to_string()));
    handled(&mut editor, Command::Commit);

    let step = editor.program().steps.last().unwrap().clone();
    let args: LookupTableArgs = serde_json::from_value(step.args).unwrap();
    let row = |name: &str, values: &[f64]| Row {
        name: name.into(),
        values: values.to_vec(),
    };
    assert_eq!(
        args,
        LookupTableArgs {
            input: "size".into(),
            columns: vec!["d".into(), "k".into()],
            rows: vec![row("M4", &[4.0, 2.8]), row("M5", &[5.0, 3.5])],
            selected: "M5".into(),
        }
    );
    let part = built(&editor).unwrap();
    let values = read_parameter_values(&part.current()).unwrap();
    let value = |name: &str| {
        values
            .0
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.clone())
    };
    assert_eq!(value("size"), Some(ParameterValue::Text("M5".into())));
    assert_eq!(value("size.k"), Some(ParameterValue::Number(3.5)));

    handled(&mut editor, Command::Open { id: step.id });
    handled(&mut editor, dialog("row:1", Value::Remove));
    handled(&mut editor, dialog("row:0", Value::Remove));
    handled(&mut editor, dialog("column:0", Value::Remove));
    let rows = items(&mut editor, "rows");
    assert_eq!(rows, [("row:0".to_string(), "Row".to_string())]);
    handled(&mut editor, Command::Commit);
    let args: LookupTableArgs =
        serde_json::from_value(editor.program().steps.last().unwrap().args.clone()).unwrap();
    assert_eq!(args.selected, "M4");
    assert_eq!(args.columns, ["k"]);
    assert_eq!(args.rows, [row("M4", &[2.8])]);
}
