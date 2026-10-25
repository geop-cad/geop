//! The steps of parameters edited as the front end edits them: a table's
//! dialog, in a popup, its input, columns, rows and cells set through the
//! one table field.

use geop_core_editor::{
    editing::{Control, Layout, Table, TableEdit, Tone, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{Stepping, built, dialog, editor_with, handled},
};
use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    program::Program,
};

use crate::parameters::{
    DefineParametersArgs, LookupTableArgs, NumberParameter, Row,
    steps::{LOOKUP, PARAMETERS},
};

/// An editor of what parameters offer, on an empty program.
fn editor() -> Editor {
    let registry = crate::register(Registry::default()).expect("one name each");
    editor_with(registry, Program::new())
}

/// The table field `key` of the step edited.
fn table(stepping: &Stepping, key: &str) -> Table {
    match stepping.presentation().dialog.get(key) {
        Some(Control::Table(table)) => table.clone(),
        other => panic!("no table {key}: {other:?}"),
    }
}

/// The texts of the table's cells, row by row.
fn cells(table: &Table) -> Vec<Vec<String>> {
    (table.rows.iter())
        .map(|row| row.cells.iter().map(|c| c.text.clone()).collect())
        .collect()
}

/// The command editing the table field `key`.
fn edit(key: &str, what: TableEdit) -> Command {
    dialog(key, Value::Table(what))
}

/// A table is made in its dialog, a popup: the input it is looked up by
/// typed, a column added and named in its header, a row added, named in
/// its first cell and chosen, each cell typed. Committed, the part is
/// built in the row chosen, its columns read as `input.column`; the last
/// row is not taken out, and taking out the row chosen chooses the first
/// left.
#[test]
fn a_table_is_made_in_its_popup_dialog() {
    let mut s = Stepping::new(editor(), LOOKUP);
    assert_eq!(s.presentation().layout, Layout::Popup);
    let shown = table(&s, "table");
    assert!(shown.selectable && shown.columns_editable && shown.rows_editable);
    assert_eq!(cells(&shown), [["A", "1"]]);
    assert!(!shown.columns[0].editable && shown.columns[1].editable);
    assert!(!shown.rows[0].removable, "the last row stays");

    s.dialog("input", Value::Text("size".into()));
    s.dialog(
        "table",
        Value::Table(TableEdit::Column {
            column: 1,
            text: "d".into(),
        }),
    );
    s.dialog("table", Value::Table(TableEdit::AddColumn));
    s.dialog(
        "table",
        Value::Table(TableEdit::Column {
            column: 2,
            text: "k".into(),
        }),
    );
    s.dialog("table", Value::Table(TableEdit::AddRow));
    for (row, column, text) in [
        (0, 0, "M4"),
        (0, 1, "4"),
        (0, 2, "2.8"),
        (1, 0, "M5"),
        (1, 1, "5"),
        (1, 2, "3.5"),
    ] {
        s.dialog(
            "table",
            Value::Table(TableEdit::Cell {
                row,
                column,
                text: text.into(),
            }),
        );
    }
    s.dialog("table", Value::Table(TableEdit::SelectRow { row: 1 }));
    let shown = table(&s, "table");
    assert_eq!(cells(&shown), [["M4", "4", "2.8"], ["M5", "5", "3.5"]]);
    assert_eq!(
        shown
            .columns
            .iter()
            .map(|c| c.label.as_str())
            .collect::<Vec<_>>(),
        ["Row", "d", "k"]
    );
    assert!(!shown.rows[0].selected && shown.rows[1].selected);
    assert!(shown.rows.iter().all(|r| r.removable));

    let Stepping { mut editor, .. } = s;
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
    handled(&mut editor, edit("table", TableEdit::RemoveRow { row: 1 }));
    handled(&mut editor, edit("table", TableEdit::RemoveRow { row: 0 }));
    handled(
        &mut editor,
        edit("table", TableEdit::RemoveColumn { column: 1 }),
    );
    handled(&mut editor, Command::Commit);
    let args: LookupTableArgs =
        serde_json::from_value(editor.program().steps.last().unwrap().args.clone()).unwrap();
    assert_eq!(args.selected, "M4");
    assert_eq!(args.columns, ["k"]);
    assert_eq!(args.rows, [row("M4", &[2.8])]);
}

/// A block pasted from a spreadsheet fills the cells from where it is
/// pasted on, adds rows for the lines that run over the last, and drops
/// what runs past the last column; a row moved takes its place among the
/// others, and the first column's header and the step's own column are not
/// the user's to change.
#[test]
fn a_table_takes_a_pasted_block_and_moves_rows() {
    let mut s = Stepping::new(editor(), LOOKUP);
    let block = vec![
        vec!["M3".to_string(), "3".to_string(), "dropped".to_string()],
        vec!["M4".to_string(), "4".to_string()],
        vec!["M5".to_string(), "5".to_string()],
    ];
    s.dialog(
        "table",
        Value::Table(TableEdit::Paste {
            row: 0,
            column: 0,
            cells: block,
        }),
    );
    assert_eq!(
        cells(&table(&s, "table")),
        [["M3", "3"], ["M4", "4"], ["M5", "5"]]
    );
    s.dialog("table", Value::Table(TableEdit::MoveRow { row: 2, to: 0 }));
    assert_eq!(
        cells(&table(&s, "table")),
        [["M5", "5"], ["M3", "3"], ["M4", "4"]]
    );
    s.dialog(
        "table",
        Value::Table(TableEdit::Column {
            column: 0,
            text: "Name".into(),
        }),
    );
    s.dialog("table", Value::Table(TableEdit::RemoveColumn { column: 0 }));
    let shown = table(&s, "table");
    assert_eq!(shown.columns[0].label, "Row");
    assert_eq!(shown.columns.len(), 2);
    let args: LookupTableArgs = s.args();
    assert_eq!(
        args.selected, "M3",
        "the row chosen stays chosen under its new name"
    );
    assert_eq!(args.rows[1].name, "M3");
}

/// The parameters are a table too, in a popup: a row each of name, value,
/// and what a slider offers; what a value comes to is said beside it, and
/// why a formula does not.
#[test]
fn the_parameters_are_edited_as_a_table() {
    let mut s = Stepping::new(editor(), PARAMETERS);
    assert_eq!(s.presentation().layout, Layout::Popup);
    assert_eq!(table(&s, "parameters").rows.len(), 0);
    s.dialog("parameters", Value::Table(TableEdit::AddRow));
    s.dialog("parameters", Value::Table(TableEdit::AddRow));
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 0,
            column: 0,
            text: "width".into(),
        }),
    );
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 0,
            column: 1,
            text: "20".into(),
        }),
    );
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 0,
            column: 2,
            text: "5".into(),
        }),
    );
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 0,
            column: 3,
            text: "50".into(),
        }),
    );
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 1,
            column: 0,
            text: "half".into(),
        }),
    );
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 1,
            column: 1,
            text: "width / 2".into(),
        }),
    );
    let shown = table(&s, "parameters");
    assert_eq!(
        cells(&shown),
        [["width", "20", "5", "50"], ["half", "width / 2", "", ""]]
    );
    assert_eq!(shown.rows[1].cells[1].detail.as_deref(), Some("= 10"));
    assert_eq!(shown.rows[1].cells[1].tone, Tone::Hint);
    s.dialog(
        "parameters",
        Value::Table(TableEdit::Cell {
            row: 1,
            column: 1,
            text: "nothing * 2".into(),
        }),
    );
    let shown = table(&s, "parameters");
    assert_eq!(shown.rows[1].cells[1].tone, Tone::Error);
    assert!(
        shown.rows[1].cells[1]
            .detail
            .as_deref()
            .is_some_and(|d| d.contains("nothing")),
        "{shown:?}"
    );
    s.dialog("parameters", Value::Table(TableEdit::RemoveRow { row: 1 }));
    let args: DefineParametersArgs = s.args();
    assert_eq!(
        args.values,
        [NumberParameter {
            name: "width".into(),
            expression: "20".into(),
            min: Some(5.0),
            max: Some(50.0),
        }]
    );
}
