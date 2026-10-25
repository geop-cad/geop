//! [`EditForm`]: a [`Form`] built together with what setting each of its
//! fields does — for an operation with many fields, whose events are then
//! answered by the setter of the field they set (see [`EditForm::set`])
//! rather than by matching their keys. Every setter gets the whole
//! [`Edit`]: a field may set the step's arguments or its session.
//!
//! An operation only ever hands out the form ([`EditForm::into_form`]);
//! the setters stay its own, built again for each event. A setter may capture what the form was built from — the part
//! as the step sees it — for what setting a field implies for others:
//! another sketch picked brings its own axis, a selection picks the
//! construction it fits.

use geop_core_program::{numeric::NumericInput, reader::Reader};

use crate::{
    editing::{Action, CanvasEvent, Choice, Event, Form, Layout, Number, Table, TableEdit, Value},
    selectable::{Selectable, selected},
};

/// What a setter edits: the step's arguments, its session, and the keys of
/// the visuals selected.
pub struct Edit<'e, A, T> {
    pub args: &'e mut A,
    pub session: &'e mut T,
    pub selection: &'e mut Vec<String>,
}

impl<A, T> Edit<'_, A, T> {
    /// The same edit, to hand to one more setter.
    pub fn again(&mut self) -> Edit<'_, A, T> {
        Edit {
            args: &mut *self.args,
            session: &mut *self.session,
            selection: &mut *self.selection,
        }
    }
}

/// What a step edited through an [`EditForm`] keeps while it is edited, as
/// its editing state ([`crate::editable::Editable::EditingState`]): its
/// session `T`, and the keys of the visuals selected.
#[derive(Clone, Debug, Default)]
pub struct FormState<T> {
    pub session: T,
    pub selection: Vec<String>,
}

impl<T: Clone> FormState<T> {
    /// The arguments `args` and this state after `event`: a field set by
    /// its setter in the form `form` makes of them as they were, the
    /// selection replaced, and what the pointer or a key did passed to
    /// `canvas`.
    pub fn handle<'f, A: Clone>(
        &self,
        args: &A,
        event: Event,
        form: impl FnOnce(&A, &Self) -> EditForm<'f, A, T>,
        canvas: impl FnOnce(Edit<'_, A, T>, &CanvasEvent),
    ) -> (A, Self) {
        let mut edited = args.clone();
        let mut state = self.clone();
        let edit = Edit {
            args: &mut edited,
            session: &mut state.session,
            selection: &mut state.selection,
        };
        match event {
            Event::Field { key, value } => form(args, self).set(&key, edit, value),
            Event::Select(keys) => *edit.selection = keys,
            Event::Canvas(event) => canvas(edit, &event),
        }
        (edited, state)
    }
}

/// What setting a field does.
type Setter<'a, A, T> = Box<dyn Fn(Edit<'_, A, T>, Value) + 'a>;

/// A [`Form`] and what setting each of its fields does, for the arguments
/// `A` and the session `T`.
pub struct EditForm<'a, A, T = ()> {
    pub form: Form,
    setters: Vec<(String, Setter<'a, A, T>)>,
}

impl<A, T> Default for EditForm<'_, A, T> {
    fn default() -> Self {
        Self {
            form: Form::default(),
            setters: Vec::new(),
        }
    }
}

impl<'a, A, T> EditForm<'a, A, T> {
    /// The form, as an editor is shown it.
    pub fn into_form(self) -> Form {
        self.form
    }

    /// What the field `key` does with `value`: its setter's. Nothing for a
    /// key no field has — a field shown only to be read.
    pub fn set(&self, key: &str, edit: Edit<'_, A, T>, value: Value) {
        if let Some((_, setter)) = self.setters.iter().find(|(k, _)| k == key) {
            setter(edit, value);
        }
    }

    /// `setter` for the key `key`: for a control already in the dialog, or
    /// one of its items — a list's entries each have their own key.
    pub fn on(&mut self, key: impl Into<String>, setter: impl Fn(Edit<'_, A, T>, Value) + 'a) {
        self.setters.push((key.into(), Box::new(setter)));
    }

    /// Shown as `layout` says (see [`Layout`]).
    pub fn layout(&mut self, layout: Layout) -> &mut Self {
        self.form.layout = layout;
        self
    }

    /// Text typed — a name, a title — `set` to what is typed (see
    /// [`crate::editing::Dialog::input`]).
    pub fn input(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        placeholder: Option<&str>,
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.input(key, label, value, placeholder);
        self.on(key, move |edit, value| {
            if let Value::Text(text) = value {
                set(edit, &text);
            }
        });
        self
    }

    /// A table edited in place (see [`Table`]): `edit` is run for each
    /// thing done to it. A block pasted is taken apart here, into a row
    /// added for each line below the last and the cells set, in reading
    /// order — so `edit` never sees a [`TableEdit::Paste`], and a cell set
    /// in a row just added finds the row there.
    pub fn table(
        &mut self,
        key: &str,
        table: Table,
        edit: impl Fn(Edit<'_, A, T>, TableEdit) + 'a,
    ) -> &mut Self {
        let (rows, columns) = (table.rows.len(), table.columns.len());
        self.form.dialog.table(key, table);
        self.on(key, move |mut e, value| {
            let Value::Table(what) = value else {
                return;
            };
            let TableEdit::Paste { row, column, cells } = what else {
                return edit(e, what);
            };
            let mut rows = rows;
            for (r, line) in cells.into_iter().enumerate() {
                let row = row + r;
                while row >= rows {
                    edit(e.again(), TableEdit::AddRow);
                    rows += 1;
                }
                for (c, text) in line.into_iter().enumerate() {
                    let column = column + c;
                    if column < columns {
                        edit(e.again(), TableEdit::Cell { row, column, text });
                    }
                }
            }
        });
        self
    }

    /// Things to do or choose; pressing one runs `run` with its value.
    pub fn actions(
        &mut self,
        key: &str,
        actions: Vec<Action>,
        run: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.actions(key, actions);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                run(edit, &choice);
            }
        });
        self
    }

    pub fn checkbox(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: bool,
        set: impl Fn(Edit<'_, A, T>, bool) + 'a,
    ) -> &mut Self {
        self.form.dialog.checkbox(key, label, value);
        self.on(key, move |edit, value| {
            if let Value::Bool(b) = value {
                set(edit, b);
            }
        });
        self
    }

    pub fn number(
        &mut self,
        key: &str,
        number: Number,
        set: impl Fn(Edit<'_, A, T>, f64) + 'a,
    ) -> &mut Self {
        self.form.dialog.number(key, number);
        self.on(key, move |edit, value| {
            if let Value::Number(v) = value {
                set(edit, v);
            }
        });
        self
    }

    /// A number field of a [`NumericInput`] (see [`Number::numeric`]):
    /// text typed sets it — plain if it is a number, a formula otherwise —
    /// and a number slid or dragged sets it plain. A formula that does not
    /// evaluate is kept, as typed, its field saying why: the step then fails
    /// naming it, and the text is there to be corrected.
    pub fn numeric(
        &mut self,
        key: &str,
        number: Number,
        set: impl Fn(Edit<'_, A, T>, NumericInput) + 'a,
    ) -> &mut Self {
        self.form.dialog.number(key, number);
        self.on(key, move |edit, value| {
            if let Some(input) = value.numeric_input() {
                set(edit, input);
            }
        });
        self
    }

    /// One of `options`, by value — found by typing, if `searchable`.
    pub fn select(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        options: Vec<Choice>,
        searchable: bool,
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        (self.form.dialog).select(key, label, value, options, searchable);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit, &choice);
            }
        });
        self
    }

    /// A file the program reads (see [`crate::editing::Dialog::file`]).
    pub fn file(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        files: Vec<String>,
        accept: &[&str],
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.file(key, label, value, files, accept);
        self.on(key, move |edit, value| {
            if let Value::Choice(choice) = value {
                set(edit, &choice);
            }
        });
        self
    }

    /// A colour, `#rrggbb`.
    pub fn color(
        &mut self,
        key: &str,
        label: impl Into<String>,
        value: impl Into<String>,
        set: impl Fn(Edit<'_, A, T>, &str) + 'a,
    ) -> &mut Self {
        self.form.dialog.color(key, label, value);
        self.on(key, move |edit, value| {
            if let Value::Text(color) = value {
                set(edit, &color);
            }
        });
        self
    }

    /// Values picked, holding `value`, each selected by a click as `P`
    /// says in the part `reader` reads (see [`Selectable`]): `set` gets what
    /// the field holds after a pick — the values held, and the one a click
    /// selected, or without the one it took out.
    pub fn pick<P: Selectable>(
        &mut self,
        key: &str,
        label: impl Into<String>,
        reader: &Reader<'a>,
        value: &[P],
        multiple: bool,
        set: impl Fn(Edit<'_, A, T>, Vec<P>) + 'a,
    ) -> &mut Self {
        (self.form.dialog).pick(key, label, reader, value, multiple);
        let reader = reader.clone();
        self.on(key, move |edit, value| {
            if let Value::Elements(picks) = value {
                set(edit, selected(&reader, &picks));
            }
        });
        self
    }

    /// One value picked, holding `value` if there is one (see
    /// [`EditForm::pick`]): `set` gets the value a click picks. Emptying
    /// the field sets nothing: what a step needs one of is never none.
    pub fn pick_one<P: Selectable>(
        &mut self,
        key: &str,
        label: impl Into<String>,
        reader: &Reader<'a>,
        value: Option<&P>,
        set: impl Fn(Edit<'_, A, T>, P) + 'a,
    ) -> &mut Self {
        let value = value.map(std::slice::from_ref).unwrap_or_default();
        self.pick(key, label, reader, value, false, move |edit, mut picked| {
            if let Some(one) = picked.pop() {
                set(edit, one);
            }
        })
    }
}

/// A select field's table of options, each `(option, value, label)`: the
/// value it gives `chosen`, and the choices it offers.
pub fn choices<T: PartialEq>(options: &[(T, &str, &str)], chosen: &T) -> (String, Vec<Choice>) {
    let value = (options.iter().find(|o| o.0 == *chosen)).map_or("", |o| o.1);
    let choices = (options.iter()).map(|&(_, v, l)| Choice::new(v, l));
    (value.to_string(), choices.collect())
}

/// The option of a select field's table `options` (see [`choices`]) that
/// has the value `value`.
pub fn chosen<T: Copy>(options: &[(T, &str, &str)], value: &str) -> Option<T> {
    options.iter().find(|o| o.1 == value).map(|o| o.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::editing::{Cell, Column, ColumnKind, TableRow};

    /// A block pasted into a table reaches the operation as the rows added
    /// below the last and the cells set, in reading order, and never as a
    /// paste; what runs past the last column is dropped.
    #[test]
    fn a_paste_is_taken_apart_into_rows_added_and_cells_set() {
        let columns = vec![
            Column::new("a", ColumnKind::Text),
            Column::new("b", ColumnKind::Text),
        ];
        let rows = vec![TableRow::new(vec![Cell::new(""), Cell::new("")])];
        let table = Table::new(columns, rows, "");
        let mut form = EditForm::<Vec<TableEdit>>::default();
        form.table("t", table, |edit, what| edit.args.push(what));
        let mut seen = Vec::new();
        let mut state = FormState::<()>::default();
        let edit = Edit {
            args: &mut seen,
            session: &mut state.session,
            selection: &mut state.selection,
        };
        let paste = TableEdit::Paste {
            row: 0,
            column: 1,
            cells: vec![
                vec!["x".into(), "over".into()],
                vec!["y".into()],
                vec!["z".into()],
            ],
        };
        form.set("t", edit, Value::Table(paste));
        let cell = |row, column, text: &str| TableEdit::Cell {
            row,
            column,
            text: text.into(),
        };
        assert_eq!(
            seen,
            [
                cell(0, 1, "x"),
                TableEdit::AddRow,
                cell(1, 1, "y"),
                TableEdit::AddRow,
                cell(2, 1, "z"),
            ]
        );
    }
}
