//! The steps a program says what its part is by, besides its shape: its
//! parameters ([`DefineParameters`]), the tables a part family looks its
//! size up in ([`LookupTable`]) and its properties ([`PartProperties`]).
//! Each produces targets as any step does: a formula reads a parameter's
//! target, and what lists the part reads its properties'. Every step that
//! defines parameters adds them to the part's [`ParameterValues`], a chain.
//!
//! What a parameter is set to where the part is built is the run's input
//! of its name — a [`Setting`] a program placing the part gives — and its
//! own definition where none is given. The colour is set the same way, by
//! the input [`COLOR`].

use std::borrow::Cow;

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Cell, Column, ColumnKind, Layout, Table, TableEdit, TableRow, Tone, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    numeric::{
        ParameterValue, ParameterValues, evaluate, names_in, parameter, parameter_values,
        read_parameter, read_parameter_values,
    },
    part::StepContext,
    reader::Reader,
    target::{Target, TargetReference},
    values::Setting,
};
use serde::{Deserialize, Serialize};

use crate::parameters::definition::{
    COLOR, Material, NumberParameter, Row, validate_color, validate_name,
};

/// The kind of the step defining a program's parameters, and its id.
pub const PARAMETERS: &str = "parameters";

/// The kind of the step defining a part's properties, and its id.
pub const PROPERTIES: &str = "properties";

/// The kind of the step looking a part family's size up in a table.
pub const LOOKUP: &str = "lookup_table";

/// Defines the program's number parameters, each a target of its own
/// ([`parameter`]): what the run's input of its name sets it to, else what
/// it is defined as. A formula may read any other parameter of the step,
/// defined before it or after, and those the steps before defined — a
/// table's columns; one reading one that reads it back fails, saying so.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DefineParameters;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DefineParametersArgs {
    #[serde(default)]
    pub values: Vec<NumberParameter>,
}

part_operation!(DefineParameters(DefineParametersArgs));

impl PartOperation for DefineParameters {
    const KIND: &'static str = PARAMETERS;
    const INFO: OperationInfo = OperationInfo::new(
        "Parameters",
        "The numbers the design is given by, which a program placing the part may set.",
        OperationGroup::Part,
        OperationTier::Menu,
    )
    .icon(crate::parameters::icons::PARAMETERS);
    type Args = DefineParametersArgs;
    type Session = ();

    fn new_args(_: &Context) -> DefineParametersArgs {
        DefineParametersArgs::default()
    }

    /// Each parameter, in an order in which a formula's parameters come
    /// before it, and then the step's entries of the part's parameter
    /// values.
    fn run(ctx: &mut StepContext, args: &DefineParametersArgs) -> GeopResult<()> {
        let step = ctx.step();
        for (i, p) in args.values.iter().enumerate() {
            validate_name(&p.name).map_err(|e| e.with_context(format!("parameters({step})")))?;
            if args.values[..i].iter().any(|q| q.name == p.name) {
                return Err(GeopError::new(format!(
                    "there is more than one parameter {:?}",
                    p.name
                )));
            }
        }
        for p in in_order(&args.values) {
            ctx.produce(&parameter(&p.name), p, value_of)?;
        }
        let names: Vec<String> = args.values.iter().map(|p| p.name.clone()).collect();
        ctx.produce(&parameter_values(), &names, add_values)
    }

    /// The parameters as a table, in a popup: a row each, of its name, its
    /// value — a number or a formula, and what it comes to in the part as
    /// the step built it — and what a slider offers, from and to. A row
    /// added is a fresh `length` of 10. A name typed here renames the
    /// definition alone; the Parameters panel renames what reads it too.
    fn form<'a>(
        ctx: &'a Context<'a>,
        args: &DefineParametersArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, DefineParametersArgs> {
        let mut f = EditForm::<DefineParametersArgs>::default();
        f.layout(Layout::Popup);
        let reader = ctx.built.as_ref().unwrap_or(&ctx.before);
        let columns = vec![
            Column::new("Name", ColumnKind::Text),
            Column::new("Value", ColumnKind::Formula),
            Column::new("Slider from", ColumnKind::Number { unit: Unit::Length }),
            Column::new("to", ColumnKind::Number { unit: Unit::Length }),
        ];
        let optional = |n: Option<f64>| Cell::new(n.map(|n| n.to_string()).unwrap_or_default());
        let rows = (args.values.iter())
            .map(|p| {
                let value = match evaluate(&p.expression, |name| read_parameter(reader, name)) {
                    Ok(value) => Cell::new(&p.expression).detail(format!("= {value}"), Tone::Hint),
                    Err(e) => Cell::new(&p.expression).detail(e.root_message(), Tone::Error),
                };
                TableRow::new(vec![
                    Cell::new(&p.name),
                    value,
                    optional(p.min),
                    optional(p.max),
                ])
            })
            .collect();
        let table = Table::new(columns, rows, "No parameters yet: add a row.");
        f.table("parameters", table, |edit, what| {
            let values = &mut edit.args.values;
            match what {
                TableEdit::Cell { row, column, text } => {
                    let Some(p) = values.get_mut(row) else {
                        return;
                    };
                    match column {
                        0 => p.name = text.trim().to_string(),
                        1 => p.expression = text.trim().to_string(),
                        2 => p.min = text.trim().parse().ok(),
                        3 => p.max = text.trim().parse().ok(),
                        _ => {}
                    }
                }
                TableEdit::AddRow => {
                    let names: Vec<String> = values.iter().map(|p| p.name.clone()).collect();
                    values.push(NumberParameter {
                        name: fresh_name(&names, "length"),
                        expression: "10".into(),
                        min: None,
                        max: None,
                    });
                }
                TableEdit::RemoveRow { row } => {
                    if row < values.len() {
                        values.remove(row);
                    }
                }
                TableEdit::MoveRow { row, to } => move_row(values, row, to),
                // No row is chosen, and the columns are the step's.
                TableEdit::SelectRow { .. }
                | TableEdit::Column { .. }
                | TableEdit::AddColumn
                | TableEdit::RemoveColumn { .. }
                | TableEdit::Paste { .. } => {}
            }
        });
        f
    }
}

/// The row `row` of `rows` moved to be the row `to`; nothing, for a row
/// there is none of.
fn move_row<R>(rows: &mut Vec<R>, row: usize, to: usize) {
    if row < rows.len() {
        let moved = rows.remove(row);
        rows.insert(to.min(rows.len()), moved);
    }
}

/// The part's parameter values as the steps before left them, and the
/// parameters `names` as they come to — a parameter that failed left out.
/// The make of a step's entries of the [`ParameterValues`] chain.
#[allow(clippy::ptr_arg, reason = "a make takes its arguments as produced")]
fn add_values(reader: &Reader, names: &Vec<String>) -> GeopResult<ParameterValues> {
    let mut values: ParameterValues = reader.previous()?;
    values.0.extend((names.iter()).filter_map(|name| {
        let value = reader.retrieve_element(&ElementReference::whole(parameter(name)));
        Some((name.clone(), value.ok()?.clone()))
    }));
    Ok(values)
}

/// `values` in an order in which a number comes after every parameter of
/// the step its formula names: the numbers as their formulas' names are
/// defined, then — reading what is not there, or reading each other — the
/// rest, which fail.
fn in_order(values: &[NumberParameter]) -> Vec<&NumberParameter> {
    let mut ordered: Vec<&NumberParameter> = Vec::with_capacity(values.len());
    let mut left: Vec<&NumberParameter> = values.iter().collect();
    loop {
        let ready = left.iter().position(|p| {
            names_in(&p.expression).iter().all(|name| {
                ordered.iter().any(|q| q.name == *name) || !values.iter().any(|q| q.name == *name)
            })
        });
        match ready {
            Some(at) => ordered.push(left.remove(at)),
            None => break,
        }
    }
    ordered.extend(left);
    ordered
}

/// What the parameter `p` is set to: the run's input of its name if it is
/// given a number, else its formula of the other parameters.
fn value_of(reader: &Reader, p: &NumberParameter) -> GeopResult<ParameterValue> {
    match reader.retrieve_input(&p.name) {
        Some(Setting::Number(value)) => Ok(ParameterValue::Number(value)),
        _ => Ok(ParameterValue::Number(evaluate(&p.expression, |name| {
            read_parameter(reader, name)
        })?)),
    }
}

/// Looks the row a part family is built in up in a table: the row the
/// run's text input `input` names — `size = "M4"` from a program placing
/// the part — else the row `selected`. Defines the parameter `input` as the
/// row's name, and `input.column` as its value in each column (`size.d`),
/// each a target of its own ([`parameter`]), and adds them to the part's
/// [`ParameterValues`]. A text that names no row fails, naming the input
/// and the rows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LookupTable;

/// A table of the variants of a part family, one per row, looked up by the
/// input `input`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LookupTableArgs {
    pub input: String,
    pub columns: Vec<String>,
    pub rows: Vec<Row>,
    /// The row built where the run names none.
    pub selected: String,
}

impl LookupTableArgs {
    /// Checks the names of the input and the columns, that every row has a
    /// value per column, and that the row `selected` is there.
    pub fn validate(&self) -> GeopResult<()> {
        let ctx = |e: GeopError| e.with_context(format!("the table {:?}", self.input));
        validate_name(&self.input).map_err(ctx)?;
        for c in &self.columns {
            validate_name(c).map_err(|e| ctx(e.with_context("a column")))?;
        }
        for row in &self.rows {
            if row.values.len() != self.columns.len() {
                return Err(ctx(GeopError::new(format!(
                    "row {:?} has {} values for {} columns",
                    row.name,
                    row.values.len(),
                    self.columns.len()
                ))));
            }
        }
        if !self.rows.iter().any(|r| r.name == self.selected) {
            return Err(ctx(GeopError::new(format!(
                "there is no row {:?} to select",
                self.selected
            ))));
        }
        Ok(())
    }

    /// The name the value of `column` is read by: `size.d`.
    pub fn column(&self, column: &str) -> String {
        format!("{}.{column}", self.input)
    }
}

part_operation!(LookupTable(LookupTableArgs));

impl PartOperation for LookupTable {
    const KIND: &'static str = LOOKUP;
    const INFO: OperationInfo = OperationInfo::new(
        "Table",
        "A table of the variants of a part family — screw sizes, say — of which a program placing the part picks one by name.",
        OperationGroup::Part,
        OperationTier::Menu,
    )
    .icon(crate::parameters::icons::LOOKUP_TABLE);
    type Args = LookupTableArgs;
    type Session = ();

    fn new_args(_: &Context) -> LookupTableArgs {
        LookupTableArgs {
            input: "size".into(),
            columns: vec!["d".into()],
            rows: vec![Row {
                name: "A".into(),
                values: vec![1.0],
            }],
            selected: "A".into(),
        }
    }

    /// The row, then each column of it, then the step's entries of the
    /// part's parameter values.
    fn run(ctx: &mut StepContext, args: &LookupTableArgs) -> GeopResult<()> {
        args.validate()
            .map_err(|e| e.with_context(format!("lookup_table({})", ctx.step())))?;
        ctx.produce(&parameter(&args.input), args, row_of)?;
        let mut names = vec![args.input.clone()];
        for (k, column) in args.columns.iter().enumerate() {
            let name = args.column(column);
            ctx.produce(
                &parameter(&name),
                &(args.input.clone(), k, args.rows.clone()),
                column_of,
            )?;
            names.push(name);
        }
        ctx.produce(&parameter_values(), &names, add_values)
    }

    /// The table, in a popup: the input it is looked up by, and its rows —
    /// each named in the first column, the one built where the run names
    /// none chosen — with a value per column; the columns are the user's,
    /// named in their headers. A column or a row added is named afresh and
    /// its values are 0; the last row is not taken out, and taking out the
    /// row chosen chooses the first left.
    fn form<'a>(
        _: &'a Context<'a>,
        args: &LookupTableArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, LookupTableArgs> {
        let mut f = EditForm::<LookupTableArgs>::default();
        f.layout(Layout::Popup);
        f.input(
            "input",
            "Looked up by",
            &args.input,
            Some("size"),
            |edit, name| edit.args.input = name.trim().to_string(),
        );
        let mut columns = vec![Column::new("Row", ColumnKind::Text)];
        columns.extend((args.columns.iter()).map(|column| {
            Column::new(column, ColumnKind::Number { unit: Unit::Length }).editable()
        }));
        let rows = (args.rows.iter())
            .map(|row| TableRow {
                cells: (std::iter::once(Cell::new(&row.name)))
                    .chain(row.values.iter().map(|v| Cell::number(*v)))
                    .collect(),
                selected: row.name == args.selected,
                removable: args.rows.len() > 1,
            })
            .collect();
        let table = Table::new(columns, rows, "No rows: add one.")
            .columns_editable()
            .selectable();
        f.table("table", table, |edit, what| {
            let args = &mut *edit.args;
            match what {
                TableEdit::Cell {
                    row,
                    column: 0,
                    text,
                } => {
                    let Some(r) = args.rows.get_mut(row) else {
                        return;
                    };
                    let name = text.trim().to_string();
                    if args.selected == r.name {
                        args.selected = name.clone();
                    }
                    r.name = name;
                }
                TableEdit::Cell { row, column, text } => {
                    let cell = (args.rows.get_mut(row)).and_then(|r| r.values.get_mut(column - 1));
                    if let (Ok(n), Some(cell)) = (text.trim().parse::<f64>(), cell) {
                        *cell = n;
                    }
                }
                TableEdit::AddRow => {
                    let names: Vec<String> = args.rows.iter().map(|r| r.name.clone()).collect();
                    args.rows.push(Row {
                        name: fresh_name(&names, "row"),
                        values: vec![0.0; args.columns.len()],
                    });
                }
                TableEdit::RemoveRow { row } => {
                    if args.rows.len() > 1 && row < args.rows.len() {
                        let removed = args.rows.remove(row);
                        if removed.name == args.selected {
                            args.selected = args.rows[0].name.clone();
                        }
                    }
                }
                TableEdit::MoveRow { row, to } => move_row(&mut args.rows, row, to),
                TableEdit::SelectRow { row } => {
                    if let Some(r) = args.rows.get(row) {
                        args.selected = r.name.clone();
                    }
                }
                // The first column names the rows, and is the step's.
                TableEdit::Column { column: 0, .. } | TableEdit::RemoveColumn { column: 0 } => {}
                TableEdit::Column { column, text } => {
                    if let Some(c) = args.columns.get_mut(column - 1) {
                        *c = text.trim().to_string();
                    }
                }
                TableEdit::AddColumn => {
                    args.columns.push(fresh_name(&args.columns, "value"));
                    for row in &mut args.rows {
                        row.values.push(0.0);
                    }
                }
                TableEdit::RemoveColumn { column } => {
                    if column - 1 < args.columns.len() {
                        args.columns.remove(column - 1);
                        for row in &mut args.rows {
                            if column - 1 < row.values.len() {
                                row.values.remove(column - 1);
                            }
                        }
                    }
                }
                TableEdit::Paste { .. } => {}
            }
        });
        f
    }
}

/// `base`, or `base2`, `base3`, ... — the first not among `names`.
fn fresh_name(names: &[String], base: &str) -> String {
    let taken = |name: &str| names.iter().any(|n| n == name);
    let mut name = base.to_string();
    let mut n = 1;
    while taken(&name) {
        n += 1;
        name = format!("{base}{n}");
    }
    name
}

/// The row the table `args` is set to: the one the run's input names, else
/// its own selection. Fails for an input naming no row, or no text.
fn row_of(reader: &Reader, args: &LookupTableArgs) -> GeopResult<ParameterValue> {
    let wanted = match reader.retrieve_input(&args.input) {
        None => args.selected.clone(),
        Some(Setting::Text(row)) => row,
        Some(Setting::Number(n)) => {
            return Err(GeopError::new(format!(
                "the input {:?} is the number {n}: it names a row of its table by text",
                args.input
            )));
        }
    };
    match args.rows.iter().any(|row| row.name == wanted) {
        true => Ok(ParameterValue::Text(wanted)),
        false => {
            let rows: Vec<&str> = args.rows.iter().map(|row| row.name.as_str()).collect();
            Err(GeopError::new(format!(
                "the input {:?} is {wanted:?}, which names no row of its table: {}",
                args.input,
                rows.join(", ")
            )))
        }
    }
}

/// The value in the column at `k` of the row the table looked up by
/// `input`, of `rows`, is set to.
fn column_of(
    reader: &Reader,
    (input, k, rows): &(String, usize, Vec<Row>),
) -> GeopResult<ParameterValue> {
    let row = reader.retrieve_element(&ElementReference::whole(parameter(input)))?;
    let ParameterValue::Text(row) = row else {
        return Err(GeopError::new(format!(
            "the parameter {input:?} names no row"
        )));
    };
    let values = (rows.iter().find(|r| r.name == *row))
        .map(|r| &r.values)
        .ok_or_else(|| GeopError::new(format!("the table {input:?} has no row {row:?}")))?;
    (values.get(*k).copied())
        .map(ParameterValue::Number)
        .ok_or_else(|| {
            GeopError::new(format!(
                "row {row:?} of the table {input:?} has no column {k}"
            ))
        })
}

/// Defines what the part is called and ordered as, its colour, and what
/// it is made of: its [`properties`]. The colour is the run's input
/// [`COLOR`] where it is given one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PartProperties;

/// What a part is, besides its shape.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PartPropertiesArgs {
    /// What the part is, in words — `ISO 4762 socket head cap screw` —
    /// where it is listed: a bill of materials, a drawing's. None for a
    /// part known by its file's name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// What the part is ordered as, before the parameters it is built with
    /// (see [`Properties::designation`]): a norm — `ISO 4762` — or a
    /// catalogue or part number. None for a part that is made rather than
    /// bought.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub designation: Option<String>,
    /// The part's colour, `#rrggbb`; none for the viewer's own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    /// What the part is made of; none for a part whose material is not
    /// given — weighed as water, 1000 kg/m³, and said so.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material: Option<Material>,
}

part_operation!(PartProperties(PartPropertiesArgs));

impl PartOperation for PartProperties {
    const KIND: &'static str = PROPERTIES;
    const INFO: OperationInfo = OperationInfo::new(
        "Properties",
        "What the part is called and ordered as, its colour, and what it is made of.",
        OperationGroup::Part,
        OperationTier::Menu,
    )
    .icon(crate::parameters::icons::PROPERTIES);
    type Args = PartPropertiesArgs;
    type Session = ();

    fn new_args(_: &Context) -> PartPropertiesArgs {
        PartPropertiesArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &PartPropertiesArgs) -> GeopResult<()> {
        let step = ctx.step();
        if let Some(color) = &args.color {
            validate_color(color).map_err(|e| e.with_context(format!("properties({step})")))?;
        }
        if let Some(Material { name, density }) = &args.material
            && !(density.is_finite() && *density > 0.0)
        {
            return Err(GeopError::new(format!(
                "the material {name:?} has a density of {density} kg/m³: it must be a positive number"
            )));
        }
        ctx.produce(&properties(), args, |reader, args| {
            let color = match reader.retrieve_input(COLOR) {
                Some(Setting::Text(color)) if validate_color(&color).is_ok() => Some(color),
                _ => args.color.clone(),
            };
            let designation = (args.designation.clone()).map(|designation| {
                let values = read_parameter_values(reader).unwrap_or_default();
                designate(&designation, &values)
            });
            Ok(Properties {
                title: args.title.clone(),
                designation,
                color,
                material: args.material.clone(),
            })
        })
    }

    /// What is set is edited where the program's parameters are listed.
    fn form<'a>(
        _: &'a Context<'a>,
        _: &PartPropertiesArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, PartPropertiesArgs> {
        let mut f = EditForm::<PartPropertiesArgs>::default();
        let text = "The part's title, designation, colour and material — edited under Parameters.";
        f.form.dialog.text("properties", text, Tone::Hint);
        f
    }
}

/// What the part designated `designation` built with `values` is ordered
/// as: the designation, then each parameter in order — a table's row by its
/// name, a number after its own, a table's columns not at all —
/// `ISO 4762 M4x12`, `T-slot 2020 length 500`.
fn designate(designation: &str, values: &ParameterValues) -> String {
    let mut words = vec![designation.to_string()];
    for (name, value) in &values.0 {
        match value {
            ParameterValue::Text(row) => words.push(row.clone()),
            ParameterValue::Number(_) if name.contains('.') => {}
            ParameterValue::Number(n) => words.push(format!("{name} {n}")),
        }
    }
    words.join(" ")
}

/// What a part is, besides its shape, as its program says: what it is
/// called and ordered as, its colour and what it is made of.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Properties {
    pub title: Option<String>,
    /// What it is ordered as, with what it was built with: none for a part
    /// made rather than bought.
    pub designation: Option<String>,
    pub color: Option<String>,
    pub material: Option<Material>,
}

/// Defined once, by the program's [`PartProperties`] step.
impl Target for Properties {}

/// Read whole.
impl Kind for Properties {
    const NAME: &'static str = "properties";
}

impl HasElements<Properties> for Properties {
    type Element<'a> = &'a Properties;
    const SINGLETON: Option<&'static str> = Some(PROPERTIES);

    fn element(&self, name: &str) -> GeopResult<&Properties> {
        whole(self, name)
    }
}

/// The part's properties, defined by its [`PartProperties`] step.
pub fn properties() -> TargetReference<Properties> {
    TargetReference::new(PROPERTIES)
}

/// The properties of the part `reader` reads: none set, for a part whose
/// program says nothing of them.
pub fn read_properties<'a>(reader: &Reader<'a>) -> GeopResult<Cow<'a, Properties>> {
    if !reader.defines(&properties()) {
        return Ok(Cow::Owned(Properties::default()));
    }
    Ok(Cow::Borrowed(reader.retrieve_element(
        &ElementReference::whole(properties()),
    )?))
}

/// What a part is weighed as when its material is not given: water, 1000
/// kg/m³.
const UNGIVEN_DENSITY: f64 = 1000.0;

/// What a part is made of, as it is weighed.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialOf {
    pub name: String,
    /// kg/m³.
    pub density: f64,
    /// Whether no material was given, and it is weighed as water.
    pub assumed: bool,
}

impl Properties {
    /// What its part is made of: its material — or, not given, water, and
    /// said so.
    pub fn material(&self) -> MaterialOf {
        match &self.material {
            Some(material) => MaterialOf {
                name: material.name.clone(),
                density: material.density,
                assumed: false,
            },
            None => MaterialOf {
                name: "Water (no material given)".into(),
                density: UNGIVEN_DENSITY,
                assumed: true,
            },
        }
    }

    /// Its part's colour, `#rrggbb`: none for the viewer's own.
    pub fn color(&self) -> Option<&str> {
        self.color.as_deref()
    }

    /// What its part is called where it is listed: its title, else `stem`
    /// — the name of the file it is placed from.
    pub fn name<'a>(&'a self, stem: &'a str) -> &'a str {
        self.title.as_deref().unwrap_or(stem)
    }
}

/// The name of the file `file`, without folders or `.geop`: what a part
/// placed from it is listed as without a title, and what a file written
/// from it is named after — `part` if that leaves nothing.
pub fn stem(file: &str) -> &str {
    let file = file.rsplit(['/', ':']).next().unwrap_or_default();
    match file.trim_end_matches(".geop") {
        "" => "part",
        stem => stem,
    }
}
