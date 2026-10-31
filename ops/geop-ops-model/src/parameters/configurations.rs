//! A program's configurations ([`ConfigurationsPanel`]): the variants of
//! one design — a light and a full version of a bracket, a long and a short
//! shaft — each setting the parameters, and which steps run.
//!
//! A configuration is a row of the table looked up by the parameter
//! [`CONFIGURATION`] (a [`LookupTable`] step, as any part family's), so
//! that each column is a parameter of the configuration built,
//! `configuration.length`, which formulas read as any other. A step that
//! runs in some configurations only says so by its formula of whether it
//! runs ([`ProgramStep::active`]): `configuration.fillet1`, a column of 1
//! where it runs and 0 where it is suppressed. Nothing else is special
//! about a configuration: the row built is the table's own, which a
//! program placing the part sets by name as it sets any table's
//! (`configuration = "light"`), and the bill of materials, the drawings
//! and everything else read the part as built.
//!
//! The panel shows each table a program is looked up in — its
//! configurations first — with the row built, to switch; adds a
//! configuration; and suppresses a step in the configuration built, or —
//! where there are no two configurations — in all of it.

use geop_core_editor::{
    editor::{Changed, EditorContext},
    registry::Panel,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    numeric::{evaluate, read_parameter},
    program::{Program, ProgramStep},
    reader::Reader,
};
use serde::{Deserialize, Serialize};

use crate::parameters::{
    definition::{CONFIGURATION, Row, validate_name},
    steps::{LOOKUP, LookupTableArgs},
};

/// The configurations panel: see the module docs.
pub struct ConfigurationsPanel;

/// A table a program is looked up in, as the panel shows it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TableChoice {
    /// The step looking it up.
    pub step: String,
    /// The parameter it is looked up by: [`CONFIGURATION`] for the
    /// program's configurations.
    pub input: String,
    /// Its rows, by name, in order.
    pub rows: Vec<String>,
    /// The row built.
    pub selected: String,
}

/// What the configurations panel shows.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ConfigurationsState {
    /// Each table the program is looked up in, its configurations first,
    /// then the others in the program's order.
    pub tables: Vec<TableChoice>,
}

/// What the configurations panel is sent.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum ConfigurationsCommand {
    /// Build the row `row` of the table looked up by `input`.
    Select { input: String, row: String },
    /// Add a configuration, `name` — a fresh one, if none — as the one
    /// built is, and build it. The first one added makes the table, its
    /// first row `Default`: the program as it was.
    Add {
        #[serde(default)]
        name: Option<String>,
    },
    /// Suppress the step `step` in the configuration built, or run it
    /// again there, if it is suppressed; where there are no two
    /// configurations, everywhere.
    Suppress { step: String },
}

impl Panel for ConfigurationsPanel {
    const KEY: &'static str = "configurations";
    type State = ConfigurationsState;
    type Command = ConfigurationsCommand;

    fn state(&self, program: &Program, _: &Reader) -> ConfigurationsState {
        let mut tables: Vec<TableChoice> = (program.steps.iter())
            .filter(|step| step.operation == LOOKUP)
            .filter_map(|step| {
                let table = serde_json::from_value::<LookupTableArgs>(step.args.clone()).ok()?;
                Some(TableChoice {
                    step: step.id.clone(),
                    input: table.input,
                    rows: table.rows.into_iter().map(|row| row.name).collect(),
                    selected: table.selected,
                })
            })
            .collect();
        tables.sort_by_key(|table| table.input != CONFIGURATION);
        ConfigurationsState { tables }
    }

    fn command(
        &self,
        ctx: &mut EditorContext,
        command: ConfigurationsCommand,
    ) -> GeopResult<Changed> {
        match command {
            ConfigurationsCommand::Select { input, row } => {
                ctx.edit_program(|program| select(program, &input, &row))?;
            }
            ConfigurationsCommand::Add { name } => {
                ctx.idle()?;
                ctx.edit_program(|program| add(program, name.as_deref()))?;
            }
            ConfigurationsCommand::Suppress { step } => {
                let runs = runs(ctx.program(), &ctx.part(), &step)?;
                ctx.edit_program(|program| suppress(program, &step, runs))?;
            }
        }
        Ok(Changed::Program)
    }
}

/// The step of `program` looking up the table by `input`, its position and
/// its table.
fn table(program: &Program, input: &str) -> Option<(usize, LookupTableArgs)> {
    (program.steps.iter().enumerate()).find_map(|(at, step)| {
        let table = serde_json::from_value::<LookupTableArgs>(step.args.clone()).ok()?;
        (step.operation == LOOKUP && table.input == input).then_some((at, table))
    })
}

/// Writes `table` as the arguments of the step at `at` of `program`.
fn write_table(program: &mut Program, at: usize, table: &LookupTableArgs) -> GeopResult<()> {
    let step = (program.steps.get_mut(at))
        .ok_or_else(|| GeopError::new(format!("the program has no step at {at}")))?;
    step.args = serde_json::to_value(table)
        .map_err(|e| GeopError::new(format!("the table {:?}: {e}", table.input)))?;
    Ok(())
}

/// `program` building the row `row` of the table looked up by `input`.
fn select(program: &mut Program, input: &str, row: &str) -> GeopResult<()> {
    let (at, mut table) = table(program, input)
        .ok_or_else(|| GeopError::new(format!("the program looks up no table by {input:?}")))?;
    if !table.rows.iter().any(|r| r.name == row) {
        let rows: Vec<&str> = table.rows.iter().map(|r| r.name.as_str()).collect();
        return Err(GeopError::new(format!(
            "the table {input:?} has no row {row:?}: its rows are {}",
            rows.join(", ")
        )));
    }
    table.selected = row.to_string();
    write_table(program, at, &table)
}

/// `program` with the configuration `name` added — a fresh one for none —
/// as the configuration built is, and building it; the first one making
/// the table of configurations, first in the program, its first row
/// `Default`.
fn add(program: &mut Program, name: Option<&str>) -> GeopResult<()> {
    let (at, mut table) = match table(program, CONFIGURATION) {
        Some(found) => found,
        None => {
            let table = LookupTableArgs {
                input: CONFIGURATION.into(),
                columns: Vec::new(),
                rows: vec![Row {
                    name: "Default".into(),
                    values: Vec::new(),
                }],
                selected: "Default".into(),
            };
            let id = (std::iter::once("configurations".to_string()))
                .chain((2..).map(|n| format!("configurations{n}")))
                .find(|id| program.step(id).is_err())
                .unwrap_or_default();
            program.steps.insert(0, ProgramStep::of(id, &table)?);
            (0, table)
        }
    };
    let taken = |name: &str| table.rows.iter().any(|r| r.name == name);
    let name = match name.map(str::trim) {
        Some("") => return Err(GeopError::new("a configuration needs a name")),
        Some(name) if taken(name) => {
            return Err(GeopError::new(format!(
                "there is a configuration {name:?} already"
            )));
        }
        Some(name) => name.to_string(),
        None => (2..)
            .map(|n| format!("Configuration {n}"))
            .find(|name| !taken(name))
            .unwrap_or_default(),
    };
    let values = (table.rows.iter())
        .find(|r| r.name == table.selected)
        .map(|r| r.values.clone())
        .unwrap_or_else(|| vec![0.0; table.columns.len()]);
    table.rows.push(Row {
        name: name.clone(),
        values,
    });
    table.selected = name;
    write_table(program, at, &table)
}

/// Whether the step `id` of `program` runs in the part `part` reads, as
/// its formula says (see [`ProgramStep::active`]): a formula that comes to
/// nothing is taken to run, so suppressing it is what the user is offered.
fn runs(program: &Program, part: &Reader, id: &str) -> GeopResult<bool> {
    let step = program
        .step(id)
        .map_err(|e| e.with_context("suppressing a step: commit it first"))?;
    Ok(match &step.active {
        None => true,
        Some(formula) => evaluate(formula, |name| read_parameter(part, name)) != Ok(0.0),
    })
}

/// The name of the column of the table of configurations that says where
/// the step `id` runs: its id, in the letters a parameter's name allows.
fn column_of(id: &str) -> String {
    let name: String = (id.chars())
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect();
    match name.starts_with(|c: char| c.is_ascii_alphabetic() || c == '_') {
        true => name,
        false => format!("_{name}"),
    }
}

/// `program` with the step `id`, which `runs` now, suppressed — or run
/// again, if it does not — in the configuration built: by a column of the
/// table of configurations, 1 where it runs, which its formula reads (and
/// none, once it runs in all of them). Where there are no two
/// configurations, it is suppressed, or runs again, everywhere: by a
/// formula of `0`, or none.
fn suppress(program: &mut Program, id: &str, runs: bool) -> GeopResult<()> {
    let at = program.index_of(id)?;
    let configurations = table(program, CONFIGURATION).filter(|(_, table)| table.rows.len() > 1);
    let Some((table_at, mut table)) = configurations else {
        program.step_mut(id)?.active = (runs).then(|| "0".to_string());
        return Ok(());
    };
    if table_at > at {
        return Err(GeopError::new(format!(
            "the configurations are looked up after the step `{id}`, by the step `{}`: move \
             that step before it to suppress `{id}` in one configuration",
            program
                .steps
                .get(table_at)
                .map_or("", |step| step.id.as_str())
        )));
    }
    // The column the step's formula reads, if it reads one; else a fresh
    // one, of where it runs now in every configuration.
    let read = |column: &str| format!("{CONFIGURATION}.{column}");
    let active = program.step(id)?.active.clone();
    let column = match (table.columns.iter()).find(|c| active.as_deref() == Some(read(c).as_str()))
    {
        Some(column) => column.clone(),
        None => {
            let base = column_of(id);
            validate_name(&base)?;
            let column = (std::iter::once(base.clone()))
                .chain((2..).map(|n| format!("{base}{n}")))
                .find(|c| !table.columns.contains(c))
                .unwrap_or(base);
            table.columns.push(column.clone());
            for row in &mut table.rows {
                row.values.push(f64::from(u8::from(runs)));
            }
            column
        }
    };
    let k = (table.columns.iter().position(|c| *c == column))
        .ok_or_else(|| GeopError::new(format!("the table has no column {column:?}")))?;
    let selected = table.selected.clone();
    if let Some(row) = table.rows.iter_mut().find(|r| r.name == selected)
        && let Some(value) = row.values.get_mut(k)
    {
        *value = f64::from(u8::from(!runs));
    }
    // A step that runs in every configuration needs no column.
    let everywhere = (table.rows.iter()).all(|r| r.values.get(k).is_some_and(|v| *v != 0.0));
    let active = match everywhere {
        true => {
            table.columns.retain(|c| *c != column);
            for row in &mut table.rows {
                row.values = (row.values.iter().enumerate())
                    .filter(|(j, _)| *j != k)
                    .map(|(_, v)| *v)
                    .collect();
            }
            None
        }
        false => Some(read(&column)),
    };
    write_table(program, table_at, &table)?;
    program.step_mut(id)?.active = active;
    Ok(())
}
