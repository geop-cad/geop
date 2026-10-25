//! The parameters of a [`Program`], as its steps define them (see
//! [`crate::parameters::steps`]): read, set, renamed, and what reads each.

use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    numeric::{
        ParameterValue, formulas_in, names_in, parameter_of, read_parameter_values,
        rename_parameter_in,
    },
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    runner::Runner,
};
use serde::Serialize;

use crate::parameters::{
    definition::{NumberParameter, Parameter, ParameterKind, Parameters, validate_name},
    steps::{
        DefineParameters, DefineParametersArgs, LOOKUP, LookupTable, LookupTableArgs, PARAMETERS,
        PROPERTIES, PartPropertiesArgs,
    },
};

/// The parameters and properties of `program`, as its steps define them
/// (see [`crate::parameters::steps`]): its tables, as its [`LookupTable`] steps look
/// them up, after its numbers — none, where it has no such step.
pub fn parameters_of(program: &Program) -> Parameters {
    let args = |kind: &str| program.steps.iter().find(|s| s.operation == kind);
    let tables = (program.steps.iter())
        .filter(|s| s.operation == LOOKUP)
        .filter_map(|step| serde_json::from_value::<LookupTableArgs>(step.args.clone()).ok())
        .map(|table| Parameter {
            name: table.input,
            kind: ParameterKind::Table {
                columns: table.columns,
                rows: table.rows,
                selected: table.selected,
            },
        });
    let numbers = (args(PARAMETERS))
        .and_then(|step| serde_json::from_value::<DefineParametersArgs>(step.args.clone()).ok())
        .unwrap_or_default()
        .values
        .into_iter()
        .map(|p| Parameter {
            name: p.name,
            kind: ParameterKind::Number {
                expression: p.expression,
                min: p.min,
                max: p.max,
            },
        });
    let properties = (args(PROPERTIES))
        .and_then(|step| serde_json::from_value::<PartPropertiesArgs>(step.args.clone()).ok())
        .unwrap_or_default();
    Parameters {
        color: properties.color,
        material: properties.material,
        title: properties.title,
        designation: properties.designation,
        values: numbers.chain(tables).collect(),
    }
}

/// `program` with its parameters and properties set to `parameters`: in
/// the steps that define them, put first where it has none yet — and none,
/// where there is nothing to define. Each table is a [`LookupTable`] step
/// of its own, `table_<name>`, before the numbers, whose formulas may read
/// its columns. The steps check them as they run.
pub fn set_parameters(program: &mut Program, parameters: Parameters) {
    let properties = PartPropertiesArgs {
        title: parameters.title,
        designation: parameters.designation,
        color: parameters.color,
        material: parameters.material,
    };
    let mut numbers = Vec::new();
    let mut tables = Vec::new();
    for p in parameters.values {
        match p.kind {
            ParameterKind::Number {
                expression,
                min,
                max,
            } => numbers.push(NumberParameter {
                name: p.name,
                expression,
                min,
                max,
            }),
            ParameterKind::Table {
                columns,
                rows,
                selected,
            } => tables.push(LookupTableArgs {
                input: p.name,
                columns,
                rows,
                selected,
            }),
        }
    }
    let values = DefineParametersArgs { values: numbers };
    let empty = properties == PartPropertiesArgs::default();
    set_step(program, PROPERTIES, (!empty).then_some(&properties));
    set_step(
        program,
        PARAMETERS,
        (!values.values.is_empty()).then_some(&values),
    );
    program.steps.retain(|step| step.operation != LOOKUP);
    for (k, table) in tables.iter().enumerate() {
        program.steps.insert(k, lookup_step(table));
    }
}

/// The step looking `table` up.
#[allow(
    clippy::expect_used,
    reason = "the arguments of a lookup step are plain data"
)]
fn lookup_step(table: &LookupTableArgs) -> ProgramStep {
    ProgramStep {
        id: format!("table_{}", table.input),
        operation: LOOKUP.to_string(),
        args: serde_json::to_value(table).expect("a step's arguments are plain data"),
    }
}

/// The step of `program` of the operation `kind` holding `args`: put first
/// where there is none, under the kind's name; taken out, for none.
#[allow(
    clippy::expect_used,
    reason = "the arguments of a parameters or properties step are plain data"
)]
fn set_step<A: Serialize>(program: &mut Program, kind: &str, args: Option<&A>) {
    let steps = &mut program.steps;
    let at = steps.iter().position(|s| s.operation == kind);
    let Some(args) = args else {
        if let Some(at) = at {
            steps.remove(at);
        }
        return;
    };
    let args = serde_json::to_value(args).expect("a step's arguments are plain data");
    match at {
        Some(at) => steps[at].args = args,
        None => {
            let taken = |id: &str| steps.iter().any(|s| s.id == id);
            let id = (std::iter::once(kind.to_string()))
                .chain((1..).map(|n| format!("{kind}{n}")))
                .find(|id| !taken(id))
                .unwrap_or_else(|| kind.to_string());
            let step = ProgramStep {
                id,
                operation: kind.to_string(),
                args,
            };
            steps.insert(0, step);
        }
    }
}

/// What the parameters of `program` come to as it defines them, by the names a
/// formula reads them by: a number by its name, a table's columns as
/// `table.column` — its steps defining them run on their own (see
/// [`crate::parameters::steps::DefineParameters`], [`LookupTable`]). Fails naming one
/// that does not come to anything.
pub fn parameter_numbers(program: &Program) -> GeopResult<BTreeMap<String, f64>> {
    let steps = (program.steps.iter())
        .filter(|step| step.operation == PARAMETERS || step.operation == LOOKUP)
        .cloned()
        .collect();
    let registry = (OperationRegistry::default().with(DefineParameters))
        .and_then(|registry| registry.with(LookupTable))?;
    let runner = Runner::new(registry);
    let mut part = Part::default();
    let defining = Program::from_steps(steps);
    let output = runner.run(&mut part, &defining, None);
    if let Some((step, e)) = output.step_errors.into_iter().next() {
        return Err(e.with_context(format!("step {step:?}")));
    }
    let reader = part.current();
    let values = read_parameter_values(&reader)?;
    for p in &parameters_of(program).values {
        if !values.0.iter().any(|(name, _)| *name == p.name) {
            return Err(GeopError::new(format!(
                "the parameter {:?} comes to nothing",
                p.name
            )));
        }
    }
    Ok((values.0.iter())
        .filter_map(|(name, value)| match value {
            ParameterValue::Number(n) => Some((name.clone(), *n)),
            ParameterValue::Text(_) => None,
        })
        .collect())
}

/// What reads each parameter of `program`, by its name: the ids of the steps, and
/// the names of the other parameters, whose formulas name it — what
/// fails if it goes. A name no parameter has is listed too: what reads
/// it fails already. A step's formulas are found in its saved arguments
/// ([`formulas_in`]).
pub fn parameter_uses(program: &Program) -> BTreeMap<String, Vec<String>> {
    let mut uses: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut note = |reader: &str, formula: &str| {
        for name in names_in(formula) {
            let readers = uses.entry(parameter_of(&name).to_string()).or_default();
            if !readers.iter().any(|r| r == reader) {
                readers.push(reader.to_string());
            }
        }
    };
    for p in &parameters_of(program).values {
        if let ParameterKind::Number { expression, .. } = &p.kind {
            note(&p.name, expression);
        }
    }
    for step in &program.steps {
        for formula in formulas_in(&step.args) {
            note(&step.id, formula);
        }
    }
    uses
}

/// The parameter `from` of `program` named `to`, and every formula reading it —
/// the other parameters', every step's — reading `to`. Fails for a
/// parameter there is none of, and for a name that is taken or no name.
pub fn rename_parameter(program: &mut Program, from: &str, to: &str) -> GeopResult<()> {
    let mut parameters = parameters_of(program);
    if parameters.get(from).is_none() {
        return Err(GeopError::new(format!("there is no parameter {from:?}")));
    }
    if from == to {
        return Ok(());
    }
    validate_name(to)?;
    if parameters.get(to).is_some() {
        return Err(GeopError::new(format!(
            "there is a parameter {to:?} already"
        )));
    }
    for step in &mut program.steps {
        rename_parameter_in(&mut step.args, from, to);
    }
    parameters.rename(from, to);
    set_parameters(program, parameters);
    Ok(())
}
