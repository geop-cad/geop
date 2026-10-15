//! The steps a program says what its part is by, besides its shape: its
//! parameters ([`DefineParameters`]) and its properties
//! ([`PartProperties`]). Each defines targets as any step does: a formula
//! reads a parameter's target, and what lists the part reads its
//! properties'.
//!
//! What a parameter is set to where the part is built is the run's input
//! of its name — a [`Setting`] a program placing the part gives — and its
//! own definition where none is given. The colour is set the same way, by
//! the input [`COLOR`].

use std::borrow::Cow;

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    editing::Tone,
    numeric_input::{ParameterValue, evaluate, names_in, parameter, read_parameter},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::{Input, Reader, State},
    target::TargetReference,
};
use serde::{Deserialize, Serialize};

use crate::{
    BuildPart, Design, PartReader, Runner, StateValue, StepContext,
    parameters::{
        COLOR, Material, Parameter, ParameterKind, Parameters, Setting, validate_color,
    },
};

/// The kind of the step defining a program's parameters, and its id.
pub const PARAMETERS: &str = "parameters";

/// The kind of the step defining a part's properties, and its id.
pub const PROPERTIES: &str = "properties";

/// Defines the program's parameters, each a target of its own
/// ([`parameter`]): what the run's input of its name sets it to, else what
/// it is defined as. A number's formula may read any other parameter of the
/// step, defined before it or after; one reading one that reads it back
/// fails, saying so.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DefineParameters;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DefineParametersArgs {
    #[serde(default)]
    pub values: Vec<Parameter>,
}

part_operation!(DefineParameters(DefineParametersArgs));

impl PartOperation for DefineParameters {
    const KIND: &'static str = PARAMETERS;
    const INFO: OperationInfo = OperationInfo::new(
        "Parameters",
        "The values the design is given by — numbers, tables of variants — which a program placing the part may set.",
        OperationGroup::Part,
        OperationTier::Menu,
    );
    type Args = DefineParametersArgs;
    type Input = Setting;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    fn new_args(_: &StepContext) -> DefineParametersArgs {
        DefineParametersArgs::default()
    }

    /// Each parameter, in an order in which a formula's parameters come
    /// before it — tables first, which read none — and then the list of
    /// them all.
    fn run(
        step: &str,
        _: &Runner,
        part: &mut BuildPart,
        args: &DefineParametersArgs,
    ) -> GeopResult<()> {
        let parameters = Parameters {
            values: args.values.clone(),
            ..Parameters::default()
        };
        parameters
            .validate()
            .map_err(|e| e.with_context(format!("parameters({step})")))?;
        for p in in_order(&args.values) {
            part.define_target(&parameter(&p.name), p, value_of)?;
        }
        let names: Vec<String> = args.values.iter().map(|p| p.name.clone()).collect();
        part.define_target(&parameter_values(), &names, |reader, names| {
            Ok(ParameterValues(
                (names.iter())
                    .filter_map(|name| {
                        let value = reader.retrieve_element(&ElementReference::whole(
                            parameter(name),
                        ));
                        Some((name.clone(), value.ok()?.clone()))
                    })
                    .collect(),
            ))
        })
    }

    /// What is defined is edited where the program's parameters are listed.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &DefineParametersArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, DefineParametersArgs> {
        let mut f = EditForm::<Design, DefineParametersArgs>::default();
        let names: Vec<&str> = args.values.iter().map(|p| p.name.as_str()).collect();
        let text = match names.is_empty() {
            true => "No parameters: add them under Parameters.".to_string(),
            false => format!("{} — edited under Parameters.", names.join(", ")),
        };
        f.form.dialog.text("parameters", text, Tone::Hint);
        f
    }
}

/// `values` in an order in which a number comes after every parameter its
/// formula names: the tables, then the numbers as their formulas' names
/// are defined, then — reading what is not there, or reading each other —
/// the rest, which fail.
fn in_order(values: &[Parameter]) -> Vec<&Parameter> {
    let mut ordered: Vec<&Parameter> = Vec::with_capacity(values.len());
    let mut left: Vec<&Parameter> = Vec::new();
    for p in values {
        match &p.kind {
            ParameterKind::Table { .. } => ordered.push(p),
            ParameterKind::Number { .. } => left.push(p),
        }
    }
    let defines = |p: &Parameter, name: &str| p.name == name.split('.').next().unwrap_or(name);
    loop {
        let ready = left.iter().position(|p| {
            let ParameterKind::Number { expression, .. } = &p.kind else {
                return true;
            };
            names_in(expression).iter().all(|name| {
                ordered.iter().any(|q| defines(q, name))
                    || !values.iter().any(|q| defines(q, name))
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
/// given one, else its definition — a number's formula of the other
/// parameters, a table's row selected.
fn value_of(reader: &PartReader, p: &Parameter) -> GeopResult<ParameterValue> {
    let given = reader.retrieve_input(&p.name);
    match &p.kind {
        ParameterKind::Number { expression, .. } => match given {
            Some(Setting::Number(value)) => Ok(ParameterValue::Number(value)),
            _ => Ok(ParameterValue::Number(evaluate(expression, |name| {
                read_parameter(reader, name)
            })?)),
        },
        ParameterKind::Table {
            columns,
            rows,
            selected,
        } => {
            let wanted = match &given {
                Some(Setting::Text(row)) if rows.iter().any(|r| &r.name == row) => row,
                _ => selected,
            };
            let row = (rows.iter().find(|r| &r.name == wanted))
                .ok_or_else(|| GeopError::new(format!("the table has no row {wanted:?}")))?;
            Ok(ParameterValue::Row {
                name: row.name.clone(),
                columns: columns.iter().cloned().zip(row.values.iter().copied()).collect(),
            })
        }
    }
}

/// Every parameter of the part, by name, in the order the step defines
/// them, as each comes to: what lists or designates the part reads. A
/// parameter that failed is left out.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ParameterValues(pub Vec<(String, ParameterValue)>);

/// Read whole.
impl Kind for ParameterValues {
    const NAME: &'static str = "parameter_values";
}

impl HasElements<ParameterValues> for ParameterValues {
    type Element<'a> = &'a ParameterValues;
    const SINGLETON: Option<&'static str> = Some("parameter_values");

    fn element(&self, name: &str) -> GeopResult<&ParameterValues> {
        whole(self, name)
    }
}

/// The target of every parameter of the part, as they come to.
pub fn parameter_values() -> TargetReference<ParameterValues> {
    TargetReference::new("parameter_values")
}

/// Every parameter of the part `reader` reads, as they come to: none, for
/// a part whose program defines none.
pub fn read_parameter_values<'a, I: Input, S: State>(
    reader: &Reader<'a, I, S>,
) -> GeopResult<Cow<'a, ParameterValues>> {
    if !reader.defines(&parameter_values()) {
        return Ok(Cow::Owned(ParameterValues::default()));
    }
    Ok(Cow::Borrowed(
        reader.retrieve_element(&ElementReference::whole(parameter_values()))?,
    ))
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
    );
    type Args = PartPropertiesArgs;
    type Input = Setting;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    fn new_args(_: &StepContext) -> PartPropertiesArgs {
        PartPropertiesArgs::default()
    }

    fn run(
        step: &str,
        _: &Runner,
        part: &mut BuildPart,
        args: &PartPropertiesArgs,
    ) -> GeopResult<()> {
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
        part.define_target(&properties(), args, |reader, args| {
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
        _: &'a StepContext<'a>,
        _: &PartPropertiesArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, PartPropertiesArgs> {
        let mut f = EditForm::<Design, PartPropertiesArgs>::default();
        let text = "The part's title, designation, colour and material — edited under Parameters.";
        f.form.dialog.text("properties", text, Tone::Hint);
        f
    }
}

/// What the part designated `designation` built with `values` is ordered
/// as: the designation, then each parameter in order — a table's row by its
/// name, a number after its own — `ISO 4762 M4x12`, `T-slot 2020 length
/// 500`.
fn designate(designation: &str, values: &ParameterValues) -> String {
    let mut words = vec![designation.to_string()];
    for (name, value) in &values.0 {
        match value {
            ParameterValue::Row { name: row, .. } => words.push(row.clone()),
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
pub fn read_properties<'a, I: Input, S: State>(
    reader: &Reader<'a, I, S>,
) -> GeopResult<Cow<'a, Properties>> {
    if !reader.defines(&properties()) {
        return Ok(Cow::Owned(Properties::default()));
    }
    Ok(Cow::Borrowed(
        reader.retrieve_element(&ElementReference::whole(properties()))?,
    ))
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
