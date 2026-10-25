//! The parameters panel ([`ParametersPanel`]): what a program's parameters
//! are defined as and come to, what reads each, the materials a part's can
//! be — and setting and renaming them — and the part drawn in its colour
//! ([`PartColor`]).

use std::collections::BTreeMap;

use geop_core_editor::{
    editor::{Changed, EditorContext},
    registry::{Drawer, Panel, StructureItem},
    view::PartView,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference,
    numeric::{ParameterValue, parameter, read_parameter_values},
    program::Program,
    reader::Reader,
    values::{Setting, Settings},
};
use serde::{Deserialize, Serialize};

use crate::parameters::{
    COLOR, Material, Parameters, parameter_uses, parameters_of, read_properties, rename_parameter,
    set_parameters,
};

/// The parameters panel: see the module docs.
pub struct ParametersPanel;

/// What the parameters panel shows.
#[derive(Debug, Serialize, Deserialize)]
pub struct ParametersPanelState {
    /// What the program's parameters resolve to, and why those that do
    /// not resolve fail.
    pub parameters: ParametersState,
    /// What reads each parameter, by name: the steps and the other
    /// parameters whose formulas do, which fail if it goes (see
    /// [`parameter_uses`]).
    pub parameter_uses: BTreeMap<String, Vec<String>>,
    /// The materials a part's can be picked from (see [`Material`]).
    pub materials: Vec<Material>,
}

/// What the parameters panel is sent.
#[derive(Serialize, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum ParametersCommand {
    /// The program's parameters are now these: what every step reading
    /// one is built with again. Allowed while a step is edited, which then
    /// sees them.
    SetParameters { parameters: Parameters },
    /// Name the parameter `from` `to`, and every formula reading it read
    /// `to` (see [`rename_parameter`]): the other parameters', the steps'.
    /// Refused while a step is edited.
    RenameParameter { from: String, to: String },
}

impl Panel for ParametersPanel {
    const KEY: &'static str = "parameters";
    type State = ParametersPanelState;
    type Command = ParametersCommand;

    fn state(&self, program: &Program, part: &Reader) -> ParametersPanelState {
        ParametersPanelState {
            parameters: ParametersState::of(program, part),
            parameter_uses: parameter_uses(program),
            materials: Material::listed().collect(),
        }
    }

    fn command(&self, ctx: &mut EditorContext, command: ParametersCommand) -> GeopResult<Changed> {
        match command {
            ParametersCommand::SetParameters { parameters } => {
                parameters.validate()?;
                // A step put first moves the others along.
                ctx.edit_program(|program| {
                    set_parameters(program, parameters.clone());
                    Ok(())
                })?;
            }
            ParametersCommand::RenameParameter { from, to } => {
                ctx.idle()?;
                ctx.edit_program(|program| rename_parameter(program, &from, &to))?;
            }
        }
        Ok(Changed::Program)
    }
}

/// A program's parameters as an editor shows them: what its steps define
/// them as (see [`crate::parameters::steps`]) — numbers, and the tables its lookup
/// steps look up — what each comes to — a number, a table's row and its
/// columns as `table.column`, the colour — and why those that do not fail.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParametersState {
    pub defined: Parameters,
    pub values: Settings,
    pub errors: BTreeMap<String, String>,
}

impl ParametersState {
    /// The parameters `program` defines, as the part `reader` reads came to
    /// with them.
    pub fn of(program: &Program, reader: &Reader) -> Self {
        let defined = parameters_of(program);
        let mut values = Settings::new();
        let mut errors = BTreeMap::new();
        for p in &defined.values {
            let reference = ElementReference::whole(parameter(&p.name));
            match reader.retrieve_element(&reference) {
                Ok(ParameterValue::Number(n)) => {
                    values.insert(p.name.clone(), Setting::Number(*n));
                }
                Ok(ParameterValue::Text(row)) => {
                    values.insert(p.name.clone(), Setting::Text(row.clone()));
                }
                Err(e) => {
                    errors.insert(p.name.clone(), e.root_message().to_string());
                }
            }
        }
        // A table's columns, as its lookup step defines them.
        let columns = read_parameter_values(reader).unwrap_or_default();
        for (name, value) in &columns.0 {
            if let (Some((table, _)), ParameterValue::Number(n)) = (name.split_once('.'), value)
                && defined.get(table).is_some()
            {
                values.insert(name.clone(), Setting::Number(*n));
            }
        }
        if let Some(color) = read_properties(reader).ok().and_then(|p| p.color.clone()) {
            values.insert(COLOR.into(), Setting::Text(color));
        }
        Self {
            defined,
            values,
            errors,
        }
    }
}

/// Draws a part in the colour its properties give it (see
/// [`crate::parameters::PartProperties`]).
pub struct PartColor;

impl Drawer for PartColor {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        view.color = read_properties(reader)?.color().map(str::to_string);
        Ok(())
    }

    fn structure(&self, _: &Reader) -> GeopResult<Vec<StructureItem>> {
        Ok(Vec::new())
    }
}
