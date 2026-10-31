//! [`Parameters`]: the named values a program's design is given by —
//! numbers, tables of variants, the part's colour and what it is made of.
//!
//! A parameter is defined once, in the program, and read by name wherever
//! a number is typed ([`NumericInput`](geop_core_program::numeric::NumericInput)):
//! a sketch dimension of `width / 2`, a number parameter of `2 * wall`. A
//! table is a family of variants — screw sizes, say — of which one row is
//! selected: `screw.diameter` is that row's value in the column
//! `diameter`.
//!
//! What a parameter is defined as is the program's own, in a step of it —
//! a number in [`crate::parameters::steps::DefineParameters`], a table in a
//! [`crate::parameters::steps::LookupTable`] of its own; what it is *built with* may be set
//! by a program placing the part, by the same name: the run's input of
//! that name (see [`geop_core_program::values::Settings`]). So a placed screw is made M5 by the
//! program placing it, without touching the screw's own file.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::numeric::FUNCTION_NAMES;
use serde::{Deserialize, Serialize};

/// The name the part's colour is set by.
pub const COLOR: &str = "color";

/// The parameter a program's configurations are looked up by: the table of
/// its configurations is the one looked up by it (see
/// [`crate::parameters::configurations`]).
pub const CONFIGURATION: &str = "configuration";

/// What a part is made of: a name, and its density in kg/m³ — what its
/// mass and inertia are computed with (see `geop-ops-inspect`). Lengths are
/// millimetres, so a part of `V` mm³ weighs `density · V · 1e-9` kg.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub name: String,
    pub density: f64,
}

/// Materials an engineer reaches for, by name, with their densities in
/// kg/m³ — what a material is picked from; any other is given by its
/// density.
const MATERIALS: [(&str, f64); 10] = [
    ("Aluminium 6061", 2700.0),
    ("Steel", 7850.0),
    ("Stainless steel 304", 8000.0),
    ("Brass", 8500.0),
    ("Titanium Ti-6Al-4V", 4430.0),
    ("PLA", 1240.0),
    ("ABS", 1050.0),
    ("PETG", 1270.0),
    ("Nylon PA12", 1010.0),
    ("Acetal (POM)", 1410.0),
];

impl Material {
    /// The materials of [`MATERIALS`].
    pub fn listed() -> impl Iterator<Item = Self> {
        (MATERIALS.iter()).map(|&(name, density)| Self {
            name: name.into(),
            density,
        })
    }

    /// The material of [`MATERIALS`] called `name`, if there is one.
    pub fn named(name: &str) -> Option<Self> {
        Self::listed().find(|material| material.name == name)
    }
}

/// One row of a table ([`ParameterKind::Table`],
/// [`crate::parameters::steps::LookupTableArgs`]): a variant, by name, with a value per
/// column.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Row {
    pub name: String,
    pub values: Vec<f64>,
}

/// What a parameter of a program is, as an editor lists them all: a number
/// its [`crate::parameters::steps::DefineParameters`] step defines, or a table a
/// [`crate::parameters::steps::LookupTable`] step of its own looks up.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ParameterKind {
    /// A number, given as a formula of the parameters defined before it —
    /// `12`, `width / 2`. `min` and `max` are what a slider offers when the
    /// part is placed, not what is valid.
    Number {
        expression: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        min: Option<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        max: Option<f64>,
    },
    /// A family of variants, one per row, of which `selected` is the one
    /// built: its name is the parameter's value, and `name.column` its
    /// value in `column`.
    Table {
        columns: Vec<String>,
        rows: Vec<Row>,
        selected: String,
    },
}

/// A named parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    #[serde(flatten)]
    pub kind: ParameterKind,
}

/// A number parameter, as its [`crate::parameters::steps::DefineParameters`] step holds
/// it: a formula of the parameters defined before it — `12`, `width / 2`.
/// `min` and `max` are what a slider offers when the part is placed, not
/// what is valid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NumberParameter {
    pub name: String,
    pub expression: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
}

/// A program's parameters and properties, as an editor lists and edits them
/// together: the part's colour and material, what it is called and ordered
/// as ([`crate::parameters::steps::PartProperties`]), and the values it is designed
/// with ([`crate::parameters::steps::DefineParameters`]), in order.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Parameters {
    /// The part's colour, `#rrggbb`; none for the viewer's own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    /// What the part is made of; none for a part whose material is not
    /// given, which has a volume and a centroid but no mass.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material: Option<Material>,
    /// What the part is, in words — `ISO 4762 socket head cap screw` —
    /// where it is listed: a bill of materials, a drawing's. None for a
    /// part known by its file's name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// What the part is ordered as, before the values it is built with
    /// (see [`Parameters::designate`]): a norm — `ISO 4762`, for `ISO 4762
    /// M4x12` — or a catalogue or part number. None for a part that is
    /// made rather than bought.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub designation: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub values: Vec<Parameter>,
}

/// Whether `name` can name a parameter: a letter or `_`, then letters,
/// digits and `_` — so that a formula reads it as one, and `name.column`
/// names a table's column without ambiguity.
pub fn validate_name(name: &str) -> GeopResult<()> {
    let mut chars = name.chars();
    let ok = chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_');
    if !ok {
        return Err(GeopError::new(format!(
            "{name:?} is no parameter name: use letters, digits and _, starting with a letter"
        )));
    }
    if FUNCTION_NAMES.contains(&name) || name == "pi" || name == COLOR {
        return Err(GeopError::new(format!("{name:?} is a reserved name")));
    }
    Ok(())
}

/// Whether `color` is a colour, `#rrggbb`.
pub fn validate_color(color: &str) -> GeopResult<()> {
    let hex = color.strip_prefix('#').unwrap_or("");
    if hex.len() != 6 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(GeopError::new(format!(
            "{color:?} is no colour of the form #rrggbb"
        )));
    }
    Ok(())
}

impl NumberParameter {
    /// The number parameter `name`, `expression` by default — a formula of
    /// those before it, or a value — offered from `min` to `max`.
    pub fn new(name: &str, expression: &str, min: f64, max: f64) -> Self {
        Self {
            name: name.into(),
            expression: expression.into(),
            min: Some(min),
            max: Some(max),
        }
    }
}

impl Parameter {
    /// The number parameter `name`, `expression` by default — a formula of
    /// those before it, or a value — offered from `min` to `max`.
    pub fn number(name: &str, expression: &str, min: f64, max: f64) -> Self {
        Self {
            name: name.into(),
            kind: ParameterKind::Number {
                expression: expression.into(),
                min: Some(min),
                max: Some(max),
            },
        }
    }
}

impl Parameters {
    pub fn is_empty(&self) -> bool {
        self.color.is_none()
            && self.material.is_none()
            && self.title.is_none()
            && self.designation.is_none()
            && self.values.is_empty()
    }

    /// The parameter `name`.
    pub fn get(&self, name: &str) -> Option<&Parameter> {
        self.values.iter().find(|p| p.name == name)
    }

    /// Checks every name — valid, and unique — every table, and the colour.
    /// Formulas are checked as they are resolved: one that fails is said
    /// for that parameter, not for the program.
    pub fn validate(&self) -> GeopResult<()> {
        if let Some(color) = &self.color {
            validate_color(color)?;
        }
        if let Some(Material { name, density }) = &self.material
            && !(density.is_finite() && *density > 0.0)
        {
            return Err(GeopError::new(format!(
                "the material {name:?} has a density of {density} kg/m³: it must be a positive number"
            )));
        }
        for (i, p) in self.values.iter().enumerate() {
            validate_name(&p.name)?;
            if self.values[..i].iter().any(|q| q.name == p.name) {
                return Err(GeopError::new(format!(
                    "there is more than one parameter {:?}",
                    p.name
                )));
            }
            if let ParameterKind::Table {
                columns,
                rows,
                selected,
            } = &p.kind
            {
                for c in columns {
                    validate_name(c)
                        .map_err(|e| e.with_context(format!("column of the table {:?}", p.name)))?;
                }
                for row in rows {
                    if row.values.len() != columns.len() {
                        return Err(GeopError::new(format!(
                            "row {:?} of the table {:?} has {} values for {} columns",
                            row.name,
                            p.name,
                            row.values.len(),
                            columns.len()
                        )));
                    }
                }
                if !rows.iter().any(|r| &r.name == selected) {
                    return Err(GeopError::new(format!(
                        "the table {:?} has no row {selected:?} to select",
                        p.name
                    )));
                }
            }
        }
        Ok(())
    }
}
