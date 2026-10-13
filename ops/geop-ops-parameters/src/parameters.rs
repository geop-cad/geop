//! [`Parameters`]: the named values a program's design is given by —
//! numbers, tables of variants, the part's colour and what it is made of.
//!
//! A parameter is defined once, in the program, and read by name wherever
//! a number is typed ([`NumericInput`](geop_core_editor::numeric_input::NumericInput)):
//! a sketch dimension of `width / 2`, a number parameter of `2 * wall`. A
//! table is a family of variants — screw sizes, say — of which one row is
//! selected: `screw.diameter` is that row's value in the column
//! `diameter`.
//!
//! What a parameter is defined as is the program's own; what it is *built
//! with* may be overridden by a program placing the part, by the same name
//! (see [`Settings`]). So a placed screw is made M5 by the program placing
//! it, without touching the screw's own file. [`Parameters::resolve`] turns
//! the definitions and the overrides into what a build is given: numbers,
//! its inputs, and the rows and the colour chosen, which the part's
//! [`About`](crate::about::About) says.

use std::collections::BTreeMap;

use geop_core_editor::numeric_input::{FUNCTION_NAMES, evaluate, rename_in};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::repeat,
};
use serde::{Deserialize, Serialize};

use crate::Design;

/// What a parameter is set to where a program places the part: a number,
/// or the name of a table's row, or a colour. Saved as itself — `2.5`,
/// `"M5"`, `"#ff8800"`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Setting {
    Number(f64),
    Text(String),
}

/// Parameter settings, by name.
pub type Settings = BTreeMap<String, Setting>;

/// The name the part's colour is set by.
pub const COLOR: &str = "color";

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

/// One row of a [`ParameterKind::Table`]: a variant, by name, with a value
/// per column.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Row {
    pub name: String,
    pub values: Vec<f64>,
}

/// What a parameter is.
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

/// A program's parameters: the part's colour and material, what it is
/// called when it is ordered, and the values it is designed with, in order
/// — each may read those before it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Parameters {
    /// The part's colour, `#rrggbb`; none for the viewer's own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    /// What the part is made of; none for a part whose material is not
    /// given — weighed as water, 1000 kg/m³, and said so.
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

/// What a program's parameters resolve to: the numbers a build is given
/// as its inputs — a number parameter's value, a table's columns as
/// `table.column` — and what else was chosen: each table's row, by the
/// table's name, and the colour. Why those that do not resolve fail, by
/// name.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Resolved {
    pub numbers: BTreeMap<String, Design>,
    pub chosen: Settings,
    pub errors: BTreeMap<String, String>,
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

/// Whether `color` is a colour as [`Parameters::color`] holds one.
pub fn validate_color(color: &str) -> GeopResult<()> {
    let hex = color.strip_prefix('#').unwrap_or("");
    if hex.len() != 6 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(GeopError::new(format!(
            "{color:?} is no colour of the form #rrggbb"
        )));
    }
    Ok(())
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

    /// What the part built with the parameter values `values` is ordered
    /// as: its [`Parameters::designation`], then each parameter in order —
    /// a table's row by its name, a number after its own — `ISO 4762
    /// M4x12`, `T-slot 2020 length 500`. None for a part with no
    /// designation.
    pub fn designate(&self, values: &Settings) -> Option<String> {
        let mut words = vec![self.designation.clone()?];
        for parameter in &self.values {
            match (&parameter.kind, values.get(&parameter.name)) {
                (ParameterKind::Table { .. }, Some(Setting::Text(row))) => words.push(row.clone()),
                (ParameterKind::Number { .. }, Some(Setting::Number(n))) => {
                    words.push(format!("{} {n}", parameter.name))
                }
                _ => {}
            }
        }
        Some(words.join(" "))
    }

    /// The parameter `name`.
    pub fn get(&self, name: &str) -> Option<&Parameter> {
        self.values.iter().find(|p| p.name == name)
    }

    /// The parameter `from` named `to`, and every formula of the others
    /// reading it reading `to` (see [`rename_in`]).
    pub fn rename(&mut self, from: &str, to: &str) {
        for p in &mut self.values {
            if p.name == from {
                p.name = to.to_string();
            }
            if let ParameterKind::Number { expression, .. } = &mut p.kind {
                *expression = rename_in(expression, from, to);
            }
        }
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

    /// What the parameters are built with, `overrides` — what a program
    /// placing the part gives it — taking the place of their own: a number
    /// by value, a table by the name of its row, the colour by itself. A
    /// number's formula may read any other parameter, defined before it or
    /// after: the tables first, then the numbers whose formulas can be
    /// evaluated, again and again while that resolves more. What is left —
    /// reading a parameter there is none of, or one that reads it back —
    /// fails, saying why.
    pub fn resolve(&self, overrides: &Settings) -> Resolved {
        let mut out = Resolved::default();
        let color = match overrides.get(COLOR) {
            Some(Setting::Text(c)) if validate_color(c).is_ok() => Some(c.clone()),
            _ => self.color.clone(),
        };
        if let Some(color) = color {
            out.chosen.insert(COLOR.into(), Setting::Text(color));
        }
        let mut numbers = Vec::new();
        for p in &self.values {
            match &p.kind {
                ParameterKind::Number { expression, .. } => match overrides.get(&p.name) {
                    Some(&Setting::Number(v)) => {
                        out.numbers.insert(p.name.clone(), Design::from_f64(v));
                        out.chosen.insert(p.name.clone(), Setting::Number(v));
                    }
                    _ => numbers.push((&p.name, expression)),
                },
                ParameterKind::Table {
                    columns,
                    rows,
                    selected,
                } => {
                    let wanted = match overrides.get(&p.name) {
                        Some(Setting::Text(row)) => row,
                        _ => selected,
                    };
                    let Some(row) = (rows.iter().find(|r| &r.name == wanted))
                        .or_else(|| rows.iter().find(|r| &r.name == selected))
                    else {
                        out.errors
                            .insert(p.name.clone(), format!("the table has no row {selected:?}"));
                        continue;
                    };
                    out.chosen
                        .insert(p.name.clone(), Setting::Text(row.name.clone()));
                    for (column, value) in columns.iter().zip(&row.values) {
                        out.numbers
                            .insert(format!("{}.{column}", p.name), Design::from_f64(*value));
                    }
                }
            }
        }
        // Each pass evaluates what it can; one that resolves nothing more
        // leaves what never will.
        let unresolved = repeat::resolve(numbers, |(name, expression)| {
            let v = evaluate(expression, |n| out.numbers.get(n).map(|v| v.to_f64()))?;
            out.numbers.insert((*name).clone(), Design::from_f64(v));
            out.chosen.insert((*name).clone(), Setting::Number(v));
            Ok::<_, GeopError>(())
        });
        for ((name, _), e) in unresolved {
            out.errors
                .insert(name.clone(), e.root_message().to_string());
        }
        out
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn number(resolved: &Resolved, name: &str) -> f64 {
        resolved.numbers[name].to_f64()
    }

    /// Numbers read what is defined before or after them, a table gives
    /// its row's values by column, and overrides take the place of
    /// definitions — the numbers reading them following.
    #[test]
    fn parameters_resolve_in_order_with_overrides() {
        let parameters = Parameters {
            title: None,
            designation: None,
            color: Some("#ff8800".into()),
            material: None,
            values: vec![
                Parameter {
                    name: "screw".into(),
                    kind: ParameterKind::Table {
                        columns: vec!["d".into(), "head".into()],
                        rows: vec![
                            Row {
                                name: "M3".into(),
                                values: vec![3.0, 5.5],
                            },
                            Row {
                                name: "M5".into(),
                                values: vec![5.0, 8.5],
                            },
                        ],
                        selected: "M3".into(),
                    },
                },
                Parameter {
                    name: "hole".into(),
                    kind: ParameterKind::Number {
                        expression: "screw.d + 0.2".into(),
                        min: None,
                        max: None,
                    },
                },
                Parameter {
                    name: "broken".into(),
                    kind: ParameterKind::Number {
                        expression: "later * 2".into(),
                        min: None,
                        max: None,
                    },
                },
            ],
        };
        parameters.validate().unwrap();
        let own = parameters.resolve(&Settings::new());
        assert!((number(&own, "hole") - 3.2).abs() < 1e-12);
        assert_eq!(own.chosen[COLOR], Setting::Text("#ff8800".into()));
        assert!(own.errors.contains_key("broken"));

        let overrides = Settings::from([
            ("screw".to_string(), Setting::Text("M5".into())),
            (COLOR.to_string(), Setting::Text("#000000".into())),
        ]);
        let placed = parameters.resolve(&overrides);
        assert_eq!(number(&placed, "screw.head"), 8.5);
        assert!((number(&placed, "hole") - 5.2).abs() < 1e-12);
        assert_eq!(placed.chosen[COLOR], Setting::Text("#000000".into()));
        assert_eq!(placed.chosen["screw"], Setting::Text("M5".into()));

        // Read before it is defined: the same.
        let mut reordered = parameters.clone();
        reordered.values.rotate_left(1);
        let later = reordered.resolve(&Settings::new());
        assert_eq!(number(&later, "hole"), number(&own, "hole"));
        // Reading one that never resolves: it fails too, saying so.
        let mut cycle = parameters.clone();
        cycle.values[1].kind = ParameterKind::Number {
            expression: "broken + 1".into(),
            min: None,
            max: None,
        };
        let cyclic = cycle.resolve(&Settings::new());
        assert!(cyclic.errors.contains_key("hole") && cyclic.errors.contains_key("broken"));

        let mut bad = parameters.clone();
        bad.values[1].name = "screw".into();
        assert!(bad.validate().is_err());
        bad.values[1].name = "sin".into();
        assert!(bad.validate().is_err());
    }

    /// A designation lists the rows and the numbers a part is built with.
    #[test]
    fn a_designation_lists_what_the_part_is_built_with() {
        let parameters = Parameters {
            designation: Some("ISO 4762".into()),
            values: vec![Parameter::number("length", "12", 4.0, 40.0)],
            ..Parameters::default()
        };
        let resolved = parameters.resolve(&Settings::new());
        assert_eq!(
            parameters.designate(&resolved.chosen).as_deref(),
            Some("ISO 4762 length 12")
        );
    }
}
