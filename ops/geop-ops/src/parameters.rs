//! [`Parameters`]: the named values a program's design is given by —
//! numbers, tables of variants, the part's colour and what it is made of —
//! and [`evaluate`], the formulas that read them.
//!
//! A parameter is defined once, in the program, and read by name wherever
//! a value is typed: a sketch dimension of `width / 2`, a number parameter
//! of `2 * wall`. A table is a family of variants — screw sizes, say — of
//! which one row is selected: `screw` is that row's name, and
//! `screw.diameter` its value in the column `diameter`.
//!
//! What a parameter is defined as is the program's own; what it is *built
//! with* may be overridden by a program placing the part, by the same name
//! (see [`crate::part::State`] and [`crate::Library::component`]). So a
//! placed screw is made M5 by the program placing it, without touching the
//! screw's own file. [`Parameters::resolve`] turns the definitions and the
//! overrides into the values a build reads.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use serde::{Deserialize, Serialize};

use crate::{Design, part::ParamValue, part::State};

/// The name the part's colour is read by.
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
pub const MATERIALS: [(&str, f64); 10] = [
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
    /// Where a placed part is: a pose, in the frame of the part it is
    /// placed in. Not defined by a formula but given — by a program, or by
    /// the solve of the mates (see [`crate::assembly`]).
    Pose,
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

/// The values a program's parameters resolve to, and why those that do
/// not resolve fail, by name.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Resolved {
    pub values: State,
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
    if FUNCTIONS.iter().any(|(f, _)| *f == name) || name == "pi" || name == COLOR {
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
    /// The parameter `name` that is where a placed part is.
    pub fn pose(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            kind: ParameterKind::Pose,
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
    pub fn designate(&self, values: &State) -> Option<String> {
        let mut words = vec![self.designation.clone()?];
        for parameter in &self.values {
            match (&parameter.kind, values.get(&parameter.name)) {
                (ParameterKind::Table { .. }, Some(ParamValue::Text(row))) => {
                    words.push(row.clone())
                }
                (ParameterKind::Number { .. }, Some(ParamValue::Number(n))) => {
                    words.push(format!("{} {}", parameter.name, n.to_f64()))
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

    /// The values the parameters are built with, `overrides` — what a
    /// program placing the part gives it — taking the place of their own:
    /// a number by value, a table by the name of its row, the colour by
    /// itself. A number's formula may read any other parameter, defined
    /// before it or after: the tables first, then the numbers whose
    /// formulas can be evaluated, again and again while that resolves
    /// more. What is left — reading a parameter there is none of, or one
    /// that reads it back — fails, saying why.
    pub fn resolve(&self, overrides: &State) -> Resolved {
        let mut out = Resolved::default();
        let color = match overrides.get(COLOR) {
            Some(ParamValue::Text(c)) if validate_color(c).is_ok() => Some(c.clone()),
            _ => self.color.clone(),
        };
        if let Some(color) = color {
            out.values.insert(COLOR.into(), ParamValue::Text(color));
        }
        let mut numbers = Vec::new();
        for p in &self.values {
            match &p.kind {
                ParameterKind::Number { expression, .. } => match overrides.get(&p.name) {
                    Some(ParamValue::Number(v)) => {
                        out.values.insert(p.name.clone(), ParamValue::Number(*v));
                    }
                    _ => numbers.push((&p.name, expression)),
                },
                // Given, not defined: nothing to resolve.
                ParameterKind::Pose => {}
                ParameterKind::Table {
                    columns,
                    rows,
                    selected,
                } => {
                    let wanted = match overrides.get(&p.name) {
                        Some(ParamValue::Text(row)) => row,
                        _ => selected,
                    };
                    let Some(row) = rows
                        .iter()
                        .find(|r| &r.name == wanted)
                        .or_else(|| rows.iter().find(|r| &r.name == selected))
                    else {
                        out.errors
                            .insert(p.name.clone(), format!("the table has no row {selected:?}"));
                        continue;
                    };
                    out.values
                        .insert(p.name.clone(), ParamValue::Text(row.name.clone()));
                    for (column, value) in columns.iter().zip(&row.values) {
                        out.values.insert(
                            format!("{}.{column}", p.name),
                            ParamValue::Number(Design::from_f64(*value)),
                        );
                    }
                }
            }
        }
        // Each pass evaluates what it can; one that resolves nothing more
        // leaves what never will.
        while !numbers.is_empty() {
            let count = numbers.len();
            let mut left = Vec::new();
            let mut failures = Vec::new();
            for (name, expression) in numbers {
                match evaluate(expression, |n| number(&out.values, n)) {
                    Ok(v) => {
                        out.values
                            .insert(name.clone(), ParamValue::Number(Design::from_f64(v)));
                    }
                    Err(e) => {
                        left.push((name, expression));
                        failures.push((name, e));
                    }
                }
            }
            if left.len() == count {
                for (name, e) in failures {
                    out.errors
                        .insert(name.clone(), e.root_message().to_string());
                }
                break;
            }
            numbers = left;
        }
        out
    }
}

/// The number `name` in `values`, if it is one.
pub fn number(values: &State, name: &str) -> Option<f64> {
    match values.get(name) {
        Some(ParamValue::Number(v)) => Some(v.to_f64()),
        _ => None,
    }
}

/// The functions a formula can call, by name, with how many arguments.
/// Angles are in degrees, as everywhere a designer types one.
const FUNCTIONS: [(&str, usize); 13] = [
    ("sqrt", 1),
    ("abs", 1),
    ("sin", 1),
    ("cos", 1),
    ("tan", 1),
    ("asin", 1),
    ("acos", 1),
    ("atan", 1),
    ("round", 1),
    ("floor", 1),
    ("ceil", 1),
    ("min", 2),
    ("max", 2),
];

fn call(f: &str, args: &[f64]) -> f64 {
    let rad = |x: f64| x.to_radians();
    match (f, args) {
        ("sqrt", [x]) => x.sqrt(),
        ("abs", [x]) => x.abs(),
        ("sin", [x]) => rad(*x).sin(),
        ("cos", [x]) => rad(*x).cos(),
        ("tan", [x]) => rad(*x).tan(),
        ("asin", [x]) => x.asin().to_degrees(),
        ("acos", [x]) => x.acos().to_degrees(),
        ("atan", [x]) => x.atan().to_degrees(),
        ("round", [x]) => x.round(),
        ("floor", [x]) => x.floor(),
        ("ceil", [x]) => x.ceil(),
        ("min", [a, b]) => a.min(*b),
        ("max", [a, b]) => a.max(*b),
        _ => unreachable!("arity checked against FUNCTIONS"),
    }
}

/// Whether `text` is a formula rather than a plain number: a plain number
/// is a value typed, a formula a value that follows what it reads.
pub fn is_formula(text: &str) -> bool {
    text.trim().parse::<f64>().is_err()
}

/// A number an operation is given — a length, an angle, a count: a plain
/// value, or a formula of the part's parameters that it follows, as a
/// sketch's dimensions can be (see [`evaluate`]). Serialized as a number
/// when plain and as the formula's text otherwise — `12.5`, `"width / 2"` —
/// so a file written before an argument took formulas reads the same.
///
/// A step reads it with [`Formula::evaluate`], which declares every
/// parameter the formula reads: a change to one of them rebuilds the step,
/// a change to any other does not (see [`crate::ProgramRunner`]).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Formula {
    Plain(f64),
    Expression(String),
}

impl From<f64> for Formula {
    fn from(value: f64) -> Self {
        Formula::Plain(value)
    }
}

impl From<&str> for Formula {
    /// `text` as typed: plain if it is a number, a formula otherwise.
    fn from(text: &str) -> Self {
        let text = text.trim();
        match text.parse() {
            Ok(value) => Formula::Plain(value),
            Err(_) => Formula::Expression(text.to_string()),
        }
    }
}

impl From<String> for Formula {
    fn from(text: String) -> Self {
        Formula::from(text.as_str())
    }
}

impl std::fmt::Display for Formula {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Formula::Plain(value) => write!(f, "{value}"),
            Formula::Expression(text) => f.write_str(text),
        }
    }
}

impl Formula {
    /// Its value if it is plain; `None` for a formula.
    pub fn plain(&self) -> Option<f64> {
        match self {
            Formula::Plain(value) => Some(*value),
            Formula::Expression(_) => None,
        }
    }

    /// The formula it follows, if it is one.
    pub fn expression(&self) -> Option<&str> {
        match self {
            Formula::Plain(_) => None,
            Formula::Expression(text) => Some(text),
        }
    }

    /// The formula it follows, to change, if it is one.
    pub fn expression_mut(&mut self) -> Option<&mut String> {
        match self {
            Formula::Plain(_) => None,
            Formula::Expression(text) => Some(text),
        }
    }

    /// Its value as the step building `part` reads it: every parameter
    /// the formula reads declared read (see [`crate::Part::evaluate`]).
    pub fn evaluate<S: Scalar>(&self, part: &mut crate::Part<S>) -> GeopResult<f64> {
        match self {
            Formula::Plain(value) => Ok(*value),
            Formula::Expression(text) => part.evaluate(text),
        }
    }

    /// Its value with the parameter values `inputs`, declaring nothing:
    /// what a dialog shows, and what a handle is drawn at.
    pub fn peek(&self, inputs: &State) -> GeopResult<f64> {
        match self {
            Formula::Plain(value) => Ok(*value),
            Formula::Expression(text) => evaluate(text, |name| number(inputs, name)),
        }
    }
}

/// The texts of those of `formulas` that are formulas, not plain numbers:
/// what an operation lists as its formulas (see
/// [`crate::operation::Operation::formulas`]).
pub fn expressions<'a>(formulas: impl IntoIterator<Item = &'a mut Formula>) -> Vec<&'a mut String> {
    formulas
        .into_iter()
        .filter_map(Formula::expression_mut)
        .collect()
}

/// The parameter a name in a formula reads: itself, or for a table's
/// column, `size.diameter`, the table `size`.
pub fn parameter_of(name: &str) -> &str {
    name.split('.').next().unwrap_or(name)
}

/// `expression` reading the parameter `to` wherever it read `from` — and,
/// for a table, `to.column` for `from.column`. Numbers, functions and
/// everything else stay as they are written.
pub fn rename_in(expression: &str, from: &str, to: &str) -> String {
    let mut out = String::with_capacity(expression.len());
    let mut rest = expression;
    while let Some(c) = rest.chars().next() {
        let len = if c.is_ascii_digit() || c == '.' {
            number_len(rest)
        } else if c.is_ascii_alphabetic() || c == '_' {
            let len = name_len(rest);
            let name = &rest[..len];
            match name.strip_prefix(from) {
                Some(column) if column.is_empty() || column.starts_with('.') => {
                    out.push_str(to);
                    out.push_str(column);
                }
                _ => out.push_str(name),
            }
            rest = &rest[len..];
            continue;
        } else {
            c.len_utf8()
        };
        out.push_str(&rest[..len]);
        rest = &rest[len..];
    }
    out
}

/// How long the number `text` starts with is: digits and points, then
/// perhaps an exponent, `1e-3`.
fn number_len(text: &str) -> usize {
    let mut len = text
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(text.len());
    let rest = &text[len..];
    if rest.starts_with(['e', 'E']) {
        let digits = rest[1..].trim_start_matches(['+', '-']);
        if digits.starts_with(|c: char| c.is_ascii_digit()) {
            len += rest.len() - digits.len();
            len += digits
                .find(|c: char| !c.is_ascii_digit())
                .unwrap_or(digits.len());
        }
    }
    len
}

/// How long the name `text` starts with is: letters, digits, `_` and `.`.
fn name_len(text: &str) -> usize {
    text.find(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '.'))
        .unwrap_or(text.len())
}

/// The names of parameters `expression` reads, in order, each once.
pub fn names_in(expression: &str) -> Vec<String> {
    let mut names = Vec::new();
    let _ = evaluate(expression, |name| {
        if !names.iter().any(|n| n == name) {
            names.push(name.to_string());
        }
        Some(1.0)
    });
    names
}

/// The value of the formula `expression`, with `lookup` the value of each
/// parameter it names: numbers, `+ - * / ^`, parentheses, `pi`, and the
/// functions `sqrt abs sin cos tan asin acos atan round floor ceil min
/// max`, with angles in degrees. Fails for anything else, for a name
/// `lookup` does not know, and for a value that is no finite number.
pub fn evaluate(expression: &str, lookup: impl FnMut(&str) -> Option<f64>) -> GeopResult<f64> {
    let mut parser = Parser {
        text: expression,
        at: 0,
        lookup,
    };
    let ctx = |e: GeopError| e.with_context(format!("evaluating {expression:?}"));
    let value = parser.sum().map_err(ctx)?;
    parser.skip_space();
    if parser.at < expression.len() {
        return Err(ctx(GeopError::new(format!(
            "unexpected {:?}",
            &expression[parser.at..]
        ))));
    }
    if !value.is_finite() {
        return Err(ctx(GeopError::new(format!("{value} is no finite number"))));
    }
    Ok(value)
}

/// A recursive-descent reader of a formula.
struct Parser<'t, F> {
    text: &'t str,
    at: usize,
    lookup: F,
}

impl<F: FnMut(&str) -> Option<f64>> Parser<'_, F> {
    fn skip_space(&mut self) {
        while self.peek().is_some_and(char::is_whitespace) {
            self.at += 1;
        }
    }

    fn peek(&self) -> Option<char> {
        self.text[self.at..].chars().next()
    }

    /// Takes `c` if it comes next.
    fn eat(&mut self, c: char) -> bool {
        self.skip_space();
        if self.peek() == Some(c) {
            self.at += c.len_utf8();
            true
        } else {
            false
        }
    }

    fn sum(&mut self) -> GeopResult<f64> {
        let mut v = self.product()?;
        loop {
            if self.eat('+') {
                v += self.product()?;
            } else if self.eat('-') {
                v -= self.product()?;
            } else {
                return Ok(v);
            }
        }
    }

    fn product(&mut self) -> GeopResult<f64> {
        let mut v = self.unary()?;
        loop {
            if self.eat('*') {
                v *= self.unary()?;
            } else if self.eat('/') {
                v /= self.unary()?;
            } else {
                return Ok(v);
            }
        }
    }

    fn unary(&mut self) -> GeopResult<f64> {
        if self.eat('-') {
            return Ok(-self.unary()?);
        }
        if self.eat('+') {
            return self.unary();
        }
        let base = self.atom()?;
        if self.eat('^') {
            return Ok(base.powf(self.unary()?));
        }
        Ok(base)
    }

    fn atom(&mut self) -> GeopResult<f64> {
        self.skip_space();
        if self.eat('(') {
            let v = self.sum()?;
            if !self.eat(')') {
                return Err(GeopError::new("a ( is not closed"));
            }
            return Ok(v);
        }
        let start = self.at;
        match self.peek() {
            Some(c) if c.is_ascii_digit() || c == '.' => {
                self.at += number_len(&self.text[start..]);
                let number = &self.text[start..self.at];
                number
                    .parse()
                    .map_err(|_| GeopError::new(format!("{number:?} is no number")))
            }
            Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                self.at += name_len(&self.text[start..]);
                let name = &self.text[start..self.at];
                if self.eat('(') {
                    let Some(&(_, arity)) = FUNCTIONS.iter().find(|(f, _)| *f == name) else {
                        return Err(GeopError::new(format!("there is no function {name:?}")));
                    };
                    let mut args = vec![self.sum()?];
                    while self.eat(',') {
                        args.push(self.sum()?);
                    }
                    if !self.eat(')') {
                        return Err(GeopError::new(format!("{name}( is not closed")));
                    }
                    if args.len() != arity {
                        return Err(GeopError::new(format!(
                            "{name} takes {arity} argument(s), not {}",
                            args.len()
                        )));
                    }
                    return Ok(call(name, &args));
                }
                if name == "pi" {
                    return Ok(std::f64::consts::PI);
                }
                (self.lookup)(name)
                    .ok_or_else(|| GeopError::new(format!("there is no parameter {name:?}")))
            }
            Some(c) => Err(GeopError::new(format!("unexpected {c:?}"))),
            None => Err(GeopError::new("the formula ends too early")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(text: &str) -> GeopResult<f64> {
        evaluate(text, |name| match name {
            "width" => Some(10.0),
            "screw.d" => Some(5.0),
            _ => None,
        })
    }

    #[test]
    fn formulas_evaluate() {
        assert_eq!(eval("1 + 2 * 3").unwrap(), 7.0);
        assert_eq!(eval("(1 + 2) * 3").unwrap(), 9.0);
        assert_eq!(eval("-2 ^ 2").unwrap(), -4.0);
        assert_eq!(eval("2 ^ 3 ^ 2").unwrap(), 512.0);
        assert_eq!(eval("width / 2 - screw.d").unwrap(), 0.0);
        assert_eq!(eval("max(width, 3) + min(1, 2)").unwrap(), 11.0);
        assert!((eval("cos(60)").unwrap() - 0.5).abs() < 1e-12);
        assert_eq!(eval("1.5e1").unwrap(), 15.0);
        assert!((eval("2*pi").unwrap() - std::f64::consts::TAU).abs() < 1e-12);
        for bad in [
            "",
            "1 +",
            "(1",
            "height",
            "sqrt(1, 2)",
            "1 / 0",
            "2 3",
            "foo(1)",
        ] {
            assert!(eval(bad).is_err(), "{bad:?}");
        }
        assert_eq!(names_in("width * screw.d + width"), ["width", "screw.d"]);
        assert!(!is_formula(" 12.5 ") && is_formula("width"));
    }

    /// A formula typed is plain if it is a number, and saves as one; any
    /// other saves as its text. Both read back as they were, and evaluate
    /// against the parameters.
    #[test]
    fn formulas_save_as_numbers_or_text() {
        let plain = Formula::from(" 12.5 ");
        let formula = Formula::from(" width / 2 ");
        assert_eq!(plain, Formula::Plain(12.5));
        assert_eq!(formula, Formula::Expression("width / 2".into()));
        assert_eq!(serde_json::to_string(&plain).unwrap(), "12.5");
        assert_eq!(serde_json::to_string(&formula).unwrap(), r#""width / 2""#);
        for f in [&plain, &formula] {
            let back: Formula = serde_json::from_str(&serde_json::to_string(f).unwrap()).unwrap();
            assert_eq!(&back, f);
        }
        let inputs = State::from([(
            "width".to_string(),
            ParamValue::Number(Design::from_f64(10.0)),
        )]);
        assert_eq!(plain.peek(&inputs).unwrap(), 12.5);
        assert_eq!(formula.peek(&inputs).unwrap(), 5.0);
        assert!(Formula::from("depth").peek(&inputs).is_err());
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
        let own = parameters.resolve(&State::new());
        assert!((number(&own.values, "hole").unwrap() - 3.2).abs() < 1e-12);
        assert_eq!(own.values[COLOR], ParamValue::Text("#ff8800".into()));
        assert!(own.errors.contains_key("broken"));

        let overrides = State::from([
            ("screw".to_string(), ParamValue::Text("M5".into())),
            (COLOR.to_string(), ParamValue::Text("#000000".into())),
        ]);
        let placed = parameters.resolve(&overrides);
        assert_eq!(number(&placed.values, "screw.head"), Some(8.5));
        assert!((number(&placed.values, "hole").unwrap() - 5.2).abs() < 1e-12);
        assert_eq!(placed.values[COLOR], ParamValue::Text("#000000".into()));

        // Read before it is defined: the same.
        let mut reordered = parameters.clone();
        reordered.values.rotate_left(1);
        let later = reordered.resolve(&State::new());
        assert_eq!(number(&later.values, "hole"), number(&own.values, "hole"));
        // Reading one that never resolves: it fails too, saying so.
        let mut cycle = parameters.clone();
        cycle.values[1].kind = ParameterKind::Number {
            expression: "broken + 1".into(),
            min: None,
            max: None,
        };
        let cyclic = cycle.resolve(&State::new());
        assert!(cyclic.errors.contains_key("hole") && cyclic.errors.contains_key("broken"));

        let mut bad = parameters.clone();
        bad.values[1].name = "screw".into();
        assert!(bad.validate().is_err());
        bad.values[1].name = "sin".into();
        assert!(bad.validate().is_err());
    }

    /// A rename touches the name and a table's columns, and nothing that
    /// only looks alike: a longer name, a number's exponent, a function.
    #[test]
    fn renaming_reads_names_as_the_parser_does() {
        assert_eq!(
            rename_in("w/2 + widths + 1e5 + w.col*min(w, 2e-3)", "w", "width"),
            "width/2 + widths + 1e5 + width.col*min(width, 2e-3)"
        );
        assert_eq!(rename_in("e + 2e3 + E", "e", "x"), "x + 2e3 + E");
        assert_eq!(names_in("2e3 * e"), ["e"]);
    }
}
