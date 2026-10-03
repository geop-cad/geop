//! [`Parameters`]: the named values a program's design is given by —
//! numbers, tables of variants and the part's colour — and [`evaluate`],
//! the formulas that read them.
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

/// A program's parameters: the part's colour, and the values it is
/// designed with, in order — each may read those before it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Parameters {
    /// The part's colour, `#rrggbb`; none for the viewer's own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
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

impl Parameters {
    pub fn is_empty(&self) -> bool {
        self.color.is_none() && self.values.is_empty()
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
                while self.peek().is_some_and(|c| c.is_ascii_digit() || c == '.') {
                    self.at += 1;
                }
                // An exponent: `1e-3`.
                let rest = &self.text[self.at..];
                if rest.starts_with(['e', 'E']) {
                    let digits = rest[1..].trim_start_matches(['+', '-']);
                    if digits.starts_with(|c: char| c.is_ascii_digit()) {
                        self.at += rest.len() - digits.len();
                        while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                            self.at += 1;
                        }
                    }
                }
                let number = &self.text[start..self.at];
                number
                    .parse()
                    .map_err(|_| GeopError::new(format!("{number:?} is no number")))
            }
            Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                while self
                    .peek()
                    .is_some_and(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.')
                {
                    self.at += 1;
                }
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

    /// Numbers read what is defined before or after them, a table gives
    /// its row's values by column, and overrides take the place of
    /// definitions — the numbers reading them following.
    #[test]
    fn parameters_resolve_in_order_with_overrides() {
        let parameters = Parameters {
            color: Some("#ff8800".into()),
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
}
