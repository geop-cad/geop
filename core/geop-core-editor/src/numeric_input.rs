//! [`NumericInput`]: a number an operation is given — a length, an angle, a
//! count — typed into a dialog's number field as a plain value or as a
//! formula of the program's parameters, which it follows ([`evaluate`]).
//! An operation's arguments take one wherever they take a number.
//!
//! A step resolves it where it builds ([`NumericInput::resolve`]), reading each
//! parameter the formula names as an input of the part: the read is
//! recorded, so a change to one of them rebuilds the step, a change to any
//! other does not. Saved, a formula is told apart from every other value
//! in a step's arguments ([`formulas_in`], [`rename_parameter_in`]), so
//! what reads a parameter is found, and renamed, without asking the
//! operation.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::part::{Input, Reader, State};
use serde::{Deserialize, Serialize};

use crate::editing::Value;

/// A number an operation is given: a plain value, or a formula of the
/// program's parameters that it follows. Saved as a number when plain, and
/// as `{"formula": "width / 2"}` otherwise.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum NumericInput {
    Plain(f64),
    Formula { formula: String },
}

/// Zero, plain.
impl Default for NumericInput {
    fn default() -> Self {
        NumericInput::Plain(0.0)
    }
}

impl From<f64> for NumericInput {
    fn from(value: f64) -> Self {
        NumericInput::Plain(value)
    }
}

impl From<&str> for NumericInput {
    /// `text` as typed: plain if it is a number, a formula otherwise.
    fn from(text: &str) -> Self {
        let text = text.trim();
        match text.parse() {
            Ok(value) => NumericInput::Plain(value),
            Err(_) => NumericInput::Formula {
                formula: text.to_string(),
            },
        }
    }
}

impl From<String> for NumericInput {
    fn from(text: String) -> Self {
        NumericInput::from(text.as_str())
    }
}

impl std::fmt::Display for NumericInput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NumericInput::Plain(value) => write!(f, "{value}"),
            NumericInput::Formula { formula } => f.write_str(formula),
        }
    }
}

impl NumericInput {
    /// Its value if it is plain; `None` for a formula.
    pub fn plain(&self) -> Option<f64> {
        match self {
            NumericInput::Plain(value) => Some(*value),
            NumericInput::Formula { .. } => None,
        }
    }

    /// Its value, each parameter it names read as an input — a number —
    /// through `reader`: in a target's generator, every one of them
    /// recorded read.
    pub fn resolve<I: Input + Scalar, S: State>(&self, reader: &Reader<I, S>) -> GeopResult<f64> {
        match self {
            NumericInput::Plain(value) => Ok(*value),
            NumericInput::Formula { formula } => evaluate(formula, |name| {
                reader.retrieve_input(name).map(|input| input.to_f64())
            }),
        }
    }

    /// What a number field of it set to `value` makes it: a number slid or
    /// dragged plain, text typed plain if it is a number and a formula
    /// otherwise. `None` for anything else, and for empty text. A formula
    /// that does not evaluate is kept, as typed: the step then fails naming
    /// it, and the field shows the text to correct.
    pub fn from_value(value: &Value) -> Option<NumericInput> {
        match value {
            Value::Number(v) => Some(NumericInput::Plain(*v)),
            Value::Text(text) if !text.trim().is_empty() => Some(NumericInput::from(text.as_str())),
            _ => None,
        }
    }
}

/// The text of every formula in `args`, a step's saved arguments: what
/// tells which parameters the step reads (see [`names_in`]).
pub fn formulas_in(args: &serde_json::Value) -> Vec<&str> {
    let mut found = Vec::new();
    visit(args, &mut |formula| found.push(formula));
    found
}

/// `args`, a step's saved arguments, with every formula in it reading the
/// parameter `to` wherever it read `from` (see [`rename_in`]).
pub fn rename_parameter_in(args: &mut serde_json::Value, from: &str, to: &str) {
    visit_mut(args, &mut |formula| *formula = rename_in(formula, from, to));
}

/// The text of a saved formula, if `value` is one.
fn saved_formula(value: &serde_json::Value) -> Option<&str> {
    let object = value.as_object()?;
    (object.len() == 1).then_some(())?;
    object.get("formula")?.as_str()
}

fn visit<'a>(value: &'a serde_json::Value, found: &mut impl FnMut(&'a str)) {
    if let Some(formula) = saved_formula(value) {
        return found(formula);
    }
    match value {
        serde_json::Value::Array(items) => items.iter().for_each(|item| visit(item, found)),
        serde_json::Value::Object(fields) => fields.values().for_each(|field| visit(field, found)),
        _ => {}
    }
}

fn visit_mut(value: &mut serde_json::Value, edit: &mut impl FnMut(&mut String)) {
    if saved_formula(value).is_some()
        && let Some(serde_json::Value::String(formula)) = value.get_mut("formula")
    {
        return edit(formula);
    }
    match value {
        serde_json::Value::Array(items) => items.iter_mut().for_each(|item| visit_mut(item, edit)),
        serde_json::Value::Object(fields) => {
            fields.values_mut().for_each(|field| visit_mut(field, edit))
        }
        _ => {}
    }
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
            let (name, after) = rest.split_at(name_len(rest));
            match name.strip_prefix(from) {
                Some(column) if column.is_empty() || column.starts_with('.') => {
                    out.push_str(to);
                    out.push_str(column);
                }
                _ => out.push_str(name),
            }
            rest = after;
            continue;
        } else {
            c.len_utf8()
        };
        let (taken, after) = rest.split_at(len);
        out.push_str(taken);
        rest = after;
    }
    out
}

/// How long the number `text` starts with is: digits and points, then
/// perhaps an exponent, `1e-3`.
fn number_len(text: &str) -> usize {
    let mut len = text
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(text.len());
    let rest = text.get(len..).unwrap_or("");
    if let Some(exponent) = rest.strip_prefix(['e', 'E']) {
        let digits = exponent.trim_start_matches(['+', '-']);
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

/// A function a formula can call: of one argument or of two.
#[derive(Clone, Copy)]
enum Function {
    One(fn(f64) -> f64),
    Two(fn(f64, f64) -> f64),
}

/// The functions a formula can call, by name. Angles are in degrees, as
/// everywhere a designer types one.
pub const FUNCTION_NAMES: [&str; 13] = [
    "sqrt", "abs", "sin", "cos", "tan", "asin", "acos", "atan", "round", "floor", "ceil", "min",
    "max",
];

fn function(name: &str) -> Option<Function> {
    use Function::{One, Two};
    Some(match name {
        "sqrt" => One(f64::sqrt),
        "abs" => One(f64::abs),
        "sin" => One(|x| x.to_radians().sin()),
        "cos" => One(|x| x.to_radians().cos()),
        "tan" => One(|x| x.to_radians().tan()),
        "asin" => One(|x| x.asin().to_degrees()),
        "acos" => One(|x| x.acos().to_degrees()),
        "atan" => One(|x| x.atan().to_degrees()),
        "round" => One(f64::round),
        "floor" => One(f64::floor),
        "ceil" => One(f64::ceil),
        "min" => Two(f64::min),
        "max" => Two(f64::max),
        _ => return None,
    })
}

/// The value of the formula `expression`, with `lookup` the value of each
/// parameter it names: numbers, `+ - * / ^`, parentheses, `pi`, and the
/// functions of [`FUNCTION_NAMES`], with angles in degrees. Fails for
/// anything else, for a name `lookup` does not know, and for a value that
/// is no finite number.
pub fn evaluate(expression: &str, lookup: impl FnMut(&str) -> Option<f64>) -> GeopResult<f64> {
    let mut parser = Parser {
        rest: expression,
        lookup,
    };
    let ctx = |e: GeopError| e.with_context(format!("evaluating {expression:?}"));
    let value = parser.sum().map_err(ctx)?;
    parser.skip_space();
    if !parser.rest.is_empty() {
        return Err(ctx(GeopError::new(format!("unexpected {:?}", parser.rest))));
    }
    if !value.is_finite() {
        return Err(ctx(GeopError::new(format!("{value} is no finite number"))));
    }
    Ok(value)
}

/// A recursive-descent reader of a formula: what is left of it to read.
struct Parser<'t, F> {
    rest: &'t str,
    lookup: F,
}

impl<'t, F: FnMut(&str) -> Option<f64>> Parser<'t, F> {
    fn skip_space(&mut self) {
        self.rest = self.rest.trim_start();
    }

    /// Takes the first `len` bytes of what is left.
    fn take(&mut self, len: usize) -> &'t str {
        let (taken, rest) = self.rest.split_at(len.min(self.rest.len()));
        self.rest = rest;
        taken
    }

    /// Takes `c` if it comes next.
    fn eat(&mut self, c: char) -> bool {
        self.skip_space();
        match self.rest.strip_prefix(c) {
            Some(rest) => {
                self.rest = rest;
                true
            }
            None => false,
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
        match self.rest.chars().next() {
            Some(c) if c.is_ascii_digit() || c == '.' => {
                let number = self.take(number_len(self.rest));
                number
                    .parse()
                    .map_err(|_| GeopError::new(format!("{number:?} is no number")))
            }
            Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                let name = self.take(name_len(self.rest));
                if self.eat('(') {
                    return self.call(name);
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

    /// The function `name` called on the arguments that follow its `(`.
    fn call(&mut self, name: &str) -> GeopResult<f64> {
        let function = function(name)
            .ok_or_else(|| GeopError::new(format!("there is no function {name:?}")))?;
        let mut args = vec![self.sum()?];
        while self.eat(',') {
            args.push(self.sum()?);
        }
        if !self.eat(')') {
            return Err(GeopError::new(format!("{name}( is not closed")));
        }
        match (function, args.as_slice()) {
            (Function::One(f), [x]) => Ok(f(*x)),
            (Function::Two(f), [x, y]) => Ok(f(*x, *y)),
            (Function::One(_), _) => Err(GeopError::new(format!(
                "{name} takes 1 argument, not {}",
                args.len()
            ))),
            (Function::Two(_), _) => Err(GeopError::new(format!(
                "{name} takes 2 arguments, not {}",
                args.len()
            ))),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use geop_core_math::scalars::ScalInF64;
    use geop_core_program::{part::Part, target::TargetReference};

    use super::*;
    use crate::editing::is_formula;

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
            "min(1)",
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
    /// other saves as `{"formula": text}`. Both read back as they were.
    #[test]
    fn formulas_save_as_numbers_or_formulas() {
        let plain = NumericInput::from(" 12.5 ");
        let formula = NumericInput::from(" width / 2 ");
        assert_eq!(plain, NumericInput::Plain(12.5));
        assert_eq!(formula.to_string(), "width / 2");
        assert_eq!(
            serde_json::to_value(&plain).unwrap(),
            serde_json::json!(12.5)
        );
        assert_eq!(
            serde_json::to_value(&formula).unwrap(),
            serde_json::json!({"formula": "width / 2"})
        );
        for f in [&plain, &formula] {
            let back: NumericInput =
                serde_json::from_value(serde_json::to_value(f).unwrap()).unwrap();
            assert_eq!(&back, f);
        }
    }

    /// The formulas of a step's saved arguments are found, and renamed,
    /// wherever they are in them; text that is no formula is left alone.
    #[test]
    fn formulas_are_found_and_renamed_in_saved_arguments() {
        let mut args = serde_json::json!({
            "width": {"formula": "w / 2"},
            "name": "w",
            "holes": [{"depth": {"formula": "w.depth + w"}}, {"depth": 3.0}],
        });
        assert_eq!(formulas_in(&args), ["w.depth + w", "w / 2"]);
        rename_parameter_in(&mut args, "w", "wall");
        assert_eq!(
            args,
            serde_json::json!({
                "width": {"formula": "wall / 2"},
                "name": "w",
                "holes": [{"depth": {"formula": "wall.depth + wall"}}, {"depth": 3.0}],
            })
        );
    }

    /// A formula resolved in a target's generator reads its parameters as
    /// inputs: a change to one of them rebuilds the target, a change to any
    /// other does not.
    #[test]
    fn a_formula_reads_the_parameters_it_names() {
        type Sc = ScalInF64;
        let mut part = Part::<Sc, Sc>::default();
        let half = TargetReference::<f64>::new("half");
        let formula = NumericInput::from("width / 2");
        let run = |part: &mut Part<Sc, Sc>, width: f64, other: f64| {
            let inputs = BTreeMap::from([
                ("width".to_string(), Sc::from_f64(width)),
                ("other".to_string(), Sc::from_f64(other)),
            ]);
            part.begin_run(&inputs, &BTreeMap::new());
            part.begin_step();
            (part.define_target(&half, &formula, |reader, f| f.resolve(reader))).unwrap();
            part.end_run(0);
        };
        run(&mut part, 4.0, 0.0);
        let reader = part.current();
        assert!(reader.defines(&half));
        run(&mut part, 4.0, 1.0);
        assert!(part.current().defines(&half));
        assert_eq!(
            NumericInput::from("width / 2")
                .resolve(&part.current())
                .unwrap(),
            2.0
        );
    }
}
