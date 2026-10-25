//! [`NumericInput`]: a number an operation is given — a length, an angle, a
//! count — typed into a dialog's number field as a plain value or as a
//! formula of the program's parameters, which it follows ([`evaluate`]).
//! An operation's arguments take one wherever they take a number. (How a
//! dialog shows one, the editor says.)
//!
//! A step resolves it where it builds ([`NumericInput::resolve`]), reading each
//! parameter the formula names — a target the program defines
//! ([`parameter`]): the read is recorded, so a change to one of them
//! rebuilds the step, a change to any other does not.

use std::borrow::Cow;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    element::{ElementReference, HasElements, Kind, whole},
    reader::Reader,
    target::{Target, TargetReference},
};

/// What a parameter of the program comes to, as its target holds it
/// ([`parameter`]): a number, or a text — the row of a table a part family
/// is looked up in, by name.
#[derive(Clone, Debug, PartialEq)]
pub enum ParameterValue {
    Number(f64),
    Text(String),
}

/// Defined once, by the step that defines the parameter.
impl Target for ParameterValue {}

/// Read whole.
impl Kind for ParameterValue {
    const NAME: &'static str = "parameter";
}

impl HasElements<ParameterValue> for ParameterValue {
    type Element<'a> = &'a ParameterValue;

    fn element(&self, name: &str) -> GeopResult<&ParameterValue> {
        whole(self, name)
    }
}

/// The target of the parameter `name`: `width`, or `size.d` for the column
/// `d` of the row a table looks up by the parameter `size`.
pub fn parameter(name: &str) -> TargetReference<ParameterValue> {
    TargetReference::new(format!("parameter:{name}"))
}

/// Every parameter of the part, by name, in the order the steps defining
/// them define them, as each comes to: what lists or designates the part
/// reads. A parameter that failed is left out.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ParameterValues(pub Vec<(String, ParameterValue)>);

/// Each step defining parameters adds its own.
impl Target for ParameterValues {
    const CHAIN: bool = true;
}

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
pub fn read_parameter_values<'a>(reader: &Reader<'a>) -> GeopResult<Cow<'a, ParameterValues>> {
    if !reader.defines(&parameter_values()) {
        return Ok(Cow::Owned(ParameterValues::default()));
    }
    Ok(Cow::Borrowed(reader.retrieve_element(
        &ElementReference::whole(parameter_values()),
    )?))
}

/// The number `name` stands for in a formula, as `reader` reads it, the
/// read recorded: a number parameter's value — `width`, or `size.d`, a
/// column of the row a table looked up. None where no step defines that
/// parameter, or it failed, or it is no number.
pub fn read_parameter(reader: &Reader, name: &str) -> Option<f64> {
    let reference = parameter(name);
    if !reader.defines(&reference) {
        return None;
    }
    match reader.retrieve_element(&ElementReference::whole(reference)) {
        Ok(ParameterValue::Number(value)) => Some(*value),
        _ => None,
    }
}

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

    /// Its value, each parameter it names read through `reader` (see
    /// [`read_parameter`]): in a target's generator, every one of them
    /// recorded read.
    pub fn resolve(&self, reader: &Reader) -> GeopResult<f64> {
        match self {
            NumericInput::Plain(value) => Ok(*value),
            NumericInput::Formula { formula } => {
                evaluate(formula, |name| read_parameter(reader, name))
            }
        }
    }
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
    use super::*;
    use crate::{operation_registry::OperationRegistry, part::Part, runner::Runner};

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

    /// A formula resolved in a target's generator reads its parameters as
    /// inputs: a change to one of them rebuilds the target, a change to any
    /// other does not.
    #[test]
    fn a_formula_reads_the_parameters_it_names() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static MADE: AtomicUsize = AtomicUsize::new(0);
        let runner = Runner::new(OperationRegistry::default());
        let mut part = Part::default();
        let half = TargetReference::<f64>::new("half");
        let formula = NumericInput::from("width / 2");
        let run = |part: &mut Part, width: f64, other: f64| {
            let given = |_: &Reader, &value: &f64| Ok(ParameterValue::Number(value));
            (runner.run_step(part, "parameters", |ctx| {
                ctx.produce(&parameter("width"), &width, given)?;
                ctx.produce(&parameter("other"), &other, given)
            }))
            .unwrap();
            (runner.run_step(part, "half", |ctx| {
                ctx.produce(&half, &formula, |reader, f| {
                    MADE.fetch_add(1, Ordering::Relaxed);
                    f.resolve(reader)
                })
            }))
            .unwrap();
            MADE.load(Ordering::Relaxed)
        };
        assert_eq!(run(&mut part, 4.0, 0.0), 1);
        assert_eq!(run(&mut part, 4.0, 1.0), 1, "it reads no other parameter");
        assert_eq!(run(&mut part, 6.0, 1.0), 2);
        assert_eq!(formula.resolve(&part.current()).unwrap(), 3.0);
    }
}
