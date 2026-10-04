//! Typed access to the instances of an [`Exchange`]: an entity of an
//! expected type and its parameters by position, every failure naming the
//! instance by `#id` and type.

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::part21::{Exchange, Instance, Record, Value};

/// The error for an instance the importer does not support: what it is,
/// where, and why.
pub fn unsupported(id: u64, instance: &Instance, why: &str) -> GeopError {
    GeopError::new(format!(
        "#{id} {} is not supported: {why}",
        instance.type_name()
    ))
}

/// One record of an instance, with its parameters.
#[derive(Clone, Copy)]
pub struct Args<'a> {
    pub id: u64,
    pub record: &'a Record,
}

impl<'a> Args<'a> {
    fn error(&self, k: usize, what: &str) -> GeopError {
        GeopError::new(format!(
            "#{} {}: parameter {} {what}",
            self.id,
            self.record.name,
            k + 1
        ))
    }

    pub fn get(&self, k: usize) -> GeopResult<&'a Value> {
        self.record
            .args
            .get(k)
            .ok_or_else(|| self.error(k, "is missing"))
    }

    pub fn is_null(&self, k: usize) -> bool {
        matches!(self.record.args.get(k), None | Some(Value::Null))
    }

    pub fn real(&self, k: usize) -> GeopResult<f64> {
        real(self.get(k)?).ok_or_else(|| self.error(k, "is not a number"))
    }

    pub fn integer(&self, k: usize) -> GeopResult<i64> {
        match self.get(k)? {
            Value::Integer(i) => Ok(*i),
            _ => Err(self.error(k, "is not an integer")),
        }
    }

    pub fn reference(&self, k: usize) -> GeopResult<u64> {
        match self.get(k)? {
            Value::Ref(id) => Ok(*id),
            _ => Err(self.error(k, "is not a reference to an instance")),
        }
    }

    pub fn list(&self, k: usize) -> GeopResult<&'a [Value]> {
        match self.get(k)? {
            Value::List(values) => Ok(values),
            _ => Err(self.error(k, "is not a list")),
        }
    }

    pub fn references(&self, k: usize) -> GeopResult<Vec<u64>> {
        self.list(k)?
            .iter()
            .map(|v| match v {
                Value::Ref(id) => Ok(*id),
                _ => Err(self.error(k, "is not a list of references")),
            })
            .collect()
    }

    pub fn reals(&self, k: usize) -> GeopResult<Vec<f64>> {
        self.list(k)?
            .iter()
            .map(|v| real(v).ok_or_else(|| self.error(k, "is not a list of numbers")))
            .collect()
    }

    pub fn integers(&self, k: usize) -> GeopResult<Vec<i64>> {
        self.list(k)?
            .iter()
            .map(|v| match v {
                Value::Integer(i) => Ok(*i),
                _ => Err(self.error(k, "is not a list of integers")),
            })
            .collect()
    }

    /// A `BOOLEAN` or `LOGICAL`: `.T.` is true, `.F.` false.
    pub fn logical(&self, k: usize) -> GeopResult<bool> {
        match self.get(k)? {
            Value::Enum(e) if e == "T" => Ok(true),
            Value::Enum(e) if e == "F" => Ok(false),
            _ => Err(self.error(k, "is not .T. or .F.")),
        }
    }

    pub fn enumeration(&self, k: usize) -> GeopResult<&'a str> {
        match self.get(k)? {
            Value::Enum(e) => Ok(e),
            _ => Err(self.error(k, "is not an enumeration value")),
        }
    }

    pub fn string(&self, k: usize) -> GeopResult<&'a str> {
        match self.get(k)? {
            Value::String(s) => Ok(s),
            _ => Err(self.error(k, "is not a string")),
        }
    }
}

/// A number, as written: a real, an integer, or a measure of either.
pub fn real(value: &Value) -> Option<f64> {
    match value {
        Value::Real(r) => Some(*r),
        Value::Integer(i) => Some(*i as f64),
        Value::Typed(_, inner) => real(inner),
        _ => None,
    }
}

/// The instances of a file, read by type.
#[derive(Clone, Copy)]
pub struct Reader<'a> {
    pub exchange: &'a Exchange,
}

impl<'a> Reader<'a> {
    pub fn instance(&self, id: u64) -> GeopResult<&'a Instance> {
        self.exchange.get(id)
    }

    /// The record `name` of the instance `#id`, which has to be of that
    /// type.
    pub fn args(&self, id: u64, name: &str) -> GeopResult<Args<'a>> {
        let instance = self.instance(id)?;
        let record = instance.record(name).ok_or_else(|| {
            GeopError::new(format!(
                "#{id} is a {}, where a {name} was expected",
                instance.type_name()
            ))
        })?;
        Ok(Args { id, record })
    }

    /// The record of the first of `names` the instance `#id` is of, and
    /// which one it is.
    pub fn args_of_any(
        &self,
        id: u64,
        names: &[&'static str],
    ) -> GeopResult<(&'static str, Args<'a>)> {
        let instance = self.instance(id)?;
        for &name in names {
            if let Some(record) = instance.record(name) {
                return Ok((name, Args { id, record }));
            }
        }
        Err(GeopError::new(format!(
            "#{id} is a {}, where one of {} was expected",
            instance.type_name(),
            names.join(", ")
        )))
    }
}
