//! [`Records`]: what a part records of some of its entities besides their
//! shape, by name — the cable a solid is swept from, the sheet a sheet-metal
//! body is bent from — as the value of a target the steps that make them
//! revise.

use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::target::Revisable;

/// Records of `T`, by the name of what each is of.
#[derive(Clone, Debug, PartialEq)]
pub struct Records<T>(BTreeMap<String, T>);

impl<T> Default for Records<T> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

/// A part has few records of a kind: a revision copies them.
impl<T: Clone + PartialEq + 'static> Revisable for Records<T> {
    fn to_revise(&self) -> Self {
        self.clone()
    }
}

impl<T> Records<T> {
    /// Records `record` under `name`. Fails, leaving the records as they
    /// are, if there is one of that name already.
    pub fn add(&mut self, name: impl Into<String>, record: T) -> GeopResult<()> {
        let name = name.into();
        if self.0.contains_key(&name) {
            return Err(GeopError::new(format!(
                "the part has a record of {name:?} already"
            )));
        }
        self.0.insert(name, record);
        Ok(())
    }

    /// Records `record` under `name`, in place of what was.
    pub fn insert(&mut self, name: impl Into<String>, record: T) {
        self.0.insert(name.into(), record);
    }

    /// Takes the record of `name` out, if there is one: what it was of is
    /// gone.
    pub fn remove(&mut self, name: &str) -> Option<T> {
        self.0.remove(name)
    }

    /// The record of `name`, if there is one.
    pub fn get(&self, name: &str) -> Option<&T> {
        self.0.get(name)
    }

    /// Every record, by name, in the order of the names.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &T)> {
        self.0.iter().map(|(name, record)| (name.as_str(), record))
    }
}
