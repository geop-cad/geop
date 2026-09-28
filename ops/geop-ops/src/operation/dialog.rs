//! Dialogs: what an editor shows for a step beyond its arguments' schema.
//!
//! The schema ([`super::ArgSchema`]) says what each argument is and how it
//! is entered, the same for every step of an operation. What the part a step
//! is applied to makes of the arguments' current values is the step's
//! [`Dialog`]: what each picked entity can be used as, which of a choice's
//! options fit what is picked. An editor lays out the form from the schema
//! and fills it in from the dialog, and needs to know neither operation.

use std::collections::BTreeMap;

use serde::Serialize;

use super::Role;

/// What the part makes of a step's arguments, per argument that has
/// anything to say.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Dialog {
    /// By argument, as its field is named.
    pub args: BTreeMap<&'static str, ArgDialog>,
}

/// What the part makes of one argument's value.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ArgDialog {
    /// Entities picked to build on: per entity, in order, the roles it can
    /// fill — none for one the part does not have.
    Selection { roles: Vec<Vec<Role>> },
    /// A choice among options: those that can be chosen now, by how they
    /// are spelled.
    Options { fit: Vec<&'static str> },
}
