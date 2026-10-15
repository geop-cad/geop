//! Instances: parts other programs build, run in a part by a name of their
//! own ([`ProgramRunner::load_subpart`]) — the part of each instance. That part is
//! read through the part it is in ([`Reader::inside`]): an element of it
//! is named by the instance's path before its target's name —
//! `bolt/model`, or `asm/bolt/model` for a part in a part's instance
//! ([`INSTANCE_SEPARATOR`]) — and a pick of it names it so.
//!
//! The part keeps the part of each instance, and runs it again in every
//! run that loads it: what reads what changed is built again, the rest is
//! reused, as in any run. It is run with the state of the part under the
//! instance's name: the state `bolt/b.pose` of the part is the state
//! `b.pose` of the instance `bolt`. So whatever solves the part's state
//! solves its instances' too, and what the instance's steps require of its
//! own state comes up to the part behind its name.
//!
//! Where an instance is placed is no concern of the program's: the
//! operation that loads it defines a target of its own that says so, in
//! the terms of the state it is moved by.

use geop_core_math::geop_error::{GeopError, GeopResult};

#[cfg(doc)]
use crate::{part::Reader, program_runner::ProgramRunner};

/// What separates the names of a path of instances, and an instance's path
/// from the name of a target in it: `asm/bolt/model`.
pub const INSTANCE_SEPARATOR: char = '/';

/// The path of the instance `name` of the instance at `outer`: `asm/bolt`
/// — `name` itself, for none.
pub fn instance_path(outer: &str, name: &str) -> String {
    match outer.is_empty() {
        true => name.to_string(),
        false => format!("{outer}{INSTANCE_SEPARATOR}{name}"),
    }
}

/// The instance path a target's name begins with, and the name of the
/// target in that instance: `("asm/bolt", "model")` for
/// `asm/bolt/model`; an empty path for a target of the part itself.
pub fn split_path(target: &str) -> (&str, &str) {
    target
        .rsplit_once(INSTANCE_SEPARATOR)
        .unwrap_or(("", target))
}

/// What the target of an instance's part holds: only that it is kept,
/// loaded by the step that made it. What is in it, the part keeps (see
/// [`ProgramRunner::load_subpart`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Kept;

/// What the target of an instance's part is named, before the instance's
/// name.
const PREFIX: &str = "subpart:";

/// The name of the target of the part of the instance `name`.
pub(crate) fn subpart_target(name: &str) -> String {
    format!("{PREFIX}{name}")
}

/// The name of the instance whose part's target is named `target`, if it
/// is one.
pub(crate) fn subpart_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}

/// Checks that `name` can name an instance: not empty, and no path.
pub(crate) fn check_name(name: &str) -> GeopResult<()> {
    match name.is_empty() || name.contains(INSTANCE_SEPARATOR) {
        true => Err(GeopError::new(format!(
            "{name:?} cannot name an instance: it is empty or contains {INSTANCE_SEPARATOR:?}"
        ))),
        false => Ok(()),
    }
}

/// An element of a part or of one of its instances, as it is in its own
/// part ([`Reader::retrieve_placed`]): where that part is placed, the
/// operation that loaded it says.
#[derive(Clone, Debug, PartialEq)]
pub struct InInstance<E> {
    pub element: E,
    /// The path of the instance it is of: empty for the part itself.
    pub instance: String,
}
