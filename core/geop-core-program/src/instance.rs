//! Instances: parts placed in a part. A step places the part another
//! program builds ([`Part::define_instance`]), by a name of its own, and
//! that part is then read through the part it is placed in: an element of
//! it is named by the instance's path before its target's name —
//! `bolt/model`, or `asm/bolt/model` for a part placed in a placed part
//! ([`INSTANCE_SEPARATOR`]) — and a pick of it names it so.
//!
//! An instance is run with the state of the part it is placed in under its
//! name: the state `bolt/b.pose` of the part is the state `b.pose` of the
//! instance `bolt`. So whatever solves the part's state solves its
//! instances' too, and what the instance's steps require of its own state
//! comes up to the part it is placed in behind its name.
//!
//! Every instance is a part of its own, run with its own state — a copy
//! of another too, which goes where that one goes but may be moved inside
//! differently: a patterned servo, each horn at an angle of its own.
//!
//! Where an instance is ([`Placement`]) is a value of the state, or a
//! fixed motion after another instance, composed outermost first
//! ([`Reader::placement`]). What a placement is — a pose, say — is the
//! state's own business: the crate that defines the state composes it.

use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::part::{Input, Part, Reader, State};

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
    target.rsplit_once(INSTANCE_SEPARATOR).unwrap_or(("", target))
}

/// Where an instance is in the part it is placed in.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement<S> {
    /// Where the state `state` of the part puts it: moved by whatever
    /// solves the state.
    State(String),
    /// `motion` after where the instance `of` of the same part is: a copy,
    /// which goes with what it copies. Only where it is: what is in it, it
    /// is run for itself, with its own state.
    After { of: String, motion: S },
}

/// A part placed in a part: what an instance's target holds — the part
/// its program built, its own.
///
/// TODO: Later we should share the part of every instance whose run read
/// none of the state: a part of no state is the same wherever it is
/// placed. For now every instance is built on its own.
pub struct Instance<I, S> {
    pub part: Box<Part<I, S>>,
    pub placement: Placement<S>,
}

/// Equal when it is the very same build, placed the same way: a part built
/// again counts as changed, however alike. A version is only compared with
/// the one it replaces, which still holds its part, so no two builds alive
/// share an address.
impl<I, S: PartialEq> PartialEq for Instance<I, S> {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(&*self.part, &*other.part) && self.placement == other.placement
    }
}

impl<I, S> std::fmt::Debug for Instance<I, S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Instance")
    }
}

impl<I: Input, S: State> Instance<I, S> {
    /// The part it places, as its program left it.
    pub fn reader(&self) -> Reader<'_, I, S> {
        self.part.current()
    }
}

/// What the target of an instance is named, before the instance's name.
const PREFIX: &str = "instance:";

/// The name of the target of the instance `name`.
pub(crate) fn instance_target(name: &str) -> String {
    format!("{PREFIX}{name}")
}

/// The name of the instance whose target is named `target`, if it is one.
pub(crate) fn instance_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}

/// What makes an instance from its arguments, what it reads, and the state
/// of the part under the instance's name — `b.pose` for `bolt/b.pose` —
/// which its part is to be run with.
pub type InstanceGenerator<I, S, Args> =
    fn(&Reader<I, S>, &Args, &BTreeMap<String, S>) -> GeopResult<Instance<I, S>>;

/// An instance's name, its arguments and its generator: the arguments of
/// its target. Two are equal where all three are.
pub(crate) struct Defining<I, S, Args> {
    pub(crate) name: String,
    pub(crate) args: Args,
    pub(crate) generator: InstanceGenerator<I, S, Args>,
}

// By hand, since a derive would ask the same of `I` and `S`.
impl<I, S, Args: Clone> Clone for Defining<I, S, Args> {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            args: self.args.clone(),
            generator: self.generator,
        }
    }
}

impl<I, S, Args: PartialEq> PartialEq for Defining<I, S, Args> {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.args == other.args
            && std::ptr::fn_addr_eq(self.generator, other.generator)
    }
}

/// The instance `defining` makes, given the state under its name.
pub(crate) fn place<I: Input, S: State, Args>(
    reader: &Reader<I, S>,
    defining: &Defining<I, S, Args>,
) -> GeopResult<Instance<I, S>> {
    let prefix = format!("{}{INSTANCE_SEPARATOR}", defining.name);
    let state = reader.states_under(&prefix);
    (defining.generator)(reader, &defining.args, &state)
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

/// One factor of where an instance is, as [`Reader::placement`] gives it,
/// outermost first: the instance is where they all are, composed in order.
#[derive(Clone, Debug, PartialEq)]
pub enum Factor<S> {
    /// The value of the state `state` of the part read — named as the part
    /// read names it, `asm/b.pose` — which a solve moves, and by which a
    /// constraint is differentiated.
    State { state: String, value: S },
    /// A value nothing in the part read moves: a copy's motion after what
    /// it copies.
    Fixed(S),
}

/// An element of a part or of an instance placed in it, as it is in its
/// own part ([`Reader::retrieve_placed`]): where that part is, the
/// instance's [`Reader::placement`] says.
#[derive(Clone, Debug, PartialEq)]
pub struct InInstance<E> {
    pub element: E,
    /// The path of the instance it is of: empty for the part itself.
    pub instance: String,
}
