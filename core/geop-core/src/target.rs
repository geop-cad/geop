use crate::part::{Dependency, Revision};
use std::{any::Any, collections::HashMap};

/// The stable name of a target, as programs and arguments refer to it, and
/// the type of its value.
pub struct TargetReference<T>(String, std::marker::PhantomData<fn() -> T>);

// By hand, since a derive would ask the same of `T`, and a reference is only
// a name.
impl<T> Clone for TargetReference<T> {
    fn clone(&self) -> Self {
        Self::new(self.0.clone())
    }
}

impl<T> PartialEq for TargetReference<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T> Eq for TargetReference<T> {}

impl<T> std::fmt::Debug for TargetReference<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "TargetReference({:?})", self.0)
    }
}

impl<T> TargetReference<T> {
    pub fn new(name: impl Into<String>) -> Self {
        Self(name.into(), std::marker::PhantomData)
    }

    pub fn name(&self) -> &str {
        &self.0
    }
}

/// A value that steps revise one after another, such as a part's topology
/// or an assembly, rather than define once, such as a datum. Every revision
/// is kept, for the steps that read it and for the next run, so a revision
/// must cost what it changes, not what the value holds: share structure
/// between revisions (`imbl`), and let `==` skip what two revisions share.
///
/// Only a `Revisable` target can be revised:
///
/// ```compile_fail
/// # use geop_core::{part::{Input, Part, State}, target::TargetReference};
/// #[derive(Clone, PartialEq)]
/// struct Datum;
///
/// fn move_datum<I: Input, S: State>(part: &mut Part<I, S>) {
///     let datum = TargetReference::<Datum>::new("datum");
///     part.revise_target(&datum, &(), |datum, _, _| Ok(datum));
/// }
/// ```
///
/// ```
/// # use geop_core::{part::{Input, Part, State}, target::{Revisable, TargetReference}};
/// #[derive(Clone, PartialEq)]
/// struct Topology;
///
/// impl Revisable for Topology {
///     fn to_revise(&self) -> Self {
///         self.clone()
///     }
/// }
///
/// fn cut<I: Input, S: State>(part: &mut Part<I, S>) {
///     let topology = TargetReference::<Topology>::new("topology");
///     part.revise_target(&topology, &(), |topology, _, _| Ok(topology));
/// }
/// ```
pub trait Revisable: PartialEq + 'static {
    /// A copy to revise, sharing everything the revision leaves untouched.
    fn to_revise(&self) -> Self;
}

/// One version of a target: what one step made it. A target is defined by
/// one step and, if [`Revisable`], then revised by later ones, each revision
/// a version of its own.
pub(crate) struct Target {
    /// The step that made this version.
    pub(crate) step: String,
    pub(crate) args: Box<dyn Any>,
    pub(crate) dependencies: Vec<Dependency>,
    /// The value, or what made building it fail.
    pub(crate) data: Result<Box<dyn Any>, String>,
    /// The run in which `data` last became different.
    pub(crate) changed_at: Revision,
    /// The run in which the version was last made, rebuilt or reused.
    pub(crate) verified_at: Revision,
}

/// The versions of every target: by its name, then by the step that made
/// each. A step makes at most one version of a target, so the two identify
/// a version, in this run and the next.
#[derive(Default)]
pub(crate) struct TargetRegistry(HashMap<String, Vec<Target>>);

impl TargetRegistry {
    pub(crate) fn versions(&self, name: &str) -> impl Iterator<Item = &Target> {
        self.0.get(name).into_iter().flatten()
    }

    /// The version of `name` that `step` made.
    pub(crate) fn get(&self, name: &str, step: &str) -> Option<&Target> {
        self.versions(name).find(|target| target.step == step)
    }

    pub(crate) fn get_mut(&mut self, name: &str, step: &str) -> Option<&mut Target> {
        (self.0.get_mut(name)?.iter_mut()).find(|target| target.step == step)
    }

    /// Stores `target` as the version of `name` its step made, in place of
    /// the one it made before.
    pub(crate) fn insert(&mut self, name: &str, target: Target) {
        match self.get_mut(name, &target.step) {
            Some(old) => *old = target,
            None => self.0.entry(name.to_string()).or_default().push(target),
        }
    }

    pub(crate) fn retain(&mut self, keep: impl Fn(&Target) -> bool) {
        self.0.retain(|_, versions| {
            versions.retain(|target| keep(target));
            !versions.is_empty()
        });
    }
}
