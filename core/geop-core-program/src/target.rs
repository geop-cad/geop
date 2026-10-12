use std::{any::Any, collections::HashMap};

use geop_core_math::geop_error::{GeopError, GeopResult};

/// Counts the runs of a part, starting at 1. Revision 0 is before the first
/// run: what was never set has not changed since then.
pub(crate) type Revision = u64;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum Dependency {
    /// The version of target `name` that the step at position `step` made.
    Target {
        name: String,
        step: usize,
    },
    /// That no version of target `name` was made before: what a revision
    /// starting from the default read.
    Undefined(String),
    Input(String),
    State(String),
}

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

/// One version of a target: what one step made it. A target is defined by
/// one step, or revised by one step after another
/// ([`crate::part::Part::revise_target`]), each revision a version of its
/// own.
pub(crate) struct Target {
    /// The position of the step that made this version.
    pub(crate) step: usize,
    /// The arguments it was made from: what a reuse compares.
    args: Box<dyn Any>,
    pub(crate) dependencies: Vec<Dependency>,
    /// The value, or what made building it fail.
    data: Result<Box<dyn Any>, String>,
    /// The run in which `data` last became different.
    pub(crate) changed_at: Revision,
    /// The run it was made or reused in last: what it read is unchanged
    /// since then, or it is made again.
    pub(crate) run: Revision,
}

impl Target {
    /// The version the step at position `step` made of `args` in the run
    /// `run`: `made`, having read `dependencies`, and changed then unless
    /// it is equal to `previous`, the version that step made before.
    pub(crate) fn new<Out: PartialEq + 'static, Args: 'static>(
        step: usize,
        args: Args,
        dependencies: Vec<Dependency>,
        made: Result<Out, String>,
        previous: Option<&Target>,
        run: Revision,
    ) -> Self {
        let changed_at = match (previous, &made) {
            (Some(old), Ok(new)) if old.holds(new) => old.changed_at,
            _ => run,
        };
        Self {
            step,
            args: Box::new(args),
            dependencies,
            data: made.map(|data| Box::new(data) as Box<dyn Any>),
            changed_at,
            run,
        }
    }

    /// Its value, a `T` — or why there is none: it failed, or it is no `T`.
    /// `name` is the target's, for the error.
    pub(crate) fn value<T: 'static>(&self, name: &str) -> GeopResult<&T> {
        let data = self.data.as_ref().map_err(|e| {
            GeopError::new(format!(
                "target `{name}` failed in the step at position {}: {e}",
                self.step
            ))
        })?;
        data.downcast_ref::<T>().ok_or_else(|| {
            GeopError::new(format!(
                "target `{name}` is not a `{}`",
                std::any::type_name::<T>()
            ))
        })
    }

    /// Whether it was made, as an `Out`, from `args`: what a reuse asks.
    pub(crate) fn made_from<Out: 'static, Args: PartialEq + 'static>(&self, args: &Args) -> bool {
        self.data.as_ref().is_ok_and(|data| data.is::<Out>())
            && self.args.downcast_ref::<Args>() == Some(args)
    }

    /// Whether its value is `value`.
    fn holds<Out: PartialEq + 'static>(&self, value: &Out) -> bool {
        (self.data.as_ref().ok()).and_then(|data| data.downcast_ref::<Out>()) == Some(value)
    }
}

/// The versions of every target: by its name, then by the position of the
/// step that made each. A step makes at most one version of a target, so
/// the two identify a version, in this run and the next.
#[derive(Default)]
pub(crate) struct TargetRegistry(HashMap<String, Vec<Target>>);

impl TargetRegistry {
    pub(crate) fn versions(&self, name: &str) -> impl Iterator<Item = &Target> {
        self.0.get(name).into_iter().flatten()
    }

    /// The name of every target that has a version.
    pub(crate) fn names(&self) -> impl Iterator<Item = &str> {
        self.0.keys().map(String::as_str)
    }

    /// The version of `name` that the step at position `step` made.
    pub(crate) fn get(&self, name: &str, step: usize) -> Option<&Target> {
        self.versions(name).find(|target| target.step == step)
    }

    pub(crate) fn get_mut(&mut self, name: &str, step: usize) -> Option<&mut Target> {
        (self.0.get_mut(name)?.iter_mut()).find(|target| target.step == step)
    }

    /// Stores `target` as the version of `name` its step made, in place of
    /// the one it made before.
    pub(crate) fn insert(&mut self, name: &str, target: Target) {
        match self.get_mut(name, target.step) {
            Some(old) => *old = target,
            None => self.0.entry(name.to_string()).or_default().push(target),
        }
    }

    /// What the versions read, all of them.
    pub(crate) fn dependencies(&self) -> impl Iterator<Item = &Dependency> {
        (self.0.values().flatten()).flat_map(|target| &target.dependencies)
    }

    pub(crate) fn retain(&mut self, keep: impl Fn(&Target) -> bool) {
        self.0.retain(|_, versions| {
            versions.retain(|target| keep(target));
            !versions.is_empty()
        });
    }
}
