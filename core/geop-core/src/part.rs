use std::collections::{BTreeMap, HashMap};

use indexmap::IndexSet;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    constraint::Constraint,
    target::{Revisable, Target, TargetReference, TargetRegistry},
};

/// A value a run is given by name, which no step changes: a parameter.
pub trait Input: Clone + PartialEq + 'static {}

impl<T: Clone + PartialEq + 'static> Input for T {}

/// A value the state holds by name, which a solver moves until the
/// [`Constraint`]s hold: a pose, say. A name the run gives no value reads as
/// the default.
pub trait State: Clone + PartialEq + Default + std::fmt::Debug + 'static {
    /// The numbers the state, and the constraints on it, are made of. `==` on
    /// them asks whether two enclosures are the same, as a check for change
    /// does, never whether two values could be equal.
    type Scalar: geop_core_math::scalars::Scalar + PartialEq;
    /// What a constraint's derivative by a value of the state is: for a pose,
    /// its twist.
    type Tangent: Clone + PartialEq + std::fmt::Debug + 'static;
}

/// Counts the runs of a part, starting at 1. Revision 0 is before the first
/// run: what was never set has not changed since then.
pub type Revision = u64;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum Dependency {
    /// The version of target `name` that step `step` made.
    Target {
        name: String,
        step: String,
    },
    Input(String),
    State(String),
}

/// A value given to a run from outside, and the run in which it last changed.
/// A value that is removed stays as `None`, so that what read it sees the
/// change.
struct Given<T> {
    value: Option<T>,
    changed_at: Revision,
}

/// Makes `given` hold exactly `values`, marking what differs as changed in
/// `revision`.
fn replace_given<T: PartialEq + Clone>(
    given: &mut BTreeMap<String, Given<T>>,
    values: &BTreeMap<String, T>,
    revision: Revision,
) {
    for (name, entry) in given.iter_mut() {
        let value = values.get(name);
        if entry.value.as_ref() != value {
            entry.value = value.cloned();
            entry.changed_at = revision;
        }
    }
    for (name, value) in values {
        given.entry(name.clone()).or_insert_with(|| Given {
            value: Some(value.clone()),
            changed_at: revision,
        });
    }
}

fn changed_at<T>(given: &BTreeMap<String, Given<T>>, name: &str) -> Revision {
    given.get(name).map_or(0, |entry| entry.changed_at)
}

/// Where in the last run a read looks from.
#[derive(Clone, Copy)]
enum At {
    /// Where the run is now: it sees every version made so far.
    Now,
    /// Before the step at this index: it sees the versions earlier steps made.
    BeforeStep(usize),
}

/// What a program builds, and what it builds from. Each run states its
/// inputs and state in full. Its steps define targets, in order, and may
/// revise [`Revisable`] targets earlier steps defined; every revision is a
/// version of its own, and a read sees the latest version made before it in
/// the same run. Steps also define [`Constraint`]s on the state, which the
/// run reports for a solver to drive to zero.
///
/// A version is reused from the last run when its step made it from equal
/// arguments and nothing it read has changed since. A version rebuilt into
/// a value equal to the last one counts as unchanged, so what read it is
/// reused in turn.
pub struct Part<I, S> {
    revision: Revision,
    /// The steps of the current or last run, in order. A step's id is what
    /// identifies it across runs; its position only orders it within one.
    steps: IndexSet<String>,
    targets: TargetRegistry,
    /// For each target, the step that made its latest version so far in
    /// this run.
    heads: HashMap<String, String>,
    inputs: BTreeMap<String, Given<I>>,
    state: BTreeMap<String, Given<S>>,
}

impl<I, S> Default for Part<I, S> {
    fn default() -> Self {
        Self {
            revision: 0,
            steps: IndexSet::new(),
            targets: TargetRegistry::default(),
            heads: HashMap::new(),
            inputs: BTreeMap::new(),
            state: BTreeMap::new(),
        }
    }
}

impl<I: Input, S: State> Part<I, S> {
    pub(crate) fn begin_run(&mut self, inputs: &BTreeMap<String, I>, state: &BTreeMap<String, S>) {
        self.revision += 1;
        self.steps.clear();
        self.heads.clear();
        replace_given(&mut self.inputs, inputs, self.revision);
        replace_given(&mut self.state, state, self.revision);
    }

    /// Starts the step `step`. Fails if a step of that id ran before in this
    /// run: the id is what tells one step's versions from another's.
    pub(crate) fn begin_step(&mut self, step: &str) -> GeopResult<()> {
        if !self.steps.insert(step.to_string()) {
            return Err(GeopError::new(format!(
                "step `{step}` appears twice in the program"
            )));
        }
        Ok(())
    }

    /// Deletes the versions no step of this run made.
    pub(crate) fn end_run(&mut self) {
        let revision = self.revision;
        self.targets.retain(|target| target.verified_at == revision);
    }

    fn step(&self) -> &str {
        self.steps.last().map_or("", String::as_str)
    }

    /// Where `step` came in the current or last run.
    fn position(&self, step: &str) -> Option<usize> {
        self.steps.get_index_of(step)
    }

    /// The part as it is now: during a run, with what the steps so far made;
    /// after one, the finished part.
    pub fn current(&self) -> View<'_, I, S> {
        View {
            part: self,
            at: At::Now,
        }
    }

    /// The part as it was before the step at `step_index` of the last run.
    pub fn view(&self, step_index: usize) -> View<'_, I, S> {
        View {
            part: self,
            at: At::BeforeStep(step_index),
        }
    }

    /// The latest version of `name` made so far in this run.
    fn head(&self, name: &str) -> Option<&Target> {
        self.targets.get(name, self.heads.get(name)?)
    }

    /// The version of `name` a read at `at` sees.
    fn resolve(&self, name: &str, at: At) -> GeopResult<&Target> {
        let found = match at {
            At::Now => self.head(name),
            At::BeforeStep(index) => self
                .targets
                .versions(name)
                .filter(|target| target.verified_at == self.revision)
                .filter_map(|target| Some((self.position(&target.step)?, target)))
                .filter(|&(position, _)| position < index)
                .max_by_key(|&(position, _)| position)
                .map(|(_, target)| target),
        };
        found.ok_or_else(|| {
            let before = match at {
                At::Now => format!("step `{}`", self.step()),
                At::BeforeStep(index) => match self.steps.get_index(index) {
                    Some(step) => format!("step `{step}`"),
                    None => "the end".to_string(),
                },
            };
            GeopError::new(format!("target `{name}` is not defined before {before}"))
        })
    }

    fn lookup<T: 'static>(
        &self,
        reference: &TargetReference<T>,
        at: At,
    ) -> GeopResult<(&Target, &T)> {
        let name = reference.name();
        let target = self.resolve(name, at)?;
        let data = target.data.as_ref().map_err(|e| {
            GeopError::new(format!(
                "target `{name}` failed in step `{}`: {e}",
                target.step
            ))
        })?;
        let data = data.downcast_ref::<T>().ok_or_else(|| {
            GeopError::new(format!(
                "target `{name}` is not a `{}`",
                std::any::type_name::<T>()
            ))
        })?;
        Ok((target, data))
    }

    /// The input's value, if this run gives one.
    fn input(&self, name: &str) -> Option<I> {
        self.inputs.get(name).and_then(|entry| entry.value.clone())
    }

    /// The state's value; the default if this run gives none.
    fn state(&self, name: &str) -> S {
        self.state
            .get(name)
            .and_then(|entry| entry.value.clone())
            .unwrap_or_default()
    }

    /// Whether `dependency` is unchanged since `revision`. A version counts as
    /// changed unless a read now still sees it, and its value is no newer
    /// than `revision`.
    fn unchanged_since(&self, dependency: &Dependency, revision: Revision) -> bool {
        match dependency {
            Dependency::Target { name, step } => {
                self.heads.get(name) == Some(step)
                    && (self.targets.get(name, step)).is_some_and(|t| t.changed_at <= revision)
            }
            Dependency::Input(name) => changed_at(&self.inputs, name) <= revision,
            Dependency::State(name) => changed_at(&self.state, name) <= revision,
        }
    }

    /// Defines the target `reference` as what `generator` makes of `args`.
    /// Fails if a step has defined it before in this run.
    pub fn define_target<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: impl FnOnce(&mut Reader<I, S>, &Args) -> GeopResult<Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        if let Some(head) = self.head(name) {
            return Err(GeopError::new(format!(
                "target `{name}` is defined twice: by step `{}` and by step `{}`",
                head.step,
                self.step()
            )));
        }
        self.make_version(reference, args, generator)
    }

    /// Revises the target `reference`, which an earlier step defined, to
    /// what `generator` makes of its latest version and `args`. The latest
    /// version stays as it is, for the steps that read it and for the next
    /// run; the generator revises a copy of it ([`Revisable::to_revise`]).
    pub fn revise_target<Out: Revisable, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: impl FnOnce(Out, &mut Reader<I, S>, &Args) -> GeopResult<Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        if let Some(head) = self.head(name)
            && head.step == self.step()
        {
            return Err(GeopError::new(format!(
                "target `{name}` is made twice by step `{}`",
                self.step()
            )));
        }
        self.make_version(reference, args, |reader, args| {
            let previous = reader.retrieve_target(reference)?.to_revise();
            generator(previous, reader, args)
        })
    }

    /// Makes this step's version of `reference`, reusing the last run's when
    /// nothing it read has changed, and makes it the head.
    fn make_version<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: impl FnOnce(&mut Reader<I, S>, &Args) -> GeopResult<Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        let step = (self.steps.last().cloned())
            .ok_or_else(|| GeopError::new(format!("target `{name}` is made outside of a step")))?;
        let revision = self.revision;
        let reusable = self.targets.get(name, &step).is_some_and(|target| {
            target.data.as_ref().is_ok_and(|data| data.is::<Out>())
                && target.args.downcast_ref::<Args>() == Some(args)
                && target
                    .dependencies
                    .iter()
                    .all(|dependency| self.unchanged_since(dependency, target.verified_at))
        });
        if reusable && let Some(target) = self.targets.get_mut(name, &step) {
            target.verified_at = revision;
            self.heads.insert(name.to_string(), step);
            return Ok(());
        }

        let mut reader = Reader {
            part: self,
            dependencies: Vec::new(),
        };
        let built = generator(&mut reader, args);
        let mut dependencies = reader.dependencies;
        dependencies.sort();
        dependencies.dedup();

        let (data, result) = match built {
            Ok(data) => (Ok(Box::new(data) as Box<dyn std::any::Any>), Ok(())),
            Err(e) => (
                Err(e.root_message().to_string()),
                Err(e.with_context(format!("making target `{name}` in step `{step}`"))),
            ),
        };
        let changed_at = match (self.targets.get(name, &step), &data) {
            (Some(old), Ok(new))
                if old
                    .data
                    .as_ref()
                    .is_ok_and(|old| old.downcast_ref::<Out>() == new.downcast_ref::<Out>()) =>
            {
                old.changed_at
            }
            _ => revision,
        };
        self.targets.insert(
            name,
            Target {
                step: step.clone(),
                args: Box::new(args.clone()),
                dependencies,
                data,
                changed_at,
                verified_at: revision,
            },
        );
        self.heads.insert(name.to_string(), step);
        result
    }

    /// Defines the constraint `name` as what `generator` makes of `args`.
    /// A constraint is a target like any other: rebuilt only when what it
    /// read changes, the state above all.
    pub fn define_constraint<Args: PartialEq + Clone + 'static>(
        &mut self,
        name: &str,
        args: &Args,
        generator: impl FnOnce(&mut Reader<I, S>, &Args) -> GeopResult<Constraint<S>>,
    ) -> GeopResult<()> {
        let reference = TargetReference::<Constraint<S>>::new(name);
        self.define_target(&reference, args, |reader, args| {
            let constraint = generator(reader, args)?;
            constraint.check(name, |state| {
                reader
                    .dependencies
                    .contains(&Dependency::State(state.to_string()))
            })?;
            Ok(constraint)
        })
    }

    /// The constraints defined so far in this run, or in the last.
    pub fn constraints(&self) -> BTreeMap<&str, &Constraint<S>> {
        self.heads
            .iter()
            .filter_map(|(name, step)| {
                let data = self.targets.get(name, step)?.data.as_ref().ok()?;
                Some((name.as_str(), data.downcast_ref::<Constraint<S>>()?))
            })
            .collect()
    }
}

/// What a target's generator reads the part through. It records each read,
/// so that the target is rebuilt when any of it changes.
pub struct Reader<'a, I, S> {
    part: &'a Part<I, S>,
    dependencies: Vec<Dependency>,
}

impl<'a, I: Input, S: State> Reader<'a, I, S> {
    pub fn retrieve_target<T: 'static>(
        &mut self,
        reference: &TargetReference<T>,
    ) -> GeopResult<&'a T> {
        let (target, data) = self.part.lookup(reference, At::Now)?;
        self.dependencies.push(Dependency::Target {
            name: reference.name().to_string(),
            step: target.step.clone(),
        });
        Ok(data)
    }

    pub fn retrieve_input(&mut self, name: &str) -> Option<I> {
        self.dependencies.push(Dependency::Input(name.to_string()));
        self.part.input(name)
    }

    pub fn retrieve_state(&mut self, name: &str) -> S {
        self.dependencies.push(Dependency::State(name.to_string()));
        self.part.state(name)
    }
}

/// The part read at one position: now ([`Part::current`]), or before one
/// step of the last run ([`Part::view`]), which is what that step is edited
/// against and what scrubbing the timeline shows. Reading builds nothing, so
/// a view shows the same however the editor got there.
pub struct View<'a, I, S> {
    part: &'a Part<I, S>,
    at: At,
}

impl<'a, I: Input, S: State> View<'a, I, S> {
    pub fn retrieve_target<T: 'static>(&self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        Ok(self.part.lookup(reference, self.at)?.1)
    }

    pub fn retrieve_input(&self, name: &str) -> Option<I> {
        self.part.input(name)
    }

    pub fn retrieve_state(&self, name: &str) -> S {
        self.part.state(name)
    }
}
