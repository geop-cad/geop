use std::{
    any::Any,
    collections::{BTreeMap, BTreeSet, HashSet},
    rc::Rc,
};

use indexmap::IndexSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::system::Variable,
};

use crate::{
    constraint::Constraint,
    target::{Dependency, Revisable, Revision, Target, TargetReference, TargetRegistry},
};

/// A value a run is given by name, which no step changes: a parameter.
pub trait Input: Clone + PartialEq + 'static {}

impl<T: Clone + PartialEq + 'static> Input for T {}

/// A value the state holds by name, which a solver moves until the
/// [`Constraint`]s hold: a pose, say. A name the run gives no value reads as
/// the default.
///
/// A solver moves a value by steps along it ([`Variable::retract`]): a
/// number by adding to it, a pose by the exponential map of a twist. A
/// constraint's derivatives are by such a step from where the value is.
/// Its numbers ([`Variable::Scalar`]) are what the state, and the
/// constraints on it, are made of; `==` on them asks whether two enclosures
/// are the same, as a check for change does, never whether two values
/// could be equal.
pub trait State:
    Variable<Scalar: geop_core_math::scalars::Scalar + PartialEq> + PartialEq + Default + 'static
{
}

/// A value given to a run from outside, and the run in which it last changed.
/// A value that is removed stays as `None` while anything depends on it, so
/// that what read it sees the change.
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
    /// What tells one state of the part from another: it ticks with every
    /// run, and with every [`Part::update`] between runs. When a version
    /// was made, and when it last changed, is told by it.
    revision: Revision,
    /// The current or last run: what tells its versions from those only
    /// kept for later steps.
    run: Revision,
    /// The steps of the current or last run, in order. A step's id is what
    /// identifies it across runs; its position only orders it within one.
    steps: IndexSet<String>,
    targets: TargetRegistry<I, S>,
    /// The constraints the steps of the current or last run defined.
    constraints: BTreeSet<String>,
    inputs: BTreeMap<String, Given<I>>,
    state: BTreeMap<String, Given<S>>,
    /// How many versions of targets it has built — generators run — ever.
    built: usize,
}

impl<I, S> Default for Part<I, S> {
    fn default() -> Self {
        Self {
            revision: 0,
            run: 0,
            steps: IndexSet::new(),
            targets: TargetRegistry::default(),
            constraints: BTreeSet::new(),
            inputs: BTreeMap::new(),
            state: BTreeMap::new(),
            built: 0,
        }
    }
}

impl<I: Input, S: State> Part<I, S> {
    /// Starts a run with the given `inputs` and `state`, complete: what the
    /// last run gave and this one does not is removed. A [`crate::program_runner::ProgramRunner`]
    /// drives a part through a program; these drive it by hand, as a test of
    /// one operation does.
    pub fn begin_run(&mut self, inputs: &BTreeMap<String, I>, state: &BTreeMap<String, S>) {
        self.revision += 1;
        self.run = self.revision;
        self.steps.clear();
        self.constraints.clear();
        replace_given(&mut self.inputs, inputs, self.revision);
        replace_given(&mut self.state, state, self.revision);
    }

    /// Starts the step `step`. Fails if a step of that id ran before in this
    /// run: the id is what tells one step's versions from another's.
    pub fn begin_step(&mut self, step: &str) -> GeopResult<()> {
        if !self.steps.insert(step.to_string()) {
            return Err(GeopError::new(format!(
                "step `{step}` appears twice in the program"
            )));
        }
        Ok(())
    }

    /// Deletes the versions no step made, of this run or of the `later` steps
    /// it stopped before, and the removed inputs and state no version left
    /// depends on.
    pub fn end_run(&mut self, later: HashSet<&str>) {
        let run = self.run;
        (self.targets).retain(|target| target.run == run || later.contains(target.step.as_str()));
        let read: HashSet<&Dependency> = self.targets.dependencies().collect();
        self.inputs.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::Input(name.clone()))
        });
        self.state.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::State(name.clone()))
        });
    }

    fn step(&self) -> &str {
        self.steps.last().map_or("", String::as_str)
    }

    /// How many versions of targets it has built — generators run — ever.
    /// What a run costs grows with this, so a test can bound the work an
    /// edit does by counting, rather than timing it.
    pub fn built(&self) -> usize {
        self.built
    }

    /// The part as it is now: during a run, with what the steps so far made;
    /// after one, the finished part.
    pub fn current(&self) -> View<'_, I, S> {
        View {
            part: self,
            before: self.steps.len(),
        }
    }

    /// The part as it was before the step `step` of the last run.
    pub fn view_before(&self, step: &str) -> GeopResult<View<'_, I, S>> {
        let before = (self.steps.get_index_of(step))
            .ok_or_else(|| GeopError::new(format!("step `{step}` is not in the last run")))?;
        Ok(View { part: self, before })
    }

    /// The part as the step `step` of the last run left it, if it ran.
    pub fn view_after(&self, step: &str) -> Option<View<'_, I, S>> {
        let at = self.steps.get_index_of(step)?;
        Some(View {
            part: self,
            before: at + 1,
        })
    }

    /// The version of `name` a read sees before the step at position
    /// `before` of this run: of the versions this run made, the one whose
    /// step came last.
    fn resolve(&self, name: &str, before: usize) -> GeopResult<&Target<I, S>> {
        self.targets
            .versions(name)
            .filter(|target| target.run == self.run)
            .filter_map(|target| Some((self.steps.get_index_of(&target.step)?, target)))
            .filter(|&(position, _)| position < before)
            .max_by_key(|&(position, _)| position)
            .map(|(_, target)| target)
            .ok_or_else(|| {
                let reader = match (self.steps.get_index(before), self.steps.last()) {
                    (Some(step), _) => format!("before step `{step}`"),
                    (None, Some(last)) => format!("in step `{last}` or before"),
                    (None, None) => "before any step".to_string(),
                };
                GeopError::new(format!("target `{name}` is not defined {reader}"))
            })
    }

    /// The version of `name` a read sees now.
    fn latest(&self, name: &str) -> Option<&Target<I, S>> {
        self.resolve(name, self.steps.len()).ok()
    }

    fn lookup<T: 'static>(
        &self,
        reference: &TargetReference<T>,
        before: usize,
    ) -> GeopResult<(&Target<I, S>, &T)> {
        let name = reference.name();
        let target = self.resolve(name, before)?;
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

    /// Whether `dependency` of the target `own`, read before the step at
    /// position `before`, is unchanged since `revision`. A version counts as
    /// changed unless a read there still sees it, and its value is no newer
    /// than `revision`. A target reads its own name as of the steps before
    /// its own (see [`Reader`]).
    fn unchanged_since(
        &self,
        dependency: &Dependency,
        revision: Revision,
        own: &str,
        before: usize,
    ) -> bool {
        match dependency {
            Dependency::Target { name, step } => (self.resolve(name, sees(name, own, before)).ok())
                .is_some_and(|target| &target.step == step && target.changed_at <= revision),
            Dependency::Input(name) => changed_at(&self.inputs, name) <= revision,
            Dependency::State(name) => changed_at(&self.state, name) <= revision,
        }
    }

    /// Defines the target `reference` as what `generator` makes of `args`.
    /// Fails if a step has defined it before in this run.
    ///
    /// A generator is a plain function, not a closure: what it makes may
    /// depend only on `args`, which a reuse compares, and on what it reads,
    /// which is recorded. A value captured from outside would be neither, and
    /// the version would be reused after it changed:
    ///
    /// ```compile_fail
    /// # use geop_core_program::{part::{Input, Part, State}, target::TargetReference};
    /// fn define<I: Input, S: State>(part: &mut Part<I, S>, outside: u32) {
    ///     let reference = TargetReference::<u32>::new("t");
    ///     part.define_target(&reference, &(), |_, _| Ok(outside));
    /// }
    /// ```
    pub fn define_target<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: Generator<I, S, Args, Out>,
    ) -> GeopResult<()> {
        self.define(reference, args, generator)
    }

    /// [`Part::define_target`], for the framework's own generators, which
    /// wrap an operation's.
    fn define<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: impl Fn(&mut Reader<I, S>, &Args) -> GeopResult<Out> + 'static,
    ) -> GeopResult<()> {
        let name = reference.name();
        if let Some(latest) = self.latest(name) {
            return Err(GeopError::new(format!(
                "target `{name}` is defined twice: by step `{}` and by step `{}`",
                latest.step,
                self.step()
            )));
        }
        self.make_version(reference, args, generator)
    }

    /// Revises the target `reference`, which an earlier step defined, to
    /// what `generator` makes of its latest version and `args`. The latest
    /// version stays as it is, for the steps that read it and for the next
    /// run; the generator revises a copy of it ([`Revisable::to_revise`]).
    /// A plain function, like [`Part::define_target`]'s.
    pub fn revise_target<Out: Revisable, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: fn(Out, &mut Reader<I, S>, &Args) -> GeopResult<Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        if let Some(latest) = self.latest(name)
            && latest.step == self.step()
        {
            return Err(GeopError::new(format!(
                "target `{name}` is made twice by step `{}`",
                self.step()
            )));
        }
        let own = reference.clone();
        self.make_version(reference, args, move |reader, args| {
            let previous = reader.retrieve_target(&own)?.to_revise();
            generator(previous, reader, args)
        })
    }

    /// Revises the target `reference` as [`Part::revise_target`] does, or,
    /// if no step has made it yet in this run, makes it from its default:
    /// for a value a program's steps build up one after another from
    /// nothing, like a part's model. A version that failed is one made: a
    /// step after it fails too, rather than starting afresh.
    pub fn revise_or_start<Out: Revisable + Default, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: fn(Out, &mut Reader<I, S>, &Args) -> GeopResult<Out>,
    ) -> GeopResult<()> {
        if self.latest(reference.name()).is_some() {
            return self.revise_target(reference, args, generator);
        }
        self.make_version(reference, args, move |reader, args| {
            generator(Out::default(), reader, args)
        })
    }

    /// Makes this step's version of `reference`, reusing the last run's when
    /// nothing it read has changed. The version keeps how it is made, to
    /// be made again between runs ([`Part::update`]).
    fn make_version<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: impl Fn(&mut Reader<I, S>, &Args) -> GeopResult<Out> + 'static,
    ) -> GeopResult<()> {
        let name = reference.name();
        let step = (self.steps.last().cloned())
            .ok_or_else(|| GeopError::new(format!("target `{name}` is made outside of a step")))?;
        let (run, before) = (self.run, self.steps.len());
        let reusable = self.targets.get(name, &step).is_some_and(|target| {
            target.data.as_ref().is_ok_and(|data| data.is::<Out>())
                && target.args.downcast_ref::<Args>() == Some(args)
                && (target.dependencies.iter()).all(|dependency| {
                    self.unchanged_since(dependency, target.verified_at, name, before)
                })
        });
        if reusable && let Some(target) = self.targets.get_mut(name, &step) {
            target.verified_at = self.revision;
            target.run = run;
            return Ok(());
        }
        let args = Rc::new(args.clone());
        let kept = args.clone();
        let remake = Remake {
            make: Rc::new(move |reader| Ok(Box::new(generator(reader, &kept)?) as Box<dyn Any>)),
            same: |a, b| a.downcast_ref::<Out>() == b.downcast_ref::<Out>(),
        };
        self.make(name, step, args, run, before, remake)
    }

    /// Makes the version of `name` of the step `step` with `remake`,
    /// reading before the step at position `before`, and keeps it with what
    /// it read: changed now, unless it is equal to the version it replaces.
    fn make(
        &mut self,
        name: &str,
        step: String,
        args: Rc<dyn Any>,
        run: Revision,
        before: usize,
        remake: Remake<I, S>,
    ) -> GeopResult<()> {
        self.built += 1;
        let mut reader = Reader {
            part: self,
            dependencies: Vec::new(),
            own: name.to_string(),
            before,
        };
        let made = (remake.make)(&mut reader)
            .map_err(|e| e.with_context(format!("making target `{name}` in step `{step}`")));
        let mut dependencies = reader.dependencies;
        dependencies.sort();
        dependencies.dedup();
        let (data, result) = match made {
            Ok(data) => (Ok(data), Ok(())),
            Err(e) => (Err(e.root_message().to_string()), Err(e)),
        };
        let changed_at = match (self.targets.get(name, &step), &data) {
            (Some(old), Ok(new))
                if (old.data.as_ref())
                    .is_ok_and(|old_data| (old.remake.same)(&**old_data, &**new)) =>
            {
                old.changed_at
            }
            _ => self.revision,
        };
        let target = Target {
            step,
            args,
            dependencies,
            data,
            changed_at,
            verified_at: self.revision,
            run,
            remake,
        };
        self.targets.insert(name, target);
        result
    }

    /// Brings the targets `names` up to date with the state `state` — and,
    /// first, every version they read that is not — as the last run would
    /// have made them with it, without running the program again: how a
    /// solver moving the state reads its constraints. Only what read state
    /// that changed is made again. Until the next run, the part's other
    /// targets stay as the last run left them, and so does what failed to
    /// be made: an update keeps going, and the failure is the version's.
    pub fn update(&mut self, state: &BTreeMap<String, S>, names: &[&str]) {
        self.revision += 1;
        replace_given(&mut self.state, state, self.revision);
        for name in names {
            if let Some(step) = self.latest(name).map(|target| target.step.clone()) {
                self.bring_up_to_date(name, &step);
            }
        }
    }

    /// The version of `name` of the step `step`, made again if anything it
    /// read changed since it was made — what it read brought up to date
    /// first.
    fn bring_up_to_date(&mut self, name: &str, step: &str) {
        let revision = self.revision;
        let Some(target) = self.targets.get(name, step) else {
            return;
        };
        if target.verified_at == revision {
            return;
        }
        let Some(position) = self.steps.get_index_of(step) else {
            return;
        };
        let before = position + 1;
        let dependencies = target.dependencies.clone();
        for dependency in &dependencies {
            if let Dependency::Target { name: read, step } = dependency
                && let Ok(seen) = self.resolve(read, sees(read, name, before))
                && &seen.step == step
            {
                let seen = seen.step.clone();
                self.bring_up_to_date(read, &seen);
            }
        }
        let Some(target) = self.targets.get(name, step) else {
            return;
        };
        let fresh = (dependencies.iter())
            .all(|dependency| self.unchanged_since(dependency, target.verified_at, name, before));
        if fresh {
            if let Some(target) = self.targets.get_mut(name, step) {
                target.verified_at = revision;
            }
            return;
        }
        let (args, run, remake) = (target.args.clone(), target.run, target.remake.clone());
        // A failure is the version's to say: what reads it fails in turn.
        let _ = self.make(name, step.to_string(), args, run, before, remake);
    }

    /// Defines the constraint `name` as what `generator` makes of `args`.
    /// A constraint is a target like any other: rebuilt only when what it
    /// read changes, the state above all. Only what is defined here is
    /// reported as a constraint, and only after its derivatives are checked.
    /// A plain function, like [`Part::define_target`]'s.
    pub fn define_constraint<Args: PartialEq + Clone + 'static>(
        &mut self,
        name: &str,
        args: &Args,
        generator: Generator<I, S, Args, Constraint<S>>,
    ) -> GeopResult<()> {
        let reference = TargetReference::<Constraint<S>>::new(name);
        let own = name.to_string();
        self.define(&reference, args, move |reader, args| {
            let constraint = generator(reader, args)?;
            constraint.check(&own, |state| {
                (reader.dependencies)
                    .contains(&Dependency::State(state.to_string()))
                    .then(|| reader.part.state(state).dimension())
            })?;
            Ok(constraint)
        })?;
        self.constraints.insert(name.to_string());
        Ok(())
    }

    /// The constraints defined so far in this run, or in the last.
    pub fn constraints(&self) -> BTreeMap<&str, &Constraint<S>> {
        self.constraints
            .iter()
            .filter_map(|name| {
                let data = self.latest(name)?.data.as_ref().ok()?;
                Some((name.as_str(), data.downcast_ref::<Constraint<S>>()?))
            })
            .collect()
    }
}

/// What makes a target's value from its arguments and what it reads.
pub type Generator<I, S, Args, Out> = fn(&mut Reader<I, S>, &Args) -> GeopResult<Out>;

/// A target's generator together with its arguments: what makes a version
/// of it, from what it reads alone.
type Make<I, S> = Rc<dyn Fn(&mut Reader<I, S>) -> GeopResult<Box<dyn Any>>>;

/// How a version of a target is made again: its generator with its
/// arguments, and how two values it made are compared.
pub(crate) struct Remake<I, S> {
    make: Make<I, S>,
    same: fn(&dyn Any, &dyn Any) -> bool,
}

// By hand, since a derive would ask the same of `I` and `S`.
impl<I, S> Clone for Remake<I, S> {
    fn clone(&self) -> Self {
        Remake {
            make: self.make.clone(),
            same: self.same,
        }
    }
}

/// Before which step position the target `own`, made before the step at
/// position `before`, reads `name`: its own name as of the steps before
/// its own — what it revises — anything else as of its own step.
fn sees(name: &str, own: &str, before: usize) -> usize {
    match name == own {
        true => before.saturating_sub(1),
        false => before,
    }
}

/// What a target's generator reads the part through, as of its own step.
/// It records each read, so that the target is rebuilt when any of it
/// changes.
pub struct Reader<'a, I, S> {
    part: &'a Part<I, S>,
    dependencies: Vec<Dependency>,
    /// The target being made.
    own: String,
    /// The position of the step after the one it is made by.
    before: usize,
}

/// What reads a part — a generator through its [`Reader`], which records
/// each read, or an editor through a [`View`], which does not — so that what
/// is worked out from the part is written once for both. A view also reads
/// through `&self`, by its inherent methods of the same names.
pub trait Retrieve<'a, I, S> {
    fn retrieve_target<T: 'static>(&mut self, reference: &TargetReference<T>) -> GeopResult<&'a T>;

    fn retrieve_input(&mut self, name: &str) -> Option<I>;

    fn retrieve_state(&mut self, name: &str) -> S;
}

impl<'a, I: Input, S: State> Retrieve<'a, I, S> for Reader<'a, I, S> {
    fn retrieve_target<T: 'static>(&mut self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        let before = sees(reference.name(), &self.own, self.before);
        let (target, data) = self.part.lookup(reference, before)?;
        self.dependencies.push(Dependency::Target {
            name: reference.name().to_string(),
            step: target.step.clone(),
        });
        Ok(data)
    }

    fn retrieve_input(&mut self, name: &str) -> Option<I> {
        self.dependencies.push(Dependency::Input(name.to_string()));
        self.part.input(name)
    }

    fn retrieve_state(&mut self, name: &str) -> S {
        self.dependencies.push(Dependency::State(name.to_string()));
        self.part.state(name)
    }
}

impl<'a, I: Input, S: State> Retrieve<'a, I, S> for View<'a, I, S> {
    fn retrieve_target<T: 'static>(&mut self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        View::retrieve_target(self, reference)
    }

    fn retrieve_input(&mut self, name: &str) -> Option<I> {
        View::retrieve_input(self, name)
    }

    fn retrieve_state(&mut self, name: &str) -> S {
        View::retrieve_state(self, name)
    }
}

/// The part read before one position of the run: now ([`Part::current`]),
/// or before one step of the last run ([`Part::view_before`]), which is what
/// that step is edited against and what scrubbing the timeline shows.
/// Reading builds nothing, so a view shows the same however the editor got
/// there.
pub struct View<'a, I, S> {
    part: &'a Part<I, S>,
    before: usize,
}

// By hand, since a derive would ask the same of `I` and `S`: a view is
// only where it looks from.
impl<I, S> Clone for View<'_, I, S> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<I, S> Copy for View<'_, I, S> {}

impl<'a, I: Input, S: State> View<'a, I, S> {
    pub fn retrieve_target<T: 'static>(&self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        Ok(self.part.lookup(reference, self.before)?.1)
    }

    pub fn retrieve_input(&self, name: &str) -> Option<I> {
        self.part.input(name)
    }

    pub fn retrieve_state(&self, name: &str) -> S {
        self.part.state(name)
    }

    /// Every target the view sees whose value is a `T`, by name, in the
    /// order the steps made them — of one step, by name: the sketches a
    /// step could pick, say, the newest last.
    pub fn targets_of<T: 'static>(&self) -> Vec<(&'a str, &'a T)> {
        let mut found: Vec<(Option<usize>, &'a str, &'a T)> = (self.part.targets.names())
            .filter_map(|name| {
                let target = self.part.resolve(name, self.before).ok()?;
                let data = target.data.as_ref().ok()?.downcast_ref::<T>()?;
                Some((self.part.steps.get_index_of(&target.step), name, data))
            })
            .collect();
        found.sort_by_key(|&(position, name, _)| (position, name));
        found
            .into_iter()
            .map(|(_, name, data)| (name, data))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test::{Pose, Sc};
    use geop_core_math::scalars::Scalar;

    /// A removed input is kept, as gone, while a version depends on it, and
    /// forgotten once none does.
    #[test]
    fn removed_input_is_forgotten_once_nothing_reads_it() {
        let mut part = Part::<Sc, Pose>::default();
        let x = BTreeMap::from([("x".to_string(), Sc::ONE)]);
        let reads_x = |part: &mut Part<Sc, Pose>| {
            part.begin_step("s").unwrap();
            let reference = TargetReference::<Option<Sc>>::new("a");
            part.define_target(&reference, &(), |reader, _| Ok(reader.retrieve_input("x")))
                .unwrap();
        };

        part.begin_run(&x, &BTreeMap::new());
        reads_x(&mut part);
        part.end_run(HashSet::new());

        part.begin_run(&BTreeMap::new(), &BTreeMap::new());
        reads_x(&mut part);
        part.end_run(HashSet::new());
        assert!(part.inputs.contains_key("x"));

        part.begin_run(&BTreeMap::new(), &BTreeMap::new());
        part.end_run(HashSet::new());
        assert!(!part.inputs.contains_key("x"));
    }
}
