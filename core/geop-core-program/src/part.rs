use std::{
    cell::RefCell,
    collections::{BTreeMap, HashSet},
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    element::{ElementReference, HasElements, Kind},
    instance::{
        Defining, Factor, InInstance, INSTANCE_SEPARATOR, Instance, InstanceGenerator, Placement,
        check_name, instance_name, instance_target, place, split_path,
    },
    revisioned_value::RevisionedValue,
    target::{Dependency, Revision, Target, TargetReference, TargetRegistry},
};

/// A value a run is given by name, up front, which no step changes: a
/// parameter.
pub trait Input: Clone + PartialEq + 'static {}

impl<T: Clone + PartialEq + 'static> Input for T {}

/// A value the state holds by name: what a step's targets read, like an
/// input, but declared by reading it — a name the run gives no value reads
/// as the default — rather than given up front. Outside the part, the state
/// is what moves: kept with the program between edits, and changed by
/// whatever solves it.
pub trait State: Clone + PartialEq + Default + 'static {}

impl<T: Clone + PartialEq + Default + 'static> State for T {}

/// What a program builds, and what it builds from. Each run states its
/// inputs and state in full. Its steps define targets, in order, and may
/// revise targets earlier steps made; every revision is a
/// version of its own, and a read sees the latest version made before it in
/// the same run. A part is read only through a [`Reader`].
///
/// A version is reused from the last run when its step made it from equal
/// arguments and nothing it read has changed since. A version rebuilt into
/// a value equal to the last one counts as unchanged, so what read it is
/// reused in turn.
pub struct Part<I, S> {
    /// The current or last run, counted from 1: what tells its versions
    /// from those only kept for later steps, and when a version was made or
    /// last changed.
    run: Revision,
    inputs: BTreeMap<String, RevisionedValue<I>>,
    state: BTreeMap<String, RevisionedValue<S>>,
    /// How many steps the current or last run began. A step is told by its
    /// position, in this run and the next.
    steps: usize,
    targets: TargetRegistry,
}

impl<I, S> Default for Part<I, S> {
    fn default() -> Self {
        Self {
            run: 0,
            inputs: BTreeMap::new(),
            steps: 0,
            state: BTreeMap::new(),
            targets: TargetRegistry::default(),
        }
    }
}

impl<I: Input, S: State> Part<I, S> {
    /// Starts a run with the given `inputs` and `state`, complete: what the
    /// last run gave and this one does not is removed. A [`crate::program_runner::ProgramRunner`]
    /// drives a part through a program; these drive it by hand, as a test of
    /// one operation does.
    pub fn begin_run(&mut self, inputs: &BTreeMap<String, I>, state: &BTreeMap<String, S>) {
        self.run += 1;
        self.steps = 0;
        RevisionedValue::replace_all(&mut self.inputs, inputs, self.run);
        RevisionedValue::replace_all(&mut self.state, state, self.run);
    }

    /// Starts the next step.
    pub fn begin_step(&mut self) {
        self.steps += 1;
    }

    /// Deletes the versions no step made, of this run or of the `later`
    /// steps after the ones it ran, which it stopped before, and the removed
    /// inputs and state no version left depends on.
    pub fn end_run(&mut self, later: usize) {
        let (run, ran) = (self.run, self.steps);
        (self.targets)
            .retain(|target| target.run == run || (ran..ran + later).contains(&target.step));
        let read: HashSet<&Dependency> = self.targets.dependencies().collect();
        let prefixes: Vec<&str> = (read.iter())
            .filter_map(|dependency| match dependency {
                Dependency::StatesUnder(prefix) => Some(prefix.as_str()),
                _ => None,
            })
            .collect();
        self.inputs.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::Input(name.clone()))
        });
        self.state.retain(|name, entry| {
            entry.value.is_some()
                || read.contains(&Dependency::State(name.clone()))
                || prefixes.iter().any(|prefix| name.starts_with(prefix))
        });
    }

    /// The part as it is before the step at position `before`, recording
    /// nothing: what that step is edited against, and what scrubbing the
    /// timeline shows.
    pub fn reader(&self, before: usize) -> Reader<'_, I, S> {
        Reader {
            part: self,
            before,
            recording: None,
        }
    }

    /// The part as it is now: during a run, with what the steps so far made;
    /// after one, the finished part.
    pub fn current(&self) -> Reader<'_, I, S> {
        self.reader(self.steps)
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
        let name = reference.name();
        if let Ok(latest) = self.current().version(name) {
            return Err(GeopError::new(format!(
                "target `{name}` is defined twice: by the steps at positions {} and {}",
                latest.step,
                self.steps - 1
            )));
        }
        self.make_version(reference, args, generator)
    }

    /// Revises the target `reference`: makes this step's version of it, by
    /// `generator`, from the latest version, which the generator reads as
    /// [`Reader::previous`] — the default, if no step has made it yet in this
    /// run: a value a program's steps build up one after another from
    /// nothing, like a part's topology. The latest version stays as it is,
    /// for the steps that read it and for the next run. A version that
    /// failed is one made: a step after it fails too, rather than starting
    /// afresh. A plain function, like [`Part::define_target`]'s.
    pub fn revise_target<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: Generator<I, S, Args, Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        if let Ok(latest) = self.current().version(name)
            && latest.step + 1 == self.steps
        {
            return Err(GeopError::new(format!(
                "target `{name}` is made twice by the step at position {}",
                latest.step
            )));
        }
        self.make_version(reference, args, generator)
    }

    /// Places the instance `name`: what `generator` makes of `args`, given
    /// the state under its name — `b.pose` for the state `name/b.pose` —
    /// to run its part with (see [`crate::instance`]). Fails if a step has
    /// placed an instance of that name before in this run. A plain
    /// function, like [`Part::define_target`]'s.
    pub fn define_instance<Args: PartialEq + Clone + 'static>(
        &mut self,
        name: &str,
        args: &Args,
        generator: InstanceGenerator<I, S, Args>,
    ) -> GeopResult<()> {
        check_name(name)?;
        let defining = Defining {
            name: name.to_string(),
            args: args.clone(),
            generator,
        };
        let reference = TargetReference::<Instance<I, S>>::new(instance_target(name));
        self.define_target(&reference, &defining, place)
    }

    /// Makes this step's version of `reference`, reusing the last run's when
    /// nothing it read has changed, and keeps it with what it read: changed
    /// now, unless it is equal to the version it replaces.
    fn make_version<Out: PartialEq + 'static, Args: PartialEq + Clone + 'static>(
        &mut self,
        reference: &TargetReference<Out>,
        args: &Args,
        generator: Generator<I, S, Args, Out>,
    ) -> GeopResult<()> {
        let name = reference.name();
        let step = (self.steps.checked_sub(1))
            .ok_or_else(|| GeopError::new(format!("target `{name}` is made outside of a step")))?;
        let (run, before) = (self.run, self.steps);
        // A version is reused if it was made from equal arguments and
        // nothing it read changed since: a target it read is still seen
        // where it was read — its own name as of the steps before its own
        // (see `sees`) — made by the same step and no newer, and what it
        // found undefined still is.
        let reusable = self.targets.get(name, step).is_some_and(|target| {
            let unchanged = |dependency: &Dependency| match dependency {
                Dependency::Target { name: read, step } => (self.reader(sees(read, name, before)))
                    .version(read)
                    .is_ok_and(|seen| seen.step == *step && seen.changed_at <= target.run),
                Dependency::Undefined(read) => (self.reader(sees(read, name, before)))
                    .version(read)
                    .is_err(),
                Dependency::Input(read) => {
                    RevisionedValue::changed_at(&self.inputs, read) <= target.run
                }
                Dependency::State(read) => {
                    RevisionedValue::changed_at(&self.state, read) <= target.run
                }
                Dependency::StatesUnder(prefix) => (under(&self.state, prefix))
                    .all(|(_, entry)| entry.changed_at <= target.run),
            };
            target.made_from::<Out, Args>(args) && target.dependencies.iter().all(unchanged)
        });
        if reusable && let Some(target) = self.targets.get_mut(name, step) {
            target.run = run;
            return Ok(());
        }

        let reader = Reader {
            part: self,
            before,
            recording: Some(Recording {
                own: name.to_string(),
                dependencies: RefCell::new(Vec::new()),
            }),
        };
        let made = generator(&reader, args).map_err(|e| {
            e.with_context(format!(
                "making target `{name}` in the step at position {step}"
            ))
        });
        let mut dependencies = (reader.recording)
            .map(|recording| recording.dependencies.into_inner())
            .unwrap_or_default();
        dependencies.sort();
        dependencies.dedup();
        let (data, result) = match made {
            Ok(data) => (Ok(data), Ok(())),
            Err(e) => (Err(e.root_message().to_string()), Err(e)),
        };
        let previous = self.targets.get(name, step);
        let target = Target::new(step, args.clone(), dependencies, data, previous, run);
        self.targets.insert(name, target);
        result
    }
}

/// The entries of `values` whose names start with `prefix`: a range of
/// them, as they are sorted by name.
fn under<'v, T>(
    values: &'v BTreeMap<String, T>,
    prefix: &str,
) -> impl Iterator<Item = (&'v String, &'v T)> {
    (values.range(prefix.to_string()..)).take_while(move |(name, _)| name.starts_with(prefix))
}

/// What makes a target's value from its arguments and what it reads.
pub type Generator<I, S, Args, Out> = fn(&Reader<I, S>, &Args) -> GeopResult<Out>;

/// Before which step position the target `own`, made before the step at
/// position `before`, reads `name`: its own name as of the steps before
/// its own — what it revises — anything else as of its own step.
fn sees(name: &str, own: &str, before: usize) -> usize {
    match name == own {
        true => before.saturating_sub(1),
        false => before,
    }
}

/// What a target being made has read.
struct Recording {
    /// The target being made.
    own: String,
    dependencies: RefCell<Vec<Dependency>>,
}

/// The part as it is before one position of the run, and the only way it
/// is read: by a target's generator, as of its own step, recording every
/// read so that the target is rebuilt when any of it changes — or by an
/// editor, recording nothing: the part now ([`Part::current`]), or before
/// or after one step of the last run ([`Part::reader`]), what that
/// step is edited against and what scrubbing the timeline shows.
///
/// A target is read by its elements ([`ElementReference`]), never as a
/// whole: a target meant to be read whole has a kind of element that is all
/// of it (see [`HasElements`]).
pub struct Reader<'a, I, S> {
    part: &'a Part<I, S>,
    /// The position of the step it reads before.
    before: usize,
    recording: Option<Recording>,
}

// By hand, since a derive would ask the same of `I` and `S`.
impl<I, S> Clone for Reader<'_, I, S> {
    fn clone(&self) -> Self {
        Self {
            part: self.part,
            before: self.before,
            recording: (self.recording.as_ref()).map(|recording| Recording {
                own: recording.own.clone(),
                dependencies: recording.dependencies.clone(),
            }),
        }
    }
}

impl<'a, I: Input, S: State> Reader<'a, I, S> {
    /// Records `dependency`, if it records.
    fn record(&self, dependency: Dependency) {
        if let Some(recording) = &self.recording {
            recording.dependencies.borrow_mut().push(dependency);
        }
    }

    /// The version of `name` it sees: of the versions this run made, the
    /// one whose step came last before it — as of the step before for the
    /// target being made.
    fn version(&self, name: &str) -> GeopResult<&'a Target> {
        let part = self.part;
        let before = match &self.recording {
            Some(recording) => sees(name, &recording.own, self.before),
            None => self.before,
        };
        (part.targets.versions(name))
            .filter(|target| target.run == part.run && target.step < before)
            .max_by_key(|target| target.step)
            .ok_or_else(|| {
                let reader = match before.checked_sub(1) {
                    Some(last) => format!("by the step at position {last} or before"),
                    None => "before any step".to_string(),
                };
                GeopError::new(format!("target `{name}` is not defined {reader}"))
            })
    }

    /// The target `reference`, whole, the read recorded: for the framework,
    /// which revises targets whole.
    pub(crate) fn target<T: 'static>(&self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        let name = reference.name();
        let target = self.version(name)?;
        self.record(Dependency::Target {
            name: name.to_string(),
            step: target.step,
        });
        target.value(name)
    }

    /// A copy of the target being made as the steps before left it: what a
    /// revision ([`Part::revise_target`]) starts from — the default, if no
    /// step made it yet in this run. The read is recorded.
    pub fn previous<T: Clone + Default + 'static>(&self) -> GeopResult<T> {
        let own = (self.recording.as_ref())
            .ok_or_else(|| GeopError::new("only a target being made has a previous version"))?
            .own
            .clone();
        if self.version(&own).is_err() {
            self.record(Dependency::Undefined(own));
            return Ok(T::default());
        }
        self.target(&TargetReference::<T>::new(own)).cloned()
    }

    /// The element `reference` names, of the part itself. One of an
    /// instance placed in it is an error naming the instance: it is where
    /// that instance is, which [`Reader::retrieve_placed`] says.
    pub fn retrieve_element<T: HasElements<K>, K: Kind>(
        &self,
        reference: &ElementReference<T, K>,
    ) -> GeopResult<T::Element<'a>> {
        let placed = self.retrieve_placed(reference)?;
        match placed.instance.is_empty() {
            true => Ok(placed.element),
            false => Err(GeopError::new(format!(
                "{reference:?} is of the placed part `{}`: this takes elements of the part itself",
                placed.instance
            ))),
        }
    }

    /// The element `reference` names, of the part itself or of an instance
    /// placed in it — however deep — as it is in its own part, with the
    /// instance's path: where that is, [`Reader::placement`] says. The read
    /// is recorded as one of the instance, which is made again whenever
    /// its part is built again.
    pub fn retrieve_placed<T: HasElements<K>, K: Kind>(
        &self,
        reference: &ElementReference<T, K>,
    ) -> GeopResult<InInstance<T::Element<'a>>> {
        let (path, target) = split_path(reference.target().name());
        let resolve = |reader: &Reader<'a, I, S>| {
            (reader.target(&TargetReference::<T>::new(target))?).element(reference.name())
        };
        let element = match path.is_empty() {
            true => resolve(self),
            false => self.inside(path).and_then(|inside| resolve(&inside)),
        };
        let element = element.map_err(|e| e.with_context(format!("resolving {reference:?}")))?;
        Ok(InInstance {
            element,
            instance: path.to_string(),
        })
    }

    /// The instance `name` placed in the part, the read recorded.
    pub fn instance(&self, name: &str) -> GeopResult<&'a Instance<I, S>> {
        self.target(&TargetReference::new(instance_target(name)))
            .map_err(|e| e.with_context(format!("the placed part `{name}`")))
    }

    /// Every instance placed in the part — not in its instances — by name,
    /// in the order the steps placed them, each read recorded.
    pub fn instances(&self) -> Vec<(&'a str, &'a Instance<I, S>)> {
        (self.versions_of::<Instance<I, S>>().into_iter())
            .filter_map(|(target, version)| {
                let name = instance_name(target)?;
                Some((name, version.value::<Instance<I, S>>(target).ok()?))
            })
            .collect()
    }

    /// The part of the instance at `path` — `bolt`, or `asm/bolt` — as its
    /// program left it, the read of each instance on the way recorded.
    pub fn inside(&self, path: &str) -> GeopResult<Reader<'a, I, S>> {
        let mut names = path.split(INSTANCE_SEPARATOR);
        let first = names.next().unwrap_or_default();
        let mut inside = self.instance(first)?.reader();
        for name in names {
            inside = inside.instance(name)?.reader();
        }
        Ok(inside)
    }

    /// Where the instance at `path` is in the part, as factors composed
    /// outermost first ([`Factor`]): the values of the state that place it
    /// and the parts it is placed in, read and recorded. A state of an
    /// instance's own — what places a part placed in it — is named as the
    /// part names it, behind the instance's path.
    pub fn placement(&self, path: &str) -> GeopResult<Vec<Factor<S>>> {
        let ctx = |e: GeopError| e.with_context(format!("where the placed part {path:?} is"));
        let mut factors = Vec::new();
        let mut level = self.clone();
        let mut prefix = String::new();
        for name in path.split(INSTANCE_SEPARATOR) {
            let mut placed = self.placed(&level, name, &prefix, 0).map_err(ctx)?;
            factors.append(&mut placed);
            prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
            level = level.instance(name).map_err(ctx)?.reader();
        }
        Ok(factors)
    }

    /// How deep a chain of copies may go before it is taken for a cycle.
    const COPIES: usize = 1 << 16;

    /// The factors placing the instance `name` of the part `level` reads —
    /// the part at `prefix` in the part read — in that part: its state's
    /// value, or a copy's motion after where what it copies is. A state is
    /// read from the part read's own, behind `prefix`, and recorded as
    /// read; where the run gives none, `level`'s own is where it is.
    fn placed(
        &self,
        level: &Reader<'a, I, S>,
        name: &str,
        prefix: &str,
        depth: usize,
    ) -> GeopResult<Vec<Factor<S>>> {
        if depth > Self::COPIES {
            return Err(GeopError::new(format!("the copies of `{name}` form a cycle")));
        }
        Ok(match &level.instance(name)?.placement {
            Placement::State(state) => {
                let state_here = format!("{prefix}{state}");
                vec![Factor::State {
                    value: (self.given_state(&state_here))
                        .unwrap_or_else(|| level.retrieve_state(state)),
                    state: state_here,
                }]
            }
            Placement::After { of, motion } => {
                let mut factors = vec![Factor::Fixed(motion.clone())];
                factors.append(&mut self.placed(level, of, prefix, depth + 1)?);
                factors
            }
        })
    }

    /// The input's value, if the run gives one.
    pub fn retrieve_input(&self, name: &str) -> Option<I> {
        self.record(Dependency::Input(name.to_string()));
        (self.part.inputs.get(name)).and_then(|entry| entry.value.clone())
    }

    /// The state's value; the default if the run gives none.
    pub fn retrieve_state(&self, name: &str) -> S {
        self.given_state(name).unwrap_or_default()
    }

    /// The state's value, if the run gives one.
    pub fn given_state(&self, name: &str) -> Option<S> {
        self.record(Dependency::State(name.to_string()));
        (self.part.state.get(name)).and_then(|entry| entry.value.clone())
    }

    /// Every value the run gives of the state under `prefix`, by its name
    /// after it, the read recorded: whatever of it changes, appears or
    /// goes, what read it is made again.
    pub fn states_under(&self, prefix: &str) -> BTreeMap<String, S> {
        self.record(Dependency::StatesUnder(prefix.to_string()));
        (under(&self.part.state, prefix))
            .filter_map(|(name, entry)| {
                let name = name.strip_prefix(prefix)?;
                Some((name.to_string(), entry.value.clone()?))
            })
            .collect()
    }

    /// Whether it has read the state `name`: what a check of a value read
    /// from the state, a constraint's derivatives, asks.
    pub fn has_read_state(&self, name: &str) -> bool {
        (self.recording.as_ref()).is_some_and(|recording| {
            (recording.dependencies.borrow()).contains(&Dependency::State(name.to_string()))
        })
    }

    /// Whether a step before it made a version of `reference`, built or
    /// failed, the read recorded: what tells a target no step made, which a
    /// collection reads as empty, from one that failed. [`Reader::targets_of`]
    /// lists only versions built, since a failed one has no value to be of
    /// a type.
    pub fn defines<T>(&self, reference: &TargetReference<T>) -> bool {
        let name = reference.name();
        let Ok(target) = self.version(name) else {
            return false;
        };
        self.record(Dependency::Target {
            name: name.to_string(),
            step: target.step,
        });
        true
    }

    /// Every target it sees whose value is a `T`, in the order the steps
    /// made them — of one step, by name: the sketches a step could pick,
    /// say, the newest last.
    pub fn targets_of<T: 'static>(&self) -> Vec<TargetReference<T>> {
        (self.versions_of::<T>().into_iter())
            .filter(|(_, target)| target.failure("").is_none())
            .map(|(name, _)| TargetReference::new(name))
            .collect()
    }

    /// Why every target it sees that is defined to hold a `T` but failed
    /// did, in the order the steps made them: what a question over all of
    /// them — every constraint of a part — cannot be answered without.
    pub fn failed_targets_of<T: 'static>(&self) -> Vec<GeopError> {
        (self.versions_of::<T>().into_iter())
            .filter_map(|(name, target)| target.failure(name))
            .collect()
    }

    /// The latest version it sees of every target defined to hold a `T`,
    /// made or failed, in the order the steps made them, each read
    /// recorded.
    fn versions_of<T: 'static>(&self) -> Vec<(&'a str, &'a Target)> {
        let mut found: Vec<(usize, &'a str, &'a Target)> = (self.part.targets.names())
            .filter_map(|name| {
                let target = self.version(name).ok().filter(|target| target.is::<T>())?;
                self.record(Dependency::Target {
                    name: name.to_string(),
                    step: target.step,
                });
                Some((target.step, name, target))
            })
            .collect();
        found.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
        (found.into_iter()).map(|(_, name, target)| (name, target)).collect()
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
            part.begin_step();
            let reference = TargetReference::<Option<Sc>>::new("a");
            part.define_target(&reference, &(), |reader, _| Ok(reader.retrieve_input("x")))
                .unwrap();
        };

        part.begin_run(&x, &BTreeMap::new());
        reads_x(&mut part);
        part.end_run(0);

        part.begin_run(&BTreeMap::new(), &BTreeMap::new());
        reads_x(&mut part);
        part.end_run(0);
        assert!(part.inputs.contains_key("x"));

        part.begin_run(&BTreeMap::new(), &BTreeMap::new());
        part.end_run(0);
        assert!(!part.inputs.contains_key("x"));
    }
}
