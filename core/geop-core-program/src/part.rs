use std::{
    cell::RefCell,
    collections::{BTreeMap, HashSet},
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    element::{ElementReference, HasElements, Kind},
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
        self.inputs.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::Input(name.clone()))
        });
        self.state.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::State(name.clone()))
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

    /// The element `reference` names.
    pub fn retrieve_element<T: HasElements<K>, K: Kind>(
        &self,
        reference: &ElementReference<T, K>,
    ) -> GeopResult<T::Element<'a>> {
        (self.target(reference.target())?.element(reference.name()))
            .map_err(|e| e.with_context(format!("resolving {reference:?}")))
    }

    /// The input's value, if the run gives one.
    pub fn retrieve_input(&self, name: &str) -> Option<I> {
        self.record(Dependency::Input(name.to_string()));
        (self.part.inputs.get(name)).and_then(|entry| entry.value.clone())
    }

    /// The state's value; the default if the run gives none.
    pub fn retrieve_state(&self, name: &str) -> S {
        self.record(Dependency::State(name.to_string()));
        (self.part.state.get(name))
            .and_then(|entry| entry.value.clone())
            .unwrap_or_default()
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
        let mut found: Vec<(usize, &'a str)> = (self.part.targets.names())
            .filter_map(|name| {
                let target = self.version(name).ok()?;
                target.value::<T>(name).ok()?;
                self.record(Dependency::Target {
                    name: name.to_string(),
                    step: target.step,
                });
                Some((target.step, name))
            })
            .collect();
        found.sort();
        (found.into_iter())
            .map(|(_, name)| TargetReference::new(name))
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
