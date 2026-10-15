use std::{
    cell::RefCell,
    collections::{BTreeMap, HashSet},
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    element::{ElementReference, HasElements, Kind},
    instance::{
        INSTANCE_SEPARATOR, InInstance, Kept, check_name, split_path, subpart_name,
        subpart_target,
    },
    program::Program,
    program_runner::{ProgramRunner, Same},
    workspace::{Loaded, Location},
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
    /// The run whose versions are its current ones: the last that ran its
    /// steps — earlier than `run` once a run carried them over.
    built: Revision,
    inputs: BTreeMap<String, RevisionedValue<I>>,
    state: BTreeMap<String, RevisionedValue<S>>,
    /// How many steps the current or last run began. A step is told by its
    /// position, in this run and the next.
    steps: usize,
    targets: TargetRegistry,
    /// The part of each of its instances, by the instance's name: run again
    /// in every run that loads it ([`ProgramRunner::load_subpart`]).
    ///
    /// TODO: Later we should share the part of every instance whose run
    /// read none of the state: a part of no state is the same wherever it
    /// is placed. For now every instance runs its part on its own.
    subparts: BTreeMap<String, Subpart<I, S>>,
    /// Whether the current run changed anything in it so far: made a
    /// version different from before, or changed the part of an instance.
    changing: bool,
    /// The last run that changed anything in it — made or deleted a
    /// version different from before, or changed the part of an instance.
    changed_at: Revision,
}

/// The part of an instance, as the part it is in keeps it.
struct Subpart<I, S> {
    part: Part<I, S>,
    /// The position of the step that loaded it last.
    step: usize,
    /// What it was run with last, if that run built every step: what tells
    /// whether a run would only reuse what it built.
    built: Option<Built>,
}

/// The program a part ran, where it ran it, and the revision of the files
/// it read.
struct Built {
    program: Rc<Program>,
    location: Location,
    revision: u64,
}

impl<I, S> Default for Part<I, S> {
    fn default() -> Self {
        Self {
            run: 0,
            built: 0,
            inputs: BTreeMap::new(),
            steps: 0,
            state: BTreeMap::new(),
            targets: TargetRegistry::default(),
            subparts: BTreeMap::new(),
            changing: false,
            changed_at: 0,
        }
    }
}

impl<I, S> Part<I, S> {
    /// The last run that changed anything in it: made or deleted a version
    /// different from before, or changed the part of an instance — 0, before
    /// any did.
    pub fn changed_at(&self) -> Revision {
        self.changed_at
    }
}

impl<I: Input, S: State> Part<I, S> {
    /// Starts a run with the given `inputs` and `state`, complete: what the
    /// last run gave and this one does not is removed. A [`crate::program_runner::ProgramRunner`]
    /// drives a part through a program; these drive it by hand, as a test of
    /// one operation does.
    pub fn begin_run(&mut self, inputs: &BTreeMap<String, I>, state: &BTreeMap<String, S>) {
        self.begin_run_as(self.next_run(), inputs, state);
    }

    /// The revision of the run after the last.
    pub(crate) fn next_run(&self) -> Revision {
        self.run + 1
    }

    /// Starts the run `revision`, later than the last (see
    /// [`Part::begin_run`]). Whether it is given any input or value of the
    /// state otherwise than the last run was.
    pub(crate) fn begin_run_as(
        &mut self,
        revision: Revision,
        inputs: &BTreeMap<String, I>,
        state: &BTreeMap<String, S>,
    ) -> bool {
        self.run = revision;
        self.built = revision;
        self.steps = 0;
        self.changing = false;
        let inputs = RevisionedValue::replace_all(&mut self.inputs, inputs, self.run);
        let state = RevisionedValue::replace_all(&mut self.state, state, self.run);
        inputs || state
    }


    /// Carries what the last run built over into the current one, as a run
    /// of the same program would reuse it: its versions stay its current
    /// ones, as the run `built` made them, its `steps` steps all run.
    pub(crate) fn carry(&mut self, (built, steps): (Revision, usize)) {
        self.built = built;
        self.steps = steps;
    }

    /// The run whose versions are its current ones, and how many steps the
    /// last run began: what a run carries over ([`Part::carry`]).
    pub(crate) fn last_built(&self) -> (Revision, usize) {
        (self.built, self.steps)
    }

    /// Starts the next step.
    pub fn begin_step(&mut self) {
        self.steps += 1;
    }

    /// Deletes the versions no step made, and the parts of instances no
    /// step loaded, of this run or of the `later` steps after the ones it
    /// ran, which it stopped before; and the removed inputs and state no
    /// version left depends on. A run that changed anything in it — made or
    /// deleted a version different from before, or changed or dropped the
    /// part of an instance — is when it last changed ([`Part::changed_at`]).
    pub fn end_run(&mut self, later: usize) {
        let (run, ran) = (self.run, self.steps);
        let kept = |step: usize, made: Revision| made == run || (ran..ran + later).contains(&step);
        let deleted = self.targets.retain(|target| kept(target.step, target.run));
        let placed = self.subparts.len();
        (self.subparts).retain(|_, subpart| kept(subpart.step, subpart.part.run));
        if self.changing || deleted || self.subparts.len() < placed {
            self.changed_at = run;
        }
        let read: HashSet<&Dependency> = self.targets.dependencies().collect();
        self.inputs.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::Input(name.clone()))
        });
        self.state.retain(|name, entry| {
            entry.value.is_some() || read.contains(&Dependency::State(name.clone()))
        });
    }

    /// The part at `path` — itself, for the empty path — as it holds it:
    /// none, if it holds no part there.
    fn at(&self, path: &str) -> Option<&Part<I, S>> {
        match path.is_empty() {
            true => Some(self),
            false => (path.split(INSTANCE_SEPARATOR))
                .try_fold(self, |part, name| Some(&part.subparts.get(name)?.part)),
        }
    }

    /// The part as it is before the step at position `before`, recording
    /// nothing: what that step is edited against, and what scrubbing the
    /// timeline shows.
    pub fn reader(&self, before: usize) -> Reader<'_, I, S> {
        Reader {
            part: self,
            before,
            recording: None,
            path: String::new(),
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

    /// Loads the part of the instance `name`, `subprogram` run by `runner`
    /// (see [`ProgramRunner::load_subpart`]).
    pub(crate) fn load_subpart(
        &mut self,
        runner: &ProgramRunner<I, S>,
        name: &str,
        loaded: &Loaded<S>,
        inputs: &BTreeMap<String, I>,
    ) -> GeopResult<()> {
        check_name(name)?;
        let ctx = |e: GeopError| e.with_context(format!("loading the part of `{name}`"));
        // Its target, first: one loaded twice in a run is defined twice.
        let reference = TargetReference::<Kept>::new(subpart_target(name));
        self.define_target(&reference, &(), |_, _| Ok(Kept)).map_err(ctx)?;
        let step = self.steps - 1;
        let prefix = format!("{name}{INSTANCE_SEPARATOR}");
        let mut state = (*loaded.state).clone();
        state.extend((under(&self.state, &prefix)).filter_map(|(state, entry)| {
            let state = state.strip_prefix(&prefix)?;
            Some((state.to_string(), entry.value.clone()?))
        }));
        let run = self.run;
        let subpart = (self.subparts.entry(name.to_string())).or_insert_with(|| Subpart {
            part: Part::default(),
            step,
            built: None,
        });
        subpart.step = step;
        let (program, revision) = (&loaded.program, runner.revision());
        let same = Same {
            program: subpart.built.as_ref().is_some_and(|built| {
                (Rc::ptr_eq(&built.program, program) || built.program == *program)
                    && built.location == runner.location
                    && built.revision == revision
            }),
        };
        let end = program.steps.len();
        let output = runner.run_as(&mut subpart.part, run, program, end, inputs, &state, same);
        subpart.built = (output.step_errors.is_empty()).then(|| Built {
            program: program.clone(),
            location: runner.location.clone(),
            revision,
        });
        self.changing |= subpart.part.changed_at == run;
        match output.step_errors.into_iter().next() {
            Some((step, e)) => Err(ctx(e.with_context(format!("step `{step}`")))),
            None => Ok(()),
        }
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
        // found undefined still is. What it read of a part placed in it is
        // read where that part is.
        let changed_at = |read: &str, given: fn(&Part<I, S>, &str) -> Revision| {
            let (path, read) = split_path(read);
            self.at(path)
                .map_or(Revision::MAX, |part| given(part, read))
        };
        let reusable = self.targets.get(name, step).is_some_and(|target| {
            let unchanged = |dependency: &Dependency| match dependency {
                Dependency::Target { name: read, step } => (self.reader(sees(read, name, before)))
                    .version(read)
                    .is_ok_and(|seen| seen.step == *step && seen.changed_at <= target.run),
                Dependency::Undefined(read) => (self.reader(sees(read, name, before)))
                    .version(read)
                    .is_err(),
                Dependency::Input(read) => {
                    changed_at(read, |part, read| {
                        RevisionedValue::changed_at(&part.inputs, read)
                    }) <= target.run
                }
                Dependency::State(read) => {
                    changed_at(read, |part, read| {
                        RevisionedValue::changed_at(&part.state, read)
                    }) <= target.run
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
            recording: Some(Rc::new(Recording {
                own: name.to_string(),
                dependencies: RefCell::new(Vec::new()),
            })),
            path: String::new(),
        };
        let made = generator(&reader, args).map_err(|e| {
            e.with_context(format!(
                "making target `{name}` in the step at position {step}"
            ))
        });
        let mut dependencies = (reader.recording)
            .map(|recording| recording.dependencies.take())
            .unwrap_or_default();
        dependencies.sort();
        dependencies.dedup();
        let (data, result) = match made {
            Ok(data) => (Ok(data), Ok(())),
            Err(e) => (Err(e.root_message().to_string()), Err(e)),
        };
        let previous = self.targets.get(name, step);
        let target = Target::new(step, args.clone(), dependencies, data, previous, run);
        self.changing |= target.changed_at == run;
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
///
/// The part of an instance placed in it is read by a reader of its own
/// ([`Reader::inside`]), which records what it reads, behind the
/// instance's path, as read by the target being made.
pub struct Reader<'a, I, S> {
    part: &'a Part<I, S>,
    /// The position of the step it reads before.
    before: usize,
    /// What the target being made has read, shared by the readers of the
    /// parts placed in its part.
    recording: Option<Rc<Recording>>,
    /// The path of the part it reads in the part of the target being made,
    /// with a separator after it: empty for that part itself.
    path: String,
}

// By hand, since a derive would ask the same of `I` and `S`.
impl<I, S> Clone for Reader<'_, I, S> {
    fn clone(&self) -> Self {
        Self {
            part: self.part,
            before: self.before,
            recording: self.recording.clone(),
            path: self.path.clone(),
        }
    }
}

impl<'a, I: Input, S: State> Reader<'a, I, S> {
    /// Records `dependency`, if it records: of the part it reads, named as
    /// the part of the target being made names it.
    fn record(&self, dependency: Dependency) {
        if let Some(recording) = &self.recording {
            recording
                .dependencies
                .borrow_mut()
                .push(dependency.behind(&self.path));
        }
    }

    /// The last run that changed anything in the part it reads (see
    /// [`Part::changed_at`]).
    pub fn changed_at(&self) -> Revision {
        self.part.changed_at
    }

    /// The version of `name` it sees: of the versions this run made, the
    /// one whose step came last before it — as of the step before for the
    /// target being made.
    fn version(&self, name: &str) -> GeopResult<&'a Target> {
        let (path, local) = split_path(name);
        if !path.is_empty() {
            return self.inside(path)?.version(local);
        }
        self.own_version(name).ok_or_else(|| {
            let reader = match self.seen_before(name).checked_sub(1) {
                Some(last) => format!("by the step at position {last} or before"),
                None => "before any step".to_string(),
            };
            GeopError::new(format!("target `{name}` is not defined {reader}"))
        })
    }

    /// The version of the part's own target `name` it sees (see
    /// [`Reader::version`]), if any.
    fn own_version(&self, name: &str) -> Option<&'a Target> {
        let (part, before) = (self.part, self.seen_before(name));
        (part.targets.versions(name))
            .filter(|target| target.run == part.built && target.step < before)
            .max_by_key(|target| target.step)
    }

    /// Before which step position it sees the part's own target `name`: its
    /// own position, but the step before for the target being made.
    fn seen_before(&self, name: &str) -> usize {
        match &self.recording {
            Some(recording) if self.path.is_empty() => sees(name, &recording.own, self.before),
            _ => self.before,
        }
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

    /// The element `reference` names, of the part itself or of the part of
    /// an instance — however deep — as it is in its own part, with the
    /// instance's path: where that is placed, the operation that loaded it
    /// says. The read is recorded behind the instance's path (see
    /// [`Reader::inside`]).
    pub fn retrieve_placed<T: HasElements<K>, K: Kind>(
        &self,
        reference: &ElementReference<T, K>,
    ) -> GeopResult<InInstance<T::Element<'a>>> {
        let (path, target) = split_path(reference.target().name());
        let resolve = |reader: &Reader<'a, I, S>| {
            let value = match (reader.own_version(target), T::undefined()) {
                (None, Some(undefined)) => {
                    reader.record(Dependency::Undefined(target.to_string()));
                    undefined
                }
                _ => reader.target(&TargetReference::<T>::new(target))?,
            };
            value.element(reference.name())
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

    /// The name of every instance whose part the part has loaded — not
    /// those of its instances — in the order the steps loaded them, each
    /// read recorded.
    pub fn subparts(&self) -> Vec<&'a str> {
        (self.versions_of::<Kept>().into_iter())
            .filter_map(|(target, _)| subpart_name(target))
            .collect()
    }

    /// The part of the instance at `path` — `bolt`, or `asm/bolt` — as its
    /// program left it in this run, the read of each instance on the way
    /// recorded, and what it reads recorded behind the path.
    pub fn inside(&self, path: &str) -> GeopResult<Reader<'a, I, S>> {
        let mut level = self.clone();
        for name in path.split(INSTANCE_SEPARATOR) {
            let kept = TargetReference::<Kept>::new(subpart_target(name));
            (level.target(&kept))
                .map_err(|e| e.with_context(format!("the part of `{name}`")))?;
            let subpart = (level.part.subparts.get(name))
                .ok_or_else(|| GeopError::new(format!("the part of `{name}` is not kept")))?;
            level = Reader {
                part: &subpart.part,
                before: subpart.part.steps,
                recording: level.recording.clone(),
                path: format!("{}{name}{INSTANCE_SEPARATOR}", level.path),
            };
        }
        Ok(level)
    }

    /// The input's value, if the run gives one. An input behind an
    /// instance's path — `bolt/d` — is read inside the instance's part.
    pub fn retrieve_input(&self, name: &str) -> Option<I> {
        let (path, local) = split_path(name);
        if !path.is_empty() {
            return self.inside(path).ok()?.retrieve_input(local);
        }
        self.record(Dependency::Input(name.to_string()));
        (self.part.inputs.get(name)).and_then(|entry| entry.value.clone())
    }

    /// The state's value; the default if the run gives none.
    pub fn retrieve_state(&self, name: &str) -> S {
        self.given_state(name).unwrap_or_default()
    }

    /// The state's value, if the run gives one. A state behind an
    /// instance's path — `bolt/b.pose` — is read inside the instance's part
    /// ([`Reader::inside`]), as it is run with it: the part's value, else
    /// the instance part's own — and none before a step loaded it.
    pub fn given_state(&self, name: &str) -> Option<S> {
        let (path, local) = split_path(name);
        if !path.is_empty() {
            return self.inside(path).ok()?.given_state(local);
        }
        self.record(Dependency::State(name.to_string()));
        (self.part.state.get(name)).and_then(|entry| entry.value.clone())
    }

    /// Whether it has read the state `name`: what a check of a value read
    /// from the state, a constraint's derivatives, asks.
    pub fn has_read_state(&self, name: &str) -> bool {
        (self.recording.as_ref()).is_some_and(|recording| {
            let read = Dependency::State(name.to_string()).behind(&self.path);
            recording.dependencies.borrow().contains(&read)
        })
    }

    /// Whether a step before it made a version of `reference`, built or
    /// failed, the read recorded: what tells a target no step made, which a
    /// collection reads as empty, from one that failed. [`Reader::targets_of`]
    /// lists only versions built, since a failed one has no value to be of
    /// a type. Its absence is recorded too: a step making it later makes
    /// what read it again.
    pub fn defines<T>(&self, reference: &TargetReference<T>) -> bool {
        let name = reference.name();
        let Ok(target) = self.version(name) else {
            self.record(Dependency::Undefined(name.to_string()));
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
                let target = self.own_version(name).filter(|target| target.is::<T>())?;
                self.record(Dependency::Target {
                    name: name.to_string(),
                    step: target.step,
                });
                Some((target.step, name, target))
            })
            .collect();
        found.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
        (found.into_iter())
            .map(|(_, name, target)| (name, target))
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
