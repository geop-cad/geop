//! [`Reader`]: the only way a part is read — by a make, which records every
//! read ([`Read`]) on the node it makes, or by an editor, recording
//! nothing.

use std::{any::TypeId, cell::RefCell, collections::HashSet, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    element::{ElementReference, HasElements, Kind},
    part::Part,
    placed::{InInstance, PLACED, PlacedPart, instance_path, split_path},
    program::Program,
    runner::Runner,
    target::{Node, Read, TargetReference},
    values::{Setting, StateValue},
    workspace::Location,
};

/// What the node being made has read, in order, each once.
#[derive(Default)]
pub(crate) struct Recording {
    reads: RefCell<Vec<Read>>,
    keys: RefCell<HashSet<String>>,
}

impl Recording {
    fn record(&self, read: Read) {
        if self.keys.borrow_mut().insert(read.key()) {
            self.reads.borrow_mut().push(read);
        }
    }

    pub(crate) fn take(self) -> Vec<Read> {
        self.reads.into_inner()
    }
}

/// The part as it is before one step, and the only way it is read: by a
/// make, as of its own step, recording every read so that its node is made
/// again when any of it changes — or by an editor, recording nothing: the
/// part now ([`Part::current`]), or before or after one step of the last run
/// ([`Part::before`], [`Part::after`]), what that step is edited against and
/// what scrubbing the timeline shows.
///
/// A target is read by its elements ([`ElementReference`]), never as a
/// whole: a target meant to be read whole has a kind of element that is all
/// of it (see [`HasElements`]).
///
/// A part placed in it is read by a reader of its own
/// ([`Reader::placed_part`]), which records what it reads as read inside
/// the placed part ([`Read::Inside`]).
#[derive(Clone)]
pub struct Reader<'a> {
    part: &'a Part,
    /// The position of the step it reads before.
    before: usize,
    /// The target being made, if it reads for a make: it reads its own
    /// target as the steps before left it.
    own: Option<&'a str>,
    /// What the step being run made of the target being made last: what a
    /// placed part is built again from.
    last: Option<&'a Node>,
    runner: Option<&'a Runner>,
    recording: Option<&'a Recording>,
    /// The path of the part it reads in the part of the node being made:
    /// empty for that part itself.
    path: Vec<String>,
}

impl<'a> Reader<'a> {
    /// The part `part` before the step at position `before`, recording
    /// nothing.
    pub(crate) fn new(part: &'a Part, before: usize) -> Self {
        Self {
            part,
            before,
            own: None,
            last: None,
            runner: None,
            recording: None,
            path: Vec::new(),
        }
    }

    /// The reader of the make of `own`, by the step at position `before`,
    /// recording into `recording`. `last` is what the step made of it last.
    pub(crate) fn making(
        part: &'a Part,
        before: usize,
        own: &'a str,
        last: Option<&'a Node>,
        runner: &'a Runner,
        recording: &'a Recording,
    ) -> Self {
        Self {
            part,
            before,
            own: Some(own),
            last,
            runner: Some(runner),
            recording: Some(recording),
            path: Vec::new(),
        }
    }

    /// Records `read`, if it records: as read inside the part it reads.
    fn record(&self, read: Read) {
        if let Some(recording) = self.recording {
            let read = (self.path.iter().rev()).fold(read, |read, instance| Read::Inside {
                instance: instance.clone(),
                read: Box::new(read),
            });
            recording.record(read);
        }
    }

    /// The part it reads, and the target being made in it, if any.
    pub(crate) fn part(&self) -> &'a Part {
        self.part
    }

    /// The position of the step it reads before.
    pub(crate) fn position(&self) -> usize {
        self.before
    }

    /// The target being made, if it reads for a make of its own part.
    pub(crate) fn own(&self) -> Option<&'a str> {
        self.own
    }

    /// What the step being run made of the target being made last.
    pub(crate) fn last(&self) -> Option<&'a Node> {
        self.last
    }

    /// The runner of the step being run: an error for a reader of an
    /// editor, which runs nothing.
    pub(crate) fn runner(&self) -> GeopResult<&'a Runner> {
        self.runner
            .ok_or_else(|| GeopError::new("only a make reads files and places parts"))
    }

    /// Records the read of `read` as such, unchanged: what a placed part
    /// passes up of what it read.
    pub(crate) fn record_raw(&self, read: Read) {
        self.record(read);
    }

    /// Changes exactly when a run changes any node of the part it reads
    /// (see [`Part::revision`]).
    pub fn revision(&self) -> u64 {
        self.part.revision()
    }

    /// The node of `name` it sees, the read recorded, or why there is none.
    fn node(&self, name: &str) -> GeopResult<&'a Node> {
        match self.part.visible(name, self.before, self.own) {
            Some((_, node)) => {
                self.record(Read::Target {
                    name: name.to_string(),
                    producer: node.step.clone(),
                    version: node.version,
                });
                Ok(node)
            }
            None => {
                self.record(Read::Absent(name.to_string()));
                Err(GeopError::new(format!(
                    "no step before here produces target `{name}`"
                )))
            }
        }
    }

    /// The target `reference`, whole, the read recorded.
    pub(crate) fn target<T: 'static>(&self, reference: &TargetReference<T>) -> GeopResult<&'a T> {
        let name = reference.name();
        self.node(name)?.get(name)
    }

    /// A copy of the target being made as the steps before left it: what a
    /// chain target ([`crate::target::Target::CHAIN`]) is produced from —
    /// the default, if no step produced it before. The read is recorded.
    pub fn previous<T: Clone + Default + 'static>(&self) -> GeopResult<T> {
        let own = self
            .own
            .filter(|_| self.path.is_empty())
            .ok_or_else(|| GeopError::new("only a target being made has a previous version"))?;
        if self.part.visible(own, self.before, self.own).is_none() {
            self.record(Read::Absent(own.to_string()));
            return Ok(T::default());
        }
        self.target(&TargetReference::<T>::new(own)).cloned()
    }

    /// The element `reference` names, of the part itself. One of a part
    /// placed in it is an error naming the instance: it is where that
    /// instance is, which [`Reader::retrieve_placed`] says.
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

    /// The element `reference` names, of the part itself or of a part
    /// placed in it — however deep — as it is in its own part, with the
    /// instance's path: where that is placed, the step that placed it
    /// says. The read is recorded inside the placed part.
    pub fn retrieve_placed<T: HasElements<K>, K: Kind>(
        &self,
        reference: &ElementReference<T, K>,
    ) -> GeopResult<InInstance<T::Element<'a>>> {
        let (path, target) = split_path(reference.target().name());
        let resolve = |reader: &Reader<'a>| {
            let value = match (
                reader.part.visible(target, reader.before, reader.own),
                T::undefined(),
            ) {
                (None, Some(undefined)) => {
                    reader.record(Read::Absent(target.to_string()));
                    undefined
                }
                _ => reader.target(&TargetReference::<T>::new(target))?,
            };
            value.element(reference.name())
        };
        let element = match path.is_empty() {
            true => resolve(self),
            false => self.placed_part(path).and_then(|inside| resolve(&inside)),
        };
        let element = element.map_err(|e| e.with_context(format!("resolving {reference:?}")))?;
        Ok(InInstance {
            element,
            instance: path.to_string(),
        })
    }

    /// The part placed at `path` — `bolt`, or `asm/bolt` — as it was built,
    /// every read of it recorded as read inside it.
    pub fn placed_part(&self, path: &str) -> GeopResult<Reader<'a>> {
        let mut level = self.clone();
        for name in path.split(crate::placed::INSTANCE_SEPARATOR) {
            let inner = (level.part.placed_at(name, level.before, level.own))
                .map_err(|e| e.with_context(format!("the placed part {path:?}")))?;
            let mut inside = Reader::new(inner, inner.ran());
            inside.runner = level.runner;
            inside.recording = level.recording;
            inside.path = level.path.clone();
            inside.path.push(name.to_string());
            level = inside;
        }
        Ok(level)
    }

    /// The name of every part placed in the part it reads — not in those —
    /// in the order the steps placed them. The listing is recorded.
    pub fn instances(&self) -> Vec<&'a str> {
        (self.listed::<PlacedPart>().into_iter())
            .filter(|(_, node)| node.value.is_ok())
            .filter_map(|(name, _)| name.strip_prefix(PLACED))
            .collect()
    }

    /// The input's value, if the run gives one, the read recorded. An input
    /// behind an instance's path — `bolt/d` — is read inside the placed
    /// part.
    pub fn retrieve_input(&self, name: &str) -> Option<Setting> {
        let (path, local) = split_path(name);
        if !path.is_empty() {
            return self.placed_part(path).ok()?.retrieve_input(local);
        }
        let value = self.part.inputs.get(name).cloned();
        self.record(Read::Input(name.to_string(), value.clone()));
        value
    }

    /// The state's value; the default if the run gives none.
    pub fn retrieve_state(&self, name: &str) -> StateValue {
        self.given_state(name).unwrap_or_default()
    }

    /// The state's value, if the run gives one, the read recorded. A state
    /// behind an instance's path — `bolt/b.pose` — is the part's own value
    /// of that name where it gives one: the value the placed part is built
    /// with. Else it is read inside the placed part — the placed program's
    /// own, or none before a step placed it.
    pub fn given_state(&self, name: &str) -> Option<StateValue> {
        let value = self.part.state.get(name).cloned();
        self.record(Read::State(name.to_string(), value.clone()));
        match (value, name.split_once(crate::placed::INSTANCE_SEPARATOR)) {
            (Some(value), _) => Some(value),
            (None, None) => None,
            (None, Some((instance, rest))) => self.placed_part(instance).ok()?.given_state(rest),
        }
    }

    /// Whether it has read the state `name`: what a check of a value read
    /// from the state, a constraint's derivatives, asks.
    pub fn has_read_state(&self, name: &str) -> bool {
        (self.recording).is_some_and(|recording| {
            let read = Read::State(name.to_string(), None);
            let key = (self.path.iter().rev())
                .fold(read.key(), |key, instance| format!("{instance}/{key}"));
            recording.has(&key)
        })
    }

    /// Every target the make it reads for read so far that itself read a
    /// state, directly or through what it read, with that state: what
    /// moves with the state without the make's derivatives seeing it.
    pub fn targets_reading_state(&self) -> Vec<(String, String)> {
        let Some(recording) = self.recording else {
            return Vec::new();
        };
        let reads = recording.reads.borrow();
        (reads.iter())
            .filter(|read| !matches!(innermost(read), Read::State(..)))
            .filter_map(|read| {
                let state = self.part.state_read(read, self.before, self.own)?;
                Some((target_of(read), state))
            })
            .collect()
    }

    /// Whether a step before it produced `reference`, made or failed, the
    /// read recorded: what tells a target no step produced, which a
    /// collection reads as empty, from one that failed.
    pub fn defines<T>(&self, reference: &TargetReference<T>) -> bool {
        self.node(reference.name()).is_ok()
    }

    /// Every target it sees whose value is a `T`, in the order the steps
    /// produced them — of one step, by name: the sketches a step could
    /// pick, say, the newest last. Which there are is recorded.
    pub fn targets_of<T: 'static>(&self) -> Vec<TargetReference<T>> {
        (self.listed::<T>().into_iter())
            .filter(|(_, node)| node.value.is_ok())
            .map(|(name, _)| TargetReference::new(name))
            .collect()
    }

    /// Why every target it sees that is produced to hold a `T` but failed
    /// did, in the order the steps produced them: what a question over all
    /// of them — every constraint of a part — cannot be answered without.
    pub fn failed_targets_of<T: 'static>(&self) -> Vec<GeopError> {
        (self.listed::<T>().into_iter())
            .filter_map(|(name, node)| {
                self.record(Read::Target {
                    name: name.to_string(),
                    producer: node.step.clone(),
                    version: node.version,
                });
                node.failure(name)
            })
            .collect()
    }

    /// Every target produced to hold a `T` it sees, the listing recorded.
    fn listed<T: 'static>(&self) -> Vec<(&'a str, &'a Node)> {
        let listed = self.part.listed(TypeId::of::<T>(), self.before, self.own);
        self.record(Read::Listed {
            kind: TypeId::of::<T>(),
            targets: (listed.iter())
                .map(|(name, node)| (name.to_string(), node.step.clone()))
                .collect(),
        });
        listed
    }

    /// The text of the file the program of the step being run names
    /// `name`, and its path. The read is recorded.
    pub fn read_file(&self, name: &str) -> GeopResult<(String, Rc<str>)> {
        let runner = self.runner()?;
        let (path, text) = runner.read(name)?;
        self.record(Read::File(path.clone(), runner.revision(&path)));
        Ok((path, text))
    }

    /// The program in the file the program of the step being run names
    /// `name`, and where it would be placed from there (see
    /// [`Runner::program`]). The read is recorded.
    pub fn program(&self, name: &str) -> GeopResult<(Rc<Program>, Location)> {
        let runner = self.runner()?;
        let (program, location) = runner.program(name)?;
        self.record(Read::File(
            location.file.clone(),
            runner.revision(&location.file),
        ));
        Ok((program, location))
    }
}

/// What `read` read, inside the placed parts it went into.
fn innermost(read: &Read) -> &Read {
    match read {
        Read::Inside { read, .. } => innermost(read),
        read => read,
    }
}

/// The name of the target `read` read, behind the path of the placed part
/// it is in.
fn target_of(read: &Read) -> String {
    match read {
        Read::Target { name, .. } | Read::Absent(name) | Read::Input(name, _) => name.clone(),
        Read::State(name, _) | Read::File(name, _) => name.clone(),
        Read::Listed { .. } => String::new(),
        Read::Inside { instance, read } => instance_path(instance, &target_of(read)),
    }
}

impl Recording {
    fn has(&self, key: &str) -> bool {
        self.keys.borrow().contains(key)
    }
}
