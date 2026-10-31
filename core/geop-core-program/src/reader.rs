//! [`Reader`]: the only way a part is read — by a make, which records every
//! read ([`Read`]) on the node it makes, or by an editor, recording
//! nothing.

use std::{any::TypeId, cell::RefCell, collections::HashSet, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    element::{ElementReference, HasElements, Kind},
    part::Part,
    placed::{
        INSTANCE_SEPARATOR, InInstance, Located, OUTSIDE, PLACED, PlacedPart, key_behind,
        outermost_instance,
    },
    program::Program,
    runner::{Runner, quoted},
    target::{Node, Read, TargetReference},
    values::{Setting, Settings, StateValue, StateValues},
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
/// the placed part ([`Read::Inside`]). So is the part it is itself placed
/// in, while it is built there — the path [`OUTSIDE`], `..`: a step of a
/// part placed in an assembly reads a part placed beside it there as
/// `../bracket`, in context.
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
    /// The part the part it reads is placed in, as the step placing it
    /// reads it, while it is built there: what `..` reads. None for a part
    /// built on its own, and for a part placed in the part read.
    outside: Option<Rc<Reader<'a>>>,
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
            outside: None,
        }
    }

    /// The reader of the make of `own`, by the step at position `before`,
    /// recording into `recording`. `last` is what the step made of it last;
    /// `outside`, the part the part is placed in, if it is built placed.
    pub(crate) fn making(
        part: &'a Part,
        before: usize,
        own: &'a str,
        last: Option<&'a Node>,
        runner: &'a Runner,
        recording: &'a Recording,
        outside: Option<Rc<Reader<'a>>>,
    ) -> Self {
        Self {
            part,
            before,
            own: Some(own),
            last,
            runner: Some(runner),
            recording: Some(recording),
            path: Vec::new(),
            outside,
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

    /// What tells the part it reads from another ([`Seen`]): equal for two
    /// readers exactly when they see the same values — of one part or of
    /// two — so a view drawn of one is a view of the other: what an editor
    /// sends a viewer again only when it changed.
    pub fn seen(&self) -> Seen {
        Seen {
            versions: self.part.versions(self.before, self.own),
            inputs: self.part.inputs.clone(),
            state: self.part.state.clone(),
        }
    }

    /// What `read` makes of the part it reads, and what of the part it read
    /// to make it: equal reads ([`Reader::reads_same`]) make it alike, so
    /// what an editor draws of a part is kept by what each drawer read,
    /// and drawn again only when that changed.
    pub fn recorded<T>(&self, read: impl FnOnce(&Reader) -> T) -> (T, Reads) {
        let recording = Recording::default();
        let reader = Reader {
            recording: Some(&recording),
            path: Vec::new(),
            ..self.clone()
        };
        let made = read(&reader);
        (made, Reads(recording.take()))
    }

    /// Whether everything `reads` read ([`Reader::recorded`]) reads the
    /// same through it.
    pub fn reads_same(&self, reads: &Reads) -> bool {
        (reads.0.iter()).all(|read| {
            (self.part).holds(
                read,
                self.before,
                self.own,
                self.runner,
                self.outside_reader(),
            )
        })
    }

    /// The target being made, if it reads for a make of its own part.
    pub(crate) fn own(&self) -> Option<&'a str> {
        self.own
    }

    /// The same, as of the make of `own`, recording nothing: what reads a
    /// placed part's step, `placed:<instance>`, as its make did.
    pub(crate) fn for_make_of(mut self, own: &'a str) -> Self {
        self.own = Some(own);
        self
    }

    /// What the step being run made of the target being made last.
    pub(crate) fn last(&self) -> Option<&'a Node> {
        self.last
    }

    /// The part the part it reads is placed in, while it is built there.
    pub(crate) fn outside_reader(&self) -> Option<&Reader<'a>> {
        self.outside.as_deref()
    }

    /// The same, of a part placed in the part `outside` reads — as the
    /// step placing it reads that part ([`Part::placing`]): what it reads
    /// in context, through `..`, is read there. What an editor edits a step
    /// of a placed part against, where it is placed.
    pub fn placed_in(mut self, outside: Reader<'a>) -> Self {
        self.outside = Some(Rc::new(outside));
        self
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
                let seen: Vec<String> = (self.part.versions(self.before, self.own).into_iter())
                    .map(|(seen, _)| format!("`{seen}`"))
                    .collect();
                let seen = match seen.is_empty() {
                    true => "none".to_string(),
                    false => seen.join(", "),
                };
                let suppressed = self.part.suppressed_before(self.before);
                let suppressed = match suppressed.is_empty() {
                    true => String::new(),
                    false => format!("; suppressed before here: {}", quoted(&suppressed)),
                };
                Err(GeopError::new(format!(
                    "no step before here produces target `{name}` (the targets before here: \
                     {seen}{suppressed})"
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
        let (path, target) = (reference.instance(), reference.target().name());
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
    /// every read of it recorded as read inside it. `..` ([`OUTSIDE`]) is
    /// the part it is placed in — `../bracket` a part placed beside it —
    /// as the step placing it sees that part: only parts placed before it,
    /// so that no two parts read each other.
    pub fn placed_part(&self, path: &str) -> GeopResult<Reader<'a>> {
        let ctx = |e: GeopError| e.with_context(format!("the placed part {path:?}"));
        let mut level = self.clone();
        for name in path.split(INSTANCE_SEPARATOR) {
            if name == OUTSIDE {
                level = level.outside().map_err(ctx)?.0;
                continue;
            }
            let inner = level.part.placed_at(name, level.before, level.own);
            let inner = match (inner, level.path.last()) {
                (Ok(inner), _) => inner,
                (Err(e), Some(last)) if last == OUTSIDE => {
                    let me = (level.own.and_then(|own| own.strip_prefix(PLACED))).unwrap_or("");
                    return Err(ctx(e.with_context(format!(
                        "`{me}` reads `{name}` placed beside it in context: a part reads only \
                         the parts placed before it, so that no two parts read each other, \
                         and `{name}` is placed after `{me}`, or not at all"
                    ))));
                }
                (Err(e), _) => return Err(ctx(e)),
            };
            let mut inside = Reader::new(inner, inner.ran());
            inside.runner = level.runner;
            inside.recording = level.recording;
            inside.path = level.path.clone();
            inside.path.push(name.to_string());
            level = inside;
        }
        Ok(level)
    }

    /// The part the part it reads is placed in, as the step placing it
    /// reads it — every read of it recorded as read there, through `..` —
    /// and the instance that step places it as. An error for a part built
    /// on its own, which no part places.
    pub fn outside(&self) -> GeopResult<(Reader<'a>, &'a str)> {
        let outside = self.outside.as_deref().ok_or_else(|| {
            GeopError::new(format!(
                "`{OUTSIDE}` is the part this one is placed in, read in context: this one is \
                 built on its own, placed in none — open the part that places it"
            ))
        })?;
        let me = (outside.own.and_then(|own| own.strip_prefix(PLACED)))
            .ok_or_else(|| GeopError::new("the part this one is placed in places no part here"))?;
        let mut level = outside.clone();
        level.recording = self.recording;
        level.runner = self.runner.or(outside.runner);
        level.path = self.path.clone();
        level.path.push(OUTSIDE.to_string());
        Ok((level, me))
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
    /// behind an instance's path — `/bolt:d` — is read inside the placed
    /// part.
    pub fn retrieve_input(&self, name: &str) -> Option<Setting> {
        if let Some((instance, inside)) = outermost_instance(name) {
            return self.placed_part(instance).ok()?.retrieve_input(inside);
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
    /// behind an instance's path — `/bolt:b.pose` — is the part's own value
    /// of that name where it gives one: the value the placed part is built
    /// with. Else it is read inside the placed part — the placed program's
    /// own, or none before a step placed it.
    pub fn given_state(&self, name: &str) -> Option<StateValue> {
        let value = self.part.state.get(name).cloned();
        self.record(Read::State(name.to_string(), value.clone()));
        match (value, outermost_instance(name)) {
            (Some(value), _) => Some(value),
            (None, None) => None,
            (None, Some((instance, inside))) => {
                self.placed_part(instance).ok()?.given_state(inside)
            }
        }
    }

    /// Whether it has read the state `name`: what a check of a value read
    /// from the state, a constraint's derivatives, asks.
    pub fn has_read_state(&self, name: &str) -> bool {
        (self.recording).is_some_and(|recording| {
            let read = Read::State(name.to_string(), None);
            let key = (self.path.iter().rev())
                .fold(read.key(), |key, instance| key_behind(instance, &key));
            recording.has(&key)
        })
    }

    /// Every target the make it reads for read so far that itself read a
    /// state, directly or through what it read, with that state: what
    /// moves with the state without the make's derivatives seeing it.
    pub fn targets_reading_state(&self) -> Vec<(Located, String)> {
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
            kind: std::any::type_name::<T>().into(),
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

    /// The bytes of the file the program of the step being run names
    /// `name`, and its path. The read is recorded.
    pub fn read_bytes(&self, name: &str) -> GeopResult<(String, Rc<[u8]>)> {
        let runner = self.runner()?;
        let (path, bytes) = runner.read_bytes(name)?;
        self.record(Read::File(path.clone(), runner.revision(&path)));
        Ok((path, bytes))
    }

    /// What the file the program of the step being run names `name` reads
    /// as, as `kind` — a STEP file's bodies — made of its text by `make`,
    /// and its path: made once for every build its workspace's cache keeps
    /// it for (see [`crate::cache`]). The read is recorded.
    pub fn read_file_as<T: Serialize + DeserializeOwned + 'static>(
        &self,
        name: &str,
        kind: &'static str,
        make: impl FnOnce(&str) -> GeopResult<T>,
    ) -> GeopResult<(String, Rc<T>)> {
        let (path, text) = self.read_file(name)?;
        let cache = (self.runner()?.workspace.as_deref()).and_then(|workspace| workspace.cache());
        let value = match cache {
            Some(cache) => cache.made_of(kind, &path, &text, make)?,
            None => Rc::new(make(&text)?),
        };
        Ok((path, value))
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

/// What a reader read of its part ([`Reader::recorded`]), each thing once.
#[derive(Clone, Default)]
pub struct Reads(pub(crate) Vec<Read>);

/// Everything a [`Reader`] reads from ([`Reader::seen`]): the version of
/// every target it sees, by name, and the inputs and the state its part
/// was run with — which a placed part's pose is read from, not built into
/// its node, so that moving a part builds nothing.
#[derive(Clone, Debug, PartialEq)]
pub struct Seen {
    pub versions: Vec<(String, u64)>,
    pub inputs: Settings,
    pub state: StateValues,
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
fn target_of(read: &Read) -> Located {
    match read {
        Read::Target { name, .. } | Read::Absent(name) | Read::Input(name, _) => Located::new(name),
        Read::State(name, _) | Read::File(name, _) => Located::new(name),
        Read::Listed { .. } => Located::default(),
        Read::Inside { instance, read } => target_of(read).behind(instance),
    }
}

impl Recording {
    fn has(&self, key: &str) -> bool {
        self.keys.borrow().contains(key)
    }
}
