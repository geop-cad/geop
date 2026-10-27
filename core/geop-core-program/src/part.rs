//! [`Part`]: what a program builds, as a memo table — one node per step and
//! target it produces — and [`StepContext`], what a step declares its
//! nodes through ([`StepContext::produce`]).
//!
//! A make runs when its node is new, its arguments differ, or something it
//! read changed; nothing else ever runs. Each node keeps what it read
//! ([`Read`]); it is reused while its arguments are equal and every read
//! holds now. A node made again into a value equal to the last keeps its
//! version, so what read it is reused in turn. A node of a target that is
//! kept between processes is taken from what an earlier process kept, if
//! it was made so there, before it is made (see [`crate::kept`]).

use std::{
    any::TypeId,
    collections::{BTreeMap, BTreeSet, HashMap},
    fmt::Debug,
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    kept::{Derivation, Digest, Hasher, Keeper, KeptRead, digest_debug},
    placed::{PLACED, PlacedPart, placed},
    reader::{Reader, Recording},
    runner::Runner,
    target::{Node, Provenance, Read, Target, TargetReference, next_version},
    values::{Settings, StateValues},
};

/// What makes a node's value from its arguments and what it reads. A plain
/// function, not a closure: what it makes may depend only on its
/// arguments, which a reuse compares, and on what it reads, which is
/// recorded. A value captured from outside would be neither, and the node
/// would be reused after it changed:
///
/// ```compile_fail
/// # use geop_core_program::{part::StepContext, target::TargetReference};
/// fn produce(ctx: &mut StepContext, outside: u32) {
///     let reference = TargetReference::<u32>::new("t");
///     ctx.produce(&reference, &(), |_, _| Ok(outside));
/// }
/// ```
///
/// For the same reason arguments and values compare by value: arguments
/// holding an `Rc` or a closure, compared by address, would make a node
/// again for arguments alike, and a value compared so would count as
/// changed when it is not, and could never be checked against a part
/// built from scratch (`testing::assert_incremental`).
pub type Make<Args, Out> = fn(&Reader, &Args) -> GeopResult<Out>;

/// What a program builds: a node per step and target it produced, the
/// order of the steps it last ran, and the inputs and state it ran with.
/// Read only through a [`Reader`]: [`Part::current`], [`Part::before`],
/// [`Part::after`].
#[derive(Clone, Default)]
pub struct Part {
    /// The nodes of each target, one per step that produces it.
    nodes: HashMap<String, Vec<Node>>,
    /// The targets each step produced: what a step's end and a run's
    /// sweep look at.
    produced: HashMap<String, BTreeSet<String>>,
    /// The steps of the program it last ran, in order — those after where
    /// it stopped too.
    order: Vec<String>,
    positions: HashMap<String, usize>,
    /// How many steps of `order` ran: the rest, after the marker, keep
    /// their nodes for when they run again.
    ran: usize,
    /// The position of the step running, during a run.
    running: Option<usize>,
    /// Counts its runs: what tells a node the step running produced from
    /// one of an earlier run.
    run: u64,
    pub(crate) inputs: Settings,
    pub(crate) state: StateValues,
    /// The steps of the last run that failed, with why.
    failures: Vec<(String, String)>,
    /// Changes whenever a run changes any node (see [`Part::revision`]).
    revision: u64,
    changed: bool,
    /// The state it read, by name: what a part placing it reads of its own
    /// state for it.
    pub(crate) states_read: BTreeSet<String>,
    /// The files it read, and in which revision.
    pub(crate) files_read: BTreeMap<String, u64>,
}

impl Part {
    /// An empty part, to be run with `inputs` and `state`: what a test
    /// running steps by hand ([`Runner::run_step`]) starts from. A run of a
    /// program gives its own.
    pub fn given(inputs: Settings, state: StateValues) -> Self {
        Self {
            inputs,
            state,
            ..Self::default()
        }
    }

    /// The steps of the program it last ran, in order.
    pub fn order(&self) -> &[String] {
        &self.order
    }

    /// The part as it is now: during a run, with what the steps so far
    /// produced; after one, the finished part.
    pub fn current(&self) -> Reader<'_> {
        Reader::new(self, self.ran)
    }

    /// The part as it is before the step `step` of the last run: what that
    /// step is edited against. The finished part for a step that did not
    /// run.
    pub fn before(&self, step: &str) -> Reader<'_> {
        Reader::new(self, self.position(step).unwrap_or(self.ran))
    }

    /// The part as it is after the step `step` of the last run, with what it
    /// produced. The finished part for a step that did not run.
    pub fn after(&self, step: &str) -> Reader<'_> {
        Reader::new(self, self.position(step).map_or(self.ran, |at| at + 1))
    }

    /// Changes exactly when a run changes any of its nodes: what tells a
    /// view of it to draw again. Equal for two parts only where one is a
    /// copy of the other since.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// The steps of its last run that failed, in order, with why.
    pub fn failures(&self) -> &[(String, String)] {
        &self.failures
    }

    /// How many steps of the last run ran.
    pub(crate) fn ran(&self) -> usize {
        self.ran
    }

    /// The position of the step `step` in the last run.
    fn position(&self, step: &str) -> Option<usize> {
        self.positions.get(step).copied()
    }

    /// The node `step` produced of `name`.
    pub(crate) fn node(&self, name: &str, step: &str) -> Option<&Node> {
        (self.nodes.get(name)?.iter()).find(|node| node.step == step)
    }

    /// The node of `name` a reader before position `before` sees: of the
    /// steps before it, the one last in the program — and, for the node
    /// being made, `own`, the nodes its own step produced already in this
    /// run, of the other targets. With its position.
    pub(crate) fn visible(
        &self,
        name: &str,
        before: usize,
        own: Option<&str>,
    ) -> Option<(usize, &Node)> {
        (self.nodes.get(name)?.iter())
            .filter_map(|node| {
                let at = self.position(&node.step)?;
                let seen = at < before.min(self.ran)
                    || (Some(at) == self.running
                        && at == before
                        && node.produced_in == self.run
                        && own != Some(name));
                seen.then_some((at, node))
            })
            .max_by_key(|(at, _)| *at)
    }

    /// The version of every target a reader before `before` sees, by
    /// name (see [`Reader::seen`]).
    pub(crate) fn versions(&self, before: usize, own: Option<&str>) -> Vec<(String, u64)> {
        let mut versions: Vec<(String, u64)> = (self.nodes.keys())
            .filter_map(|name| {
                let (_, node) = self.visible(name, before, own)?;
                Some((name.clone(), node.version))
            })
            .collect();
        versions.sort();
        versions
    }

    /// Every target a reader before `before` sees a node of a `T` of,
    /// produced or failed, in the order the steps produced them — of one
    /// step, by name.
    pub(crate) fn listed(
        &self,
        kind: TypeId,
        before: usize,
        own: Option<&str>,
    ) -> Vec<(&str, &Node)> {
        self.listed_where(|node| node.kind == kind, before, own)
    }

    /// [`Part::listed`], of the targets whose node `of_kind` takes.
    fn listed_where(
        &self,
        of_kind: impl Fn(&Node) -> bool,
        before: usize,
        own: Option<&str>,
    ) -> Vec<(&str, &Node)> {
        let mut found: Vec<(usize, &str, &Node)> = (self.nodes.keys())
            .filter_map(|name| {
                let (at, node) = self.visible(name, before, own)?;
                of_kind(node).then_some((at, name.as_str(), node))
            })
            .collect();
        found.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
        (found.into_iter())
            .map(|(_, name, node)| (name, node))
            .collect()
    }

    /// The parts placed in the finished part — not in those — in the order
    /// the steps placed them: each by its instance name, with the step that
    /// placed it and its build.
    pub fn placed(&self) -> Vec<(&str, &str, &PlacedPart)> {
        (self
            .listed(TypeId::of::<PlacedPart>(), self.ran, None)
            .into_iter())
        .filter_map(|(name, node)| {
            let instance = name.strip_prefix(PLACED)?;
            let placed = node.get::<PlacedPart>(name).ok()?;
            Some((instance, node.step.as_str(), placed))
        })
        .collect()
    }

    /// The part placed as `instance` that a reader before `before` sees.
    pub(crate) fn placed_at(
        &self,
        instance: &str,
        before: usize,
        own: Option<&str>,
    ) -> GeopResult<&Rc<Part>> {
        let reference = placed(instance);
        let name = reference.name();
        let (_, node) = self.visible(name, before, own).ok_or_else(|| {
            GeopError::new(format!("no part is placed as `{instance}` before here"))
        })?;
        Ok(&node.get::<PlacedPart>(name)?.part)
    }

    /// Whether `read`, made by a reader before `before` for the node `own`,
    /// reads the same now. A file is read through a runner: without one, it
    /// is not known to read the same.
    pub(crate) fn holds(
        &self,
        read: &Read,
        before: usize,
        own: Option<&str>,
        runner: Option<&Runner>,
    ) -> bool {
        match read {
            Read::Target {
                name,
                producer,
                version,
            } => (self.visible(name, before, own))
                .is_some_and(|(_, node)| node.step == *producer && node.version == *version),
            Read::Absent(name) => self.visible(name, before, own).is_none(),
            Read::Input(name, value) => self.inputs.get(name) == value.as_ref(),
            Read::State(name, value) => self.state.get(name) == value.as_ref(),
            Read::File(path, revision) => runner.is_some_and(|r| r.revision(path) == *revision),
            Read::Listed { kind, targets, .. } => {
                let now = self.listed(*kind, before, own);
                now.len() == targets.len()
                    && (now.iter().zip(targets))
                        .all(|((name, node), (was, by))| name == was && node.step == *by)
            }
            Read::Inside { instance, read } => self
                .placed_at(instance, before, own)
                .is_ok_and(|inner| inner.holds(read, inner.ran, None, runner)),
        }
    }

    /// A state `read`, made before `before` for the node `own`, made its
    /// reader read — itself, or through the target it read — if any.
    pub(crate) fn state_read(
        &self,
        read: &Read,
        before: usize,
        own: Option<&str>,
    ) -> Option<String> {
        match read {
            Read::State(name, _) => Some(name.clone()),
            Read::Target { name, producer, .. } => self.node(name, producer)?.state_read.clone(),
            Read::Inside { instance, read } => {
                let inner = self.placed_at(instance, before, own).ok()?;
                let state = inner.state_read(read, inner.ran, None)?;
                Some(crate::placed::key_behind(instance, &state))
            }
            Read::Absent(_) | Read::Input(..) | Read::File(..) | Read::Listed { .. } => None,
        }
    }

    /// What `read`, made by a reader before `before` for the node `own`,
    /// read, as another process would say it, with the digest of what it
    /// read (see [`crate::kept`]): none for what has no digest — a node that
    /// has none, a file read without a workspace.
    pub(crate) fn read_digest(
        &self,
        read: &Read,
        before: usize,
        own: Option<&str>,
        runner: &Runner,
    ) -> Option<(KeptRead, Digest)> {
        Some(match read {
            Read::Target { name, .. } | Read::Absent(name) => (
                KeptRead::Target(name.clone()),
                self.target_digest(name, before, own)?,
            ),
            Read::Input(name, _) => (KeptRead::Input(name.clone()), self.input_digest(name)),
            Read::State(name, _) => (KeptRead::State(name.clone()), self.state_digest(name)),
            Read::File(path, _) => (KeptRead::File(path.clone()), runner.file_digest(path)?),
            Read::Listed {
                kind_name, targets, ..
            } => (
                KeptRead::Listed(kind_name.to_string()),
                listed_digest(
                    targets
                        .iter()
                        .map(|(name, by)| (name.as_str(), by.as_str())),
                ),
            ),
            Read::Inside { instance, read } => {
                let inner = self.placed_at(instance, before, own).ok()?;
                let (read, digest) = inner.read_digest(read, inner.ran, None, runner)?;
                (KeptRead::Inside(instance.clone(), Box::new(read)), digest)
            }
        })
    }

    /// What `kept`, a read made in another process, reads for a reader
    /// before `before` for the node `own` now, as a read of this part, with
    /// the digest of what it reads: [`Part::read_digest`] undone. None for
    /// what has no digest, and for a listing of a type no node of which is
    /// listed, which this process cannot name.
    pub(crate) fn resolve(
        &self,
        kept: &KeptRead,
        before: usize,
        own: Option<&str>,
        runner: &Runner,
    ) -> Option<(Read, Digest)> {
        Some(match kept {
            KeptRead::Target(name) => {
                let digest = self.target_digest(name, before, own)?;
                let read = match self.visible(name, before, own) {
                    Some((_, node)) => Read::Target {
                        name: name.clone(),
                        producer: node.step.clone(),
                        version: node.version,
                    },
                    None => Read::Absent(name.clone()),
                };
                (read, digest)
            }
            KeptRead::Input(name) => (
                Read::Input(name.clone(), self.inputs.get(name).cloned()),
                self.input_digest(name),
            ),
            KeptRead::State(name) => (
                Read::State(name.clone(), self.state.get(name).cloned()),
                self.state_digest(name),
            ),
            KeptRead::File(path) => (
                Read::File(path.clone(), runner.revision(path)),
                runner.file_digest(path)?,
            ),
            KeptRead::Listed(kind_name) => {
                let listed = self.listed_where(|node| node.kind_name == kind_name, before, own);
                let (_, first) = listed.first()?;
                let read = Read::Listed {
                    kind: first.kind,
                    kind_name: first.kind_name,
                    targets: (listed.iter())
                        .map(|(name, node)| (name.to_string(), node.step.clone()))
                        .collect(),
                };
                let digest = listed_digest(
                    listed
                        .iter()
                        .map(|(name, node)| (*name, node.step.as_str())),
                );
                (read, digest)
            }
            KeptRead::Inside(instance, kept) => {
                let inner = self.placed_at(instance, before, own).ok()?;
                let (read, digest) = inner.resolve(kept, inner.ran, None, runner)?;
                let read = Read::Inside {
                    instance: instance.clone(),
                    read: Box::new(read),
                };
                (read, digest)
            }
        })
    }

    /// The digest of the node of `name` a reader before `before` sees, or
    /// that it sees none.
    fn target_digest(&self, name: &str, before: usize, own: Option<&str>) -> Option<Digest> {
        match self.visible(name, before, own) {
            Some((_, node)) => node.digest(),
            None => Some(digest_debug("absent", name)),
        }
    }

    fn input_digest(&self, name: &str) -> Digest {
        digest_debug("input", &self.inputs.get(name))
    }

    fn state_digest(&self, name: &str) -> Digest {
        digest_debug("state", &self.state.get(name))
    }

    /// Every node of a kept type it holds that has a digest and a value,
    /// with its target's name: what is kept of it between processes.
    pub(crate) fn kept_nodes(&self) -> impl Iterator<Item = (&str, &Node)> {
        (self.nodes.iter())
            .flat_map(|(name, nodes)| nodes.iter().map(move |node| (name.as_str(), node)))
            .filter(|(_, node)| node.write.is_some() && node.kept.is_some() && node.value.is_ok())
    }

    /// Starts a run of the steps `order`, given `inputs` and `state`.
    pub(crate) fn begin(&mut self, order: Vec<String>, inputs: &Settings, state: &StateValues) {
        self.positions.clear();
        for (at, step) in order.iter().enumerate() {
            self.positions.entry(step.clone()).or_insert(at);
        }
        self.order = order;
        self.ran = 0;
        self.running = None;
        self.run += 1;
        self.changed = false;
        if self.inputs != *inputs {
            self.inputs = inputs.clone();
        }
        if self.state != *state {
            self.state = state.clone();
        }
    }

    /// Starts the step at position `at`.
    pub(crate) fn begin_step(&mut self, at: usize) {
        self.ran = at;
        self.running = Some(at);
    }

    /// Ends the step at position `at`: drops the nodes it did not produce
    /// in this run.
    pub(crate) fn end_step(&mut self, at: usize) {
        self.running = None;
        self.ran = at + 1;
        let Some(step) = self.order.get(at).cloned() else {
            return;
        };
        let run = self.run;
        self.drop_nodes(&step, |node| node.produced_in != run);
    }

    /// Drops the nodes of `step` that `stale` says, and forgets the step if
    /// none is left.
    fn drop_nodes(&mut self, step: &str, stale: impl Fn(&Node) -> bool) {
        let Some(names) = self.produced.get_mut(step) else {
            return;
        };
        let mut dropped = Vec::new();
        for name in names.iter() {
            if let Some(nodes) = self.nodes.get_mut(name) {
                let before = nodes.len();
                nodes.retain(|node| node.step != step || !stale(node));
                if nodes.len() < before {
                    dropped.push(name.clone());
                }
                if nodes.is_empty() {
                    self.nodes.remove(name);
                }
            }
        }
        for name in &dropped {
            names.remove(name);
        }
        if names.is_empty() {
            self.produced.remove(step);
        }
        self.changed |= !dropped.is_empty();
    }

    /// Ends the run: drops the nodes of steps no longer in the program —
    /// those after where it stopped stay — and notes what it read from
    /// outside and how its steps failed.
    pub(crate) fn end(&mut self, failures: Vec<(String, String)>) {
        self.running = None;
        let gone: Vec<String> = (self.produced.keys())
            .filter(|step| !self.positions.contains_key(*step))
            .cloned()
            .collect();
        for step in gone {
            self.drop_nodes(&step, |_| true);
        }
        if self.changed || self.failures != failures {
            self.revision = next_version();
        }
        self.failures = failures;
        self.states_read.clear();
        self.files_read.clear();
        for read in (self.nodes.values().flatten()).flat_map(|node| node.reads.iter()) {
            match read {
                Read::State(name, _) => {
                    self.states_read.insert(name.clone());
                }
                Read::File(path, revision) => {
                    self.files_read.insert(path.clone(), *revision);
                }
                _ => {}
            }
        }
    }

    /// Stores `node` as what its step produced of `name`.
    fn insert(&mut self, name: &str, node: Node) {
        (self.produced.entry(node.step.clone()).or_default()).insert(name.to_string());
        let nodes = self.nodes.entry(name.to_string()).or_default();
        match nodes.iter_mut().find(|old| old.step == node.step) {
            Some(old) => *old = node,
            None => nodes.push(node),
        }
    }

    /// Where the part, as it is now, differs from `other`: the name of
    /// every target whose current node holds a value — or a failure —
    /// other than the other part's, by the `PartialEq` of its type, or that
    /// only one of them has; and so in the part placed by every placed
    /// node, behind its name. What tells a part a run reused nodes in from
    /// one built from scratch (see `testing::assert_incremental`).
    pub fn differences(&self, other: &Part) -> Vec<String> {
        let names: BTreeSet<&str> = (self.nodes.keys().chain(other.nodes.keys()))
            .map(String::as_str)
            .collect();
        let mut differences = Vec::new();
        for name in names {
            let own = self.visible(name, self.ran, None).map(|(_, node)| node);
            let others = other.visible(name, other.ran, None).map(|(_, node)| node);
            match (own, others) {
                (Some(a), Some(b)) => {
                    let placed = (a.value.as_ref().ok())
                        .and_then(|a| a.downcast_ref::<PlacedPart>())
                        .zip(
                            b.value
                                .as_ref()
                                .ok()
                                .and_then(|b| b.downcast_ref::<PlacedPart>()),
                        );
                    match placed {
                        Some((a, b)) => differences.extend(
                            (a.part.differences(&b.part).into_iter())
                                .map(|inside| format!("{name}: {inside}")),
                        ),
                        None if !a.same_as(b) => differences.push(name.to_string()),
                        None => {}
                    }
                }
                (None, None) => {}
                _ => differences.push(name.to_string()),
            }
        }
        differences
    }

    /// Whether it holds what `other` holds: every target's current node the
    /// same. What tells two builds of a placed part apart.
    pub(crate) fn same_as(&self, other: &Part) -> bool {
        self.differences(other).is_empty()
    }
}

/// What a step is run with: the part, the runner running it, and its own
/// id — what everything it produces is named after. A step only declares
/// what it produces ([`StepContext::produce`]); what takes work is done in
/// the makes.
pub struct StepContext<'a> {
    runner: &'a Runner,
    part: &'a mut Part,
    step: &'a str,
    position: usize,
}

impl<'a> StepContext<'a> {
    pub(crate) fn new(
        runner: &'a Runner,
        part: &'a mut Part,
        step: &'a str,
        position: usize,
    ) -> Self {
        Self {
            runner,
            part,
            step,
            position,
        }
    }

    /// The id of the step.
    pub fn step(&self) -> &'a str {
        self.step
    }

    /// The runner running the step: the files around its program.
    pub fn runner(&self) -> &'a Runner {
        self.runner
    }

    /// The part as the step sees it so far, recording nothing: what a step
    /// that declares one node per element of something asks.
    pub fn reader(&self) -> Reader<'_> {
        Reader::new(self.part, self.position)
    }

    /// Produces the step's node of `target`: reused if it was made from
    /// equal `args` and everything it read reads the same now, else made by
    /// `make`, at once, so that a later node of the step can read it. Fails
    /// as `make` fails, or if the step produced `target` already, or — for
    /// a target that is not a chain ([`Target::CHAIN`]) — if a step before
    /// produced it, naming both.
    pub fn produce<T: Target, Args: PartialEq + Clone + Debug + 'static>(
        &mut self,
        target: &TargetReference<T>,
        args: &Args,
        make: Make<Args, T>,
    ) -> GeopResult<()> {
        let (name, step, at) = (target.name(), self.step, self.position);
        let part = &*self.part;
        let old = part.node(name, step);
        if old.is_some_and(|node| node.produced_in == part.run) {
            return Err(GeopError::new(format!(
                "target `{name}` is produced twice by step `{step}`"
            )));
        }
        if !T::CHAIN
            && let Some((_, earlier)) = part.visible(name, at, Some(name))
        {
            return Err(GeopError::new(format!(
                "target `{name}` is produced by step `{}` and again by step `{step}`: only a \
                 chain target may be produced by several steps",
                earlier.step
            )));
        }
        let reusable = old.is_some_and(|node| {
            node.made_from::<T, Args>(args)
                && (node.reads.iter())
                    .all(|read| part.holds(read, at, Some(name), Some(self.runner)))
        });
        if reusable {
            let run = self.part.run;
            let nodes = self.part.nodes.get_mut(name).into_iter().flatten();
            for node in nodes.filter(|node| node.step == step) {
                node.produced_in = run;
            }
            return Ok(());
        }

        let args_digest = digest_debug("args", args);
        if let Some(keeper) = T::KEEPER
            && let Some(node) = self.restore(name, args, args_digest, &keeper)
        {
            self.runner.count_restored();
            let changed = old.is_none_or(|old| old.version != node.version);
            self.part.changed |= changed;
            self.part.insert(name, node);
            return Ok(());
        }

        self.runner.count_make(T::KEEPER.is_some());
        let part = &*self.part;
        let recording = Recording::default();
        let reader = Reader::making(part, at, name, old, self.runner, &recording);
        let made = make(&reader, args)
            .map_err(|e| e.with_context(format!("making target `{name}` in step `{step}`")));
        let reads = recording.take();
        let state_read = (reads.iter()).find_map(|read| part.state_read(read, at, Some(name)));
        let kept = (reads.iter())
            .map(|read| part.read_digest(read, at, Some(name), self.runner))
            .collect::<Option<Vec<_>>>()
            .map(|reads| {
                let derivation = Derivation {
                    args: args_digest,
                    reads,
                };
                let digest = derivation.digest(step, name, std::any::type_name::<T>());
                (Rc::new(derivation), digest)
            });
        let (value, result) = match made {
            Ok(value) => (Ok(value), Ok(())),
            Err(e) => (Err(e.root_message().to_string()), Err(e)),
        };
        let from = Provenance {
            reads,
            state_read,
            kept,
        };
        let node = Node::new(step, args.clone(), value, from, old, part.run);
        let changed = old.is_none_or(|old| old.version != node.version);
        self.part.changed |= changed;
        self.part.insert(name, node);
        result
    }

    /// The node of `name` an earlier process kept for the program the step
    /// is of, made of arguments of the digest `args_digest` — `args` — if
    /// it kept one and every read it was made from reads the same now (see
    /// [`crate::kept`]): the one used last, of those that do.
    fn restore<T: Target, Args: PartialEq + Clone + 'static>(
        &self,
        name: &str,
        args: &Args,
        args_digest: Digest,
        keeper: &Keeper<T>,
    ) -> Option<Node> {
        let (runner, part, step, at) = (self.runner, &*self.part, self.step, self.position);
        let cache = runner.workspace.as_deref()?.cache()?;
        let path = &runner.location.file;
        let records = cache.builds(path, |builds| builds.records(step, name))?;
        let kind = std::any::type_name::<T>();
        for record in records.iter().rev() {
            if record.kind != kind || record.derivation.args != args_digest {
                continue;
            }
            let reads = (record.derivation.reads.iter())
                .map(|(kept, digest)| {
                    let (read, now) = part.resolve(kept, at, Some(name), runner)?;
                    (now == *digest).then_some(read)
                })
                .collect::<Option<Vec<_>>>();
            let Some(reads) = reads else {
                continue;
            };
            let Some(Ok(value)) = cache.builds(path, |builds| builds.value(record, keeper)) else {
                continue;
            };
            cache.builds(path, |builds| builds.used(record));
            let from = Provenance {
                state_read: (reads.iter()).find_map(|read| part.state_read(read, at, Some(name))),
                reads,
                kept: Some((Rc::new(record.derivation.clone()), record.digest)),
            };
            let old = part.node(name, step);
            let node = Node::new(step, args.clone(), Ok(value), from, old, part.run);
            // Kept already: nothing to tell the cache of it.
            node.noted.set(true);
            return Some(node);
        }
        None
    }
}

/// The digest of a listing of targets: each by its name and the step that
/// produced it, as [`Read::Listed`] holds them.
fn listed_digest<'n>(targets: impl Iterator<Item = (&'n str, &'n str)>) -> Digest {
    let mut hasher = Hasher::default();
    hasher.text("listed");
    for (name, by) in targets {
        hasher.text(name).text(by);
    }
    hasher.finish()
}
