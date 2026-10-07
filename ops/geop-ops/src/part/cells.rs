//! [`Store`]: what a [`Part`](super::Part) holds, in *cells*, so that what a
//! step of a program reads of it, and what it writes, is known without the
//! step saying.
//!
//! A cell is the unit a read or a write is told apart in: the topology
//! (one cell: operations on it are a chain anyway), the entry of one name,
//! one instance, one program input, one extension — and a few sections
//! kept whole (sketches, datums). Every access to the data goes through a
//! method of `Store`, whose fields are private, so none can be missed:
//!
//! - A *read* is noted in the [`Log`] the runner installs for a step, if
//!   there is one.
//! - A *write* is noted there too, and gives the cell a new *version*,
//!   unique across all of them. Two parts with the same version of a cell
//!   hold the same content of it. A write is also a read: an operation that
//!   changes a cell is taken to depend on it. (Adding an entry under a new
//!   name reads that name, absent.) The exception is a collection, below.
//! - Reading a whole collection (every instance, every name) is a read of
//!   the collection's own cell, which any write to a member bumps.
//!
//! The runner uses this to skip a step whose reads have the versions they
//! had when it last ran, and to put what it wrote then into the part
//! instead ([`Store::replay`]).

use std::{
    any::Any,
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, Mutex, PoisonError,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
};

use geop_core_math::scalars::Scalar;
use geop_core_topology::Model;
use indexmap::IndexMap;

use super::{
    Instance,
    extension::Extensions,
    feature::Feature,
    ids::{DatumId, InstanceId, RefId, Sketch3dId, SketchId},
    names::NameRegistry,
    sketch::PlacedSketch,
    sketch3d::PartSketch3d,
    state::State,
};
use crate::assembly::Mate;
use geop_core_math::primitives::Datum;

/// One thing a step can read or write of a part (see the module).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Cell {
    /// The B-rep: every vertex, edge, face and solid.
    Topology,
    /// What the name registry says of one name — taken, by what, or free.
    Name(String),
    /// The registry as a whole: every name.
    Names,
    /// Every sketch, planar and 3-D.
    Sketches,
    /// Every datum.
    Datums,
    /// What operation families recorded on solids (see
    /// [`Part::body_data`](super::Part::body_data)).
    BodyData,
    /// The mate under one name.
    Mate(String),
    /// Every mate: the list of them.
    Mates,
    /// What the steps that combined tools with a solid did.
    Features,
    /// What the part's parameters are defined as.
    Parameters,
    /// The part placed under one name.
    Instance(String),
    /// Every part placed: the list of them.
    Instances,
    /// The value of one program input.
    State(String),
    /// Every program input: the list of them.
    Inputs,
    /// What one operation family kept (see [`Extension`](super::Extension)).
    Ext(&'static str),
}

impl Cell {
    /// Whether the cell is a list of others (see the module): writing one
    /// of them is not reading the list, or every step that adds a name
    /// would depend on the one before it.
    fn is_collection(&self) -> bool {
        matches!(
            self,
            Cell::Names | Cell::Instances | Cell::Mates | Cell::Inputs
        )
    }
}

/// What a step read and wrote of a part: see [`Log::take`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Access {
    pub reads: BTreeSet<Cell>,
    /// The cells it wrote. Those that are no collection it also read.
    pub writes: BTreeSet<Cell>,
}

/// What is noted of a step's accesses to a part and every copy of it (see
/// [`Store::record`]).
#[derive(Debug, Default)]
pub struct Log {
    /// Read so often that it is a flag before it is a set entry.
    topology_read: AtomicBool,
    access: Mutex<Access>,
}

impl Log {
    pub fn new() -> Arc<Self> {
        Arc::default()
    }

    fn read(&self, cell: &Cell) {
        if *cell == Cell::Topology {
            self.topology_read.store(true, Ordering::Relaxed);
            return;
        }
        let mut access = self.access.lock().unwrap_or_else(PoisonError::into_inner);
        if !access.reads.contains(cell) {
            access.reads.insert(cell.clone());
        }
    }

    fn write(&self, cell: &Cell) {
        if !cell.is_collection() {
            self.read(cell);
        }
        let mut access = self.access.lock().unwrap_or_else(PoisonError::into_inner);
        access.writes.insert(cell.clone());
    }

    /// What was noted, so far.
    pub fn take(&self) -> Access {
        let mut access = self
            .access
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        if self.topology_read.load(Ordering::Relaxed) {
            access.reads.insert(Cell::Topology);
        }
        access
    }
}

/// The version the next write gives its cell: unique across all parts.
static NEXT_VERSION: AtomicU64 = AtomicU64::new(1);

/// What a part holds (see the module).
#[derive(Clone)]
pub(super) struct Store<S: Scalar> {
    topology: Arc<Model<S>>,
    names: Arc<NameRegistry>,
    sketches: IndexMap<SketchId, PlacedSketch<S>>,
    sketches3d: IndexMap<Sketch3dId, PartSketch3d>,
    datums: IndexMap<DatumId, Datum<S>>,
    instances: IndexMap<InstanceId, Arc<Instance<S>>>,
    body_data: BTreeMap<String, Arc<dyn Any + Send + Sync>>,
    mates: BTreeMap<String, Mate>,
    features: Vec<(String, Arc<Feature<S>>)>,
    state: State,
    declared: State,
    parameters: crate::parameters::Parameters,
    extensions: Extensions<S>,
    versions: Arc<BTreeMap<Cell, u64>>,
    log: Option<Arc<Log>>,
}

impl<S: Scalar> Store<S> {
    pub(super) fn new() -> Self {
        Self {
            topology: Arc::new(Model::new()),
            names: Arc::new(NameRegistry::new()),
            sketches: IndexMap::new(),
            sketches3d: IndexMap::new(),
            datums: IndexMap::new(),
            instances: IndexMap::new(),
            body_data: BTreeMap::new(),
            mates: BTreeMap::new(),
            features: Vec::new(),
            state: State::new(),
            declared: State::new(),
            parameters: crate::parameters::Parameters::default(),
            extensions: Extensions::new(),
            versions: Arc::new(BTreeMap::new()),
            log: None,
        }
    }

    // --- recording ---

    /// Notes every access to this part, and to every copy made of it from
    /// now on, in `log`.
    pub(super) fn record(&mut self, log: Arc<Log>) {
        self.log = Some(log);
    }

    /// Stops noting accesses, and returns the log that did.
    pub(super) fn stop_recording(&mut self) -> Option<Arc<Log>> {
        self.log.take()
    }

    /// Whether `log` is the one noting accesses to this part: if not, the
    /// part is not a copy of the one that was recorded, and nothing is
    /// known of what made it.
    pub(super) fn is_recording(&self, log: &Arc<Log>) -> bool {
        self.log.as_ref().is_some_and(|l| Arc::ptr_eq(l, log))
    }

    pub(super) fn read(&self, cell: Cell) {
        if let Some(log) = &self.log {
            log.read(&cell);
        }
    }

    /// Notes a write to `cell`, and gives it a version of its own.
    pub(super) fn write(&mut self, cell: Cell) {
        if let Some(log) = &self.log {
            log.write(&cell);
        }
        let version = NEXT_VERSION.fetch_add(1, Ordering::Relaxed);
        Arc::make_mut(&mut self.versions).insert(cell, version);
    }

    /// The version of `cell`: 0 for one never written.
    pub(super) fn version(&self, cell: &Cell) -> u64 {
        self.versions.get(cell).copied().unwrap_or(0)
    }

    // --- topology ---

    pub(super) fn topology(&self) -> &Model<S> {
        self.read(Cell::Topology);
        &self.topology
    }

    pub(super) fn topology_mut(&mut self) -> &mut Model<S> {
        self.write(Cell::Topology);
        Arc::make_mut(&mut self.topology)
    }

    // --- names ---

    /// The registry, to list every name.
    pub(super) fn names(&self) -> &NameRegistry {
        self.read(Cell::Names);
        &self.names
    }

    pub(super) fn name_of(&self, id: RefId) -> Option<&str> {
        match self.names.name_of(id) {
            Some(name) => {
                self.read(Cell::Name(name.to_string()));
                Some(name)
            }
            None => {
                // Nothing is named so: that holds until a name is made.
                self.read(Cell::Names);
                None
            }
        }
    }

    pub(super) fn id_of(&self, name: &str) -> Option<RefId> {
        self.read(Cell::Name(name.to_string()));
        self.names.id_of(name)
    }

    pub(super) fn insert_name(
        &mut self,
        id: impl Into<RefId>,
        name: impl Into<String>,
    ) -> geop_core_math::geop_error::GeopResult<()> {
        let name = name.into();
        self.write(Cell::Name(name.clone()));
        self.write(Cell::Names);
        Arc::make_mut(&mut self.names).insert(id, name)
    }

    pub(super) fn rename(
        &mut self,
        id: RefId,
        new_name: impl Into<String>,
    ) -> geop_core_math::geop_error::GeopResult<()> {
        let new_name = new_name.into();
        if let Some(old) = self.names.name_of(id) {
            let old = old.to_string();
            self.write(Cell::Name(old));
        }
        self.write(Cell::Name(new_name.clone()));
        self.write(Cell::Names);
        Arc::make_mut(&mut self.names).rename(id, new_name)
    }

    pub(super) fn remove_name(&mut self, id: impl Into<RefId>) {
        let id = id.into();
        if let Some(name) = self.names.name_of(id) {
            let name = name.to_string();
            self.write(Cell::Name(name));
            self.write(Cell::Names);
        }
        Arc::make_mut(&mut self.names).remove(id);
    }

    /// Forgets the names of every entity `alive` rejects.
    pub(super) fn retain_names(&mut self, mut alive: impl FnMut(RefId) -> bool) {
        let dead: Vec<String> = self
            .names
            .iter()
            .filter(|&(id, _)| !alive(id))
            .map(|(_, name)| name.to_string())
            .collect();
        if dead.is_empty() {
            return;
        }
        for name in dead {
            self.write(Cell::Name(name));
        }
        self.write(Cell::Names);
        Arc::make_mut(&mut self.names).retain(alive);
    }

    // --- sketches and datums ---

    pub(super) fn sketches(&self) -> &IndexMap<SketchId, PlacedSketch<S>> {
        self.read(Cell::Sketches);
        &self.sketches
    }

    pub(super) fn sketches_mut(&mut self) -> &mut IndexMap<SketchId, PlacedSketch<S>> {
        self.write(Cell::Sketches);
        &mut self.sketches
    }

    pub(super) fn sketches3d(&self) -> &IndexMap<Sketch3dId, PartSketch3d> {
        self.read(Cell::Sketches);
        &self.sketches3d
    }

    pub(super) fn sketches3d_mut(&mut self) -> &mut IndexMap<Sketch3dId, PartSketch3d> {
        self.write(Cell::Sketches);
        &mut self.sketches3d
    }

    pub(super) fn datums(&self) -> &IndexMap<DatumId, Datum<S>> {
        self.read(Cell::Datums);
        &self.datums
    }

    pub(super) fn datums_mut(&mut self) -> &mut IndexMap<DatumId, Datum<S>> {
        self.write(Cell::Datums);
        &mut self.datums
    }

    // --- instances ---

    /// Every instance, to list.
    pub(super) fn instances(&self) -> &IndexMap<InstanceId, Arc<Instance<S>>> {
        self.read(Cell::Instances);
        &self.instances
    }

    /// The instance `id`, a read of the one placed under its name.
    pub(super) fn instance(&self, id: InstanceId) -> Option<&Instance<S>> {
        match self.names.name_of(RefId::Instance(id)) {
            Some(name) => self.read(Cell::Instance(name.to_string())),
            None => self.read(Cell::Instances),
        }
        self.instances.get(&id).map(|i| &**i)
    }

    /// Places `instance` as `id`, named `name`.
    pub(super) fn insert_instance(&mut self, id: InstanceId, name: &str, instance: Instance<S>) {
        self.write(Cell::Instance(name.to_string()));
        self.write(Cell::Instances);
        self.instances.insert(id, Arc::new(instance));
    }

    // --- body data, mates, features ---

    pub(super) fn body_data(&self) -> &BTreeMap<String, Arc<dyn Any + Send + Sync>> {
        self.read(Cell::BodyData);
        &self.body_data
    }

    pub(super) fn body_data_mut(&mut self) -> &mut BTreeMap<String, Arc<dyn Any + Send + Sync>> {
        self.write(Cell::BodyData);
        &mut self.body_data
    }

    /// Every mate, to list.
    pub(super) fn mates(&self) -> &BTreeMap<String, Mate> {
        self.read(Cell::Mates);
        &self.mates
    }

    /// Whether there is a mate named `name`.
    pub(super) fn has_mate(&self, name: &str) -> bool {
        self.read(Cell::Mate(name.to_string()));
        self.mates.contains_key(name)
    }

    pub(super) fn insert_mate(&mut self, name: String, mate: Mate) {
        self.write(Cell::Mate(name.clone()));
        self.write(Cell::Mates);
        self.mates.insert(name, mate);
    }

    pub(super) fn features(&self) -> &[(String, Arc<Feature<S>>)] {
        self.read(Cell::Features);
        &self.features
    }

    pub(super) fn features_mut(&mut self) -> &mut Vec<(String, Arc<Feature<S>>)> {
        self.write(Cell::Features);
        &mut self.features
    }

    // --- inputs ---

    /// Every input value, to list.
    pub(super) fn state(&self) -> &State {
        self.read(Cell::Inputs);
        &self.state
    }

    /// The value of the input `name`.
    pub(super) fn input(&self, name: &str) -> Option<&crate::part::ParamValue> {
        self.read(Cell::State(name.to_string()));
        self.state.get(name)
    }

    /// Sets the values the part is built with, as the runner does before a
    /// step: each input whose value differs from what it was is written.
    pub(super) fn set_state(&mut self, state: State) {
        let names: BTreeSet<&String> = self.state.keys().chain(state.keys()).collect();
        let changed: Vec<String> = names
            .into_iter()
            .filter(|name| self.state.get(*name) != state.get(*name))
            .cloned()
            .collect();
        self.state = state;
        if !changed.is_empty() {
            self.write_unlogged(Cell::Inputs);
        }
        for name in changed {
            self.write_unlogged(Cell::State(name));
        }
    }

    /// Gives `cell` a new version without noting a write: the runner
    /// changes what a part is built with, and no step did.
    fn write_unlogged(&mut self, cell: Cell) {
        let version = NEXT_VERSION.fetch_add(1, Ordering::Relaxed);
        Arc::make_mut(&mut self.versions).insert(cell, version);
    }

    pub(super) fn declared(&self) -> &State {
        &self.declared
    }

    pub(super) fn declared_mut(&mut self) -> &mut State {
        &mut self.declared
    }

    pub(super) fn parameters(&self) -> &crate::parameters::Parameters {
        self.read(Cell::Parameters);
        &self.parameters
    }

    pub(super) fn set_parameters(&mut self, parameters: crate::parameters::Parameters) {
        if self.parameters != parameters {
            self.write_unlogged(Cell::Parameters);
        }
        self.parameters = parameters;
    }

    // --- replaying ---

    /// Puts into this part what a step wrote when it last ran: the `writes`
    /// of the step, which turned the part `before` into the part `after`.
    /// This part is the one the step would run on now, and has each cell
    /// the step read as `before` had it, so the step would write the same.
    pub(super) fn replay(&mut self, before: &Self, after: &Self, writes: &BTreeSet<Cell>) {
        for cell in writes {
            match cell {
                Cell::Topology => self.topology = after.topology.clone(),
                Cell::Name(name) => {
                    let names = Arc::make_mut(&mut self.names);
                    if let Some(id) = names.id_of(name) {
                        names.remove(id);
                    }
                    if let Some(id) = after.names.id_of(name) {
                        names
                            .insert(id, name.as_str())
                            .expect("the name was freed, and the id is the one of a name");
                    }
                }
                Cell::Sketches => {
                    self.sketches = after.sketches.clone();
                    self.sketches3d = after.sketches3d.clone();
                }
                Cell::Datums => self.datums = after.datums.clone(),
                Cell::BodyData => self.body_data = after.body_data.clone(),
                Cell::Mate(name) => match after.mates.get(name) {
                    Some(mate) => {
                        self.mates.insert(name.clone(), mate.clone());
                    }
                    None => {
                        self.mates.remove(name);
                    }
                },
                Cell::Features => self.features = after.features.clone(),
                Cell::Instance(name) => {
                    let id = InstanceId::named(name);
                    self.instances.shift_remove(&id);
                    if let Some(instance) = after.instances.get(&id) {
                        self.instances.insert(id, instance.clone());
                    }
                }
                Cell::Ext(name) => self.extensions.copy_entry(&after.extensions, name),
                // Lists have nothing of their own to copy; the program's
                // inputs and parameters are not written by a step.
                Cell::Names | Cell::Instances | Cell::Mates | Cell::Inputs => {}
                Cell::State(_) | Cell::Parameters => {}
            }
        }
        // What a step declared it read (see `Part::declared`).
        for (name, value) in after.declared.iter() {
            if !before.declared.contains_key(name) {
                self.declared.insert(name.clone(), value.clone());
            }
        }
        // A cell is as the step left it only if what it was written onto is
        // what it was written onto before; else it is another content.
        let versions = Arc::make_mut(&mut self.versions);
        for cell in writes {
            let same_base = versions.get(cell) == before.versions.get(cell);
            let version = if same_base {
                after.version(cell)
            } else {
                NEXT_VERSION.fetch_add(1, Ordering::Relaxed)
            };
            versions.insert(cell.clone(), version);
        }
    }

    /// Whether every cell has the version it has in `other`: the two hold
    /// the same content.
    pub(super) fn same_versions(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.versions, &other.versions) || self.versions == other.versions
    }

    // --- extensions ---

    pub(super) fn extensions(&self) -> &Extensions<S> {
        &self.extensions
    }

    pub(super) fn extensions_mut(&mut self) -> &mut Extensions<S> {
        &mut self.extensions
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use geop_core_math::{scalars::scal_in_f64::ScalInF64, vector::Vector3};

    use super::*;
    use crate::part::Part;

    type S = ScalInF64;

    fn cells(cells: impl IntoIterator<Item = Cell>) -> BTreeSet<Cell> {
        cells.into_iter().collect()
    }

    fn instance() -> Instance<S> {
        Instance::of("bolt.geop".into(), Part::new(), BTreeSet::new())
    }

    /// Placing a part reads and writes its own name and the list of parts
    /// placed, and nothing else: not the topology, not the other parts.
    #[test]
    fn placing_a_part_touches_only_its_own_cells() {
        let mut part = Part::<S>::new();
        part.add_instance(instance(), "first").unwrap();

        let log = Log::new();
        part.store.record(log.clone());
        part.add_instance(instance(), "second").unwrap();
        let access = log.take();

        // The lists it adds to, it does not read.
        let own = cells([Cell::Name("second".into()), Cell::Instance("second".into())]);
        assert_eq!(access.reads, own);
        let mut writes = own;
        writes.extend([Cell::Names, Cell::Instances]);
        assert_eq!(access.writes, writes);
    }

    /// Looking at the topology is a read of all of it, and building in it a
    /// write of it and of the names it gives.
    #[test]
    fn topology_is_one_cell() {
        let mut part = Part::<S>::new();
        let log = Log::new();
        part.store.record(log.clone());
        assert!(part.topology().solids.is_empty());
        assert_eq!(log.take().reads, cells([Cell::Topology]));

        part.mvfs(Vector3::zero(), "v", "f", "s").unwrap();
        let access = log.take();
        assert!(access.writes.contains(&Cell::Topology));
        assert!(access.writes.contains(&Cell::Name("s".into())));
        assert!(!access.writes.contains(&Cell::Instances));
    }

    /// A copy of a recorded part is recorded too, in the same log; and a
    /// write gives its cell a version no other has, which a copy that was
    /// not written keeps.
    #[test]
    fn copies_share_the_log_and_versions_tell_what_was_written() {
        let mut part = Part::<S>::new();
        part.add_instance(instance(), "first").unwrap();
        let first = Cell::Instance("first".into());
        let before = part.store.version(&first);
        assert_ne!(before, 0);

        let log = Log::new();
        part.store.record(log.clone());
        let mut copy = part.clone();
        assert!(copy.store.is_recording(&log));
        copy.add_instance(instance(), "second").unwrap();

        assert!(log.take().writes.contains(&Cell::Instance("second".into())));
        assert_eq!(copy.store.version(&first), before);
        assert_eq!(part.store.version(&Cell::Instance("second".into())), 0);
        assert_ne!(copy.store.version(&Cell::Instance("second".into())), 0);
    }

    /// An input is read by name, and listing them all is a read of all.
    #[test]
    fn inputs_are_read_by_name() {
        let mut part = Part::<S>::new().with_state(State::from([(
            "width".to_string(),
            crate::part::ParamValue::Text("a".into()),
        )]));
        let log = Log::new();
        part.store.record(log.clone());
        part.input("width");
        assert_eq!(log.take().reads, cells([Cell::State("width".into())]));
        part.state();
        assert!(log.take().reads.contains(&Cell::Inputs));
        part.store.stop_recording();
    }
}
