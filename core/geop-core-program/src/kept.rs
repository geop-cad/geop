//! What a program built, kept between processes: reopening a program whose
//! steps and inputs did not change builds nothing.
//!
//! Every node knows how it was made, in terms that mean the same in every
//! process ([`Derivation`]): a digest of its arguments, and for each thing it
//! read, what it read ([`KeptRead`]) and a digest of what that was — of the
//! node it read, of an input's or a state's value, of a file's text. A
//! node's own digest ([`Digest`]) is one of how it was made, so two nodes of
//! one digest hold one value, in any process: makes are plain functions of
//! their arguments and what they read (see [`crate::part::Make`]). A node
//! made again into a value equal to the last keeps its digest, as it keeps
//! its version.
//!
//! A target whose type says how its values are written ([`Keeper`]) has its
//! nodes kept in the cache file of the program they were built by, beside it
//! (`part.geop.geopc`, see [`crate::cache`]): the value, how it was made and
//! its digest ([`Record`]). A node a run would make is taken from there
//! instead when its arguments are the same and every read reads the same
//! now — the same digest, in this process — and only then: what a record
//! says is never trusted, it is checked. Parts of values shared between
//! records — the faces two revisions of a model have alike — are written
//! once ([`Pool`]).
//!
//! What is made of a part outside its program — a drawing of it, which an
//! editor makes of what a reader reads — is kept the same way, as made of
//! the program as a whole rather than by one of its steps ([`made_kept`]).
//!
//! A cache file is of one build of the kernel and one version of its format
//! ([`FORMAT`]): any other is not read, and is replaced when it is written
//! again.

use std::{
    collections::{BTreeMap, BTreeSet},
    fmt::Write as _,
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::DeserializeOwned};
use serde_json::value::RawValue;

use crate::{
    reader::{Reader, Reads},
    runner::Runner,
};

/// What `make` makes of what `reader` reads — what an editor draws of the
/// part — kept between processes as `key` of the program `runner` runs,
/// whose part `reader` reads: read back as `keeper` wrote it where an
/// earlier process made it of what reads the same now, else made, and
/// noted to be kept. With what it read ([`Reader::recorded`]).
pub fn made_kept<T: 'static>(
    reader: &Reader,
    runner: &Runner,
    key: &str,
    keeper: Keeper<T>,
    make: impl FnOnce(&Reader) -> GeopResult<T>,
) -> (GeopResult<Rc<T>>, Reads) {
    let (part, before, own) = (reader.part(), reader.position(), reader.own());
    let path = &runner.location.file;
    let Some(cache) = runner.workspace.as_deref().and_then(|w| w.cache()) else {
        let (made, reads) = reader.recorded(make);
        return (made.map(Rc::new), reads);
    };
    let kind = std::any::type_name::<T>();
    let args = digest_debug("made of the program", key);
    let records = cache.builds(path, |builds| builds.records("", key));
    for record in records.iter().flatten().rev() {
        if record.kind != kind || record.derivation.args != args {
            continue;
        }
        let reads = (record.derivation.reads.iter())
            .map(|(kept, digest)| {
                let (read, now) = part.resolve(kept, before, own, runner)?;
                (now == *digest).then_some(read)
            })
            .collect::<Option<Vec<_>>>();
        let Some(reads) = reads else {
            continue;
        };
        if let Some(Ok(value)) = cache.builds(path, |builds| builds.value(record, &keeper)) {
            cache.builds(path, |builds| builds.used(record));
            runner.count_restored();
            return (Ok(Rc::new(value)), Reads(reads));
        }
    }
    let (made, reads) = reader.recorded(make);
    let made = made.map(Rc::new);
    let derivation = (reads.0.iter())
        .map(|read| part.read_digest(read, before, own, runner))
        .collect::<Option<Vec<_>>>()
        .map(|reads| Derivation { args, reads });
    if let (Ok(value), Some(derivation)) = (&made, derivation) {
        let digest = derivation.digest("", key, kind);
        let value = value.clone();
        cache.builds(path, |builds| {
            if !builds.has("", key, &derivation, digest) {
                builds.add(Made {
                    step: String::new(),
                    target: key.to_string(),
                    kind,
                    derivation: Rc::new(derivation),
                    digest,
                    write: Box::new(move |pool| (keeper.write)(&value, pool)),
                });
            }
        });
    }
    (made, reads)
}

/// The version of the format a build is kept in: a cache file of another
/// version is not read.
pub const FORMAT: u32 = 1;

/// How many records of one target of one step a cache file keeps — those
/// used last: the builds of a program with as many sets of inputs (the sizes
/// of a screw placed in an assembly), and of the edits before the last.
const KEPT_RECORDS: usize = 16;

/// How the values of a target type are written to be read back exactly, in
/// another process (see the [module](self)): a target without one is made
/// again in every process.
pub struct Keeper<T> {
    /// The value as JSON, its parts that other values may share put in the
    /// pool.
    pub write: fn(&T, &mut Pool) -> GeopResult<String>,
    /// The value `write` wrote, its parts taken from the pool.
    pub read: fn(&str, &Pool) -> GeopResult<T>,
}

impl<T> Clone for Keeper<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for Keeper<T> {}

impl<T: Serialize + DeserializeOwned> Keeper<T> {
    /// A value written by its own serde, which must write it exactly
    /// ([`geop_core_math::exact`]): read back, it is the value written.
    pub const SERDE: Self = Self {
        write: write_serde::<T>,
        read: read_serde::<T>,
    };
}

fn write_serde<T: Serialize>(value: &T, _: &mut Pool) -> GeopResult<String> {
    to_text(value)
}

fn read_serde<T: DeserializeOwned>(text: &str, _: &Pool) -> GeopResult<T> {
    from_text(text)
}

/// `value` as JSON.
fn to_text<T: Serialize>(value: &T) -> GeopResult<String> {
    serde_json::to_string(value).map_err(|e| GeopError::new(format!("writing a kept value: {e}")))
}

/// The value the JSON `text` holds.
fn from_text<T: DeserializeOwned>(text: &str) -> GeopResult<T> {
    serde_json::from_str(text).map_err(|e| GeopError::new(format!("reading a kept value: {e}")))
}

/// Values the records of one cache file share, each written once, by the
/// digest of its text: a model's faces, which every revision of the model
/// that kept them holds alike.
#[derive(Default)]
pub struct Pool {
    values: BTreeMap<String, Box<RawValue>>,
    /// What was put since the last [`Pool::take_used`]: what the record
    /// being written uses.
    used: BTreeSet<String>,
    /// What was read of it shared ([`Pool::shared`]), by key.
    read: std::cell::RefCell<BTreeMap<String, Rc<dyn std::any::Any>>>,
}

impl Pool {
    /// Puts `value`, and gives what it is taken by ([`Pool::get`]).
    pub fn put<V: Serialize>(&mut self, value: &V) -> GeopResult<String> {
        let text = to_text(value)?;
        let key = format!("{:032x}", digest_text(&text));
        if !self.values.contains_key(&key) {
            let raw = RawValue::from_string(text)
                .map_err(|e| GeopError::new(format!("writing a kept value: {e}")))?;
            self.values.insert(key.clone(), raw);
        }
        self.used.insert(key.clone());
        Ok(key)
    }

    /// The value put as `key`.
    pub fn get<V: DeserializeOwned>(&self, key: &str) -> GeopResult<V> {
        let raw = (self.values.get(key)).ok_or_else(|| {
            GeopError::new(format!("a kept value names {key}, which is not kept"))
        })?;
        from_text(raw.get())
    }

    /// What `read` reads of the value put as `key`, read once for every
    /// record that reads it so: what many records hold alike — the drawing
    /// of a part placed a thousand times — is one value in memory. `read`
    /// reads the same of the same key, as a `V`.
    pub fn shared<V: 'static>(
        &self,
        key: &str,
        read: impl FnOnce() -> GeopResult<V>,
    ) -> GeopResult<Rc<V>> {
        let at = format!("{}:{key}", std::any::type_name::<V>());
        if let Some(kept) = self.read.borrow().get(&at)
            && let Ok(kept) = kept.clone().downcast::<V>()
        {
            return Ok(kept);
        }
        let kept = Rc::new(read()?);
        (self.read.borrow_mut()).insert(at, kept.clone());
        Ok(kept)
    }

    /// How many values it holds.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether it holds none.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// What was put since the last time it was asked.
    fn take_used(&mut self) -> BTreeSet<String> {
        std::mem::take(&mut self.used)
    }
}

/// A digest of how a node was made, or of what it read: the same in every
/// process for the same thing (see the [module](self)). Written as hex.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Digest(pub u128);

impl Serialize for Digest {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(&format_args!("{:032x}", self.0))
    }
}

impl<'de> Deserialize<'de> for Digest {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = String::deserialize(deserializer)?;
        u128::from_str_radix(&text, 16)
            .map(Digest)
            .map_err(serde::de::Error::custom)
    }
}

/// A 128-bit FNV-1a hash, fed piece by piece: written out here rather than
/// the standard library's hasher, which may change between Rust versions.
pub(crate) struct Hasher(u128);

impl Default for Hasher {
    fn default() -> Self {
        Self(0x6c62272e07bb014262b821756295c58d)
    }
}

impl Hasher {
    pub(crate) fn bytes(&mut self, bytes: &[u8]) {
        const PRIME: u128 = 0x0000000001000000000000000000013B;
        for &byte in bytes {
            self.0 ^= u128::from(byte);
            self.0 = self.0.wrapping_mul(PRIME);
        }
    }

    /// `text`, told from what follows: its length first.
    pub(crate) fn text(&mut self, text: &str) -> &mut Self {
        self.bytes(&(text.len() as u64).to_le_bytes());
        self.bytes(text.as_bytes());
        self
    }

    pub(crate) fn digest(&mut self, digest: Digest) -> &mut Self {
        self.bytes(&digest.0.to_le_bytes());
        self
    }

    pub(crate) fn finish(&self) -> Digest {
        Digest(self.0)
    }
}

impl std::fmt::Write for Hasher {
    fn write_str(&mut self, s: &str) -> std::fmt::Result {
        self.bytes(s.as_bytes());
        Ok(())
    }
}

/// The digest of `text`.
pub(crate) fn digest_text(text: &str) -> u128 {
    let mut hasher = Hasher::default();
    hasher.bytes(text.as_bytes());
    hasher.0
}

/// The digest of `value` as `Debug` writes it: of a node's arguments, an
/// input's or a state's value. `Debug` is derived for them, every field
/// written, and a number written exactly — an interval's both bounds, to
/// the bit — so values that write alike are equal.
pub(crate) fn digest_debug<T: std::fmt::Debug + ?Sized>(what: &str, value: &T) -> Digest {
    let mut hasher = Hasher::default();
    hasher.text(what);
    // Writing into a hasher does not fail.
    let _ = write!(hasher, "{value:?}");
    hasher.finish()
}

/// What a node read, as a [`crate::target::Read`] says it, in terms that
/// mean the same in another process: no version, no type id.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) enum KeptRead {
    /// Whatever node of the target the reader sees, or that there is none.
    Target(String),
    Input(String),
    State(String),
    File(String),
    /// Which targets of the type of that name there are.
    Listed(String),
    Inside(String, Box<KeptRead>),
}

/// How a node was made, in terms that mean the same in every process: its
/// arguments' digest, and each read with the digest of what it read.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct Derivation {
    pub args: Digest,
    pub reads: Vec<(KeptRead, Digest)>,
}

impl Derivation {
    /// The digest of the node `step` made so of `target`, a `kind`.
    pub(crate) fn digest(&self, step: &str, target: &str, kind: &str) -> Digest {
        let mut hasher = Hasher::default();
        hasher.text(step).text(target).text(kind).digest(self.args);
        for (read, digest) in &self.reads {
            let _ = write!(hasher, "{read:?}");
            hasher.digest(*digest);
        }
        hasher.finish()
    }
}

/// A node kept: the step and target it is of, the type of its value, how
/// it was made and its digest — and its value, as its type's [`Keeper`]
/// wrote it, with the parts of the pool it uses.
#[derive(Serialize, Deserialize)]
pub(crate) struct Record {
    pub step: String,
    pub target: String,
    pub kind: String,
    pub derivation: Derivation,
    pub digest: Digest,
    value: Box<RawValue>,
    uses: BTreeSet<String>,
}

impl Record {
    /// Whether it is the record of a node so made, of that digest.
    fn is(&self, derivation: &Derivation, digest: Digest) -> bool {
        self.digest == digest && self.derivation == *derivation
    }
}

/// A node made in this process, to be kept when the cache is next written
/// ([`crate::cache::Cache::flush`]).
pub(crate) struct Made {
    pub step: String,
    pub target: String,
    pub kind: &'static str,
    pub derivation: Rc<Derivation>,
    pub digest: Digest,
    /// Writes the value made, as its type's keeper does.
    pub write: WriteMade,
}

/// What writes the value of a node made, to keep it ([`Made::write`]).
pub(crate) type WriteMade = Box<dyn Fn(&mut Pool) -> GeopResult<String>>;

/// A cache file of builds, as it is written.
#[derive(Serialize, Deserialize)]
struct File {
    format: u32,
    /// The build of the kernel that wrote it ([`crate::cache::CacheStore::build`]).
    build: String,
    records: Vec<Record>,
    pool: BTreeMap<String, Box<RawValue>>,
}

/// What a program built, as kept: the records of its cache file — by step
/// and target, the one used last last — and what was made since, to be
/// kept too.
#[derive(Default)]
pub(crate) struct Builds {
    records: BTreeMap<(String, String), Vec<Rc<Record>>>,
    pool: Pool,
    made: Vec<Made>,
    /// The steps of the program, as its last run had them: a record of a
    /// step no longer in it is not kept.
    order: Option<BTreeSet<String>>,
    changed: bool,
}

impl Builds {
    /// The builds the cache file `text` keeps, if it is of this format and
    /// of the kernel's build `build`; none otherwise.
    pub(crate) fn read(text: Option<&str>, build: &str) -> Self {
        let file = text.and_then(|text| serde_json::from_str::<File>(text).ok());
        let Some(file) = file.filter(|file| file.format == FORMAT && file.build == build) else {
            return Self::default();
        };
        let mut records: BTreeMap<_, Vec<Rc<Record>>> = BTreeMap::new();
        for record in file.records {
            let key = (record.step.clone(), record.target.clone());
            records.entry(key).or_default().push(Rc::new(record));
        }
        Self {
            records,
            pool: Pool {
                values: file.pool,
                ..Pool::default()
            },
            ..Self::default()
        }
    }

    /// The records of the target `target` of the step `step`.
    pub(crate) fn records(&self, step: &str, target: &str) -> Vec<Rc<Record>> {
        let key = (step.to_string(), target.to_string());
        self.records.get(&key).cloned().unwrap_or_default()
    }

    /// The value `record` keeps, read by `keeper`.
    pub(crate) fn value<T>(&self, record: &Record, keeper: &Keeper<T>) -> GeopResult<T> {
        (keeper.read)(record.value.get(), &self.pool).map_err(|e| {
            e.with_context(format!(
                "reading the kept target `{}` of step `{}`",
                record.target, record.step
            ))
        })
    }

    /// Notes that `record` was used: it is kept before records used longer
    /// ago.
    pub(crate) fn used(&mut self, record: &Rc<Record>) {
        let key = (record.step.clone(), record.target.clone());
        if let Some(records) = self.records.get_mut(&key)
            && let Some(at) = records.iter().position(|r| Rc::ptr_eq(r, record))
        {
            let record = records.remove(at);
            records.push(record);
        }
    }

    /// Whether a node so made, of that digest, is kept or to be kept.
    pub(crate) fn has(
        &self,
        step: &str,
        target: &str,
        derivation: &Derivation,
        digest: Digest,
    ) -> bool {
        let key = (step.to_string(), target.to_string());
        let kept = (self.records.get(&key).into_iter().flatten()).any(|r| r.is(derivation, digest));
        kept || (self.made.iter()).any(|m| {
            m.step == step
                && m.target == target
                && m.digest == digest
                && *m.derivation == *derivation
        })
    }

    /// Notes `made`, to be kept: of the nodes of one target of one step
    /// made since the file was written, the [`KEPT_RECORDS`] made last.
    pub(crate) fn add(&mut self, made: Made) {
        let same = |m: &Made| m.step == made.step && m.target == made.target;
        if self.made.iter().filter(|m| same(m)).count() >= KEPT_RECORDS
            && let Some(at) = self.made.iter().position(same)
        {
            self.made.remove(at);
        }
        self.made.push(made);
        self.changed = true;
    }

    /// Notes the steps of the program, as its last run had them.
    pub(crate) fn order(&mut self, order: &[String]) {
        let order: BTreeSet<String> = order.iter().cloned().collect();
        if self.order.as_ref() != Some(&order) {
            self.changed = true;
            self.order = Some(order);
        }
    }

    /// The cache file to write, if anything changed since it was read or
    /// written: what was made since written into its records, of the steps
    /// the program still has, the [`KEPT_RECORDS`] of each target used last,
    /// and of the pool only what those use. A value that cannot be written
    /// — an enclosure gone infinite — is only not kept.
    pub(crate) fn write(&mut self, build: &str) -> Option<String> {
        if !std::mem::take(&mut self.changed) {
            return None;
        }
        for made in std::mem::take(&mut self.made) {
            let key = (made.step.clone(), made.target.clone());
            let records = self.records.entry(key).or_default();
            if let Some(at) = records
                .iter()
                .position(|r| r.is(&made.derivation, made.digest))
            {
                let record = records.remove(at);
                records.push(record);
                continue;
            }
            self.pool.take_used();
            let Ok(text) = (made.write)(&mut self.pool) else {
                continue;
            };
            let Ok(value) = RawValue::from_string(text) else {
                continue;
            };
            records.push(Rc::new(Record {
                step: made.step,
                target: made.target,
                kind: made.kind.to_string(),
                derivation: (*made.derivation).clone(),
                digest: made.digest,
                value,
                uses: self.pool.take_used(),
            }));
        }
        if let Some(order) = &self.order {
            // What was made of the program as a whole is of no step.
            (self.records).retain(|(step, _), _| step.is_empty() || order.contains(step));
        }
        for records in self.records.values_mut() {
            let excess = records.len().saturating_sub(KEPT_RECORDS);
            records.drain(..excess);
        }
        let used: BTreeSet<&String> = (self.records.values().flatten())
            .flat_map(|record| &record.uses)
            .collect();
        let used: BTreeSet<String> = used.into_iter().cloned().collect();
        self.pool.values.retain(|key, _| used.contains(key));
        let file = FileRef {
            format: FORMAT,
            build,
            records: self.records.values().flatten().map(|r| &**r).collect(),
            pool: &self.pool.values,
        };
        serde_json::to_string(&file).ok()
    }
}

/// [`File`], borrowed, to be written.
#[derive(Serialize)]
struct FileRef<'a> {
    format: u32,
    build: &'a str,
    records: Vec<&'a Record>,
    pool: &'a BTreeMap<String, Box<RawValue>>,
}
