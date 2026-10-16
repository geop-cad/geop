//! Placed parts: the part another program builds, placed in a part as a
//! node of its own ([`placed`]), whose value is the built part
//! ([`PlacedPart`]), made by [`build_placed`].
//!
//! A placed part is built from exactly its program, its inputs and the
//! slice of the placing part's state under its name — the state
//! `/bolt:b.pose` of the part is the state `b.pose` of the part placed as
//! `bolt` ([`key_behind`]). Equal builds are shared through the runner's build cache: a
//! thousand screws of one size are one build. Where the part goes is no
//! concern of the build: the pose that places it is the placing part's
//! state, read by whatever draws or mates it.
//!
//! An element of a placed part is named by the instance's path before its
//! target's name — `/bolt:model`, or `/asm/bolt:model` for a part placed in
//! a placed part, and `/:model` for one of the part itself ([`Located`]) —
//! and read through the placing
//! part's reader ([`crate::reader::Reader::retrieve_placed`],
//! [`crate::reader::Reader::placed_part`]), which records it as read inside
//! the placed part: a node reading the second link of an arm is made again
//! when that link changes, not when anything in the arm does.

use std::{
    collections::HashMap,
    hash::{Hash, Hasher},
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    part::{Part, StepContext},
    program::Program,
    reader::Reader,
    target::{Read, Target, TargetReference},
    values::{Settings, StateValues},
};

/// What separates the names of a path of instances, and begins a located
/// name: `/asm/bolt:model`.
pub const INSTANCE_SEPARATOR: char = '/';

/// What separates a located name's instance path from the name in that
/// instance: `/asm/bolt:model`. A name is split at the first one after the
/// path, so the name itself may hold more: `/asm:placed:bolt`.
pub const TARGET_SEPARATOR: char = ':';

/// The path of the instance `name` of the instance at `outer`: `asm/bolt`
/// — `name` itself, for none. A path is held without the leading
/// [`INSTANCE_SEPARATOR`] its located names are spelled with.
pub fn instance_path(outer: &str, name: &str) -> String {
    match outer.is_empty() {
        true => name.to_string(),
        false => format!("{outer}{INSTANCE_SEPARATOR}{name}"),
    }
}

/// The path of the instance the instance at `path` is placed in, and its
/// name there: `("asm", "bolt")` for `asm/bolt`, an empty path for an
/// instance of the part itself — [`instance_path`] undone.
pub fn split_instance_path(path: &str) -> (&str, &str) {
    path.rsplit_once(INSTANCE_SEPARATOR).unwrap_or(("", path))
}

/// A name behind the path of the instance it is in: a target of a part
/// placed in the part — however deep — or of the part itself, its path
/// empty. Spelled `/asm/bolt:model`, and `/:model` for the part itself
/// ([`Located::parse`], its `Display`), and saved and sent to a front end
/// so.
///
/// The tables of one part — its state, its inputs — name what is its own
/// bare, `b.pose`, and what is of a part placed in it located,
/// `/bolt:b.pose` ([`Located::key`], [`key_behind`]).
#[derive(Clone, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Located {
    path: String,
    name: String,
}

impl Located {
    /// The name `name` of the part itself.
    pub fn new(name: impl Into<String>) -> Self {
        Self::in_instance("", name)
    }

    /// The name `name` of the instance at `path`: `asm/bolt`, empty for the
    /// part itself.
    pub fn in_instance(path: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            name: name.into(),
        }
    }

    /// The path of the instance it is in, without the leading separator:
    /// `asm/bolt`, empty for the part itself.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// The name in that instance.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The same name as its instance's own part names it.
    pub fn local(&self) -> Self {
        Self::new(self.name.clone())
    }

    /// The same name of the instance `instance` of the part — of the
    /// instance at that path, if it is already of one: what a name of a
    /// placed part is, read from the part it is placed in.
    pub fn behind(&self, instance: &str) -> Self {
        let path = match self.path.is_empty() {
            true => instance.to_string(),
            false => instance_path(instance, &self.path),
        };
        Self::in_instance(path, self.name.clone())
    }

    /// The name `text` spells: `/asm/bolt:model`, or `/:model` for the
    /// part itself. Anything else is an error, a bare `model` too: a name
    /// that does not say where it is would be read in the wrong part.
    pub fn parse(text: &str) -> GeopResult<Self> {
        let error = |why: &str| {
            GeopError::new(format!(
                "{text:?} names nothing: {why}; a name is spelled \
                 `/asm/bolt:model`, or `/:model` for the part itself"
            ))
        };
        let rest = (text.strip_prefix(INSTANCE_SEPARATOR))
            .ok_or_else(|| error("it does not begin with `/`"))?;
        let (path, name) = (rest.split_once(TARGET_SEPARATOR))
            .ok_or_else(|| error("it has no `:` before the name"))?;
        if name.is_empty() {
            return Err(error("the name is empty"));
        }
        if !path.is_empty() && path.split(INSTANCE_SEPARATOR).any(str::is_empty) {
            return Err(error("an instance of its path is empty"));
        }
        Ok(Self::in_instance(path, name))
    }

    /// It as a key of a part's own table: its bare name for one of the
    /// part itself, else as it is spelled.
    pub fn key(&self) -> String {
        match self.path.is_empty() {
            true => self.name.clone(),
            false => self.to_string(),
        }
    }

    /// The name a key of a part's own table names ([`Located::key`]).
    pub fn of_key(key: &str) -> GeopResult<Self> {
        match key.starts_with(INSTANCE_SEPARATOR) {
            true => Self::parse(key),
            false => Ok(Self::new(key)),
        }
    }
}

impl std::fmt::Display for Located {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let (path, name) = (&self.path, &self.name);
        write!(f, "{INSTANCE_SEPARATOR}{path}{TARGET_SEPARATOR}{name}")
    }
}

impl std::fmt::Debug for Located {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:?}", self.to_string())
    }
}

impl std::str::FromStr for Located {
    type Err = GeopError;

    fn from_str(text: &str) -> GeopResult<Self> {
        Self::parse(text)
    }
}

impl serde::Serialize for Located {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

impl<'de> serde::Deserialize<'de> for Located {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = String::deserialize(deserializer)?;
        Self::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// The key `key` of the part placed as `instance` — its own, `b.pose`, or
/// one of a part placed in it, `/bolt:b.pose` — as a key of the part it is
/// placed in: `/asm:b.pose`, `/asm/bolt:b.pose` (see [`Located::key`]).
pub fn key_behind(instance: &str, key: &str) -> String {
    match key.starts_with(INSTANCE_SEPARATOR) {
        true => format!("{INSTANCE_SEPARATOR}{instance}{key}"),
        false => format!("{INSTANCE_SEPARATOR}{instance}{TARGET_SEPARATOR}{key}"),
    }
}

/// The key of the part placed as `instance` that `key`, a key of the part
/// it is placed in, is, if it is one of that part: [`key_behind`] undone.
pub fn key_inside<'k>(instance: &str, key: &'k str) -> Option<&'k str> {
    let rest = (key.strip_prefix(INSTANCE_SEPARATOR))?.strip_prefix(instance)?;
    match rest.strip_prefix(TARGET_SEPARATOR) {
        Some(own) => Some(own),
        None => rest.starts_with(INSTANCE_SEPARATOR).then_some(rest),
    }
}

/// The outermost instance a key of a part's own table is of, and the key
/// as that instance's part names it: none for a key of the part's own.
pub fn outermost_instance(key: &str) -> Option<(&str, &str)> {
    let rest = key.strip_prefix(INSTANCE_SEPARATOR)?;
    let end = rest.find([INSTANCE_SEPARATOR, TARGET_SEPARATOR])?;
    let instance = rest.get(..end)?;
    Some((instance, key_inside(instance, key)?))
}

/// Checks that `name` can name an instance: not empty, and neither a path
/// nor a located name.
fn check_name(name: &str) -> GeopResult<()> {
    match name.is_empty() || name.contains([INSTANCE_SEPARATOR, TARGET_SEPARATOR]) {
        true => Err(GeopError::new(format!(
            "{name:?} cannot name an instance: it is empty or contains \
             {INSTANCE_SEPARATOR:?} or {TARGET_SEPARATOR:?}"
        ))),
        false => Ok(()),
    }
}

/// An element of a part or of a part placed in it, as it is in its own
/// part ([`Reader::retrieve_placed`]): where that part is placed, the
/// step that placed it says.
#[derive(Clone, Debug, PartialEq)]
pub struct InInstance<E> {
    pub element: E,
    /// The path of the instance it is of: empty for the part itself.
    pub instance: String,
}

/// What the target of a placed part is named, before the instance's name.
pub(crate) const PLACED: &str = "placed:";

/// The target of the part placed as `instance`.
pub fn placed(instance: &str) -> TargetReference<PlacedPart> {
    TargetReference::new(format!("{PLACED}{instance}"))
}

/// A placed part, as built: shared by every instance built alike.
#[derive(Clone)]
pub struct PlacedPart {
    pub part: Rc<Part>,
}

/// The same build, or builds holding the same.
impl PartialEq for PlacedPart {
    fn eq(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.part, &other.part) || self.part.same_as(&other.part)
    }
}

impl Target for PlacedPart {}

/// What a part is placed from: the program file, as the placing program
/// names it, and what its parameters are set to.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlacedArgs {
    pub file: String,
    pub inputs: Settings,
}

/// Places the part the program in `args.file` builds as `instance`, its
/// parameters set to `args.inputs` ([`build_placed`]). Fails if a step of
/// the part placed fails — after placing it as far as it builds.
pub fn place(ctx: &mut StepContext, instance: &str, args: &PlacedArgs) -> GeopResult<()> {
    let ctx_error = |e: GeopError| e.with_context(format!("placing `{instance}`"));
    ctx.produce(&placed(instance), args, build_placed)
        .map_err(ctx_error)?;
    let reader = ctx.reader();
    let part = reader.part().placed_at(instance, reader.position(), None)?;
    match part.failures().first() {
        Some((step, e)) => Err(ctx_error(
            GeopError::new(e.clone()).with_context(format!("step `{step}` of {:?}", args.file)),
        )),
        None => Ok(()),
    }
}

/// The make of a placed part (see [`place`]): the program in the file
/// `args.file` names, run with `args.inputs` and the state the placing part
/// gives under the instance's name, the program's own where it gives none.
/// Taken from the runner's build cache where an equal build is there; else
/// built from the part the instance had before, if it had one, so that
/// only what reads what changed is made again. Records the file and every
/// state and file the part placed read.
pub fn build_placed(reader: &Reader, args: &PlacedArgs) -> GeopResult<PlacedPart> {
    let own = reader.own().unwrap_or_default();
    let instance = own.strip_prefix(PLACED).ok_or_else(|| {
        GeopError::new(format!(
            "target `{own}` is no placed part's: name it by `placed`"
        ))
    })?;
    check_name(instance)?;
    let runner = reader.runner()?;
    let (program, location) = reader.program(&args.file)?;
    let given = &reader.part().state;
    // The values the part gives under the instance's name, each with its
    // name in the placed part: `/bolt:b.pose` is its `b.pose`.
    let prefix = format!("{INSTANCE_SEPARATOR}{instance}");
    let slice = || {
        (given.range(prefix.clone()..))
            .take_while(|(name, _)| name.starts_with(&prefix))
            .filter_map(|(name, value)| Some((name, key_inside(instance, name)?, value)))
    };
    let mut state = program.state.clone();
    state.extend(slice().map(|(_, inside, value)| (inside.to_string(), value.clone())));
    let key = BuildKey {
        revision: runner.revision(&location.file),
        path: location.file.clone(),
        program: program.clone(),
        inputs: args.inputs.clone(),
        state,
    };
    let part = match runner.cached(&key) {
        Some(part) => part,
        None => {
            let last = (reader.last())
                .and_then(|node| node.value.as_ref().ok())
                .and_then(|value| value.downcast_ref::<PlacedPart>());
            let mut part = last.map(|last| (*last.part).clone()).unwrap_or_default();
            let nested = runner.nested(location);
            nested.build(&mut part, &program.steps, &key.inputs, &key.state, None);
            let part = Rc::new(part);
            runner.cache(key, part.clone());
            part
        }
    };
    // The slice it is given, and what it read besides: a value of the slice
    // that changes or goes, or one it read that comes, builds it again, so
    // its state is always the one it is placed with.
    for (name, _, value) in slice() {
        reader.record_raw(Read::State(name.clone(), Some(value.clone())));
    }
    for name in &part.states_read {
        let name = key_behind(instance, name);
        let value = given.get(&name).cloned();
        reader.record_raw(Read::State(name, value));
    }
    for (path, revision) in &part.files_read {
        reader.record_raw(Read::File(path.clone(), *revision));
    }
    Ok(PlacedPart { part })
}

/// What a placed part is built from, exactly: the program — its file, that
/// file's revision, and the program itself, the same `Rc` while the file
/// does not change — its inputs and its state.
#[derive(Clone, Debug)]
pub(crate) struct BuildKey {
    path: String,
    revision: u64,
    program: Rc<Program>,
    inputs: Settings,
    state: StateValues,
}

/// The same file in the same revision, the same program — the very one, or
/// one equal to it — and equal inputs and state.
impl PartialEq for BuildKey {
    fn eq(&self, other: &Self) -> bool {
        self.path == other.path
            && self.revision == other.revision
            && (Rc::ptr_eq(&self.program, &other.program) || self.program == other.program)
            && self.inputs == other.inputs
            && self.state == other.state
    }
}

impl BuildKey {
    /// A hash of it: the same for equal keys.
    fn hash(&self) -> u64 {
        let mut hasher = std::hash::DefaultHasher::new();
        self.path.hash(&mut hasher);
        self.revision.hash(&mut hasher);
        // Numbers hash by their text: equal values give equal text.
        serde_json::to_string(&self.inputs)
            .unwrap_or_default()
            .hash(&mut hasher);
        serde_json::to_string(&self.state)
            .unwrap_or_default()
            .hash(&mut hasher);
        hasher.finish()
    }
}

/// The parts placed so far, by what they were built from: what equal
/// builds share. Swept at the end of every run of a part that places
/// none of its own, of what no part holds any more.
#[derive(Default)]
pub(crate) struct BuildCache(HashMap<u64, Vec<(BuildKey, Rc<Part>)>>);

impl BuildCache {
    /// The part built from `key`, if one is, and every file it read is as
    /// it read it — `revision` saying how each is now.
    pub(crate) fn get(&self, key: &BuildKey, revision: impl Fn(&str) -> u64) -> Option<Rc<Part>> {
        let builds = self.0.get(&key.hash())?;
        (builds.iter())
            .find(|(built, part)| {
                built == key && (part.files_read.iter()).all(|(path, read)| revision(path) == *read)
            })
            .map(|(_, part)| part.clone())
    }

    pub(crate) fn insert(&mut self, key: BuildKey, part: Rc<Part>) {
        let builds = self.0.entry(key.hash()).or_default();
        builds.retain(|(built, _)| *built != key);
        builds.push((key, part));
    }

    /// Drops the builds no part holds: only the cache does. A build held
    /// only by a build dropped goes too.
    pub(crate) fn sweep(&mut self) {
        loop {
            let mut dropped = false;
            self.0.retain(|_, builds| {
                let before = builds.len();
                builds.retain(|(_, part)| Rc::strong_count(part) > 1);
                dropped |= builds.len() < before;
                !builds.is_empty()
            });
            if !dropped {
                break;
            }
        }
    }

    /// How many builds it holds.
    pub(crate) fn len(&self) -> usize {
        self.0.values().map(Vec::len).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A located name reads back as it is written: of the part itself, and
    /// behind a path of instances.
    #[test]
    fn located_names_read_back_as_written() {
        for text in ["/:model", "/a/b:model", "/bolt:b.pose"] {
            let located = Located::parse(text).unwrap();
            assert_eq!(located.to_string(), text);
            let read: Located = serde_json::from_value(serde_json::json!(text)).unwrap();
            assert_eq!(read, located);
            assert_eq!(
                serde_json::to_value(&located).unwrap(),
                serde_json::json!(text)
            );
        }
        let own = Located::parse("/:model").unwrap();
        assert_eq!((own.path(), own.name()), ("", "model"));
        let deep = Located::parse("/a/b:model").unwrap();
        assert_eq!((deep.path(), deep.name()), ("a/b", "model"));
        assert_eq!(deep, own.behind("b").behind("a"));
        assert_eq!(deep.local(), own);
    }

    /// A name is split at the first `:` after the path: what follows is the
    /// name, `:` and all.
    #[test]
    fn a_located_name_may_hold_a_colon() {
        let located = Located::parse("/a:b:c").unwrap();
        assert_eq!((located.path(), located.name()), ("a", "b:c"));
        assert_eq!(located.to_string(), "/a:b:c");
        let own = Located::parse("/:placed:bolt").unwrap();
        assert_eq!((own.path(), own.name()), ("", "placed:bolt"));
    }

    /// What does not say where it is names nothing.
    #[test]
    fn a_name_that_does_not_say_where_it_is_is_refused() {
        for text in [
            "model",
            "a/b:model",
            "/model",
            "/a:",
            "/a//b:model",
            "/a/:model",
            "",
        ] {
            assert!(Located::parse(text).is_err(), "{text:?}");
            assert!(serde_json::from_value::<Located>(serde_json::json!(text)).is_err());
        }
    }

    /// A key of a part's own table is bare for its own, located for a
    /// placed part's, and moves behind and back out of an instance.
    #[test]
    fn keys_move_behind_an_instance_and_back() {
        assert_eq!(Located::new("b.pose").key(), "b.pose");
        assert_eq!(Located::of_key("b.pose").unwrap(), Located::new("b.pose"));
        assert_eq!(key_behind("bolt", "b.pose"), "/bolt:b.pose");
        assert_eq!(key_behind("asm", "/bolt:b.pose"), "/asm/bolt:b.pose");
        assert_eq!(key_inside("asm", "/asm/bolt:b.pose"), Some("/bolt:b.pose"));
        assert_eq!(key_inside("bolt", "/bolt:b.pose"), Some("b.pose"));
        assert_eq!(key_inside("bolt", "/bolt2:b.pose"), None);
        assert_eq!(key_inside("bolt", "b.pose"), None);
        assert_eq!(
            outermost_instance("/asm/bolt:b.pose"),
            Some(("asm", "/bolt:b.pose"))
        );
        assert_eq!(outermost_instance("/bolt:x:y"), Some(("bolt", "x:y")));
        assert_eq!(outermost_instance("b.pose"), None);
        let located = Located::of_key("/asm/bolt:b.pose").unwrap();
        assert_eq!(located.key(), "/asm/bolt:b.pose");
        assert_eq!((located.path(), located.name()), ("asm/bolt", "b.pose"));
    }

    /// An instance is named by one name: no path, nothing located.
    #[test]
    fn an_instance_name_holds_no_separator() {
        assert!(check_name("bolt").is_ok());
        for name in ["", "a/b", "a:b"] {
            assert!(check_name(name).is_err(), "{name:?}");
        }
    }
}
