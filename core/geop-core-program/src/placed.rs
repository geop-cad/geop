//! Placed parts: the part another program builds, placed in a part as a
//! node of its own ([`placed`]), whose value is the built part
//! ([`PlacedPart`]), made by [`build_placed`].
//!
//! A placed part is built from exactly its program, its inputs and the
//! slice of the placing part's state under its name — the state
//! `bolt/b.pose` of the part is the state `b.pose` of the part placed as
//! `bolt`. Equal builds are shared through the runner's build cache: a
//! thousand screws of one size are one build. Where the part goes is no
//! concern of the build: the pose that places it is the placing part's
//! state, read by whatever draws or mates it.
//!
//! An element of a placed part is named by the instance's path before its
//! target's name — `bolt/model`, or `asm/bolt/model` for a part placed in a
//! placed part ([`INSTANCE_SEPARATOR`]) — and read through the placing
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
    reader::Reader,
    target::{Read, Target, TargetReference},
    values::{Settings, StateValues},
};

/// What separates the names of a path of instances, and an instance's path
/// from the name of a target in it: `asm/bolt/model`.
pub const INSTANCE_SEPARATOR: char = '/';

/// The path of the instance `name` of the instance at `outer`: `asm/bolt`
/// — `name` itself, for none.
pub fn instance_path(outer: &str, name: &str) -> String {
    match outer.is_empty() {
        true => name.to_string(),
        false => format!("{outer}{INSTANCE_SEPARATOR}{name}"),
    }
}

/// The instance path a target's name begins with, and the name of the
/// target in that instance: `("asm/bolt", "model")` for
/// `asm/bolt/model`; an empty path for a target of the part itself.
pub fn split_path(target: &str) -> (&str, &str) {
    target
        .rsplit_once(INSTANCE_SEPARATOR)
        .unwrap_or(("", target))
}

/// Checks that `name` can name an instance: not empty, and no path.
fn check_name(name: &str) -> GeopResult<()> {
    match name.is_empty() || name.contains(INSTANCE_SEPARATOR) {
        true => Err(GeopError::new(format!(
            "{name:?} cannot name an instance: it is empty or contains {INSTANCE_SEPARATOR:?}"
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
    let prefix = format!("{instance}{INSTANCE_SEPARATOR}");
    let given = &reader.part().state;
    let mut state = program.state.clone();
    state.extend(
        (given.range(prefix.clone()..))
            .take_while(|(name, _)| name.starts_with(&prefix))
            .filter_map(|(name, value)| {
                Some((name.strip_prefix(&prefix)?.to_string(), value.clone()))
            }),
    );
    let key = BuildKey {
        revision: runner.revision(&location.file),
        path: location.file.clone(),
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
    let slice = (given.range(prefix.clone()..)).take_while(|(name, _)| name.starts_with(&prefix));
    for (name, value) in slice {
        reader.record_raw(Read::State(name.clone(), Some(value.clone())));
    }
    for name in &part.states_read {
        let name = format!("{prefix}{name}");
        let value = given.get(&name).cloned();
        reader.record_raw(Read::State(name, value));
    }
    for (path, revision) in &part.files_read {
        reader.record_raw(Read::File(path.clone(), *revision));
    }
    Ok(PlacedPart { part })
}

/// What a placed part is built from, exactly: the program, by its file and
/// that file's revision, its inputs and its state.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BuildKey {
    path: String,
    revision: u64,
    inputs: Settings,
    state: StateValues,
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
