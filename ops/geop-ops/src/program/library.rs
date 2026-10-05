//! [`Library`]: where a program finds the parts it places — the parts other
//! program files build, by file name — and the other files its steps read,
//! as an import reads a STEP file; and [`Workspace`], the library of a set
//! of files.
//!
//! Placing a part runs the program of another file, which may place parts
//! of its own. So the files of a workspace form a graph, and it has to be
//! acyclic: a file that places itself, directly or through others, would
//! never finish building. [`Workspace`] refuses such a placement with an
//! error naming the cycle.
//!
//! Files name each other by `/`-separated paths relative to the file doing
//! the naming — `bolt.geop`, `../parts/bolt.geop` — so a folder of files
//! can be moved as a whole. [`resolve`] turns such a reference into the
//! path a workspace knows the file by, [`relative`] goes back.
//!
//! A file of a library every workspace shares — the standard parts an
//! application offers — is named by a scheme instead: `std:hex_nut.geop`
//! (see [`is_shared`]). Such a name means the same file from anywhere, so
//! it is never relative.

use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::{Program, ProgramRunner};
use crate::{
    operation::Operations,
    part::{Component, Part, State},
};

/// Where a program being built finds the parts it places.
pub trait Library<S: Scalar> {
    /// The part the program in `file` builds — `file` relative to the
    /// program being built — with its state `overrides` instead of
    /// its own: a part placed flexibly, whose parts the program placing it
    /// moves (see [`crate::part::State`]).
    fn component(&self, file: &str, overrides: &State) -> GeopResult<Arc<Component<S>>>;

    /// The files the program being built could place or read, as it would
    /// name them: every file but its own — programs (see [`is_program`])
    /// and any other.
    fn files(&self) -> Vec<String>;

    /// The file `file`, relative to the program being built — a file a
    /// step reads as data, as an import reads a STEP file: its path, as
    /// the library names files (see [`Component::files`]), and its text.
    fn read(&self, file: &str) -> GeopResult<(String, String)>;

    /// Where a step keeps what is costly to work out from data it read,
    /// under a key naming that data — an import's bodies, by the STEP
    /// file's text — so building it again, here or in another process,
    /// takes it from there. None keeps nothing.
    fn cache(&self) -> Option<&dyn Cache> {
        None
    }
}

/// What steps keep what is costly to work out in, by key (see
/// [`Library::cache`]): in memory, or on disk where a host gives one —
/// `geop serve` keeps a `.geop-cache` folder in the workspace.
///
/// A key names everything the value was worked out from, so a value kept
/// is right for as long as the key is the same; nothing is ever forgotten
/// for being stale.
pub trait Cache {
    /// The bytes kept under `key`, if any.
    fn load(&self, key: &str) -> Option<Vec<u8>>;
    /// Keeps `bytes` under `key`. Failing to keep them is no error: they
    /// are worked out again.
    fn store(&self, key: &str, bytes: &[u8]);
}

/// A [`Cache`] in memory: for as long as the process lives.
#[derive(Default)]
pub struct MemoryCache(RefCell<BTreeMap<String, Arc<Vec<u8>>>>);

impl Cache for MemoryCache {
    fn load(&self, key: &str) -> Option<Vec<u8>> {
        self.0.borrow().get(key).map(|bytes| bytes.as_ref().clone())
    }

    fn store(&self, key: &str, bytes: &[u8]) {
        self.0
            .borrow_mut()
            .insert(key.to_string(), Arc::new(bytes.to_vec()));
    }
}

/// Whether the file `path` is a program, which a part can be placed from:
/// a `.geop` file.
pub fn is_program(path: &str) -> bool {
    path.ends_with(".geop")
}

/// The library of a program that places no parts: it has no files.
pub struct NoFiles;

impl<S: Scalar> Library<S> for NoFiles {
    fn component(&self, file: &str, _: &State) -> GeopResult<Arc<Component<S>>> {
        Err(GeopError::new(format!(
            "cannot place {file:?}: this program is built without any other files"
        )))
    }

    fn files(&self) -> Vec<String> {
        Vec::new()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        Err(GeopError::new(format!(
            "cannot read {file:?}: this program is built without any other files"
        )))
    }
}

/// Where a [`Workspace`] reads files from.
pub trait Files {
    /// The text of the file `path`.
    fn read(&self, path: &str) -> GeopResult<String>;
    /// Every file there is, by path: programs and the files they read.
    fn list(&self) -> Vec<String>;
}

/// [`Files`] a [`Workspace`] can change ([`Workspace::write`]).
pub trait FilesMut: Files {
    /// Sets the file `path` to `text`, or — `None` — removes it.
    fn write(&mut self, path: &str, text: Option<String>);
}

/// Files held in memory, by path: what an editor is sent by its front end.
impl Files for BTreeMap<String, String> {
    fn read(&self, path: &str) -> GeopResult<String> {
        self.get(path)
            .cloned()
            .ok_or_else(|| GeopError::new(format!("there is no file {path:?}")))
    }

    fn list(&self) -> Vec<String> {
        self.keys().cloned().collect()
    }
}

impl FilesMut for BTreeMap<String, String> {
    fn write(&mut self, path: &str, text: Option<String>) {
        match text {
            Some(text) => self.insert(path.to_string(), text),
            None => self.remove(path),
        };
    }
}

/// A build of a file with state overridden: which overrides, as their
/// JSON, and what it built.
type Variant<S> = (String, Arc<Component<S>>);

/// The library of a set of files, whose programs are written in the
/// operations `O`: [`Workspace::scope`] is the library a program of one of
/// them is built with.
///
/// Every file it builds is kept, and placed again as it is for as long as
/// the files it was built from stay as they are — however many steps or
/// rebuilds place it. A file built with state of its own overridden —
/// placed flexibly — is built incrementally, by a runner of its own:
/// moving one of its parts runs again only the steps that read where it
/// is, which for a file that only places parts is placing them.
///
/// Changing a file ([`Workspace::write`]) forgets only what was built from
/// it: the file itself, and every file placing it, however deep — each
/// component knows the files it was built from ([`Component::files`]). A
/// runner forgets only the steps from the first that placed it (see
/// [`ProgramRunner::forget`]). A file written as it already was changes
/// nothing.
pub struct Workspace<O, S: Scalar, F = BTreeMap<String, String>> {
    files: F,
    built: RefCell<BTreeMap<String, Arc<Component<S>>>>,
    /// Per file: the last build of it with state overridden, and
    /// which overrides, as their JSON.
    variants: RefCell<BTreeMap<String, Variant<S>>>,
    /// Per file: the runner building it with state overridden.
    runners: RefCell<BTreeMap<String, ProgramRunner<S, O>>>,
    /// Where the steps of its programs keep what is costly (see
    /// [`Library::cache`]): in memory unless a host gives another.
    cache: Box<dyn Cache>,
}

impl<O: Operations, S: Scalar, F: Files> Workspace<O, S, F> {
    pub fn new(files: F) -> Self {
        Self {
            files,
            built: RefCell::new(BTreeMap::new()),
            variants: RefCell::new(BTreeMap::new()),
            runners: RefCell::new(BTreeMap::new()),
            cache: Box::new(MemoryCache::default()),
        }
    }

    /// Keeps what steps work out in `cache` from now on — on disk, say,
    /// where it outlives the process (see [`Cache`]).
    pub fn set_cache(&mut self, cache: Box<dyn Cache>) {
        self.cache = cache;
    }

    pub fn files(&self) -> &F {
        &self.files
    }

    /// Forgets what was built from any of the files `changed` — paths as
    /// [`resolve`] gives them: the parts that read one, and the steps of
    /// every runner from the first that read one (see
    /// [`ProgramRunner::forget`]).
    fn forget(&mut self, changed: &BTreeSet<String>) {
        let reads = |component: &Component<S>| !component.files.is_disjoint(changed);
        self.built.get_mut().retain(|_, c| !reads(c));
        self.variants.get_mut().retain(|_, (_, c)| !reads(c));
        for runner in self.runners.get_mut().values_mut() {
            runner.forget(changed);
        }
    }

    /// Keeps `part` as what the file `path` builds, as an editor built it
    /// — from the files `read` — so placing that file elsewhere takes it
    /// rather than building the program again. Only while the files stay
    /// as they are: writing one of them forgets it, as it forgets what was
    /// built here. A build kept already stays, so it is drawn as before.
    pub fn keep(&mut self, path: &str, part: Part<S>, mut read: BTreeSet<String>) {
        let path = resolve("", path);
        read.insert(path.clone());
        self.built
            .get_mut()
            .entry(path.clone())
            .or_insert_with(|| Arc::new(Component::new(path, part, read)));
    }

    /// The library the program of the file `file` is built with.
    pub fn scope(&self, file: &str) -> Scope<'_, O, S, F> {
        let file = resolve("", file);
        Scope {
            workspace: self,
            building: vec![file.clone()],
            file,
            placed: RefCell::new(BTreeSet::new()),
        }
    }
}

impl<O: Operations, S: Scalar, F: FilesMut> Workspace<O, S, F> {
    /// Sets the file `path` to `text`, or — `None` — removes it, and
    /// forgets what was built from it (see [`Workspace`]). Says whether it
    /// changed anything: written as it was, nothing is forgotten.
    pub fn write(&mut self, path: &str, text: Option<String>) -> bool {
        if self.files.read(path).ok() == text {
            return false;
        }
        self.files.write(path, text);
        self.forget(&BTreeSet::from([resolve("", path)]));
        true
    }
}

/// The library one program of a [`Workspace`] is built with: the file it
/// is in, which the files it places are named relative to, and the files
/// being built around it, which it must not place.
pub struct Scope<'w, O, S: Scalar, F> {
    workspace: &'w Workspace<O, S, F>,
    file: String,
    /// The files being built, outermost first: the one that placed the
    /// next, down to `file`.
    building: Vec<String>,
    /// Every file the parts it placed were built from.
    placed: RefCell<BTreeSet<String>>,
}

impl<O: Operations, S: Scalar, F: Files> Scope<'_, O, S, F> {
    /// The error for placing `path`, which `cycle` — the files from the one
    /// it would place again — already places.
    fn cycle(&self, path: &str, from: usize) -> GeopError {
        let chain: Vec<&str> = self.building[from..]
            .iter()
            .map(String::as_str)
            .chain([path])
            .collect();
        GeopError::new(format!(
            "{} places {path:?}, which places it back: {} — files must not place themselves, directly or through others",
            self.file,
            chain.join(" -> "),
        ))
    }

    /// The library the program of `path` is built with, from here.
    fn child(&self, path: &str) -> GeopResult<Scope<'_, O, S, F>> {
        if let Some(from) = self.building.iter().position(|f| f == path) {
            return Err(self.cycle(path, from));
        }
        Ok(Scope {
            workspace: self.workspace,
            file: path.to_string(),
            building: [self.building.clone(), vec![path.to_string()]].concat(),
            placed: RefCell::new(BTreeSet::new()),
        })
    }

    /// `path` built with its state `overrides` — which change some —
    /// by its runner, from where the last such build left off.
    fn rebuild(&self, path: &str, overrides: &State) -> GeopResult<Arc<Component<S>>> {
        let key = serde_json::to_string(overrides)
            .map_err(|e| GeopError::new(format!("writing state: {e}")))?;
        if let Some((built_with, component)) = self.workspace.variants.borrow().get(path)
            && *built_with == key
        {
            return Ok(component.clone());
        }
        let child = self.child(path)?;
        let mut program = Program::<O>::from_json(&self.workspace.files.read(path)?)?;
        program.state.extend(overrides.clone());
        // Out of the map while it runs: what it builds may rebuild other
        // files with runners of their own.
        let mut runner = self
            .workspace
            .runners
            .borrow_mut()
            .remove(path)
            .unwrap_or_default();
        runner.run(&program, None, &child);
        let failed = runner.results().iter().find_map(|r| r.error.clone());
        let part = runner.part().clone();
        self.workspace
            .runners
            .borrow_mut()
            .insert(path.to_string(), runner);
        if let Some(error) = failed {
            return Err(GeopError::new(error).with_context(format!("building {path}")));
        }
        let mut files = child.placed.into_inner();
        files.insert(path.to_string());
        let component = Arc::new(Component::new(path.to_string(), part, files));
        self.workspace
            .variants
            .borrow_mut()
            .insert(path.to_string(), (key, component.clone()));
        Ok(component)
    }

    fn build(&self, path: &str) -> GeopResult<Arc<Component<S>>> {
        let child = self.child(path)?;
        let text = self.workspace.files.read(path)?;
        let part = Program::<O>::from_json(&text)
            .and_then(|program| program.build(&child))
            .map_err(|e| e.with_context(format!("building {path}")))?;
        let mut files = child.placed.into_inner();
        files.insert(path.to_string());
        let component = Arc::new(Component::new(path.to_string(), part, files));
        self.workspace
            .built
            .borrow_mut()
            .insert(path.to_string(), component.clone());
        Ok(component)
    }
}

impl<O: Operations, S: Scalar, F: Files> Library<S> for Scope<'_, O, S, F> {
    fn component(&self, file: &str, overrides: &State) -> GeopResult<Arc<Component<S>>> {
        let path = resolve(&self.file, file);
        let built = self.workspace.built.borrow().get(&path).cloned();
        let mut component = match built {
            Some(component) => component,
            None => self.build(&path)?,
        };
        // Overriding a parameter with the value it was built with changes
        // nothing.
        let own = component.part.inputs();
        let changed: State = overrides
            .iter()
            .filter(|(name, value)| own.get(*name) != Some(value))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        if !changed.is_empty() {
            component = self.rebuild(&path, overrides)?;
        }
        // Built before, for another file, it may place one of those being
        // built here.
        if let Some(from) = self
            .building
            .iter()
            .position(|f| component.files.contains(f))
        {
            return Err(self.cycle(&path, from));
        }
        self.placed
            .borrow_mut()
            .extend(component.files.iter().cloned());
        Ok(component)
    }

    fn files(&self) -> Vec<String> {
        self.workspace
            .files
            .list()
            .into_iter()
            .map(|f| resolve("", &f))
            .filter(|f| *f != self.file)
            .map(|f| relative(&self.file, &f))
            .collect()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        let path = resolve(&self.file, file);
        let text = self.workspace.files.read(&path)?;
        Ok((path, text))
    }

    fn cache(&self) -> Option<&dyn Cache> {
        Some(self.workspace.cache.as_ref())
    }
}

/// Whether `reference` names a file of a library every workspace shares,
/// by a scheme of two letters or more and `:` — `std:hex_nut.geop`. It
/// names the same file from anywhere: it is never relative. (A single
/// letter before the `:` is a drive, `C:/parts/bolt.geop`.)
pub fn is_shared(reference: &str) -> bool {
    reference.split_once(':').is_some_and(|(scheme, _)| {
        scheme.len() >= 2 && scheme.chars().all(|c| c.is_ascii_alphabetic())
    })
}

/// The path of the file `reference` names, from the file `from`: relative
/// to the folder `from` is in, unless it starts with `/` or is a shared
/// library's (see [`is_shared`]). `.` and `..` are taken out where they
/// can be, and `\` read as `/`.
pub fn resolve(from: &str, reference: &str) -> String {
    if is_shared(reference) {
        return reference.to_string();
    }
    let reference = reference.replace('\\', "/");
    let from = from.replace('\\', "/");
    let joined = match (reference.starts_with('/'), from.rsplit_once('/')) {
        (true, _) | (false, None) => reference,
        (false, Some((folder, _))) => format!("{folder}/{reference}"),
    };
    let absolute = joined.starts_with('/');
    let mut parts: Vec<&str> = Vec::new();
    for part in joined.split('/') {
        match part {
            "" | "." => {}
            ".." if parts.last().is_some_and(|p| *p != "..") => {
                parts.pop();
            }
            ".." if absolute => {}
            part => parts.push(part),
        }
    }
    let path = parts.join("/");
    if absolute { format!("/{path}") } else { path }
}

/// How the file `from` names the file `to`, both paths as [`resolve`]
/// gives them: relative to the folder `from` is in — a shared library's
/// file by its own name (see [`is_shared`]).
pub fn relative(from: &str, to: &str) -> String {
    if is_shared(to) {
        return to.to_string();
    }
    let folders: Vec<&str> = from.split('/').collect();
    let folders = &folders[..folders.len() - 1];
    let target: Vec<&str> = to.split('/').collect();
    let common = folders
        .iter()
        .zip(&target)
        .take_while(|(a, b)| a == b)
        .count()
        .min(target.len() - 1);
    let up = std::iter::repeat_n("..", folders.len() - common);
    up.chain(target[common..].iter().copied())
        .collect::<Vec<_>>()
        .join("/")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn references_resolve_relative_to_their_file() {
        assert_eq!(resolve("asm.geop", "bolt.geop"), "bolt.geop");
        assert_eq!(resolve("a/asm.geop", "bolt.geop"), "a/bolt.geop");
        assert_eq!(resolve("a/asm.geop", "../b/bolt.geop"), "b/bolt.geop");
        assert_eq!(resolve("a/asm.geop", "./x/../bolt.geop"), "a/bolt.geop");
        assert_eq!(
            resolve("/home/a/asm.geop", "bolt.geop"),
            "/home/a/bolt.geop"
        );
        assert_eq!(resolve("a/asm.geop", "/b/bolt.geop"), "/b/bolt.geop");
        assert_eq!(resolve("asm.geop", "../bolt.geop"), "../bolt.geop");
        assert_eq!(resolve("a/b/asm.geop", "std:nut.geop"), "std:nut.geop");
        assert_eq!(resolve("asm.geop", "C:/b/bolt.geop"), "C:/b/bolt.geop");
    }

    #[test]
    fn shared_files_are_named_alike_from_anywhere() {
        assert!(is_shared("std:nut.geop"));
        assert!(!is_shared("C:/parts/nut.geop"));
        assert!(!is_shared("parts/nut.geop"));
        assert_eq!(relative("a/b/asm.geop", "std:nut.geop"), "std:nut.geop");
    }

    #[test]
    fn relative_names_resolve_back() {
        for (from, to) in [
            ("asm.geop", "bolt.geop"),
            ("a/asm.geop", "a/bolt.geop"),
            ("a/asm.geop", "b/c/bolt.geop"),
            ("a/b/asm.geop", "bolt.geop"),
        ] {
            let name = relative(from, to);
            assert_eq!(resolve(from, &name), to, "{from} -> {to} named {name}");
        }
        assert_eq!(relative("a/asm.geop", "b/bolt.geop"), "../b/bolt.geop");
    }
}
