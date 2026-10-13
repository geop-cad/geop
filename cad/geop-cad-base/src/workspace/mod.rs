//! [`Workspace`]: the [`Library`] of a set of program files — the parts
//! they build, by file name, and the other files their steps read.
//!
//! Placing a part runs the program of another file, which may place parts
//! of its own. So the files of a workspace form a graph, and it has to be
//! acyclic: a file that places itself, directly or through others, would
//! never finish building. A [`Workspace`] refuses such a placement with an
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
    cell::{Cell, RefCell},
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_ops_assembly::{Component, Library};
use geop_ops_parameters::{Files, ProgramFile, Registry, Runner, Settings, StateValues};

use crate::part;

/// Whether the file `path` is a program, which a part can be placed from:
/// a `.geop` file.
pub fn is_program(path: &str) -> bool {
    path.ends_with(".geop")
}

/// Where a [`Workspace`] reads files from.
pub trait FileStore {
    /// The text of the file `path`.
    fn read(&self, path: &str) -> GeopResult<String>;
    /// Every file there is, by path: programs and the files they read.
    fn list(&self) -> Vec<String>;
}

/// [`FileStore`] a [`Workspace`] can change ([`Workspace::write`]).
pub trait FileStoreMut: FileStore {
    /// Sets the file `path` to `text`, or — `None` — removes it.
    fn write(&mut self, path: &str, text: Option<String>);
}

/// Files held in memory, by path: what an editor is sent by its front end.
impl FileStore for BTreeMap<String, String> {
    fn read(&self, path: &str) -> GeopResult<String> {
        self.get(path)
            .cloned()
            .ok_or_else(|| GeopError::new(format!("there is no file {path:?}")))
    }

    fn list(&self) -> Vec<String> {
        self.keys().cloned().collect()
    }
}

impl FileStoreMut for BTreeMap<String, String> {
    fn write(&mut self, path: &str, text: Option<String>) {
        match text {
            Some(text) => self.insert(path.to_string(), text),
            None => self.remove(path),
        };
    }
}

/// The library of a set of files, whose programs are written in the
/// operations `registry` makes: [`Workspace::scope`] is the library a
/// program of one of them is built with.
///
/// Every file it builds is kept, by the values its parameters were built
/// with and the state it was run with, and placed again as it is for as
/// long as the files it was built from stay as they are — however many
/// steps or runs place it. One run with a state of its own given — a part
/// placed in a part, moved there — is kept while it is placed: a solve
/// tries many. Changing a
/// file ([`Workspace::write`]) forgets only what was built from it: the
/// file itself, and every file placing it, however deep — each component
/// knows the files it was built from ([`Component::files`]). A file
/// written as it already was changes nothing.
pub struct Workspace<F = BTreeMap<String, String>> {
    files: RefCell<F>,
    registry: fn() -> GeopResult<Registry>,
    /// Each file's program, read once: by its path.
    programs: RefCell<BTreeMap<String, Rc<ProgramFile>>>,
    /// What each file built, by its path, the values its parameters were
    /// built with and the state it was given, as their JSON.
    built: RefCell<BTreeMap<(String, String, String), Rc<Component>>>,
    /// How many components it has built: what tells their keys apart.
    builds: Cell<u64>,
}

impl<F: FileStore + 'static> Workspace<F> {
    pub fn new(files: F, registry: fn() -> GeopResult<Registry>) -> Rc<Self> {
        Rc::new(Self {
            files: RefCell::new(files),
            registry,
            programs: RefCell::new(BTreeMap::new()),
            built: RefCell::new(BTreeMap::new()),
            builds: Cell::new(0),
        })
    }

    /// The files, to read.
    pub fn files(&self) -> std::cell::Ref<'_, F> {
        self.files.borrow()
    }

    /// The operations its programs are written in.
    pub fn registry(&self) -> GeopResult<Registry> {
        (self.registry)()
    }

    /// The library the program of the file `file` is built with — and the
    /// files it reads.
    pub fn scope(self: &Rc<Self>, file: &str) -> Rc<dyn Library> {
        let file = resolve("", file);
        Rc::new(Scope {
            workspace: self.clone(),
            building: vec![file.clone()],
            file,
            placed: RefCell::new(BTreeSet::new()),
        })
    }

    /// The program of the file `path`, read the first time it is asked for.
    fn program(&self, path: &str) -> GeopResult<Rc<ProgramFile>> {
        if let Some(program) = self.programs.borrow().get(path) {
            return Ok(program.clone());
        }
        let text = self.files.borrow().read(path)?;
        let program =
            ProgramFile::from_json(&text).map_err(|e| e.with_context(format!("reading {path}")))?;
        let program = Rc::new(program);
        (self.programs.borrow_mut()).insert(path.to_string(), program.clone());
        Ok(program)
    }

    /// Forgets what was read from and built from any of the files
    /// `changed` — paths as [`resolve`] gives them.
    fn forget(&self, changed: &BTreeSet<String>) {
        (self.programs.borrow_mut()).retain(|path, _| !changed.contains(path));
        (self.built.borrow_mut()).retain(|_, component| component.files.is_disjoint(changed));
    }
}

impl<F: FileStoreMut + 'static> Workspace<F> {
    /// Sets the file `path` to `text`, or — `None` — removes it, and
    /// forgets what was built from it (see [`Workspace`]). Says whether it
    /// changed anything: written as it was, nothing is forgotten.
    pub fn write(&self, path: &str, text: Option<String>) -> bool {
        if self.files.borrow().read(path).ok() == text {
            return false;
        }
        self.files.borrow_mut().write(path, text);
        self.forget(&BTreeSet::from([resolve("", path)]));
        true
    }
}

/// The library one program of a [`Workspace`] is built with: the file it
/// is in, which the files it places are named relative to, and the files
/// being built around it, which it must not place.
struct Scope<F: 'static> {
    workspace: Rc<Workspace<F>>,
    file: String,
    /// The files being built, outermost first: the one that placed the
    /// next, down to `file`.
    building: Vec<String>,
    /// Every file the parts it placed were built from.
    placed: RefCell<BTreeSet<String>>,
}

impl<F: FileStore + 'static> Scope<F> {
    /// The error for placing `path`, which the files being built from
    /// `from` on already place.
    fn cycle(&self, path: &str, from: usize) -> GeopError {
        let chain: Vec<&str> = (self.building[from..].iter())
            .map(String::as_str)
            .chain([path])
            .collect();
        GeopError::new(format!(
            "{} places {path:?}, which places it back: {} — files must not place themselves, directly or through others",
            self.file,
            chain.join(" -> "),
        ))
    }

    /// The program of `path` built with its parameters `overrides` and the
    /// values `state` gives of its state instead of its own — a run of its
    /// own, whose part the component keeps.
    fn build(
        &self,
        path: &str,
        file: &ProgramFile,
        overrides: &Settings,
        state: &StateValues,
    ) -> GeopResult<Component> {
        if let Some(from) = self.building.iter().position(|f| f == path) {
            return Err(self.cycle(path, from));
        }
        let child = Rc::new(Scope {
            workspace: self.workspace.clone(),
            file: path.to_string(),
            building: [self.building.clone(), vec![path.to_string()]].concat(),
            placed: RefCell::new(BTreeSet::new()),
        });
        let mut runner = Runner::new(self.workspace.registry()?);
        runner.given = Some(part::given(file.about(path, overrides), child.clone()));
        (file.run(&mut runner, overrides, state))
            .map_err(|e| e.with_context(format!("building {path}")))?;
        let mut files = child.placed.take();
        files.insert(path.to_string());
        let builds = self.workspace.builds.get() + 1;
        self.workspace.builds.set(builds);
        let key = format!("{path}#{builds}");
        let parameters = file.parameters.clone();
        Ok(Component::new(
            path.to_string(),
            files,
            key,
            parameters,
            runner.part,
            part::draw,
        ))
    }
}

impl<F: FileStore + 'static> Library for Scope<F> {
    fn component(
        &self,
        file: &str,
        overrides: &Settings,
        state: &StateValues,
    ) -> GeopResult<Rc<Component>> {
        let path = resolve(&self.file, file);
        let program = self.workspace.program(&path)?;
        // Kept by the values it is built with: overriding a parameter with
        // the value it has anyway builds nothing new.
        let values = program.parameters.resolve(overrides).chosen;
        let json = |what: &str, value: serde_json::Result<String>| {
            value.map_err(|e| GeopError::new(format!("writing {what}: {e}")))
        };
        let key = (
            path.clone(),
            json("parameter values", serde_json::to_string(&values))?,
            json("the state", serde_json::to_string(state))?,
        );
        let cached = self.workspace.built.borrow().get(&key).cloned();
        let component = match cached {
            Some(component) => component,
            None => {
                let component = Rc::new(self.build(&path, &program, overrides, state)?);
                let mut built = self.workspace.built.borrow_mut();
                // A run given a state no longer placed is not run again.
                built.retain(|(_, _, given), kept| given == "{}" || Rc::strong_count(kept) > 1);
                built.insert(key, component.clone());
                component
            }
        };
        // Built before, for another file, it may place one of those being
        // built here.
        if let Some(from) = (self.building.iter()).position(|f| component.files.contains(f)) {
            return Err(self.cycle(&path, from));
        }
        (self.placed.borrow_mut()).extend(component.files.iter().cloned());
        Ok(component)
    }
}

impl<F: FileStore + 'static> Files for Scope<F> {
    fn files(&self) -> Vec<String> {
        (self.workspace.files.borrow().list().into_iter())
            .map(|f| resolve("", &f))
            .filter(|f| *f != self.file)
            .map(|f| relative(&self.file, &f))
            .collect()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        let path = resolve(&self.file, file);
        let text = self.workspace.files.borrow().read(&path)?;
        Ok((path, text))
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
