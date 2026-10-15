//! [`Workspace`]: a set of program files — the parts they build, by file
//! name, and the other files their steps read — as programs are run with it
//! (see [`geop_core_program::workspace`]).
//!
//! Placing a part runs the program of another file, which may place parts
//! of its own. So the files of a workspace form a graph, and it has to be
//! acyclic: a file that places itself, directly or through others, would
//! never finish building. The runner refuses such a placement with an
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
use geop_core_program::workspace::Loaded;
use geop_ops_parameters::{ProgramFile, Registry, StateValue};

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

/// A set of files, whose programs are written in the operations
/// `registry` makes: the [`geop_core_program::workspace::Workspace`] its
/// programs are run with, reading the programs they place and the files
/// they import from it.
///
/// Each file is read once, and read again once it changes
/// ([`Workspace::write`]): a program is the same `Rc` while its file does
/// not change, so a part placed from it again is carried over. Every part
/// placed is a part of its own, kept by the part placing it (see
/// [`geop_core_program::instance`]).
pub struct Workspace<F: 'static = BTreeMap<String, String>> {
    files: RefCell<F>,
    registry: fn() -> GeopResult<Registry>,
    /// Each file's program, read once: by its path.
    programs: RefCell<BTreeMap<String, Loaded<StateValue>>>,
    /// Each other file's text, read once: by its path.
    texts: RefCell<BTreeMap<String, Rc<str>>>,
    /// How many times a file changed ([`Workspace::write`]).
    revision: Cell<u64>,
}

impl<F: FileStore + 'static> Workspace<F> {
    pub fn new(files: F, registry: fn() -> GeopResult<Registry>) -> Rc<Self> {
        Rc::new(Self {
            files: RefCell::new(files),
            registry,
            programs: RefCell::new(BTreeMap::new()),
            texts: RefCell::new(BTreeMap::new()),
            revision: Cell::new(0),
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

    /// Forgets what was read from any of the files `changed` — paths as
    /// [`resolve`] gives them.
    fn forget(&self, changed: &BTreeSet<String>) {
        (self.programs.borrow_mut()).retain(|path, _| !changed.contains(path));
        (self.texts.borrow_mut()).retain(|path, _| !changed.contains(path));
    }
}

impl<F: FileStoreMut + 'static> Workspace<F> {
    /// Sets the file `path` to `text`, or — `None` — removes it, and
    /// forgets what was read from it (see [`Workspace`]). Says whether it
    /// changed anything: written as it was, nothing is forgotten.
    pub fn write(&self, path: &str, text: Option<String>) -> bool {
        if self.files.borrow().read(path).ok() == text {
            return false;
        }
        self.files.borrow_mut().write(path, text);
        self.forget(&BTreeSet::from([resolve("", path)]));
        self.revision.set(self.revision.get() + 1);
        true
    }
}

impl<F: FileStore + 'static> geop_core_program::workspace::Workspace<StateValue>
    for Workspace<F>
{
    fn program(&self, path: &str) -> GeopResult<Loaded<StateValue>> {
        if let Some(program) = self.programs.borrow().get(path) {
            return Ok(program.clone());
        }
        let text = self.files.borrow().read(path)?;
        let file =
            ProgramFile::from_json(&text).map_err(|e| e.with_context(format!("reading {path}")))?;
        let loaded = Loaded {
            program: Rc::new(file.program),
            state: Rc::new(file.state),
        };
        (self.programs.borrow_mut()).insert(path.to_string(), loaded.clone());
        Ok(loaded)
    }

    fn read(&self, path: &str) -> GeopResult<Rc<str>> {
        if let Some(text) = self.texts.borrow().get(path) {
            return Ok(text.clone());
        }
        let text: Rc<str> = self.files.borrow().read(path)?.into();
        (self.texts.borrow_mut()).insert(path.to_string(), text.clone());
        Ok(text)
    }

    fn list(&self) -> Vec<String> {
        (self.files.borrow().list().into_iter())
            .map(|f| resolve("", &f))
            .collect()
    }

    fn resolve(&self, from: &str, name: &str) -> String {
        resolve(from, name)
    }

    fn relative(&self, from: &str, path: &str) -> String {
        relative(from, path)
    }

    fn revision(&self) -> u64 {
        self.revision.get()
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
