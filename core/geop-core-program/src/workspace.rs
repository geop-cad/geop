//! [`Workspace`]: where the programs a program places, and the files its
//! steps read, come from — the one thing a run reads that is not given to
//! it as inputs or state, and not built by its steps — and [`Files`], a
//! workspace of files read from a [`FileStore`].
//!
//! A program never knows where it is. The runner running it does
//! ([`Location`]): the file it is in, and the files placing it. A make
//! reads a file through its reader ([`crate::reader::Reader::read_file`],
//! [`crate::reader::Reader::program`]), which records the file's revision
//! ([`Workspace::revision`]): its node is made again exactly when the file
//! changed.
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

use crate::{cache::Cache, operation_registry::OperationRegistry, program::Program};

/// Programs and other files, by path.
pub trait Workspace {
    /// The program in the file at `path`, with the state it was saved
    /// with: the same each time it is asked for while the file does not
    /// change.
    fn program(&self, path: &str) -> GeopResult<Rc<Program>>;

    /// The text of the file at `path`: a STEP file a step imports. The same
    /// each time it is asked for while the file does not change.
    fn read(&self, path: &str) -> GeopResult<Rc<str>>;

    /// Every file there is, by path.
    fn list(&self) -> Vec<String>;

    /// The path of the file the file at `from` names `name`.
    fn resolve(&self, from: &str, name: &str) -> String;

    /// How the file at `from` names the file at `path`.
    fn relative(&self, from: &str, path: &str) -> String;

    /// The revision of the file at `path`: changes whenever the file
    /// does, so a node that read it is made again.
    fn revision(&self, path: &str) -> u64;

    /// What it keeps of what its files were made into, shared by every
    /// build reading them: none, for a workspace that keeps nothing, whose
    /// files are made into what they read as each time they are read.
    fn cache(&self) -> Option<&Cache> {
        None
    }
}

/// Where the program a runner runs is: the file it is in, and the files
/// placing it, outermost first — which it must not place again, or the
/// files would place themselves. The runner's to know, never the
/// program's.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Location {
    pub file: String,
    pub within: Vec<String>,
}

/// Whether the file `path` is a program, which a part can be placed from:
/// a `.geop` file.
pub fn is_program(path: &str) -> bool {
    path.ends_with(".geop")
}

/// What the content of a file of bytes that are not text starts with: a
/// `data:` URL of them, in base64 (see [`content`]).
pub const BYTES_PREFIX: &str = "data:application/octet-stream;base64,";

/// The content of a file of `bytes`, as a workspace holds it: a file holds
/// text, so the text they spell where they are UTF-8 throughout, and
/// otherwise — a binary STL, a STEP file with a Latin-1 name in it — a
/// `data:` URL of them in base64. A front end reading a file a user chose
/// holds it the same way, so files travel as text everywhere and only
/// what reads bytes ([`bytes`]) or text ([`text`]) decodes them.
pub fn content(bytes: Vec<u8>) -> String {
    match String::from_utf8(bytes) {
        Ok(text) => text,
        Err(e) => format!("{BYTES_PREFIX}{}", base64(e.as_bytes())),
    }
}

/// The bytes of a file whose content is `content` (see [`content`]).
pub fn bytes(content: &str) -> GeopResult<std::borrow::Cow<'_, [u8]>> {
    match content.strip_prefix(BYTES_PREFIX) {
        Some(encoded) => Ok(std::borrow::Cow::Owned(from_base64(encoded)?)),
        None => Ok(std::borrow::Cow::Borrowed(content.as_bytes())),
    }
}

/// The text of a file whose content is `content` (see [`content`]): its
/// bytes read as UTF-8, what is not replaced.
pub fn text(content: &str) -> std::borrow::Cow<'_, str> {
    match bytes(content) {
        Ok(std::borrow::Cow::Owned(bytes)) => {
            std::borrow::Cow::Owned(String::from_utf8_lossy(&bytes).into_owned())
        }
        _ => std::borrow::Cow::Borrowed(content),
    }
}

/// The base64 alphabet of RFC 4648.
const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// `bytes` as base64 text (RFC 4648, padded).
pub fn base64(bytes: &[u8]) -> String {
    let mut text = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [0, 1, 2].map(|k| chunk.get(k).copied().unwrap_or(0) as u32);
        let n = (b[0] << 16) | (b[1] << 8) | b[2];
        for k in 0..4 {
            text.push(match BASE64.get((n >> (18 - 6 * k) & 63) as usize) {
                Some(&digit) if k <= chunk.len() => digit as char,
                _ => '=',
            });
        }
    }
    text
}

/// The bytes of the base64 text `text` (RFC 4648, padded or not).
fn from_base64(text: &str) -> GeopResult<Vec<u8>> {
    let digits = text.trim_end_matches('=').as_bytes();
    let mut bytes = Vec::with_capacity(digits.len() * 3 / 4);
    let (mut n, mut bits) = (0u32, 0);
    for &c in digits {
        let Some(value) = BASE64.iter().position(|&d| d == c) else {
            return Err(GeopError::new(format!(
                "{:?} is no base64 digit",
                c as char
            )));
        };
        n = (n << 6) | value as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            bytes.push((n >> bits) as u8);
            n &= (1 << bits) - 1;
        }
    }
    Ok(bytes)
}

/// Where [`Files`] reads files from.
pub trait FileStore {
    /// The text of the file `path`.
    fn read(&self, path: &str) -> GeopResult<String>;
    /// Every file there is, by path: programs and the files they read.
    fn list(&self) -> Vec<String>;
}

/// [`FileStore`] [`Files`] can change ([`Files::write`]).
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

/// Files of a store chosen at run time: an editor's, whose files are
/// whatever its front end keeps, with whatever library beside them.
impl FileStore for Box<dyn FileStoreMut> {
    fn read(&self, path: &str) -> GeopResult<String> {
        (**self).read(path)
    }

    fn list(&self) -> Vec<String> {
        (**self).list()
    }
}

impl FileStoreMut for Box<dyn FileStoreMut> {
    fn write(&mut self, path: &str, text: Option<String>) {
        (**self).write(path, text)
    }
}

/// A set of files, whose programs are written in the operations of
/// `registry`: the [`Workspace`] its programs are run with, reading the
/// programs they place and the files they import from it. A program read
/// is checked against the operations (see [`Program::validate`]).
///
/// Each file is read once, and read again once it changes
/// ([`Files::write`]), when its revision changes too.
pub struct Files<F: 'static = BTreeMap<String, String>> {
    files: RefCell<F>,
    registry: Rc<OperationRegistry>,
    /// Each file's program, read once: by its path.
    programs: RefCell<BTreeMap<String, Rc<Program>>>,
    /// Each other file's text, read once: by its path.
    texts: RefCell<BTreeMap<String, Rc<str>>>,
    /// How many times a file changed ([`Files::write`]).
    revision: Cell<u64>,
    /// The revision of each file that changed: the count when it last did.
    revisions: RefCell<BTreeMap<String, u64>>,
    /// What its files were made into.
    cache: Cache,
}

impl<F: FileStore + 'static> Files<F> {
    /// The files `files`, what they are made into kept in memory.
    pub fn new(files: F, registry: Rc<OperationRegistry>) -> Rc<Self> {
        Self::with_cache(files, registry, Cache::new())
    }

    /// The files `files`, what they are made into kept in `cache`.
    pub fn with_cache(files: F, registry: Rc<OperationRegistry>, cache: Cache) -> Rc<Self> {
        Rc::new(Self {
            files: RefCell::new(files),
            registry,
            programs: RefCell::new(BTreeMap::new()),
            texts: RefCell::new(BTreeMap::new()),
            revision: Cell::new(0),
            revisions: RefCell::new(BTreeMap::new()),
            cache,
        })
    }

    /// The files, to read.
    pub fn files(&self) -> std::cell::Ref<'_, F> {
        self.files.borrow()
    }

    /// The operations its programs are written in.
    pub fn registry(&self) -> &Rc<OperationRegistry> {
        &self.registry
    }

    /// Forgets what was read from any of the files `changed` — paths as
    /// [`resolve`] gives them.
    fn forget(&self, changed: &BTreeSet<String>) {
        (self.programs.borrow_mut()).retain(|path, _| !changed.contains(path));
        (self.texts.borrow_mut()).retain(|path, _| !changed.contains(path));
    }
}

impl<F: FileStoreMut + 'static> Files<F> {
    /// Sets the file `path` to `text`, or — `None` — removes it, and
    /// forgets what was read from it (see [`Files`]). Says whether it
    /// changed anything: written as it was, nothing is forgotten.
    pub fn write(&self, path: &str, text: Option<String>) -> bool {
        if self.files.borrow().read(path).ok() == text {
            return false;
        }
        self.files.borrow_mut().write(path, text);
        let path = resolve("", path);
        self.forget(&BTreeSet::from([path.clone()]));
        self.revision.set(self.revision.get() + 1);
        (self.revisions.borrow_mut()).insert(path, self.revision.get());
        true
    }
}

impl<F: FileStore + 'static> Workspace for Files<F> {
    fn program(&self, path: &str) -> GeopResult<Rc<Program>> {
        if let Some(program) = self.programs.borrow().get(path) {
            return Ok(program.clone());
        }
        let text = self.files.borrow().read(path)?;
        let program = (Program::from_json(&text))
            .and_then(|program| program.validate(&self.registry).map(|()| program))
            .map_err(|e| e.with_context(format!("reading {path}")))?;
        let program = Rc::new(program);
        (self.programs.borrow_mut()).insert(path.to_string(), program.clone());
        Ok(program)
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

    fn revision(&self, path: &str) -> u64 {
        self.revisions
            .borrow()
            .get(path)
            .copied()
            .unwrap_or_default()
    }

    fn cache(&self) -> Option<&Cache> {
        Some(&self.cache)
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
    let mut folders: Vec<&str> = from.split('/').collect();
    folders.pop();
    let target: Vec<&str> = to.split('/').collect();
    let common = folders
        .iter()
        .zip(&target)
        .take_while(|(a, b)| a == b)
        .count()
        .min(target.len().saturating_sub(1));
    let up = std::iter::repeat_n("..", folders.len() - common);
    up.chain(target.iter().skip(common).copied())
        .collect::<Vec<_>>()
        .join("/")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Text is held as it is; bytes that are not text as a `data:` URL,
    /// which gives them back whole, and as text with what is not UTF-8
    /// replaced. Base64 as RFC 4648 spells it.
    #[test]
    fn files_hold_bytes_as_text() {
        assert_eq!(content(b"solid cube".to_vec()), "solid cube");
        assert_eq!(&*bytes("solid cube").unwrap(), b"solid cube");
        assert_eq!(base64(b"foobar"), "Zm9vYmFy");
        assert_eq!(base64(b"fo"), "Zm8=");
        // As the front end holds them (`web/test/files.test.ts`).
        assert_eq!(
            content(vec![0xff, 0xf8, 0xf1]),
            format!("{BYTES_PREFIX}//jx")
        );
        for n in 0..7 {
            let binary: Vec<u8> = (0..n).map(|k| 0xff - 7 * k as u8).collect();
            let held = content(binary.clone());
            if n > 0 {
                assert!(held.starts_with(BYTES_PREFIX), "{held}");
            }
            assert_eq!(&*bytes(&held).unwrap(), &binary[..]);
        }
        let latin1 = content(b"caf\xe9".to_vec());
        assert_eq!(text(&latin1), "caf\u{fffd}");
        assert!(bytes(&format!("{BYTES_PREFIX}!!")).is_err());
    }

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
