//! [`Cache`]: what the text of a file was made into — a STEP file's bodies —
//! kept by the workspace, so that a file is made into the same thing once
//! for every build that reads it (a step's preview, the program it is
//! committed to, a part placed from it in another file), and, with a
//! [`CacheStore`], once for every process.
//!
//! What a cache keeps is the answer to "what does this text read as", and
//! nothing else: it is looked up by the file's path and checked against
//! the file's whole text in memory, and against the text's digest and
//! length, and the kernel build that made it, on disk. A value from
//! another build of the kernel may have been made by other code, so it is
//! made again rather than trusted.
//!
//! On disk, what the file `path` was made into is kept in `path.geopc`
//! beside it: `AX-12A.STEP.geopc`. Its values are written exactly
//! ([`geop_core_math::exact`]), so the value read back is the value that
//! was made. A `.geopc` file is a cache: deleting it costs a make, and it
//! belongs in no repository.

use std::{
    any::Any,
    cell::{Cell, RefCell},
    collections::BTreeMap,
    rc::Rc,
};

#[cfg(test)]
use geop_core_math::geop_error::GeopError;
use geop_core_math::geop_error::GeopResult;
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::workspace::is_shared;

/// The extension of a cache file: what the file it is beside was made
/// into ([`Cache`]).
pub const CACHE_EXTENSION: &str = "geopc";

/// Where a [`Cache`] keeps what it made between processes: a folder on
/// disk, for a kernel run by the command line or an editor's host process.
pub trait CacheStore {
    /// What tells this build of the kernel from every other: what it made
    /// is only read back by the same build.
    fn build(&self) -> &str;
    /// The text of the cache file `path`, if there is one.
    fn read(&self, path: &str) -> Option<String>;
    /// Writes the cache file `path`. A cache that could not be written is
    /// made again next time, nothing else, so a failure is the store's to
    /// report, if anywhere.
    fn write(&self, path: &str, text: &str);
}

/// What a [`Cache`] keeps in memory, by the kind of value and the file's
/// path: the text it was made from, and what it was made into.
type Kept = BTreeMap<(&'static str, String), (Rc<str>, Rc<dyn Any>)>;

/// What the text of a file was made into, kept by the file's path (see
/// the [module](self)).
#[derive(Default)]
pub struct Cache {
    store: Option<Box<dyn CacheStore>>,
    kept: RefCell<Kept>,
    /// How many values it made: what a test of what is made again counts.
    made: Cell<u64>,
}

/// A cache file: what the file beside it was made into, and from what.
#[derive(Serialize, Deserialize)]
struct Entry<T> {
    /// The build of the kernel that made it ([`CacheStore::build`]).
    build: String,
    /// What it was made into: [`Cache::made_of`]'s `kind`.
    kind: String,
    /// The text it was made from: its digest and length ([`source`]).
    source: String,
    value: T,
}

impl Cache {
    /// A cache kept in memory only.
    pub fn new() -> Self {
        Self::default()
    }

    /// A cache kept in memory, and between processes in `store`.
    pub fn stored(store: Box<dyn CacheStore>) -> Self {
        Self {
            store: Some(store),
            ..Self::default()
        }
    }

    /// What the text `text` of the file `path` reads as, as `kind`: made by
    /// `make` once, for this text of this file, and kept. A value that
    /// could not be made is not kept: it is made again when asked again.
    pub fn made_of<T: Serialize + DeserializeOwned + 'static>(
        &self,
        kind: &'static str,
        path: &str,
        text: &Rc<str>,
        make: impl FnOnce(&str) -> GeopResult<T>,
    ) -> GeopResult<Rc<T>> {
        let key = (kind, path.to_string());
        if let Some((from, value)) = self.kept.borrow().get(&key)
            && (Rc::ptr_eq(from, text) || **from == **text)
            && let Ok(value) = value.clone().downcast::<T>()
        {
            return Ok(value);
        }
        let value = match self.read(kind, path, text) {
            Some(value) => Rc::new(value),
            None => {
                self.made.set(self.made.get() + 1);
                let value = Rc::new(make(text)?);
                self.write(kind, path, text, &*value);
                value
            }
        };
        let kept: Rc<dyn Any> = value.clone();
        self.kept.borrow_mut().insert(key, (text.clone(), kept));
        Ok(value)
    }

    /// How many values it made, rather than found kept.
    pub fn made(&self) -> u64 {
        self.made.get()
    }

    /// What its store kept of `text` of `path` as `kind`, if it was made by
    /// this build from this text. A file that does not read, of another
    /// build or text or kind, is none.
    fn read<T: DeserializeOwned>(&self, kind: &str, path: &str, text: &str) -> Option<T> {
        let store = self.store.as_ref().filter(|_| !is_shared(path))?;
        let kept = store.read(&cache_path(path))?;
        let entry: Entry<T> = serde_json::from_str(&kept).ok()?;
        (entry.build == store.build() && entry.kind == kind && entry.source == source(text))
            .then_some(entry.value)
    }

    /// Keeps `value`, made of `text` of `path` as `kind`, in its store.
    fn write<T: Serialize>(&self, kind: &str, path: &str, text: &str, value: &T) {
        // A shared library's file is not where the files of the workspace
        // are: nothing is kept beside it.
        let Some(store) = self.store.as_ref().filter(|_| !is_shared(path)) else {
            return;
        };
        let entry = Entry {
            build: store.build().to_string(),
            kind: kind.to_string(),
            source: source(text),
            value,
        };
        // A value that does not serialize — an enclosure gone infinite — is
        // only not kept.
        if let Ok(written) = serde_json::to_string(&entry) {
            store.write(&cache_path(path), &written);
        }
    }
}

/// The cache file of the file `path`: beside it, `path.geopc`.
pub fn cache_path(path: &str) -> String {
    format!("{path}.{CACHE_EXTENSION}")
}

/// What tells a text from others: its digest and its length.
fn source(text: &str) -> String {
    format!("{:032x}:{}", digest(text), text.len())
}

/// A 128-bit FNV-1a hash of `text`: written out here rather than the
/// standard library's hasher, which may change between Rust versions, so
/// that what is saved of a text keeps naming the same text.
pub fn digest(text: &str) -> u128 {
    const PRIME: u128 = 0x0000000001000000000000000000013B;
    let mut hash: u128 = 0x6c62272e07bb014262b821756295c58d;
    for byte in text.bytes() {
        hash ^= u128::from(byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Cache files in memory, as a store on disk keeps them, of the build
    /// `build`.
    #[derive(Clone)]
    struct Shelf {
        build: String,
        files: Rc<RefCell<BTreeMap<String, String>>>,
    }

    impl CacheStore for Shelf {
        fn build(&self) -> &str {
            &self.build
        }

        fn read(&self, path: &str) -> Option<String> {
            self.files.borrow().get(path).cloned()
        }

        fn write(&self, path: &str, text: &str) {
            (self.files.borrow_mut()).insert(path.to_string(), text.to_string());
        }
    }

    fn words(text: &str) -> GeopResult<Vec<String>> {
        Ok(text.split_whitespace().map(str::to_string).collect())
    }

    /// A text is made into its value once; another text of the file is made
    /// again; a text that cannot be made is not kept.
    #[test]
    fn a_text_is_made_once() {
        let cache = Cache::new();
        let text: Rc<str> = Rc::from("a b");
        let first = cache.made_of("words", "f.txt", &text, words).unwrap();
        let again = cache
            .made_of("words", "f.txt", &Rc::from("a b"), words)
            .unwrap();
        assert!(Rc::ptr_eq(&first, &again));
        assert_eq!(cache.made(), 1);
        let other = cache
            .made_of("words", "f.txt", &Rc::from("c"), words)
            .unwrap();
        assert_eq!(*other, ["c"]);
        assert_eq!(cache.made(), 2);
        let failing = |_: &str| -> GeopResult<Vec<String>> { Err(GeopError::new("no")) };
        assert!(cache.made_of("words", "g.txt", &text, failing).is_err());
        assert!(cache.made_of("words", "g.txt", &text, words).is_ok());
        assert_eq!(cache.made(), 4);
    }

    /// What one process made is read back by the next of the same build,
    /// from `path.geopc` — and made again by another build, or from another
    /// text.
    #[test]
    fn a_store_keeps_what_was_made_between_processes() {
        let shelf = Shelf {
            build: "1".into(),
            files: Rc::default(),
        };
        let text: Rc<str> = Rc::from("a b");
        let made = |shelf: &Shelf, text: &Rc<str>| {
            let cache = Cache::stored(Box::new(shelf.clone()));
            let value = cache.made_of("words", "f.txt", text, words).unwrap();
            (value, cache.made())
        };
        assert_eq!(made(&shelf, &text), (Rc::new(words("a b").unwrap()), 1));
        assert!(shelf.files.borrow().contains_key("f.txt.geopc"));
        assert_eq!(made(&shelf, &text), (Rc::new(words("a b").unwrap()), 0));
        assert_eq!(made(&shelf, &Rc::from("a c")).1, 1);
        let rebuilt = Shelf {
            build: "2".into(),
            ..shelf
        };
        assert_eq!(made(&rebuilt, &Rc::from("a c")).1, 1);
    }
}
