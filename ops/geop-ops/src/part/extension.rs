//! A [`Part`]'s extensions: the state an operation family keeps in a part
//! for itself, and for the families that depend on it — a harness's
//! cables, a hole's cosmetic threads — found by the type that holds it.
//!
//! The framework knows no extension. An operation crate that reads another
//! family's state depends on that family's crate in `Cargo.toml`, and asks
//! for its type: `part.ext::<Cables<S>>()`. A family's state comes into
//! being at its first [`Part::ext_mut`], so there is nothing to register
//! and nothing to forget; [`Part::ext`] is `None` until then.
//!
//! A family that keeps values by name (the mates of a part, say) is an
//! [`EntryKind`]: each value is a cell of its own, so a step that adds one
//! reads no other, and the steps that add values stay independent of each
//! other (see [`Part::insert_entry`]).
//!
//! An extension travels with the part and says what it needs the framework
//! to say for it — what a viewer draws of it ([`Extension::annotations`]),
//! what a description lists of it ([`Extension::describe`]) — so a part is
//! viewed and described wherever it is, with no registry to hand.

use std::{any::Any, collections::BTreeMap};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::{Part, cache::Cache, cells::Cell};

/// Something an extension has drawn on a part, besides its topology: a
/// polyline, and what it is called.
#[derive(Clone, Debug)]
pub struct Annotation<S: Scalar> {
    pub name: String,
    /// What a viewer writes beside it: `M6x1`.
    pub label: String,
    pub polyline: Vec<Vector3<S>>,
}

/// State an operation family keeps in a [`Part`] (see the module). A part is
/// copied whole, extensions with it.
pub trait Extension<S: Scalar>: Any + Clone + Default + Send + Sync {
    /// What tells it apart from every other extension, and the key of its
    /// description. Unique across the workspace. Extensions are kept and
    /// presented in the order of their names: the order of their types' ids
    /// differs between two builds of the same program.
    const NAME: &'static str;

    /// What a viewer draws of it, as the part is placed nowhere.
    fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
        Ok(Vec::new())
    }

    /// What a description of the part lists of it, by entry name.
    fn describe(&self) -> BTreeMap<String, serde_json::Value> {
        BTreeMap::new()
    }

    /// Takes the entry `key` as `from` has it — none, if it has none —
    /// for an extension that keeps entries (see [`EntryKind`]).
    fn copy_entry(&mut self, _from: Option<&Self>, _key: &str) {}

    /// Whether it holds the part placed at the path `instance` — `bolt`,
    /// or `asm/bolt` for one placed in a part placed — where it is: so
    /// that a viewer does not offer to drag it.
    fn holds(&self, _instance: &str) -> bool {
        false
    }
}

/// A family of values kept in a part by name: the `Value`s, found by the
/// name they were given.
pub trait EntryKind<S: Scalar>: 'static + Send + Sync {
    /// What tells it apart from every other extension (see
    /// [`Extension::NAME`]).
    const NAME: &'static str;
    type Value: Clone + Send + Sync + 'static;

    /// Whether the entries hold the part placed at the path `instance` (see
    /// [`Extension::holds`]).
    fn holds(_entries: &BTreeMap<String, Self::Value>, _instance: &str) -> bool {
        false
    }

    /// What a description of the part lists of the entries, by name.
    fn describe(_entries: &BTreeMap<String, Self::Value>) -> BTreeMap<String, serde_json::Value> {
        BTreeMap::new()
    }
}

/// The extension that holds the entries of the kind `K`.
pub struct Entries<S: Scalar, K: EntryKind<S>>(
    BTreeMap<String, K::Value>,
    std::marker::PhantomData<fn() -> (S, K)>,
);

impl<S: Scalar, K: EntryKind<S>> Clone for Entries<S, K> {
    fn clone(&self) -> Self {
        Self(self.0.clone(), std::marker::PhantomData)
    }
}

impl<S: Scalar, K: EntryKind<S>> Default for Entries<S, K> {
    fn default() -> Self {
        Self(BTreeMap::new(), std::marker::PhantomData)
    }
}

impl<S: Scalar, K: EntryKind<S>> Extension<S> for Entries<S, K> {
    const NAME: &'static str = K::NAME;

    fn describe(&self) -> BTreeMap<String, serde_json::Value> {
        K::describe(&self.0)
    }

    fn holds(&self, instance: &str) -> bool {
        K::holds(&self.0, instance)
    }

    fn copy_entry(&mut self, from: Option<&Self>, key: &str) {
        match from.and_then(|from| from.0.get(key)) {
            Some(value) => {
                self.0.insert(key.to_string(), value.clone());
            }
            None => {
                self.0.remove(key);
            }
        }
    }
}

/// An [`Extension`] with its type forgotten: all a part needs to copy and
/// present it.
trait Erased<S: Scalar>: Send + Sync {
    fn clone_box(&self) -> Box<dyn Erased<S>>;
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn annotations(&self) -> GeopResult<Vec<Annotation<S>>>;
    fn describe(&self) -> BTreeMap<String, serde_json::Value>;
    fn holds(&self, instance: &str) -> bool;
    fn empty_like(&self) -> Box<dyn Erased<S>>;
    fn copy_key(&mut self, from: Option<&dyn Erased<S>>, key: &str);
}

impl<S: Scalar, E: Extension<S>> Erased<S> for E {
    fn clone_box(&self) -> Box<dyn Erased<S>> {
        Box::new(self.clone())
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
        Extension::annotations(self)
    }
    fn describe(&self) -> BTreeMap<String, serde_json::Value> {
        Extension::describe(self)
    }
    fn holds(&self, instance: &str) -> bool {
        Extension::holds(self, instance)
    }
    fn empty_like(&self) -> Box<dyn Erased<S>> {
        Box::new(E::default())
    }
    fn copy_key(&mut self, from: Option<&dyn Erased<S>>, key: &str) {
        let from = from.and_then(|from| from.as_any().downcast_ref::<E>());
        Extension::copy_entry(self, from, key);
    }
}

/// A part's extensions, by [`Extension::NAME`].
pub(super) struct Extensions<S: Scalar>(BTreeMap<&'static str, Box<dyn Erased<S>>>);

impl<S: Scalar> Extensions<S> {
    pub(super) fn new() -> Self {
        Self(BTreeMap::new())
    }
}

impl<S: Scalar> Extensions<S> {
    /// Takes the extension `name` as `from` has it: none, if it has none.
    pub(super) fn copy_entry(&mut self, from: &Self, name: &str) {
        match from.0.get_key_value(name) {
            Some((&name, extension)) => {
                self.0.insert(name, extension.clone_box());
            }
            None => {
                self.0.remove(name);
            }
        }
    }
}

impl<S: Scalar> Extensions<S> {
    /// Takes the entry `key` of the extension `name` as `from` has it.
    pub(super) fn copy_key(&mut self, from: &Self, name: &'static str, key: &str) {
        let source = from.0.get(name);
        if self.0.get(name).is_none() {
            let Some(source) = source else { return };
            self.0.insert(name, source.empty_like());
        }
        self.0
            .get_mut(name)
            .expect("inserted above")
            .copy_key(source.map(|s| &**s), key);
    }
}

impl<S: Scalar> Clone for Extensions<S> {
    fn clone(&self) -> Self {
        Self(
            self.0
                .iter()
                .map(|(&name, extension)| (name, extension.clone_box()))
                .collect(),
        )
    }
}

impl<S: Scalar> Part<S> {
    /// The state of the family `E`, if an operation has kept any in this
    /// part.
    pub fn ext<E: Extension<S>>(&self) -> Option<&E> {
        self.store.read(Cell::Ext(E::NAME));
        self.store
            .extensions()
            .0
            .get(E::NAME)?
            .as_any()
            .downcast_ref()
    }

    /// The state of the family `E`, to change: an empty one if there was
    /// none. Anything worked out of the part is forgotten, since it is
    /// about to differ.
    pub fn ext_mut<E: Extension<S> + Default>(&mut self) -> &mut E {
        self.cache = Cache::new();
        self.store.write(Cell::Ext(E::NAME));
        self.store
            .extensions_mut()
            .0
            .entry(E::NAME)
            .or_insert_with(|| Box::new(E::default()))
            .as_any_mut()
            .downcast_mut()
            .unwrap_or_else(|| {
                panic!(
                    "two extensions are named {:?}: names must be unique",
                    E::NAME
                )
            })
    }

    /// The entry `key` of the kind `K`, if there is one.
    pub fn entry<K: EntryKind<S>>(&self, key: &str) -> Option<&K::Value> {
        self.store.read(Cell::Entry(K::NAME, key.to_string()));
        self.store
            .extensions()
            .0
            .get(K::NAME)?
            .as_any()
            .downcast_ref::<Entries<S, K>>()?
            .0
            .get(key)
    }

    /// Every entry of the kind `K`, by name.
    pub fn entries<K: EntryKind<S>>(&self) -> impl Iterator<Item = (&str, &K::Value)> {
        self.store.read(Cell::Entries(K::NAME));
        self.store
            .extensions()
            .0
            .get(K::NAME)
            .and_then(|e| e.as_any().downcast_ref::<Entries<S, K>>())
            .into_iter()
            .flat_map(|entries| entries.0.iter().map(|(key, value)| (key.as_str(), value)))
    }

    /// Keeps `value` as the entry `key` of the kind `K`. Reads that entry
    /// and the list of them is written, but not read: a step that adds an
    /// entry depends on no other.
    pub fn insert_entry<K: EntryKind<S>>(&mut self, key: impl Into<String>, value: K::Value) {
        let key = key.into();
        self.cache = Cache::new();
        self.store.write(Cell::Entry(K::NAME, key.clone()));
        self.store.write(Cell::Entries(K::NAME));
        self.store
            .extensions_mut()
            .0
            .entry(K::NAME)
            .or_insert_with(|| Box::new(Entries::<S, K>::default()))
            .as_any_mut()
            .downcast_mut::<Entries<S, K>>()
            .unwrap_or_else(|| {
                panic!(
                    "two extensions are named {:?}: names must be unique",
                    K::NAME
                )
            })
            .0
            .insert(key, value);
    }

    /// Whether an extension holds the part placed at the path `instance`
    /// where it is (see [`Extension::holds`]).
    pub fn holds(&self, instance: &str) -> bool {
        self.store
            .extensions()
            .0
            .values()
            .any(|extension| extension.holds(instance))
    }

    /// What every extension has drawn on the part, in the order of their
    /// names.
    pub(crate) fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
        let mut annotations = Vec::new();
        for extension in self.store.extensions().0.values() {
            annotations.extend(extension.annotations()?);
        }
        Ok(annotations)
    }

    /// What every extension that lists anything lists of itself, by its
    /// name.
    pub(crate) fn extension_descriptions(
        &self,
    ) -> BTreeMap<String, BTreeMap<String, serde_json::Value>> {
        self.store
            .extensions()
            .0
            .iter()
            .map(|(&name, extension)| (name.to_string(), extension.describe()))
            .filter(|(_, description)| !description.is_empty())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::scalars::scal_in_f64::ScalInF64;

    use super::*;

    type S = ScalInF64;

    /// A family that keeps a number and draws one annotation of it.
    #[derive(Clone, Debug, Default, PartialEq)]
    struct Counter(i64);

    impl Extension<S> for Counter {
        const NAME: &'static str = "counter";

        fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
            Ok(vec![Annotation {
                name: "count".into(),
                label: self.0.to_string(),
                polyline: Vec::new(),
            }])
        }
    }

    /// A family that draws nothing, named to sort before [`Counter`].
    #[derive(Clone, Debug, Default, PartialEq)]
    struct Note(String);

    impl Extension<S> for Note {
        const NAME: &'static str = "bote";

        fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
            Ok(vec![Annotation {
                name: "note".into(),
                label: self.0.clone(),
                polyline: Vec::new(),
            }])
        }

        fn describe(&self) -> BTreeMap<String, serde_json::Value> {
            BTreeMap::from([("text".to_string(), self.0.clone().into())])
        }
    }

    /// Nothing is kept until a family writes; a copy keeps what was
    /// written, and changing either leaves the other as it was.
    #[test]
    fn state_is_created_on_first_write_and_copied_with_the_part() {
        let mut part = Part::<S>::new();
        assert_eq!(part.ext::<Counter>(), None);

        part.ext_mut::<Counter>().0 = 3;
        let copy = part.clone();
        part.ext_mut::<Counter>().0 = 4;

        assert_eq!(part.ext::<Counter>(), Some(&Counter(4)));
        assert_eq!(copy.ext::<Counter>(), Some(&Counter(3)));
        assert_eq!(copy.ext::<Note>(), None);
    }

    /// Extensions present and describe in the order of their names, whichever
    /// was written first, and a family that lists nothing is left out of the
    /// description.
    #[test]
    fn extensions_present_in_the_order_of_their_names() {
        let mut part = Part::<S>::new();
        part.ext_mut::<Counter>().0 = 1;
        part.ext_mut::<Note>().0 = "hi".into();

        let labels: Vec<String> = part
            .annotations()
            .unwrap()
            .into_iter()
            .map(|a| a.label)
            .collect();
        assert_eq!(labels, ["hi", "1"]);

        let descriptions = part.extension_descriptions();
        assert_eq!(descriptions.keys().collect::<Vec<_>>(), ["bote"]);
    }

    /// What was worked out of a part is forgotten when an extension is
    /// about to change it.
    #[test]
    fn writing_an_extension_forgets_what_was_worked_out() {
        let mut part = Part::<S>::new();
        assert!(part.view().unwrap().annotations.is_empty());
        part.ext_mut::<Counter>().0 = 1;
        assert_eq!(part.view().unwrap().annotations[0].label, "1");
        part.ext_mut::<Counter>().0 = 2;
        let after = part.view().unwrap();
        assert_eq!(after.annotations.len(), 1);
        assert_eq!(after.annotations[0].label, "2");
    }

    /// A family of values kept by name.
    struct Notes;

    impl EntryKind<S> for Notes {
        const NAME: &'static str = "notes";
        type Value = String;
    }

    /// Adding an entry reads and writes that entry and writes the list
    /// of them, and reads no other: steps that add entries do not depend on
    /// each other. A replay of the writes gives the entry to another part.
    #[test]
    fn entries_are_cells_of_their_own() {
        use crate::part::{Cell, Log};

        let mut part = Part::<S>::new();
        part.insert_entry::<Notes>("first", "a".to_string());

        let log = Log::new();
        part.record(&log);
        part.insert_entry::<Notes>("second", "b".to_string());
        let access = part.finish_recording(&log).unwrap();

        let entry = |key: &str| Cell::Entry("notes", key.to_string());
        assert_eq!(access.reads, [entry("second")].into());
        assert_eq!(
            access.writes,
            [entry("second"), Cell::Entries("notes")].into()
        );
        assert_eq!(part.entry::<Notes>("first").map(String::as_str), Some("a"));
        assert_eq!(part.entries::<Notes>().count(), 2);

        let mut other = Part::<S>::new();
        other.insert_entry::<Notes>("first", "a".to_string());
        let mut before = Part::<S>::new();
        before.insert_entry::<Notes>("first", "a".to_string());
        let after = std::sync::Arc::new(part);
        other.record(&Log::new());
        let replayed = other.replayed(&before, &after, &access);
        assert_eq!(
            replayed.entry::<Notes>("second").map(String::as_str),
            Some("b")
        );
    }
}
