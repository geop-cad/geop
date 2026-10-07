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
pub trait Extension<S: Scalar>: Any + Clone + Send + Sync {
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
}

/// An [`Extension`] with its type forgotten: all a part needs to copy and
/// present it.
trait Erased<S: Scalar>: Send + Sync {
    fn clone_box(&self) -> Box<dyn Erased<S>>;
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn annotations(&self) -> GeopResult<Vec<Annotation<S>>>;
    fn describe(&self) -> BTreeMap<String, serde_json::Value>;
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
}
