//! [`ElementReference`]: an element of a target by its stable name — an edge
//! or a face of a part's topology, a line of a sketch — typed by the target
//! it is in and the [`Kind`] of element it is, as a [`TargetReference`] is
//! typed by its target. What an operation is given to build on, and what a
//! pick in the viewport becomes ([`PickedElement`]).
//!
//! The crate that defines a target defines the kinds of element it holds and
//! how one is resolved ([`HasElements`]); an operation that builds on one
//! depends on that crate and names the kind in its arguments' types, so a
//! reference of the wrong kind does not compile, and no crate needs to know
//! every kind there is.

use std::marker::PhantomData;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::{
    instance::{instance_path, split_path},
    target::TargetReference,
};

/// A kind of element a target holds by stable name: an edge or a face of a
/// topology, a line of a sketch. A unit struct, defined by the crate that
/// defines the target.
pub trait Kind: 'static {
    /// The kind's name, as a pick says what it picked: `"edge"`.
    const NAME: &'static str;
}

/// A target that holds elements of the kind `K`, each by a stable name.
/// Elements are the only way a target is read: one meant to be read whole —
/// a sketch, to sweep — is a kind of its own, `impl Kind for Sketch` and
/// `impl HasElements<Sketch> for Sketch`, its one element the empty name
/// ([`ElementReference::whole`]).
pub trait HasElements<K: Kind>: 'static {
    /// What an element resolves to: a borrow of it, or what it gives.
    type Element<'a>;

    /// The name of the one target of this type a part has, if it has only
    /// one — the topology, say: a reference into it is saved as the
    /// element's name alone.
    const SINGLETON: Option<&'static str> = None;

    /// The element `name`, or why there is none.
    fn element(&self, name: &str) -> GeopResult<Self::Element<'_>>;

    /// Whether the target named `name` could be one of this type: what
    /// tells a pick of an element of it from one of a target of another
    /// type with elements of the same kind ([`ElementReference::from_picked`]).
    /// The singleton's name, for a singleton; any name, by default.
    fn names_target(name: &str) -> bool {
        Self::SINGLETON.is_none_or(|singleton| singleton == name)
    }
}

/// The element of the kind `K` named `name` in the target `target`. Saved
/// as `{"target": "s1", "name": "c3"}`; in a target that is a part's only
/// one of its type ([`HasElements::SINGLETON`]) as `"c3"`; the whole target
/// ([`ElementReference::whole`]) as `{"target": "s1"}`.
pub struct ElementReference<T, K> {
    target: TargetReference<T>,
    name: String,
    kind: PhantomData<fn() -> K>,
}

// By hand, since a derive would ask the same of `T` and `K`, and a
// reference is only names.
impl<T, K> Clone for ElementReference<T, K> {
    fn clone(&self) -> Self {
        Self::new(self.target.clone(), self.name.clone())
    }
}

impl<T, K> PartialEq for ElementReference<T, K> {
    fn eq(&self, other: &Self) -> bool {
        self.target == other.target && self.name == other.name
    }
}

impl<T, K> Eq for ElementReference<T, K> {}

impl<T, K: Kind> std::fmt::Debug for ElementReference<T, K> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}({:?}, {:?})", K::NAME, self.target.name(), self.name)
    }
}

impl<T, K> ElementReference<T, K> {
    pub fn new(target: TargetReference<T>, name: impl Into<String>) -> Self {
        Self {
            target,
            name: name.into(),
            kind: PhantomData,
        }
    }

    /// The target the element is in.
    pub fn target(&self) -> &TargetReference<T> {
        &self.target
    }

    /// The element's stable name in its target.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The path of the instance the element is of: empty for one of the
    /// part itself (see [`crate::instance`]).
    pub fn instance(&self) -> &str {
        split_path(self.target.name()).0
    }

    /// The same element as its instance's own part names it: itself, for
    /// one of the part itself.
    pub fn local(&self) -> Self {
        let target = split_path(self.target.name()).1;
        Self::new(TargetReference::new(target), self.name.clone())
    }

    /// The same element of the instance `instance` of the part — of the
    /// instance at that path, if it is already of one: what an element of
    /// a placed part is, read from the part it is placed in.
    pub fn behind(&self, instance: &str) -> Self {
        let target = instance_path(instance, self.target.name());
        Self::new(TargetReference::new(target), self.name.clone())
    }
}

impl<T: HasElements<K>, K: Kind> ElementReference<T, K> {
    /// The whole of `target`, as its element of the empty name: for a
    /// target meant to be read whole, which is a kind of its own (see
    /// [`HasElements`]).
    pub fn whole(target: TargetReference<T>) -> Self {
        Self::new(target, "")
    }

    /// The element `name` of the singleton target ([`HasElements::SINGLETON`]).
    pub fn in_singleton(name: impl Into<String>) -> GeopResult<Self> {
        let target = T::SINGLETON.ok_or_else(|| {
            GeopError::new(format!(
                "a `{}` is no part's only target of its type: name the target of {}",
                std::any::type_name::<T>(),
                K::NAME
            ))
        })?;
        Ok(Self::new(TargetReference::new(target), name))
    }

    /// The reference `picked` is, if it picked an element of the kind `K`
    /// of a target of the type `T` — of the part itself, or of an instance
    /// placed in it.
    pub fn from_picked(picked: &PickedElement) -> Option<Self> {
        (picked.kind == K::NAME && T::names_target(split_path(&picked.target).1))
            .then(|| Self::new(TargetReference::new(picked.target.clone()), &picked.name))
    }

    /// The reference as a pick names it: what the front end shows selected.
    pub fn picked(&self) -> PickedElement {
        PickedElement {
            target: self.target.name().to_string(),
            kind: K::NAME.to_string(),
            name: self.name.clone(),
        }
    }
}

/// What a target read whole resolves its one element to: itself, for the
/// empty name ([`ElementReference::whole`]); any other names nothing.
pub fn whole<'a, T>(target: &'a T, name: &str) -> GeopResult<&'a T> {
    match name {
        "" => Ok(target),
        _ => Err(GeopError::new(format!(
            "`{}` is read whole, not by `{name}`",
            std::any::type_name::<T>()
        ))),
    }
}

/// An element as a pick in the viewport names it: what a front end sends
/// and is sent, which knows no types. Only [`ElementReference::from_picked`]
/// reads it, so an operation never matches on it.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PickedElement {
    /// The name of the target the element is in, behind the path of the
    /// instance it is of: `bolt/model`.
    pub target: String,
    /// Its kind's [`Kind::NAME`].
    pub kind: String,
    /// Its stable name in the target.
    pub name: String,
}

impl PickedElement {
    /// How a dialog names it: its kind and its name.
    pub fn label(&self) -> String {
        format!("{} {}", self.kind, self.name)
    }

    /// The path of the instance it is of: empty for the part itself.
    pub fn instance(&self) -> &str {
        split_path(&self.target).0
    }

    /// The same pick as its instance's own part names it.
    pub fn local(&self) -> Self {
        Self {
            target: split_path(&self.target).1.to_string(),
            ..self.clone()
        }
    }

    /// The same pick of the instance `instance` of the part.
    pub fn behind(&self, instance: &str) -> Self {
        Self {
            target: instance_path(instance, &self.target),
            ..self.clone()
        }
    }
}

/// The saved form of an [`ElementReference`]: its name alone in a
/// singleton target, else the target's name too.
#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum Saved {
    Name(String),
    InTarget { target: String, name: String },
    Whole { target: String },
}

impl<T: HasElements<K>, K: Kind> Serialize for ElementReference<T, K> {
    fn serialize<Se: Serializer>(&self, serializer: Se) -> Result<Se::Ok, Se::Error> {
        let target = self.target.name().to_string();
        let saved = if T::SINGLETON == Some(&target) {
            Saved::Name(self.name.clone())
        } else if self.name.is_empty() {
            Saved::Whole { target }
        } else {
            Saved::InTarget {
                target,
                name: self.name.clone(),
            }
        };
        saved.serialize(serializer)
    }
}

impl<'de, T: HasElements<K>, K: Kind> Deserialize<'de> for ElementReference<T, K> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Saved::deserialize(deserializer)? {
            Saved::Name(name) => Self::in_singleton(name).map_err(serde::de::Error::custom),
            Saved::InTarget { target, name } => Ok(Self::new(TargetReference::new(target), name)),
            Saved::Whole { target } => Ok(Self::whole(TargetReference::new(target))),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::{
        part::Part,
        test::{Pose, Sc},
    };

    /// A target of named lengths, of which a part has many.
    #[derive(Clone, PartialEq)]
    struct Lengths(BTreeMap<String, f64>);

    /// A target of named lengths, of which a part has one.
    #[derive(Clone, PartialEq)]
    struct Stock(Lengths);

    struct Length;

    impl Kind for Length {
        const NAME: &'static str = "length";
    }

    impl HasElements<Length> for Lengths {
        type Element<'a> = f64;

        fn element(&self, name: &str) -> GeopResult<f64> {
            (self.0.get(name).copied()).ok_or_else(|| GeopError::new(format!("no length {name}")))
        }
    }

    impl HasElements<Length> for Stock {
        type Element<'a> = f64;
        const SINGLETON: Option<&'static str> = Some("stock");

        fn element(&self, name: &str) -> GeopResult<f64> {
            self.0.element(name)
        }
    }

    fn lengths(entries: &[(&str, f64)]) -> Lengths {
        Lengths(entries.iter().map(|&(n, v)| (n.to_string(), v)).collect())
    }

    /// An element resolves through a reader, which makes the target reading
    /// it depend on the target it is in.
    #[test]
    fn a_step_reading_an_element_is_rebuilt_when_its_target_changes() {
        let mut part = Part::<Sc, Pose>::default();
        let total = TargetReference::<f64>::new("total");
        let run = |part: &mut Part<Sc, Pose>, a: f64| {
            part.begin_run(&BTreeMap::new(), &BTreeMap::new());
            part.begin_step();
            let defined = TargetReference::<Lengths>::new("s1");
            (part.define_target(&defined, &a, |_, &a| Ok(lengths(&[("a", a)])))).unwrap();
            part.begin_step();
            let a = ElementReference::<Lengths, Length>::new(defined, "a");
            (part.define_target(
                &total,
                &a,
                |reader, a| Ok(2.0 * reader.retrieve_element(a)?),
            ))
            .unwrap();
            part.end_run(0);
        };
        run(&mut part, 1.0);
        assert_eq!(part.current().target(&total).unwrap(), &2.0);
        run(&mut part, 3.0);
        assert_eq!(part.current().target(&total).unwrap(), &6.0);
    }

    #[test]
    fn a_missing_element_says_which() {
        let mut part = Part::<Sc, Pose>::default();
        part.begin_run(&BTreeMap::new(), &BTreeMap::new());
        part.begin_step();
        let defined = TargetReference::<Lengths>::new("s1");
        (part.define_target(&defined, &(), |_, _| Ok(lengths(&[("a", 1.0)])))).unwrap();
        part.end_run(0);
        let b = ElementReference::<Lengths, Length>::new(defined, "b");
        let error = part.current().retrieve_element(&b).unwrap_err().to_string();
        assert!(error.contains("no length b"), "{error}");
        assert!(error.contains("length(\"s1\", \"b\")"), "{error}");
    }

    /// In a singleton target an element is saved by its name alone; in any
    /// other with its target's.
    #[test]
    fn saved_forms() {
        let in_stock = ElementReference::<Stock, Length>::in_singleton("a").unwrap();
        let saved = serde_json::to_value(&in_stock).unwrap();
        assert_eq!(saved, serde_json::json!("a"));
        let read: ElementReference<Stock, Length> = serde_json::from_value(saved).unwrap();
        assert_eq!(read, in_stock);

        let in_s1 = ElementReference::<Lengths, Length>::new(TargetReference::new("s1"), "a");
        let saved = serde_json::to_value(&in_s1).unwrap();
        assert_eq!(saved, serde_json::json!({"target": "s1", "name": "a"}));
        let read: ElementReference<Lengths, Length> = serde_json::from_value(saved).unwrap();
        assert_eq!(read, in_s1);

        // A bare name needs a singleton target to be in.
        assert!(serde_json::from_value::<ElementReference<Lengths, Length>>("a".into()).is_err());
    }

    /// A pick becomes a reference only of its own kind.
    #[test]
    fn a_pick_becomes_a_reference_of_its_kind() {
        let picked = PickedElement {
            target: "s1".into(),
            kind: "length".into(),
            name: "a".into(),
        };
        let reference = ElementReference::<Lengths, Length>::from_picked(&picked).unwrap();
        assert_eq!(reference.picked(), picked);
        let other = PickedElement {
            kind: "edge".into(),
            ..picked
        };
        assert!(ElementReference::<Lengths, Length>::from_picked(&other).is_none());
    }
}
