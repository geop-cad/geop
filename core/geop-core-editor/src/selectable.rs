//! [`Selectable`]: what a reference field of a dialog holds — an element of
//! a kind the user picks ([`SelectableElement`]), or one of several
//! ([`one_of!`](crate::one_of)) — and how a click on what the editor draws
//! selects one, with [`selected`], the values a reference field's picks
//! come to.
//!
//! An editor hit-tests what it draws: vertices, edges, faces, sketch
//! curves. What an operation asks for may be something else drawn as
//! those — the tangent chain of edges an edge is part of, the solid a
//! face bounds, the line a straight edge runs along. The crate defining
//! the element says how a click on what is drawn selects it, and what of
//! it to light while it is selected; the editor stays the same for all of
//! them.

use geop_core_math::{scalars::Scalar, vector::Vector3};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement},
    part::Reader,
};

use crate::editing::{Pointer, hit::Scene};

/// A kind of element of the target `T` a reference field picks.
pub trait SelectableElement<T: HasElements<Self>>: Kind + Sized {
    /// The kinds of what an editor draws that a click picks one from (see
    /// [`crate::editing::hit::Scene::pick`]): its own kind, for an element
    /// drawn as itself.
    const PICKED_FROM: &'static [&'static str];

    /// The element a click on `hit`, of one of [`Self::PICKED_FROM`],
    /// selects, read through `reader`: `hit` itself, for an element drawn
    /// as itself; none, for a hit that selects none.
    fn select(reader: &Reader, hit: &PickedElement) -> Option<ElementReference<T, Self>>;

    /// What an editor draws of `element`, to light while it is selected:
    /// the element itself, by default.
    fn drawn(reader: &Reader, element: &ElementReference<T, Self>) -> Vec<PickedElement> {
        let _ = reader;
        vec![element.picked()]
    }
}

/// What a reference field holds, one value per pick.
pub trait Selectable: Clone + PartialEq + 'static {
    /// The kinds of what an editor draws that a click picks one from.
    fn picked_from() -> Vec<&'static str>;

    /// The value `picked` names as [`Selectable::picked`] does: what a
    /// field sent back held already.
    fn from_picked(picked: &PickedElement) -> Option<Self>;

    /// The value as a pick names it: what the front end shows held.
    fn picked(&self) -> PickedElement;

    /// The value a click on `hit` selects, read through `reader`: none, for
    /// a hit that selects none.
    fn select(reader: &Reader, hit: &PickedElement) -> Option<Self>;

    /// What an editor draws of it, to light while it is held.
    fn drawn(&self, reader: &Reader) -> Vec<PickedElement>;
}

/// An element of an instance is selected and drawn as the instance's own
/// part selects and draws it, behind the instance's path: what the part is
/// placed in does not discriminate where an element is from.
impl<T: HasElements<K>, K: SelectableElement<T>> Selectable for ElementReference<T, K> {
    fn picked_from() -> Vec<&'static str> {
        K::PICKED_FROM.to_vec()
    }

    fn from_picked(picked: &PickedElement) -> Option<Self> {
        ElementReference::from_picked(picked)
    }

    fn picked(&self) -> PickedElement {
        ElementReference::picked(self)
    }

    fn select(reader: &Reader, hit: &PickedElement) -> Option<Self> {
        let instance = hit.instance();
        if instance.is_empty() {
            return K::select(reader, hit);
        }
        let inside = reader.inside(instance).ok()?;
        K::select(&inside, &hit.local()).map(|selected| selected.behind(instance))
    }

    fn drawn(&self, reader: &Reader) -> Vec<PickedElement> {
        let instance = self.instance();
        if instance.is_empty() {
            return K::drawn(reader, self);
        }
        let Ok(inside) = reader.inside(instance) else {
            return vec![self.picked()];
        };
        (K::drawn(&inside, &self.local()).into_iter())
            .map(|drawn| drawn.behind(instance))
            .collect()
    }
}

/// The values a reference field holds after an event sent it `picks`: what
/// it held, as it held it, and the clicks an editor added to it, each the
/// value it selects (see [`Selectable::select`]). A click that selects a
/// value held already takes it out again; one that selects none is
/// dropped.
pub fn selected<P: Selectable>(reader: &Reader, picks: &[PickedElement]) -> Vec<P> {
    let mut held: Vec<P> = Vec::new();
    for pick in picks {
        if let Some(value) = P::from_picked(pick) {
            held.push(value);
            continue;
        }
        let Some(value) = P::select(reader, pick) else {
            continue;
        };
        match held.iter().position(|h| *h == value) {
            Some(at) => {
                held.remove(at);
            }
            None => held.push(value),
        }
    }
    held
}

/// What a click with `pointer` on what the editor draws, `scene`, selects
/// as a `P` in the part `reader` reads — for a step that picks on the
/// canvas itself rather than through a reference field — and where it hit
/// what is drawn.
pub fn pick<P: Selectable, N: Scalar>(
    scene: &dyn Scene<N>,
    reader: &Reader,
    pointer: &Pointer<N>,
) -> Option<(P, Vector3<N>)> {
    let accepts: Vec<String> = (P::picked_from().into_iter()).map(String::from).collect();
    let hit = scene.pick(pointer, &accepts)?;
    Some((P::select(reader, &hit.element)?, hit.point))
}

/// Declares an enum of references, each variant one source of what an
/// operation takes — a line from a straight edge, a datum axis or a sketch
/// line — given `-> Out`, all resolving to the same value, `Out`
/// (`resolve`). Saved by its variant's
/// name: `{"edge": "extrude(e,end)"}`. It is [`Selectable`]: a click
/// selects the first variant that selects it. A value held is told from a
/// click by its kind, so no variant may be of a kind drawn as itself that
/// another variant selects from — a field of edges or chains would take a
/// click on an edge for an edge held. The crate using it depends on
/// `serde`.
///
/// ```ignore
/// one_of! {
///     /// What a revolve turns about.
///     pub enum AxisRef -> Axis<Design> {
///         Edge(ElementReference<NamedModel<Design>, Line>),
///         Datum(ElementReference<Datums, Line>),
///     }
/// }
/// ```
#[macro_export]
macro_rules! one_of {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident -> $out:ty {
            $($(#[$variant_meta:meta])* $variant:ident($reference:ty)),+ $(,)?
        }
    ) => {
        $crate::one_of! {
            $(#[$meta])*
            $vis enum $name {
                $($(#[$variant_meta])* $variant($reference)),+
            }
        }

        impl $name {
            /// What it refers to, read through `reader`, of the part itself
            /// (see [`geop_core_program::part::Reader::retrieve_element`]).
            pub fn resolve(
                &self,
                reader: &$crate::__program::part::Reader,
            ) -> $crate::__GeopResult<$out> {
                match self {
                    $(Self::$variant(reference) => {
                        reader.retrieve_element(reference).map(::std::convert::Into::into)
                    })+
                }
            }

            /// What it refers to, of the part itself or of an instance
            /// placed in it, as it is in its own part (see
            /// [`geop_core_program::part::Reader::retrieve_placed`]).
            #[allow(dead_code)]
            pub fn resolve_placed(
                &self,
                reader: &$crate::__program::part::Reader,
            ) -> $crate::__GeopResult<$crate::__program::instance::InInstance<$out>> {
                match self {
                    $(Self::$variant(reference) => {
                        let placed = reader.retrieve_placed(reference)?;
                        Ok($crate::__program::instance::InInstance {
                            element: placed.element.into(),
                            instance: placed.instance,
                        })
                    })+
                }
            }
        }
    };
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident {
            $($(#[$variant_meta:meta])* $variant:ident($reference:ty)),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        #[serde(rename_all = "snake_case")]
        $vis enum $name {
            $($(#[$variant_meta])* $variant($reference)),+
        }

        #[allow(dead_code)]
        impl $name {
            /// The path of the instance it is of: empty for the part itself.
            pub fn instance(&self) -> &str {
                match self {
                    $(Self::$variant(reference) => reference.instance(),)+
                }
            }

            /// The same, as its instance's own part names it.
            pub fn local(&self) -> Self {
                match self {
                    $(Self::$variant(reference) => Self::$variant(reference.local()),)+
                }
            }

            /// The same, of the instance `instance` of the part.
            pub fn behind(&self, instance: &str) -> Self {
                match self {
                    $(Self::$variant(reference) => Self::$variant(reference.behind(instance)),)+
                }
            }
        }

        impl $crate::selectable::Selectable for $name {
            fn picked_from() -> ::std::vec::Vec<&'static str> {
                let mut kinds = ::std::vec::Vec::new();
                $(for kind in <$reference as $crate::selectable::Selectable>::picked_from() {
                    if !kinds.contains(&kind) {
                        kinds.push(kind);
                    }
                })+
                kinds
            }

            fn from_picked(picked: &$crate::__program::element::PickedElement) -> Option<Self> {
                $(if let Some(reference) =
                    <$reference as $crate::selectable::Selectable>::from_picked(picked)
                {
                    return Some(Self::$variant(reference));
                })+
                None
            }

            fn picked(&self) -> $crate::__program::element::PickedElement {
                match self {
                    $(Self::$variant(reference) => {
                        $crate::selectable::Selectable::picked(reference)
                    })+
                }
            }

            fn select(
                reader: &$crate::__program::part::Reader,
                hit: &$crate::__program::element::PickedElement,
            ) -> Option<Self> {
                $(if let Some(reference) =
                    <$reference as $crate::selectable::Selectable>::select(reader, hit)
                {
                    return Some(Self::$variant(reference));
                })+
                None
            }

            fn drawn(
                &self,
                reader: &$crate::__program::part::Reader,
            ) -> ::std::vec::Vec<$crate::__program::element::PickedElement> {
                match self {
                    $(Self::$variant(reference) => {
                        $crate::selectable::Selectable::drawn(reference, reader)
                    })+
                }
            }
        }
    };
}
