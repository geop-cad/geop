//! The kinds of element a step refers to in the part's model ([`model`]):
//! a [`Vertex`], an [`Edge`], a [`Face`], a [`Solid`] — each an
//! [`ElementReference`] into the model by its stable name — an
//! [`EdgeChain`], the tangent chain of edges through an edge, and the model
//! itself, read whole. Each says how a dialog picks it
//! ([`SelectableElement`]): a vertex, an edge or a face as itself, a solid
//! by a face of it, an edge chain by an edge of it.

use geop_core_editor::selectable::SelectableElement;
use geop_core_editor::view::drawn;
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    placed::Located,
    reader::Reader,
    values::Design,
};
use geop_core_topology::{EdgeId, FaceId, SolidId, VertexId};

use crate::{
    NamedModel, RefId, model,
    tangent::{TangentChain, tangent_chain},
};

/// The name the part's model is a target by: there is one.
const MODEL: &str = "model";

/// An element of the model: the model, and the entity's id in it.
#[derive(Clone, Copy, Debug)]
pub struct ModelElement<'a, Id> {
    pub model: &'a NamedModel<Design>,
    pub id: Id,
}

/// A vertex of the model.
pub struct Vertex;

/// An edge of the model, of a solid, a sheet or a wire.
pub struct Edge;

/// A face of the model.
pub struct Face;

/// A solid of the model, as a whole.
pub struct Solid;

/// The tangent chain of edges through an edge of the model (see
/// [`tangent_chain`]): named by any edge of it, and selected by the
/// smallest name of its edges, the one name a dialog holds it by however
/// it was picked.
pub struct EdgeChain;

impl Kind for Vertex {
    const NAME: &'static str = drawn::VERTEX;
}

impl Kind for Edge {
    const NAME: &'static str = drawn::EDGE;
}

impl Kind for Face {
    const NAME: &'static str = drawn::FACE;
}

impl Kind for Solid {
    const NAME: &'static str = "solid";
}

impl Kind for EdgeChain {
    const NAME: &'static str = "edge_chain";
}

/// An edge chain of the model: the model, and the chain.
#[derive(Clone, Debug)]
pub struct ChainElement<'a> {
    pub model: &'a NamedModel<Design>,
    pub chain: TangentChain,
}

impl ChainElement<'_> {
    /// The names of its edges, in the order the chain runs.
    pub fn edge_names(&self) -> GeopResult<Vec<&str>> {
        (self.chain.edges().into_iter())
            .map(|edge| self.model.name(edge))
            .collect()
    }
}

impl HasElements<EdgeChain> for NamedModel<Design> {
    type Element<'a> = ChainElement<'a>;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<ChainElement<'_>> {
        let chain = tangent_chain(self.topology(), self.edge_id(name)?)?;
        Ok(ChainElement { model: self, chain })
    }
}

/// The model read whole: a kind of its own, its one element the empty
/// name.
impl Kind for NamedModel<Design> {
    const NAME: &'static str = "model";
}

impl HasElements<NamedModel<Design>> for NamedModel<Design> {
    type Element<'a> = &'a NamedModel<Design>;
    const SINGLETON: Option<&'static str> = Some(MODEL);

    fn element(&self, name: &str) -> GeopResult<&NamedModel<Design>> {
        whole(self, name)
    }
}

/// Elements of the model of the kind `K`, by the id `id` its names have.
macro_rules! named_elements {
    ($($kind:ty => $id:ty, $lookup:ident;)*) => {$(
        impl HasElements<$kind> for NamedModel<Design> {
            type Element<'a> = ModelElement<'a, $id>;
            const SINGLETON: Option<&'static str> = Some(MODEL);

            fn element(&self, name: &str) -> GeopResult<ModelElement<'_, $id>> {
                Ok(ModelElement {
                    model: self,
                    id: self.$lookup(name)?,
                })
            }
        }
    )*};
}

named_elements! {
    Vertex => VertexId, vertex_id;
    Edge => EdgeId, edge_id;
    Face => FaceId, face_id;
    Solid => SolidId, solid_id;
}

/// The model as `reader` reads it: empty, before the first step that
/// builds any topology.
pub fn read_model<'a>(reader: &Reader<'a>) -> GeopResult<std::borrow::Cow<'a, NamedModel<Design>>> {
    if !reader.defines(&model()) {
        return Ok(std::borrow::Cow::Owned(NamedModel::new()));
    }
    let whole = ElementReference::<NamedModel<Design>, NamedModel<Design>>::whole(model());
    Ok(std::borrow::Cow::Borrowed(reader.retrieve_element(&whole)?))
}

/// What the editor draws of the model's entity `name`: the entity itself,
/// a vertex, an edge or a face — or, for a solid, its faces.
pub(crate) fn drawn_entity(model: &NamedModel<Design>, name: &str) -> Vec<PickedElement> {
    let picked = |kind: &str, name: &str| PickedElement {
        target: Located::new(MODEL),
        kind: kind.to_string(),
        name: name.to_string(),
    };
    match model.id_of(name) {
        Some(RefId::Vertex(_)) => vec![picked(Vertex::NAME, name)],
        Some(RefId::Edge(_)) => vec![picked(Edge::NAME, name)],
        Some(RefId::Face(_)) => vec![picked(Face::NAME, name)],
        Some(RefId::Solid(solid)) => (model.topology().solid_faces(solid))
            .unwrap_or_default()
            .into_iter()
            .filter_map(|face| model.name_of(face))
            .map(|face| picked(Face::NAME, face))
            .collect(),
        None => Vec::new(),
    }
}

/// An element drawn as itself: a click on it selects it.
macro_rules! drawn_as_itself {
    ($($kind:ty),*) => {$(
        impl SelectableElement<NamedModel<Design>> for $kind {
            const PICKED_FROM: &'static [&'static str] = &[<$kind as Kind>::NAME];

            fn select(
                _: &Reader,
                hit: &PickedElement,
            ) -> Option<ElementReference<NamedModel<Design>, Self>> {
                ElementReference::from_picked(hit)
            }
        }
    )*};
}

drawn_as_itself!(Vertex, Edge, Face);

/// A solid is picked by a face of it, and lit as all of its faces.
impl SelectableElement<NamedModel<Design>> for Solid {
    const PICKED_FROM: &'static [&'static str] = &[Face::NAME];

    fn select(
        reader: &Reader,
        hit: &PickedElement,
    ) -> Option<ElementReference<NamedModel<Design>, Self>> {
        let face = ElementReference::<NamedModel<Design>, Face>::from_picked(hit)?;
        let face = reader.retrieve_element(&face).ok()?;
        let solid = face.model.topology().solid_of_face(face.id).ok()??;
        let name = face.model.name_of(solid)?;
        Some(ElementReference::new(model(), name))
    }

    fn drawn(
        reader: &Reader,
        solid: &ElementReference<NamedModel<Design>, Self>,
    ) -> Vec<PickedElement> {
        (read_model(reader).ok())
            .map(|model| drawn_entity(&model, solid.name()))
            .unwrap_or_default()
    }
}

/// An edge chain is picked by an edge of it, held by the smallest name of
/// its edges, and lit as all of them.
impl SelectableElement<NamedModel<Design>> for EdgeChain {
    const PICKED_FROM: &'static [&'static str] = &[Edge::NAME];

    fn select(
        reader: &Reader,
        hit: &PickedElement,
    ) -> Option<ElementReference<NamedModel<Design>, Self>> {
        let through = ElementReference::<NamedModel<Design>, Self>::new(model(), &hit.name);
        (hit.kind == Edge::NAME).then_some(())?;
        let chain = reader.retrieve_element(&through).ok()?;
        let first = chain.edge_names().ok()?.into_iter().min()?;
        Some(ElementReference::new(model(), first))
    }

    fn drawn(
        reader: &Reader,
        chain: &ElementReference<NamedModel<Design>, Self>,
    ) -> Vec<PickedElement> {
        let Ok(chain) = reader.retrieve_element(chain) else {
            return Vec::new();
        };
        (chain.edge_names().unwrap_or_default().into_iter())
            .map(|name| ElementReference::<NamedModel<Design>, Edge>::new(model(), name).picked())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use geop_core_editor::selectable::selected;
    use geop_core_program::part::Part;

    use super::*;

    /// A part of one step whose model is a unit cube, every entity named
    /// by its kind and its position among those: `e0` … `e11`.
    fn cube() -> Part {
        crate::testing::given_part(&crate::testing::unit_cube()).unwrap()
    }

    fn edge(name: &str) -> PickedElement {
        ElementReference::<NamedModel<Design>, Edge>::new(model(), name).picked()
    }

    /// An edge chain is picked by clicking an edge of it, held by the
    /// smallest name of its edges, lit as them, and taken out by clicking
    /// one again. On a cube, whose faces meet square, every edge is a
    /// chain of its own.
    #[test]
    fn an_edge_chain_is_picked_by_an_edge() {
        let part = cube();
        let reader = part.current();
        let chain =
            |name: &str| ElementReference::<NamedModel<Design>, EdgeChain>::new(model(), name);
        let picked: Vec<ElementReference<NamedModel<Design>, EdgeChain>> =
            selected(&reader, &[edge("e3")]);
        assert_eq!(picked, [chain("e3")]);
        assert_eq!(EdgeChain::drawn(&reader, &chain("e3")), [edge("e3")]);
        let both: Vec<ElementReference<NamedModel<Design>, EdgeChain>> =
            selected(&reader, &[chain("e3").picked(), edge("e5")]);
        assert_eq!(both, [chain("e3"), chain("e5")]);
        let again: Vec<ElementReference<NamedModel<Design>, EdgeChain>> =
            selected(&reader, &[chain("e3").picked(), edge("e3")]);
        assert_eq!(
            again,
            Vec::<ElementReference<NamedModel<Design>, EdgeChain>>::new()
        );
    }

    fn face(name: &str) -> ElementReference<NamedModel<Design>, Face> {
        ElementReference::new(model(), name)
    }

    /// Every kind resolves by name in the one model, and is saved by its
    /// name alone.
    #[test]
    fn elements_resolve_by_name() {
        let part = cube();
        let reader = part.current();
        let vertex = ElementReference::<NamedModel<Design>, Vertex>::in_singleton("v0").unwrap();
        let found = reader.retrieve_element(&vertex).unwrap();
        assert_eq!(found.model.name_of(found.id), Some("v0"));
        assert!(reader.retrieve_element(&face("f6")).is_err());
        assert_eq!(
            serde_json::to_value(face("f0")).unwrap(),
            serde_json::json!("f0")
        );
        assert_eq!(read_model(&reader).unwrap().solid_names(), ["s0"]);
        let empty = Part::default();
        assert!(
            read_model(&empty.current())
                .unwrap()
                .solid_names()
                .is_empty()
        );
    }

    /// A solid is picked by a face of it, and lit as its faces.
    #[test]
    fn a_solid_is_picked_by_its_faces() {
        let part = cube();
        let reader = part.current();
        let solid = ElementReference::<NamedModel<Design>, Solid>::new(model(), "s0");
        let picked: Vec<ElementReference<NamedModel<Design>, Solid>> =
            selected(&reader, &[face("f0").picked()]);
        assert_eq!(picked, std::slice::from_ref(&solid));
        let faces: Vec<PickedElement> = (0..6).map(|k| face(&format!("f{k}")).picked()).collect();
        let drawn = Solid::drawn(&reader, &solid);
        assert_eq!(drawn.len(), faces.len());
        assert!(faces.iter().all(|f| drawn.contains(f)));
        assert_eq!(Solid::PICKED_FROM, ["face"]);
    }
}
