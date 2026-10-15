//! [`PlacedSketch3d`]: a 3-D sketch as a part holds it — the value of its
//! target ([`sketch3d`]) — and what it is to the steps that build on it: a
//! path as a whole ([`Path`]), picked by an edge of the wire it is built
//! as.

use geop_core_editor::selectable::SelectableElement;
use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    part::Reader,
    target::TargetReference,
    values::Design,
};
use geop_ops_model::{Chain, Path, model};
use geop_ops_sketch::chain_of_pieces;
use geop_ops_view::drawn;

use crate::Sketch3d;

/// A 3-D sketch as a part holds it: its name, the id of the step that drew
/// it; the sketch, what it is given of the part brought up to date and
/// solved; and the names of the edges of the wire it is built as — what a
/// pick of one of them takes the whole sketch for, where a path is asked
/// for.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedSketch3d {
    pub name: String,
    pub sketch: Sketch3d,
    pub edges: Vec<String>,
}

/// What the target of a 3-D sketch is named, before the sketch's name.
const PREFIX: &str = "sketch3d:";

/// The 3-D sketch `name`: what the step of that id draws.
pub fn sketch3d(name: &str) -> TargetReference<PlacedSketch3d> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// The 3-D sketch `name` as a path.
pub fn sketch3d_path(name: &str) -> ElementReference<PlacedSketch3d, Path> {
    ElementReference::whole(sketch3d(name))
}

impl PlacedSketch3d {
    /// Its one chain of curves, open or closed, as a path runs along it:
    /// named after the sketch and its elements.
    pub fn path(&self) -> GeopResult<Chain<NurbCurve3D<Design>>> {
        let ctx = with_context!("the curves of the 3-D sketch {:?}", self.name);
        let chains = self.sketch.chains().with_context(ctx)?;
        let [chain] = chains.as_slice() else {
            return Err(GeopError::new(format!(
                "it has {} chains of curves: a path is one chain of curves, or one loop",
                chains.len()
            )))
            .with_context(ctx);
        };
        let geometry = self.sketch.enclose::<Design>().with_context(ctx)?;
        let pieces = chain.to_nurbs(&self.sketch, &geometry).with_context(ctx)?;
        Ok(chain_of_pieces(&self.name, pieces, chain.closed))
    }
}

/// Read whole.
impl Kind for PlacedSketch3d {
    const NAME: &'static str = "sketch3d";
}

impl HasElements<PlacedSketch3d> for PlacedSketch3d {
    type Element<'a> = &'a PlacedSketch3d;

    fn element(&self, name: &str) -> GeopResult<&PlacedSketch3d> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

impl HasElements<Path> for PlacedSketch3d {
    type Element<'a> = Chain<NurbCurve3D<Design>>;

    fn element(&self, name: &str) -> GeopResult<Chain<NurbCurve3D<Design>>> {
        whole(self, name)?.path()
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

/// A 3-D sketch is a path as a whole, picked by any edge of its wire, and
/// lit as all of them.
impl SelectableElement<PlacedSketch3d> for Path {
    const PICKED_FROM: &'static [&'static str] = &[drawn::EDGE];

    fn select(
        reader: &Reader,
        hit: &PickedElement,
    ) -> Option<ElementReference<PlacedSketch3d, Self>> {
        (hit.kind == drawn::EDGE && hit.target == model().name()).then_some(())?;
        reader
            .targets_of::<PlacedSketch3d>()
            .into_iter()
            .find(|target| {
                let whole =
                    ElementReference::<PlacedSketch3d, PlacedSketch3d>::whole(target.clone());
                (reader.retrieve_element(&whole))
                    .is_ok_and(|placed| placed.edges.contains(&hit.name))
            })
            .map(ElementReference::whole)
    }

    fn drawn(reader: &Reader, path: &ElementReference<PlacedSketch3d, Self>) -> Vec<PickedElement> {
        let whole =
            ElementReference::<PlacedSketch3d, PlacedSketch3d>::whole(path.target().clone());
        let edges = (reader.retrieve_element(&whole).ok()).map(|placed| placed.edges.clone());
        (edges.unwrap_or_default().into_iter())
            .map(|edge| PickedElement {
                target: model().name().to_string(),
                kind: drawn::EDGE.to_string(),
                name: edge,
            })
            .collect()
    }
}
