//! A [`super::NamedModel`]'s features: what each step that combined a tool with a solid
//! did — a cut, a boss, a hole — kept so that a later step can do it again
//! elsewhere (a pattern of features), and so that a face can be traced back
//! to the step that made it.
//!
//! A feature is kept as its tools, as they were built before they were
//! combined, and how each was combined: not as the arguments of its step,
//! which a part never sees. Doing a feature again is copying its tools,
//! moving them, and combining them the same way — one boolean each, with
//! a tool going up to the next face stopping at the next face *where the
//! copy is* (see `geop_ops_model::Combine`).
//!
//! How a tool is combined — [`BooleanOp`] and [`ToolReach`] — is defined
//! here rather than with the booleans: it is the vocabulary a feature is
//! recorded in, which every step that combines tools (extrude, hole, rib,
//! ...) writes, and which a pattern or mirror reads to do the feature
//! again. The booleans take their arguments in the same terms.

use std::{collections::HashMap, sync::Arc};

use geop_core_editor::selectable::SelectableElement;
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement},
    reader::Reader,
};
use geop_core_topology::build::BodySpec;
use serde::{Deserialize, Serialize};

use super::{
    BodyNames, NamedModel,
    elements::{Face, read_model},
    names::{digest, origin_of},
};

/// Which boolean to perform.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BooleanOp {
    /// Everything in either solid.
    Union,
    /// Only what is in both.
    Intersection,
    /// `solid_a` with `solid_b` removed.
    Difference,
    /// What is in either solid but not in both.
    SymmetricDifference,
}

/// What one step did by combining tools with a solid: each tool, in the
/// order it was combined.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub tools: Vec<FeatureTool>,
}

/// One tool of a [`Feature`]: the solid as its step built it, before it was
/// combined — its geometry and the names of its entities — and how it was
/// combined with the solid it was combined with.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeatureTool {
    pub spec: BodySpec,
    pub names: BodyNames,
    pub op: BooleanOp,
    pub reach: ToolReach,
}

/// How far a tool reaches when it is combined, and how what its
/// combination creates is named: what, besides the [`BooleanOp`], a step
/// combines a tool by, and a [`Feature`] records.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ToolReach {
    /// For a tool going only up to the next face of the solid: the names
    /// of its start and end faces. None for all of the tool.
    pub up_to_next: Option<(String, String)>,
    /// What tells its combination's names apart from the step's other
    /// tools' (see [`super::Namer::scoped`]). None for the one tool of its
    /// step.
    pub scope: Option<String>,
}

impl NamedModel {
    /// Records `tool` as one more tool of the feature of the step `step`.
    pub fn record_feature_tool(&mut self, step: &str, tool: FeatureTool) {
        let features = &mut self.features;
        match features.iter_mut().find(|(s, _)| s == step) {
            Some((_, feature)) => Arc::make_mut(feature).tools.push(tool),
            None => features.push((step.to_string(), Arc::new(Feature { tools: vec![tool] }))),
        }
    }

    /// The feature of the step `step`. Fails, saying which steps are
    /// features, for a step that combined nothing.
    pub fn feature(&self, step: &str) -> GeopResult<&Feature> {
        self.features
            .iter()
            .find(|(s, _)| s == step)
            .map(|(_, f)| f.as_ref())
            .ok_or_else(|| {
                let steps: Vec<&str> = self.features.iter().map(|(s, _)| s.as_str()).collect();
                GeopError::new(format!(
                    "step {step:?} is no feature: only a step that joins, cuts or intersects a tool with a solid is (the features: {steps:?})"
                ))
            })
    }

    /// Every feature, by its step, in the order the steps ran.
    pub fn features(&self) -> impl Iterator<Item = (&str, &Feature)> {
        self.features.iter().map(|(s, f)| (s.as_str(), f.as_ref()))
    }

    /// Which feature made each face, as [`FeatureFaces::of`] tells.
    pub fn feature_faces(&self) -> FeatureFaces<'_> {
        let mut steps = HashMap::new();
        for (step, feature) in &self.features {
            for tool in &feature.tools {
                for face in &tool.names.faces {
                    steps.insert(face.clone(), step.as_str());
                    steps.insert(digest(face), step.as_str());
                }
            }
        }
        FeatureFaces { steps }
    }
}

/// Which feature made a face (see [`NamedModel::feature_faces`]).
pub struct FeatureFaces<'a> {
    /// The step of each face of a tool, by the face's name and its digest.
    steps: HashMap<String, &'a str>,
}

impl FeatureFaces<'_> {
    /// The step whose feature made the face named `face`: the face of one
    /// of its tools, or a piece of one that a boolean split off (see
    /// [`super::Namer::piece`]) however often it was split since.
    /// None for a face no feature made: one of a solid built on its own,
    /// or what a later step made of it.
    pub fn of(&self, face: &str) -> Option<&str> {
        // Each origin a shorter name than the piece's: the chain ends.
        std::iter::successors(Some(face), |&name| origin_of(name))
            .find_map(|name| self.steps.get(name).copied())
    }
}

/// What a step did by combining tools with a solid — a cut, a boss, a hole
/// — as an element of the model: named by the step, and picked by a face
/// it made (see [`FeatureFaces`]), lit as every face of it that is left. A
/// feature of a part placed in the part is picked as one of that part,
/// behind its instance.
pub struct FeatureElement;

/// A feature of the part, or of a part placed in it, by the step that
/// made it.
pub type FeatureRef = ElementReference<NamedModel, FeatureElement>;

/// The feature of the step `step`.
pub fn feature(step: impl Into<String>) -> FeatureRef {
    ElementReference::new(super::model(), step)
}

impl Kind for FeatureElement {
    const NAME: &'static str = "feature";
}

impl HasElements<FeatureElement> for NamedModel {
    type Element<'a> = &'a Feature;
    const SINGLETON: Option<&'static str> = <Self as HasElements<Face>>::SINGLETON;

    fn element(&self, name: &str) -> GeopResult<Self::Element<'_>> {
        self.feature(name)
    }
}

/// The part a pick of `instance` is of: the part `reader` reads, or the
/// part placed there.
fn part_of<'a>(reader: &Reader<'a>, instance: &str) -> Option<Reader<'a>> {
    match instance.is_empty() {
        true => Some(reader.clone()),
        false => reader.placed_part(instance).ok(),
    }
}

impl SelectableElement<NamedModel> for FeatureElement {
    const PICKED_FROM: &'static [&'static str] = &[Face::NAME];

    fn select(reader: &Reader, hit: &PickedElement) -> Option<FeatureRef> {
        let face = ElementReference::<NamedModel, Face>::from_picked(hit)?;
        let instance = face.instance().to_string();
        let part = read_model(&part_of(reader, &instance)?).ok()?;
        part.face_id(face.name()).ok()?;
        let step = part.feature_faces().of(face.name())?.to_string();
        Some(match instance.is_empty() {
            true => feature(step),
            false => feature(step).behind(&instance),
        })
    }

    fn drawn(reader: &Reader, feature: &FeatureRef) -> Vec<PickedElement> {
        let instance = feature.instance();
        let Some(part) = part_of(reader, instance).and_then(|r| read_model(&r).ok()) else {
            return Vec::new();
        };
        let made = part.feature_faces();
        let mut faces: Vec<_> = part.topology().faces.keys().copied().collect();
        faces.sort_by_key(|face| face.0);
        (faces.into_iter())
            .filter_map(|face| part.name_of(face))
            .filter(|&face| made.of(face) == Some(feature.name()))
            .map(|face| {
                let face = ElementReference::<NamedModel, Face>::new(super::model(), face);
                match instance.is_empty() {
                    true => face.picked(),
                    false => face.behind(instance).picked(),
                }
            })
            .collect()
    }
}
