//! A [`Part`]'s features: what each step that combined a tool with a solid
//! did — a cut, a boss, a hole — kept so that a later step can do it again
//! elsewhere (a pattern of features), and so that a face can be traced back
//! to the step that made it.
//!
//! A feature is kept as its tools, as they were built before they were
//! combined, and how each was combined: not as the arguments of its step,
//! which a part never sees. Doing a feature again is copying its tools,
//! moving them, and combining them the same way — one boolean each, with
//! a tool going up to the next face stopping at the next face *where the
//! copy is* (see `geop_ops_booleans::Combine`).

use std::{collections::HashMap, sync::Arc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::build::BodySpec;
use serde::{Deserialize, Serialize};

use super::{BodyNames, Part, names::digest};

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
}

/// What one step did by combining tools with a solid: each tool, in the
/// order it was combined.
#[derive(Clone, Debug, Default)]
pub struct Feature<S: Scalar> {
    pub tools: Vec<FeatureTool<S>>,
}

/// One tool of a [`Feature`]: the solid as its step built it, before it was
/// combined — its geometry and the names of its entities — and how it was
/// combined with the solid it was combined with.
#[derive(Clone, Debug)]
pub struct FeatureTool<S: Scalar> {
    pub spec: BodySpec<S>,
    pub names: BodyNames,
    pub op: BooleanOp,
    /// For a tool going only up to the next face of the solid: the names
    /// of its start and end faces.
    pub up_to_next: Option<(String, String)>,
    /// What told its combination's names apart from the step's other
    /// tools'.
    pub scope: Option<String>,
}

impl<S: Scalar> Part<S> {
    /// Records `tool` as one more tool of the feature of the step `step`.
    pub fn record_feature_tool(&mut self, step: &str, tool: FeatureTool<S>) {
        match self.features.iter_mut().find(|(s, _)| s == step) {
            Some((_, feature)) => Arc::make_mut(feature).tools.push(tool),
            None => self
                .features
                .push((step.to_string(), Arc::new(Feature { tools: vec![tool] }))),
        }
    }

    /// The feature of the step `step`. Fails, saying which steps are
    /// features, for a step that combined nothing.
    pub fn feature(&self, step: &str) -> GeopResult<&Feature<S>> {
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
    pub fn features(&self) -> impl Iterator<Item = (&str, &Feature<S>)> {
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

/// Which feature made a face (see [`Part::feature_faces`]).
pub struct FeatureFaces<'a> {
    /// The step of each face of a tool, by the face's name and its digest.
    steps: HashMap<String, &'a str>,
}

impl FeatureFaces<'_> {
    /// The step whose feature made the face named `face`: the face of one
    /// of its tools, or a piece of one that a boolean split off — named
    /// after it, first of the names in its own (see
    /// `geop_ops_booleans::naming`) — however often it was split since.
    /// None for a face no feature made: one of a solid built on its own,
    /// or what a later step made of it.
    pub fn of(&self, face: &str) -> Option<&str> {
        let mut name = face;
        loop {
            if let Some(step) = self.steps.get(name) {
                return Some(step);
            }
            name = origin(name)?;
        }
    }
}

/// What the entity named `name` was split off: the first of the names in
/// its arguments — an argument that is a name itself, `kind(...)`, or a
/// long one's digest. None, if it has none.
fn origin(name: &str) -> Option<&str> {
    let open = name.find('(')?;
    let inner = name[open + 1..].strip_suffix(')')?;
    let mut depth = 0usize;
    let mut start = 0usize;
    // The first argument is the step's id, never a name.
    let mut first = true;
    for (k, c) in inner
        .char_indices()
        .chain(std::iter::once((inner.len(), ',')))
    {
        match c {
            '(' => depth += 1,
            ')' => depth = depth.saturating_sub(1),
            ',' if depth == 0 => {
                let argument = &inner[start..k];
                if !first && (argument.contains('(') || argument.starts_with('#')) {
                    return Some(argument);
                }
                first = false;
                start = k + 1;
            }
            _ => {}
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn origin_is_the_first_name_among_the_arguments() {
        assert_eq!(
            origin("combine(holes,3,linear_pattern(holes,3,extrude(pin,c1)),extrude(plate,end))"),
            Some("linear_pattern(holes,3,extrude(pin,c1))")
        );
        assert_eq!(origin("combine(e,#0123,x)"), Some("#0123"));
        assert_eq!(origin("extrude(plate,end)"), None);
        assert_eq!(origin("extrude(plate)"), None);
    }
}
