//! Patterns of features: copies of what earlier steps did to a solid — a
//! cut, a boss, a hole with its thread — done again elsewhere.
//!
//! # Copied tools, not steps run again
//!
//! A feature is done again by copying the tools its step combined, as that
//! step built them, moving the copies and combining each the same way (see
//! [`geop_ops::Feature`]) — not by running the step again with moved
//! arguments. A step's arguments are the program's, which a part never
//! sees, and running one again would mean moving a sketch, a face picked by
//! name, the points a hole is drilled at — each operation in its own way,
//! and with the names of what it picked meaning nothing where the copy is.
//! A tool needs none of that: it is a solid, moved exactly as a body is
//! (see `geop_core_topology::Model::transform_body`), and combined by the
//! one boolean every combining step already uses. A sketch is not solved
//! again, and every operation that combines a tool — extrudes, revolves,
//! sweeps, lofts, holes, ribs, patterns themselves — can be patterned
//! without a line of its own.
//!
//! What that gives up is an end chosen where the copy is. A tool going up
//! to the next face is recorded as reaching far, with its start and end
//! faces named: its copy is cut back to the next face *where it is*, as
//! the step did it (see `geop_ops_booleans::Combine::apply`). A tool going
//! through all reaches as far past the solid as it did where it was built,
//! and a blind one is as deep: a copy where the solid is thicker in the
//! direction of the tool goes as far as the original went, not further.
//! An extrude's fallbacks when nothing stops it all round (going up to the
//! first contact, or all the way) are not tried again for a copy: a copy
//! that nothing stops all round is refused.

use std::collections::BTreeSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::{NamedModel, operation::EntityRef};
use geop_ops_booleans::{Combine, Tool, boolean::Reach};

use crate::common::{Names, middle};

/// The steps of the features `refs` refer to, each once, in the order the
/// steps ran — the order a copy of each must be combined in, for a hole cut
/// through a boss to cut its copy too. Anything but a feature is refused.
pub(crate) fn feature_steps<S: Scalar>(
    part: &NamedModel<S>,
    refs: &[EntityRef],
) -> GeopResult<Vec<String>> {
    let mut picked = BTreeSet::new();
    for entity in refs {
        match entity {
            EntityRef::Feature { name } => {
                part.feature(name)?;
                picked.insert(name.as_str());
            }
            other => {
                return Err(GeopError::new(format!(
                    "{other} is no feature: pick a feature by a face it made"
                )));
            }
        }
    }
    Ok(part
        .features()
        .map(|(step, _)| step)
        .filter(|step| picked.contains(step))
        .map(str::to_string)
        .collect())
}

/// The name of the solid the features of `steps` lie on: the one bounded
/// by faces they made. Refused if they made none that is still there, or
/// lie on several.
fn feature_solid<S: Scalar>(part: &NamedModel<S>, steps: &[String]) -> GeopResult<String> {
    let made = part.feature_faces();
    let model = part.topology();
    let mut solids = BTreeSet::new();
    for &face in model.faces.keys() {
        let Some(name) = part.name_of(face) else {
            continue;
        };
        if made
            .of(name)
            .is_some_and(|step| steps.iter().any(|s| s == step))
            && let Body::Solid(solid) = model.body_of_face(face)?
            && let Some(solid) = part.name_of(solid)
        {
            solids.insert(solid.to_string());
        }
    }
    let mut solids = solids.into_iter();
    match (solids.next(), solids.next()) {
        (Some(solid), None) => Ok(solid),
        (None, _) => Err(GeopError::new(format!(
            "nothing the features {steps:?} made is left on a solid: there is nothing to pattern them on"
        ))),
        (Some(a), Some(b)) => Err(GeopError::new(format!(
            "the features {steps:?} lie on more than one solid ({a:?}, {b:?}, ...): pattern the features of each solid in a step of its own"
        ))),
    }
}

/// The middle of the corners of the tools of the features `refs` refers
/// to, where they were built: where a pattern of them shows its handles.
/// None, if it refers to none.
pub(crate) fn features_center<S: Scalar>(
    part: &NamedModel<S>,
    refs: &[EntityRef],
) -> Option<Vector3<S>> {
    let mut corners = Vec::new();
    for step in feature_steps(part, refs).ok()? {
        for tool in &part.feature(&step).ok()?.tools {
            corners.extend(tool.spec.vertices.iter().copied());
        }
    }
    middle(corners)
}

/// Does the features `refs` refers to again at every placement — a label
/// and the motion from where the features are — as the step
/// `operation_id`, whose names `names` gives: every tool of each copied,
/// the copy of each entity named `X` named `names.copy(label, X)`, moved,
/// and combined with the solid the features lie on as its feature combined
/// it. The result is named `names.namer`'s root, and what each copy's combination
/// creates `combine(P,label,F,...)` for the feature `F` (and the scope its
/// tool had in `F`). The cosmetic threads a feature recorded are the
/// threads' to copy (see [`crate::common::run_pattern`]).
///
/// The features are done again in the order their steps ran, the copies
/// of each feature's tools combined one after the other.
pub(crate) fn repeat_features<S: Scalar>(
    part: &mut NamedModel<S>,
    names: &Names,
    operation_id: &str,
    refs: &[EntityRef],
    placements: &[(String, Motion<S>)],
) -> GeopResult<()> {
    let namer = &names.namer;
    let steps = feature_steps(part, refs)?;
    let mut target = feature_solid(part, &steps)?;
    for step in &steps {
        let ctx = with_context!("patterning feature {step:?}");
        let feature = part.feature(step).with_context(ctx)?.clone();
        for group in feature.tools.chunk_by(|a, b| a.op == b.op) {
            let mut tools = Vec::with_capacity(group.len() * placements.len());
            for (label, motion) in placements {
                for tool in group {
                    let copied = tool.names.renamed(|name| names.copy(label, name));
                    let built = part
                        .build_body(tool.spec.clone(), copied)
                        .with_context(ctx)?;
                    let solid = built.solid.ok_or_else(|| {
                        GeopError::new(format!("a tool of feature {step:?} is no solid"))
                    })?;
                    part.transform_body(Body::Solid(solid), motion)
                        .with_context(ctx)?;
                    let scope = match &tool.scope {
                        Some(scope) => format!("{label},{step},{scope}"),
                        None => format!("{label},{step}"),
                    };
                    tools.push(Tool {
                        solid,
                        up_to_next: tool
                            .up_to_next
                            .as_ref()
                            .map(|(start, end)| (names.copy(label, start), names.copy(label, end))),
                        scope: Some(scope),
                    });
                }
            }
            let combine = Combine::With {
                op: group[0].op,
                target,
            };
            combine
                .apply(part, namer, operation_id, &tools)
                .and_then(Reach::stopped)
                .with_context(ctx)?;
            target = namer.root();
        }
    }
    Ok(())
}
