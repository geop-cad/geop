//! [`combine_tools`]: what an extrude, a revolve, a hole or a pattern does with
//! the tools it built, as the [`Combine`] of its step says — kept as a new
//! body, or combined with a solid of the part by the boolean.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::{Body, SolidId};
use geop_ops_model::{BooleanOp, Combine, FeatureTool, NamedModel, Namer, Tool};

use crate::{
    boolean::{Reach, boolean, boolean_up_to_next, piece_up_to_next},
    remesh::RemeshParams,
};

/// Combines the `tools` built, one after the other, as `how` says, as the
/// step `step`, whose names `namer` builds: the result is named `namer`'s
/// root, and what the `k`-th tool's combination creates is named after
/// `combine(E)` — scoped by the tool's own scope, if it has one. An
/// empty result — an intersection with a solid the new one does not
/// touch — leaves no solid, and no tool. A tool going up to the next
/// face that nothing stops all round [`Reach::RunsOn`], and leaves
/// `model` part way through: a caller going another way tries on a copy.
///
/// A new body is the tools as built — a tool going up to the next face
/// of whichever other solid it meets first (see [`piece_up_to_next`]),
/// those solids left as they are — joined into one.
///
/// Combined with a target, each tool is recorded, as built, as the
/// feature of the step (see [`geop_ops_model::Feature`]): what a pattern of
/// the feature copies and combines the same way. A new body is no
/// feature: it is a body, patterned as one.
pub fn combine_tools<S: Scalar>(
    model: &mut NamedModel<S>,
    namer: &Namer,
    step: &str,
    tools: &[Tool],
    how: &Combine,
) -> GeopResult<Reach<()>> {
    let (op, target) = match how {
        Combine::NewBody => return new_body(model, namer, step, tools),
        Combine::With { op, target } => (*op, target),
    };
    let ctx = with_context!("combining with {:?} ({op:?})", target.name());
    let mut target = model.solid_id(target.name()).with_context(ctx)?;
    for tool in tools {
        let (spec, names) = model.body_record(tool.solid.into()).with_context(ctx)?;
        model.record_feature_tool(
            step,
            FeatureTool {
                spec,
                names,
                op,
                reach: tool.reach.clone(),
            },
        );
    }
    for (k, tool) in tools.iter().enumerate() {
        let combine = tool_namer(tool, step)?;
        let params = RemeshParams::default();
        let result = match &tool.reach.up_to_next {
            None => boolean(model, &combine, target, tool.solid, op, params),
            Some((start, end)) => match boolean_up_to_next(
                model,
                &combine,
                target,
                tool.solid,
                op,
                (start, end),
                params,
            )
            .with_context(ctx)?
            {
                Reach::Stopped(result) => Ok(result),
                Reach::RunsOn(e) => return Ok(Reach::RunsOn(e)),
            },
        }
        .with_context(ctx)?;
        let Some(result) = result else {
            // Nothing left to combine the other tools with.
            let rest: Vec<Body> = tools[k + 1..].iter().map(|t| t.solid.into()).collect();
            model.assemble_solid(&rest, &[], namer.root())?;
            return Ok(Reach::Stopped(()));
        };
        model.rename(result, namer.root())?;
        target = result;
    }
    Ok(Reach::Stopped(()))
}

/// The new body `tools` make, named `namer`'s root (see [`combine_tools`]).
fn new_body<S: Scalar>(
    model: &mut NamedModel<S>,
    namer: &Namer,
    step: &str,
    tools: &[Tool],
) -> GeopResult<Reach<()>> {
    let built: Vec<SolidId> = tools.iter().map(|t| t.solid).collect();
    let mut stops: Vec<SolidId> = (model.topology().solids.keys().copied())
        .filter(|s| !built.contains(s))
        .collect();
    stops.sort_by_key(|s| s.0);
    let params = RemeshParams::default();
    let mut body: Option<SolidId> = None;
    for tool in tools {
        let combine = tool_namer(tool, step)?;
        let piece = match &tool.reach.up_to_next {
            None => tool.solid,
            Some(_) if stops.is_empty() => {
                return Err(GeopError::new(
                    "up to next: there is no other solid to go up to the next face of",
                ));
            }
            Some((start, end)) => {
                match piece_up_to_next(model, &combine, tool.solid, &stops, (start, end), params)? {
                    Reach::Stopped(piece) => piece,
                    Reach::RunsOn(e) => return Ok(Reach::RunsOn(e)),
                }
            }
        };
        body = Some(match body {
            None => piece,
            Some(body) => boolean(
                model,
                &combine.scoped("join"),
                body,
                piece,
                BooleanOp::Union,
                params,
            )?
            .ok_or_else(|| GeopError::new("joining the sides of a new body left nothing"))?,
        });
    }
    if let Some(body) = body
        && model.name_of(body) != Some(namer.root().as_str())
    {
        model.rename(body, namer.root())?;
    }
    Ok(Reach::Stopped(()))
}

/// The names of what combining `tool` as the step `step` creates:
/// `combine(step)`, scoped by its reach's scope, if it has one.
fn tool_namer(tool: &Tool, step: &str) -> GeopResult<Namer> {
    let combine = Namer::new("combine", step)?;
    Ok(match &tool.reach.scope {
        Some(scope) => combine.scoped(scope),
        None => combine,
    })
}
