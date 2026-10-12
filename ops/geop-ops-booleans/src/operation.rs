//! [`Boolean`]: combine two solids — and [`Combine`], the same done by an
//! extrude or revolve with the solid it builds. [`Split`]: cut a solid into
//! pieces with a face standing on its own.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Choice, Dialog, Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_core_topology::{Body, SolidId};
use geop_ops::operation::Input;
use geop_ops::{
    Design, FeatureTool, NamedModel, Namer, ParamValue, ToolReach,
    operation::{Build, BuildReader, EntityRef, Lookup, LegacyModelStep, Role, Seen, legacy_build_model},
};
use serde::{Deserialize, Serialize};

use crate::{
    boolean::{BooleanOp, Reach, boolean, boolean_up_to_next, piece_up_to_next},
    remesh::remesh::RemeshParams,
    split::split,
};

/// Combines the solids named `a` and `b` into one solid named `boolean(B)`
/// for the operation `B`, consuming both.
///
/// Every face, edge and vertex that survives keeps its name. What the
/// boolean creates is named after what it was made from, see
/// [`crate::naming`] — for example `boolean(B,E1,E2,i,n)` for the
/// `i`-th of the `n` points where edges `E1` and `E2` cross.
///
/// An empty result — intersecting solids that don't overlap, say — is an
/// answer, not an error: both operands are consumed and no solid is named
/// `boolean(B)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Boolean;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BooleanArgs {
    /// The first solid; for a difference, the one cut from.
    pub a: String,
    /// The second solid; for a difference, the one cut away.
    pub b: String,
    /// How to combine them.
    pub op: BooleanOp,
}

impl LegacyModelStep for Boolean {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &BooleanArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("boolean({step}, {args:?})");
        let namer = Namer::new("boolean", step)?;
        let a = model.solid_id(&args.a).with_context(ctx)?;
        let b = model.solid_id(&args.b).with_context(ctx)?;
        boolean(model, &namer, a, b, args.op, RemeshParams::default()).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Boolean {
    const INFO: OperationInfo = OperationInfo::new(
        "boolean",
        "Boolean",
        "Unite, intersect or subtract two solids.",
        OperationGroup::Bodies,
        OperationTier::Big,
    )
    .icon(crate::icons::BOOLEAN);
    type Args = BooleanArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The difference of the two newest solids: the older cut by the one
    /// just built as a tool.
    fn new_args(context: &Context<Input, ParamValue>) -> BooleanArgs {
        let solids = Seen::new(context.before).model().solid_names();
        let newest = |back: usize| {
            (solids.len().checked_sub(back))
                .and_then(|i| solids.get(i).cloned())
                .unwrap_or_default()
        };
        BooleanArgs {
            a: newest(2),
            b: newest(1),
            op: BooleanOp::Difference,
        }
    }

    fn run(step: &str, part: &mut Build, args: &BooleanArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The two solids, picked, and how to combine them.
    fn presentation(_: &Context<Input, ParamValue>, args: &BooleanArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let ops = OPS
            .iter()
            .map(|&(_, value, label, _)| Choice::new(value, label));
        let solid = |name: &String| EntityRef::of_name(EntityRef::solid, name);
        (form.dialog)
            .reference("a", "a", solid(&args.a), &[Role::Solid], None, false)
            .reference("b", "b", solid(&args.b), &[Role::Solid], None, false)
            .select("op", "op", value_of(args.op), ops.collect(), false);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &BooleanArgs,
        _: &(),
        event: Event<Design>,
    ) -> (BooleanArgs, ()) {
        let mut args = args.clone();
        let solid = |picked: &[EntityRef]| {
            EntityRef::names_of(EntityRef::solid, picked)
                .pop()
                .unwrap_or_default()
        };
        if let Event::Field { key, value } = event {
            match (key.as_str(), value) {
                ("a", Value::Entities(picked)) => args.a = solid(&picked),
                ("b", Value::Entities(picked)) => args.b = solid(&picked),
                ("op", Value::Choice(choice)) => {
                    if let Some(op) = op_of(&choice) {
                        args.op = op;
                    }
                }
                _ => {}
            }
        }
        (args, ())
    }
}

/// What an extrude or revolve does with the solid it builds: keep it as a
/// new body, or combine it by `op` with the solid named `target` — which
/// that consumes, like a [`Boolean`] does. A [`BooleanOp::Difference`] cuts
/// the new solid out of the target: an extruded pocket, a drilled hole.
///
/// A step may build its solid in several parts, each combined in turn — the
/// two sides of an extrude, one of which goes up to the next face of the
/// target (see [`Tool`]).
///
/// Either way the step's result is named after the step — `extrude(E)` —
/// so what comes after refers to "what step `E` left" however it was made.
/// Combined, the built solid is only a tool, gone once the step is done,
/// and what the combination creates is named `combine(E,...)`, as
/// [`Boolean`] names its own.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum Combine {
    #[default]
    NewBody,
    With {
        op: BooleanOp,
        target: String,
    },
}

impl Combine {
    /// The solid it combines with, if any.
    pub fn target(&self) -> Option<&str> {
        match self {
            Combine::NewBody => None,
            Combine::With { target, .. } => Some(target),
        }
    }

    /// The default for a step after `before`: joining the newest solid, if
    /// there is one — building onto what is there is the common case.
    pub fn new_for<S: Scalar>(before: &NamedModel<S>) -> Self {
        match before.newest_solid() {
            Some(target) => Combine::With {
                op: BooleanOp::Union,
                target,
            },
            None => Combine::NewBody,
        }
    }

    /// A join for a positive `value`, a cut for a negative one — what an
    /// extrude up out of a face, or down into it, means — when `value`
    /// crosses to the other side of zero from `from`. On the same side, a
    /// join or a cut stays what was chosen: an extrude may cut upwards into
    /// a solid above its sketch. A new body and an intersection stay what
    /// they are.
    pub fn follow_sign(&mut self, from: f64, value: f64) {
        let crossed = (from > 0.0) != (value > 0.0) || (from < 0.0) != (value < 0.0);
        if let Combine::With { op, .. } = self
            && crossed
            && *op != BooleanOp::Intersection
        {
            if value > 0.0 {
                *op = BooleanOp::Union;
            } else if value < 0.0 {
                *op = BooleanOp::Difference;
            }
        }
    }

    /// The field of `dialog` that picks the solid to combine with — or to
    /// go as far as — labelled `label`, holding `target`, if any.
    pub fn target_field<S: Scalar>(dialog: &mut Dialog<S>, label: &str, target: &[String]) {
        let picked = EntityRef::of_names(EntityRef::solid, target);
        dialog.reference(TARGET, label, picked, &[Role::Solid], None, false);
    }

    /// Its fields in `dialog`: the mode, and the target to pick.
    pub fn show<S: Scalar>(&self, dialog: &mut Dialog<S>) {
        let mode = match self {
            Combine::NewBody => NEW_BODY,
            Combine::With { op, .. } => value_of(*op),
        };
        let modes = std::iter::once(Choice::new(NEW_BODY, "New body")).chain(
            OPS.iter()
                .map(|&(_, value, _, label)| Choice::new(value, label)),
        );
        dialog.select("combine", "combine", mode, modes.collect(), false);
        if let Combine::With { target, .. } = self {
            Self::target_field(dialog, "target", std::slice::from_ref(target));
        }
    }

    /// Its field `key` set to `value`, if `key` is one of [`Combine::show`]'s
    /// — a mode chosen without a target takes the newest solid of `before`
    /// — and whether it was.
    pub fn set<S: Scalar>(&mut self, key: &str, value: &Value, before: &NamedModel<S>) -> bool {
        match (key, value) {
            ("combine", Value::Choice(mode)) => {
                let target = (self.target().map(str::to_string))
                    .or_else(|| before.newest_solid())
                    .unwrap_or_default();
                if mode == NEW_BODY {
                    *self = Combine::NewBody;
                } else if let Some(op) = op_of(mode) {
                    *self = Combine::With { op, target };
                }
                true
            }
            (TARGET, Value::Entities(picked)) => {
                if let Combine::With { target, .. } = self {
                    *target = EntityRef::names_of(EntityRef::solid, picked)
                        .pop()
                        .unwrap_or_default();
                }
                true
            }
            _ => false,
        }
    }

    /// The name the built solid gets: the step's own as a new body, or a
    /// name within the step for a tool the combination consumes.
    pub fn built_name(&self, namer: &Namer) -> String {
        match self {
            Combine::NewBody => namer.root(),
            Combine::With { .. } => namer.name(&["tool"]),
        }
    }

    /// Combines the `tools` built, one after the other, as the step `step`,
    /// whose names `namer` builds: the result is named `namer`'s root, and
    /// what the `k`-th tool's combination creates is named after
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
    /// feature of the step (see [`geop_ops::Feature`]): what a pattern of
    /// the feature copies and combines the same way. A new body is no
    /// feature: it is a body, patterned as one.
    pub fn apply<S: Scalar>(
        &self,
        model: &mut NamedModel<S>,
        namer: &Namer,
        step: &str,
        tools: &[Tool],
    ) -> GeopResult<Reach<()>> {
        let (op, target) = match self {
            Combine::NewBody => return new_body(model, namer, step, tools),
            Combine::With { op, target } => (*op, target),
        };
        let ctx = with_context!("combining with {target:?} ({op:?})");
        let mut target = model.solid_id(target).with_context(ctx)?;
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
            let combine = tool.namer(step)?;
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
}

/// The key of the field that picks the solid a step combines with (see
/// [`Combine::target_field`]).
const TARGET: &str = "combine_target";

/// How a new body is offered, among the ways to combine.
const NEW_BODY: &str = "new_body";

/// The boolean operations, as a dialog offers them: `(op, value, label as
/// a boolean, label as a way to combine)`.
const OPS: [(BooleanOp, &str, &str, &str); 3] = [
    (BooleanOp::Union, "union", "Union", "Join"),
    (BooleanOp::Difference, "difference", "Difference", "Cut"),
    (
        BooleanOp::Intersection,
        "intersection",
        "Intersection",
        "Intersect",
    ),
];

/// How `op` is offered (see [`OPS`]).
fn value_of(op: BooleanOp) -> &'static str {
    OPS.iter()
        .find(|o| o.0 == op)
        .expect("every op is offered")
        .1
}

/// The op offered as `value` (see [`OPS`]).
fn op_of(value: &str) -> Option<BooleanOp> {
    OPS.iter().find(|o| o.1 == value).map(|o| o.0)
}

/// The new body `tools` make, named `namer`'s root (see [`Combine::apply`]).
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
        let combine = tool.namer(step)?;
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

/// One solid an extrude or revolve built, for [`Combine::apply`] to
/// combine: the whole of it, or, with an `up_to_next` reach, only up to the
/// next face of the target (see [`boolean_up_to_next`]).
#[derive(Clone, Debug)]
pub struct Tool {
    pub solid: SolidId,
    pub reach: ToolReach,
}

impl Tool {
    /// All of `solid`, the one tool of its step.
    pub fn whole(solid: SolidId) -> Self {
        Tool {
            solid,
            reach: ToolReach::default(),
        }
    }

    /// The names of what combining it as the step `step` creates:
    /// `combine(step)`, scoped by its reach's scope, if it has one.
    fn namer(&self, step: &str) -> GeopResult<Namer> {
        let combine = Namer::new("combine", step)?;
        Ok(match &self.reach.scope {
            Some(scope) => combine.scoped(scope),
            None => combine,
        })
    }
}

/// Cuts the solid named `solid` into pieces with the face named `face` —
/// a face standing on its own, such as an extrude or revolve builds as a
/// face, and with it every face of its sheet. The pieces are named
/// `split(S,0)`, `split(S,1)`, ... for the operation `S`, the solid is
/// consumed, and the face is left as it is (see [`crate::split::split`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Split;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SplitArgs {
    /// The solid to cut.
    pub solid: String,
    /// A face of the sheet to cut it with.
    pub face: String,
}

impl LegacyModelStep for Split {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &SplitArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("split({step}, {args:?})");
        let namer = Namer::new("split", step)?;
        let solid = model.solid_id(&args.solid).with_context(ctx)?;
        let face = model.face_id(&args.face).with_context(ctx)?;
        let Body::Sheet(sheet) = model.topology().body_of_face(face)? else {
            return Err(GeopError::new(format!(
                "{:?} is a face of a solid; cut with a face standing on its own, as an extrude or revolve builds in face mode",
                args.face
            )))
            .with_context(ctx);
        };
        split(model, &namer, solid, sheet, RemeshParams::default()).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Split {
    const INFO: OperationInfo = OperationInfo::new(
        "split",
        "Split",
        "Cut a solid into pieces with a face standing on its own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::SPLIT);
    type Args = SplitArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// The newest solid, cut by the newest face standing on its own.
    fn new_args(context: &Context<Input, ParamValue>) -> SplitArgs {
        let seen = Seen::new(context.before);
        SplitArgs {
            solid: seen.model().newest_solid().unwrap_or_default(),
            face: seen.model().sheet_face_names().pop().unwrap_or_default(),
        }
    }

    fn run(step: &str, part: &mut Build, args: &SplitArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The solid and the face, picked.
    fn presentation(_: &Context<Input, ParamValue>, args: &SplitArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        let solid = EntityRef::of_name(EntityRef::solid, &args.solid);
        let face = EntityRef::of_name(EntityRef::face, &args.face);
        (form.dialog)
            .reference("solid", "solid", solid, &[Role::Solid], None, false)
            .reference("face", "face", face, &[Role::Sheet], None, false);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &SplitArgs,
        _: &(),
        event: Event<Design>,
    ) -> (SplitArgs, ()) {
        let mut args = args.clone();
        if let Event::Field { key, value } = event {
            match (key.as_str(), value) {
                ("solid", Value::Entities(picked)) => {
                    args.solid =
                        EntityRef::last_name(EntityRef::solid, &picked).unwrap_or_default();
                }
                ("face", Value::Entities(picked)) => {
                    args.face = EntityRef::last_name(EntityRef::face, &picked).unwrap_or_default();
                }
                _ => {}
            }
        }
        (args, ())
    }
}
