//! [`Boolean`]: combine two solids. [`Split`]: cut a solid into pieces with
//! a face standing on its own.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, part::StepContext, reader::Reader, values::Design,
};
use geop_lib_booleans::{RemeshParams, boolean, split};
use geop_ops_model::{
    BooleanOp, Face, ModelOperation, NamedModel, Namer, SolidRef, build_model, model, read_model,
    solid,
};
use serde::{Deserialize, Serialize};

/// Combines the solids named `a` and `b` into one solid named `boolean(B)`
/// for the operation `B`, consuming both.
///
/// Every face, edge and vertex that survives keeps its name. What the
/// boolean creates is named after what it was made from, see
/// `geop_lib_booleans` — for example `boolean(B,E1,E2,i,n)` for the
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
    pub a: SolidRef,
    /// The second solid; for a difference, the one cut away.
    pub b: SolidRef,
    /// How to combine them.
    pub op: BooleanOp,
}

part_operation!(Boolean(BooleanArgs), Split(SplitArgs));

impl PartOperation for Boolean {
    const KIND: &'static str = "boolean";
    const INFO: OperationInfo = OperationInfo::new(
        "Boolean",
        "Unite, intersect or subtract two solids.",
        OperationGroup::Bodies,
        OperationTier::Big,
    )
    .icon(crate::icons::BOOLEAN);
    type Args = BooleanArgs;
    type Session = ();

    /// The difference of the two newest solids: the older cut by the one
    /// just built as a tool.
    fn new_args(context: &Context) -> BooleanArgs {
        let solids =
            (read_model(&context.before).map(|model| model.solid_names())).unwrap_or_default();
        let newest = |back: usize| {
            (solids.len().checked_sub(back))
                .and_then(|i| solids.get(i).cloned())
                .unwrap_or_default()
        };
        BooleanArgs {
            a: solid(newest(2)),
            b: solid(newest(1)),
            op: BooleanOp::Difference,
        }
    }

    fn run(ctx: &mut StepContext, args: &BooleanArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The two solids, picked, and how to combine them.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &BooleanArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, BooleanArgs> {
        let mut f = EditForm::<Design, BooleanArgs>::default();
        let before = &context.before;
        f.pick_one("a", "a", before, Some(&args.a), |edit, a| edit.args.a = a)
            .pick_one("b", "b", before, Some(&args.b), |edit, b| edit.args.b = b)
            .select(
                "op",
                "op",
                args.op.value(),
                BooleanOp::choices(),
                false,
                |edit, choice| {
                    if let Some(op) = BooleanOp::from_value(choice) {
                        edit.args.op = op;
                    }
                },
            );
        f
    }
}

impl ModelOperation for Boolean {
    fn build(
        model: &mut NamedModel<Design>,
        _: &Reader,
        step: &str,
        args: &BooleanArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("boolean", step)?;
        let a = model.solid_id(args.a.name())?;
        let b = model.solid_id(args.b.name())?;
        boolean(model, &namer, a, b, args.op, RemeshParams::default())?;
        Ok(())
    }
}

/// Cuts the solid named `solid` into pieces with the face named `face` —
/// a face standing on its own, such as an extrude or revolve builds as a
/// face, and with it every face of its sheet. The pieces are named
/// `split(S,0)`, `split(S,1)`, ... for the operation `S`, the solid is
/// consumed, and the face is left as it is (see [`geop_lib_booleans::split`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Split;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SplitArgs {
    /// The solid to cut.
    pub solid: SolidRef,
    /// A face of the sheet to cut it with.
    pub face: ElementReference<NamedModel<Design>, Face>,
}

impl PartOperation for Split {
    const KIND: &'static str = "split";
    const INFO: OperationInfo = OperationInfo::new(
        "Split",
        "Cut a solid into pieces with a face standing on its own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::SPLIT);
    type Args = SplitArgs;
    type Session = ();

    /// The newest solid, cut by the newest face standing on its own.
    fn new_args(context: &Context) -> SplitArgs {
        let before = read_model(&context.before).unwrap_or_default();
        SplitArgs {
            solid: solid(before.newest_solid().unwrap_or_default()),
            face: ElementReference::new(
                model(),
                before.sheet_face_names().pop().unwrap_or_default(),
            ),
        }
    }

    fn run(ctx: &mut StepContext, args: &SplitArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The solid and the face, picked.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &SplitArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, SplitArgs> {
        let mut f = EditForm::<Design, SplitArgs>::default();
        let before = &context.before;
        f.pick_one(
            "solid",
            "solid",
            before,
            Some(&args.solid),
            |edit, solid| edit.args.solid = solid,
        )
        .pick_one("face", "face", before, Some(&args.face), |edit, face| {
            edit.args.face = face
        });
        f
    }
}

impl ModelOperation for Split {
    fn build(
        model: &mut NamedModel<Design>,
        _: &Reader,
        step: &str,
        args: &SplitArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("split", step)?;
        let solid = model.solid_id(args.solid.name())?;
        let face = model.face_id(args.face.name())?;
        let sheet = model.sheet_of_face(
            face,
            "cut with a face standing on its own, as an extrude or revolve builds in face mode",
        )?;
        split(model, &namer, solid, sheet, RemeshParams::default())?;
        Ok(())
    }
}
