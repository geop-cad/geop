//! [`Boolean`]: combine two solids. [`Split`]: cut a solid into pieces with
//! a face standing on its own. [`Imprint`]: divide two bodies where they
//! cross. [`Section`]: the curves where they cross, as a wire.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    one_of, part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::FrameAxis,
};
use geop_core_program::{element::ElementReference, part::StepContext, reader::Reader};
use geop_lib_booleans::{
    RemeshParams, boolean, imprint, imprint_on_face, section, section_by_plane, split,
};
use geop_ops_model::{
    Body, BooleanOp, Face, ModelOperation, NamedModel, Namer, Plane, SolidRef, build_model,
    datums::{Datums, origin_plane},
    element, model, read_model, solid,
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

part_operation!(
    Boolean(BooleanArgs),
    Split(SplitArgs),
    Imprint(ImprintArgs),
    Section(SectionArgs)
);

impl PartOperation for Boolean {
    const KIND: &'static str = "boolean";
    const INFO: OperationInfo = OperationInfo::new(
        "Boolean",
        "Unite, intersect or subtract two solids.",
        OperationGroup::Bodies,
        OperationTier::Big,
    )
    .shortcut("B")
    .icon(crate::icons::BOOLEAN);
    type Args = BooleanArgs;
    type Session = ();

    /// The difference of the two newest solids: the older cut by the one
    /// just built as a tool.
    fn new_args(context: &Context) -> BooleanArgs {
        let [a, b] = newest_two(context);
        BooleanArgs {
            a: solid(a),
            b: solid(b),
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
    ) -> EditForm<'a, BooleanArgs> {
        let mut f = EditForm::<BooleanArgs>::default();
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
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &BooleanArgs) -> GeopResult<()> {
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
/// `split(Scalar,0)`, `split(Scalar,1)`, ... for the operation `Scalar`, the solid is
/// consumed, and the face is left as it is (see [`geop_lib_booleans::split`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Split;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SplitArgs {
    /// The solid to cut.
    pub solid: SolidRef,
    /// A face of the sheet to cut it with.
    pub face: ElementReference<NamedModel, Face>,
}

impl PartOperation for Split {
    const KIND: &'static str = "split";
    const INFO: OperationInfo = OperationInfo::new(
        "Split",
        "Cut a solid into pieces with a face standing on its own.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .shortcut("Shift+B")
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
    ) -> EditForm<'a, SplitArgs> {
        let mut f = EditForm::<SplitArgs>::default();
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
    fn build(model: &mut NamedModel, _: &Reader, step: &str, args: &SplitArgs) -> GeopResult<()> {
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

/// Imprints the body `tool` onto the body `body`, keeping both: every face
/// of each is divided along the curves where the other's boundary crosses
/// it, and nothing is taken away. With `face`, only that face of `body` is
/// divided, and `tool` stays as it is (see
/// [`geop_lib_booleans::imprint_on_face`]).
///
/// What the imprint creates is named as a boolean names it, `imprint(I,..)`
/// for the operation `I`; `body`'s edges along the curves keep those names,
/// and `tool`'s copies of them are named `imprint(I,X)` for the name `X`
/// they share (see [`geop_lib_booleans::imprint`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Imprint;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImprintArgs {
    /// The body imprinted on: a solid, or a face of a sheet.
    pub body: ElementReference<NamedModel, Body>,
    /// The body imprinted with.
    pub tool: ElementReference<NamedModel, Body>,
    /// Only this face of `body` is divided, if given.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<ElementReference<NamedModel, Face>>,
}

/// The newest solid but one and the newest: the solids a step combining
/// two is most likely about. Empty where the part has fewer.
fn newest_two(context: &Context) -> [String; 2] {
    let solids = (read_model(&context.before).map(|model| model.solid_names())).unwrap_or_default();
    let newest = |back: usize| {
        (solids.len().checked_sub(back))
            .and_then(|i| solids.get(i).cloned())
            .unwrap_or_default()
    };
    [newest(2), newest(1)]
}

impl PartOperation for Imprint {
    const KIND: &'static str = "imprint";
    const INFO: OperationInfo = OperationInfo::new(
        "Imprint",
        "Divide two bodies where they cross, keeping both.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::IMPRINT);
    type Args = ImprintArgs;
    type Session = ();

    /// The two newest solids, the newer imprinted on the older.
    fn new_args(context: &Context) -> ImprintArgs {
        let [body, tool] = newest_two(context);
        ImprintArgs {
            body: element(body),
            tool: element(tool),
            face: None,
        }
    }

    fn run(ctx: &mut StepContext, args: &ImprintArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The two bodies, picked, and the one face to divide, if only one.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ImprintArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ImprintArgs> {
        let mut f = EditForm::<ImprintArgs>::default();
        let before = &context.before;
        f.pick_one("body", "body", before, Some(&args.body), |edit, body| {
            edit.args.body = body
        })
        .pick_one("tool", "tool", before, Some(&args.tool), |edit, tool| {
            edit.args.tool = tool
        })
        .pick(
            "face",
            "only face",
            before,
            args.face.as_slice(),
            false,
            |edit, mut face| edit.args.face = face.pop(),
        );
        f
    }
}

impl ModelOperation for Imprint {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &ImprintArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("imprint", step)?;
        let body = reader.retrieve_element(&args.body)?;
        let tool = reader.retrieve_element(&args.tool)?;
        if body == tool {
            return Err(GeopError::new(format!(
                "{:?} is imprinted on itself: pick another body as the tool",
                args.tool.name()
            )));
        }
        let params = RemeshParams::default();
        let Some(face) = &args.face else {
            imprint(model, &namer, body, tool, params)?;
            return Ok(());
        };
        let face_id = model.face_id(face.name())?;
        if model.topology().body_of_face(face_id)? != body {
            return Err(GeopError::new(format!(
                "the face {:?} is not of the body {:?}: pick a face of the body imprinted on",
                face.name(),
                args.body.name()
            )));
        }
        // The tool stays as it is: a copy of it is imprinted, and consumed.
        let copy = model
            .copy_body(tool, |name| namer.name(&["copy", name]))?
            .body()?;
        imprint_on_face(model, &namer, face_id, &[copy], params)?;
        Ok(())
    }
}

one_of! {
    /// What a section cuts a body with: a datum plane, or another body.
    pub enum SectionWith {
        Plane(ElementReference<Datums, Plane>),
        Body(ElementReference<NamedModel, Body>),
    }
}

/// The curves where the boundary of the body `body` crosses a plane or
/// another body's boundary, as a wire of edges standing on their own, both
/// left as they are: what a sweep runs along (an edge of the wire picked as
/// a path is the run of the wire it is in) or a 3-D sketch takes a curve
/// from.
///
/// Its edges and vertices are named as a boolean of copies of the two
/// names what it creates, `section(S,..)` for the operation `S` (see
/// [`geop_lib_booleans::section`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Section;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SectionArgs {
    /// The body cut: a solid, or a face of a sheet.
    pub body: ElementReference<NamedModel, Body>,
    /// What it is cut with.
    pub with: SectionWith,
}

impl PartOperation for Section {
    const KIND: &'static str = "section";
    const INFO: OperationInfo = OperationInfo::new(
        "Section",
        "The curves where a body crosses a plane or another body.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::SECTION);
    type Args = SectionArgs;
    type Session = ();

    /// The newest solid, cut by the origin's `xy` plane.
    fn new_args(context: &Context) -> SectionArgs {
        let [_, body] = newest_two(context);
        SectionArgs {
            body: element(body),
            with: SectionWith::Plane(origin_plane(FrameAxis::Z)),
        }
    }

    fn run(ctx: &mut StepContext, args: &SectionArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The body and the plane or body cutting it, picked.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &SectionArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, SectionArgs> {
        let mut f = EditForm::<SectionArgs>::default();
        let before = &context.before;
        f.pick_one("body", "body", before, Some(&args.body), |edit, body| {
            edit.args.body = body
        })
        .pick_one("with", "with", before, Some(&args.with), |edit, with| {
            edit.args.with = with
        });
        f
    }
}

impl ModelOperation for Section {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &SectionArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("section", step)?;
        let body = reader.retrieve_element(&args.body)?;
        let params = RemeshParams::default();
        match &args.with {
            SectionWith::Plane(plane) => {
                let plane = reader.retrieve_element(plane)?;
                section_by_plane(model, &namer, body, &plane, params)?;
            }
            SectionWith::Body(other) => {
                let other = reader.retrieve_element(other)?;
                if body == other {
                    return Err(GeopError::new(format!(
                        "{:?} is cut with itself: pick another body, or a plane",
                        args.body.name()
                    )));
                }
                section(model, &namer, body, other, params)?;
            }
        }
        Ok(())
    }
}
