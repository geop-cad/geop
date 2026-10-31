//! Parts derived from another: [`DerivedPart`] brings the bodies another
//! program file builds into the part, and [`MirrorPart`] mirrors every body
//! of the part in place — the other hand of it. A mirrored component of an
//! assembly is a placed part derived so: its file's program with a
//! [`MirrorPart`] step after it (see [`crate::AddPartArgs::mirror`]).

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::Choice,
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    numeric::{ParameterValue, ParameterValues, parameter_values},
    part::StepContext,
    placed::{PlacedArgs, build_derived},
    program::ProgramStep,
    reader::Reader,
    values::Settings,
    workspace::is_program,
};
use geop_core_topology::Body;
use geop_ops_model::{
    ModelOperation, NamedModel, Namer, build_model, parameters::file_choices, read_model,
};
use serde::{Deserialize, Serialize};

/// The plane of a part's own origin a part is mirrored in, by its normal:
/// `x` for its `yz` plane.
pub(crate) fn plane_name(normal: FrameAxis) -> &'static str {
    match normal {
        FrameAxis::X => "yz",
        FrameAxis::Y => "zx",
        FrameAxis::Z => "xy",
    }
}

/// Every body of `model` — its solids, then the sheets — in the order they
/// were made.
fn bodies(model: &NamedModel) -> Vec<Body> {
    let topology = model.topology();
    let solids = topology.solids.keys().map(|&solid| Body::Solid(solid));
    let sheets = (topology.shells.iter())
        .filter(|(_, shell)| shell.solid.is_none())
        .map(|(&shell, _)| Body::Sheet(shell));
    solids.chain(sheets).collect()
}

/// Mirrors every body of the part in place, in a plane of its own origin:
/// the other hand of it, every entity keeping its name — so a step or a
/// mate naming one still finds it. Its datums stay as they are: a frame
/// mirrored is no frame. The part says it is mirrored where it is listed
/// (the parameter `mirrored`, the plane's name): a bill of materials tells
/// it from the part as it was.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MirrorPart;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MirrorPartArgs {
    /// The normal of the plane of the part's origin mirrored in: `x` for
    /// its `yz` plane.
    pub normal: FrameAxis,
}

/// The parameter a mirrored part says it is mirrored by.
pub const MIRRORED: &str = "mirrored";

part_operation!(MirrorPart(MirrorPartArgs));

impl PartOperation for MirrorPart {
    const KIND: &'static str = "mirror_part";
    const INFO: OperationInfo = OperationInfo::new(
        "Mirror part",
        "Mirror the whole part in place in a plane of its origin: the other hand of it.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::MIRROR_PART);
    type Args = MirrorPartArgs;
    type Session = ();

    /// In the origin's `yz` plane.
    fn new_args(_: &Context) -> MirrorPartArgs {
        MirrorPartArgs {
            normal: FrameAxis::X,
        }
    }

    /// The bodies mirrored, then the part said to be mirrored.
    fn run(ctx: &mut StepContext, args: &MirrorPartArgs) -> GeopResult<()> {
        build_model::<Self>(ctx, args)?;
        ctx.produce(&parameter_values(), &args.normal, |reader, normal| {
            let mut values: ParameterValues = reader.previous()?;
            values.0.push((
                MIRRORED.to_string(),
                ParameterValue::Text(plane_name(*normal).to_string()),
            ));
            Ok(values)
        })
    }

    /// The plane, of three.
    fn form<'a>(
        _: &'a Context<'a>,
        args: &MirrorPartArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, MirrorPartArgs> {
        let mut f = EditForm::<MirrorPartArgs>::default();
        f.select(
            "normal",
            "Mirrored in",
            plane_name(args.normal),
            plane_choices(),
            false,
            |edit, plane| {
                if let Some(normal) = FrameAxis::ALL.into_iter().find(|n| plane_name(*n) == plane) {
                    edit.args.normal = normal;
                }
            },
        );
        f
    }
}

/// The planes of a part's origin, to choose one from.
pub(crate) fn plane_choices() -> Vec<Choice> {
    (FrameAxis::ALL.into_iter())
        .map(|normal| {
            let name = plane_name(normal);
            Choice::new(name, format!("its {name} plane"))
        })
        .collect()
}

impl ModelOperation for MirrorPart {
    fn build(
        model: &mut NamedModel,
        _: &Reader,
        step: &str,
        args: &MirrorPartArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("mirror_part({step}, {args:?})");
        let mirror = mirror_in(args.normal).with_context(ctx)?;
        for body in bodies(model) {
            model.transform_body(body, &mirror).with_context(ctx)?;
        }
        Ok(())
    }
}

/// The mirror in the plane of the origin normal to `normal`.
fn mirror_in(normal: FrameAxis) -> GeopResult<Motion> {
    let mut n = Vector3::zero();
    n[normal as usize] = Scalar::ONE;
    Motion::mirror(&Vector3::zero(), &n)
}

/// The step mirroring a part in the plane of its origin normal to
/// `normal`, after the steps of its own program `ids` names: what a
/// mirrored component is placed with (see [`crate::AddPartArgs::mirror`]).
pub(crate) fn mirror_step(normal: FrameAxis) -> GeopResult<ProgramStep> {
    ProgramStep::new::<MirrorPart>("mirrored", &MirrorPartArgs { normal })
}

/// Brings the bodies the program in another file builds into the part —
/// its parameters set as `parameters` says, else as its own — each entity
/// named `X` there named `derived(D,X)` here, for the step `D`: a part
/// derived from another, which follows it as it changes. Mirrored after
/// (see [`MirrorPart`]), it is the other hand of that part, a part of its
/// own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DerivedPart;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DerivedPartArgs {
    /// The program file whose bodies are brought in, relative to this
    /// program's own file; empty until one is chosen.
    #[serde(default)]
    pub file: String,
    /// The values its parameters are given instead of its own, by name.
    #[serde(default, skip_serializing_if = "Settings::is_empty")]
    pub parameters: Settings,
}

part_operation!(DerivedPart(DerivedPartArgs));

impl PartOperation for DerivedPart {
    const KIND: &'static str = "derived_part";
    const INFO: OperationInfo = OperationInfo::new(
        "Derived part",
        "Bring the bodies another program file builds into the part, following it as it changes.",
        OperationGroup::Bodies,
        OperationTier::Menu,
    )
    .icon(crate::icons::DERIVED_PART);
    type Args = DerivedPartArgs;
    type Session = ();

    fn new_args(_: &Context) -> DerivedPartArgs {
        DerivedPartArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &DerivedPartArgs) -> GeopResult<()> {
        if args.file.is_empty() {
            return Err(GeopError::new("choose a file to derive the part from"));
        }
        build_model::<Self>(ctx, args)
    }

    /// The file, chosen from the workspace's programs.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &DerivedPartArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, DerivedPartArgs> {
        let mut f = EditForm::<DerivedPartArgs>::default();
        let files = file_choices(context.runner, is_program);
        f.file(
            "file",
            "file",
            args.file.clone(),
            files,
            &[],
            |edit, file| {
                edit.args.file = file.to_string();
            },
        );
        f
    }
}

impl ModelOperation for DerivedPart {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &DerivedPartArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("derived_part({step}, file={:?})", args.file);
        let placed = PlacedArgs {
            file: args.file.clone(),
            inputs: args.parameters.clone(),
            then: Vec::new(),
        };
        let derived = build_derived(reader, &placed).with_context(ctx)?;
        if let Some((failed, e)) = derived.part.failures().first() {
            let e = e.clone();
            return Err(e.with_context(format!("step `{failed}` of {:?}", args.file)))
                .with_context(ctx);
        }
        let source = read_model(&derived.part.current()).with_context(ctx)?;
        let namer = Namer::new("derived", step).with_context(ctx)?;
        for body in bodies(&source) {
            let (spec, names) = source.body_record(body).with_context(ctx)?;
            let names = names.renamed(|name| namer.name(&[name]));
            model.build_body(spec, names).with_context(ctx)?;
        }
        Ok(())
    }
}
