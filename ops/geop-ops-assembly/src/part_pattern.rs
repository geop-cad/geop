//! [`PartPattern`]: copies of a placed part wherever a list of frames puts
//! them.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Choice, Tone},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    numeric::NumericInput,
    part::StepContext,
    placed::PlacedArgs,
    reader::Reader,
    target::{Target, TargetReference},
};
use geop_ops_model::Namer;
use geop_ops_model::parameters::{Placement, instance, instances, located};
use geop_ops_model::spacing::{Frames, Spacing};
use serde::{Deserialize, Serialize};

use crate::placed::define_placed;

/// Places copies of the part placed by the step `part` — the same part,
/// built once — wherever `frames` puts them (see [`Frames`]): in a row
/// or a grid, round an axis, along a curve, at points — the part at the
/// first — from a table of offsets, filling a face, or at the holes of a
/// feature, of the part or of a part placed in it. Each copy is a placed
/// part of its own, named `part_pattern(P,i)` for the step `P` and the
/// copy labelled `i` — `1`, `2`, ..., or `i.j` in a grid or a fill —
/// its entities behind that name like any placed part's:
/// `part_pattern(bolts,2)/extrude(head,end)`.
///
/// A copy goes where the pattern puts it, relative to where the part is —
/// no mate moves it, and when the part moves, its copies move with it.
/// What the copies are laid out along, round or at is picked in the part
/// or in a part placed in it, where it is as the copies are placed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PartPattern;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartPatternArgs {
    /// The placed part copied, by the id of the step that placed it.
    #[serde(default)]
    pub part: String,
    /// Where the copies go.
    pub frames: Frames,
}

/// The placed parts the steps before have placed, by name, in order.
fn placed_names(before: &Reader) -> Vec<String> {
    (instances(before).unwrap_or_default().into_iter())
        .map(|(name, _)| name.to_string())
        .collect()
}

/// The copies a part pattern places: the file and inputs of the part it
/// copies — a copy places what the part was placed from — and each copy's
/// label and the motion from the part to it.
#[derive(Clone, Debug, PartialEq)]
struct Copies {
    placed: PlacedArgs,
    motions: Vec<(String, Pose)>,
}

/// Each part pattern's own, made before its copies are placed.
impl Target for Copies {}

/// Read whole.
impl Kind for Copies {
    const NAME: &'static str = "copies";
}

impl HasElements<Copies> for Copies {
    type Element<'a> = &'a Copies;

    fn element(&self, name: &str) -> GeopResult<&Copies> {
        whole(self, name)
    }
}

/// The copies the part pattern `step` places.
fn copies(step: &str) -> TargetReference<Copies> {
    TargetReference::new(format!("part_pattern:{step}"))
}

/// The copies `args` asks for, of the part as the steps before left it:
/// the part's file and inputs, and the motions to where the frames put
/// them, as many as they come to — a face filled through where the part
/// is.
fn plan(reader: &Reader, args: &PartPatternArgs) -> GeopResult<Copies> {
    if args.part.is_empty() {
        return Err(GeopError::new("choose the placed part to pattern"));
    }
    let Ok(seed) = instance(reader, &args.part) else {
        return Err(GeopError::new(format!(
            "{:?} is no part placed so far: pattern one an earlier step placed",
            args.part
        )));
    };
    let anchor = || Ok(located(reader, &args.part)?.position());
    let motions = args.frames.motions(reader, &anchor)?;
    if motions.is_empty() {
        return Err(GeopError::new(
            "the pattern puts no copy anywhere but where the part is: it needs 2 or more instances",
        ));
    }
    Ok(Copies {
        placed: PlacedArgs {
            file: seed.file.clone(),
            inputs: seed.inputs.clone(),
            then: seed.then.clone(),
        },
        motions,
    })
}

part_operation!(PartPattern(PartPatternArgs));

impl PartOperation for PartPattern {
    const KIND: &'static str = "part_pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Part pattern",
        "Place copies of a placed part in a row, round an axis, along a curve, at points or holes, from a table or filling a face.",
        OperationGroup::Assembly,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::PART_PATTERN);
    type Args = PartPatternArgs;
    type Session = ();

    /// The newest placed part, six round an axis still to pick.
    fn new_args(context: &Context) -> PartPatternArgs {
        PartPatternArgs {
            part: placed_names(&context.before).pop().unwrap_or_default(),
            frames: Frames::Circular {
                axis: None,
                reversed: false,
                count: NumericInput::Plain(6.0),
                angle: Spacing::extent(360.0),
            },
        }
    }

    /// Places the copies, each where the pattern puts it after the part:
    /// the axis resolved in the part as the steps before left it.
    ///
    /// Which copies there are, and where, is a node of its own ([`plan`]),
    /// made from what it reads of the part: the step declares a copy for
    /// each motion it holds and computes nothing itself.
    fn run(ctx: &mut StepContext, args: &PartPatternArgs) -> GeopResult<()> {
        let step = ctx.step();
        let context = with_context!("part_pattern({step}, {args:?})");
        ctx.produce(&copies(step), args, plan)
            .with_context(context)?;
        let Copies { placed, motions } = (ctx.reader())
            .retrieve_element(&ElementReference::whole(copies(step)))
            .with_context(context)?
            .clone();
        let namer = Namer::new("part_pattern", step)?;
        for (label, motion) in motions {
            let name = namer.name(&[&label]);
            let placement = Placement::After {
                of: args.part.clone(),
                motion,
            };
            define_placed(ctx, &name, &placed, placement).with_context(context)?;
        }
        Ok(())
    }

    /// The part to copy, and where the copies go.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &PartPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, PartPatternArgs> {
        let mut f = EditForm::<PartPatternArgs>::default();
        let before = &context.before;
        let options = std::iter::once(Choice::new("", "Choose a placed part…"))
            .chain(
                placed_names(before)
                    .into_iter()
                    .map(|name| Choice::new(name.clone(), name)),
            )
            .collect();
        f.select(
            "part",
            "part",
            args.part.clone(),
            options,
            true,
            |Edit { args, .. }, part| args.part = part.to_string(),
        );
        let at = located(before, &args.part).ok().map(|pose| pose.position());
        args.frames
            .fields(&mut f, before, at, |args: &mut PartPatternArgs, frames| {
                args.frames = frames
            });
        f.form.dialog.text(
            "hint",
            "Each copy is a placed part of its own, named part_pattern(step,i); it moves with the part.",
            Tone::Hint,
        );
        f
    }
}
