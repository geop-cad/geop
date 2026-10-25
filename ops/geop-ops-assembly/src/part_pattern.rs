//! [`PartPattern`]: copies of a placed part, in a row or round an axis.

use geop_core_editor::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Choice, Number, Tone, Unit},
    one_of, part_operation,
    part_operation::PartOperation,
};
use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion},
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    numeric::NumericInput,
    part::StepContext,
    reader::Reader,
    target::{Target, TargetReference},
    values::{Design, Settings},
};
use geop_ops_model::datums::Datums;
use geop_ops_model::{Line, NamedModel, Namer, Round};
use geop_ops_model::parameters::{Placement, instance, instances, located};
use geop_ops_pattern::spacing::{Spacing, count_field, whole_count};
use geop_ops_sketch::PlacedSketch;
use serde::{Deserialize, Serialize};

use crate::placed::define_placed;

one_of! {
    /// The line copies of a placed part go along, or the axis they go round,
    /// in the part or in a part placed in it: a straight edge, a circular
    /// edge or a round face, a datum axis, a sketch's line.
    pub enum PatternAxis -> Axis<Design> {
        Line(ElementReference<NamedModel<Design>, Line>),
        Round(ElementReference<NamedModel<Design>, Round>),
        Datum(ElementReference<Datums, Line>),
        Sketch(ElementReference<PlacedSketch<Design>, Line>),
    }
}

/// Places copies of the part placed by the step `part` — the same part,
/// built once — in a row along a line, or round an axis: `count` in all,
/// the part itself the first. Each copy is a placed part of its own, named
/// `part_pattern(P,k)` for the step `P` and `k` from 1, its entities behind
/// that name like any placed part's: `part_pattern(bolts,2)/extrude(head,end)`.
///
/// A copy goes where the pattern puts it, relative to where the part is —
/// no mate moves it, and when the part moves, its copies move with it. The
/// axis is picked in the part or in a part placed in it — an edge, a datum
/// axis, a sketch's line — where it is as the copies are placed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PartPattern;

/// How the copies are laid out.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Layout {
    /// In a row along the line's direction, `spacing` apart.
    Linear { spacing: Spacing },
    /// Turned round the axis, `angle` degrees apart.
    Circular { angle: Spacing },
}

impl Layout {
    /// How far apart the copies are.
    fn spacing(&self) -> &Spacing {
        match self {
            Layout::Linear { spacing } => spacing,
            Layout::Circular { angle } => angle,
        }
    }

    /// How far apart the copies are, to change.
    fn spacing_mut(&mut self) -> &mut Spacing {
        match self {
            Layout::Linear { spacing } => spacing,
            Layout::Circular { angle } => angle,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartPatternArgs {
    /// The placed part copied, by the id of the step that placed it.
    #[serde(default)]
    pub part: String,
    /// The line the copies go along, or the axis they go round: a straight
    /// or circular edge, a round face, a datum axis.
    #[serde(default)]
    pub axis: Option<PatternAxis>,
    /// Go against the line, or turn the other way round the axis.
    #[serde(default)]
    pub reversed: bool,
    pub layout: Layout,
    /// How many there are, the part itself included: a number, or a
    /// formula of the part's parameters.
    pub count: NumericInput,
}

/// The motions moving the part to each copy, `k` from 1: along `axis` or
/// round it, as `args` lays them out, its formulas read through `reader`.
fn motions(
    args: &PartPatternArgs,
    axis: &Axis<Design>,
    reader: &Reader,
) -> GeopResult<Vec<Pose<Design>>> {
    let count = whole_count(&args.count, reader)?;
    if count < 2 {
        return Err(GeopError::new(format!(
            "a pattern of {count} is no pattern: it needs 2 or more"
        )));
    }
    let direction = if args.reversed {
        axis.direction.neg()
    } else {
        axis.direction
    };
    match &args.layout {
        Layout::Linear { spacing } => {
            let row = spacing.along(count, reader)?;
            (1..count)
                .map(|k| {
                    let offset = row.offset(&direction, k)?.expect("a copy is no seed");
                    Pose::new(offset, Quaternion::identity())
                })
                .collect()
        }
        Layout::Circular { angle } => {
            let row = angle.round(count, reader)?;
            (1..count)
                .map(|k| row.turn(&axis.point, &direction, k))
                .collect()
        }
    }
}

/// The placed parts the steps before have placed, by name, in order.
fn placed_names(before: &Reader) -> Vec<String> {
    (instances(before).unwrap_or_default().into_iter())
        .map(|(name, _)| name.to_string())
        .collect()
}

/// Checks the arguments ask for a pattern at all, before anything is
/// resolved.
fn check(args: &PartPatternArgs) -> GeopResult<&PatternAxis> {
    if args.part.is_empty() {
        return Err(GeopError::new("choose the placed part to pattern"));
    }
    args.axis.as_ref().ok_or_else(|| {
        GeopError::new("pick the line the copies go along, or the axis they go round")
    })
}

/// The copies a part pattern places: the file and inputs of the part it
/// copies — a copy places what the part was placed from — and the motion
/// from the part to each copy, `k` from 1.
#[derive(Clone, Debug, PartialEq)]
struct Copies {
    file: String,
    inputs: Settings,
    motions: Vec<Pose<Design>>,
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
/// the part's file and inputs, and the motions round or along the axis
/// where it is, as many as the count comes to.
fn plan(reader: &Reader, args: &PartPatternArgs) -> GeopResult<Copies> {
    let axis = check(args)?;
    let Ok(seed) = instance(reader, &args.part) else {
        return Err(GeopError::new(format!(
            "{:?} is no part placed so far: pattern one an earlier step placed",
            args.part
        )));
    };
    let axis = axis.resolve_placed(reader)?;
    let motion = located(reader, &axis.instance)?.motion();
    let axis = axis.element.transform(&motion);
    Ok(Copies {
        file: seed.file.clone(),
        inputs: seed.inputs.clone(),
        motions: motions(args, &axis, reader)?,
    })
}

part_operation!(PartPattern(PartPatternArgs));

impl PartOperation for PartPattern {
    const KIND: &'static str = "part_pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Part pattern",
        "Place copies of a placed part in a row along a line, or round an axis.",
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
            axis: None,
            reversed: false,
            layout: Layout::Circular {
                angle: Spacing::step(60.0),
            },
            count: NumericInput::Plain(6.0),
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
        let Copies {
            file,
            inputs,
            motions,
        } = (ctx.reader())
            .retrieve_element(&ElementReference::whole(copies(step)))
            .with_context(context)?
            .clone();
        let namer = Namer::new("part_pattern", step)?;
        for (k, motion) in (1..).zip(motions) {
            let name = namer.name(&[&k.to_string()]);
            let placement = Placement::After {
                of: args.part.clone(),
                motion,
            };
            define_placed(ctx, &name, &file, &inputs, placement).with_context(context)?;
        }
        Ok(())
    }

    /// The part to copy, the line or axis, the layout and how many.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &PartPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, PartPatternArgs> {
        let mut f = EditForm::<Design, PartPatternArgs>::default();
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
        f.pick_one(
            "axis",
            "along or round",
            before,
            args.axis.as_ref(),
            |e, axis| e.args.axis = Some(axis),
        );
        f.checkbox(
            "reversed",
            "reverse direction",
            args.reversed,
            |Edit { args, .. }, reversed| args.reversed = reversed,
        );
        let circular = matches!(args.layout, Layout::Circular { .. });
        let choices = vec![
            Choice::new("circular", "round the axis"),
            Choice::new("linear", "along the line"),
        ];
        let way = if circular { "circular" } else { "linear" };
        f.select(
            "layout",
            "layout",
            way,
            choices,
            false,
            |Edit { args, .. }, way| {
                args.layout = match (way, &args.layout) {
                    ("linear", Layout::Circular { .. }) => Layout::Linear {
                        spacing: Spacing::step(1.0),
                    },
                    ("circular", Layout::Linear { .. }) => Layout::Circular {
                        angle: Spacing::extent(360.0),
                    },
                    (_, layout) => layout.clone(),
                };
            },
        );
        let (key, labels, unit) = if circular {
            ("angle", ["angle between", "total angle"], Unit::Angle)
        } else {
            ("spacing", ["spacing", "total"], Unit::Length)
        };
        args.layout.spacing().fields(
            &mut f,
            key,
            labels,
            |label, value| Number::numeric(value, label, before, unit),
            |args: &mut PartPatternArgs, spacing| *args.layout.spacing_mut() = spacing,
        );
        count_field(&mut f, "count", &args.count, before, None, |edit, count| {
            edit.args.count = count
        });
        f.form.dialog.text(
            "hint",
            "Each copy is a placed part of its own, named part_pattern(step,k); it moves with the part.",
            Tone::Hint,
        );
        f
    }
}
