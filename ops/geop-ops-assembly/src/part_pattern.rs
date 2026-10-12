//! [`PartPattern`]: copies of a placed part, in a row or round an axis.

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion},
    with_context,
};
use geop_core_program::{
    editing::{Event, Form},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
    part::Retrieve,
};
use geop_ops::{
    Design, Namer, ParamValue,
    operation::{
        Aspects, Build, BuildReader, EntityRef, Input, Lookup, Role, Seen,
        pattern::{Spacing, count_field, count_of, whole_count},
    },
    parameters::{Formula, expressions},
    placed::{Instance, Placement, instance, instance_name},
    ui::{Choice, Edit, EditForm, Tone, Unit, formula_number},
};
use serde::{Deserialize, Serialize};

/// Places copies of the part placed by the step `part` — the same part,
/// built once — in a row along a line, or round an axis: `count` in all,
/// the part itself the first. Each copy is a placed part of its own, named
/// `part_pattern(P,k)` for the step `P` and `k` from 1, its entities behind
/// that name like any placed part's: `part_pattern(bolts,2)/extrude(head,end)`.
///
/// A copy goes where the pattern puts it, relative to where the part is —
/// no mate moves it, and when the part moves, its copies move with it. The
/// axis is picked in the part itself: an edge, a datum axis, a sketch's
/// line — not one of a placed part.
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
    pub axis: Option<EntityRef>,
    /// Go against the line, or turn the other way round the axis.
    #[serde(default)]
    pub reversed: bool,
    pub layout: Layout,
    /// How many there are, the part itself included: a number, or a
    /// formula of the part's parameters.
    pub count: Formula,
}

/// The motions moving the part to each copy, `k` from 1: along `axis` or
/// round it, as `args` lays them out, its formulas read by `lookup`.
fn motions(
    args: &PartPatternArgs,
    axis: &Axis<Design>,
    lookup: &mut (impl Lookup<Design> + ?Sized),
) -> GeopResult<Vec<Pose<Design>>> {
    let count = whole_count(&args.count, args.count.evaluate(lookup)?)?;
    if count < 2 {
        return Err(GeopError::new(format!(
            "a pattern of {count} is no pattern: it needs 2 or more"
        )));
    }
    let value = args.layout.spacing().value().evaluate(lookup)?;
    let direction = if args.reversed {
        axis.direction.neg()
    } else {
        axis.direction
    };
    match &args.layout {
        Layout::Linear { spacing } => {
            let row = spacing.along(count, value)?;
            (1..count)
                .map(|k| {
                    let offset = row.offset(&direction, k)?.expect("a copy is no seed");
                    Pose::new(offset, Quaternion::identity())
                })
                .collect()
        }
        Layout::Circular { angle } => {
            let row = angle.round(count, value)?;
            (1..count)
                .map(|k| row.turn(&axis.point, &direction, k))
                .collect()
        }
    }
}

/// The placed parts the steps before have placed, by name, in order.
fn placed_names(before: geop_ops::operation::BuildView) -> Vec<String> {
    (before.targets_of::<Instance>().into_iter())
        .filter_map(|(target, _)| instance_name(target).map(str::to_string))
        .collect()
}

/// The copy of the placed part `seed` going `motion` after it: the
/// generator of a copy's instance.
fn copy(reader: &mut BuildReader, (seed, motion): &(String, Pose<Design>)) -> GeopResult<Instance> {
    let placed = reader.retrieve_target(&instance(seed))?;
    Ok(Instance {
        component: placed.component.clone(),
        placement: Placement::After {
            of: seed.clone(),
            motion: *motion,
        },
        fixed: false,
    })
}

/// Checks the arguments ask for a pattern at all, before anything is
/// resolved.
fn check(args: &PartPatternArgs) -> GeopResult<&EntityRef> {
    if args.part.is_empty() {
        return Err(GeopError::new("choose the placed part to pattern"));
    }
    args.axis.as_ref().ok_or_else(|| {
        GeopError::new("pick the line the copies go along, or the axis they go round")
    })
}

impl Operation for PartPattern {
    const INFO: OperationInfo = OperationInfo::new(
        "part_pattern",
        "Part pattern",
        "Place copies of a placed part in a row along a line, or round an axis.",
        OperationGroup::Assembly,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::PART_PATTERN);
    type Args = PartPatternArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut PartPatternArgs) -> Vec<&mut String> {
        expressions([&mut args.count, args.layout.spacing_mut().value_mut()])
    }

    /// The newest placed part, six round an axis still to pick.
    fn new_args(context: &Context<Input, ParamValue>) -> PartPatternArgs {
        PartPatternArgs {
            part: placed_names(context.before).pop().unwrap_or_default(),
            axis: None,
            reversed: false,
            layout: Layout::Circular {
                angle: Spacing::step(60.0),
            },
            count: Formula::Plain(6.0),
        }
    }

    /// Places the copies, each where the pattern puts it after the part:
    /// the axis resolved in the part as the steps before left it.
    fn run(step: &str, part: &mut Build, args: &PartPatternArgs) -> GeopResult<()> {
        let ctx = with_context!("part_pattern({step}, {args:?})");
        let axis = check(args).with_context(ctx)?;
        if part
            .current()
            .retrieve_target(&instance(&args.part))
            .is_err()
        {
            return Err(GeopError::new(format!(
                "{:?} is no part placed so far: pattern one an earlier step placed",
                args.part
            )))
            .with_context(ctx);
        }
        let mut seen = Seen::new(part.current());
        let axis = Aspects::axis_of(axis, &mut seen).with_context(ctx)?;
        let motions = motions(args, &axis, &mut seen).with_context(ctx)?;
        let namer = Namer::new("part_pattern", step)?;
        for (k, motion) in (1..).zip(motions) {
            let name = namer.name(&[&k.to_string()]);
            part.define_target(&instance(&name), &(args.part.clone(), motion), copy)
                .with_context(ctx)?;
        }
        Ok(())
    }

    /// The part to copy, the line or axis, the layout and how many.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &PartPatternArgs,
        _: &(),
    ) -> Form<Design> {
        form(context, args).into_form()
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &PartPatternArgs,
        _: &(),
        event: Event<Design>,
    ) -> (PartPatternArgs, ()) {
        (form(context, args).apply(args, event), ())
    }
}

/// The part to copy, the line or axis, the layout and how many.
fn form(
    context: &Context<Input, ParamValue>,
    args: &PartPatternArgs,
) -> EditForm<'static, Design, PartPatternArgs> {
    let mut f = EditForm::<Design, PartPatternArgs>::default();
    let options = std::iter::once(Choice::new("", "Choose a placed part…"))
        .chain(
            placed_names(context.before)
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
    f.reference(
        "axis",
        "along or round",
        args.axis.iter().cloned().collect(),
        &[Role::Line, Role::Round],
        None,
        false,
        |e, picked| e.args.axis = picked.into_iter().next(),
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
    let mut seen = Seen::new(context.before);
    let (key, labels, unit) = if circular {
        ("angle", ["angle between", "total angle"], Unit::Angle)
    } else {
        ("spacing", ["spacing", "total"], Unit::Length)
    };
    args.layout
        .spacing()
        .show(&mut f.form.dialog, key, labels, |label, value| {
            formula_number(label, value, &mut seen, unit)
        });
    for field in [format!("{key}_mode"), key.to_string()] {
        let at = field.clone();
        f.on(field, move |Edit { args, .. }, value| {
            args.layout.spacing_mut().set(key, &at, &value);
        });
    }
    count_field(&mut f.form.dialog, "count", &args.count, &mut seen, None);
    f.on("count", |Edit { args, .. }, value| {
        if let Some(count) = count_of(&value) {
            args.count = count;
        }
    });
    f.form.dialog.text(
        "hint",
        "Each copy is a placed part of its own, named part_pattern(step,k); it moves with the part.",
        Tone::Hint,
    );
    f
}
