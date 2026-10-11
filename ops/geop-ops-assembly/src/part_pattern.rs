//! [`PartPattern`]: copies of a placed part, in a row or round an axis.

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
    part::Retrieve,
};
use geop_ops::{
    Design, Namer, ParamValue,
    operation::{Aspects, Build, BuildReader, EntityRef, Input, Role, Seen},
    placed::{Instance, Placement, instance, instance_name},
    ui::{Choice, Edit, EditForm, Number, Tone, Unit},
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
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Layout {
    /// Each copy `spacing` further along the line's direction.
    Linear { spacing: f64 },
    /// Each copy turned `angle` degrees further round the axis.
    Circular { angle: f64 },
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
    pub layout: Layout,
    /// How many there are, the part itself included.
    pub count: usize,
}

/// The motion moving the part to copy `k`: along `axis` by `k` spacings,
/// or round it by `k` angles.
fn motion<S: Scalar>(layout: Layout, axis: &Axis<S>, k: usize) -> GeopResult<Pose<S>> {
    let k = S::from_i64(k as i64);
    match layout {
        Layout::Linear { spacing } => Pose::new(
            axis.direction.prod_scalar(S::from_f64(spacing).mul(k)),
            Quaternion::identity(),
        ),
        Layout::Circular { angle } => Pose::rotation_about(
            &axis.point,
            &axis.direction,
            S::from_f64(angle).mul(k).degrees_to_radians()?,
        ),
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
    let Some(axis) = &args.axis else {
        return Err(GeopError::new(
            "pick the line the copies go along, or the axis they go round",
        ));
    };
    if args.count < 2 {
        return Err(GeopError::new(format!(
            "a pattern of {} is no pattern: it needs 2 or more",
            args.count
        )));
    }
    let step = match args.layout {
        Layout::Linear { spacing } => spacing,
        Layout::Circular { angle } => angle,
    };
    if !step.is_finite() || step == 0.0 {
        return Err(GeopError::new(format!(
            "the copies would all be in one place: {:?}",
            args.layout
        )));
    }
    Ok(axis)
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

    /// The newest placed part, six round an axis still to pick.
    fn new_args(context: &Context<Input, ParamValue>) -> PartPatternArgs {
        PartPatternArgs {
            part: placed_names(context.before).pop().unwrap_or_default(),
            axis: None,
            layout: Layout::Circular { angle: 60.0 },
            count: 6,
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
        let axis = Aspects::axis_of(axis, &mut Seen::new(part.current())).with_context(ctx)?;
        let namer = Namer::new("part_pattern", step)?;
        for k in 1..args.count {
            let motion = motion(args.layout, &axis, k).with_context(ctx)?;
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
            args.layout = match (way, args.layout) {
                ("linear", Layout::Circular { .. }) => Layout::Linear { spacing: 1.0 },
                ("circular", Layout::Linear { .. }) => Layout::Circular {
                    angle: 360.0 / args.count.max(1) as f64,
                },
                (_, layout) => layout,
            };
        },
    );
    match args.layout {
        Layout::Linear { spacing } => {
            f.number(
                "spacing",
                Number::new("spacing", spacing, Unit::Length),
                |Edit { args, .. }, v| args.layout = Layout::Linear { spacing: v },
            );
        }
        Layout::Circular { angle } => {
            f.number(
                "angle",
                Number::new("angle between", angle, Unit::Angle),
                |Edit { args, .. }, v| args.layout = Layout::Circular { angle: v },
            );
        }
    }
    f.number(
        "count",
        Number::new("count", args.count as f64, Unit::Count).range(2.0, 100.0),
        |Edit { args, .. }, v| args.count = v.round().max(2.0) as usize,
    );
    f.form.dialog.text(
        "hint",
        "Each copy is a placed part of its own, named part_pattern(step,k); it moves with the part.",
        Tone::Hint,
    );
    f
}
