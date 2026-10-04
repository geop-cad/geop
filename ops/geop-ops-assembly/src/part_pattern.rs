//! [`PartPattern`]: copies of a placed part, in a row or round an axis.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{
    Context, Instance, Library, Namer, Part,
    operation::{Aspects, EntityRef, Operation, Role},
    ui::{Choice, Form, Number, Tone, Unit},
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
/// axis is picked in the part as placed so far: an edge or axis of a part
/// placed before moves the pattern with that part.
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

/// The line or axis `entity` is in `part`: a straight edge's or datum
/// axis's line, or the axis something round turns about.
fn axis_of<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let aspects = Aspects::of(entity, part)?;
    let axis = aspects.line.or(aspects.round).ok_or_else(|| {
        GeopError::new(format!(
            "{entity} has no line or axis: pick a straight or circular edge, a round face or a datum axis"
        ))
    })?;
    Ok((axis.point, axis.direction.normalize()?))
}

/// The motion moving the part to copy `k`: along `direction` by `k`
/// spacings, or round the axis through `point` by `k` angles.
fn motion<S: Scalar>(
    layout: Layout,
    point: &Vector3<S>,
    direction: &Vector3<S>,
    k: usize,
) -> GeopResult<Pose<S>> {
    let k = S::from_i64(k as i64);
    match layout {
        Layout::Linear { spacing } => Pose::new(
            direction.prod_scalar(S::from_f64(spacing).mul(k)),
            Quaternion::identity(),
        ),
        Layout::Circular { angle } => {
            let half = S::from_f64(angle).mul(k).mul(S::PI.div(S::from_i64(360))?);
            let rotation = Quaternion::new(
                half.cos(),
                direction[0].mul(half.sin()),
                direction[1].mul(half.sin()),
                direction[2].mul(half.sin()),
            );
            // Turned about the axis through `point`: `p + R (x - p)`.
            let turned = Pose::new(Vector3::zero(), rotation)?;
            let position = point.sub(&turned.apply(point));
            Pose::new(position, rotation)
        }
    }
}

impl Operation for PartPattern {
    type Args = PartPatternArgs;
    type Session = ();

    /// The newest placed part, six round an axis still to pick.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> PartPatternArgs {
        let part = before
            .instances()
            .filter_map(|(id, _)| before.name_of(id))
            .last()
            .unwrap_or_default()
            .to_string();
        PartPatternArgs {
            part,
            axis: None,
            layout: Layout::Circular { angle: 60.0 },
            count: 6,
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &PartPatternArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("part_pattern({operation_id}, {args:?})");
        if args.part.is_empty() {
            return Err(GeopError::new("choose the placed part to pattern")).with_context(ctx);
        }
        let Some(axis) = &args.axis else {
            return Err(GeopError::new(
                "pick the line the copies go along, or the axis they go round",
            ))
            .with_context(ctx);
        };
        if args.count < 2 {
            return Err(GeopError::new(format!(
                "a pattern of {} is no pattern: it needs 2 or more",
                args.count
            )))
            .with_context(ctx);
        }
        let step = match args.layout {
            Layout::Linear { spacing } => spacing,
            Layout::Circular { angle } => angle,
        };
        if !step.is_finite() || step == 0.0 {
            return Err(GeopError::new(format!(
                "the copies would all be in one place: {:?}",
                args.layout
            )))
            .with_context(ctx);
        }
        let seed = part
            .instance(part.instance_id(&args.part).map_err(|e| {
                e.with_context(format!(
                    "{:?} is no part placed so far: pattern one an earlier step placed",
                    args.part
                ))
            })?)
            .with_context(ctx)?
            .clone();
        let (point, direction) = axis_of(&part, axis).with_context(ctx)?;
        let namer = Namer::new("part_pattern", operation_id)?;
        for k in 1..args.count {
            let pose = motion(args.layout, &point, &direction, k)
                .with_context(ctx)?
                .compose(&seed.pose);
            let copy = Instance {
                component: seed.component.clone(),
                pose,
                parameter: None,
                fixed: true,
                flexible: false,
            };
            part.add_instance(copy, namer.name(&[&k.to_string()]))
                .with_context(ctx)?;
        }
        Ok(part)
    }

    /// The part to copy, the line or axis, the layout and how many.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &PartPatternArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, PartPatternArgs> {
        let mut f = Form::<S, PartPatternArgs>::new();
        let before = context.before;
        let options = std::iter::once(Choice::new("", "Choose a placed part…"))
            .chain(
                before
                    .instances()
                    .filter_map(|(id, _)| before.name_of(id))
                    .map(|name| Choice::new(name, name)),
            )
            .collect();
        f.select(
            "part",
            "part",
            args.part.clone(),
            options,
            true,
            |args, part| args.part = part.to_string(),
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
        f.select("layout", "layout", way, choices, false, |args, way| {
            args.layout = match (way, args.layout) {
                ("linear", Layout::Circular { .. }) => Layout::Linear { spacing: 1.0 },
                ("circular", Layout::Linear { .. }) => Layout::Circular {
                    angle: 360.0 / args.count.max(1) as f64,
                },
                (_, layout) => layout,
            };
        });
        match args.layout {
            Layout::Linear { spacing } => {
                f.number(
                    "spacing",
                    Number::new("spacing", spacing, Unit::Length),
                    |args, v| args.layout = Layout::Linear { spacing: v },
                );
            }
            Layout::Circular { angle } => {
                f.number(
                    "angle",
                    Number::new("angle between", angle, Unit::Angle),
                    |args, v| args.layout = Layout::Circular { angle: v },
                );
            }
        }
        f.number(
            "count",
            Number::new("count", args.count as f64, Unit::Count).range(2.0, 100.0),
            |args, v| args.count = v.round().max(2.0) as usize,
        );
        f.text(
            "hint",
            "Each copy is a placed part of its own, named part_pattern(step,k); it moves with the part.",
            Tone::Hint,
        );
        f
    }
}
