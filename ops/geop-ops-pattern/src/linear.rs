//! [`LinearPattern`]: copies of bodies in a row, or in a grid of two rows.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{DatumComponent, FrameAxis, Motion},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, ORIGIN, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use crate::common::{
    Spacing, bodies_field, center, combine_instances, copy_seeds, count_number, count_of,
    direction, newest_solid, seed_instances, seeds, track,
};

/// Copies the bodies `bodies` — solids, and sheets by one of their faces —
/// in a row along a direction, or in a grid along two: `count` instances
/// along each, the bodies themselves the first, each `Spacing::Step` from
/// the last or spread evenly over `Spacing::Extent`.
///
/// The copies are kept as new bodies, or combined as [`Combine`] says:
/// joined to, cut from or intersected with the target, the bodies
/// themselves too unless they are the target — so a tool built as a new
/// body, patterned and cut from a plate, drills a row of holes. Combined,
/// the result is named `linear_pattern(P)` for the operation `P`.
///
/// The copy of each entity named `X` in the instance `i` along the first
/// direction is named `linear_pattern(P,i,X)` — `i` counting from `1`, the
/// bodies themselves being `0` — and in the instance `i`, `j` of a grid
/// `linear_pattern(P,i.j,X)`: "copy 3's top face" is
/// `linear_pattern(P,3,extrude(E,end))` however the bodies change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LinearPattern;

/// One direction of a [`LinearPattern`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Direction {
    /// What gives the direction: a line — a straight edge, an axis, a
    /// sketch line — or a plane, along its normal; none, for a step that
    /// has not picked one.
    pub along: Option<EntityRef>,
    /// Go against it instead.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances along it, the bodies themselves included.
    pub count: usize,
    /// How far apart they are.
    pub spacing: Spacing,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinearPatternArgs {
    /// The bodies to copy: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
    pub first: Direction,
    /// A second direction, making a grid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub second: Option<Direction>,
    /// Keep the copies as new bodies, or combine them with a solid.
    #[serde(default)]
    pub combine: Combine,
}

impl Direction {
    /// Along the origin's axis `axis`, three instances a unit apart.
    fn along(axis: FrameAxis) -> Self {
        Direction {
            along: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Axis(axis),
            )),
            reversed: false,
            count: 3,
            spacing: Spacing::Step(1.0),
        }
    }

    /// The unit vector it goes along in `part`.
    fn unit<S: Scalar>(&self, part: &Part<S>) -> GeopResult<Vector3<S>> {
        let along = self
            .along
            .as_ref()
            .ok_or_else(|| GeopError::new("pick a direction to pattern along"))?;
        let d = direction(part, along)?;
        Ok(if self.reversed { d.neg() } else { d })
    }

    /// How far apart two neighbouring instances are, as `step / count`:
    /// none for a single instance, which has no neighbour.
    fn step(&self) -> GeopResult<Option<(f64, usize)>> {
        let value = self.spacing.value();
        if !value.is_finite() || value == 0.0 {
            return Err(GeopError::new(format!(
                "the spacing {value} puts every copy on top of the bodies"
            )));
        }
        if self.count == 0 {
            return Err(GeopError::new("a pattern has at least one instance"));
        }
        Ok((self.count > 1).then_some(match self.spacing {
            Spacing::Step(step) => (step, 1),
            Spacing::Extent(extent) => (extent, self.count - 1),
        }))
    }

    /// Where its instance `i` is, relative to the bodies: `i` steps along
    /// `unit` — none for the bodies themselves.
    fn offset<S: Scalar>(
        unit: &Vector3<S>,
        step: Option<(f64, usize)>,
        i: usize,
    ) -> GeopResult<Option<Vector3<S>>> {
        let Some((value, divisor)) = step.filter(|_| i > 0) else {
            return Ok(None);
        };
        let distance = S::from_f64(value)
            .mul(S::from_i64(i as i64))
            .div(S::from_i64(divisor as i64))?;
        Ok(Some(unit.prod_scalar(distance)))
    }

    /// Its fields in `form`, keyed with `suffix`: what it goes along, its
    /// sense, the count and the spacing, with handles from `at`.
    fn show<'a, S: Scalar>(
        &self,
        form: &mut Form<'a, S, LinearPatternArgs>,
        before: &Part<S>,
        at: Option<Vector3<S>>,
        suffix: &str,
        get: fn(&mut LinearPatternArgs) -> &mut Direction,
    ) {
        form.reference(
            &format!("direction{suffix}"),
            "direction",
            self.along.iter().cloned().collect(),
            &[Role::Line, Role::Plane],
            None,
            false,
            move |edit, picked| get(edit.args).along = picked.into_iter().next(),
        );
        form.checkbox(
            &format!("reversed{suffix}"),
            "reverse direction",
            self.reversed,
            move |args, b| get(args).reversed = b,
        );
        // Moving the spacing's handle by the unit direction adds one to it;
        // the count's, by one step, adds an instance.
        let unit = self.unit(before).ok();
        let step = self.step().ok().flatten();
        let step_length = step.map(|(value, divisor)| value / divisor as f64);
        let handles = at.zip(unit);
        let count_handle = handles.zip(step_length).and_then(|((at, unit), step)| {
            let end = (self.count.saturating_sub(1)) as f64 * step;
            track(
                at.add(&unit.prod_scalar(S::from_f64(end))),
                unit.prod_scalar(S::from_f64(step)),
            )
        });
        let mut count = count_number(self.count, count_handle);
        count.step = 1.0;
        form.number(&format!("count{suffix}"), count, move |args, value| {
            get(args).count = count_of(value)
        });
        let spacing_handle = handles.and_then(|(at, unit)| {
            track(
                at.add(&unit.prod_scalar(S::from_f64(self.spacing.value()))),
                unit,
            )
        });
        self.spacing.show(
            form,
            &format!("spacing{suffix}"),
            ["spacing", "total"],
            move |label, value| {
                Number::new(label, value, Unit::Length).handle(spacing_handle.clone())
            },
            move |args| &mut get(args).spacing,
        );
    }
}

impl Operation for LinearPattern {
    type Args = LinearPatternArgs;
    type Session = ();

    /// The newest solid, three times a unit apart along the origin's `x`
    /// axis, kept as new bodies.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> LinearPatternArgs {
        LinearPatternArgs {
            bodies: newest_solid(before),
            first: Direction::along(FrameAxis::X),
            second: None,
            combine: Combine::NewBody,
        }
    }

    /// The bodies, picked; each direction — the second turned on or off —
    /// with its count and spacing, both dragged by handles; and how to
    /// combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &LinearPatternArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, LinearPatternArgs> {
        let before = context.before;
        let mut f = Form::<S, LinearPatternArgs>::new();
        bodies_field(&mut f, &args.bodies, |args| &mut args.bodies);
        let at = center(before, &args.bodies);
        args.first
            .show(&mut f, before, at, "", |args| &mut args.first);
        f.checkbox(
            "grid",
            "second direction",
            args.second.is_some(),
            |args, on| {
                args.second = on.then(|| Direction::along(FrameAxis::Y));
            },
        );
        if let Some(second) = &args.second {
            second.show(&mut f, before, at, "2", |args| {
                args.second
                    .get_or_insert_with(|| Direction::along(FrameAxis::Y))
            });
        }
        args.combine.show(&mut f, before, |args| &mut args.combine);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &LinearPatternArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("linear_pattern({operation_id}, {args:?})");
        let namer = Namer::new("linear_pattern", operation_id)?;
        let seeds = seeds(&part, &args.bodies).with_context(ctx)?;
        let first = args.first.unit(&part).with_context(ctx)?;
        let first_step = args.first.step().with_context(ctx)?;
        let second = match &args.second {
            None => None,
            Some(direction) => {
                let unit = direction.unit(&part).with_context(ctx)?;
                let cross = first.prod_cross(&unit);
                if (0..3).all(|k| cross[k].could_be_equal(S::ZERO)) {
                    return Err(GeopError::new(
                        "the two directions are parallel: the grid's rows would lie on top of each other",
                    ))
                    .with_context(ctx);
                }
                Some((direction, unit, direction.step().with_context(ctx)?))
            }
        };
        let rows = second.as_ref().map_or(1, |(d, ..)| d.count);
        let mut instances = seed_instances(&seeds, if second.is_some() { "0.0" } else { "0" });
        for i in 0..args.first.count {
            for j in 0..rows {
                let along = Direction::offset(&first, first_step, i).with_context(ctx)?;
                let across = match &second {
                    Some((_, unit, step)) => Direction::offset(unit, *step, j).with_context(ctx)?,
                    None => None,
                };
                let offset = match (along, across) {
                    (None, None) => continue,
                    (Some(a), None) => a,
                    (None, Some(b)) => b,
                    (Some(a), Some(b)) => a.add(&b),
                };
                let label = match second {
                    Some(_) => format!("{i}.{j}"),
                    None => i.to_string(),
                };
                instances.extend(
                    copy_seeds(
                        &mut part,
                        &seeds,
                        &Motion::translation(offset),
                        &label,
                        |name| namer.name(&[&label, name]),
                    )
                    .with_context(ctx)?,
                );
            }
        }
        combine_instances(&mut part, &namer, operation_id, &args.combine, &instances)
            .with_context(ctx)?;
        Ok(part)
    }
}
