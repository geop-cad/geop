//! [`CircularPattern`]: copies of bodies turned around an axis.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::Scalar,
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
    Spacing, axis, bodies_field, combine_instances, copy_seeds, count_number, count_of,
    newest_solid, seed_instances, seeds,
};

/// Copies the bodies `bodies` — solids, and sheets by one of their faces —
/// turned around an axis: `count` instances, the bodies themselves the
/// first, each `Spacing::Step` degrees from the last, or spread evenly over
/// `Spacing::Extent` degrees — over a full turn of 360°, the last a step
/// short of coming round onto the first again. Turning goes
/// counter-clockwise looking against the axis, the other way if
/// `reversed`.
///
/// The axis is a line — a straight edge, an axis, a sketch line — or what
/// a round entity turns around: a circular edge, a cylindrical face.
///
/// Kept, combined and named as [`crate::LinearPattern`] does, as
/// `circular_pattern(P)` and `circular_pattern(P,i,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CircularPattern;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CircularPatternArgs {
    /// The bodies to copy: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
    /// What to turn around; none, for a step that has not picked it.
    pub axis: Option<EntityRef>,
    /// Turn the other way.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances, the bodies themselves included.
    pub count: usize,
    /// How far apart they are, in degrees.
    pub angle: Spacing,
    /// Keep the copies as new bodies, or combine them with a solid.
    #[serde(default)]
    pub combine: Combine,
}

impl CircularPatternArgs {
    /// How far the instance `k` is turned, in degrees, as `value * k /
    /// divisor`. Refuses spacings that put copies on the bodies.
    fn angles(&self) -> GeopResult<(f64, usize)> {
        let value = self.angle.value();
        if !value.is_finite() || value == 0.0 {
            return Err(GeopError::new(format!(
                "the angle {value}° puts every copy on top of the bodies"
            )));
        }
        if self.count == 0 {
            return Err(GeopError::new("a pattern has at least one instance"));
        }
        let span = |step: f64| step.abs() * (self.count - 1) as f64;
        Ok(match self.angle {
            Spacing::Extent(extent) if extent.abs() == 360.0 => (extent, self.count),
            Spacing::Extent(extent) if extent.abs() > 360.0 => {
                return Err(GeopError::new(format!(
                    "the pattern spans {extent}°, more than a full turn: copies would come round onto each other"
                )));
            }
            Spacing::Extent(extent) => (extent, (self.count - 1).max(1)),
            Spacing::Step(step) if span(step) >= 360.0 => {
                return Err(GeopError::new(format!(
                    "{} steps of {step}° come round onto the bodies again: spread the instances over 360° instead",
                    self.count - 1
                )));
            }
            Spacing::Step(step) => (step, 1),
        })
    }
}

impl Operation for CircularPattern {
    type Args = CircularPatternArgs;
    type Session = ();

    /// The newest solid, four times around the origin's `z` axis, kept as
    /// new bodies.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> CircularPatternArgs {
        CircularPatternArgs {
            bodies: newest_solid(before),
            axis: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Axis(FrameAxis::Z),
            )),
            reversed: false,
            count: 4,
            angle: Spacing::Extent(360.0),
            combine: Combine::NewBody,
        }
    }

    /// The bodies and the axis, picked; the count and the angles; and how
    /// to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &CircularPatternArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, CircularPatternArgs> {
        let before = context.before;
        let mut f = Form::<S, CircularPatternArgs>::new();
        bodies_field(&mut f, &args.bodies, |args| &mut args.bodies);
        f.reference(
            "axis",
            "axis",
            args.axis.iter().cloned().collect(),
            &[Role::Line, Role::Round],
            None,
            false,
            |edit, picked| edit.args.axis = picked.into_iter().next(),
        );
        f.checkbox("reversed", "reverse direction", args.reversed, |args, b| {
            args.reversed = b
        });
        let mut count = count_number(args.count, None);
        count.step = 1.0;
        f.number("count", count, |args, value| args.count = count_of(value));
        args.angle.show(
            &mut f,
            "angle",
            ["angle", "total angle"],
            |label, value| Number::new(label, value, Unit::Angle).range(-360.0, 360.0),
            |args| &mut args.angle,
        );
        args.combine.show(&mut f, before, |args| &mut args.combine);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &CircularPatternArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("circular_pattern({operation_id}, {args:?})");
        let namer = Namer::new("circular_pattern", operation_id)?;
        let seeds = seeds(&part, &args.bodies).with_context(ctx)?;
        let Some(axis_ref) = &args.axis else {
            return Err(GeopError::new("pick an axis to turn around")).with_context(ctx);
        };
        let axis = axis(&part, axis_ref).with_context(ctx)?;
        let direction = if args.reversed {
            axis.direction.neg()
        } else {
            axis.direction
        };
        let (degrees, divisor) = args.angles().with_context(ctx)?;
        let mut instances = seed_instances(&seeds, "0");
        for k in 1..args.count {
            // `degrees * k / divisor` in radians, enclosed in one go rather
            // than accumulated step by step.
            let radians = S::from_f64(degrees)
                .mul(S::from_i64(k as i64))
                .mul(S::PI)
                .div(S::from_i64(180 * divisor as i64))
                .with_context(ctx)?;
            let motion = Pose::rotation_about(&axis.point, &direction, radians)
                .with_context(ctx)?
                .motion();
            let label = k.to_string();
            instances.extend(
                copy_seeds(&mut part, &seeds, &motion, &label, |name| {
                    namer.name(&[&label, name])
                })
                .with_context(ctx)?,
            );
        }
        combine_instances(&mut part, &namer, operation_id, &args.combine, &instances)
            .with_context(ctx)?;
        Ok(part)
    }
}
