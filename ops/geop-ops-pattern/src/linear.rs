//! [`LinearPattern`]: copies of bodies in a row, or in a grid of two rows.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    editing::{Event, Form, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::parameters::expressions;
use geop_ops::{
    Design, NamedModel, ParamValue,
    operation::{Build, BuildReader, EntityRef, Lookup, ModelStep, Role, Seen},
    origin_axis,
    parameters::Formula,
    ui::formula_number,
};
use serde::{Deserialize, Serialize};

use crate::common::{
    Naming, Pattern, Seeds, Spacing, build_pattern, count_field, count_of, direction, run_pattern,
    track, whole_count,
};

/// Copies the bodies `bodies` — solids, and sheets by one of their faces —
/// or the features `features` in a row along a direction, or in a grid
/// along two: `count` instances along each, the bodies or features
/// themselves the first, each `Spacing::Step` from the last or spread
/// evenly over `Spacing::Extent`.
///
/// A feature — a cut, a boss, a hole — is done again at each instance:
/// its tools copied and combined with the solid it lies on as it combined
/// them (see [`crate::features`]), its cosmetic threads copied with it.
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
    /// How many instances along it, the bodies themselves included: a
    /// number, or a formula of the part's parameters.
    pub count: Formula,
    /// How far apart they are.
    pub spacing: Spacing,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinearPatternArgs {
    /// The bodies, or the features, to copy, and how to combine copies of
    /// bodies.
    #[serde(flatten)]
    pub seeds: Seeds,
    pub first: Direction,
    /// A second direction, making a grid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub second: Option<Direction>,
}

impl Pattern for LinearPatternArgs {
    const NAMING: Naming = Naming {
        kind: "linear_pattern",
        labelled: true,
    };

    fn seeds(&self) -> &Seeds {
        &self.seeds
    }

    /// `0`, or in a grid `0.0`.
    fn seed_label(&self) -> &'static str {
        if self.second.is_some() { "0.0" } else { "0" }
    }

    fn placements<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Vec<(String, Motion<S>)>> {
        let first = self.first.unit(lookup)?;
        let (first_count, first_step) = self.first.measure(|f| f.evaluate(lookup))?;
        let second = match &self.second {
            None => None,
            Some(direction) => {
                let unit = direction.unit(lookup)?;
                let cross = first.prod_cross(&unit);
                if (0..3).all(|k| cross[k].could_be_equal(S::ZERO)) {
                    return Err(GeopError::new(
                        "the two directions are parallel: the grid's rows would lie on top of each other",
                    ));
                }
                let (count, step) = direction.measure(|f| f.evaluate(lookup))?;
                Some((count, unit, step))
            }
        };
        let rows = second.as_ref().map_or(1, |(count, ..)| *count);
        let mut placements = Vec::new();
        for i in 0..first_count {
            for j in 0..rows {
                let along = Direction::offset(&first, first_step, i)?;
                let across = match &second {
                    Some((_, unit, step)) => Direction::offset(unit, *step, j)?,
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
                placements.push((label, Motion::translation(offset)));
            }
        }
        Ok(placements)
    }
}

impl Direction {
    /// Along the origin's axis `axis`, three instances a unit apart.
    fn along(axis: FrameAxis) -> Self {
        Direction {
            along: Some(origin_axis(axis)),
            reversed: false,
            count: Formula::Plain(3.0),
            spacing: Spacing::step(1.0),
        }
    }

    /// The unit vector it goes along, as `lookup` has the part.
    fn unit<S: Scalar>(&self, lookup: &mut (impl Lookup<S> + ?Sized)) -> GeopResult<Vector3<S>> {
        let along = (self.along.as_ref())
            .ok_or_else(|| GeopError::new("pick a direction to pattern along"))?;
        let d = direction(lookup, along)?;
        Ok(if self.reversed { d.neg() } else { d })
    }

    /// How many instances there are, and how far apart two neighbouring
    /// ones, as `step / count`: none for a single instance, which has no
    /// neighbour. `value` gives the count's and the spacing's values.
    fn measure(
        &self,
        mut value: impl FnMut(&Formula) -> GeopResult<f64>,
    ) -> GeopResult<(usize, Option<(f64, usize)>)> {
        let count = whole_count(&self.count, value(&self.count)?)?;
        let spacing = value(self.spacing.value())?;
        if !spacing.is_finite() || spacing == 0.0 {
            return Err(GeopError::new(format!(
                "the spacing {spacing} puts every copy on top of the bodies"
            )));
        }
        let step = (count > 1).then_some(match self.spacing {
            Spacing::Step(_) => (spacing, 1),
            Spacing::Extent(_) => (spacing, count - 1),
        });
        Ok((count, step))
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
    fn show(
        &self,
        form: &mut Form<Design>,
        seen: &mut Seen,
        at: Option<Vector3<Design>>,
        suffix: &str,
    ) {
        let dialog = &mut form.dialog;
        let along = self.along.iter().cloned().collect();
        let roles = [Role::Line, Role::Plane];
        (dialog.reference(
            &format!("direction{suffix}"),
            "direction",
            along,
            &roles,
            None,
            false,
        ))
        .checkbox(
            &format!("reversed{suffix}"),
            "reverse direction",
            self.reversed,
        );
        // Moving the spacing's handle by the unit direction adds one to it;
        // the count's, by one step, adds an instance.
        let unit = self.unit(seen).ok();
        let measured = self.measure(|f| f.evaluate(seen)).ok();
        let handles = at.zip(unit);
        let count_handle = handles
            .zip(measured)
            .and_then(|((at, unit), (count, step))| {
                let (value, divisor) = step?;
                let step = value / divisor as f64;
                let end = (count - 1) as f64 * step;
                track(
                    at.add(&unit.prod_scalar(Design::from_f64(end))),
                    unit.prod_scalar(Design::from_f64(step)),
                )
            });
        count_field(
            dialog,
            &format!("count{suffix}"),
            &self.count,
            seen,
            count_handle,
        );
        let spacing_key = format!("spacing{suffix}");
        self.spacing.show(
            dialog,
            &spacing_key,
            ["spacing", "total"],
            |label, value| {
                let number = formula_number(label, value, seen, Unit::Length);
                let at = handles.and_then(|(at, unit)| {
                    track(
                        at.add(&unit.prod_scalar(Design::from_f64(number.value))),
                        unit,
                    )
                });
                number.handle(at)
            },
        );
    }

    /// Its field `field` of those [`Direction::show`] shows, keyed with
    /// `suffix`, set to `value`, and whether it was one.
    fn set(&mut self, suffix: &str, field: &str, value: &Value) -> bool {
        let key = |name: &str| format!("{name}{suffix}");
        if field == key("direction") {
            if let Value::Entities(picked) = value {
                self.along = picked.first().cloned();
            }
        } else if field == key("reversed") {
            if let Value::Bool(b) = value {
                self.reversed = *b;
            }
        } else if field == key("count") {
            if let Some(count) = count_of(value) {
                self.count = count;
            }
        } else {
            return self.spacing.set(&key("spacing"), field, value);
        }
        true
    }
}

impl ModelStep for LinearPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &LinearPatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}

impl Operation for LinearPattern {
    const INFO: OperationInfo = OperationInfo::new(
        "linear_pattern",
        "Linear pattern",
        "Copy bodies in a row along a direction, or in a grid along two.",
        OperationGroup::Bodies,
        OperationTier::Big,
    );
    type Args = LinearPatternArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut LinearPatternArgs) -> Vec<&mut String> {
        expressions(
            std::iter::once(&mut args.first)
                .chain(args.second.as_mut())
                .flat_map(|d| [&mut d.count, d.spacing.value_mut()]),
        )
    }

    /// Three times a unit apart along the origin's `x` axis, kept as new
    /// bodies.
    fn new_args(_: &Context<Input, ParamValue>) -> LinearPatternArgs {
        LinearPatternArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            first: Direction::along(FrameAxis::X),
            second: None,
        }
    }

    fn run(step: &str, part: &mut Build, args: &LinearPatternArgs) -> GeopResult<()> {
        run_pattern::<Self>(part, step, args)
    }

    /// The bodies or the features, picked; each direction — the second
    /// turned on or off — with its count and spacing, both dragged by
    /// handles; and how to combine copies of bodies.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &LinearPatternArgs,
        _: &(),
    ) -> Form<Design> {
        let mut seen = Seen::new(context.before);
        let mut form = Form::default();
        let at = args.seeds.center(seen.model());
        args.seeds.show(&mut form, |form| {
            args.first.show(form, &mut seen, at, "");
            (form.dialog).checkbox("grid", "second direction", args.second.is_some());
            if let Some(second) = &args.second {
                second.show(form, &mut seen, at, "2");
            }
        });
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &LinearPatternArgs,
        _: &(),
        event: Event<Design>,
    ) -> (LinearPatternArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        let seen = Seen::new(context.before);
        match (key.as_str(), &value) {
            ("grid", Value::Bool(on)) => {
                args.second = on.then(|| Direction::along(FrameAxis::Y));
            }
            (key, value) => {
                let _ = args.seeds.set(key, value, seen.model())
                    || args.first.set("", key, value)
                    || (args.second.as_mut()).is_some_and(|second| second.set("2", key, value));
            }
        }
        (args, ())
    }
}
