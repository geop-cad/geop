//! [`CircularPattern`]: copies of bodies turned around an axis.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
};
use geop_core_program::{
    editing::{Event, Form, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::parameters::expressions;
use geop_ops::{
    Design, NamedModel, ParamValue,
    operation::{
        Aspects, Build, BuildReader, EntityRef, Lookup, ModelStep, Role, Seen,
        pattern::{Row, Spacing, count_field, count_of, whole_count},
    },
    origin_axis,
    parameters::Formula,
    ui::formula_number,
};
use serde::{Deserialize, Serialize};

use crate::common::{Naming, Pattern, Seeds, build_pattern, run_pattern};

/// Copies the bodies `bodies` — solids, and sheets by one of their faces —
/// or does the features `features` again, turned around an axis: `count` instances, the bodies themselves the
/// first, each `Spacing::Step` degrees from the last, or spread evenly over
/// `Spacing::Extent` degrees — over a full turn of 360°, the last a step
/// short of coming round onto the first again. Turning goes
/// counter-clockwise looking against the axis, the other way if
/// `reversed`.
///
/// The axis is a line — a straight edge, an axis, a sketch line — or what
/// a round entity turns around: a circular edge, a cylindrical face.
///
/// Kept, combined — a feature as it combined — and named as
/// [`crate::LinearPattern`] does, as
/// `circular_pattern(P)` and `circular_pattern(P,i,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CircularPattern;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CircularPatternArgs {
    /// The bodies, or the features, to copy, and how to combine copies of
    /// bodies.
    #[serde(flatten)]
    pub seeds: Seeds,
    /// What to turn around; none, for a step that has not picked it.
    pub axis: Option<EntityRef>,
    /// Turn the other way.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances, the bodies themselves included: a number, or a
    /// formula of the part's parameters.
    pub count: Formula,
    /// How far apart they are, in degrees.
    pub angle: Spacing,
}

impl Pattern for CircularPatternArgs {
    const NAMING: Naming = Naming {
        kind: "circular_pattern",
        labelled: true,
    };

    fn seeds(&self) -> &Seeds {
        &self.seeds
    }

    fn seed_label(&self) -> &'static str {
        "0"
    }

    fn placements<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Vec<(String, Motion<S>)>> {
        let Some(axis_ref) = &self.axis else {
            return Err(GeopError::new("pick an axis to turn around"));
        };
        let axis = Aspects::axis_of(axis_ref, lookup)?;
        let direction = if self.reversed {
            axis.direction.neg()
        } else {
            axis.direction
        };
        let row = self.row(|f| f.evaluate(lookup))?;
        let mut placements = Vec::new();
        for k in 1..row.count {
            let motion = row.turn(&axis.point, &direction, k)?.motion();
            placements.push((k.to_string(), motion));
        }
        Ok(placements)
    }
}

impl CircularPatternArgs {
    /// The instances round the axis, with `value` giving the count's and
    /// the angle's values.
    fn row(&self, mut value: impl FnMut(&Formula) -> GeopResult<f64>) -> GeopResult<Row> {
        let count = whole_count(&self.count, value(&self.count)?)?;
        self.angle.round(count, value(self.angle.value())?)
    }
}

impl ModelStep for CircularPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &CircularPatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}

impl Operation for CircularPattern {
    const INFO: OperationInfo = OperationInfo::new(
        "circular_pattern",
        "Circular pattern",
        "Copy bodies turned around an axis.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .icon(geop_ops::ui::icons::CIRCULAR_PATTERN);
    type Args = CircularPatternArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut CircularPatternArgs) -> Vec<&mut String> {
        expressions([&mut args.count, args.angle.value_mut()])
    }

    /// Four times around the origin's `z` axis, kept as new bodies.
    fn new_args(_: &Context<Input, ParamValue>) -> CircularPatternArgs {
        CircularPatternArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            axis: Some(origin_axis(FrameAxis::Z)),
            reversed: false,
            count: Formula::Plain(4.0),
            angle: Spacing::extent(360.0),
        }
    }

    fn run(step: &str, part: &mut Build, args: &CircularPatternArgs) -> GeopResult<()> {
        run_pattern::<Self>(part, step, args)
    }

    /// The bodies or the features, and the axis, picked; the count and the
    /// angles; and how to combine copies of bodies.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &CircularPatternArgs,
        _: &(),
    ) -> Form<Design> {
        let mut seen = Seen::new(context.before);
        let mut form = Form::default();
        args.seeds.show(&mut form, |form| {
            let dialog = &mut form.dialog;
            let axis = args.axis.iter().cloned().collect();
            (dialog.reference(
                "axis",
                "axis",
                axis,
                &[Role::Line, Role::Round],
                None,
                false,
            ))
            .checkbox("reversed", "reverse direction", args.reversed);
            count_field(dialog, "count", &args.count, &mut seen, None);
            args.angle
                .show(dialog, "angle", ["angle", "total angle"], |label, value| {
                    formula_number(label, value, &mut seen, Unit::Angle).range(-360.0, 360.0)
                });
        });
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &CircularPatternArgs,
        _: &(),
        event: Event<Design>,
    ) -> (CircularPatternArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), &value) {
            ("axis", Value::Entities(picked)) => args.axis = picked.first().cloned(),
            ("reversed", Value::Bool(b)) => args.reversed = *b,
            ("count", value) => {
                if let Some(count) = count_of(value) {
                    args.count = count;
                }
            }
            (key, value) => {
                let _ = args.angle.set("angle", key, value)
                    || (args.seeds).set(key, value, Seen::new(context.before).model());
            }
        }
        (args, ())
    }
}
