//! [`CircularPattern`]: copies of bodies turned around an axis.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
};
use geop_core_program::{numeric::NumericInput, part::StepContext, reader::Reader, values::Design};
use geop_ops_datums::origin_axis;
use geop_ops_model::{ModelOperation, NamedModel};
use serde::{Deserialize, Serialize};

use crate::{
    common::{Naming, Pattern, Seeds, build_pattern, run_pattern},
    references::TurnAxis,
    spacing::{Row, Spacing, count_field, whole_count},
};

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
/// Copies are held apart or combined — a feature as it combined — and named as
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
    pub axis: Option<TurnAxis>,
    /// Turn the other way.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances, the bodies themselves included: a number, or a
    /// formula of the part's parameters.
    pub count: NumericInput,
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

    fn placements(&self, reader: &Reader) -> GeopResult<Vec<(String, Motion<Design>)>> {
        let Some(axis) = &self.axis else {
            return Err(GeopError::new("pick an axis to turn around"));
        };
        let axis = axis.resolve(reader)?;
        let direction = if self.reversed {
            axis.direction.neg()
        } else {
            axis.direction
        };
        let row = self.row(reader)?;
        let mut placements = Vec::new();
        for k in 1..row.count {
            let motion = row.turn(&axis.point, &direction, k)?.motion();
            placements.push((k.to_string(), motion));
        }
        Ok(placements)
    }
}

impl CircularPatternArgs {
    /// The instances round the axis, as `reader` reads the parameters.
    fn row(&self, reader: &Reader) -> GeopResult<Row> {
        let count = whole_count(&self.count, reader)?;
        self.angle.round(count, reader)
    }
}

part_operation!(CircularPattern(CircularPatternArgs));

impl PartOperation for CircularPattern {
    const KIND: &'static str = "circular_pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Circular pattern",
        "Copy bodies turned around an axis.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .icon(geop_core_editor::view::icons::CIRCULAR_PATTERN);
    type Args = CircularPatternArgs;
    type Session = ();

    /// Four times around the origin's `z` axis, kept as new bodies.
    fn new_args(_: &Context) -> CircularPatternArgs {
        CircularPatternArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
            reversed: false,
            count: NumericInput::Plain(4.0),
            angle: Spacing::extent(360.0),
        }
    }

    fn run(ctx: &mut StepContext, args: &CircularPatternArgs) -> GeopResult<()> {
        let _step = ctx.step();
        run_pattern::<Self>(ctx, args)
    }

    /// The bodies or the features, and the axis, picked; the count and the
    /// angles; and how to combine copies of bodies.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &CircularPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, CircularPatternArgs> {
        let before = &context.before;
        let mut f = EditForm::default();
        let set_seeds = |args: &mut CircularPatternArgs, seeds| args.seeds = seeds;
        (args.seeds).fields(&mut f, before, set_seeds, |f| {
            f.pick_one(
                "axis",
                "axis",
                before,
                args.axis.as_ref(),
                |edit, picked| edit.args.axis = Some(picked),
            )
            .checkbox("reversed", "reverse direction", args.reversed, |edit, b| {
                edit.args.reversed = b
            });
            count_field(f, "count", &args.count, before, None, |edit, count| {
                edit.args.count = count
            });
            args.angle.fields(
                f,
                "angle",
                ["angle", "total angle"],
                |label, value| {
                    Number::numeric(value, label, before, Unit::Angle).range(-360.0, 360.0)
                },
                |args, angle| args.angle = angle,
            );
        });
        f
    }
}

impl ModelOperation for CircularPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &CircularPatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}
