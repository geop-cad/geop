//! [`LinearPattern`]: copies of bodies in a row, or in a grid of two rows.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    editing::Unit,
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_editor::numeric_input::NumericInput;
use geop_ops_datums::origin_axis;
use geop_ops_model::{Design, ModelOperation, NamedModel};
use geop_ops_parameters::{BuildPart, PartReader, StateValue, StepContext};
use serde::{Deserialize, Serialize};

use crate::{
    common::{Naming, Pattern, Seeds, build_pattern, run_pattern, track},
    references::PatternDirection,
    spacing::{Row, Spacing, count_field, whole_count},
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
/// The copies are kept as new bodies, or combined as [`geop_ops_booleans::Combine`] says:
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
    pub along: Option<PatternDirection>,
    /// Go against it instead.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances along it, the bodies themselves included: a
    /// number, or a formula of the part's parameters.
    pub count: NumericInput,
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

    fn placements(&self, reader: &PartReader) -> GeopResult<Vec<(String, Motion<Design>)>> {
        let first = self.first.unit(reader)?;
        let first_row = self.first.row(reader)?;
        let second = match &self.second {
            None => None,
            Some(direction) => {
                let unit = direction.unit(reader)?;
                let cross = first.prod_cross(&unit);
                if (0..3).all(|k| cross[k].could_be_equal(Design::ZERO)) {
                    return Err(GeopError::new(
                        "the two directions are parallel: the grid's rows would lie on top of each other",
                    ));
                }
                Some((unit, direction.row(reader)?))
            }
        };
        let rows = second.as_ref().map_or(1, |(_, row)| row.count);
        let mut placements = Vec::new();
        for i in 0..first_row.count {
            for j in 0..rows {
                let along = first_row.offset(&first, i)?;
                let across = match &second {
                    Some((unit, row)) => row.offset(unit, j)?,
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
            along: Some(PatternDirection::Datum(origin_axis(axis))),
            reversed: false,
            count: NumericInput::Plain(3.0),
            spacing: Spacing::step(1.0),
        }
    }

    /// The unit vector it goes along, as `reader` reads the part.
    fn unit(&self, reader: &PartReader) -> GeopResult<Vector3<Design>> {
        let along = (self.along.as_ref())
            .ok_or_else(|| GeopError::new("pick a direction to pattern along"))?;
        let d = along.direction(reader)?;
        Ok(if self.reversed { d.neg() } else { d })
    }

    /// The instances along it, as `reader` reads the parameters.
    fn row(&self, reader: &PartReader) -> GeopResult<Row> {
        let count = whole_count(&self.count, reader)?;
        self.spacing.along(count, reader)
    }

    /// Its fields in `f`, keyed with `suffix`: what it goes along, its
    /// sense, the count and the spacing, with handles from `at`. `set` puts
    /// the direction set where the arguments keep it.
    fn fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, Design, A>,
        before: &PartReader<'a>,
        at: Option<Vector3<Design>>,
        suffix: &str,
        set: impl Fn(&mut A, Direction) + Clone + 'a,
    ) {
        // What setting one of its fields to `change` makes of it.
        fn with<A, V>(
            direction: Direction,
            set: impl Fn(&mut A, Direction) + Clone,
            change: fn(&mut Direction, V),
        ) -> impl Fn(&mut A, V) + Clone {
            move |args, value| {
                let mut direction = direction.clone();
                change(&mut direction, value);
                set(args, direction);
            }
        }
        let key = |name: &str| format!("{name}{suffix}");
        let set_along = with(self.clone(), set.clone(), |d, picked| d.along = Some(picked));
        let set_reversed = with(self.clone(), set.clone(), |d, b| d.reversed = b);
        f.pick_one(
            &key("direction"),
            "direction",
            before,
            self.along.as_ref(),
            move |edit, p| set_along(edit.args, p),
        )
        .checkbox(
            &key("reversed"),
            "reverse direction",
            self.reversed,
            move |edit, b| set_reversed(edit.args, b),
        );
        // Moving the spacing's handle by the unit direction adds one to it;
        // the count's, by one step, adds an instance.
        let unit = self.unit(before).ok();
        let row = self.row(before).ok();
        let handles = at.zip(unit);
        let count_handle =
            handles
                .zip(row.filter(|row| row.count > 1))
                .and_then(|((at, unit), row)| {
                    let step = row.step();
                    let end = (row.count - 1) as f64 * step;
                    track(
                        at.add(&unit.prod_scalar(Design::from_f64(end))),
                        unit.prod_scalar(Design::from_f64(step)),
                    )
                });
        let set_count = with(self.clone(), set.clone(), |d, count| d.count = count);
        count_field(
            f,
            &key("count"),
            &self.count,
            before,
            count_handle,
            move |edit, count| set_count(edit.args, count),
        );
        self.spacing.fields(
            f,
            &key("spacing"),
            ["spacing", "total"],
            |label, value| {
                let number = value.number(label, before, Unit::Length);
                let at = handles.and_then(|(at, unit)| {
                    track(
                        at.add(&unit.prod_scalar(Design::from_f64(number.value))),
                        unit,
                    )
                });
                number.handle(at)
            },
            with(self.clone(), set.clone(), |d, spacing| d.spacing = spacing),
        );
    }
}

part_operation!(LinearPattern(LinearPatternArgs));

impl PartOperation for LinearPattern {
    const KIND: &'static str = "linear_pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Linear pattern",
        "Copy bodies in a row along a direction, or in a grid along two.",
        OperationGroup::Bodies,
        OperationTier::Big,
    )
    .icon(geop_ops_view::icons::LINEAR_PATTERN);
    type Args = LinearPatternArgs;
    type Input = Design;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    /// Three times a unit apart along the origin's `x` axis, kept as new
    /// bodies.
    fn new_args(_: &StepContext) -> LinearPatternArgs {
        LinearPatternArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            first: Direction::along(FrameAxis::X),
            second: None,
        }
    }

    fn run(step: &str, part: &mut BuildPart, args: &LinearPatternArgs) -> GeopResult<()> {
        run_pattern::<Self>(part, step, args)
    }

    /// The bodies or the features, picked; each direction — the second
    /// turned on or off — with its count and spacing, both dragged by
    /// handles; and how to combine copies of bodies.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &LinearPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, LinearPatternArgs> {
        let before = &context.before;
        let mut f = EditForm::default();
        let at = args.seeds.center(before);
        let set_seeds = |args: &mut LinearPatternArgs, seeds| args.seeds = seeds;
        (args.seeds).fields(&mut f, before, set_seeds, |f| {
            args.first
                .fields(f, before, at, "", |args, first| args.first = first);
            f.checkbox(
                "grid",
                "second direction",
                args.second.is_some(),
                |edit, on| {
                    edit.args.second = on.then(|| Direction::along(FrameAxis::Y));
                },
            );
            if let Some(second) = &args.second {
                second.fields(f, before, at, "2", |args, second| {
                    args.second = Some(second)
                });
            }
        });
        f
    }
}

impl ModelOperation for LinearPattern {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &PartReader,
        step: &str,
        args: &LinearPatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}
