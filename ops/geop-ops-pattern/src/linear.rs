//! [`LinearPattern`]: copies of bodies in a row, or in a grid of two rows.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{FrameAxis, Motion},
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_ops_model::spacing::{Direction, Frames};
use geop_ops_model::{ModelOperation, NamedModel};
use serde::{Deserialize, Serialize};

use crate::common::{Naming, Pattern, Seeds, build_pattern, placed_by, run_pattern};

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
/// The copies are kept as new bodies, or combined as [`geop_ops_model::Combine`] says:
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

    fn placements(&self, reader: &Reader) -> GeopResult<Vec<(String, Motion)>> {
        let frames = Frames::Linear {
            first: self.first.clone(),
            second: self.second.clone(),
        };
        placed_by(&frames, reader, &self.seeds)
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
    .shortcut("P")
    .icon(geop_core_editor::view::icons::LINEAR_PATTERN);
    type Args = LinearPatternArgs;
    type Session = ();

    /// Three times a unit apart along the origin's `x` axis, kept as new
    /// bodies.
    fn new_args(_: &Context) -> LinearPatternArgs {
        LinearPatternArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            first: Direction::along(FrameAxis::X),
            second: None,
        }
    }

    fn run(ctx: &mut StepContext, args: &LinearPatternArgs) -> GeopResult<()> {
        let _step = ctx.step();
        run_pattern::<Self>(ctx, args)
    }

    /// The bodies or the features, picked; each direction — the second
    /// turned on or off — with its count and spacing, both dragged by
    /// handles; and how to combine copies of bodies.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &LinearPatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, LinearPatternArgs> {
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
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &LinearPatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}
