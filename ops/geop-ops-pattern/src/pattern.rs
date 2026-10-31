//! [`Pattern`]: copies of bodies or features wherever a list of frames
//! puts them — in a row, round an axis, along a curve, at points, at the
//! offsets of a table, filling a face, at holes.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{geop_error::GeopResult, primitives::Motion};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_ops_model::spacing::{Frames, PointsRef};
use geop_ops_model::{ModelOperation, NamedModel};
use serde::{Deserialize, Serialize};

use crate::common::{Naming, Pattern as Repeats, Seeds, build_pattern, placed_by, run_pattern};

/// Copies the bodies `bodies` — solids, and sheets by one of their faces —
/// or does the features `features` again wherever `frames` puts them (see
/// [`Frames`]): in a row or a grid, round an axis, along a curve, at
/// points — the seed at the first, or where it is said to be — at the
/// offsets of a table, filling a flat face on a grid, or at the holes
/// features drilled. Linear and circular patterns are two of these, with
/// a dialog of their own ([`crate::LinearPattern`],
/// [`crate::CircularPattern`]).
///
/// Copies are kept apart or combined as [`crate::LinearPattern`]'s are, a
/// feature as it combined, its cosmetic threads and its holes copied with
/// it; they are named `pattern(P)` and `pattern(P,i,X)` for the copy
/// labelled `i` — `1`, `2`, ... along a curve, at points, at holes and
/// from a table; `i.j` in a grid or a fill.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pattern;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PatternArgs {
    /// The bodies, or the features, to copy, and how to combine copies of
    /// bodies.
    #[serde(flatten)]
    pub seeds: Seeds,
    /// Where the copies go.
    pub frames: Frames,
}

impl Repeats for PatternArgs {
    const NAMING: Naming = Naming {
        kind: "pattern",
        labelled: true,
    };

    fn seeds(&self) -> &Seeds {
        &self.seeds
    }

    /// `0`, or `0.0` in a grid or a fill.
    fn seed_label(&self) -> &'static str {
        match &self.frames {
            Frames::Linear {
                second: Some(_), ..
            }
            | Frames::Fill { .. } => "0.0",
            _ => "0",
        }
    }

    fn placements(&self, reader: &Reader) -> GeopResult<Vec<(String, Motion)>> {
        placed_by(&self.frames, reader, &self.seeds)
    }
}

part_operation!(Pattern(PatternArgs));

impl PartOperation for Pattern {
    const KIND: &'static str = "pattern";
    const INFO: OperationInfo = OperationInfo::new(
        "Pattern",
        "Copy bodies or features along a curve, at points, from a table of offsets, filling a face or at holes — or in a row, or round an axis.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .shortcut("Shift+A")
    .icon(crate::icons::PATTERN);
    type Args = PatternArgs;
    type Session = ();

    /// At points still to pick, kept as new bodies.
    fn new_args(_: &Context) -> PatternArgs {
        PatternArgs {
            seeds: Seeds::default(),
            frames: Frames::Points {
                points: Vec::<PointsRef>::new(),
                from: None,
            },
        }
    }

    fn run(ctx: &mut StepContext, args: &PatternArgs) -> GeopResult<()> {
        run_pattern::<Self>(ctx, args)
    }

    /// The bodies or the features, picked; where the copies go — which
    /// way, and what that way takes; and how to combine copies of bodies.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &PatternArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, PatternArgs> {
        let before = &context.before;
        let mut f = EditForm::default();
        let at = args.seeds.center(before);
        let set_seeds = |args: &mut PatternArgs, seeds| args.seeds = seeds;
        (args.seeds).fields(&mut f, before, set_seeds, |f| {
            args.frames
                .fields(f, before, at, |args: &mut PatternArgs, frames| {
                    args.frames = frames
                });
        });
        f
    }
}

impl ModelOperation for Pattern {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &PatternArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}
