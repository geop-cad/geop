//! [`Mirror`]: the mirror image of bodies in a plane.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
};
use geop_core_program::{part::StepContext, reader::Reader, values::Design};
use geop_ops_model::datums::origin_plane;
use geop_ops_model::sketch::SketchPlane;
use geop_ops_model::{ModelOperation, NamedModel};
use serde::{Deserialize, Serialize};

use crate::common::{Naming, Pattern, Seeds, build_pattern, run_pattern};

/// Mirrors the bodies `bodies` — solids, and sheets by one of their faces —
/// or does the features `features` again mirrored, in a plane: a planar face, a datum plane, a frame's plane. The mirror
/// image is a valid body of its own, its faces turned around so that they
/// face out again; the bodies themselves stay.
///
/// The image is kept as new bodies, or combined as [`geop_ops_model::Combine`] says —
/// joined to the bodies themselves, it makes a symmetric part of a half
/// built against the plane. Combined, the result is named `mirror(M)` for
/// the operation `M`; the image of each entity named `X` is named
/// `mirror(M,X)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Mirror;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MirrorArgs {
    /// The bodies, or the features, to mirror, and how to combine an image
    /// of bodies.
    #[serde(flatten)]
    pub seeds: Seeds,
    /// The plane to mirror in — a flat face, a datum plane, a sketch's
    /// plane; none, for a step that has not picked it.
    pub plane: Option<SketchPlane>,
}

impl Pattern for MirrorArgs {
    const NAMING: Naming = Naming {
        kind: "mirror",
        labelled: false,
    };

    fn seeds(&self) -> &Seeds {
        &self.seeds
    }

    fn seed_label(&self) -> &'static str {
        "seed"
    }

    /// The one image, mirrored in the plane.
    fn placements(&self, reader: &Reader) -> GeopResult<Vec<(String, Motion<Design>)>> {
        let Some(plane) = &self.plane else {
            return Err(GeopError::new("pick a plane to mirror in"));
        };
        let plane = plane.resolve(reader)?;
        Ok(vec![(
            "image".to_string(),
            Motion::mirror(plane.origin(), plane.w())?,
        )])
    }
}

part_operation!(Mirror(MirrorArgs));

impl PartOperation for Mirror {
    const KIND: &'static str = "mirror";
    const INFO: OperationInfo = OperationInfo::new(
        "Mirror",
        "Mirror bodies in a plane.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .icon(geop_core_editor::view::icons::MIRROR);
    type Args = MirrorArgs;
    type Session = ();

    /// Mirrored in the origin's `yz` plane, kept as a new body.
    fn new_args(_: &Context) -> MirrorArgs {
        MirrorArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            plane: Some(SketchPlane::Datum(origin_plane(FrameAxis::X))),
        }
    }

    fn run(ctx: &mut StepContext, args: &MirrorArgs) -> GeopResult<()> {
        let _step = ctx.step();
        run_pattern::<Self>(ctx, args)
    }

    /// The bodies or the features, and the plane, picked, and how to
    /// combine an image of bodies.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &MirrorArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, MirrorArgs> {
        let before = &context.before;
        let mut f = EditForm::default();
        let set_seeds = |args: &mut MirrorArgs, seeds| args.seeds = seeds;
        (args.seeds).fields(&mut f, before, set_seeds, |f| {
            f.pick_one(
                "plane",
                "plane",
                before,
                args.plane.as_ref(),
                |edit, picked| edit.args.plane = Some(picked),
            );
        });
        f
    }
}

impl ModelOperation for Mirror {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &MirrorArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}
