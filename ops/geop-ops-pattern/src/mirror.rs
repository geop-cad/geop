//! [`Mirror`]: the mirror image of bodies in a plane.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel,
    operation::{
        Aspects, Build, BuildReader, EntityRef, Lookup, PartOperation, Role, Seen, StepContext,
    },
    origin_plane, part_operation,
    ui::{EditForm, FormState},
};
use serde::{Deserialize, Serialize};

use crate::common::{Naming, Pattern, Seeds, build_pattern, run_pattern};

/// Mirrors the bodies `bodies` — solids, and sheets by one of their faces —
/// or does the features `features` again mirrored, in a plane: a planar face, a datum plane, a frame's plane. The mirror
/// image is a valid body of its own, its faces turned around so that they
/// face out again; the bodies themselves stay.
///
/// The image is kept as new bodies, or combined as [`Combine`] says —
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
    /// The plane to mirror in; none, for a step that has not picked it.
    pub plane: Option<EntityRef>,
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
    fn placements<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Vec<(String, Motion<S>)>> {
        let Some(plane_ref) = &self.plane else {
            return Err(GeopError::new("pick a plane to mirror in"));
        };
        let plane = (Aspects::of(plane_ref, lookup)?.plane)
            .ok_or_else(|| GeopError::new(format!("{plane_ref} is not planar")))?;
        Ok(vec![(
            "image".to_string(),
            Motion::mirror(plane.origin(), plane.w())?,
        )])
    }
}

part_operation!(Mirror);

impl PartOperation for Mirror {
    const INFO: OperationInfo = OperationInfo::new(
        "mirror",
        "Mirror",
        "Mirror bodies in a plane.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .icon(geop_ops::ui::icons::MIRROR);
    type Args = MirrorArgs;
    type Session = ();

    /// Mirrored in the origin's `yz` plane, kept as a new body.
    fn new_args(_: &StepContext) -> MirrorArgs {
        MirrorArgs {
            // Nothing picked: what is patterned is the user's choice, a body
            // or a feature, never guessed.
            seeds: Seeds::default(),
            plane: Some(origin_plane(FrameAxis::X)),
        }
    }

    fn run(step: &str, part: &mut Build, args: &MirrorArgs) -> GeopResult<()> {
        run_pattern::<Self>(part, step, args)
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &MirrorArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }

    /// The bodies or the features, and the plane, picked, and how to
    /// combine an image of bodies.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &MirrorArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, MirrorArgs> {
        let mut f = EditForm::default();
        let set_seeds = |args: &mut MirrorArgs, seeds| args.seeds = seeds;
        let before = Seen::new(context.before);
        (args.seeds).fields(&mut f, before.model(), set_seeds, |f| {
            let plane = args.plane.iter().cloned().collect();
            f.reference(
                "plane",
                "plane",
                plane,
                &[Role::Plane],
                None,
                false,
                |edit, picked| edit.args.plane = picked.first().cloned(),
            );
        });
        f
    }
}
