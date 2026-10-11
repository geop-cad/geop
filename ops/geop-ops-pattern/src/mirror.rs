//! [`Mirror`]: the mirror image of bodies in a plane.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{FrameAxis, Motion},
    scalars::Scalar,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::operation::Input;
use geop_ops::{
    Design, NamedModel, ParamValue,
    operation::{Aspects, Build, BuildReader, EntityRef, Lookup, ModelStep, Role, Seen},
    origin_plane,
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

impl ModelStep for Mirror {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &MirrorArgs,
    ) -> GeopResult<()> {
        build_pattern(model, reader, step, args)
    }
}

impl Operation for Mirror {
    const INFO: OperationInfo = OperationInfo::new(
        "mirror",
        "Mirror",
        "Mirror bodies in a plane.",
        OperationGroup::Bodies,
        OperationTier::Small,
    )
    .icon(geop_ops::ui::icons::MIRROR);
    type Args = MirrorArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// Mirrored in the origin's `yz` plane, kept as a new body.
    fn new_args(_: &Context<Input, ParamValue>) -> MirrorArgs {
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

    /// The bodies or the features, and the plane, picked, and how to
    /// combine an image of bodies.
    fn presentation(_: &Context<Input, ParamValue>, args: &MirrorArgs, _: &()) -> Form<Design> {
        let mut form = Form::default();
        args.seeds.show(&mut form, |form| {
            let plane = args.plane.iter().cloned().collect();
            (form.dialog).reference("plane", "plane", plane, &[Role::Plane], None, false);
        });
        form
    }

    fn process_user_event(
        context: &Context<Input, ParamValue>,
        args: &MirrorArgs,
        _: &(),
        event: Event<Design>,
    ) -> (MirrorArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), &value) {
            ("plane", Value::Entities(picked)) => args.plane = picked.first().cloned(),
            (key, value) => {
                args.seeds
                    .set(key, value, Seen::new(context.before).model());
            }
        }
        (args, ())
    }
}
