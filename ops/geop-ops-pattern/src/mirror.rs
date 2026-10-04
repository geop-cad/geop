//! [`Mirror`]: a mirror image of bodies in a plane.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{DatumComponent, FrameAxis, Motion},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, ORIGIN, Part,
    operation::{Aspects, EntityRef, Operation, Role},
    ui::Form,
};
use geop_ops_booleans::Combine;
use serde::{Deserialize, Serialize};

use crate::common::{
    bodies_field, combine_instances, copy_seeds, newest_solid, seed_instances, seeds,
};

/// Mirrors the bodies `bodies` — solids, and sheets by one of their faces —
/// in a plane: a planar face, a datum plane, a frame's plane. The mirror
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
    /// The bodies to mirror: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
    /// The plane to mirror in; none, for a step that has not picked it.
    pub plane: Option<EntityRef>,
    /// Keep the image as new bodies, or combine it with a solid.
    #[serde(default)]
    pub combine: Combine,
}

impl Operation for Mirror {
    type Args = MirrorArgs;
    type Session = ();

    /// The newest solid, mirrored in the origin's `yz` plane, kept as a
    /// new body.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> MirrorArgs {
        MirrorArgs {
            bodies: newest_solid(before),
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            combine: Combine::NewBody,
        }
    }

    /// The bodies and the plane, picked, and how to combine.
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &MirrorArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, MirrorArgs> {
        let mut f = Form::<S, MirrorArgs>::new();
        bodies_field(&mut f, &args.bodies, |args| &mut args.bodies);
        f.reference(
            "plane",
            "plane",
            args.plane.iter().cloned().collect(),
            &[Role::Plane],
            None,
            false,
            |edit, picked| edit.args.plane = picked.into_iter().next(),
        );
        args.combine
            .show(&mut f, context.before, |args| &mut args.combine);
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &MirrorArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("mirror({operation_id}, {args:?})");
        let namer = Namer::new("mirror", operation_id)?;
        let seeds = seeds(&part, &args.bodies).with_context(ctx)?;
        let Some(plane_ref) = &args.plane else {
            return Err(GeopError::new("pick a plane to mirror in")).with_context(ctx);
        };
        let plane = Aspects::of(plane_ref, &part)
            .with_context(ctx)?
            .plane
            .ok_or_else(|| GeopError::new(format!("{plane_ref} is not planar")))
            .with_context(ctx)?;
        let motion = Motion::mirror(plane.origin(), plane.w()).with_context(ctx)?;
        let mut instances = seed_instances(&seeds, "seed");
        instances.extend(
            copy_seeds(&mut part, &seeds, &motion, "image", |name| {
                namer.name(&[name])
            })
            .with_context(ctx)?,
        );
        combine_instances(&mut part, &namer, operation_id, &args.combine, &instances)
            .with_context(ctx)?;
        Ok(part)
    }
}
