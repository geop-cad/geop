//! [`OffsetSurface`]: faces copied a distance along their normals — the
//! inner side of a shell (see [`geop_ops_shell::shell::offset_faces`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Number, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{Build, BuildReader, EntityRef, Input, LegacyModelStep, Role, legacy_build_model},
};
use geop_ops_shell::shell::offset_faces;
use serde::{Deserialize, Serialize};

/// Copies the faces named `faces` — of a solid or standing on their own —
/// `distance` along their normals, against them for a negative distance,
/// into a sheet of their own, for the operation `O`: where faces meet,
/// their copies meet where the offsets of their surfaces do. The faces stay
/// as they are; the copy of each face, edge and vertex `X` is `offset(O,X)`.
///
/// What can be offset exactly — planes, and surfaces of revolution whose
/// profile is straight or circular — is all that is offset: other faces,
/// and edges that are neither straight nor circular, are refused.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OffsetSurface;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OffsetSurfaceArgs {
    /// The faces to copy.
    pub faces: Vec<String>,
    /// How far along their normals.
    pub distance: f64,
}

impl LegacyModelStep for OffsetSurface {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &OffsetSurfaceArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("offset_surface({step}, {args:?})");
        let namer = Namer::new("offset", step)?;
        if !args.distance.is_finite() {
            return Err(GeopError::new("the distance is not a number")).with_context(ctx);
        }
        if args.faces.is_empty() {
            return Err(GeopError::new("no faces to offset")).with_context(ctx);
        }
        let faces = (args.faces.iter())
            .map(|name| model.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        offset_faces(model, &namer, &faces, Design::from_f64(args.distance)).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for OffsetSurface {
    const INFO: OperationInfo = OperationInfo::new(
        "offset_surface",
        "Offset surface",
        "Copy faces a distance along their normals into a face standing on its own.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::OFFSET_SURFACE);
    type Args = OffsetSurfaceArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// Nothing picked yet, a tenth out.
    fn new_args(_: &Context<Input, ParamValue>) -> OffsetSurfaceArgs {
        OffsetSurfaceArgs {
            faces: Vec::new(),
            distance: 0.1,
        }
    }

    fn run(step: &str, part: &mut Build, args: &OffsetSurfaceArgs) -> GeopResult<()> {
        legacy_build_model::<Self>(part, step, args)
    }

    /// The faces, picked, and the distance.
    fn presentation(
        _: &Context<Input, ParamValue>,
        args: &OffsetSurfaceArgs,
        _: &(),
    ) -> Form<Design> {
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let faces = EntityRef::of_names(EntityRef::face, &args.faces);
        dialog.reference("faces", "faces", faces, &[Role::Face], None, true);
        let distance = Number::new("distance", args.distance, Unit::Length).range(-1.0, 1.0);
        dialog.number("distance", distance);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &OffsetSurfaceArgs,
        _: &(),
        event: Event<Design>,
    ) -> (OffsetSurfaceArgs, ()) {
        let mut args = args.clone();
        if let Event::Field { key, value } = event {
            match (key.as_str(), value) {
                ("faces", Value::Entities(picked)) => {
                    args.faces = EntityRef::names_of(EntityRef::face, &picked)
                }
                ("distance", Value::Number(d)) => args.distance = d,
                _ => {}
            }
        }
        (args, ())
    }
}

#[cfg(test)]
mod tests;
