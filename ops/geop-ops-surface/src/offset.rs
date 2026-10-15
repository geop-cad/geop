//! [`OffsetSurface`]: faces copied a distance along their normals — the
//! inner side of a shell (see [`geop_ops_shell::shell::offset_faces`]).

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::element::ElementReference;
use geop_ops_model::{Design, Face, ModelOperation, NamedModel, Namer, build_model};
use geop_ops_parameters::{Setting, BuildPart, Runner, PartReader, StateValue, StepContext};
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
    pub faces: Vec<ElementReference<NamedModel<Design>, Face>>,
    /// How far along their normals.
    pub distance: f64,
}

part_operation!(OffsetSurface(OffsetSurfaceArgs));

impl PartOperation for OffsetSurface {
    const KIND: &'static str = "offset_surface";
    const INFO: OperationInfo = OperationInfo::new(
        "Offset surface",
        "Copy faces a distance along their normals into a face standing on its own.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::OFFSET_SURFACE);
    type Args = OffsetSurfaceArgs;
    type Input = Setting;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    /// Nothing picked yet, a tenth out.
    fn new_args(_: &StepContext) -> OffsetSurfaceArgs {
        OffsetSurfaceArgs {
            faces: Vec::new(),
            distance: 0.1,
        }
    }

    fn run(
        step: &str,
        _: &Runner,
        part: &mut BuildPart,
        args: &OffsetSurfaceArgs,
    ) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The faces, picked, and the distance.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &OffsetSurfaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, OffsetSurfaceArgs> {
        let mut f = EditForm::<Design, OffsetSurfaceArgs>::default();
        f.pick(
            "faces",
            "faces",
            &context.before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        let distance = Number::new("distance", args.distance, Unit::Length).range(-1.0, 1.0);
        f.number("distance", distance, |edit, d| edit.args.distance = d);
        f
    }
}

impl ModelOperation for OffsetSurface {
    fn build(
        model: &mut NamedModel<Design>,
        _: &PartReader,
        step: &str,
        args: &OffsetSurfaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("offset", step)?;
        if !args.distance.is_finite() {
            return Err(GeopError::new("the distance is not a number"));
        }
        if args.faces.is_empty() {
            return Err(GeopError::new("no faces to offset"));
        }
        let faces = (args.faces.iter())
            .map(|face| model.face_id(face.name()))
            .collect::<GeopResult<Vec<_>>>()?;
        offset_faces(model, &namer, &faces, Design::from_f64(args.distance))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
