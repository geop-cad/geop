//! [`ExtractFace`]: copy a face out of its body, into a face standing on
//! its own.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::element::ElementReference;
use geop_ops_model::{Design, Face, ModelOperation, NamedModel, Namer, build_model, model};
use geop_ops_parameters::{Setting, BuildPart, Runner, PartReader, StateValue, StepContext};
use serde::{Deserialize, Serialize};

/// Copies the face named `face` — of a solid, or of a sheet — into a sheet
/// of its own: a face standing on its own, on the same surface and within
/// the same boundary, sharing nothing with the original, which stays as it
/// is. A solid can be split with it.
///
/// The copy of the face, and of each of its edges and vertices `X`, is
/// named `extract(E,X)` for the operation `E`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExtractFace;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExtractFaceArgs {
    /// The face to copy.
    pub face: ElementReference<NamedModel<Design>, Face>,
}

part_operation!(ExtractFace(ExtractFaceArgs));

impl PartOperation for ExtractFace {
    const KIND: &'static str = "extract_face";
    const INFO: OperationInfo = OperationInfo::new(
        "Extract face",
        "Copy a face out of its body into a face standing on its own, which a solid can be split with.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::EXTRACT_FACE);
    type Args = ExtractFaceArgs;
    type Input = Setting;
    type State = StateValue;
    type Scalar = Design;
    type Session = ();

    /// Nothing picked yet: no face has the empty name.
    fn new_args(_: &StepContext) -> ExtractFaceArgs {
        ExtractFaceArgs {
            face: ElementReference::new(model(), ""),
        }
    }

    fn run(step: &str, _: &Runner, part: &mut BuildPart, args: &ExtractFaceArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The face, picked.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &ExtractFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ExtractFaceArgs> {
        let mut f = EditForm::<Design, ExtractFaceArgs>::default();
        f.pick_one(
            "face",
            "face",
            &context.before,
            Some(&args.face),
            |edit, face| edit.args.face = face,
        );
        f
    }
}

impl ModelOperation for ExtractFace {
    fn build(
        model: &mut NamedModel<Design>,
        _: &PartReader,
        step: &str,
        args: &ExtractFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("extract", step)?;
        let face = model.face_id(args.face.name())?;
        model.copy_faces(&[face], None, |name| namer.name(&[name]))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_ops_extrude_revolve::shapes::block;
    use geop_ops_model::testing::{apply_to_model, assert_valid_sheets};

    /// A face of a cube, extracted, stands on its own beside the cube,
    /// which is left as it was.
    #[test]
    fn extracts_a_face_of_a_cube() {
        let mut part = NamedModel::<Design>::new();
        let cube = block(&mut part, "c", [0.0; 3], [1.0; 3]);
        let face = part.topology().solid_faces(cube).unwrap()[0];
        let face = part.name_of(face).unwrap().to_string();
        let args = ExtractFaceArgs {
            face: ElementReference::new(model(), &face),
        };
        let part = apply_to_model::<ExtractFace>(part, "x", &args).unwrap();
        assert_valid_sheets(&part);
        assert_eq!(part.sheet_face_names(), [format!("extract(x,{face})")]);
        assert_eq!(part.topology().solid_faces(cube).unwrap().len(), 6);
        assert_eq!(part.topology().faces.len(), 7);
        assert_eq!(part.topology().vertices.len(), 12);
    }
}
