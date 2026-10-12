//! [`ExtractFace`]: copy a face out of its body, into a face standing on
//! its own.

use geop_core_math::geop_error::GeopResult;
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, EntityRef, NamedModel, Namer,
    operation::{BuildReader, PartOperation, Role, StepContext},
    part_operation,
    ui::{EditForm, FormState},
};
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

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExtractFaceArgs {
    /// The face to copy.
    pub face: String,
}

part_operation!(ExtractFace(ExtractFaceArgs));

impl PartOperation for ExtractFace {
    const INFO: OperationInfo = OperationInfo::new(
        "extract_face",
        "Extract face",
        "Copy a face out of its body into a face standing on its own, which a solid can be split with.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::EXTRACT_FACE);
    type Args = ExtractFaceArgs;
    type Session = ();

    /// Nothing picked yet.
    fn new_args(_: &StepContext) -> ExtractFaceArgs {
        ExtractFaceArgs::default()
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &ExtractFaceArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("extract", step)?;
        let face = model.face_id(&args.face)?;
        model.copy_faces(&[face], None, |name| namer.name(&[name]))?;
        Ok(())
    }

    /// The face, picked.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &ExtractFaceArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ExtractFaceArgs> {
        let mut f = EditForm::<Design, ExtractFaceArgs>::default();
        f.pick_one(
            "face",
            "face",
            EntityRef::face,
            &args.face,
            &[Role::Face],
            |edit, name| edit.args.face = name,
        );
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::scalars::ScalInF64 as S;
    use geop_ops::{operation::apply_to_model, testing::assert_valid_sheets};
    use geop_ops_extrude_revolve::shapes::block;

    /// A face of a cube, extracted, stands on its own beside the cube,
    /// which is left as it was.
    #[test]
    fn extracts_a_face_of_a_cube() {
        let mut part = NamedModel::<S>::new();
        let cube = block(&mut part, "c", [0.0; 3], [1.0; 3]);
        let face = part.topology().solid_faces(cube).unwrap()[0];
        let face = part.name_of(face).unwrap().to_string();
        let args = ExtractFaceArgs { face: face.clone() };
        let part = apply_to_model::<ExtractFace>(part, "x", &args).unwrap();
        assert_valid_sheets(&part);
        assert_eq!(part.sheet_face_names(), [format!("extract(x,{face})")]);
        assert_eq!(part.topology().solid_faces(cube).unwrap().len(), 6);
        assert_eq!(part.topology().faces.len(), 7);
        assert_eq!(part.topology().vertices.len(), 12);
    }
}
