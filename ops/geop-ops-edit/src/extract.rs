//! [`ExtractFace`]: copy a face out of its body, into a face standing on
//! its own.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::{
    Design, EntityRef, NamedModel, Namer, ParamValue,
    operation::{Build, BuildReader, Input, ModelStep, Role, build_model},
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

impl ModelStep for ExtractFace {
    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &ExtractFaceArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("extract_face({step}, {args:?})");
        let namer = Namer::new("extract", step)?;
        let face = model.face_id(&args.face).with_context(ctx)?;
        (model.copy_faces(&[face], None, |name| namer.name(&[name]))).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for ExtractFace {
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
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    /// Nothing picked yet.
    fn new_args(_: &Context<Input, ParamValue>) -> ExtractFaceArgs {
        ExtractFaceArgs::default()
    }

    fn run(step: &str, part: &mut Build, args: &ExtractFaceArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The face, picked.
    fn presentation(
        _: &Context<Input, ParamValue>,
        args: &ExtractFaceArgs,
        _: &(),
    ) -> Form<Design> {
        let mut form = Form::default();
        let face = EntityRef::of_name(EntityRef::face, &args.face);
        (form.dialog).reference("face", "face", face, &[Role::Face], None, false);
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &ExtractFaceArgs,
        _: &(),
        event: Event<Design>,
    ) -> (ExtractFaceArgs, ()) {
        let mut args = args.clone();
        if let Event::Field {
            key,
            value: Value::Entities(picked),
        } = event
            && key == "face"
        {
            args.face = EntityRef::last_name(EntityRef::face, &picked).unwrap_or_default();
        }
        (args, ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{
        scalars::{ScalInF64 as S, Scalar},
        vector::Vector3,
    };
    use geop_core_topology::validation::{ValidationParameters, validate};
    use geop_ops::operation::apply_to_model;
    use geop_ops_extrude_revolve::shapes::cube_solid;

    fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A face of a cube, extracted, stands on its own beside the cube,
    /// which is left as it was.
    #[test]
    fn extracts_a_face_of_a_cube() {
        let mut part = NamedModel::<S>::new();
        let cube = cube_solid(&mut part, "c", v3(0.0, 0.0, 0.0), v3(1.0, 1.0, 1.0)).unwrap();
        let face = part.topology().solid_faces(cube).unwrap()[0];
        let face = part.name_of(face).unwrap().to_string();
        let args = ExtractFaceArgs { face: face.clone() };
        let part = apply_to_model::<ExtractFace>(part, "x", &args).unwrap();
        validate(&ValidationParameters::default(), part.topology()).unwrap();
        part.check_names().unwrap();
        assert_eq!(part.sheet_face_names(), [format!("extract(x,{face})")]);
        assert_eq!(part.topology().solid_faces(cube).unwrap().len(), 6);
        assert_eq!(part.topology().faces.len(), 7);
        assert_eq!(part.topology().vertices.len(), 12);
    }
}
