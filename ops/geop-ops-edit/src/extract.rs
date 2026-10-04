//! [`ExtractFace`]: copy a face out of its body, into a face standing on
//! its own.

use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Operation, Role},
    ui::Form,
};
use serde::{Deserialize, Serialize};

use crate::{face_name, face_ref};

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

impl Operation for ExtractFace {
    type Args = ExtractFaceArgs;
    type Session = ();

    /// Nothing picked yet.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> ExtractFaceArgs {
        ExtractFaceArgs::default()
    }

    /// The face, picked.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ExtractFaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ExtractFaceArgs> {
        let mut f = Form::<S, ExtractFaceArgs>::new();
        f.reference(
            "face",
            "face",
            face_ref(&args.face),
            &[Role::Face],
            None,
            false,
            |e, p| e.args.face = face_name(&p),
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ExtractFaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("extract_face({operation_id}, {args:?})");
        let namer = Namer::new("extract", operation_id)?;
        let face = part.face_id(&args.face).with_context(ctx)?;
        part.copy_faces(&[face], None, |name| namer.name(&[name]))
            .with_context(ctx)?;
        Ok(part)
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
    use geop_ops::NoFiles;
    use geop_ops_extrude_revolve::shapes::cube_solid;

    fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A face of a cube, extracted, stands on its own beside the cube,
    /// which is left as it was.
    #[test]
    fn extracts_a_face_of_a_cube() {
        let mut part = Part::<S>::new();
        let cube = cube_solid(&mut part, "c", v3(0.0, 0.0, 0.0), v3(1.0, 1.0, 1.0)).unwrap();
        let face = part.topology().solid_faces(cube).unwrap()[0];
        let face = part.name_of(face).unwrap().to_string();
        let args = ExtractFaceArgs { face: face.clone() };
        let part = ExtractFace.apply(part, "x", &args, &NoFiles).unwrap();
        validate(&ValidationParameters::default(), part.topology()).unwrap();
        part.check_names().unwrap();
        assert_eq!(part.sheet_face_names(), [format!("extract(x,{face})")]);
        assert_eq!(part.topology().solid_faces(cube).unwrap().len(), 6);
        assert_eq!(part.topology().faces.len(), 7);
        assert_eq!(part.topology().vertices.len(), 12);
    }
}
