//! [`OffsetSurface`]: faces copied a distance along their normals — the
//! inner side of a shell (see [`geop_ops_shell::shell::offset_faces`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Operation, Role},
    ui::{Form, Number, Unit},
};
use geop_ops_shell::shell::offset_faces;
use serde::{Deserialize, Serialize};

use crate::{face, picked_names, refs};

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

impl Operation for OffsetSurface {
    type Args = OffsetSurfaceArgs;
    type Session = ();

    /// Nothing picked yet, a tenth out.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> OffsetSurfaceArgs {
        OffsetSurfaceArgs {
            faces: Vec::new(),
            distance: 0.1,
        }
    }

    /// The faces, picked, and the distance.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &OffsetSurfaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, OffsetSurfaceArgs> {
        let mut f = Form::<S, OffsetSurfaceArgs>::new();
        f.reference(
            "faces",
            "faces",
            refs(&args.faces, face),
            &[Role::Face],
            None,
            true,
            |e, picked| e.args.faces = picked_names(&picked),
        );
        f.number(
            "distance",
            Number::new("distance", args.distance, Unit::Length).range(-1.0, 1.0),
            |args, d| args.distance = d,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &OffsetSurfaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("offset_surface({operation_id}, {args:?})");
        let namer = Namer::new("offset", operation_id)?;
        if !args.distance.is_finite() {
            return Err(GeopError::new("the distance is not a number")).with_context(ctx);
        }
        if args.faces.is_empty() {
            return Err(GeopError::new("no faces to offset")).with_context(ctx);
        }
        let faces = args
            .faces
            .iter()
            .map(|name| part.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        offset_faces(&mut part, &namer, &faces, S::from_f64(args.distance)).with_context(ctx)?;
        Ok(part)
    }
}

#[cfg(test)]
mod tests;
