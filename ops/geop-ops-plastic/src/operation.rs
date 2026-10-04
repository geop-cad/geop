//! The plastic features as operations of a program: [`Draft`].

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::{Form, Number, Unit},
};
use serde::{Deserialize, Serialize};

use crate::draft::draft;

/// Faces by name, as a reference field holds them.
fn face_refs(names: &[String]) -> Vec<EntityRef> {
    names
        .iter()
        .map(|name| EntityRef::Face { name: name.clone() })
        .collect()
}

/// The names of the faces a reference field holds.
fn face_names(picked: &[EntityRef]) -> Vec<String> {
    picked
        .iter()
        .filter_map(|e| match e {
            EntityRef::Face { name } => Some(name.clone()),
            _ => None,
        })
        .collect()
}

/// Tilts the planar faces `faces` of a solid by `angle` degrees about the
/// plane `neutral`, so the part comes out of a mould pulled along the
/// neutral plane's normal — into the solid, for a face of it picked as the
/// neutral plane — or the other way if `reversed`: past the neutral plane,
/// that way, the solid gets narrower. The solid is consumed and the result
/// named `draft(D)` for the operation `D`; every face, edge and vertex keeps
/// its name (see [`crate::draft::draft`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Draft;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DraftArgs {
    /// The faces to tilt, all planar, all of one solid.
    pub faces: Vec<String>,
    /// The plane they tilt about: a planar face, a datum plane.
    pub neutral: Option<EntityRef>,
    /// How far they tilt, in degrees.
    pub angle: f64,
    /// Pull the other way.
    #[serde(default)]
    pub reversed: bool,
}

impl Operation for Draft {
    type Args = DraftArgs;
    type Session = ();

    /// No faces yet, no neutral plane, and three degrees.
    fn new_args<S: Scalar>(&self, _: &Part<S>) -> DraftArgs {
        DraftArgs {
            faces: Vec::new(),
            neutral: None,
            angle: 3.0,
            reversed: false,
        }
    }

    /// The faces and the neutral plane, picked, the angle and the pull
    /// direction.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &DraftArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, DraftArgs> {
        let mut f = Form::<S, DraftArgs>::new();
        f.reference(
            "faces",
            "faces",
            face_refs(&args.faces),
            &[Role::Plane],
            None,
            true,
            |e, picked| e.args.faces = face_names(&picked),
        );
        f.reference(
            "neutral",
            "neutral plane",
            args.neutral.iter().cloned().collect(),
            &[Role::Plane],
            None,
            false,
            |e, picked| e.args.neutral = picked.into_iter().next(),
        );
        f.number(
            "angle",
            Number::new("angle", args.angle, Unit::Angle).range(-45.0, 45.0),
            |args, angle| args.angle = angle,
        );
        f.checkbox(
            "reversed",
            "reverse pull direction",
            args.reversed,
            |args, b| args.reversed = b,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &DraftArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("draft({operation_id}, {args:?})");
        let namer = Namer::new("draft", operation_id)?;
        let faces = args
            .faces
            .iter()
            .map(|name| part.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        let Some(neutral) = &args.neutral else {
            return Err(GeopError::new("pick the neutral plane")).with_context(ctx);
        };
        let frame = neutral.resolve_plane(&part).with_context(ctx)?;
        // A face's frame points out of its solid; the pull goes into it.
        let into_solid = matches!(neutral, EntityRef::Face { .. });
        let pull = if into_solid != args.reversed {
            frame.w().neg()
        } else {
            *frame.w()
        };
        let plane = Plane::try_new(*frame.origin(), pull).with_context(ctx)?;
        if !args.angle.is_finite() {
            return Err(GeopError::new("the angle is not a number")).with_context(ctx);
        }
        let angle = S::from_f64(args.angle.to_radians());
        draft(&mut part, &namer, &faces, &plane, angle).with_context(ctx)?;
        Ok(part)
    }
}
