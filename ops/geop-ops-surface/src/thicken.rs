//! [`Thicken`]: a sheet made a solid of one thickness — the sheet shelled
//! (see [`geop_ops_shell::shell::thicken`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::{
    Context, Library, Namer, Part,
    operation::{Operation, Role},
    ui::{Choice, Form, Number, Unit},
};
use geop_ops_shell::shell::{offset_faces, thicken};
use serde::{Deserialize, Serialize};

use crate::{face, picked_names, refs, sheet_of};

/// Which side of a sheet a [`Thicken`] adds its material on, as the sheet's
/// faces' normals point.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThickenSide {
    /// Behind the sheet, against its normals.
    #[default]
    Against,
    /// In front of it, along its normals.
    Along,
    /// Half on either side.
    Both,
}

impl ThickenSide {
    const ALL: [ThickenSide; 3] = [ThickenSide::Against, ThickenSide::Along, ThickenSide::Both];

    fn key(self) -> &'static str {
        match self {
            ThickenSide::Against => "against",
            ThickenSide::Along => "along",
            ThickenSide::Both => "both",
        }
    }

    fn label(self) -> &'static str {
        match self {
            ThickenSide::Against => "behind the normal",
            ThickenSide::Along => "along the normal",
            ThickenSide::Both => "both sides",
        }
    }
}

/// Thickens the sheet the face named `face` stands in into a solid of walls
/// `thickness` thick, on the `side` of it asked for, for the operation `T`:
/// the sheet is consumed, the solid named `thicken(T)`.
///
/// On one side, the sheet's faces, edges and vertices keep their names and
/// bound the solid on that side; the inner copy of each `X` is
/// `thicken(T,X)`, and along a free edge `E` of the sheet the wall is
/// `thicken(T,E,side)`, the straight edge from a vertex `V` of it to its
/// copy `thicken(T,V,side)`. On both sides, the sheet is first copied
/// half the thickness along its normals, each copy `X'` named
/// `thicken(T,both,X)` — and then that is thickened behind its normals,
/// named as above after `X'`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Thicken;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThickenArgs {
    /// A face of the sheet to thicken.
    pub face: String,
    pub thickness: f64,
    #[serde(default)]
    pub side: ThickenSide,
}

impl Operation for Thicken {
    type Args = ThickenArgs;
    type Session = ();

    /// The newest sheet, a tenth thick, behind its normals.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> ThickenArgs {
        ThickenArgs {
            face: before.sheet_face_names().pop().unwrap_or_default(),
            thickness: 0.1,
            side: ThickenSide::Against,
        }
    }

    /// The sheet, picked, the thickness and the side.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ThickenArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ThickenArgs> {
        let mut f = Form::<S, ThickenArgs>::new();
        let picked: Vec<String> = (!args.face.is_empty())
            .then(|| args.face.clone())
            .into_iter()
            .collect();
        f.reference(
            "face",
            "sheet",
            refs(&picked, face),
            &[Role::Sheet],
            None,
            false,
            |e, picked| e.args.face = picked_names(&picked).pop().unwrap_or_default(),
        );
        f.number(
            "thickness",
            Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0),
            |args, t| args.thickness = t,
        );
        f.select(
            "side",
            "side",
            args.side.key(),
            ThickenSide::ALL
                .iter()
                .map(|s| Choice::new(s.key(), s.label()))
                .collect(),
            false,
            |args, key| {
                if let Some(side) = ThickenSide::ALL.into_iter().find(|s| s.key() == key) {
                    args.side = side;
                }
            },
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ThickenArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("thicken({operation_id}, {args:?})");
        let namer = Namer::new("thicken", operation_id)?;
        if !args.thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number")).with_context(ctx);
        }
        let (_, sheet) = sheet_of(&part, &args.face).with_context(ctx)?;
        let thickness = S::from_f64(args.thickness);
        match args.side {
            ThickenSide::Against | ThickenSide::Along => {
                let along = args.side == ThickenSide::Along;
                thicken(&mut part, &namer, sheet, thickness, along).with_context(ctx)?;
            }
            ThickenSide::Both => {
                let faces = part
                    .topology()
                    .body_faces(Body::Sheet(sheet))
                    .with_context(ctx)?;
                let half = thickness.div(S::TWO).with_context(ctx)?;
                let copy = offset_faces(&mut part, &namer.scoped("both"), &faces, half)
                    .with_context(ctx)?;
                part.assemble_sheet(&[Body::Sheet(sheet)], &[])
                    .with_context(ctx)?;
                thicken(&mut part, &namer, copy.shells[0], thickness, false).with_context(ctx)?;
            }
        }
        Ok(part)
    }
}

#[cfg(test)]
pub(crate) mod tests;
