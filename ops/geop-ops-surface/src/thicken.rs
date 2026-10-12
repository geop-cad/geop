//! [`Thicken`]: a sheet made a solid of one thickness — the sheet shelled
//! (see [`geop_ops_shell::shell::thicken`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Field, Scalar},
};
use geop_core_topology::Body;
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{BuildReader, EntityRef, Lookup, PartOperation, Role, Seen, StepContext},
    part_operation,
    ui::{Choice, EditForm, FormState, Number, Unit},
};
use geop_ops_shell::shell::{offset_faces, thicken};
use serde::{Deserialize, Serialize};

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

part_operation!(Thicken);

impl PartOperation for Thicken {
    const INFO: OperationInfo = OperationInfo::new(
        "thicken",
        "Thicken",
        "Make a solid of a face standing on its own, a thickness on either side of it or on both.",
        OperationGroup::Surface,
        OperationTier::Menu,
    )
    .experimental()
    .icon(crate::icons::THICKEN);
    type Args = ThickenArgs;
    type Session = ();

    /// The newest sheet, a tenth thick, behind its normals.
    fn new_args(context: &StepContext) -> ThickenArgs {
        let seen = Seen::new(context.before);
        ThickenArgs {
            face: seen.model().sheet_face_names().pop().unwrap_or_default(),
            thickness: 0.1,
            side: ThickenSide::Against,
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        _: &mut BuildReader,
        step: &str,
        args: &ThickenArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("thicken", step)?;
        if !args.thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number"));
        }
        let sheet = model.sheet_of_face(model.face_id(&args.face)?, "a sheet is thickened")?;
        let thickness = Design::from_f64(args.thickness);
        match args.side {
            ThickenSide::Against | ThickenSide::Along => {
                let along = args.side == ThickenSide::Along;
                thicken(model, &namer, sheet, thickness, along)?;
            }
            ThickenSide::Both => {
                let faces = model.topology().body_faces(Body::Sheet(sheet))?;
                let half = thickness.div(Design::TWO)?;
                let copy = offset_faces(model, &namer.scoped("both"), &faces, half)?;
                model.delete_bodies(&[Body::Sheet(sheet)])?;
                thicken(model, &namer, copy.shells[0], thickness, false)?;
            }
        }
        Ok(())
    }

    /// The sheet, picked, the thickness and the side.
    fn form<'a>(
        _: &'a StepContext<'a>,
        args: &ThickenArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ThickenArgs> {
        let mut f = EditForm::<Design, ThickenArgs>::default();
        f.pick_one(
            "face",
            "sheet",
            EntityRef::face,
            &args.face,
            &[Role::Sheet],
            |edit, name| edit.args.face = name,
        );
        let thickness = Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0);
        f.number("thickness", thickness, |edit, t| edit.args.thickness = t);
        let sides = (ThickenSide::ALL.iter())
            .map(|s| Choice::new(s.key(), s.label()))
            .collect();
        f.select(
            "side",
            "side",
            args.side.key(),
            sides,
            false,
            |edit, key| {
                if let Some(side) = ThickenSide::ALL.into_iter().find(|s| s.key() == key) {
                    edit.args.side = side;
                }
            },
        );
        f
    }
}

#[cfg(test)]
pub(crate) mod tests;
