//! [`Shell`]: hollow a solid out to walls of one thickness, open where
//! faces are picked.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{BuildReader, EntityRef, Lookup, PartOperation, Role, Scope, Seen, StepContext},
    parameters::{Formula, expressions},
    part_operation,
    ui::{EditForm, FormState, Unit, formula_number},
};
use serde::{Deserialize, Serialize};

use crate::shell::shell;

/// Hollows the solid named `solid` out to walls `thickness` thick, open
/// where its faces `faces` were — closed all round, with a void inside, if
/// none is picked. The result replaces the solid and is named `shell(S)`
/// for the operation `S`; what it is made of is named as
/// [`crate::shell::shell`] says.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Shell;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShellArgs {
    /// The solid to hollow.
    pub solid: String,
    /// The faces of it to take away, leaving openings.
    #[serde(default)]
    pub faces: Vec<String>,
    /// How thick the walls are, measured inward from the solid's faces: a
    /// number, or a formula of the part's parameters.
    pub thickness: Formula,
}

part_operation!(Shell);

impl PartOperation for Shell {
    const INFO: OperationInfo = OperationInfo::new(
        "shell",
        "Shell",
        "Hollow a solid out to walls of one thickness, open where faces are picked.",
        OperationGroup::Features,
        OperationTier::Small,
    )
    .icon(crate::icons::SHELL);
    type Args = ShellArgs;
    type Session = ();

    fn formulas(args: &mut ShellArgs) -> Vec<&mut String> {
        expressions([&mut args.thickness])
    }

    /// The newest solid, closed all round, a tenth thick.
    fn new_args(context: &StepContext) -> ShellArgs {
        ShellArgs {
            solid: Seen::new(context.before)
                .model()
                .newest_solid()
                .unwrap_or_default(),
            faces: Vec::new(),
            thickness: Formula::Plain(0.1),
        }
    }

    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &ShellArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("shell", step)?;
        let thickness = args.thickness.evaluate(&mut Scope { model, reader })?;
        if !thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number"));
        }
        let solid = model.solid_id(&args.solid)?;
        let faces = (args.faces.iter())
            .map(|name| model.face_id(name))
            .collect::<GeopResult<Vec<_>>>()?;
        shell(model, &namer, solid, &faces, Design::from_f64(thickness))?;
        Ok(())
    }

    /// The solid and the faces to open it at, picked, and the thickness.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &ShellArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ShellArgs> {
        let mut seen = Seen::new(context.before);
        let mut f = EditForm::<Design, ShellArgs>::default();
        f.pick_one(
            "solid",
            "solid",
            EntityRef::solid,
            &args.solid,
            &[Role::Solid],
            |edit, solid| {
                // The faces opened were of the solid before.
                if solid != edit.args.solid {
                    edit.args.faces.clear();
                }
                edit.args.solid = solid;
            },
        );
        // A shell offsets planes and round faces: those are what can be
        // picked to open.
        let faces = EntityRef::of_names(EntityRef::face, &args.faces);
        let roles = [Role::Plane, Role::Round];
        let scope = EntityRef::of_name(EntityRef::solid, &args.solid).pop();
        f.reference(
            "faces",
            "open faces",
            faces,
            &roles,
            scope,
            true,
            |edit, picked| {
                edit.args.faces = EntityRef::names_of(EntityRef::face, &picked);
            },
        );
        f.form.dialog.optional("faces");
        let thickness = formula_number("thickness", &args.thickness, &mut seen, Unit::Length);
        f.formula("thickness", thickness.range(0.0, 1.0), |edit, thickness| {
            edit.args.thickness = thickness
        });
        f
    }
}
