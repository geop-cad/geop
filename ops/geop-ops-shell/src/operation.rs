//! [`Shell`]: hollow a solid out to walls of one thickness — another for
//! the faces picked for it — open where faces are picked.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::{Number, Unit},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::{
    element::ElementReference, numeric::NumericInput, part::StepContext, reader::Reader,
};
use geop_lib_shell::shell;
use geop_ops_model::{
    Face, ModelOperation, NamedModel, Namer, Solid, build_model, model, read_model,
};
use serde::{Deserialize, Serialize};

/// Hollows the solid named `solid` out to walls `thickness` thick, open
/// where its faces `faces` were — closed all round, with a void inside, if
/// none is picked. The result replaces the solid and is named `shell(Scalar)`
/// for the operation `Scalar`; what it is made of is named as
/// [`geop_lib_shell::shell()`] says.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Shell;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ShellArgs {
    /// The solid to hollow.
    pub solid: ElementReference<NamedModel, Solid>,
    /// The faces of it to take away, leaving openings.
    #[serde(default)]
    pub faces: Vec<ElementReference<NamedModel, Face>>,
    /// How thick the walls are, measured inward from the solid's faces: a
    /// number, or a formula of the part's parameters.
    pub thickness: NumericInput,
    /// Faces whose walls are `other_thickness` thick instead.
    #[serde(default)]
    pub other_faces: Vec<ElementReference<NamedModel, Face>>,
    /// How thick the walls of `other_faces` are.
    #[serde(default = "default_other_thickness")]
    pub other_thickness: NumericInput,
}

fn default_other_thickness() -> NumericInput {
    NumericInput::Plain(0.2)
}

part_operation!(Shell(ShellArgs));

impl PartOperation for Shell {
    const KIND: &'static str = "shell";
    const INFO: OperationInfo = OperationInfo::new(
        "Shell",
        "Hollow a solid out to walls of one thickness, open where faces are picked.",
        OperationGroup::Features,
        OperationTier::Small,
    )
    .icon(crate::icons::SHELL);
    type Args = ShellArgs;
    type Session = ();

    /// The newest solid, closed all round, a tenth thick.
    fn new_args(context: &Context) -> ShellArgs {
        let newest = read_model(&context.before)
            .ok()
            .and_then(|model| model.newest_solid());
        ShellArgs {
            solid: ElementReference::new(model(), newest.unwrap_or_default()),
            faces: Vec::new(),
            thickness: NumericInput::Plain(0.1),
            other_faces: Vec::new(),
            other_thickness: default_other_thickness(),
        }
    }

    fn run(ctx: &mut StepContext, args: &ShellArgs) -> GeopResult<()> {
        let _step = ctx.step();
        build_model::<Self>(ctx, args)
    }

    /// The solid and the faces to open it at, picked, and the thickness.
    fn form<'a>(
        context: &'a Context<'a>,
        args: &ShellArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, ShellArgs> {
        let before = &context.before;
        let mut f = EditForm::<ShellArgs>::default();
        f.pick_one(
            "solid",
            "solid",
            before,
            Some(&args.solid),
            |edit, solid| {
                // The faces opened were of the solid before.
                if solid != edit.args.solid {
                    edit.args.faces.clear();
                    edit.args.other_faces.clear();
                }
                edit.args.solid = solid;
            },
        );
        f.pick(
            "faces",
            "open faces",
            before,
            &args.faces,
            true,
            |edit, faces| edit.args.faces = faces,
        );
        f.form.dialog.optional("faces");
        let thickness = Number::numeric(&args.thickness, "thickness", before, Unit::Length);
        f.numeric("thickness", thickness.range(0.0, 1.0), |edit, thickness| {
            edit.args.thickness = thickness
        });
        f.pick(
            "other_faces",
            "faces of another thickness",
            before,
            &args.other_faces,
            true,
            |edit, faces| edit.args.other_faces = faces,
        );
        f.form.dialog.optional("other_faces");
        if !args.other_faces.is_empty() {
            let other = Number::numeric(
                &args.other_thickness,
                "their thickness",
                before,
                Unit::Length,
            );
            f.numeric(
                "other_thickness",
                other.range(0.0, 1.0),
                |edit, thickness| edit.args.other_thickness = thickness,
            );
        }
        f
    }
}

impl ModelOperation for Shell {
    fn build(
        model: &mut NamedModel,
        reader: &Reader,
        step: &str,
        args: &ShellArgs,
    ) -> GeopResult<()> {
        let namer = Namer::new("shell", step)?;
        let thickness = args.thickness.resolve(reader)?;
        if !thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number"));
        }
        let solid = model.solid_id(args.solid.name())?;
        let faces = (args.faces.iter())
            .map(|face| model.face_id(face.name()))
            .collect::<GeopResult<Vec<_>>>()?;
        let other = args.other_thickness.resolve(reader)?;
        if !other.is_finite() {
            return Err(GeopError::new("the other thickness is not a number"));
        }
        let thicker = (args.other_faces.iter())
            .map(|face| Ok((model.face_id(face.name())?, Scalar::from_f64(other))))
            .collect::<GeopResult<Vec<_>>>()?;
        shell(
            model,
            &namer,
            solid,
            &faces,
            Scalar::from_f64(thickness),
            &thicker,
        )?;
        Ok(())
    }
}
