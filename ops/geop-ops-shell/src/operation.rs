//! [`Shell`]: hollow a solid out to walls of one thickness, open where
//! faces are picked.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    editing::{Event, Form, Unit, Value},
    operation::{Context, Operation, OperationGroup, OperationInfo, OperationTier},
};
use geop_ops::{
    Design, NamedModel, Namer, ParamValue,
    operation::{
        Build, BuildReader, EntityRef, Input, Lookup, ModelStep, Role, Scope, Seen, build_model,
    },
    parameters::{Formula, expressions},
    ui::formula_number,
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

impl ModelStep for Shell {
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &ShellArgs,
    ) -> GeopResult<()> {
        let ctx = with_context!("shell({step}, {args:?})");
        let namer = Namer::new("shell", step)?;
        let thickness =
            (args.thickness.evaluate(&mut Scope { model, reader })).with_context(ctx)?;
        if !thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number")).with_context(ctx);
        }
        let solid = model.solid_id(&args.solid).with_context(ctx)?;
        let faces = (args.faces.iter())
            .map(|name| model.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        shell(model, &namer, solid, &faces, Design::from_f64(thickness)).with_context(ctx)?;
        Ok(())
    }
}

impl Operation for Shell {
    const INFO: OperationInfo = OperationInfo::new(
        "shell",
        "Shell",
        "Hollow a solid out to walls of one thickness, open where faces are picked.",
        OperationGroup::Features,
        OperationTier::Small,
    );
    type Args = ShellArgs;
    type EditingState = ();
    type Input = Input;
    type State = ParamValue;

    fn formulas(args: &mut ShellArgs) -> Vec<&mut String> {
        expressions([&mut args.thickness])
    }

    /// The newest solid, closed all round, a tenth thick.
    fn new_args(context: &Context<Input, ParamValue>) -> ShellArgs {
        ShellArgs {
            solid: (Seen::new(context.before).model().solid_names().pop()).unwrap_or_default(),
            faces: Vec::new(),
            thickness: Formula::Plain(0.1),
        }
    }

    fn run(step: &str, part: &mut Build, args: &ShellArgs) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// The solid and the faces to open it at, picked, and the thickness.
    fn presentation(
        context: &Context<Input, ParamValue>,
        args: &ShellArgs,
        _: &(),
    ) -> Form<Design> {
        let mut seen = Seen::new(context.before);
        let mut form = Form::default();
        let dialog = &mut form.dialog;
        let solid = (!args.solid.is_empty()).then(|| EntityRef::solid(args.solid.clone()));
        let picked = solid.iter().cloned().collect();
        dialog.reference("solid", "solid", picked, &[Role::Solid], None, false);
        // A shell offsets planes and round faces: those are what can be
        // picked to open.
        let faces = EntityRef::of_names(EntityRef::face, &args.faces);
        let roles = [Role::Plane, Role::Round];
        (dialog.reference("faces", "open faces", faces, &roles, solid, true)).optional("faces");
        let thickness = formula_number("thickness", &args.thickness, &mut seen, Unit::Length);
        dialog.number("thickness", thickness.range(0.0, 1.0));
        form
    }

    fn process_user_event(
        _: &Context<Input, ParamValue>,
        args: &ShellArgs,
        _: &(),
        event: Event<Design>,
    ) -> (ShellArgs, ()) {
        let mut args = args.clone();
        let Event::Field { key, value } = event else {
            return (args, ());
        };
        match (key.as_str(), value) {
            ("solid", Value::Entities(picked)) => {
                let solid = EntityRef::names_of(EntityRef::solid, &picked).pop();
                // The faces opened were of the solid before.
                if solid.as_ref() != Some(&args.solid) {
                    args.faces.clear();
                }
                args.solid = solid.unwrap_or_default();
            }
            ("faces", Value::Entities(picked)) => {
                args.faces = EntityRef::names_of(EntityRef::face, &picked);
            }
            ("thickness", value) => {
                if let Some(thickness) = Formula::from_value(&value) {
                    args.thickness = thickness;
                }
            }
            _ => {}
        }
        (args, ())
    }
}
