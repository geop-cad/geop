//! [`Shell`]: hollow a solid out to walls of one thickness, open where
//! faces are picked.

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
    /// How thick the walls are, measured inward from the solid's faces.
    pub thickness: f64,
}

/// The faces a reference field holds, by name.
fn face_refs(names: &[String]) -> Vec<EntityRef> {
    names
        .iter()
        .map(|name| EntityRef::Face { name: name.clone() })
        .collect()
}

impl Operation for Shell {
    type Args = ShellArgs;
    type Session = ();

    /// The newest solid, closed all round, a tenth thick.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> ShellArgs {
        ShellArgs {
            solid: before.solid_names().pop().unwrap_or_default(),
            faces: Vec::new(),
            thickness: 0.1,
        }
    }

    /// The solid and the faces to open it at, picked, and the thickness.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &ShellArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, ShellArgs> {
        let mut f = Form::<S, ShellArgs>::new();
        let solid = (!args.solid.is_empty()).then(|| EntityRef::Solid {
            name: args.solid.clone(),
        });
        f.reference(
            "solid",
            "solid",
            solid.iter().cloned().collect(),
            &[Role::Solid],
            None,
            false,
            |e, picked| {
                if let [EntityRef::Solid { name }] = picked.as_slice() {
                    if *name != e.args.solid {
                        e.args.faces.clear();
                    }
                    e.args.solid = name.clone();
                } else {
                    e.args.solid.clear();
                    e.args.faces.clear();
                }
            },
        );
        // A shell offsets planes and round faces: those are what can be
        // picked to open.
        f.reference(
            "faces",
            "open faces",
            face_refs(&args.faces),
            &[Role::Plane, Role::Round],
            solid,
            true,
            |e, picked| {
                e.args.faces = picked
                    .into_iter()
                    .filter_map(|entity| match entity {
                        EntityRef::Face { name } => Some(name),
                        _ => None,
                    })
                    .collect();
            },
        );
        f.optional("faces");
        f.number(
            "thickness",
            Number::new("thickness", args.thickness, Unit::Length).range(0.0, 1.0),
            |args, t| args.thickness = t,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &ShellArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("shell({operation_id}, {args:?})");
        let namer = Namer::new("shell", operation_id)?;
        let solid = part.solid_id(&args.solid).with_context(ctx)?;
        let faces = args
            .faces
            .iter()
            .map(|name| part.face_id(name))
            .collect::<GeopResult<Vec<_>>>()
            .with_context(ctx)?;
        if !args.thickness.is_finite() {
            return Err(GeopError::new("the thickness is not a number")).with_context(ctx);
        }
        shell(
            &mut part,
            &namer,
            solid,
            &faces,
            S::from_f64(args.thickness),
        )
        .with_context(ctx)?;
        Ok(part)
    }
}
