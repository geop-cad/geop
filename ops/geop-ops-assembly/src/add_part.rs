//! [`AddPart`]: place the part another program file builds in the part.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    with_context,
};
use geop_ops::{
    Context, EntityRef, Library, Namer, ORIGIN, Part,
    assembly::Mate,
    operation::INSTANCE_SEPARATOR,
    operation::Operation,
    parameters::Parameter,
    part::{ParamValue, State, pose_parameter},
    ui::{CanvasEvent, Edit, Form},
};
use serde::{Deserialize, Serialize};

use crate::editor::{self, PartSession};

/// Places the part the program in another file builds, named by the
/// step's id: its entities are then the part's, behind that name —
/// `bolt/extrude(head,end)` (see [`geop_ops::operation::INSTANCE_SEPARATOR`]).
///
/// The part goes where the program's parameter `<id>.pose` puts it (see
/// [`geop_ops::part::State`]), and the step adds its mates. It solves
/// nothing: where every placed part is, so that every mate of the program
/// holds, is the program's state, which solving the program changes (see
/// [`geop_ops::assembly`]). So every step sees each placed part where it
/// ends up, however late the mates that put it there are.
///
/// Mates that cannot hold do not fail the step — the parts are left as near
/// to holding them as they get, as a sketch whose constraints conflict is —
/// and the step's editor says which.
///
/// Every pose parameter of the program placed becomes one of this
/// program's, named behind the step's id and starting where that program
/// has it, and the part is built with them (see
/// [`geop_ops::Library::instance`]): the parts placed in it are this
/// program's to move.
///
/// A [`Mate::fixed`] among its mates, given no entity, holds the part this
/// step places.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddPart;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AddPartArgs {
    /// The program file whose part is placed, relative to this program's
    /// own file; empty until one is chosen.
    #[serde(default)]
    pub file: String,
    /// The mates this step adds, by an id of their own: `m1`, `m2`, ...
    /// Each is named `add_part(step,id)` in the part.
    #[serde(default)]
    pub mates: BTreeMap<String, Mate>,
    /// The values the part's own parameters (see
    /// [`geop_ops::parameters::Parameters`]) are given here instead of its
    /// own, by name: a number, a table's row, the colour — the part as this
    /// program places it.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub parameters: State,
}

impl AddPartArgs {
    /// The step that also holds the part it places fixed.
    pub fn fixed(mut self) -> Self {
        self.set_fixed(true);
        self
    }

    /// Holds the part this step places fixed, or — not `fixed` — frees it
    /// of every fixed mate.
    pub fn set_fixed(&mut self, fixed: bool) {
        if fixed {
            self.mates.insert("fixed".into(), Mate::fixed_here());
        } else {
            self.mates.retain(|_, mate| !mate.is_fixed());
        }
    }

    /// Whether a fixed mate holds the part this step places.
    pub(crate) fn is_fixed(&self) -> bool {
        self.mates.values().any(Mate::is_fixed)
    }
}

impl Operation for AddPart {
    type Args = AddPartArgs;
    type Session = PartSession;

    const PICKS_BUILT: bool = true;

    /// No file yet; the first part placed is fixed, the ones after it are
    /// not — something has to stay put for the others to mate to.
    fn new_args<S: Scalar>(&self, before: &Part<S>) -> AddPartArgs {
        let args = AddPartArgs::default();
        if before.instances().next().is_none() {
            args.fixed()
        } else {
            args
        }
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &AddPartArgs,
        library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("add_part({operation_id}, file={:?})", args.file);
        if args.file.is_empty() {
            return Err(GeopError::new("choose a file to place")).with_context(ctx);
        }
        // Its parameters as this step gives them, and where its parts are
        // is this program's too.
        let mut overrides = args.parameters.clone();
        let own = library
            .instance(&args.file, &State::new())
            .with_context(ctx)?;
        for (name, value) in own.part.declared() {
            if let ParamValue::Pose(pose) = *value {
                let outer = format!("{operation_id}{INSTANCE_SEPARATOR}{name}");
                let pose = part.pose_parameter(&outer, pose).with_context(ctx)?;
                overrides.insert(name.clone(), ParamValue::Pose(pose));
            }
        }
        let mut instance = library.instance(&args.file, &overrides).with_context(ctx)?;
        let parameter = pose_parameter(operation_id);
        let pose = part
            .pose_parameter(&parameter, Pose::identity())
            .with_context(ctx)?;
        instance.pose = pose.cast();
        instance.parameter = Some(Parameter::pose(parameter));
        part.add_instance(instance, operation_id)
            .with_context(ctx)?;
        let namer = Namer::new("add_part", operation_id)?;
        let own_frame = EntityRef::datum(format!("{operation_id}{INSTANCE_SEPARATOR}{ORIGIN}"));
        for (mate_id, mate) in &args.mates {
            let mut mate = mate.clone();
            if mate.is_fixed() && mate.entities.is_empty() {
                mate.entities.push(own_frame.clone());
            }
            if mate.is_complete() {
                part.add_mate(mate, namer.name(&[mate_id]))
                    .with_context(ctx)?;
            }
        }
        Ok(part)
    }

    /// The file, where it goes, and its mates: see [`crate::editor`].
    fn form<'a, S: Scalar>(
        &self,
        context: Context<'a, S>,
        args: &AddPartArgs,
        session: &PartSession,
        _: &[String],
    ) -> Form<'a, S, AddPartArgs, PartSession> {
        editor::form(context, args, session)
    }

    fn event<S: Scalar>(
        &self,
        context: Context<'_, S>,
        edit: Edit<'_, AddPartArgs, PartSession>,
        event: &CanvasEvent<S>,
    ) {
        editor::event(context, edit, event);
    }
}

#[cfg(test)]
mod tests;
