//! [`AddPart`]: place the part another program file builds in the part.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    with_context,
};
use geop_ops::operation::{OperationGroup, OperationInfo, OperationTier};
use geop_ops::{
    Design, EntityRef, Namer, ORIGIN,
    model::pose_parameter,
    operation::{Build, BuildReader, PartOperation, StepContext, instance_path},
    parameters::Values,
    part_operation,
    placed::{Instance, Placement, instance, library},
    ui::{self, CanvasEvent, Edit, EditForm, Listed, Listing, listing},
};
use serde::{Deserialize, Serialize};

use crate::{
    Mate, Resolved,
    editor::{self, PartEditing, PartSession},
    mate,
    mechanism::{define, mates_of, resolve},
};

/// Places the part the program in another file builds, named by the
/// step's id: its entities are then the part's, behind that name —
/// `bolt/extrude(head,end)` (see [`geop_ops::operation::INSTANCE_SEPARATOR`]).
///
/// The part goes where the program's state `<id>.pose` puts it, and the
/// step adds its mates. It solves nothing: where every placed part is, so
/// that every mate of the program holds, is the program's state, which the
/// editor solves for (see [`crate::mechanism`]). So every step sees each
/// placed part where it ends up, however late the mates that put it there
/// are.
///
/// Mates that cannot hold do not fail the step — the parts are left as near
/// to holding them as they get, as a sketch whose constraints conflict is —
/// and the step's editor says which.
///
/// The parts placed in the part placed are this program's to move too:
/// their poses are its state, behind the step's id, starting where the
/// placed part's program has them (see [`geop_ops::placed`]); and their
/// mates hold them here as they held them there.
///
/// A [`Mate::fixed_here`] among its mates, given no entity, holds the part this
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
    pub parameters: Values,
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

/// The part the step `step` places from `file`, its parameters as
/// `parameters` give them, where its pose state puts it — held there if
/// `fixed`: the generator of its instance.
fn place(
    reader: &mut BuildReader,
    (step, file, parameters, fixed): &(String, String, Values, bool),
) -> GeopResult<Instance> {
    let component = library(reader)?.component(file, parameters)?;
    Ok(Instance {
        component,
        placement: Placement::Parameter(pose_parameter(step)),
        fixed: *fixed,
    })
}

/// What a mate is listed as in the part's structure (see [`Listed::kind`]).
const MATE: &str = "mate";

/// A mate of the part placed, as this part has it already: the generator of
/// its target.
fn carry(_: &mut BuildReader, resolved: &Resolved) -> GeopResult<Resolved> {
    Ok(resolved.clone())
}

part_operation!(AddPart(AddPartArgs));

impl PartOperation for AddPart {
    const INFO: OperationInfo = OperationInfo::new(
        "add_part",
        "Part",
        "Place the part another program file builds, and mate it to what is already there.",
        OperationGroup::Assembly,
        OperationTier::Big,
    )
    .picks_built()
    .icon(crate::icons::ADD_PART);
    type Args = AddPartArgs;
    type Session = PartSession;

    /// No file yet; the first part placed is fixed, the ones after it are
    /// not — something has to stay put for the others to mate to.
    fn new_args(context: &StepContext) -> AddPartArgs {
        let args = AddPartArgs::default();
        match context.before.targets_of::<Instance>().is_empty() {
            true => args.fixed(),
            false => args,
        }
    }

    /// Pulls released at the event before are let go of.
    fn begin_event(session: &mut PartSession) {
        session.let_go();
    }

    /// Places the part, then defines each mate: resolved, and as a
    /// constraint on the state. A mate that cannot be resolved fails the
    /// step, after the others are defined.
    fn run(step: &str, part: &mut Build, args: &AddPartArgs) -> GeopResult<()> {
        let ctx = with_context!("add_part({step}, file={:?})", args.file);
        if args.file.is_empty() {
            return Err(GeopError::new("choose a file to place")).with_context(ctx);
        }
        let placing = (
            step.to_string(),
            args.file.clone(),
            args.parameters.clone(),
            args.is_fixed(),
        );
        part.define_target(&instance(step), &placing, place)
            .with_context(ctx)?;
        let namer = Namer::new("add_part", step)?;
        let own_frame = EntityRef::datum(instance_path(step, ORIGIN));
        let mut failed = Ok(());
        // What the structure lists: every mate the step defines.
        let mut mates = Vec::new();
        for (id, mate) in &args.mates {
            let mut mate = mate.clone();
            if mate.is_fixed() && mate.entities.is_empty() {
                mate.entities.push(own_frame.clone());
            }
            if !mate.is_complete() {
                continue;
            }
            let name = namer.name(&[id]);
            mates.push(Listed {
                kind: MATE.into(),
                name: name.clone(),
                entities: mate.entities.clone(),
            });
            let label = format!("{step} {id}");
            let defined = (part.define_target(&crate::mate(&name), &(name.clone(), mate), resolve))
                .and_then(|()| define(part, &name, &label));
            failed = failed.and(defined);
        }
        // The placed part's own mates, behind its name.
        let carried: Vec<(String, Resolved)> = match part.current().retrieve_target(&instance(step))
        {
            Ok(placed) => (mates_of(placed.component.view()).into_iter())
                .filter_map(|(name, resolved)| {
                    let name = instance_path(step, name);
                    Some((name, resolved.behind(step)?))
                })
                .collect(),
            Err(_) => Vec::new(),
        };
        for (name, resolved) in carried {
            mates.push(Listed {
                kind: MATE.into(),
                name: name.clone(),
                entities: resolved.entities(),
            });
            let defined = (part.define_target(&mate(&name), &resolved, carry))
                .and_then(|()| define(part, &name, &name));
            failed = failed.and(defined);
        }
        let listed = part.define_target(&listing(step), &Listing(mates), ui::listed);
        failed.and(listed).with_context(ctx)
    }

    /// The file, where it goes, and its mates: see [`crate::editor`].
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &AddPartArgs,
        editing: &PartEditing,
    ) -> EditForm<'a, Design, AddPartArgs, PartSession> {
        editor::form(context, args, &editing.session)
    }

    /// The part dragged; Delete removes the mate selected, Escape stops
    /// editing it: see [`crate::editor`].
    fn canvas(
        context: &StepContext,
        edit: Edit<'_, AddPartArgs, PartSession>,
        event: &CanvasEvent<Design>,
    ) {
        editor::canvas(context, edit, event)
    }
}

#[cfg(test)]
mod tests;
