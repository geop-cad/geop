//! [`AddPart`]: place the part another program file builds in the part.

use geop_core_editor::{
    edit_form::{Edit, EditForm},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    editing::CanvasEvent,
    part_operation,
    part_operation::PartOperation,
    selectable::Selectable,
};
use geop_core_program::{
    element::ElementReference,
    part::StepContext,
    values::{Design, Settings, pose_state},
};
use std::collections::BTreeMap;

use geop_core_editor::view::{Listed, Listing, listing};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    with_context,
};
use geop_ops_model::datums::{ORIGIN, datums};
use geop_ops_model::Namer;
use geop_ops_parameters::{Placement, instances};
use geop_ops_reference_geometry::Selected;
use serde::{Deserialize, Serialize};

use crate::{
    Mate, MateEntity,
    editor::{self, PartEditing, PartSession},
    mechanism::{define, resolve},
    placed::define_placed,
};

/// Places the part the program in another file builds, named by the
/// step's id: its elements are then the part's, behind that name (see
/// [`MateEntity`]).
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
/// their poses are its state, behind the step's id, which the part placed
/// is run with — starting where its own program has them — and their mates
/// come up with its constraints, to hold them here as they held them there
/// (see [`geop_core_program::instance`]).
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
    /// [`geop_ops_parameters::Parameters`]) are given here instead of its
    /// own, by name: a number, a table's row, the colour — the part as this
    /// program places it.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub parameters: Settings,
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
}

/// What a mate is listed as in the part's structure (see [`Listed::kind`]).
const MATE: &str = "mate";

part_operation!(AddPart(AddPartArgs));

impl PartOperation for AddPart {
    const KIND: &'static str = "add_part";
    const INFO: OperationInfo = OperationInfo::new(
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
    fn new_args(context: &Context) -> AddPartArgs {
        let args = AddPartArgs::default();
        match instances(&context.before).is_ok_and(|placed| placed.is_empty()) {
            true => args.fixed(),
            false => args,
        }
    }

    /// Pulls released at the event before are let go of.
    fn begin_event(session: &mut PartSession) {
        session.let_go();
    }

    /// Places the part, then defines each mate: resolved, and as
    /// constraints on the state; and lets a solve move the part, as far as
    /// its mates let it. A mate that cannot be resolved fails the step,
    /// after the others are defined. The placed part's own mates come up
    /// with its constraints (see [`geop_lib_constraints::Constraints::of`]).
    fn run(ctx: &mut StepContext, args: &AddPartArgs) -> GeopResult<()> {
        let step = ctx.step();
        let context = with_context!("add_part({step}, file={:?})", args.file);
        if args.file.is_empty() {
            return Err(GeopError::new("choose a file to place")).with_context(context);
        }
        let placement = Placement::State(pose_state(step));
        define_placed(ctx, step, &args.file, &args.parameters, placement).with_context(context)?;
        let namer = Namer::new("add_part", step)?;
        let own_frame = MateEntity::of(
            step,
            Selected::Datum(ElementReference::new(datums(), ORIGIN)),
        );
        let mut failed = Ok(());
        // What the structure lists: every mate the step defines; what the
        // state is constrained by: each, by its name and what it is called.
        let mut listed = Vec::new();
        let mut defined = Vec::new();
        for (id, mate) in &args.mates {
            let mut mate = mate.clone();
            if mate.is_fixed() && mate.entities.is_empty() {
                mate.entities.push(own_frame.clone());
            }
            if !mate.is_complete() {
                continue;
            }
            let name = namer.name(&[id]);
            listed.push(Listed {
                kind: MATE.into(),
                name: name.clone(),
                elements: mate.entities.iter().map(Selectable::picked).collect(),
            });
            let resolved = ctx.produce(&crate::mate(&name), &(name.clone(), mate), resolve);
            if resolved.is_ok() {
                defined.push((name, format!("{step} {id}")));
            }
            failed = failed.and(resolved);
        }
        let constrained = define(ctx, Some(&pose_state(step)), &defined);
        let listing = ctx.produce(&listing(step), &Listing(listed), |_, l| Ok(l.clone()));
        failed.and(constrained).and(listing).with_context(context)
    }

    /// The file, where it goes, and its mates: see [`crate::editor`].
    fn form<'a>(
        context: &'a Context<'a>,
        args: &AddPartArgs,
        editing: &PartEditing,
    ) -> EditForm<'a, Design, AddPartArgs, PartSession> {
        editor::form(context, args, &editing.session)
    }

    /// The part dragged; Delete removes the mate selected, Escape stops
    /// editing it: see [`crate::editor`].
    fn canvas(
        context: &Context,
        edit: Edit<'_, AddPartArgs, PartSession>,
        event: &CanvasEvent<Design>,
    ) {
        editor::canvas(context, edit, event)
    }
}

#[cfg(test)]
mod tests;
