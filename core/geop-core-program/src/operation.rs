use std::any::Any;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::{
    editing::{Event, Form},
    part::{self, Part, View},
    step_editor::Scene,
};

/// The numbers the state of `O`'s parts is made of: what its forms and
/// events measure in.
pub type ScalarOf<O> =
    <<O as Operation>::State as geop_core_math::solvers::system::Variable>::Scalar;

/// What an operation is to an editor: how a step spells it, its short name
/// and what it does — what an editor offers it by — and where and how
/// prominently the editor offers it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct OperationInfo {
    /// The name programs store its steps under.
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    pub group: OperationGroup,
    pub tier: OperationTier,
    /// Not yet robust enough to offer by default: an editor's reduced mode
    /// leaves it out.
    pub experimental: bool,
    /// Whether its reference fields pick in the part as the step builds it,
    /// rather than as it was before: a placed part's mates pick entities of
    /// the part it places.
    pub picks_built: bool,
    /// The extensions of the files its steps read that are no programs —
    /// data a program keeps beside it, a STEP file: what a front end keeps
    /// in a workspace with the programs, and sends the editor.
    pub reads: &'static [&'static str],
    /// What it is shown as: SVG on a 20 × 20 grid, drawn in the current
    /// colour — strokes `currentColor`, unfilled unless it says otherwise —
    /// that a front end puts in an `<svg>` of its own. None to show its
    /// label.
    pub icon: Option<&'static str>,
}

impl OperationInfo {
    /// An operation offered by default, whose reference fields pick in the
    /// part as it was before the step.
    pub const fn new(
        kind: &'static str,
        label: &'static str,
        doc: &'static str,
        group: OperationGroup,
        tier: OperationTier,
    ) -> Self {
        Self {
            kind,
            label,
            doc,
            group,
            tier,
            experimental: false,
            picks_built: false,
            reads: &[],
            icon: None,
        }
    }

    /// The same, but left out of an editor's reduced mode.
    pub const fn experimental(self) -> Self {
        Self {
            experimental: true,
            ..self
        }
    }

    /// The same, but picking in the part as the step builds it.
    pub const fn picks_built(self) -> Self {
        Self {
            picks_built: true,
            ..self
        }
    }

    /// The same, its steps reading files of the extensions `reads`.
    pub const fn reads(self, reads: &'static [&'static str]) -> Self {
        Self { reads, ..self }
    }

    /// The same, shown as `icon` (see [`OperationInfo::icon`]).
    pub const fn icon(self, icon: &'static str) -> Self {
        Self {
            icon: Some(icon),
            ..self
        }
    }
}

/// What an operation works on, as an editor groups its operations: the
/// toolbar's sections, in this order — the order of a part's workflow.
/// Serialized as the group's name, as it is shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub enum OperationGroup {
    /// Sketches, and the reference geometry they are drawn on.
    Sketch,
    /// What makes a solid of a sketch: extrusions, revolutions, sweeps and
    /// lofts.
    Solid,
    /// Features that change a solid: holes, rounds, walls, threads.
    Features,
    /// Whole bodies: patterned, mirrored, combined, cut, moved.
    #[serde(rename = "Pattern & bodies")]
    Bodies,
    /// Faces standing on their own, and freeform shapes.
    Surface,
    /// Bodies of bent sheet, and their flat patterns.
    #[serde(rename = "Sheet metal")]
    SheetMetal,
    /// Placed parts, and what runs between them.
    Assembly,
    /// What is made of a part rather than in it: drawings, exports.
    Output,
}

/// How prominently an editor offers an operation within its group, from
/// the few used most to the rarely used: the order the group lists them in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub enum OperationTier {
    /// One of the few used most: a big button.
    Big,
    /// Used often: a small button.
    Small,
    /// Used rarely: only in its group's menu.
    Menu,
}

/// What a step is edited against: its id — what everything it builds is
/// named after — the part as it was before it, the part as the step
/// built it when it last ran, if it did, and, for an event, the part as the
/// editor draws it — what a pointer is over, without drawing it again.
pub struct Context<'a, I, S: part::State> {
    pub step: &'a str,
    pub before: View<'a, I, S>,
    pub built: Option<View<'a, I, S>>,
    /// The part as drawn for picks (see [`crate::step_editor::Scene`]):
    /// given with every event an editor passes on, and nothing else.
    pub scene: Option<&'a dyn Scene<S::Scalar>>,
}

/// One kind of step. An operation is a unit struct: everything a step of
/// it needs is in its arguments, and everything it keeps while a step is
/// edited in its editing state.
pub trait Operation: 'static {
    const INFO: OperationInfo;
    /// What a step of the operation is given. It is saved with the program,
    /// so it names targets and entities, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + 'static;
    /// What the editor keeps about a step while it is edited, beyond its
    /// arguments, starting from the default whenever a step is opened.
    type EditingState: Default + 'static;
    /// The inputs and state of the parts the operation builds.
    type Input: part::Input;
    type State: part::State;

    /// The arguments of a new step: sensible defaults, taking the part it
    /// is inserted into into account.
    fn new_args(context: &Context<Self::Input, Self::State>) -> Self::Args;

    fn run(
        step: &str,
        part: &mut Part<Self::Input, Self::State>,
        args: &Self::Args,
    ) -> GeopResult<()>;

    /// What the editor shows while the step is edited. Never fails: an
    /// editor needs the form most exactly when the arguments do not build,
    /// so whatever is wrong is said in it.
    fn presentation(
        context: &Context<Self::Input, Self::State>,
        args: &Self::Args,
        editing: &Self::EditingState,
    ) -> Form<ScalarOf<Self>>;

    /// The step's arguments and editing state after `event`.
    fn process_user_event(
        context: &Context<Self::Input, Self::State>,
        args: &Self::Args,
        editing: &Self::EditingState,
        event: Event<ScalarOf<Self>>,
    ) -> (Self::Args, Self::EditingState);

    /// The text of every formula in `args` that reads inputs by name — what
    /// tells which inputs a step reads, and what renaming one renames. An
    /// operation whose arguments take formulas lists every one, used or
    /// not; none, by default.
    fn formulas(args: &mut Self::Args) -> Vec<&mut String> {
        let _ = args;
        Vec::new()
    }
}

/// A step's arguments, as saved, and its editing state after an event.
pub struct Edited {
    pub args: serde_json::Value,
    pub editing: Box<dyn Any>,
}

/// An [`Operation`] with its types erased, so that operations of all kinds
/// can be kept in one [`crate::operation_registry::OperationRegistry`].
/// Every operation is one, through its value. Arguments come as saved.
pub trait ErasedOperation<I, S: part::State> {
    fn info(&self) -> &'static OperationInfo;
    fn new_args(&self, context: &Context<I, S>) -> GeopResult<serde_json::Value>;
    fn run(&self, step: &str, part: &mut Part<I, S>, args: &serde_json::Value) -> GeopResult<()>;
    fn initial_editing(&self) -> Box<dyn Any>;
    fn presentation(
        &self,
        context: &Context<I, S>,
        args: &serde_json::Value,
        editing: &dyn Any,
    ) -> GeopResult<Form<S::Scalar>>;
    fn process_user_event(
        &self,
        context: &Context<I, S>,
        args: &serde_json::Value,
        editing: &dyn Any,
        event: Event<S::Scalar>,
    ) -> GeopResult<Edited>;

    /// The arguments `args` with `edit` applied to every formula in them
    /// (see [`Operation::formulas`]): to read them, an `edit` that changes
    /// nothing.
    fn edit_formulas(
        &self,
        args: &serde_json::Value,
        edit: &mut dyn FnMut(&mut String),
    ) -> GeopResult<serde_json::Value>;
}

/// The arguments of `O` from their saved form.
fn args<O: Operation>(json: &serde_json::Value) -> GeopResult<O::Args> {
    O::Args::deserialize(json)
        .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::INFO.kind)))
}

/// The saved form of `args`.
pub(crate) fn saved<O: Operation>(args: &O::Args) -> GeopResult<serde_json::Value> {
    serde_json::to_value(args)
        .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::INFO.kind)))
}

fn editing<O: Operation>(editing: &dyn Any) -> GeopResult<&O::EditingState> {
    editing.downcast_ref().ok_or_else(|| {
        GeopError::new(format!(
            "operation `{}` was given an editing state that is not `{}`",
            O::INFO.kind,
            std::any::type_name::<O::EditingState>()
        ))
    })
}

impl<O: Operation> ErasedOperation<O::Input, O::State> for O {
    fn info(&self) -> &'static OperationInfo {
        &O::INFO
    }

    fn new_args(&self, context: &Context<O::Input, O::State>) -> GeopResult<serde_json::Value> {
        saved::<O>(&<O as Operation>::new_args(context))
    }

    fn run(
        &self,
        step: &str,
        part: &mut Part<O::Input, O::State>,
        args: &serde_json::Value,
    ) -> GeopResult<()> {
        <O as Operation>::run(step, part, &self::args::<O>(args)?)
    }

    fn initial_editing(&self) -> Box<dyn Any> {
        Box::new(O::EditingState::default())
    }

    fn presentation(
        &self,
        context: &Context<O::Input, O::State>,
        args: &serde_json::Value,
        editing: &dyn Any,
    ) -> GeopResult<Form<ScalarOf<O>>> {
        Ok(<O as Operation>::presentation(
            context,
            &self::args::<O>(args)?,
            self::editing::<O>(editing)?,
        ))
    }

    fn process_user_event(
        &self,
        context: &Context<O::Input, O::State>,
        args: &serde_json::Value,
        editing: &dyn Any,
        event: Event<ScalarOf<O>>,
    ) -> GeopResult<Edited> {
        let (args, editing) = <O as Operation>::process_user_event(
            context,
            &self::args::<O>(args)?,
            self::editing::<O>(editing)?,
            event,
        );
        Ok(Edited {
            args: saved::<O>(&args)?,
            editing: Box::new(editing),
        })
    }

    fn edit_formulas(
        &self,
        args: &serde_json::Value,
        edit: &mut dyn FnMut(&mut String),
    ) -> GeopResult<serde_json::Value> {
        let mut args = self::args::<O>(args)?;
        for formula in O::formulas(&mut args) {
            edit(formula);
        }
        saved::<O>(&args)
    }
}
