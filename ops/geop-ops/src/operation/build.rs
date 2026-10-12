//! [`PartOperation`]: what a kind of step of a part is — what it is given,
//! what it builds, and the one form it is edited through — and
//! [`part_operation!`](crate::part_operation), which makes it a
//! [`geop_core_program::operation::Operation`]. [`build_model`] makes a
//! step's revision of the part's model, as most steps' runs do.

use std::fmt::Debug;

use geop_core_math::geop_error::{GeopError, GeopResult};

use geop_core_program::operation::{Context, OperationInfo};
use serde::{Serialize, de::DeserializeOwned};

use super::{Build, BuildReader, Input};
use crate::{
    Design,
    model::{NamedModel, ParamValue, model},
    ui::{CanvasEvent, Edit, EditForm, FormState},
};

/// What a step of a part is edited against (see [`Context`]).
pub type StepContext<'a> = Context<'a, Input, ParamValue>;

/// One kind of step of a part: an extrude, a boolean, a sketch. Everything
/// an editor needs of it besides its arguments is said by its form
/// ([`PartOperation::form`]), whose fields each set what they show — so a
/// field's key is written once. [`part_operation!`](crate::part_operation)
/// makes it an [`Operation`](geop_core_program::operation::Operation), its
/// editing state a [`FormState`] of its session.
pub trait PartOperation: 'static {
    const INFO: OperationInfo;
    /// What a step is given. It is saved with the program, so it names
    /// targets and entities, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + Debug + PartialEq + Clone + 'static;
    /// What the editor keeps while a step is edited besides its arguments
    /// and the visuals selected: `()` for most.
    type Session: Clone + Default + 'static;

    /// The arguments of a new step: sensible defaults, taking the part it
    /// is inserted into into account.
    fn new_args(context: &StepContext) -> Self::Args;

    /// The text of every formula in `args` that reads inputs by name (see
    /// [`geop_core_program::operation::Operation::formulas`]): every one,
    /// used or not.
    fn formulas(args: &mut Self::Args) -> Vec<&mut String> {
        let _ = args;
        Vec::new()
    }

    /// Runs the step `step` on `part`: by default builds it into the
    /// part's model ([`build_model`]). A step that defines targets of its
    /// own — a sketch, a datum, threads — says so here.
    fn run(step: &str, part: &mut Build, args: &Self::Args) -> GeopResult<()> {
        build_model::<Self>(part, step, args)
    }

    /// Builds the step `step` — what everything it makes is named after —
    /// into `model`, reading whatever else it builds on through `reader`
    /// (see [`super::Scope`]), so the step is rebuilt when that changes.
    /// What fails is said in the context of the step and its arguments.
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &Self::Args,
    ) -> GeopResult<()> {
        let _ = (model, reader, args);
        Err(GeopError::new(format!(
            "{}({step}) builds nothing into the model",
            Self::INFO.kind
        )))
    }

    /// What the editor shows while the step is edited, with what setting
    /// each field does. Never fails: an editor needs the form most exactly
    /// when the arguments do not build, so whatever is wrong is said in it.
    fn form<'a>(
        context: &'a StepContext<'a>,
        args: &Self::Args,
        editing: &FormState<Self::Session>,
    ) -> EditForm<'a, Design, Self::Args, Self::Session>;

    /// What the session forgets at the start of every event, before the
    /// event is handled: nothing, by default.
    fn begin_event(session: &mut Self::Session) {
        let _ = session;
    }

    /// What the pointer or a key did on the canvas: nothing, by default.
    fn canvas(
        context: &StepContext,
        edit: Edit<'_, Self::Args, Self::Session>,
        event: &CanvasEvent<Design>,
    ) {
        let _ = (context, edit, event);
    }
}

/// Makes each [`PartOperation`] named an
/// [`Operation`](geop_core_program::operation::Operation) of a part, edited
/// through its form.
#[macro_export]
macro_rules! part_operation {
    ($($operation:ty),* $(,)?) => {$(
        impl $crate::__program::operation::Operation for $operation {
            const INFO: $crate::__program::operation::OperationInfo =
                <$operation as $crate::operation::PartOperation>::INFO;
            type Args = <$operation as $crate::operation::PartOperation>::Args;
            type EditingState =
                $crate::ui::FormState<<$operation as $crate::operation::PartOperation>::Session>;
            type Input = $crate::operation::Input;
            type State = $crate::ParamValue;

            fn new_args(context: &$crate::operation::StepContext) -> Self::Args {
                <$operation as $crate::operation::PartOperation>::new_args(context)
            }

            fn run(
                step: &str,
                part: &mut $crate::operation::Build,
                args: &Self::Args,
            ) -> $crate::__program::__GeopResult<()> {
                <$operation as $crate::operation::PartOperation>::run(step, part, args)
            }

            fn presentation(
                context: &$crate::operation::StepContext,
                args: &Self::Args,
                editing: &Self::EditingState,
            ) -> $crate::ui::Form<$crate::Design> {
                <$operation as $crate::operation::PartOperation>::form(context, args, editing)
                    .into_form()
            }

            fn process_user_event(
                context: &$crate::operation::StepContext,
                args: &Self::Args,
                editing: &Self::EditingState,
                event: $crate::ui::Event<$crate::Design>,
            ) -> (Self::Args, Self::EditingState) {
                let mut editing = editing.clone();
                <$operation as $crate::operation::PartOperation>::begin_event(
                    &mut editing.session,
                );
                editing.handle(
                    args,
                    event,
                    |args, editing| {
                        <$operation as $crate::operation::PartOperation>::form(
                            context, args, editing,
                        )
                    },
                    |edit, event| {
                        <$operation as $crate::operation::PartOperation>::canvas(
                            context, edit, event,
                        )
                    },
                )
            }

            fn formulas(args: &mut Self::Args) -> Vec<&mut String> {
                <$operation as $crate::operation::PartOperation>::formulas(args)
            }
        }
    )*};
}

/// Makes the step `step`'s revision of the model: `O` building `args` into
/// the model as the steps before left it, or into an empty one, before the
/// first step that builds any.
pub fn build_model<O: PartOperation + ?Sized>(
    part: &mut Build,
    step: &str,
    args: &O::Args,
) -> GeopResult<()> {
    part.revise_or_start(&model(), &(step.to_string(), args.clone()), generate::<O>)
}

/// The generator of [`build_model`]: a function of `O` alone, so it
/// captures nothing — the step comes with the arguments.
fn generate<O: PartOperation + ?Sized>(
    mut model: NamedModel<Design>,
    reader: &mut BuildReader,
    (step, args): &(String, O::Args),
) -> GeopResult<NamedModel<Design>> {
    O::build(&mut model, reader, step, args)
        .map_err(|e| e.with_context(format!("{}({step}, {args:?})", O::INFO.kind)))?;
    Ok(model)
}

/// The generator of a target that is given: `model` itself.
pub fn given(_: &mut BuildReader, model: &NamedModel<Design>) -> GeopResult<NamedModel<Design>> {
    Ok(model.clone())
}

/// A part of one step, `given`, that defines its model as `model`: what
/// the tests of an operation, an exporter or an inspector build on.
pub fn given_part(model: &NamedModel<Design>) -> GeopResult<Build> {
    let mut part = Build::default();
    part.begin_run(&Default::default(), &Default::default());
    part.begin_step("given")?;
    part.define_target(&crate::model::model(), model, given)?;
    Ok(part)
}

/// What `O`'s step `step` with `args` makes of `model`, as a program of two
/// steps would build it — the model given, then the step — together with
/// what else the step made: the part, after its one run. For an
/// operation's tests, which build the model they act on directly.
pub fn apply<O>(model: NamedModel<Design>, step: &str, args: &O::Args) -> GeopResult<Build>
where
    O: geop_core_program::operation::Operation<Input = Input, State = ParamValue>,
{
    let mut part = given_part(&model)?;
    part.begin_step(step)?;
    O::run(step, &mut part, args)?;
    part.end_run(Default::default());
    Ok(part)
}

/// The model `O`'s step `step` with `args` leaves of `model` (see
/// [`apply`]): for the tests of an operation that builds on the model.
pub fn apply_to_model<O>(
    model: NamedModel<Design>,
    step: &str,
    args: &O::Args,
) -> GeopResult<NamedModel<Design>>
where
    O: geop_core_program::operation::Operation<Input = Input, State = ParamValue>,
{
    let part = apply::<O>(model, step, args)?;
    Ok(part
        .current()
        .retrieve_target(&crate::model::model())?
        .clone())
}
