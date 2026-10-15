//! [`PartOperation`]: an operation and how it is edited, in one trait — what
//! a step is given, how it runs, and the one form it is edited through,
//! whose fields each set what they show, so a field's key is written once —
//! and [`part_operation!`](crate::part_operation), which makes it an
//! [`Operation`](geop_core_program::operation::Operation) and an
//! [`Editable`](crate::editable::Editable), its editing state a
//! [`FormState`] of its session.

use std::fmt::Debug;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    part::{Input, Part, State},
    program_runner::ProgramRunner,
};
use serde::{Serialize, de::DeserializeOwned};

use crate::{
    edit_form::{Edit, EditForm, FormState},
    editable::{Context, OperationInfo},
    editing::CanvasEvent,
};

/// One kind of step of a part, and how it is edited.
pub trait PartOperation: 'static {
    /// The name programs store its steps under.
    const KIND: &'static str;
    const INFO: OperationInfo;
    /// What a step is given. It is saved with the program, so it names
    /// targets and elements, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + Debug + PartialEq + Clone + 'static;
    /// The inputs and state of the parts it builds, and the numbers its
    /// form measures in.
    type Input: Input;
    type State: State;
    type Scalar: Scalar + PartialEq;
    /// What the editor keeps while a step is edited besides its arguments
    /// and the visuals selected: `()` for most.
    type Session: Clone + Default + 'static;

    /// The arguments of a new step: sensible defaults, taking the part it
    /// is inserted into into account.
    fn new_args(context: &Context<Self::Input, Self::State, Self::Scalar>) -> Self::Args;

    /// Runs the step `step` on `part`; `runner` is what runs it, to run
    /// programs of the step's own with.
    fn run(
        step: &str,
        runner: &ProgramRunner<Self::Input, Self::State>,
        part: &mut Part<Self::Input, Self::State>,
        args: &Self::Args,
    ) -> GeopResult<()>;

    /// What the editor shows while the step is edited, with what setting
    /// each field does. Never fails: an editor needs the form most exactly
    /// when the arguments do not build, so whatever is wrong is said in it.
    fn form<'a>(
        context: &'a Context<'a, Self::Input, Self::State, Self::Scalar>,
        args: &Self::Args,
        editing: &FormState<Self::Session>,
    ) -> EditForm<'a, Self::Scalar, Self::Args, Self::Session>;

    /// What the session forgets at the start of every event, before the
    /// event is handled: nothing, by default.
    fn begin_event(session: &mut Self::Session) {
        let _ = session;
    }

    /// What the pointer or a key did on the canvas: nothing, by default.
    fn canvas(
        context: &Context<Self::Input, Self::State, Self::Scalar>,
        edit: Edit<'_, Self::Args, Self::Session>,
        event: &CanvasEvent<Self::Scalar>,
    ) {
        let _ = (context, edit, event);
    }
}

/// Makes each [`PartOperation`] named, `Operation(Args)`, an
/// [`Operation`](geop_core_program::operation::Operation) and an
/// [`Editable`](crate::editable::Editable), edited through its form — and
/// its arguments the [`StepArgs`](geop_core_program::operation::StepArgs)
/// of it. (Named, since what an operation's `Args` stand for is not known
/// to other crates' impls.)
#[macro_export]
macro_rules! part_operation {
    ($($operation:ident($args:ty)),* $(,)?) => {$(
        impl $crate::__program::operation::StepArgs for $args {
            const KIND: &'static str = <$operation as $crate::part_operation::PartOperation>::KIND;
        }

        impl $crate::__program::operation::Operation for $operation {
            const KIND: &'static str = <$operation as $crate::part_operation::PartOperation>::KIND;
            type Args = <$operation as $crate::part_operation::PartOperation>::Args;
            type Input = <$operation as $crate::part_operation::PartOperation>::Input;
            type State = <$operation as $crate::part_operation::PartOperation>::State;

            fn run(
                step: &str,
                runner: &$crate::__program::program_runner::ProgramRunner<
                    Self::Input,
                    Self::State,
                >,
                part: &mut $crate::__program::part::Part<Self::Input, Self::State>,
                args: &Self::Args,
            ) -> $crate::__GeopResult<()> {
                <$operation as $crate::part_operation::PartOperation>::run(step, runner, part, args)
            }
        }

        impl $crate::editable::Editable for $operation {
            const INFO: $crate::editable::OperationInfo =
                <$operation as $crate::part_operation::PartOperation>::INFO;
            type Scalar = <$operation as $crate::part_operation::PartOperation>::Scalar;
            type EditingState = $crate::edit_form::FormState<
                <$operation as $crate::part_operation::PartOperation>::Session,
            >;

            fn new_args(
                context: &$crate::editable::Context<Self::Input, Self::State, Self::Scalar>,
            ) -> Self::Args {
                <$operation as $crate::part_operation::PartOperation>::new_args(context)
            }

            fn presentation(
                context: &$crate::editable::Context<Self::Input, Self::State, Self::Scalar>,
                args: &Self::Args,
                editing: &Self::EditingState,
            ) -> $crate::editing::Form<Self::Scalar> {
                <$operation as $crate::part_operation::PartOperation>::form(context, args, editing)
                    .into_form()
            }

            fn process_user_event(
                context: &$crate::editable::Context<Self::Input, Self::State, Self::Scalar>,
                args: &Self::Args,
                editing: &Self::EditingState,
                event: $crate::editing::Event<Self::Scalar>,
            ) -> (Self::Args, Self::EditingState) {
                let mut editing = editing.clone();
                <$operation as $crate::part_operation::PartOperation>::begin_event(
                    &mut editing.session,
                );
                editing.handle(
                    args,
                    event,
                    |args, editing| {
                        <$operation as $crate::part_operation::PartOperation>::form(
                            context, args, editing,
                        )
                    },
                    |edit, event| {
                        <$operation as $crate::part_operation::PartOperation>::canvas(
                            context, edit, event,
                        )
                    },
                )
            }
        }
    )*};
}
