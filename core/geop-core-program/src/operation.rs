use std::any::Any;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Serialize, de::DeserializeOwned};

use crate::part::{self, Part, View};

pub enum UserEvent {
    PointAndClick,
    Drag,
}

pub struct EditingPresentation {
    pub camera_control: (),
    pub reference_inputs: (),
    pub drag_handles: (),
}

pub trait Operation: 'static {
    /// The name programs store the operation's steps under.
    const NAME: &'static str;
    /// What a step of the operation is given. It is saved with the program,
    /// so it names targets and entities, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + 'static;
    /// What the editor remembers about a step while it is edited, starting
    /// from the default.
    type EditingState: Default + 'static;
    /// The inputs and state of the parts the operation builds.
    type Input: part::Input;
    type State: part::State;

    fn run(
        step: &str,
        part: &mut Part<Self::Input, Self::State>,
        args: &Self::Args,
    ) -> GeopResult<()>;
    /// What the editor shows while the step is edited, given the part as it
    /// was before the step.
    fn presentation(
        part: &View<Self::Input, Self::State>,
        args: &Self::Args,
        state: &Self::EditingState,
    ) -> GeopResult<EditingPresentation>;
    /// The step's arguments and state after `event`, given the part as it was
    /// before the step.
    fn process_user_event(
        part: &View<Self::Input, Self::State>,
        args: &Self::Args,
        state: &Self::EditingState,
        event: UserEvent,
    ) -> GeopResult<(Self::Args, Self::EditingState)>;
}

/// A step's arguments and editing state after a user event, type erased.
pub struct Edited {
    pub args: Box<dyn Any>,
    pub state: Box<dyn Any>,
}

/// An [`Operation`] with its types erased, so that operations of all kinds
/// can be kept in one [`crate::operation_registry::OperationRegistry`]. Every operation is
/// one, through its value.
pub trait ErasedOperation<I, S> {
    fn run(&self, step: &str, part: &mut Part<I, S>, args: &dyn Any) -> GeopResult<()>;
    fn initial_state(&self) -> Box<dyn Any>;
    fn presentation(
        &self,
        part: &View<I, S>,
        args: &dyn Any,
        state: &dyn Any,
    ) -> GeopResult<EditingPresentation>;
    fn process_user_event(
        &self,
        part: &View<I, S>,
        args: &dyn Any,
        state: &dyn Any,
        event: UserEvent,
    ) -> GeopResult<Edited>;
    fn args_from_json(&self, json: serde_json::Value) -> GeopResult<Box<dyn Any>>;
    fn args_to_json(&self, args: &dyn Any) -> GeopResult<serde_json::Value>;
}

fn downcast<'a, O: Operation, T: 'static>(value: &'a dyn Any, what: &str) -> GeopResult<&'a T> {
    value.downcast_ref::<T>().ok_or_else(|| {
        GeopError::new(format!(
            "operation `{}` was given {what} that are not `{}`",
            O::NAME,
            std::any::type_name::<T>()
        ))
    })
}

impl<O: Operation> ErasedOperation<O::Input, O::State> for O {
    fn run(
        &self,
        step: &str,
        part: &mut Part<O::Input, O::State>,
        args: &dyn Any,
    ) -> GeopResult<()> {
        <O as Operation>::run(step, part, downcast::<O, O::Args>(args, "arguments")?)
    }

    fn initial_state(&self) -> Box<dyn Any> {
        Box::new(O::EditingState::default())
    }

    fn presentation(
        &self,
        part: &View<O::Input, O::State>,
        args: &dyn Any,
        state: &dyn Any,
    ) -> GeopResult<EditingPresentation> {
        <O as Operation>::presentation(
            part,
            downcast::<O, O::Args>(args, "arguments")?,
            downcast::<O, O::EditingState>(state, "an editing state")?,
        )
    }

    fn process_user_event(
        &self,
        part: &View<O::Input, O::State>,
        args: &dyn Any,
        state: &dyn Any,
        event: UserEvent,
    ) -> GeopResult<Edited> {
        let (args, state) = <O as Operation>::process_user_event(
            part,
            downcast::<O, O::Args>(args, "arguments")?,
            downcast::<O, O::EditingState>(state, "an editing state")?,
            event,
        )?;
        Ok(Edited {
            args: Box::new(args),
            state: Box::new(state),
        })
    }

    fn args_from_json(&self, json: serde_json::Value) -> GeopResult<Box<dyn Any>> {
        let args: O::Args = serde_json::from_value(json)
            .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::NAME)))?;
        Ok(Box::new(args))
    }

    fn args_to_json(&self, args: &dyn Any) -> GeopResult<serde_json::Value> {
        serde_json::to_value(downcast::<O, O::Args>(args, "arguments")?)
            .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::NAME)))
    }
}
