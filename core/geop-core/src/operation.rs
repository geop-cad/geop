use crate::part::Part;

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
    type Args;
    type EditingState;

    fn run(id: &str, part: &mut Part, args: &Self::Args) -> Result<(), Box<dyn std::error::Error>>;
    fn presentation(args: &Self::Args, state: &Self::EditingState) -> EditingPresentation;
    fn process_user_event(
        args: &Self::Args,
        state: &Self::EditingState,
        event: UserEvent,
    ) -> (Self::Args, Self::EditingState);
}

pub trait ErasedOperation {
    fn run(
        &self,
        id: &str,
        part: &mut Part,
        args: &dyn std::any::Any,
    ) -> Result<(), Box<dyn std::error::Error>>;
    fn presentation(
        &self,
        args: &dyn std::any::Any,
        state: &dyn std::any::Any,
    ) -> Result<EditingPresentation, Box<dyn std::error::Error>>;
    fn process_user_event(
        &self,
        args: &dyn std::any::Any,
        state: &dyn std::any::Any,
        event: UserEvent,
    ) -> Result<(Box<dyn std::any::Any>, Box<dyn std::any::Any>), Box<dyn std::error::Error>>;
}

pub struct Registered<O>(std::marker::PhantomData<fn() -> O>);

impl<O: Operation> Registered<O> {
    pub fn boxed() -> Box<dyn ErasedOperation> {
        Box::new(Registered::<O>(std::marker::PhantomData))
    }
}

impl<O: Operation> ErasedOperation for Registered<O> {
    fn run(
        &self,
        id: &str,
        part: &mut Part,
        args: &dyn std::any::Any,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let args = args
            .downcast_ref::<O::Args>()
            .ok_or("Failed to downcast args")?;
        O::run(id, part, args)?;
        Ok(())
    }

    fn presentation(
        &self,
        args: &dyn std::any::Any,
        state: &dyn std::any::Any,
    ) -> Result<EditingPresentation, Box<dyn std::error::Error>> {
        let args = args
            .downcast_ref::<O::Args>()
            .ok_or("Failed to downcast args")?;
        let state = state
            .downcast_ref::<O::EditingState>()
            .ok_or("Failed to downcast state")?;
        Ok(O::presentation(args, state))
    }

    fn process_user_event(
        &self,
        args: &dyn std::any::Any,
        state: &dyn std::any::Any,
        event: UserEvent,
    ) -> Result<(Box<dyn std::any::Any>, Box<dyn std::any::Any>), Box<dyn std::error::Error>> {
        let args = args
            .downcast_ref::<O::Args>()
            .ok_or("Failed to downcast args")?;
        let state = state
            .downcast_ref::<O::EditingState>()
            .ok_or("Failed to downcast state")?;
        let (new_args, new_state) = O::process_user_event(args, state, event);
        Ok((Box::new(new_args), Box::new(new_state)))
    }
}
