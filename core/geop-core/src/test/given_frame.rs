use crate::operation::Operation;
use crate::target::TargetReference;
use crate::test::{Frame, record_built};

/// Defines the target `target` as the frame at `(input, 0, 0)`.
#[derive(Debug, Clone, PartialEq)]
pub struct InputFrameArgs {
    pub target: String,
    pub input: String,
}

pub struct InputFrameOperation {}

impl Operation for InputFrameOperation {
    type Args = InputFrameArgs;

    type EditingState = ();

    fn run(
        _id: &str,
        part: &mut crate::part::Part,
        args: &Self::Args,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let frame_id = TargetReference::<Frame>::new(args.target.clone());
        part.define_target(frame_id, args, |part, args| {
            record_built(&args.target);
            let x = part
                .retrieve_input(&args.input)
                .ok_or_else(|| format!("input `{}` is not given", args.input))?;
            Ok(Frame {
                data: [x, 0.0, 0.0],
            })
        })
    }

    fn presentation(
        _args: &Self::Args,
        _state: &Self::EditingState,
    ) -> crate::operation::EditingPresentation {
        todo!()
    }

    fn process_user_event(
        _args: &Self::Args,
        _state: &Self::EditingState,
        _event: crate::operation::UserEvent,
    ) -> (Self::Args, Self::EditingState) {
        todo!()
    }
}

/// Defines the step's target as the frame at the first three components of
/// the state `state`.
#[derive(Debug, Clone, PartialEq)]
pub struct StateFrameArgs {
    pub state: String,
}

pub struct StateFrameOperation {}

impl Operation for StateFrameOperation {
    type Args = StateFrameArgs;

    type EditingState = ();

    fn run(
        id: &str,
        part: &mut crate::part::Part,
        args: &Self::Args,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let frame_id = TargetReference::<Frame>::new(id.to_string());
        part.define_target(frame_id, args, |part, args| {
            record_built(id);
            let pose = part.retrieve_state(&args.state);
            let [x, y, z, ..] = *pose.dual_quaternion();
            Ok(Frame { data: [x, y, z] })
        })
    }

    fn presentation(
        _args: &Self::Args,
        _state: &Self::EditingState,
    ) -> crate::operation::EditingPresentation {
        todo!()
    }

    fn process_user_event(
        _args: &Self::Args,
        _state: &Self::EditingState,
        _event: crate::operation::UserEvent,
    ) -> (Self::Args, Self::EditingState) {
        todo!()
    }
}
