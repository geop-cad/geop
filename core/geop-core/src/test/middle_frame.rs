use crate::operation::Operation;
use crate::target::TargetReference;
use crate::test::{Frame, record_built};

#[derive(Debug, Clone, PartialEq)]
pub struct MiddleFrameArgs {
    pub frame1ref: String,
    pub frame2ref: String,
}

pub struct MiddleFrameOperation {}

impl Operation for MiddleFrameOperation {
    type Args = MiddleFrameArgs;

    type EditingState = ();

    fn run(
        id: &str,
        part: &mut crate::part::Part,
        args: &Self::Args,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let frame_id = TargetReference::<Frame>::new(id.to_string());
        part.define_target(frame_id, args, |part, args| {
            record_built(id);
            let frame1id = TargetReference::<Frame>::new(args.frame1ref.clone());
            let frame2id = TargetReference::<Frame>::new(args.frame2ref.clone());
            let frame1 = part.retrieve_target(&frame1id)?;
            let frame2 = part.retrieve_target(&frame2id)?;
            Ok(Frame {
                data: [
                    (frame1.data[0] + frame2.data[0]) / 2.0,
                    (frame1.data[1] + frame2.data[1]) / 2.0,
                    (frame1.data[2] + frame2.data[2]) / 2.0,
                ],
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
