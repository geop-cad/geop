use crate::operation::Operation;
use crate::target::TargetReference;
use crate::test::Frame;

#[derive(Debug, Clone, PartialEq)]
pub struct AddTwoFrameArgs {
    pub data1: [f64; 3],
    pub data2: [f64; 3],
}

pub struct AddTwoFrameOperation {}

impl Operation for AddTwoFrameOperation {
    type Args = AddTwoFrameArgs;

    type EditingState = ();

    fn run(
        id: &String,
        part: &mut crate::part::Part,
        args: &Self::Args,
    ) -> Result<(), Box<dyn std::error::Error>>
    where
        Self: Sized,
    {
        let frame_id1 = TargetReference::<Frame>::new(id.clone() + "_1");
        part.define_target(frame_id1, &args.data1, |_part, args| {
            let frame = Frame { data: args.clone() };
            println!("Created frame1: {:?}", frame);
            Ok(frame)
        })?;

        let frame_id2 = TargetReference::<Frame>::new(id.to_string() + "_2");
        part.define_target(frame_id2, &args.data2, |_part, args| {
            let frame = Frame { data: args.clone() };
            println!("Created frame2: {:?}", frame);
            Ok(frame)
        })?;
        Ok(())
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
