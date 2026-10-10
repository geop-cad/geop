use crate::operation::Operation;
use crate::target::TargetReference;
use crate::test::{Frame, record_built};

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
        id: &str,
        part: &mut crate::part::Part,
        args: &Self::Args,
    ) -> Result<(), Box<dyn std::error::Error>> {
        for (suffix, data) in [("_1", &args.data1), ("_2", &args.data2)] {
            let name = id.to_string() + suffix;
            part.define_target(
                TargetReference::<Frame>::new(name.clone()),
                data,
                |_, data| {
                    record_built(&name);
                    Ok(Frame { data: *data })
                },
            )?;
        }
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
