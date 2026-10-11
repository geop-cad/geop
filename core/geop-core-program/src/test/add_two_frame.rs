use geop_core_math::geop_error::GeopResult;
use geop_core_math::vector::Vector3;
use serde::{Deserialize, Serialize};

use crate::editing::{Event, Form};
use crate::operation::{Context, Operation, OperationInfo};
use crate::part::Part;
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, info, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AddTwoFrameArgs {
    pub data1: Vector3<Sc>,
    pub data2: Vector3<Sc>,
}

/// Defines the targets `<step>_1` and `<step>_2` as frames at the two points.
pub struct AddTwoFrameOperation;

impl Operation for AddTwoFrameOperation {
    const INFO: OperationInfo = info("add_two_frame");
    type Args = AddTwoFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        for (suffix, data) in [("_1", &args.data1), ("_2", &args.data2)] {
            let reference = TargetReference::<Frame>::new(step.to_string() + suffix);
            let args = (label(reference.name(), step), *data);
            part.define_target(&reference, &args, |_, (label, data)| {
                record_built(label);
                Ok(Frame { data: *data })
            })?;
        }
        Ok(())
    }

    fn new_args(_: &Context<Sc, Pose>) -> Self::Args {
        Self::Args::default()
    }

    fn presentation(_: &Context<Sc, Pose>, _: &Self::Args, _: &()) -> Form<Sc> {
        Form::default()
    }

    fn process_user_event(
        _: &Context<Sc, Pose>,
        args: &Self::Args,
        _: &(),
        _: Event<Sc>,
    ) -> (Self::Args, ()) {
        (args.clone(), ())
    }
}
