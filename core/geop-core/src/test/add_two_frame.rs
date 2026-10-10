use geop_core_math::geop_error::GeopResult;
use geop_core_math::vector::Vector3;
use serde::{Deserialize, Serialize};

use crate::operation::{EditingPresentation, Operation, UserEvent};
use crate::part::{Part, View};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, no_presentation, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AddTwoFrameArgs {
    pub data1: Vector3<Sc>,
    pub data2: Vector3<Sc>,
}

/// Defines the targets `<step>_1` and `<step>_2` as frames at the two points.
pub struct AddTwoFrameOperation;

impl Operation for AddTwoFrameOperation {
    const NAME: &'static str = "add_two_frame";
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

    fn presentation(_: &View<Sc, Pose>, _: &Self::Args, _: &()) -> GeopResult<EditingPresentation> {
        Ok(no_presentation())
    }

    fn process_user_event(
        _: &View<Sc, Pose>,
        args: &Self::Args,
        _: &(),
        _: UserEvent,
    ) -> GeopResult<(Self::Args, ())> {
        Ok((args.clone(), ()))
    }
}
