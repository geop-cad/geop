use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::operation::{EditingPresentation, Operation, UserEvent};
use crate::part::{Part, View};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, no_presentation, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InputFrameArgs {
    pub target: String,
    pub input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
pub struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const NAME: &'static str = "input_frame";
    type Args = InputFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let reference = TargetReference::<Frame>::new(&args.target);
        part.define_target(&reference, args, |part, args| {
            record_built(&args.target, step);
            let x = part
                .retrieve_input(&args.input)
                .ok_or_else(|| GeopError::new(format!("input `{}` is not given", args.input)))?;
            Ok(Frame {
                data: Vector3::from_array([x, Sc::ZERO, Sc::ZERO]),
            })
        })
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StateFrameArgs {
    pub state: String,
}

/// Defines the step's target as the frame at the first three components of
/// the state `state`.
pub struct StateFrameOperation;

impl Operation for StateFrameOperation {
    const NAME: &'static str = "state_frame";
    type Args = StateFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        part.define_target(&TargetReference::<Frame>::new(step), args, |part, args| {
            record_built(step, step);
            let pose = part.retrieve_state(&args.state);
            Ok(Frame {
                data: pose.dual_quaternion().head::<3>(),
            })
        })
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
