use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::editing::{Event, Form};
use crate::operation::{Context, Operation, OperationInfo};
use crate::part::{Part, Retrieve};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, info, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct InputFrameArgs {
    pub target: String,
    pub input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
pub struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const INFO: OperationInfo = info("input_frame");
    type Args = InputFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let reference = TargetReference::<Frame>::new(&args.target);
        let args = (label(&args.target, step), args.clone());
        part.define_target(&reference, &args, |part, (label, args)| {
            record_built(label);
            let x = part
                .retrieve_input(&args.input)
                .ok_or_else(|| GeopError::new(format!("input `{}` is not given", args.input)))?;
            Ok(Frame {
                data: Vector3::from_array([x, Sc::ZERO, Sc::ZERO]),
            })
        })
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

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct StateFrameArgs {
    pub state: String,
}

/// Defines the step's target as the frame at the first three components of
/// the state `state`.
pub struct StateFrameOperation;

impl Operation for StateFrameOperation {
    const INFO: OperationInfo = info("state_frame");
    type Args = StateFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let args = (label(step, step), args.clone());
        part.define_target(
            &TargetReference::<Frame>::new(step),
            &args,
            |part, (label, args)| {
                record_built(label);
                let pose = part.retrieve_state(&args.state);
                Ok(Frame {
                    data: pose.position,
                })
            },
        )
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
