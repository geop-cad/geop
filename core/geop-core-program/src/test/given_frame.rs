use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::operation::Operation;
use crate::part::Part;
use crate::program_runner::ProgramRunner;
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct InputFrameArgs {
    pub target: String,
    pub input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
pub struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const KIND: &'static str = "input_frame";
    type Args = InputFrameArgs;
    type Input = Sc;
    type State = Pose;

    fn run(
        step: &str,
        _: &ProgramRunner<Sc, Pose>,
        part: &mut Part<Sc, Pose>,
        args: &Self::Args,
    ) -> GeopResult<()> {
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
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct StateFrameArgs {
    pub state: String,
}

/// Defines the step's target as the frame at the first three components of
/// the state `state`.
pub struct StateFrameOperation;

impl Operation for StateFrameOperation {
    const KIND: &'static str = "state_frame";
    type Args = StateFrameArgs;
    type Input = Sc;
    type State = Pose;

    fn run(
        step: &str,
        _: &ProgramRunner<Sc, Pose>,
        part: &mut Part<Sc, Pose>,
        args: &Self::Args,
    ) -> GeopResult<()> {
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
}
