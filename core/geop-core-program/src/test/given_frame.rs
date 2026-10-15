use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{
    operation::Operation,
    part::Part,
    program_runner::ProgramRunner,
    target::TargetReference,
    test::{Frame, label, record_built},
    values::{Design, Setting, StateValue},
};

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

    fn run(step: &str, _: &ProgramRunner, part: &mut Part, args: &Self::Args) -> GeopResult<()> {
        let reference = TargetReference::<Frame>::new(&args.target);
        let args = (label(&args.target, step), args.clone());
        part.define_target(&reference, &args, |part, (label, args)| {
            record_built(label);
            let x = match part.retrieve_input(&args.input) {
                Some(Setting::Number(x)) => Design::from_f64(x),
                _ => {
                    return Err(GeopError::new(format!(
                        "input `{}` is not given",
                        args.input
                    )));
                }
            };
            Ok(Frame {
                data: Vector3::from_array([x, Design::ZERO, Design::ZERO]),
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

    fn run(step: &str, _: &ProgramRunner, part: &mut Part, args: &Self::Args) -> GeopResult<()> {
        let args = (label(step, step), args.clone());
        part.define_target(
            &TargetReference::<Frame>::new(step),
            &args,
            |part, (label, args)| {
                record_built(label);
                let data = match part.retrieve_state(&args.state) {
                    StateValue::Pose(pose) => pose.position(),
                    StateValue::Number(_) => Vector3::zero(),
                };
                Ok(Frame { data })
            },
        )
    }
}
