use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Deserialize, Serialize};

use crate::element::ElementReference;
use crate::operation::Operation;
use crate::part::Part;
use crate::program_runner::ProgramRunner;
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MiddleFrameArgs {
    pub frame1: ElementReference<Frame, Frame>,
    pub frame2: ElementReference<Frame, Frame>,
}

/// Defines the step's target as the frame halfway between two others, as
/// each is in its own part.
pub struct MiddleFrameOperation;

impl Operation for MiddleFrameOperation {
    const KIND: &'static str = "middle_frame";
    type Args = MiddleFrameArgs;
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
                let frame1 = part.retrieve_placed(&args.frame1)?.element;
                let frame2 = part.retrieve_placed(&args.frame2)?.element;
                let half = Sc::from_ratio(1, 2)?;
                Ok(Frame {
                    data: frame1.data.add(&frame2.data).prod_scalar(half),
                })
            },
        )
    }
}
