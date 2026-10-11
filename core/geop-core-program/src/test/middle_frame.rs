use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use serde::{Deserialize, Serialize};

use crate::operation::{EditingPresentation, Operation, UserEvent};
use crate::part::{Part, View};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, no_presentation, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MiddleFrameArgs {
    pub frame1ref: String,
    pub frame2ref: String,
}

/// Defines the step's target as the frame halfway between two others.
pub struct MiddleFrameOperation;

impl Operation for MiddleFrameOperation {
    const NAME: &'static str = "middle_frame";
    type Args = MiddleFrameArgs;
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
                let frame1 =
                    part.retrieve_target(&TargetReference::<Frame>::new(&args.frame1ref))?;
                let frame2 =
                    part.retrieve_target(&TargetReference::<Frame>::new(&args.frame2ref))?;
                let half = Sc::from_ratio(1, 2)?;
                Ok(Frame {
                    data: frame1.data.add(&frame2.data).prod_scalar(half),
                })
            },
        )
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
