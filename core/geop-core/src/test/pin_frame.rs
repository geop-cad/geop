use std::collections::BTreeMap;

use geop_core_math::{geop_error::GeopResult, vector::Vector};
use serde::{Deserialize, Serialize};

use crate::constraint::Constraint;
use crate::operation::{EditingPresentation, Operation, UserEvent};
use crate::part::{Part, View};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, no_presentation, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PinFrameArgs {
    pub frame: String,
    pub state: String,
}

/// Constrains the position of the pose `state`, its first three
/// components, to the frame `frame`.
pub struct PinFrameOperation;

impl Operation for PinFrameOperation {
    const NAME: &'static str = "pin_frame";
    type Args = PinFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        part.define_constraint(step, args, |part, args| {
            record_built(step, step);
            let frame = part.retrieve_target(&TargetReference::<Frame>::new(&args.frame))?;
            let pose = part.retrieve_state(&args.state);
            let position = pose.dual_quaternion().head::<3>();
            Ok(Constraint {
                residuals: position.sub(&frame.data).to_array().to_vec(),
                jacobian: BTreeMap::from([(
                    args.state.clone(),
                    (0..3).map(Vector::axis).collect(),
                )]),
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
