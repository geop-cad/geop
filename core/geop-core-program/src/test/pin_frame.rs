use std::collections::BTreeMap;

use geop_core_math::geop_error::GeopResult;
use serde::{Deserialize, Serialize};

use crate::constraint::{Bound, Constraint};
use crate::editing::{Event, Form};
use crate::operation::{Context, Operation, OperationInfo};
use crate::part::{Part, Retrieve};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, identity, info, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PinFrameArgs {
    pub frame: String,
    pub state: String,
}

/// Constrains the position of the pose `state`, its first three
/// components, to the frame `frame`.
pub struct PinFrameOperation;

impl Operation for PinFrameOperation {
    const INFO: OperationInfo = info("pin_frame");
    type Args = PinFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let args = (label(step, step), args.clone());
        part.define_constraint(step, &args, |part, (label, args)| {
            record_built(label);
            let frame = part.retrieve_target(&TargetReference::<Frame>::new(&args.frame))?;
            let pose = part.retrieve_state(&args.state);
            Ok(Constraint {
                residuals: pose.position.sub(&frame.data).to_array().to_vec(),
                jacobian: BTreeMap::from([(args.state.clone(), identity(3))]),
                bound: Bound::Zero,
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
