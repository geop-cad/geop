use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_math::vector::Vector3;
use serde::{Deserialize, Serialize};

use crate::operation::{EditingPresentation, Operation, UserEvent};
use crate::part::{Part, View};
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, no_presentation, record_built};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ShiftFrameArgs {
    pub targets: Vec<String>,
    pub by: Vector3<Sc>,
}

/// Updates each of the targets `targets` by moving its frame `by`. A click
/// moves the first to the origin instead.
pub struct ShiftFrameOperation;

impl Operation for ShiftFrameOperation {
    const NAME: &'static str = "shift_frame";
    type Args = ShiftFrameArgs;
    type EditingState = ();
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        for target in &args.targets {
            let reference = TargetReference::<Frame>::new(target);
            let args = (label(target, step), args.by);
            part.revise_target(&reference, &args, |frame, _, (label, by)| {
                record_built(label);
                Ok(Frame {
                    data: frame.data.add(by),
                })
            })?;
        }
        Ok(())
    }

    fn presentation(_: &View<Sc, Pose>, _: &Self::Args, _: &()) -> GeopResult<EditingPresentation> {
        Ok(no_presentation())
    }

    fn process_user_event(
        part: &View<Sc, Pose>,
        args: &Self::Args,
        _: &(),
        _: UserEvent,
    ) -> GeopResult<(Self::Args, ())> {
        let first = args
            .targets
            .first()
            .ok_or_else(|| GeopError::new("there is no target to move"))?;
        let frame = part.retrieve_target(&TargetReference::<Frame>::new(first))?;
        let args = ShiftFrameArgs {
            targets: args.targets.clone(),
            by: frame.data.neg(),
        };
        Ok((args, ()))
    }
}
