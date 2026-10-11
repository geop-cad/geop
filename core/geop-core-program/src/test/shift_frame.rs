use geop_core_math::geop_error::GeopResult;
use geop_core_math::vector::Vector3;
use serde::{Deserialize, Serialize};

use crate::editing::{Event, Form};
use crate::operation::{Context, Operation, OperationInfo};
use crate::part::Part;
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, info, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ShiftFrameArgs {
    pub targets: Vec<String>,
    pub by: Vector3<Sc>,
}

/// Updates each of the targets `targets` by moving its frame `by`. A click
/// moves the first to the origin instead.
pub struct ShiftFrameOperation;

impl Operation for ShiftFrameOperation {
    const INFO: OperationInfo = info("shift_frame");
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

    fn new_args(_: &Context<Sc, Pose>) -> Self::Args {
        Self::Args::default()
    }

    fn presentation(_: &Context<Sc, Pose>, _: &Self::Args, _: &()) -> Form<Sc> {
        Form::default()
    }

    /// Pressing the field `origin` moves the first target to the origin.
    fn process_user_event(
        context: &Context<Sc, Pose>,
        args: &Self::Args,
        _: &(),
        event: Event<Sc>,
    ) -> (Self::Args, ()) {
        let to_origin = match event {
            Event::Field { key, .. } if key == "origin" => args.targets.first().and_then(|first| {
                let reference = TargetReference::<Frame>::new(first);
                context.before.retrieve_target(&reference).ok()
            }),
            _ => None,
        };
        let by = to_origin.map_or(args.by, |frame| frame.data.neg());
        let args = ShiftFrameArgs {
            targets: args.targets.clone(),
            by,
        };
        (args, ())
    }
}
