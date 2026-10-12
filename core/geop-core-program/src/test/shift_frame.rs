use geop_core_math::geop_error::GeopResult;
use geop_core_math::vector::Vector3;
use serde::{Deserialize, Serialize};

use crate::operation::Operation;
use crate::part::Part;
use crate::target::TargetReference;
use crate::test::{Frame, Pose, Sc, label, record_built};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ShiftFrameArgs {
    pub targets: Vec<String>,
    pub by: Vector3<Sc>,
}

/// Updates each of the targets `targets` by moving its frame `by`. A click
/// moves the first to the origin instead.
pub struct ShiftFrameOperation;

impl Operation for ShiftFrameOperation {
    const KIND: &'static str = "shift_frame";
    type Args = ShiftFrameArgs;
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        for target in &args.targets {
            let reference = TargetReference::<Frame>::new(target);
            let args = (label(target, step), args.by);
            part.revise_target(&reference, &args, |reader, (label, by)| {
                record_built(label);
                let frame: Frame = reader.previous()?;
                Ok(Frame {
                    data: frame.data.add(by),
                })
            })?;
        }
        Ok(())
    }
}
