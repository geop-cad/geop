use geop_core_math::{geop_error::GeopResult, vector::Vector3};
use serde::{Deserialize, Serialize};

use crate::{
    operation::Operation,
    part::Part,
    program_runner::ProgramRunner,
    target::TargetReference,
    test::{Frame, label, record_built},
    values::Design,
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ShiftFrameArgs {
    pub targets: Vec<String>,
    pub by: Vector3<Design>,
}

/// Updates each of the targets `targets` by moving its frame `by`. A click
/// moves the first to the origin instead.
pub struct ShiftFrameOperation;

impl Operation for ShiftFrameOperation {
    const KIND: &'static str = "shift_frame";
    type Args = ShiftFrameArgs;

    fn run(step: &str, _: &ProgramRunner, part: &mut Part, args: &Self::Args) -> GeopResult<()> {
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
