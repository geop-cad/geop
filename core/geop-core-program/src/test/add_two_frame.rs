use geop_core_math::{geop_error::GeopResult, vector::Vector3};
use serde::{Deserialize, Serialize};

use crate::{
    operation::Operation,
    part::StepContext,
    target::TargetReference,
    test::{Frame, label, record_built},
    values::Design,
};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AddTwoFrameArgs {
    pub data1: Vector3<Design>,
    pub data2: Vector3<Design>,
}

/// Defines the targets `<step>_1` and `<step>_2` as frames at the two points.
pub struct AddTwoFrameOperation;

impl Operation for AddTwoFrameOperation {
    const KIND: &'static str = "add_two_frame";
    type Args = AddTwoFrameArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        for (suffix, data) in [("_1", &args.data1), ("_2", &args.data2)] {
            let reference = TargetReference::<Frame>::new(step.to_string() + suffix);
            let args = (label(reference.name(), step), *data);
            ctx.produce(&reference, &args, |_, (label, data)| {
                record_built(label);
                Ok(Frame { data: *data })
            })?;
        }
        Ok(())
    }
}
