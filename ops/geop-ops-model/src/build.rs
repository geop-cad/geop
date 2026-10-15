//! [`ModelOperation`]: a step that builds into the part's model, as most
//! steps do, and [`build_model`], its run: the step's revision of the
//! model.

use geop_core_editor::part_operation::PartOperation;
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{part::StepContext, reader::Reader, values::Design};

use crate::{NamedModel, model};

/// A kind of step that builds into the part's model: an extrude, a
/// boolean, a fillet. Its run is [`build_model`].
pub trait ModelOperation: PartOperation {
    /// Builds the step `step` — what everything it makes is named after —
    /// into `model`, reading whatever else it builds on through `reader`,
    /// so the step is rebuilt when that changes.
    fn build(
        model: &mut NamedModel<Design>,
        reader: &Reader,
        step: &str,
        args: &Self::Args,
    ) -> GeopResult<()>;
}

/// Produces the step's node of the model, a chain: `O` building `args` into
/// the model as the steps before left it, or into an empty one, before the
/// first step that builds any. What fails is said in the context of the
/// step and its arguments.
pub fn build_model<O: ModelOperation>(ctx: &mut StepContext, args: &O::Args) -> GeopResult<()> {
    ctx.produce(
        &model(),
        &(ctx.step().to_string(), args.clone()),
        generate::<O>,
    )
}

/// The make of [`build_model`]: a function of `O` alone, so it captures
/// nothing — the step comes with the arguments.
fn generate<O: ModelOperation>(
    reader: &Reader,
    (step, args): &(String, O::Args),
) -> GeopResult<NamedModel<Design>> {
    let mut model: NamedModel<Design> = reader.previous()?;
    O::build(&mut model, reader, step, args)
        .map_err(|e| e.with_context(format!("{}({step}, {args:?})", O::KIND)))?;
    Ok(model)
}
