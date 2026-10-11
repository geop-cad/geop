//! [`ModelStep`]: a step that builds on the part's model, as most do — and
//! [`build_model`], which makes its revision of the model.

use geop_core_math::geop_error::GeopResult;

use geop_core_program::operation::Operation;

use super::{Build, BuildReader, Input};
use crate::{
    Design,
    model::{NamedModel, ParamValue, model},
};

/// An operation whose steps build on the part's [`model()`]: an extrude, a
/// boolean, a fillet. Its [`Operation::run`] is [`build_model`], with
/// whatever else the step records besides.
pub trait ModelStep: Operation<Input = Input, State = ParamValue, Args: PartialEq + Clone> {
    /// Builds the step `step` — what everything it makes is named after —
    /// into `model`, reading whatever else it builds on through `reader`
    /// (see [`super::Scope`]), so the step is rebuilt when that changes.
    fn build(
        model: &mut NamedModel<Design>,
        reader: &mut BuildReader,
        step: &str,
        args: &Self::Args,
    ) -> GeopResult<()>;
}

/// Makes the step `step`'s revision of the model: `O` building `args` into
/// the model as the steps before left it, or into an empty one, before the
/// first step that builds any.
pub fn build_model<O: ModelStep>(part: &mut Build, step: &str, args: &O::Args) -> GeopResult<()> {
    part.revise_or_start(&model(), &(step.to_string(), args.clone()), generate::<O>)
}

/// The generator of [`build_model`]: a function of `O` alone, so it
/// captures nothing — the step comes with the arguments.
fn generate<O: ModelStep>(
    mut model: NamedModel<Design>,
    reader: &mut BuildReader,
    (step, args): &(String, O::Args),
) -> GeopResult<NamedModel<Design>> {
    O::build(&mut model, reader, step, args)?;
    Ok(model)
}

/// What `O`'s step `step` with `args` makes of `model`, as a program of two
/// steps would build it — the model given, then the step — together with
/// what else the step made: the part, after its one run. For an
/// operation's tests, which build the model they act on directly.
pub fn apply<O>(model: NamedModel<Design>, step: &str, args: &O::Args) -> GeopResult<Build>
where
    O: Operation<Input = Input, State = ParamValue>,
{
    fn given(_: &mut BuildReader, model: &NamedModel<Design>) -> GeopResult<NamedModel<Design>> {
        Ok(model.clone())
    }
    let mut part = Build::default();
    part.begin_run(&Default::default(), &Default::default());
    part.begin_step("given")?;
    part.define_target(&crate::model::model(), &model, given)?;
    part.begin_step(step)?;
    O::run(step, &mut part, args)?;
    part.end_run(Default::default());
    Ok(part)
}

/// The model `O`'s step `step` with `args` leaves of `model` (see
/// [`apply`]): for the tests of an operation that builds on the model.
pub fn apply_to_model<O>(
    model: NamedModel<Design>,
    step: &str,
    args: &O::Args,
) -> GeopResult<NamedModel<Design>>
where
    O: Operation<Input = Input, State = ParamValue>,
{
    let part = apply::<O>(model, step, args)?;
    Ok(part
        .current()
        .retrieve_target(&crate::model::model())?
        .clone())
}
