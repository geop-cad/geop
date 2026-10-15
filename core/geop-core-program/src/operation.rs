use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::part::StepContext;

/// One kind of step. An operation is a unit struct: everything a step of
/// it needs is in its arguments.
pub trait Operation: 'static {
    /// The name programs store its steps under.
    const KIND: &'static str;
    /// What a step of the operation is given. It is saved with the program,
    /// so it names targets and elements, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + 'static;

    /// Declares what the step produces, through `ctx` (see
    /// [`StepContext::produce`]): cheap, since it runs in every run. What
    /// takes work is done in the makes, which run only when what they read
    /// changed.
    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()>;
}

/// Arguments that say which operation they are for: what a program written
/// in Rust — an example, a standard part, a test — pushes as a step without
/// naming the operation again.
pub trait StepArgs: Serialize {
    /// The kind of the operation they are for (see [`Operation::KIND`]).
    const KIND: &'static str;
}

/// An [`Operation`] with its types erased, so that operations of all kinds
/// can be kept in one [`crate::operation_registry::OperationRegistry`].
/// Every operation is one, through its value. Arguments come as saved.
pub trait ErasedOperation {
    fn kind(&self) -> &'static str;
    /// Whether `args` are arguments of the operation, as saved.
    fn check_args(&self, args: &serde_json::Value) -> GeopResult<()>;
    fn run(&self, ctx: &mut StepContext, args: &serde_json::Value) -> GeopResult<()>;
}

/// The arguments of `O` from their saved form.
pub fn args<O: Operation>(json: &serde_json::Value) -> GeopResult<O::Args> {
    O::Args::deserialize(json)
        .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::KIND)))
}

/// The saved form of `args`.
pub fn saved<O: Operation>(args: &O::Args) -> GeopResult<serde_json::Value> {
    serde_json::to_value(args)
        .map_err(|e| GeopError::new(format!("arguments of operation `{}`: {e}", O::KIND)))
}

impl<O: Operation> ErasedOperation for O {
    fn kind(&self) -> &'static str {
        O::KIND
    }

    fn check_args(&self, args: &serde_json::Value) -> GeopResult<()> {
        self::args::<O>(args).map(|_| ())
    }

    fn run(&self, ctx: &mut StepContext, args: &serde_json::Value) -> GeopResult<()> {
        <O as Operation>::run(ctx, &self::args::<O>(args)?)
    }
}
