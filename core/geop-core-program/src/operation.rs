use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize, de::DeserializeOwned};

use crate::part::{self, Part};

/// One kind of step. An operation is a unit struct: everything a step of
/// it needs is in its arguments.
pub trait Operation: 'static {
    /// The name programs store its steps under.
    const KIND: &'static str;
    /// What a step of the operation is given. It is saved with the program,
    /// so it names targets and elements, never holds their internal ids.
    type Args: Serialize + DeserializeOwned + 'static;
    /// The inputs and state of the parts the operation builds.
    type Input: part::Input;
    type State: part::State;

    fn run(
        step: &str,
        part: &mut Part<Self::Input, Self::State>,
        args: &Self::Args,
    ) -> GeopResult<()>;
}

/// An [`Operation`] with its types erased, so that operations of all kinds
/// can be kept in one [`crate::operation_registry::OperationRegistry`].
/// Every operation is one, through its value. Arguments come as saved.
pub trait ErasedOperation<I, S: part::State> {
    fn kind(&self) -> &'static str;
    fn run(&self, step: &str, part: &mut Part<I, S>, args: &serde_json::Value) -> GeopResult<()>;
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

impl<O: Operation> ErasedOperation<O::Input, O::State> for O {
    fn kind(&self) -> &'static str {
        O::KIND
    }

    fn run(
        &self,
        step: &str,
        part: &mut Part<O::Input, O::State>,
        args: &serde_json::Value,
    ) -> GeopResult<()> {
        <O as Operation>::run(step, part, &self::args::<O>(args)?)
    }
}
