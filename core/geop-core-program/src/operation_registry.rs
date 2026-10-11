use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    operation::{ErasedOperation, Operation, OperationInfo},
    part::State,
};

/// The operations a program may use, by name.
pub struct OperationRegistry<I, S: State>(BTreeMap<&'static str, Box<dyn ErasedOperation<I, S>>>);

impl<I, S: State> Default for OperationRegistry<I, S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

impl<I, S: State> OperationRegistry<I, S> {
    /// Adds `operation`. Fails if an operation of its name is already there: a
    /// program could not tell the two apart.
    pub fn with<O: Operation<Input = I, State = S>>(mut self, operation: O) -> GeopResult<Self> {
        if self.0.contains_key(O::INFO.kind) {
            return Err(GeopError::new(format!(
                "operation `{}` is registered twice",
                O::INFO.kind
            )));
        }
        self.0.insert(O::INFO.kind, Box::new(operation));
        Ok(self)
    }

    /// What each operation is, in the order of their names.
    pub fn infos(&self) -> impl Iterator<Item = &'static OperationInfo> + '_ {
        self.0.values().map(|operation| operation.info())
    }

    pub fn get(&self, name: &str) -> GeopResult<&dyn ErasedOperation<I, S>> {
        self.0
            .get(name)
            .map(Box::as_ref)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{name}`")))
    }
}
