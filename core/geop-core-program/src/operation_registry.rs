use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    operation::{ErasedOperation, Operation},
    part::State,
};

/// The operations a program may use, by name.
pub struct OperationRegistry<I, S: State>(BTreeMap<&'static str, Rc<dyn ErasedOperation<I, S>>>);

impl<I, S: State> Default for OperationRegistry<I, S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

impl<I, S: State> OperationRegistry<I, S> {
    /// Adds `operation`. Fails if an operation of its name is already there: a
    /// program could not tell the two apart.
    pub fn with<O: Operation<Input = I, State = S>>(self, operation: O) -> GeopResult<Self> {
        self.with_erased(Rc::new(operation))
    }

    /// Adds `operation`, its type erased (see [`OperationRegistry::with`]).
    pub fn with_erased(mut self, operation: Rc<dyn ErasedOperation<I, S>>) -> GeopResult<Self> {
        let kind = operation.kind();
        if self.0.contains_key(kind) {
            return Err(GeopError::new(format!(
                "operation `{kind}` is registered twice"
            )));
        }
        self.0.insert(kind, operation);
        Ok(self)
    }

    pub fn get(&self, name: &str) -> GeopResult<&dyn ErasedOperation<I, S>> {
        self.0
            .get(name)
            .map(Rc::as_ref)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{name}`")))
    }
}
