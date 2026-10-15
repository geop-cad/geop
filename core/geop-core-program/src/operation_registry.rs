use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::operation::{ErasedOperation, Operation};

/// The operations a program may use, by name.
#[derive(Default)]
pub struct OperationRegistry(BTreeMap<&'static str, Rc<dyn ErasedOperation>>);

impl OperationRegistry {
    /// Adds `operation`. Fails if an operation of its name is already there: a
    /// program could not tell the two apart.
    pub fn with<O: Operation>(self, operation: O) -> GeopResult<Self> {
        self.with_erased(Rc::new(operation))
    }

    /// Adds `operation`, its type erased (see [`OperationRegistry::with`]).
    pub fn with_erased(mut self, operation: Rc<dyn ErasedOperation>) -> GeopResult<Self> {
        let kind = operation.kind();
        if self.0.contains_key(kind) {
            return Err(GeopError::new(format!(
                "operation `{kind}` is registered twice"
            )));
        }
        self.0.insert(kind, operation);
        Ok(self)
    }

    pub fn get(&self, name: &str) -> GeopResult<&dyn ErasedOperation> {
        self.0
            .get(name)
            .map(Rc::as_ref)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{name}`")))
    }
}
