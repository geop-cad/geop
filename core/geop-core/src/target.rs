use crate::part::Dependency;
use std::any::Any;
pub struct TargetId<T>(pub String, std::marker::PhantomData<fn() -> T>);

impl<T> TargetId<T> {
    pub fn new(id: String) -> Self {
        Self(id, std::marker::PhantomData)
    }
}

pub struct Target<Args, T> {
    pub args: Args,
    pub depends_on: Vec<Dependency>,
    pub data: Result<T, Box<dyn std::error::Error>>,
}

pub struct ErasedTarget {
    pub args: Box<dyn Any>,
    pub depends_on: Vec<Dependency>,
    pub data: Result<Box<dyn Any>, Box<dyn std::error::Error>>,
}

pub struct TargetRegistry {
    pub targets: std::collections::HashMap<String, ErasedTarget>,
}

impl TargetRegistry {
    pub fn new() -> Self {
        Self {
            targets: std::collections::HashMap::new(),
        }
    }

    pub fn upsert_target<A: 'static, T: 'static>(&mut self, id: TargetId<T>, target: Target<A, T>) {
        self.targets.insert(
            id.0,
            ErasedTarget {
                args: Box::new(target.args),
                depends_on: target.depends_on,
                data: target.data.map(|d| Box::new(d) as Box<dyn Any>),
            },
        );
    }

    pub fn retrieve_target_content<T: 'static>(
        &self,
        id: &TargetId<T>,
    ) -> Result<&T, Box<dyn std::error::Error>> {
        self.targets
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|data| {
                data.data
                    .as_ref()
                    .map_err(|e| format!("target `{}` failed: {e}", id.0))?
                    .downcast_ref::<T>()
                    .ok_or_else(|| "Failed to downcast target data".into())
            })
    }

    pub fn retrieve_target_dependencies<Args: 'static, T: 'static>(
        &self,
        id: &TargetId<T>,
    ) -> Result<Vec<Dependency>, Box<dyn std::error::Error>> {
        self.targets
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|data| Ok(data.depends_on.clone()))
    }

    pub fn retrieve_target_args<Args: 'static + Clone, T: 'static>(
        &self,
        id: &TargetId<T>,
    ) -> Result<Args, Box<dyn std::error::Error>> {
        self.targets
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|data| {
                data.args
                    .downcast_ref::<Args>()
                    .ok_or_else(|| "Failed to downcast target args".into())
                    .map(|args| args.clone())
            })
    }
}
