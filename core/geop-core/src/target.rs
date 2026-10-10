use crate::part::Dependency;
use std::any::Any;
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TargetReference<T>(String, std::marker::PhantomData<fn() -> T>);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TargetId<T>(usize, std::marker::PhantomData<fn() -> T>);

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ErasedTargetId(usize, std::marker::PhantomData<fn() -> ()>);

impl<T> TargetReference<T> {
    pub fn new(id: String) -> Self {
        Self(id, std::marker::PhantomData)
    }
}

impl<T> TargetId<T> {
    pub fn new(id: usize) -> Self {
        Self(id, std::marker::PhantomData)
    }

    pub fn erased_id(&self) -> ErasedTargetId {
        ErasedTargetId(self.0, std::marker::PhantomData)
    }
}

pub struct Target<Args, T> {
    args: Args,
    depends_on: Vec<Dependency>,
    data: Result<T, Box<dyn std::error::Error>>,
}

impl<Args, T> Target<Args, T> {
    pub fn new(
        args: Args,
        depends_on: Vec<Dependency>,
        data: Result<T, Box<dyn std::error::Error>>,
    ) -> Self {
        Self {
            args,
            depends_on,
            data,
        }
    }
}

pub struct ErasedTarget {
    args: Box<dyn Any>,
    depends_on: Vec<Dependency>,
    data: Result<Box<dyn Any>, Box<dyn std::error::Error>>,
}

pub struct TargetRegistry {
    targets: Vec<ErasedTarget>,
    target_references: std::collections::HashMap<String, usize>,
}

impl TargetRegistry {
    pub fn new() -> Self {
        Self {
            targets: Vec::new(),
            target_references: std::collections::HashMap::new(),
        }
    }

    pub fn upsert_target<A: 'static, T: 'static>(
        &mut self,
        id: TargetReference<T>,
        target: Target<A, T>,
    ) -> TargetId<T> {
        // first check if the target already exists
        if let Some(&index) = self.target_references.get(&id.0) {
            self.targets[index] = ErasedTarget {
                args: Box::new(target.args),
                depends_on: target.depends_on,
                data: target.data.map(|d| Box::new(d) as Box<dyn Any>),
            };
            return TargetId(index, std::marker::PhantomData);
        }

        let index = self.targets.len();
        self.targets.push(ErasedTarget {
            args: Box::new(target.args),
            depends_on: target.depends_on,
            data: target.data.map(|d| Box::new(d) as Box<dyn Any>),
        });
        self.target_references.insert(id.0, index);
        TargetId(index, std::marker::PhantomData)
    }

    pub fn retrieve_target_by_reference<T: 'static>(
        &self,
        id: &TargetReference<T>,
    ) -> Result<(TargetId<T>, &T), Box<dyn std::error::Error>> {
        self.target_references
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|&index| {
                let data = &self.targets[index];
                data.data
                    .as_ref()
                    .map_err(|e| format!("target `{}` failed: {e}", id.0))?
                    .downcast_ref::<T>()
                    .ok_or_else(|| "Failed to downcast target data".into())
                    .map(|d| (TargetId(index, std::marker::PhantomData), d))
            })
    }

    pub fn retrieve_target_by_id<T: 'static>(
        &self,
        id: &TargetId<T>,
    ) -> Result<(TargetId<T>, &T), Box<dyn std::error::Error>> {
        let index = id.0;
        let data = &self.targets[index];
        data.data
            .as_ref()
            .map_err(|e| format!("target `{index}` failed: {e}"))?
            .downcast_ref::<T>()
            .ok_or_else(|| "Failed to downcast target data".into())
            .map(|d| (TargetId(index, std::marker::PhantomData), d))
    }

    pub fn retrieve_target_dependencies<Args: 'static, T: 'static>(
        &self,
        id: &TargetReference<T>,
    ) -> Result<Vec<Dependency>, Box<dyn std::error::Error>> {
        self.target_references
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|&index| Ok(self.targets[index].depends_on.clone()))
    }

    pub fn retrieve_target_args<Args: 'static + Clone, T: 'static>(
        &self,
        id: &TargetReference<T>,
    ) -> Result<Args, Box<dyn std::error::Error>> {
        self.target_references
            .get(&id.0)
            .ok_or_else(|| "Target not found or failed to retrieve data".into())
            .and_then(|&index| {
                let data = &self.targets[index];
                data.args
                    .downcast_ref::<Args>()
                    .ok_or_else(|| "Failed to downcast target args".into())
                    .map(|args| args.clone())
            })
    }
}
