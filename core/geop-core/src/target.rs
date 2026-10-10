use crate::part::{Dependency, Revision};
use slotmap::SlotMap;
use std::{any::Any, collections::HashMap};

/// The stable name of a target, as programs and arguments refer to it.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TargetReference<T>(String, std::marker::PhantomData<fn() -> T>);

impl<T> TargetReference<T> {
    pub fn new(name: String) -> Self {
        Self(name, std::marker::PhantomData)
    }

    pub fn name(&self) -> &str {
        &self.0
    }
}

slotmap::new_key_type! {
    /// A target's key within one registry. It stays the same while the target
    /// is rebuilt under its name, and finds nothing once the target is
    /// deleted. Only valid within the part that issued it, so never store it
    /// in arguments: those name targets by [`TargetReference`].
    pub struct TargetKey;
}

pub struct Target {
    /// The step that defined the target, for messages.
    pub step: String,
    pub args: Box<dyn Any>,
    pub dependencies: Vec<Dependency>,
    pub data: Result<Box<dyn Any>, Box<dyn std::error::Error>>,
    /// The run in which `data` was last built.
    pub changed_at: Revision,
    /// The run in which the target was last defined, rebuilt or reused.
    pub verified_at: Revision,
}

#[derive(Default)]
pub struct TargetRegistry {
    targets: SlotMap<TargetKey, Target>,
    names: HashMap<String, TargetKey>,
}

impl TargetRegistry {
    pub fn get(&self, name: &str) -> Option<(TargetKey, &Target)> {
        let key = *self.names.get(name)?;
        Some((key, &self.targets[key]))
    }

    pub fn get_mut(&mut self, name: &str) -> Option<&mut Target> {
        let key = *self.names.get(name)?;
        Some(&mut self.targets[key])
    }

    pub fn get_by_key(&self, key: TargetKey) -> Option<&Target> {
        self.targets.get(key)
    }

    /// Stores `target` under `name`, in place of any target already there,
    /// whose key it keeps.
    pub fn insert(&mut self, name: &str, target: Target) {
        match self.names.get(name) {
            Some(&key) => self.targets[key] = target,
            None => {
                let key = self.targets.insert(target);
                self.names.insert(name.to_string(), key);
            }
        }
    }

    pub fn retain(&mut self, keep: impl Fn(&Target) -> bool) {
        self.targets.retain(|_, target| keep(target));
        self.names.retain(|_, key| self.targets.contains_key(*key));
    }
}
