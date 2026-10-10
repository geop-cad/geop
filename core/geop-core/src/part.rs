use std::{
    cell::RefCell,
    collections::{BTreeMap, HashSet},
};

use crate::target::{ErasedTargetId, Target, TargetId, TargetReference, TargetRegistry};

// Default
#[derive(Default, Clone, PartialEq)]
pub struct Pose {
    dual_quaternion: [f64; 8],
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Dependency {
    Target(ErasedTargetId),
    Input(String),
    State(String),
}

pub struct Part {
    targets: TargetRegistry,
    inputs: BTreeMap<String, f64>, // inputs will only return value if defined. They are defined before a program and not affected by it's exectuion.
    state: BTreeMap<String, Box<Pose>>, // when accessing state that doesn't exist, it will be created automatically and cached for future executions. State is driven by the operations applied to the part, and constraints.
    residuals: BTreeMap<String, Vec<f64>>, // Error in any constraints TODO: Make this Scalar trait.
    jacobian_state: BTreeMap<String, Vec<Vec<[f64; 6]>>>, // How each value in state affects the residuals // TODO: Make this Scalar trait.
    recorded_dependencies: RefCell<Vec<Dependency>>,
    changed_dependencies: HashSet<Dependency>,
}

impl Part {
    pub fn new() -> Self {
        Self {
            targets: TargetRegistry::new(),
            inputs: BTreeMap::new(),
            state: BTreeMap::new(),
            residuals: BTreeMap::new(),
            jacobian_state: BTreeMap::new(),
            recorded_dependencies: RefCell::new(Vec::new()),
            changed_dependencies: HashSet::new(),
        }
    }

    fn start_recording(&self) {
        self.recorded_dependencies.borrow_mut().clear();
    }

    fn stop_recording(&self) -> Vec<Dependency> {
        self.recorded_dependencies.borrow_mut().drain(..).collect()
    }

    pub fn retrieve_target<T: 'static>(
        &self,
        id: &TargetReference<T>,
    ) -> Result<&T, Box<dyn std::error::Error>> {
        let (id, data) = self.targets.retrieve_target_by_reference::<T>(id)?;
        self.recorded_dependencies
            .borrow_mut()
            .push(Dependency::Target(id.erased_id()));
        Ok(data)
    }

    pub fn retrieve_input(&self, id: &str) -> Option<f64> {
        self.recorded_dependencies
            .borrow_mut()
            .push(Dependency::Input(id.to_string()));
        self.inputs.get(id).copied()
    }

    pub fn retrieve_state(&mut self, id: &str) -> Pose {
        self.recorded_dependencies
            .borrow_mut()
            .push(Dependency::State(id.to_string()));
        // rust btreemap get or insert
        self.state.entry(id.to_string()).or_default();
        self.state
            .get(id)
            .map(|boxed_pose| boxed_pose.as_ref())
            .unwrap()
            .clone()
    }

    pub(crate) fn clear_changed_dependencies(&mut self) {
        self.changed_dependencies.clear();
    }

    pub fn update_input(&mut self, id: &str, value: f64) {
        // only update if the value has changed
        if let Some(current_value) = self.inputs.get(id) {
            if *current_value == value {
                return;
            }
        }

        self.inputs.insert(id.to_string(), value);
        self.changed_dependencies
            .insert(Dependency::Input(id.to_string()));
    }

    pub fn update_state(&mut self, id: &str, value: Pose) {
        // only update if the value has changed
        if let Some(current_value) = self.state.get(id) {
            if **current_value == value {
                return;
            }
        }

        self.state.insert(id.to_string(), Box::new(value));
        self.changed_dependencies
            .insert(Dependency::State(id.to_string()));
    }

    pub fn define_target<Out: 'static, Args: 'static + PartialEq + Clone>(
        &mut self,
        id: TargetReference<Out>,
        args: &Args,
        generator: impl Fn(&Part, &Args) -> Result<Out, Box<dyn std::error::Error>>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let deps_lookup = self.targets.retrieve_target_dependencies::<Args, Out>(&id);
        let args_lookup = self.targets.retrieve_target_args::<Args, Out>(&id);

        match (deps_lookup, args_lookup) {
            (Ok(deps), Ok(existing_args)) => {
                // check if any dependencies have changed
                if !deps
                    .iter()
                    .any(|dep| self.changed_dependencies.contains(dep))
                    && existing_args == *args
                {
                    return Ok(());
                }
            }
            _ => {}
        };

        self.start_recording();
        let data = generator(self, args)?;
        let dependencies = self.stop_recording();
        // insert or update
        let id = self
            .targets
            .upsert_target::<Args, Out>(id, Target::new(args.clone(), dependencies, Ok(data)));
        self.changed_dependencies
            .insert(Dependency::Target(id.erased_id()));
        Ok(())
    }

    pub fn get_residuals(&self) -> &BTreeMap<String, Vec<f64>> {
        &self.residuals
    }

    pub fn get_jacobian_state(&self) -> &BTreeMap<String, Vec<Vec<[f64; 6]>>> {
        &self.jacobian_state
    }
}
