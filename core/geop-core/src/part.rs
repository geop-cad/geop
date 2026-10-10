use std::collections::BTreeMap;

use crate::target::{Target, TargetKey, TargetReference, TargetRegistry};

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Pose {
    dual_quaternion: [f64; 8],
}

impl Pose {
    pub fn new(dual_quaternion: [f64; 8]) -> Self {
        Self { dual_quaternion }
    }

    pub fn dual_quaternion(&self) -> &[f64; 8] {
        &self.dual_quaternion
    }
}

/// Counts the runs of a part, starting at 1. Revision 0 is before the first
/// run: what was never set has not changed since then.
pub type Revision = u64;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Dependency {
    Target(TargetKey),
    Input(String),
    State(String),
}

/// A value given to a run from outside, and the run in which it last changed.
/// A value that is removed stays as `None`, so that what read it sees the
/// change.
struct Given<T> {
    value: Option<T>,
    changed_at: Revision,
}

/// Makes `given` hold exactly `values`, marking what differs as changed in
/// `revision`.
fn replace_given<T: PartialEq + Clone>(
    given: &mut BTreeMap<String, Given<T>>,
    values: &BTreeMap<String, T>,
    revision: Revision,
) {
    for (name, entry) in given.iter_mut() {
        let value = values.get(name);
        if entry.value.as_ref() != value {
            entry.value = value.cloned();
            entry.changed_at = revision;
        }
    }
    for (name, value) in values {
        given.entry(name.clone()).or_insert_with(|| Given {
            value: Some(value.clone()),
            changed_at: revision,
        });
    }
}

fn changed_at<T>(given: &BTreeMap<String, Given<T>>, name: &str) -> Revision {
    given.get(name).map_or(0, |entry| entry.changed_at)
}

/// What a program builds, and what it builds from. Each run states its
/// inputs and state in full; targets are defined by the steps of the run,
/// in order, and a step only sees targets defined before it in the same run.
/// A target is reused from an earlier run when its arguments are equal and
/// nothing it read has changed since it was last verified.
#[derive(Default)]
pub struct Part {
    revision: Revision,
    step: String,
    targets: TargetRegistry,
    inputs: BTreeMap<String, Given<f64>>,
    state: BTreeMap<String, Given<Pose>>,
    residuals: BTreeMap<String, Vec<f64>>, // Error in any constraints TODO: Make this Scalar trait.
    jacobian_state: BTreeMap<String, Vec<Vec<[f64; 6]>>>, // How each value in state affects the residuals // TODO: Make this Scalar trait.
}

impl Part {
    pub(crate) fn begin_run(
        &mut self,
        inputs: &BTreeMap<String, f64>,
        state: &BTreeMap<String, Pose>,
    ) {
        self.revision += 1;
        replace_given(&mut self.inputs, inputs, self.revision);
        replace_given(&mut self.state, state, self.revision);
    }

    pub(crate) fn begin_step(&mut self, step: &str) {
        self.step = step.to_string();
    }

    /// Deletes the targets no step of this run defined.
    pub(crate) fn end_run(&mut self) {
        let revision = self.revision;
        self.targets.retain(|target| target.verified_at == revision);
    }

    fn lookup_target<T: 'static>(
        &self,
        reference: &TargetReference<T>,
    ) -> Result<(TargetKey, &T), Box<dyn std::error::Error>> {
        let name = reference.name();
        let (key, target) = self
            .targets
            .get(name)
            .filter(|(_, target)| target.verified_at == self.revision)
            .ok_or_else(|| format!("target `{name}` is not defined before step `{}`", self.step))?;
        let data = target
            .data
            .as_ref()
            .map_err(|e| format!("target `{name}` failed: {e}"))?;
        let data = data
            .downcast_ref::<T>()
            .ok_or_else(|| format!("target `{name}` is not a `{}`", std::any::type_name::<T>()))?;
        Ok((key, data))
    }

    pub fn retrieve_target<T: 'static>(
        &self,
        reference: &TargetReference<T>,
    ) -> Result<&T, Box<dyn std::error::Error>> {
        Ok(self.lookup_target(reference)?.1)
    }

    /// The input's value, if this run gives one.
    pub fn retrieve_input(&self, name: &str) -> Option<f64> {
        self.inputs.get(name).and_then(|entry| entry.value)
    }

    /// The state's value; the default pose if this run gives none.
    pub fn retrieve_state(&self, name: &str) -> Pose {
        self.state
            .get(name)
            .and_then(|entry| entry.value.clone())
            .unwrap_or_default()
    }

    /// Whether `dependency` is unchanged since `revision`. A target counts as
    /// changed unless it is defined, earlier in this run, from data built no
    /// later than `revision`.
    fn unchanged_since(&self, dependency: &Dependency, revision: Revision) -> bool {
        match dependency {
            Dependency::Target(key) => self.targets.get_by_key(*key).is_some_and(|target| {
                target.verified_at == self.revision && target.changed_at <= revision
            }),
            Dependency::Input(name) => changed_at(&self.inputs, name) <= revision,
            Dependency::State(name) => changed_at(&self.state, name) <= revision,
        }
    }

    /// Defines the target `reference` as what `generator` makes of `args`.
    /// Reuses what an earlier run built if `args` are equal and nothing the
    /// generator read then has changed since. A failure is stored, so that
    /// readers report it, but never reused.
    pub fn define_target<Out: 'static, Args: 'static + PartialEq + Clone>(
        &mut self,
        reference: TargetReference<Out>,
        args: &Args,
        generator: impl FnOnce(&mut Reader, &Args) -> Result<Out, Box<dyn std::error::Error>>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let name = reference.name();
        if let Some((_, existing)) = self.targets.get(name) {
            if existing.verified_at == self.revision {
                return Err(format!(
                    "target `{name}` is defined twice: by step `{}` and by step `{}`",
                    existing.step, self.step
                )
                .into());
            }
            let reusable = existing.data.as_ref().is_ok_and(|data| data.is::<Out>())
                && existing.args.downcast_ref::<Args>() == Some(args)
                && existing
                    .dependencies
                    .iter()
                    .all(|dependency| self.unchanged_since(dependency, existing.verified_at));
            if reusable {
                let (revision, step) = (self.revision, self.step.clone());
                let existing = self.targets.get_mut(name).unwrap();
                existing.verified_at = revision;
                existing.step = step;
                return Ok(());
            }
        }

        let mut reader = Reader {
            part: self,
            dependencies: Vec::new(),
        };
        let data = generator(&mut reader, args);
        let mut dependencies = reader.dependencies;
        dependencies.sort();
        dependencies.dedup();

        let error = data
            .as_ref()
            .err()
            .map(|e| format!("target `{name}` failed: {e}"));
        self.targets.insert(
            name,
            Target {
                step: self.step.clone(),
                args: Box::new(args.clone()),
                dependencies,
                data: data.map(|data| Box::new(data) as Box<dyn std::any::Any>),
                changed_at: self.revision,
                verified_at: self.revision,
            },
        );
        match error {
            Some(error) => Err(error.into()),
            None => Ok(()),
        }
    }

    pub fn get_residuals(&self) -> &BTreeMap<String, Vec<f64>> {
        &self.residuals
    }

    pub fn get_jacobian_state(&self) -> &BTreeMap<String, Vec<Vec<[f64; 6]>>> {
        &self.jacobian_state
    }
}

/// What a target's generator reads the part through. It records each read,
/// so that the target is rebuilt when any of it changes.
pub struct Reader<'a> {
    part: &'a Part,
    dependencies: Vec<Dependency>,
}

impl<'a> Reader<'a> {
    pub fn retrieve_target<T: 'static>(
        &mut self,
        reference: &TargetReference<T>,
    ) -> Result<&'a T, Box<dyn std::error::Error>> {
        let (key, data) = self.part.lookup_target(reference)?;
        self.dependencies.push(Dependency::Target(key));
        Ok(data)
    }

    pub fn retrieve_input(&mut self, name: &str) -> Option<f64> {
        self.dependencies.push(Dependency::Input(name.to_string()));
        self.part.retrieve_input(name)
    }

    pub fn retrieve_state(&mut self, name: &str) -> Pose {
        self.dependencies.push(Dependency::State(name.to_string()));
        self.part.retrieve_state(name)
    }
}
