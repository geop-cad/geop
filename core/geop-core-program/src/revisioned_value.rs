//! [`RevisionedValue`]: a value given to a part from outside, by name — an
//! input or a value of the state — with the run in which it last changed.

use std::collections::BTreeMap;

use crate::target::Revision;

/// A value given to a run from outside, and the run in which it last changed.
/// A value that is removed stays as `None` while anything depends on it, so
/// that what read it sees the change.
pub(crate) struct RevisionedValue<T> {
    pub(crate) value: Option<T>,
    pub(crate) changed_at: Revision,
}

impl<T: PartialEq + Clone> RevisionedValue<T> {
    /// Makes `given` hold exactly `values`, marking what differs as changed
    /// in `revision`: whether anything did.
    pub(crate) fn replace_all(
        given: &mut BTreeMap<String, Self>,
        values: &BTreeMap<String, T>,
        revision: Revision,
    ) -> bool {
        let mut changed = false;
        for (name, entry) in given.iter_mut() {
            let value = values.get(name);
            if entry.value.as_ref() != value {
                entry.value = value.cloned();
                entry.changed_at = revision;
                changed = true;
            }
        }
        for (name, value) in values {
            given.entry(name.clone()).or_insert_with(|| {
                changed = true;
                RevisionedValue {
                    value: Some(value.clone()),
                    changed_at: revision,
                }
            });
        }
        changed
    }
}

impl<T> RevisionedValue<T> {
    /// The run in which the value `name` of `given` last changed: 0, before
    /// the first run, for one never given.
    pub(crate) fn changed_at(given: &BTreeMap<String, Self>, name: &str) -> Revision {
        given.get(name).map_or(0, |entry| entry.changed_at)
    }
}
