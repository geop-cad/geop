//! A step's accesses to a part, noted when it runs, and replayed when it
//! need not (see [`cells`](super::cells)): what lets a runner skip a step
//! whose reads are as they were the last time it ran.

use std::sync::Arc;

use geop_core_math::scalars::Scalar;

use super::{
    Part,
    cells::{Access, Log},
};

impl<S: Scalar> Part<S> {
    /// Notes every access to the part, and to every copy made of it from
    /// now on, in `log`.
    pub(crate) fn record(&mut self, log: &Arc<Log>) {
        self.store.record(log.clone());
    }

    /// Stops noting accesses, and returns what `log` noted — unless the
    /// part is not a copy of the one recorded, and nothing is known of what
    /// made it.
    pub(crate) fn finish_recording(&mut self, log: &Arc<Log>) -> Option<Access> {
        let ours = self.store.is_recording(log);
        self.store.stop_recording();
        ours.then(|| log.take())
    }

    /// Whether a step that read `access` of `before` and wrote what it
    /// wrote would write the same when run on this part: whether each cell
    /// it read is as it was in `before`.
    pub(crate) fn would_write_the_same(&self, before: &Self, access: &Access) -> bool {
        access
            .reads
            .iter()
            .all(|cell| self.store.version(cell) == before.store.version(cell))
    }

    /// The part a step makes of this one, if it would write what it wrote
    /// when it turned `before` into `after`, with `access` (see
    /// [`Part::would_write_the_same`]). That is `after` itself, with what was
    /// worked out of it, if this part holds just what `before` did; else the cells the step wrote, taken
    /// from `after`.
    pub(crate) fn replayed(&self, before: &Self, after: &Arc<Self>, access: &Access) -> Arc<Self> {
        if self.store.same_versions(&before.store) {
            return after.clone();
        }
        let mut part = self.clone();
        part.store
            .replay(&before.store, &after.store, &access.writes);
        part.renew_revision();
        Arc::new(part)
    }
}
