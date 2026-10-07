//! A [`Part`]'s instances: other parts placed in it, each at a pose — what
//! makes a part an assembly — and the mates that hold them together (see
//! [`crate::assembly`]). Named like any other entity.

use std::{collections::BTreeSet, sync::Arc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
};

use super::Part;
use super::ids::InstanceId;
use crate::{assembly::Mate, parameters::Parameter};

/// A part placed in another, at a [`Pose`] — every point `p` of it at
/// `pose.apply(p)` — the value of the pose parameter `parameter` of the
/// part it is placed in, which a solve of the mates may change unless a
/// fixed mate holds it (see [`crate::assembly`]). A copy of a pattern of
/// placed parts has no parameter: it goes where the pattern puts it.
///
/// The part is shared — through an [`Arc`] — by every instance of it, and
/// by every rebuild that places the same file unchanged, so it is built
/// once and drawn once (see [`Part::view`]). The parts placed in it are
/// the part's to move too: their poses are state of the part it is
/// placed in, the part built with those, and its mates solved with the
/// part's.
#[derive(Clone)]
pub struct Instance<S: Scalar> {
    pub part: Arc<Part<S>>,
    /// Where the program that built `part` is, as the library that built it
    /// names files.
    pub file: String,
    /// Its own file and every file it places, directly or through others:
    /// what placing it must not form a cycle with.
    pub files: BTreeSet<String>,
    pub pose: Pose<S>,
    pub parameter: Option<Parameter>,
}

impl<S: Scalar> Instance<S> {
    /// The part built from the program in `file`, which reads `files`, at
    /// the origin and no parameter: ready to be given a place.
    pub fn of(file: String, part: Part<S>, files: BTreeSet<String>) -> Self {
        Self {
            part: Arc::new(part),
            file,
            files,
            pose: Pose::identity(),
            parameter: None,
        }
    }

    /// What tells it apart from every other build of a file: `bolt.geop#7`.
    /// A viewer keeps a part's view by it, so that it is sent once however
    /// often it is drawn.
    pub fn key(&self) -> String {
        format!("{}#{}", self.file, self.part.revision())
    }
}

impl<S: Scalar> Part<S> {
    /// Places `instance` in the part under `name`. Fails, leaving the part
    /// unchanged, if `name` is already taken.
    pub fn add_instance(
        &mut self,
        instance: Instance<S>,
        name: impl Into<String>,
    ) -> GeopResult<InstanceId> {
        let id = InstanceId(self.fresh_id());
        self.names.insert(id, name)?;
        self.instances.insert(id, instance);
        Ok(id)
    }

    pub fn instance(&self, id: InstanceId) -> GeopResult<&Instance<S>> {
        self.instances
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("Part has no instance {id}")))
    }

    /// Every instance, in the order they were placed.
    pub fn instances(&self) -> impl Iterator<Item = (InstanceId, &Instance<S>)> {
        self.instances.iter().map(|(&id, i)| (id, i))
    }

    /// Adds `mate` under `name`: a mate is no entity — nothing is built on
    /// it — but its name keeps it apart from the other mates. Fails if the
    /// name is taken.
    pub fn add_mate(&mut self, mate: Mate, name: impl Into<String>) -> GeopResult<()> {
        let name = name.into();
        if self.mates.contains_key(&name) {
            return Err(GeopError::new(format!("Part already has a mate {name:?}")));
        }
        self.mates.insert(name, mate);
        Ok(())
    }

    /// Whether a fixed mate holds the part placed at the path `instance` —
    /// `bolt`, or `asm/bolt` for one placed in a part placed.
    pub fn is_fixed(&self, instance: &str) -> bool {
        self.mates().any(|(_, mate)| {
            mate.is_fixed()
                && mate
                    .entities
                    .iter()
                    .any(|e| e.instance_path().as_deref() == Some(instance))
        })
    }

    /// Every mate, by name.
    pub fn mates(&self) -> impl Iterator<Item = (&str, &Mate)> {
        self.mates.iter().map(|(name, m)| (name.as_str(), m))
    }
}
