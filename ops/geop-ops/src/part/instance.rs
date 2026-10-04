//! A [`Part`]'s instances: other parts placed in it, each at a pose — what
//! makes a part an assembly — and the mates that hold them together (see
//! [`crate::assembly`]). Named like any other entity.

use std::{
    collections::BTreeSet,
    sync::{
        Arc, OnceLock,
        atomic::{AtomicU64, Ordering},
    },
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
};

use super::Part;
use super::ids::InstanceId;
use crate::{assembly::Mate, ui::PartView};

/// What a program file builds, ready to be placed: the part, the file it
/// came from, and every file that went into it.
///
/// Shared — through an [`Arc`] — by every instance of it, and by every
/// rebuild that places the same file unchanged, so it is built once and
/// drawn once (see [`Component::view`]).
pub struct Component<S: Scalar> {
    /// Where its program is, as the library that built it names files.
    pub file: String,
    pub part: Part<S>,
    /// Its own file and every file it places, directly or through others:
    /// what placing it must not form a cycle with.
    pub files: BTreeSet<String>,
    /// Which build of `file` it is: unique among every component made.
    build: u64,
    view: OnceLock<PartView<S>>,
    bounds: OnceLock<Option<[Vector3<S>; 2]>>,
}

/// The build number the next component gets.
static NEXT_BUILD: AtomicU64 = AtomicU64::new(1);

impl<S: Scalar> Component<S> {
    pub fn new(file: String, part: Part<S>, files: BTreeSet<String>) -> Self {
        Self {
            file,
            part,
            files,
            build: NEXT_BUILD.fetch_add(1, Ordering::Relaxed),
            view: OnceLock::new(),
            bounds: OnceLock::new(),
        }
    }

    /// The box around every vertex of the part and of the parts placed in
    /// it, as placed — `None` for a part with none: found once, and kept
    /// for as long as the component is (see [`crate::assembly`]).
    pub fn bounds(&self) -> Option<[Vector3<S>; 2]> {
        *self
            .bounds
            .get_or_init(|| crate::assembly::bounds(&self.part))
    }

    /// What tells it apart from every other component, a rebuild of the
    /// same file included: `bolt.geop#7`. A viewer keeps a component's view
    /// by it, so that it is sent once however often it is drawn.
    pub fn key(&self) -> String {
        format!("{}#{}", self.file, self.build)
    }

    /// The part as drawn, in its own frame: rasterized once, the first time
    /// it is asked for, and kept for as long as the component is. Without
    /// its sketches: what a part was drawn with is its own business, not
    /// that of the parts it is placed in.
    pub fn view(&self) -> GeopResult<&PartView<S>> {
        if let Some(view) = self.view.get() {
            return Ok(view);
        }
        let mut view = PartView::of(&self.part)?;
        view.sketches.clear();
        Ok(self.view.get_or_init(|| view))
    }
}

/// A part placed in another: a [`Component`] at a [`Pose`] — every point
/// `p` of it at `pose.apply(p)` — the value of the parameter
/// `parameter` of the part it is placed in, which a solve of the mates may
/// change unless the instance is `fixed`. A copy of a pattern of placed
/// parts has no parameter: it goes where the pattern puts it.
///
/// Placed `flexible`, the parts placed in it are the part's to move too:
/// their poses are state of the part it is placed in, its component
/// built with those, and its mates solved with the part's (see
/// [`crate::assembly`]). Placed rigid, it moves as one body, its parts
/// where its own state puts them.
#[derive(Clone)]
pub struct Instance<S: Scalar> {
    pub component: Arc<Component<S>>,
    pub pose: Pose<S>,
    pub parameter: Option<String>,
    pub fixed: bool,
    pub flexible: bool,
}

impl<S: Scalar> Instance<S> {
    pub fn part(&self) -> &Part<S> {
        &self.component.part
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

    /// Every mate, by name.
    pub fn mates(&self) -> impl Iterator<Item = (&str, &Mate)> {
        self.mates.iter().map(|(name, m)| (name.as_str(), m))
    }
}
