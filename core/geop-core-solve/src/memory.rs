//! What was worked out of an independent group of bodies, remembered by
//! the group (see [`crate::mates::Assembly::freedom`]).
//!
//! Asking how free the parts of a robot are, after a drag, asks it of every
//! group of bodies no mate ties to another — and a drag moves the bodies of
//! one. What a group's freedom is follows from the group alone: its bodies,
//! where they are, and the mates between them. So it is looked up by those,
//! as `Debug` writes them, every digit of every enclosure: two groups are
//! the same only if they are written the same, and none are ever confused.
//! A group not seen before is worked out and remembered.
//!
//! Small groups are not remembered — they cost less to work out than to
//! find — and what is remembered is bounded, forgotten whole when it grows
//! past [`LIMIT`]: how much is kept, never what is found.

use std::{
    any::{Any, TypeId},
    collections::HashMap,
    fmt::Debug,
    sync::{LazyLock, Mutex, PoisonError},
};

/// The most groups remembered, of one kind of scalar.
const LIMIT: usize = 256;

/// What is remembered, by kind of scalar, then by how the group is written.
static MEMORY: LazyLock<Mutex<HashMap<TypeId, Box<dyn Any + Send>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// What was found of `group` before, if it was: `K` is the kind of scalar
/// the group is of, which tells groups of one kind from another's.
pub(crate) fn recall<K: 'static, R: Clone + Send + 'static>(group: &impl Debug) -> Option<R> {
    let memory = MEMORY.lock().unwrap_or_else(PoisonError::into_inner);
    memory
        .get(&TypeId::of::<K>())?
        .downcast_ref::<HashMap<String, R>>()?
        .get(&format!("{group:?}"))
        .cloned()
}

/// Remembers `found` of `group`.
pub(crate) fn remember<K: 'static, R: Clone + Send + 'static>(group: &impl Debug, found: R) {
    let mut memory = MEMORY.lock().unwrap_or_else(PoisonError::into_inner);
    let entry = memory
        .entry(TypeId::of::<K>())
        .or_insert_with(|| Box::new(HashMap::<String, R>::new()));
    let Some(table) = entry.downcast_mut::<HashMap<String, R>>() else {
        return;
    };
    if table.len() >= LIMIT {
        table.clear();
    }
    table.insert(format!("{group:?}"), found);
}
