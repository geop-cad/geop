//! [`SharedMap`]: a hash map that is cheap to copy, for what a part keeps
//! for every cell of it and a runner copies for every step.
//!
//! A copy of a part per step of a program is a copy of all it holds. A map
//! of hundreds of entries copied a hundred times is slow only because it is
//! copied whole, though a step changes a few entries of it. So the map is
//! cut in [`SHARDS`] by the hash of the key, each shard behind an `Arc`: a
//! copy shares them all, and a write copies the one shard it falls in —
//! a sixty-fourth of the map.

use std::{
    collections::HashMap,
    hash::{DefaultHasher, Hash, Hasher},
    sync::Arc,
};

/// How many parts a map is cut in.
const SHARDS: usize = 64;

/// A hash map of which a copy costs a sixty-fourth of a write (see the
/// module). No order: for finding a key, not listing keys.
#[derive(Clone)]
pub(super) struct SharedMap<K, V> {
    shards: Arc<[Arc<HashMap<K, V>>; SHARDS]>,
}

impl<K: Hash + Eq + Clone, V: Clone + PartialEq> SharedMap<K, V> {
    pub(super) fn new() -> Self {
        Self {
            shards: Arc::new(std::array::from_fn(|_| Arc::new(HashMap::new()))),
        }
    }

    /// The shard `key` falls in: the same, in every build and run.
    fn shard(key: &K) -> usize {
        let mut hasher = DefaultHasher::new();
        key.hash(&mut hasher);
        (hasher.finish() % SHARDS as u64) as usize
    }

    pub(super) fn get(&self, key: &K) -> Option<&V> {
        self.shards[Self::shard(key)].get(key)
    }

    pub(super) fn insert(&mut self, key: K, value: V) {
        let shard = Self::shard(&key);
        let shards = Arc::make_mut(&mut self.shards);
        Arc::make_mut(&mut shards[shard]).insert(key, value);
    }

    /// Whether the two hold the same entries: shards they share are the
    /// same without a look.
    pub(super) fn same_as(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.shards, &other.shards)
            || self
                .shards
                .iter()
                .zip(other.shards.iter())
                .all(|(a, b)| Arc::ptr_eq(a, b) || a == b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A copy is independent of what it was copied from, and shares what
    /// neither changed.
    #[test]
    fn a_copy_shares_what_is_not_written() {
        let mut a = SharedMap::<String, u64>::new();
        for k in 0..500 {
            a.insert(format!("cell{k}"), k);
        }
        let mut b = a.clone();
        assert!(a.same_as(&b));
        b.insert("cell7".into(), 70);

        assert_eq!(a.get(&"cell7".to_string()), Some(&7));
        assert_eq!(b.get(&"cell7".to_string()), Some(&70));
        assert!(!a.same_as(&b));
        let shared = a
            .shards
            .iter()
            .zip(b.shards.iter())
            .filter(|(x, y)| Arc::ptr_eq(x, y))
            .count();
        assert_eq!(shared, SHARDS - 1, "one shard was copied for one write");
    }
}
