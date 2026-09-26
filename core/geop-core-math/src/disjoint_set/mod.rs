//! Accumulate candidate values found during a subdivision search into a
//! minimal set of merged, mutually-disjoint solutions.
//!
//! A geometric subdivision search (curve/curve, curve/surface, or a plain
//! point projection) converges on many small candidate values near each
//! genuine solution, not just one — [`DisjointSet::insert`] folds each new
//! candidate into whichever existing entries it `could_be_equal`,
//! transitively (a candidate can bridge two previously-separate entries
//! into one), so the result never contains two entries describing the same
//! physical solution.

use crate::scalars::Scalar;
use crate::vector::Vector;

/// A value that can be tested for approximate equality against another of
/// the same type and combined into the smallest value definitely
/// containing both — the building block [`DisjointSet`] merges on.
pub trait Mergeable: Copy {
    fn could_be_equal(&self, other: &Self) -> bool;
    fn union(&self, other: &Self) -> Self;
}

impl<S: Scalar> Mergeable for S {
    fn could_be_equal(&self, other: &Self) -> bool {
        Scalar::could_be_equal(*self, *other)
    }
    fn union(&self, other: &Self) -> Self {
        Scalar::union(*self, *other)
    }
}

impl<S: Scalar, const N: usize> Mergeable for Vector<S, N> {
    fn could_be_equal(&self, other: &Self) -> bool {
        Vector::could_be_equal(self, other)
    }
    fn union(&self, other: &Self) -> Self {
        Vector::union(self, other)
    }
}

impl<A: Mergeable, B: Mergeable> Mergeable for (A, B) {
    fn could_be_equal(&self, other: &Self) -> bool {
        self.0.could_be_equal(&other.0) && self.1.could_be_equal(&other.1)
    }
    fn union(&self, other: &Self) -> Self {
        (self.0.union(&other.0), self.1.union(&other.1))
    }
}

/// A set of mutually-disjoint (no two `could_be_equal`) merged values,
/// built up one candidate at a time via [`DisjointSet::insert`].
#[derive(Clone, Debug)]
pub struct DisjointSet<T: Mergeable> {
    items: Vec<T>,
}

impl<T: Mergeable> DisjointSet<T> {
    pub fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// The current number of disjoint entries.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.items.iter()
    }

    /// Fold `candidate` into this set: absorb every existing entry it
    /// `could_be_equal` (via `union`), repeating — not just once — since
    /// absorbing one entry can widen the merged result enough to now also
    /// `could_be_equal` a *different*, previously-distinct entry (e.g. a
    /// third candidate bridging two already-found ones, which a
    /// single non-repeating pass would leave as two separate entries
    /// instead of joining them into one). The fully-merged result takes the
    /// place of the earliest entry it absorbed (or is appended), so entries
    /// keep the order in which they were first found.
    pub fn insert(&mut self, mut candidate: T) {
        let mut at: Option<usize> = None;
        while let Some(i) = self
            .items
            .iter()
            .position(|item| item.could_be_equal(&candidate))
        {
            candidate = candidate.union(&self.items.remove(i));
            at = Some(at.map_or(i, |a| a.min(i)));
        }
        match at {
            Some(i) => self.items.insert(i, candidate),
            None => self.items.push(candidate),
        }
    }

    pub fn into_vec(self) -> Vec<T> {
        self.items
    }
}

impl<T: Mergeable> Default for DisjointSet<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::DisjointSet;
    use crate::{for_all_scalars, scalars::Scalar};

    fn check_disjoint_scalars_stay_separate<S: Scalar>() {
        let mut set = DisjointSet::new();
        set.insert(S::from_f64(0.1));
        set.insert(S::from_f64(0.9));
        assert_eq!(set.len(), 2);
    }
    #[test]
    fn disjoint_scalars_stay_separate() {
        for_all_scalars!(check_disjoint_scalars_stay_separate);
    }

    fn check_overlapping_scalars_merge<S: Scalar>() {
        let mut set = DisjointSet::new();
        set.insert(S::from_f64(0.5));
        set.insert(S::from_f64(0.5));
        assert_eq!(set.len(), 1);
    }
    #[test]
    fn overlapping_scalars_merge() {
        for_all_scalars!(check_overlapping_scalars_merge);
    }

    /// A third candidate bridging two previously-separate entries must
    /// merge all three into one, not just absorb into whichever entry it
    /// happened to match first.
    fn check_bridging_candidate_joins_two_solutions<S: Scalar>() {
        let mut set = DisjointSet::new();
        set.insert(S::from_f64(0.0));
        set.insert(S::from_f64(1.0));
        assert_eq!(set.len(), 2);

        // A very wide candidate spanning both — built as the union of two
        // points straddling each existing entry — should absorb both.
        let bridge = S::from_f64(-0.5).union(S::from_f64(1.5));
        set.insert(bridge);
        assert_eq!(
            set.len(),
            1,
            "a candidate overlapping both existing entries should merge them into one"
        );
    }
    #[test]
    fn bridging_candidate_joins_two_solutions() {
        for_all_scalars!(check_bridging_candidate_joins_two_solutions);
    }
}
