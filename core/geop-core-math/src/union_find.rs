//! [`UnionFind`]: which of `n` things have been joined into one group —
//! connected faces of a shell, the cells a split cuts a solid into.

/// Groups of the indices `0..n`, merged by [`UnionFind::union`].
#[derive(Clone, Debug)]
pub struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    /// `n` indices, each in a group of its own.
    pub fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    /// The representative of `i`'s group: the same for every member.
    pub fn find(&mut self, i: usize) -> usize {
        let mut root = i;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        let mut at = i;
        while self.parent[at] != root {
            at = std::mem::replace(&mut self.parent[at], root);
        }
        root
    }

    /// Merges the groups of `a` and `b`.
    pub fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.parent[a.max(b)] = a.min(b);
        }
    }

    /// Every group, each in increasing order, ordered by its smallest
    /// member.
    pub fn groups(&mut self) -> Vec<Vec<usize>> {
        let mut by_root: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
        for i in 0..self.parent.len() {
            let root = self.find(i);
            by_root.entry(root).or_default().push(i);
        }
        let mut groups: Vec<Vec<usize>> = by_root.into_values().collect();
        groups.sort_by_key(|g| g[0]);
        groups
    }
}

#[cfg(test)]
mod tests {
    use super::UnionFind;

    #[test]
    fn groups_follow_unions() {
        let mut sets = UnionFind::new(6);
        sets.union(4, 1);
        sets.union(2, 5);
        sets.union(5, 4);
        assert_eq!(sets.groups(), vec![vec![0], vec![1, 2, 4, 5], vec![3]]);
        assert_eq!(sets.find(2), sets.find(1));
    }
}
