//! Walking a chain: from a start, on to the one next node, and the one
//! after that, until the chain closes, ends, or runs into itself — a loop
//! of coedges followed round its face, a tangent chain of edges, the arcs
//! of one circle, a sketch's curves joined end to end, a pattern's joints
//! driving each other.
//!
//! What comes next is the caller's (`next`, which may refuse with an error
//! where the chain forks); the walk promises to tell the three ways a chain
//! can end apart, and to end: no node is walked twice, so a walk over
//! finitely many nodes ends after at most that many steps, whatever `next`
//! does. A chain that runs into itself somewhere other than its start is
//! reported as such, never followed round again.
//!
//! [`flood`] is the walk of a graph rather than a chain: from some nodes to
//! every node reachable from them, each visited once.

use std::{
    collections::{HashSet, VecDeque},
    hash::Hash,
};

/// How a walk ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Walked<N> {
    /// `next` led back to the start: the chain is closed.
    Closed,
    /// `next` said there is nothing after the last node: the chain is open.
    Open,
    /// `next` led to this node, walked before and not the start: the chain
    /// runs into a loop not through where it started.
    Revisited(N),
}

/// A walk: its nodes in order, the start first, and how it ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Walk<N> {
    pub nodes: Vec<N>,
    pub end: Walked<N>,
}

/// Walks from `start` along `next`: the node after each, or `None` where
/// there is none. Nodes are told apart by `key`: a node with the key of the
/// start closes the chain, one with the key of a node walked since runs
/// into it. `next` is asked once for each node, in order, so it may keep
/// state of its own.
pub fn walk_by<N, K: Eq + Hash, E>(
    start: N,
    key: impl Fn(&N) -> K,
    mut next: impl FnMut(&N) -> Result<Option<N>, E>,
) -> Result<Walk<N>, E> {
    let first = key(&start);
    let mut seen = HashSet::new();
    let mut nodes = vec![start];
    loop {
        let Some(after) = next(nodes.last().expect("never empty"))? else {
            return Ok(Walk {
                nodes,
                end: Walked::Open,
            });
        };
        let k = key(&after);
        if k == first {
            return Ok(Walk {
                nodes,
                end: Walked::Closed,
            });
        }
        if !seen.insert(k) {
            return Ok(Walk {
                nodes,
                end: Walked::Revisited(after),
            });
        }
        nodes.push(after);
    }
}

/// [`walk_by`] for nodes that are their own keys.
pub fn walk<N: Clone + Eq + Hash, E>(
    start: N,
    next: impl FnMut(&N) -> Result<Option<N>, E>,
) -> Result<Walk<N>, E> {
    walk_by(start, N::clone, next)
}

/// Which pending node a [`flood`] visits next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Take {
    /// The one handed on last: depth first, a stack.
    Newest,
    /// The one handed on first: breadth first, level by level, a queue.
    Oldest,
}

/// Visits `starts` and every node reachable from them, each once: `visit`
/// is asked for each node in turn, and hands on its neighbours, in order;
/// a node not handed on before joins the pending ones, and which of those
/// comes next `take` says. Returns the nodes in the order visited.
///
/// The order is the starts', `visit`'s and `take`'s alone: what has been
/// handed on is only ever asked about, never iterated. A flood over
/// finitely many nodes ends, whatever `visit` hands on. `visit` may keep
/// state of its own (what it found, what it refused), since each node is
/// visited once, in order.
pub fn flood<N: Clone + Eq + Hash, E>(
    starts: impl IntoIterator<Item = N>,
    take: Take,
    mut visit: impl FnMut(&N) -> Result<Vec<N>, E>,
) -> Result<Vec<N>, E> {
    let mut seen = HashSet::new();
    let mut pending = VecDeque::new();
    for start in starts {
        if seen.insert(start.clone()) {
            pending.push_back(start);
        }
    }
    let mut visited = Vec::new();
    while let Some(node) = match take {
        Take::Newest => pending.pop_back(),
        Take::Oldest => pending.pop_front(),
    } {
        for next in visit(&node)? {
            if seen.insert(next.clone()) {
                pending.push_back(next);
            }
        }
        visited.push(node);
    }
    Ok(visited)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Following `next` in a list of successors: `None` for a node with
    /// none.
    fn follow(successor: &[Option<usize>], start: usize) -> Walk<usize> {
        walk::<_, ()>(start, |&n| Ok(successor[n])).unwrap()
    }

    #[test]
    fn a_ring_closes() {
        let w = follow(&[Some(1), Some(2), Some(0)], 1);
        assert_eq!((w.nodes, w.end), (vec![1, 2, 0], Walked::Closed));
    }

    #[test]
    fn a_chain_ends() {
        let w = follow(&[Some(1), Some(2), None], 0);
        assert_eq!((w.nodes, w.end), (vec![0, 1, 2], Walked::Open));
    }

    /// A chain running into a loop not through its start is reported where
    /// it does, not followed round the loop for ever.
    #[test]
    fn running_into_a_loop_is_reported() {
        let w = follow(&[Some(1), Some(2), Some(3), Some(1)], 0);
        assert_eq!((w.nodes, w.end), (vec![0, 1, 2, 3], Walked::Revisited(1)));
    }

    /// Nodes are told apart by their keys: a node with the start's key
    /// closes the chain whatever else it carries, and `next` is asked once
    /// per node.
    #[test]
    fn nodes_are_told_apart_by_key() {
        let mut asked = 0;
        let w = walk_by::<_, _, ()>(
            (0, 'a'),
            |&(k, _)| k,
            |&(k, _)| {
                asked += 1;
                Ok(Some(((k + 1) % 3, 'b')))
            },
        )
        .unwrap();
        assert_eq!(w.nodes, [(0, 'a'), (1, 'b'), (2, 'b')]);
        assert_eq!((w.end, asked), (Walked::Closed, 3));
    }

    /// The graph 0-1, 0-2, 1-3, 2-3, and 4 on its own.
    fn neighbours(n: &usize) -> Result<Vec<usize>, ()> {
        Ok(match n {
            0 => vec![1, 2],
            1 => vec![0, 3],
            2 => vec![0, 3],
            3 => vec![1, 2],
            _ => vec![],
        })
    }

    /// Every node reachable is visited once, in the order `take` says: a
    /// queue level by level, a stack the newest first.
    #[test]
    fn a_flood_visits_what_is_reachable_once() {
        assert_eq!(flood([0], Take::Oldest, neighbours), Ok(vec![0, 1, 2, 3]));
        assert_eq!(flood([0], Take::Newest, neighbours), Ok(vec![0, 2, 3, 1]));
        assert_eq!(
            flood([4, 0, 4], Take::Oldest, neighbours),
            Ok(vec![4, 0, 1, 2, 3])
        );
    }

    /// An error from a visit ends the flood with it.
    #[test]
    fn an_error_ends_the_flood() {
        let out = flood([0usize], Take::Oldest, |&n| {
            if n == 2 {
                Err("no")
            } else {
                neighbours(&n).map_err(|_| "")
            }
        });
        assert_eq!(out, Err("no"));
    }

    /// An error from `next` ends the walk with it.
    #[test]
    fn a_fork_refused_by_next_ends_the_walk() {
        let w = walk::<usize, &str>(0, |&n| if n < 2 { Ok(Some(n + 1)) } else { Err("forks") });
        assert_eq!(w, Err("forks"));
    }
}
