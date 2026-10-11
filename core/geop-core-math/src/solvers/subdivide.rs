//! Subdivision search: the global method that finds *every* solution in a
//! region by discarding the pieces that provably hold none and splitting
//! the rest (see "Subdivide to isolate, then Newton to refine" in
//! `AGENTS.md`).
//!
//! Which pieces hold no solution, when a piece is small enough to report,
//! and how to split one are the caller's, in `visit`; this is the search
//! around them. It promises completeness: every piece `visit` keeps is
//! visited, in a fixed order, unless the node budget runs out — and then
//! it says so, with how many pieces were left, rather than returning as if
//! the search were done. A search cut short is incomplete, and neither
//! "nothing found" nor anything else may be read into it.

use std::collections::VecDeque;

/// What a visit made of its piece.
pub enum Split<T> {
    /// Nothing more to search in it: it holds no solution, or the visit
    /// has recorded the one it holds.
    Done,
    /// The piece narrowed to where its solutions can be (a clip).
    Restrict(T),
    /// The piece cut in two, both to be searched.
    Halves(T, T),
}

/// In which order pieces are visited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Order {
    /// Level by level, each piece's halves after every piece already
    /// waiting: what keeps a budget spent evenly over the region.
    BreadthFirst,
    /// Each piece's first half, and all of it, before its second.
    DepthFirst,
}

/// How a search ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub enum Searched {
    /// Every piece was visited.
    Complete,
    /// The budget ran out with `pending` pieces left unvisited: the search
    /// is incomplete.
    Exhausted { pending: usize },
}

/// Searches `roots` — in their order — and every piece a visit restricts
/// or splits them into, visiting at most `max_nodes` pieces. `visit` gets a
/// piece and the budget left after it (for a search of its own) and says
/// what to do with it.
pub fn subdivide<T, E>(
    roots: impl IntoIterator<Item = T>,
    order: Order,
    max_nodes: usize,
    mut visit: impl FnMut(T, usize) -> Result<Split<T>, E>,
) -> Result<Searched, E> {
    let mut pending: VecDeque<T> = VecDeque::new();
    match order {
        Order::BreadthFirst => pending.extend(roots),
        Order::DepthFirst => roots.into_iter().for_each(|r| pending.push_front(r)),
    }
    let mut explored = 0usize;
    // Next at the front, either way: depth first keeps a stack there.
    while let Some(piece) = pending.pop_front() {
        if explored >= max_nodes {
            return Ok(Searched::Exhausted {
                pending: pending.len() + 1,
            });
        }
        explored += 1;
        match visit(piece, max_nodes - explored)? {
            Split::Done => {}
            Split::Restrict(restricted) => match order {
                Order::BreadthFirst => pending.push_back(restricted),
                Order::DepthFirst => pending.push_front(restricted),
            },
            Split::Halves(first, second) => match order {
                Order::BreadthFirst => pending.extend([first, second]),
                Order::DepthFirst => {
                    pending.push_front(second);
                    pending.push_front(first);
                }
            },
        }
    }
    Ok(Searched::Complete)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pieces of `[0, 1)` that a bisection to `1/16` keeps around
    /// `0.3` and `0.7`, in either order, and how many it visits.
    fn search(order: Order, max_nodes: usize) -> (Searched, Vec<(f64, f64)>) {
        let mut found = Vec::new();
        let searched = subdivide::<_, ()>([(0.0, 1.0)], order, max_nodes, |(lo, hi), _| {
            if ![0.3, 0.7].iter().any(|&r| lo <= r && r < hi) {
                return Ok(Split::Done);
            }
            if hi - lo <= 1.0 / 16.0 {
                found.push((lo, hi));
                return Ok(Split::Done);
            }
            let mid = 0.5 * (lo + hi);
            Ok(Split::Halves((lo, mid), (mid, hi)))
        })
        .unwrap();
        (searched, found)
    }

    #[test]
    fn every_solution_is_found_in_either_order() {
        let both = [(0.25, 0.3125), (0.6875, 0.75)];
        for order in [Order::BreadthFirst, Order::DepthFirst] {
            let (searched, found) = search(order, 1000);
            assert_eq!(searched, Searched::Complete);
            assert_eq!(found, both);
        }
    }

    /// Breadth first visits by level; depth first finishes the left half
    /// before the right.
    #[test]
    fn the_orders_differ_only_in_order() {
        let mut seen = [Vec::new(), Vec::new()];
        for (k, order) in [Order::BreadthFirst, Order::DepthFirst]
            .into_iter()
            .enumerate()
        {
            let _ = subdivide::<_, ()>([(0u32, 4u32)], order, 100, |(lo, hi), _| {
                seen[k].push((lo, hi));
                Ok(if hi - lo == 1 {
                    Split::Done
                } else {
                    let mid = (lo + hi) / 2;
                    Split::Halves((lo, mid), (mid, hi))
                })
            })
            .unwrap();
        }
        assert_eq!(
            seen[0],
            [(0, 4), (0, 2), (2, 4), (0, 1), (1, 2), (2, 3), (3, 4)]
        );
        assert_eq!(
            seen[1],
            [(0, 4), (0, 2), (0, 1), (1, 2), (2, 4), (2, 3), (3, 4)]
        );
    }

    /// A budget too small for the search reports it incomplete, never a
    /// result.
    #[test]
    fn running_out_is_reported() {
        let (searched, _) = search(Order::BreadthFirst, 5);
        assert_eq!(searched, Searched::Exhausted { pending: 4 });
    }
}
