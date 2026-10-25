//! Walks over a cage's edges, for selecting and for the edits that follow
//! them: the *ring* across the quads either side of an edge, the *loop*
//! straight on through its vertices, and the boundary run of an open cage.

use std::collections::{BTreeSet, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::walk::{Walked, walk_by},
};

use crate::cage::{Cage, edge_key, undirected};

/// The edge ring of an edge: the edges reached from it by crossing quads
/// to their opposite side, until the ring closes, or meets the cage's
/// boundary or a face that is not a quad on both ends.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ring {
    /// Its edges in order, each directed so that the same side of the
    /// ring comes first: splitting every edge at a fraction `t` from its
    /// first vertex puts the new loop at `t` of the way across the ring.
    pub edges: Vec<(u32, u32)>,
    /// The quads crossed, by index, in the same order: the one between
    /// `edges[i]` and `edges[i + 1]` — and, for a closed ring, between the
    /// last edge and the first.
    pub quads: Vec<usize>,
    pub closed: bool,
}

impl Cage {
    /// The edge ring through the edge between `a` and `b` (see [`Ring`]).
    pub fn edge_ring(&self, a: u32, b: u32) -> GeopResult<Ring> {
        let directed = self.directed_edges();
        if !directed.contains_key(&(a, b)) && !directed.contains_key(&(b, a)) {
            return Err(GeopError::new(format!(
                "the cage has no edge {}",
                edge_key(a, b)
            )));
        }
        // Across each quad to the opposite edge, directed so that the side
        // of the entering edge's first vertex comes first again.
        let across = |(x, y): (u32, u32), taken: &[usize]| -> Option<((u32, u32), usize)> {
            let &(f, k) = directed.get(&(x, y))?;
            let face = &self.faces[f].vertices;
            if face.len() != 4 || taken.contains(&f) {
                return None;
            }
            Some(((face[(k + 3) % 4], face[(k + 2) % 4]), f))
        };
        // Forward from `(a, b)`.
        let mut quads = Vec::new();
        let forward = walk_by(
            (a, b),
            |&(x, y)| undirected(x, y),
            |&edge| {
                Ok::<_, GeopError>(across(edge, &quads).map(|(next, f)| {
                    quads.push(f);
                    next
                }))
            },
        )?;
        if forward.end == Walked::Closed {
            return Ok(Ring {
                edges: forward.nodes,
                quads,
                closed: true,
            });
        }
        // Then the other way from `(b, a)`, its edges turned round to keep
        // `a`'s side first, and put before the forward ones.
        let mut back_quads = Vec::new();
        let back = walk_by(
            (b, a),
            |&(x, y)| undirected(x, y),
            |&edge| {
                Ok::<_, GeopError>(across(edge, &quads).and_then(|(next, f)| {
                    if back_quads.contains(&f) {
                        return None;
                    }
                    back_quads.push(f);
                    Some(next)
                }))
            },
        )?;
        let mut edges: Vec<(u32, u32)> =
            back.nodes[1..].iter().rev().map(|&(x, y)| (y, x)).collect();
        edges.extend(forward.nodes);
        back_quads.reverse();
        back_quads.extend(quads);
        Ok(Ring {
            edges,
            quads: back_quads,
            closed: false,
        })
    }

    /// The edge loop through the edge between `a` and `b`: on through
    /// each vertex of four edges by the edge opposite — the one sharing no
    /// face with the edge arrived by — until the loop closes or reaches a
    /// vertex of any other number of edges, or the boundary. A boundary
    /// edge's loop is the boundary it is on (see [`Cage::boundary_run`]).
    pub fn edge_loop(&self, a: u32, b: u32) -> GeopResult<Vec<[u32; 2]>> {
        let directed = self.directed_edges();
        if !directed.contains_key(&(a, b)) && !directed.contains_key(&(b, a)) {
            return Err(GeopError::new(format!(
                "the cage has no edge {}",
                edge_key(a, b)
            )));
        }
        if self.is_boundary(a, b) {
            let run = self.boundary_run(a, b)?;
            let n = run.len();
            return Ok((0..n)
                .map(|i| undirected(run[i], run[(i + 1) % n]))
                .collect());
        }
        let mut neighbours: HashMap<u32, BTreeSet<u32>> = HashMap::new();
        for &(x, y) in directed.keys() {
            neighbours.entry(x).or_default().insert(y);
            neighbours.entry(y).or_default().insert(x);
        }
        let faces_of = |x: u32, y: u32| -> BTreeSet<usize> {
            [(x, y), (y, x)]
                .into_iter()
                .filter_map(|e| directed.get(&e).map(|&(f, _)| f))
                .collect()
        };
        // From `(from, to)`, the edge on from `to`.
        let onward = |(from, to): (u32, u32)| -> Option<(u32, u32)> {
            let around = &neighbours[&to];
            if around.len() != 4 {
                return None;
            }
            let shared = faces_of(from, to);
            let mut opposite = around
                .iter()
                .filter(|&&w| w != from && faces_of(to, w).is_disjoint(&shared));
            let w = *opposite.next()?;
            opposite.next().is_none().then_some((to, w))
        };
        let forward = walk_by(
            (a, b),
            |&(x, y)| undirected(x, y),
            |&e| Ok::<_, GeopError>(onward(e)),
        )?;
        let mut edges: Vec<[u32; 2]> = forward
            .nodes
            .iter()
            .map(|&(x, y)| undirected(x, y))
            .collect();
        if forward.end != Walked::Closed {
            let back = walk_by(
                (b, a),
                |&(x, y)| undirected(x, y),
                |&e| Ok::<_, GeopError>(onward(e)),
            )?;
            edges.extend(back.nodes[1..].iter().map(|&(x, y)| undirected(x, y)));
        }
        Ok(edges)
    }

    /// Whether only one face runs along the edge between `a` and `b`.
    pub fn is_boundary(&self, a: u32, b: u32) -> bool {
        let directed = self.directed_edges();
        directed.contains_key(&(a, b)) != directed.contains_key(&(b, a))
    }

    /// The boundary the open edge between `a` and `b` is on, as the
    /// vertices around it, in the order the faces along it run: a face
    /// closing it runs the other way. Fails for an edge that is not on the
    /// boundary, and for a boundary that touches itself at a vertex, by
    /// name.
    pub fn boundary_run(&self, a: u32, b: u32) -> GeopResult<Vec<u32>> {
        let directed = self.directed_edges();
        let open =
            |x: u32, y: u32| directed.contains_key(&(x, y)) && !directed.contains_key(&(y, x));
        let (a, b) = if open(a, b) {
            (a, b)
        } else if open(b, a) {
            (b, a)
        } else {
            return Err(GeopError::new(format!(
                "edge {} is not on the cage's boundary",
                edge_key(a, b)
            )));
        };
        let run = walk_by(
            (a, b),
            |&(x, _)| x,
            |&(_, y)| {
                let mut next = directed.keys().filter(|&&(x, z)| x == y && open(x, z));
                let Some(&(_, z)) = next.next() else {
                    return Ok(None);
                };
                if next.next().is_some() {
                    return Err(GeopError::new(format!(
                        "the boundary touches itself at vertex {}",
                        crate::cage::vertex_key(y)
                    )));
                }
                Ok(Some((y, z)))
            },
        )?;
        if run.end != Walked::Closed {
            return Err(GeopError::new(format!(
                "the boundary through edge {} does not close",
                edge_key(a, b)
            )));
        }
        Ok(run.nodes.into_iter().map(|(x, _)| x).collect())
    }
}
