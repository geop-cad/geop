use std::collections::{BTreeMap, HashSet};

use geop_core_math::geop_error::{GeopError, GeopResult};

use super::ids::{EdgeId, VertexId, WireId};
use crate::{Model, Sense};

/// Edges and vertices standing on their own, bounding no face: curves in
/// space, such as the lines, arcs and splines of a 3-D sketch, or where two
/// bodies' boundaries cross.
///
/// A wire owns its entities as a shell owns its faces: `edges` are used by
/// no coedge, and `vertices` are every vertex of the wire — the ends of its
/// edges, and points on their own, which no edge ends at. An edge or a
/// vertex belongs to one wire at most, and a vertex of a wire to nothing
/// else.
///
/// A wire is no [`crate::Body`]: nothing is cut or combined with it. Its
/// edges must not cross each other except at a shared vertex, as the edges
/// of a body must not (see [`crate::validation`]).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Wire {
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
}

impl Model {
    /// The edges of the wire `wire` in chains: each a run of edges, in
    /// order, each starting where the one before ends, with the sense it
    /// runs in along its chain — open between two vertices where the run
    /// does not go on through (an end, or a branch), or closed. Ordered by
    /// the smallest edge id each holds, which follows the order the edges
    /// were made in.
    pub fn wire_chains(&self, wire: WireId) -> GeopResult<Vec<Vec<(EdgeId, Sense)>>> {
        let wire = (self.wires.get(&wire))
            .ok_or_else(|| GeopError::new(format!("Model::wire_chains: no wire {wire}")))?;
        let mut at: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
        for &e in &wire.edges {
            let edge = self.get_edge(e)?;
            at.entry(edge.start_vertex).or_default().push(e);
            at.entry(edge.end_vertex).or_default().push(e);
        }
        // The other edge of a run through `v` from `edge`: none at an end,
        // at a branch, or where an edge closes on itself.
        let through = |v: VertexId, edge: EdgeId| match at.get(&v).map(Vec::as_slice) {
            Some(&[a, b]) if a != b => Some(if a == edge { b } else { a }),
            _ => None,
        };
        // Where `edge` starts and ends, run in `sense`.
        let ends = |edge: EdgeId, sense: Sense| {
            let e = self.get_edge(edge)?;
            Ok::<_, GeopError>(match sense {
                Sense::Forward => (e.start_vertex, e.end_vertex),
                Sense::Reversed => (e.end_vertex, e.start_vertex),
            })
        };
        let mut edges = wire.edges.clone();
        edges.sort();
        let mut used = HashSet::new();
        let mut chains = Vec::new();
        for &first in &edges {
            if used.contains(&first) {
                continue;
            }
            // Back from `first` to where its run starts, or round to it.
            let (mut edge, mut sense) = (first, Sense::Forward);
            loop {
                let (start, _) = ends(edge, sense)?;
                let Some(previous) = through(start, edge).filter(|&p| p != first) else {
                    break;
                };
                sense = if self.get_edge(previous)?.end_vertex == start {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                edge = previous;
            }
            // Forward from there.
            let mut chain = Vec::new();
            loop {
                used.insert(edge);
                chain.push((edge, sense));
                let (_, end) = ends(edge, sense)?;
                let Some(next) = through(end, edge).filter(|n| !used.contains(n)) else {
                    break;
                };
                sense = if self.get_edge(next)?.start_vertex == end {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                edge = next;
            }
            chains.push(chain);
        }
        Ok(chains)
    }
}
