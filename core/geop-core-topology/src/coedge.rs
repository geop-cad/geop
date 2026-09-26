use crate::{CoedgeId, VertexId};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_math::scalars::Scalar;

use super::ids::{EdgeId, FaceId};
use super::model::Curve2;

/// Whether a coedge traverses its underlying edge in the same direction
/// (forward: start→end) or the opposite direction (reversed: end→start).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    Forward,
    Reversed,
}

impl Sense {
    pub fn opposite(self) -> Sense {
        match self {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        }
    }
}

/// What a coedge's own `(u, v)` boundary segment is actually backed by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoedgeGeometry {
    /// A real, shared edge — this coedge is one of exactly the two
    /// (opposite `Sense`) that trace it, one per adjoining face. The usual
    /// case.
    Edge(EdgeId),
    /// A degenerate loop segment that sits at a single, already-existing
    /// vertex the whole way — no edge, and (unlike `Edge`) no second
    /// coedge sharing it, since there's nothing to share between two
    /// faces: it belongs to exactly one face's own loop. This is how a
    /// loop closes over a surface row that's collapsed to a single point
    /// (e.g. a pole) — the loop still needs *some* pcurve tracing that
    /// row's full parameter range even though there's no 3-D geometry
    /// there to back a real edge. Adds neither a new vertex nor a new
    /// edge, so it never needs to satisfy (or risk violating) the
    /// Euler–Poincaré invariant euler operators preserve — it isn't one.
    Vertex(VertexId),
}

#[derive(Clone, Debug)]
pub struct Coedge<S: Scalar> {
    pub geometry: CoedgeGeometry,
    pub sense: Sense,
    pub pcurve: Curve2<S>,
    pub next: CoedgeId,
    pub prev: CoedgeId,
    // face
    pub face: FaceId,
}

impl<S: Scalar> Coedge<S> {
    /// This coedge's own edge — an error for a `Vertex`-backed coedge
    /// (there's no edge to kill/compare/collect there). Convenience for
    /// the (common) operators that only ever apply to edge-backed
    /// coedges, e.g. `kve`/`kemr`/`ker`.
    pub fn edge(&self) -> GeopResult<EdgeId> {
        match self.geometry {
            CoedgeGeometry::Edge(id) => Ok(id),
            CoedgeGeometry::Vertex(v) => Err(GeopError::new(format!(
                "coedge is vertex-backed (vertex {}), not edge-backed",
                v.0
            ))),
        }
    }
}
