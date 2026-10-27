use crate::{CoedgeId, VertexId};
use geop_core_math::geop_error::{GeopError, GeopResult};

use super::ids::{EdgeId, FaceId};
use super::model::{Curve2, Curve3};

/// Whether a coedge traverses its underlying edge in the same direction
/// (forward: start→end) or the opposite direction (reversed: end→start).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Sense {
    Forward,
    Reversed,
}

impl Sense {
    /// The sense of a coedge running its edge forward if `forward`.
    pub fn from_forward(forward: bool) -> Sense {
        if forward {
            Sense::Forward
        } else {
            Sense::Reversed
        }
    }

    pub fn opposite(self) -> Sense {
        match self {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        }
    }

    /// `curve`, run the way a coedge of this sense runs it.
    pub fn orient(self, curve: &Curve3) -> Curve3 {
        match self {
            Sense::Forward => curve.clone(),
            Sense::Reversed => curve.reverse(),
        }
    }
}

/// What a coedge's own `(u, v)` boundary segment is actually backed by.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
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
    /// edge, so it leaves the Euler–Poincaré count as it is.
    Vertex(VertexId),
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Coedge {
    pub geometry: CoedgeGeometry,
    pub sense: Sense,
    #[serde(with = "geop_core_math::exact")]
    pub pcurve: Curve2,
    pub next: CoedgeId,
    pub prev: CoedgeId,
    // face
    pub face: FaceId,
}

impl Coedge {
    /// This coedge's own edge — an error for a `Vertex`-backed coedge
    /// (there's no edge to compare or collect there). Convenience for
    /// code that only ever applies to edge-backed coedges.
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
