use super::ids::{EdgeId, VertexId};

/// Edges and vertices standing on their own, bounding no face: curves in
/// space, such as the lines, arcs and splines of a 3-D sketch.
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
#[derive(Clone, Debug)]
pub struct Wire {
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
}
