use crate::{CoedgeId, VertexId};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundaryType {
    /// A boundary defined by a single vertex — no edge exists yet (e.g. a
    /// face fresh out of `mvfs`).
    Vertex(VertexId),
    /// A boundary defined by a starting edge loop.
    Loop(CoedgeId),
}

/// Which of a face's boundaries: its one outer loop, or the `n`-th hole.
///
/// A face's boundaries are not interchangeable — the outer loop bounds the
/// material and every hole removes from it — so code that locates a boundary
/// says *which kind* it found rather than returning a bare index into one
/// flat list. That distinction is exactly what `splice_edge_into_face` needs
/// to decide whether joining two loops merges two holes, absorbs a hole into
/// the outer loop, or splits the face in two.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundaryIndex {
    Outer,
    Hole(usize),
}
