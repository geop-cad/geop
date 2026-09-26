use super::ids::VertexId;
use super::model::Curve3;
use geop_core_math::scalars::Scalar;

/// A 1-dimensional topological entity: a bounded arc of a 3-D NURBS curve.
///
/// The curve is owned directly by its edge, and is always pre-trimmed so its
/// own domain (`curve.domain()`) is exactly this edge's span — there is no
/// separate `[start_t, end_t]` window into a possibly-larger curve.
/// `start_vertex` lies at `curve.domain().0` and `end_vertex` at
/// `curve.domain().1`.
#[derive(Clone, Debug)]
pub struct Edge<S: Scalar> {
    pub curve: Curve3<S>,
    pub start_vertex: VertexId,
    pub end_vertex: VertexId,
}
