use geop_core_math::{scalars::Scalar, vector::Vector3};

/// A 0-dimensional topological entity: a point in 3-D space.
#[derive(Clone, Debug)]
pub struct Vertex<S: Scalar> {
    pub point: Vector3<S>,
}
