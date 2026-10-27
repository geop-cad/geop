use geop_core_math::vector::Vector3;

/// A 0-dimensional topological entity: a point in 3-D space.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Vertex {
    #[serde(with = "geop_core_math::exact")]
    pub point: Vector3,
}
