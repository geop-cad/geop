use crate::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

#[derive(Clone)]
pub struct TriangleFace {
    pub a: Vector3,
    pub b: Vector3,
    pub c: Vector3,
    /// The triangle's own (flat) normal, from its winding.
    pub normal: Vector3,
    /// The normal of the *surface* this triangle approximates, at each of
    /// its three corners — what a renderer needs to shade a curved face
    /// smoothly instead of as a field of facets. `None` where no surface
    /// normal was available (a debug triangle, or a degenerate point like a
    /// sphere's pole), leaving a renderer with the flat [`Self::normal`].
    pub vertex_normals: Option<[Vector3; 3]>,
}

impl core::fmt::Debug for TriangleFace {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "TriangleFace({:?}, {:?}, {:?})", self.a, self.b, self.c)
    }
}

impl TriangleFace {
    /// Computes the normal from the cross product of (b-a) × (c-a).
    /// Fails if the cross product is zero (collinear or coincident points).
    pub fn try_new(a: Vector3, b: Vector3, c: Vector3) -> GeopResult<Self> {
        let ctx = |err: GeopError| {
            err.with_context(format!("TriangleFace::try_new({a:?}, {b:?}, {c:?})"))
        };
        let ba = b.sub(&a);
        let ca = c.sub(&a);
        let raw_normal = ba.prod_cross(&ca);
        let normal = raw_normal.normalize().with_context(&ctx)?;
        Ok(Self {
            a,
            b,
            c,
            normal,
            vertex_normals: None,
        })
    }

    /// This triangle carrying the surface normals at its corners, each
    /// oriented to agree with the triangle's own winding — a renderer picks
    /// front or back from the winding, so a vertex normal pointing the other
    /// way would light the face inside out.
    pub fn with_vertex_normals(self, normals: [Vector3; 3]) -> Self {
        let flip = normals[0]
            .prod_dot(&self.normal)
            .definitely_less(Scalar::ZERO);
        let orient = |n: Vector3| {
            if flip {
                n.prod_scalar(Scalar::ZERO.sub(Scalar::ONE))
            } else {
                n
            }
        };
        Self {
            vertex_normals: Some(normals.map(orient)),
            ..self
        }
    }
}
