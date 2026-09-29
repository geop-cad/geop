use crate::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

#[derive(Clone)]
pub struct TriangleFace<S: Scalar> {
    pub a: Vector3<S>,
    pub b: Vector3<S>,
    pub c: Vector3<S>,
    /// The triangle's own (flat) normal, from its winding.
    pub normal: Vector3<S>,
    /// The normal of the *surface* this triangle approximates, at each of
    /// its three corners — what a renderer needs to shade a curved face
    /// smoothly instead of as a field of facets. `None` where no surface
    /// normal was available (a debug triangle, or a degenerate point like a
    /// sphere's pole), leaving a renderer with the flat [`Self::normal`].
    pub vertex_normals: Option<[Vector3<S>; 3]>,
}

impl<S: Scalar> core::fmt::Debug for TriangleFace<S> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "TriangleFace({:?}, {:?}, {:?})", self.a, self.b, self.c)
    }
}

impl<S: Scalar> TriangleFace<S> {
    /// Computes the normal from the cross product of (b-a) × (c-a).
    /// Fails if the cross product is zero (collinear or coincident points).
    pub fn try_new(a: Vector3<S>, b: Vector3<S>, c: Vector3<S>) -> GeopResult<Self> {
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
    pub fn with_vertex_normals(self, normals: [Vector3<S>; 3]) -> Self {
        let flip = normals[0].prod_dot(&self.normal).definitely_less(S::ZERO);
        let orient = |n: Vector3<S>| {
            if flip {
                n.prod_scalar(S::ZERO.sub(S::ONE))
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
