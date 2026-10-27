use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use core::fmt::Display;

/// A local coordinate system: an `origin` point plus 3 linearly independent
/// (not necessarily orthogonal or unit-length) basis vectors `u`, `v`, `w`,
/// letting a caller convert a point's coordinates between this frame
/// (`uvw` space, relative to `origin`) and the ambient `xyz` space it was
/// itself expressed in.
#[derive(Clone, Debug, PartialEq)]
pub struct CoordinateSystem {
    origin: Vector3,
    u: Vector3,
    v: Vector3,
    w: Vector3,
    /// The reciprocal basis (`u*`, `v*`, `w*`), precomputed so `to_uvw` is
    /// just 3 dot products: `u* = (v × w) / det`, and cyclically for `v*`,
    /// `w*`, where `det = u . (v × w)` is the basis's signed volume.
    u_star: Vector3,
    v_star: Vector3,
    w_star: Vector3,
}

impl CoordinateSystem {
    /// Build the coordinate system from its `origin` and 3 basis vectors.
    /// Fails if `u`, `v`, `w` could be linearly dependent (zero signed
    /// volume `u . (v × w)`), since that basis can't be inverted into a
    /// reciprocal one.
    pub fn try_new(origin: Vector3, u: Vector3, v: Vector3, w: Vector3) -> GeopResult<Self> {
        let det = u.prod_dot(&v.prod_cross(&w));
        if det.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(
                "CoordinateSystem::try_new: u, v, w could be linearly dependent (zero volume)",
            ));
        }
        let inv_det = Scalar::ONE.div(det)?;
        let u_star = v.prod_cross(&w).prod_scalar(inv_det);
        let v_star = w.prod_cross(&u).prod_scalar(inv_det);
        let w_star = u.prod_cross(&v).prod_scalar(inv_det);
        Ok(Self {
            origin,
            u,
            v,
            w,
            u_star,
            v_star,
            w_star,
        })
    }

    /// The right-handed frame at `origin` with `x` as its `u` and `z` as
    /// its `w`, and `v = z x x`: for `x` square to `z` and both unit, an
    /// orthonormal one. Fails where `x` and `z` could be parallel.
    pub fn try_from_xz(origin: Vector3, x: Vector3, z: Vector3) -> GeopResult<Self> {
        Self::try_new(origin, x, z.prod_cross(&x), z)
    }

    /// The world's own axes `x`, `y` and `z`, at `origin`.
    pub fn world_at(origin: Vector3) -> Self {
        Self::try_new(origin, Vector3::axis(0), Vector3::axis(1), Vector3::axis(2))
            .expect("the world's axes are independent")
    }

    pub fn origin(&self) -> &Vector3 {
        &self.origin
    }
    pub fn u(&self) -> &Vector3 {
        &self.u
    }
    pub fn v(&self) -> &Vector3 {
        &self.v
    }
    pub fn w(&self) -> &Vector3 {
        &self.w
    }

    /// Convert a point expressed in this frame (`uvw` coordinates, relative
    /// to `origin`) to ambient `xyz` space: `origin + p_uvw[0] * u +
    /// p_uvw[1] * v + p_uvw[2] * w`.
    pub fn to_xyz(&self, p_uvw: &Vector3) -> Vector3 {
        self.origin
            .add(&self.u.prod_scalar(p_uvw[0]))
            .add(&self.v.prod_scalar(p_uvw[1]))
            .add(&self.w.prod_scalar(p_uvw[2]))
    }

    /// Convert a point expressed in this frame's `u`/`v` plane (`w = 0`) to
    /// ambient `xyz` space: `origin + p_uv[0] * u + p_uv[1] * v`.
    pub fn uv_to_xyz(&self, p_uv: &Vector2) -> Vector3 {
        self.origin
            .add(&self.u.prod_scalar(p_uv[0]))
            .add(&self.v.prod_scalar(p_uv[1]))
    }

    /// The direction `d`, given in this frame's `u`/`v` plane, in ambient
    /// `xyz` space: `d[0] * u + d[1] * v`.
    pub fn uv_direction(&self, d: &Vector2) -> Vector3 {
        self.u.prod_scalar(d[0]).add(&self.v.prod_scalar(d[1]))
    }

    /// Convert an ambient `xyz` point into this frame's `uvw` coordinates
    /// (relative to `origin`), via the precomputed reciprocal basis (`u* .
    /// (p - origin)`, `v* . (p - origin)`, `w* . (p - origin)`).
    pub fn to_uvw(&self, p_xyz: &Vector3) -> Vector3 {
        let p = p_xyz.sub(&self.origin);
        Vector3::from_array([
            p.prod_dot(&self.u_star),
            p.prod_dot(&self.v_star),
            p.prod_dot(&self.w_star),
        ])
    }

    /// The ambient `xyz` point `p_xyz` in this frame's `u`/`v` coordinates:
    /// projected onto its `u`/`v` plane along `w` (see [`Self::to_uvw`]).
    pub fn to_uv(&self, p_xyz: &Vector3) -> Vector2 {
        self.to_uvw(p_xyz).head()
    }
}

/// Its origin and axes, each exactly ([`crate::exact`]); read back, the
/// reciprocal basis is computed again from them, as it was.
impl crate::exact::Exact for CoordinateSystem {
    fn write<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use crate::exact::Written;
        let axes = [&self.origin, &self.u, &self.v, &self.w].map(Written);
        serde::Serialize::serialize(&axes, serializer)
    }

    fn read<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use crate::exact::Read;
        let [Read(origin), Read(u), Read(v), Read(w)]: [Read<Vector3>; 4] =
            serde::Deserialize::deserialize(deserializer)?;
        Self::try_new(origin, u, v, w).map_err(serde::de::Error::custom)
    }
}

/// Serializes as `{origin, u, v, w}`, for a viewer.
impl serde::Serialize for CoordinateSystem {
    fn serialize<Ser: serde::Serializer>(&self, serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("CoordinateSystem", 4)?;
        s.serialize_field("origin", &self.origin)?;
        s.serialize_field("u", &self.u)?;
        s.serialize_field("v", &self.v)?;
        s.serialize_field("w", &self.w)?;
        s.end()
    }
}

impl Display for CoordinateSystem {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "CoordinateSystem(origin={}, u={}, v={}, w={})",
            self.origin, self.u, self.v, self.w
        )
    }
}

#[cfg(test)]
mod tests {
    use super::CoordinateSystem;
    use crate::{
        scalars::Scalar,
        vector::{Vector2, Vector3},
    };

    #[test]
    fn orthonormal_round_trip() {
        let origin = Vector3::from_array([Scalar::ZERO; 3]);
        let u = Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]);
        let w = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]);
        let cs = CoordinateSystem::try_new(origin, u, v, w).unwrap();

        let p = Vector3::from_array([
            Scalar::from_f64(2.0),
            Scalar::from_f64(-3.0),
            Scalar::from_f64(5.0),
        ]);
        assert!(cs.to_uvw(&p).could_be_equal(&p));
        assert!(cs.to_xyz(&p).could_be_equal(&p));
    }

    #[test]
    fn skewed_basis_round_trip() {
        let origin = Vector3::from_array([Scalar::ZERO; 3]);
        let u = Vector3::from_array([Scalar::from_f64(1.0), Scalar::from_f64(0.5), Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::from_f64(2.0), Scalar::from_f64(0.3)]);
        let w = Vector3::from_array([Scalar::from_f64(0.2), Scalar::ZERO, Scalar::from_f64(1.5)]);
        let cs = CoordinateSystem::try_new(origin, u, v, w).unwrap();

        let p_uvw = Vector3::from_array([
            Scalar::from_f64(1.3),
            Scalar::from_f64(-0.7),
            Scalar::from_f64(2.1),
        ]);
        let p_xyz = cs.to_xyz(&p_uvw);
        let round_tripped = cs.to_uvw(&p_xyz);
        assert!(round_tripped.could_be_equal(&p_uvw));

        // u, v, w themselves must map to the standard basis vectors.
        assert!(cs.to_uvw(&u).could_be_equal(&Vector3::from_array([
            Scalar::ONE,
            Scalar::ZERO,
            Scalar::ZERO
        ])));
        assert!(cs.to_uvw(&v).could_be_equal(&Vector3::from_array([
            Scalar::ZERO,
            Scalar::ONE,
            Scalar::ZERO
        ])));
        assert!(cs.to_uvw(&w).could_be_equal(&Vector3::from_array([
            Scalar::ZERO,
            Scalar::ZERO,
            Scalar::ONE
        ])));
    }

    #[test]
    fn offset_origin_round_trip() {
        let origin = Vector3::from_array([
            Scalar::from_f64(10.0),
            Scalar::from_f64(-4.0),
            Scalar::from_f64(2.0),
        ]);
        let u = Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]);
        let w = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]);
        let cs = CoordinateSystem::try_new(origin, u, v, w).unwrap();

        // The origin itself is `(0, 0, 0)` in `uvw` space.
        assert!(
            cs.to_uvw(&origin)
                .could_be_equal(&Vector3::from_array([Scalar::ZERO; 3]))
        );
        assert!(
            cs.to_xyz(&Vector3::from_array([Scalar::ZERO; 3]))
                .could_be_equal(&origin)
        );

        let p_uvw = Vector3::from_array([
            Scalar::from_f64(1.0),
            Scalar::from_f64(2.0),
            Scalar::from_f64(3.0),
        ]);
        let p_xyz = cs.to_xyz(&p_uvw);
        assert!(p_xyz.could_be_equal(&origin.add(&p_uvw)));
        assert!(cs.to_uvw(&p_xyz).could_be_equal(&p_uvw));
    }

    #[test]
    fn uv_to_xyz_matches_to_xyz_with_zero_w() {
        let origin = Vector3::from_array([
            Scalar::from_f64(1.0),
            Scalar::from_f64(2.0),
            Scalar::from_f64(3.0),
        ]);
        let u = Vector3::from_array([Scalar::from_f64(1.0), Scalar::from_f64(0.5), Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::from_f64(2.0), Scalar::from_f64(0.3)]);
        let w = Vector3::from_array([Scalar::from_f64(0.2), Scalar::ZERO, Scalar::from_f64(1.5)]);
        let cs = CoordinateSystem::try_new(origin, u, v, w).unwrap();

        let p_uv = Vector2::from_array([Scalar::from_f64(1.3), Scalar::from_f64(-0.7)]);
        let p_uvw = Vector3::from_array([p_uv[0], p_uv[1], Scalar::ZERO]);
        assert!(cs.uv_to_xyz(&p_uv).could_be_equal(&cs.to_xyz(&p_uvw)));
    }

    #[test]
    fn degenerate_basis_fails() {
        let origin = Vector3::from_array([Scalar::ZERO; 3]);
        let u = Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]);
        let w = u.add(&v); // coplanar with u, v -- zero volume.
        assert!(CoordinateSystem::try_new(origin, u, v, w).is_err());
    }
}
