//! Curves and surfaces written exactly ([`Exact`]): their degrees, control
//! points and knots, each scalar's bounds to the bit. Read back through
//! their constructors, which check them and work out what is derived from
//! them (the box around the control points) as they did when first made.

use geop_core_math::{
    exact::{Exact, Written},
    scalars::Scalar,
    vector::Vector,
};
use serde::{Deserialize, Deserializer, Serializer, de::Error, ser::SerializeStruct};

use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};

impl<const D: usize> Exact for NurbCurve<D> {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut curve = serializer.serialize_struct("NurbCurve", 3)?;
        curve.serialize_field("degree", &self.degree)?;
        curve.serialize_field("control_points", &Written(&self.control_points))?;
        curve.serialize_field("knots", &Written(&self.knot_vector))?;
        curve.end()
    }

    fn read<'de, De: Deserializer<'de>>(deserializer: De) -> Result<Self, De::Error> {
        #[derive(Deserialize)]
        struct Curve<const D: usize> {
            degree: usize,
            #[serde(with = "geop_core_math::exact")]
            control_points: Vec<Vector<D>>,
            #[serde(with = "geop_core_math::exact")]
            knots: Vec<Scalar>,
        }
        let Curve {
            degree,
            control_points,
            knots,
        } = Curve::<D>::deserialize(deserializer)?;
        NurbCurve::try_new(degree, control_points, knots).map_err(De::Error::custom)
    }
}

impl<const D: usize> Exact for NurbSurface<D> {
    fn write<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut surface = serializer.serialize_struct("NurbSurface", 5)?;
        surface.serialize_field("degree_u", &self.degree_u)?;
        surface.serialize_field("degree_v", &self.degree_v)?;
        surface.serialize_field("control_points", &Written(&self.control_points))?;
        surface.serialize_field("knots_u", &Written(&self.knot_vector_u))?;
        surface.serialize_field("knots_v", &Written(&self.knot_vector_v))?;
        surface.end()
    }

    fn read<'de, De: Deserializer<'de>>(deserializer: De) -> Result<Self, De::Error> {
        #[derive(Deserialize)]
        struct Surface<const D: usize> {
            degree_u: usize,
            degree_v: usize,
            #[serde(with = "geop_core_math::exact")]
            control_points: Vec<Vector<D>>,
            #[serde(with = "geop_core_math::exact")]
            knots_u: Vec<Scalar>,
            #[serde(with = "geop_core_math::exact")]
            knots_v: Vec<Scalar>,
        }
        let Surface {
            degree_u,
            degree_v,
            control_points,
            knots_u,
            knots_v,
        } = Surface::<D>::deserialize(deserializer)?;
        NurbSurface::try_new(degree_u, degree_v, control_points, knots_u, knots_v)
            .map_err(De::Error::custom)
    }
}
