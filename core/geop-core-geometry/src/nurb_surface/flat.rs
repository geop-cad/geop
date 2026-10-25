//! A flat face's surface around the curves bounding it ([`FlatPatch`]): the
//! cap of a sweep, of a blend's tool, the face filling a flat loop.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::NurbSurface3D;
use crate::{
    aabb::union_aabb,
    nurb_curve::{NurbCurve, NurbCurve2D},
};

/// The box around curves drawn in a plane, as a bilinear patch of it: a
/// plane point `(x, y)` lies at `origin + x e1 + y e2`, and the patch is
/// parametrized over `[0, 1]²` along `(x, y)` — or along `(y, x)` if
/// `swapped`, whichever makes its loops wind the way a face's must.
///
/// The box encloses every curve: a NURBS curve stays within the convex hull
/// of its control points, and the box is the outer bounds of their
/// enclosures. A curve drawn in the plane is mapped into the patch's
/// parameters by an affine map of its control points ([`FlatPatch::pcurve`]),
/// exact for any NURBS.
#[derive(Clone, Debug)]
pub struct FlatPatch {
    origin: Vector3,
    e1: Vector3,
    e2: Vector3,
    lo: Vector2,
    size: Vector2,
    swapped: bool,
}

impl FlatPatch {
    /// The patch of the plane `(origin, e1, e2)` around `curves`, given in
    /// its coordinates.
    pub fn around<'a>(
        origin: Vector3,
        e1: Vector3,
        e2: Vector3,
        curves: impl IntoIterator<Item = &'a NurbCurve2D>,
        swapped: bool,
    ) -> GeopResult<Self> {
        let bounds = union_aabb(curves.into_iter().map(NurbCurve::aabb))
            .ok_or_else(|| GeopError::new("FlatPatch::around: no curves to span"))?;
        let lo: [f64; 2] = std::array::from_fn(|k| bounds[k].lower().to_f64());
        let hi: [f64; 2] = std::array::from_fn(|k| bounds[k].upper().to_f64());
        // The corners are a free choice, so sharp; the size has to reach
        // `hi`: `hi - lo` in plain numbers, unless that rounds so far down
        // that `lo + size` falls definitely short of `hi`, and then the
        // upper bound of `hi - lo`.
        let size = std::array::from_fn(|k| {
            let (from, to) = (Scalar::from_f64(lo[k]), Scalar::from_f64(hi[k]));
            let plain = Scalar::from_f64(hi[k] - lo[k]);
            if from.add(plain).definitely_less(to) {
                to.sub(from).upper()
            } else {
                plain
            }
        });
        Ok(Self {
            origin,
            e1,
            e2,
            lo: Vector2::from_array(lo.map(Scalar::from_f64)),
            size: Vector2::from_array(size),
            swapped,
        })
    }

    /// How far the patch reaches along the plane's two axes: its lowest
    /// corner and its highest, in the plane's coordinates.
    pub fn reach(&self) -> [Vector2; 2] {
        [self.lo, self.lo.add(&self.size)]
    }

    /// `curve`, drawn in the plane, in the patch's parameters.
    pub fn pcurve(&self, curve: &NurbCurve2D) -> GeopResult<NurbCurve2D> {
        let control_points = curve
            .control_points
            .iter()
            .map(|cp| {
                Ok(Vector3::from_array([
                    cp[0].sub(cp[2].mul(self.lo[0])).div(self.size[0])?,
                    cp[1].sub(cp[2].mul(self.lo[1])).div(self.size[1])?,
                    cp[2],
                ]))
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let pcurve = NurbCurve::try_new(curve.degree, control_points, curve.knot_vector.clone())?;
        Ok(if self.swapped {
            pcurve.swap_xy()
        } else {
            pcurve
        })
    }

    /// The patch itself.
    pub fn surface(&self) -> GeopResult<NurbSurface3D> {
        let corner = |i: usize, j: usize| {
            let x = if i == 1 {
                self.lo[0].add(self.size[0])
            } else {
                self.lo[0]
            };
            let y = if j == 1 {
                self.lo[1].add(self.size[1])
            } else {
                self.lo[1]
            };
            self.origin
                .add(&self.e1.prod_scalar(x))
                .add(&self.e2.prod_scalar(y))
        };
        if self.swapped {
            NurbSurface3D::bilinear(corner(0, 0), corner(0, 1), corner(1, 1), corner(1, 0))
        } else {
            NurbSurface3D::bilinear(corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A square reaching far one way along each axis and only a little the
    /// other: its box's size in plain numbers, `0.001 - -3`, rounds below
    /// what the box spans. The patch must still reach every point of it.
    #[test]
    fn patch_encloses_its_curves() {
        let corners = [(-3.0, -3.0), (0.001, -3.0), (0.001, 0.001), (-3.0, 0.001)]
            .map(|(x, y)| Vector2::from_array([Scalar::from_f64(x), Scalar::from_f64(y)]));
        let sides: Vec<NurbCurve2D> = (0..4)
            .map(|k| NurbCurve2D::line(corners[k], corners[(k + 1) % 4]).unwrap())
            .collect();
        let patch = FlatPatch::around(
            Vector3::zero(),
            Vector3::axis(0),
            Vector3::axis(1),
            &sides,
            false,
        )
        .unwrap();
        let reach = Scalar::from_f64(0.001);
        let [_, far] = patch.reach();
        for k in 0..2 {
            assert!(
                !far[k].definitely_less(reach),
                "along axis {k} the patch reaches {:?}, short of the curves' {reach:?}",
                far[k]
            );
        }
    }
}
