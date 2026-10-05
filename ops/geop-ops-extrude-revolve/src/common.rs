//! Shared geometry helpers for the constructors in this crate, and
//! [`Profile`], the named curve chain extrude and revolve sweep. The
//! topology itself is described whole and built in one go, see
//! [`crate::sweep`].

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
    nurb_surface::{NurbSurface, NurbSurface3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
};

// ── Profiles ────────────────────────────────────────────────────────────────

/// A chain of profile curves, with a stable name for every curve and every
/// joint between them — what extrude and revolve build the names of the
/// faces, edges and vertices they sweep out of it from (see
/// `geop_ops`'s crate docs). For a sketch these are its element ids;
/// for a shape built in code, positions in the chain ([`Profile::closed`]).
///
/// `joint_names[i]` names the joint where `curves[i]` starts. A closed loop
/// has one joint per curve; an open chain one more, its end.
#[derive(Clone, Debug)]
pub struct Profile<S: Scalar> {
    pub curves: Vec<NurbCurve2D<S>>,
    pub curve_names: Vec<String>,
    pub joint_names: Vec<String>,
}

impl<S: Scalar> Profile<S> {
    /// A closed loop whose curves are called `c0, c1, ...` and whose joints
    /// `p0, p1, ...`, `p0` where `c0` starts.
    pub fn closed(curves: Vec<NurbCurve2D<S>>) -> Self {
        let n = curves.len();
        Self::numbered(curves, n)
    }

    /// An open chain, named like [`Profile::closed`] plus its end joint.
    pub fn open(curves: Vec<NurbCurve2D<S>>) -> Self {
        let n = curves.len();
        Self::numbered(curves, n + 1)
    }

    fn numbered(curves: Vec<NurbCurve2D<S>>, joints: usize) -> Self {
        Self {
            curve_names: (0..curves.len()).map(|i| format!("c{i}")).collect(),
            joint_names: (0..joints).map(|i| format!("p{i}")).collect(),
            curves,
        }
    }

    /// The same chain with `prefix` put in front of every name — to keep
    /// several [`Profile::closed`] loops of one extrude apart.
    pub fn with_prefix(mut self, prefix: &str) -> Self {
        for name in self.curve_names.iter_mut().chain(&mut self.joint_names) {
            *name = format!("{prefix}{name}");
        }
        self
    }

    pub fn is_closed(&self) -> bool {
        self.joint_names.len() == self.curves.len()
    }

    /// Checks that there is one name per curve and one per joint.
    pub fn check_names(&self) -> GeopResult<()> {
        let n = self.curves.len();
        if self.curve_names.len() != n
            || !(self.joint_names.len() == n || self.joint_names.len() == n + 1)
        {
            return Err(GeopError::new(format!(
                "profile of {n} curves has {} curve names and {} joint names",
                self.curve_names.len(),
                self.joint_names.len()
            )));
        }
        Ok(())
    }

    /// The same chain traversed the other way; every name stays with its
    /// curve or joint.
    pub fn reversed(&self) -> Self {
        let n = self.curves.len();
        let joint_names = if self.is_closed() {
            // Reversed curve `m` is old curve `n - 1 - m`, which now starts
            // where it used to end: at old joint `n - m`.
            (0..n)
                .map(|m| self.joint_names[(n - m) % n].clone())
                .collect()
        } else {
            self.joint_names.iter().rev().cloned().collect()
        };
        Self {
            curves: self.curves.iter().rev().map(|c| c.reverse()).collect(),
            curve_names: self.curve_names.iter().rev().cloned().collect(),
            joint_names,
        }
    }

    /// The same chain with `f` applied to every curve, keeping the names.
    pub fn map_curves(&self, f: impl Fn(&NurbCurve2D<S>) -> NurbCurve2D<S>) -> Self {
        Self {
            curves: self.curves.iter().map(f).collect(),
            ..self.clone()
        }
    }
}

// ── Geometry helpers ─────────────────────────────────────────────────────────

/// Lift a 3-D point into homogeneous coordinates with weight 1.
pub fn pt3<S: Scalar>(p: Vector3<S>) -> Vector4<S> {
    Vector4::from_array([p[0], p[1], p[2], S::ONE])
}

/// Lift a 2-D point into homogeneous coordinates with weight 1.
pub fn pt2<S: Scalar>(p: Vector2<S>) -> Vector3<S> {
    Vector3::from_array([p[0], p[1], S::ONE])
}

/// A degree-1 line segment in 3-D from `p0` to `p1`, parametrized over `[0, 1]`.
pub fn line3<S: Scalar>(p0: Vector3<S>, p1: Vector3<S>) -> GeopResult<NurbCurve3D<S>> {
    NurbCurve::try_new(
        1,
        vec![pt3(p0), pt3(p1)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// A degree-1 line segment in parameter space from `p0` to `p1`, parametrized over `[0, 1]`.
pub fn line2<S: Scalar>(p0: Vector2<S>, p1: Vector2<S>) -> GeopResult<NurbCurve2D<S>> {
    NurbCurve::try_new(
        1,
        vec![pt2(p0), pt2(p1)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// The closed polygon through `points` (and back to the first), one line
/// per side — the simplest profile loop for [`crate::extrude::extrude`].
pub fn polygon<S: Scalar>(points: &[Vector2<S>]) -> GeopResult<Vec<NurbCurve2D<S>>> {
    (0..points.len())
        .map(|i| line2(points[i], points[(i + 1) % points.len()]))
        .collect()
}

/// The open chain of lines through `points` — the simplest profile for
/// [`crate::revolve::revolve_at_oriented`].
pub fn polyline<S: Scalar>(points: &[Vector2<S>]) -> GeopResult<Vec<NurbCurve2D<S>>> {
    points.windows(2).map(|w| line2(w[0], w[1])).collect()
}

/// First control point of `curve`, dehomogenized: its start, for the clamped
/// knot vectors every profile curve has.
pub fn start_point<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Vector2<S>> {
    let cp = curve.control_points[0];
    Ok(Vector2::from_array([cp[0].div(cp[2])?, cp[1].div(cp[2])?]))
}

/// Last control point of `curve`, dehomogenized: its end.
pub fn end_point<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Vector2<S>> {
    start_point(&curve.reverse())
}

/// Homogeneous `(w (origin + x e1 + y e2), w)` for the homogeneous 2-D point
/// `cp = (w x, w y, w)`: linear in `cp`, so no division.
pub fn embed_point<S: Scalar>(
    cp: &Vector3<S>,
    origin: &Vector3<S>,
    e1: &Vector3<S>,
    e2: &Vector3<S>,
) -> Vector4<S> {
    let p = origin
        .prod_scalar(cp[2])
        .add(&e1.prod_scalar(cp[0]))
        .add(&e2.prod_scalar(cp[1]));
    Vector4::from_array([p[0], p[1], p[2], cp[2]])
}

/// A bilinear (degree 1x1) surface patch with corners `P00, P01, P10, P11`
/// (control points laid out `[P00, P10, P11, P01]`, `num_v = 2`).
pub fn bilinear<S: Scalar>(
    p00: Vector3<S>,
    p10: Vector3<S>,
    p11: Vector3<S>,
    p01: Vector3<S>,
) -> GeopResult<NurbSurface3D<S>> {
    NurbSurface::try_new(
        1,
        1,
        vec![pt3(p00), pt3(p01), pt3(p10), pt3(p11)],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// `sqrt(2) / 2`, the rational weight that makes a 3-point quadratic Bezier
/// trace an exact 90-degree circular arc.
pub fn sqrt2_over_2<S: Scalar>() -> S {
    S::from_f64(std::f64::consts::SQRT_2 / 2.0)
}

/// A degree-2 rational Bezier curve in 3-D from `p0` to `p2` through the
/// (weighted) control point `mid`, with middle weight `w`. For an arc
/// centered at `c` with `w = sqrt2_over_2()`, pass `mid = p0 + p2 - c` (i.e.
/// `c + (p0 - c) + (p2 - c)`, the sum of the two radius vectors relative to
/// the arc's actual center, offset back into absolute coordinates) to trace
/// an exact 90-degree circular arc of radius `|p0 - c| = |p2 - c|`.
pub fn arc3<S: Scalar>(
    p0: Vector3<S>,
    mid: Vector3<S>,
    p2: Vector3<S>,
    w: S,
) -> GeopResult<NurbCurve3D<S>> {
    let cp0 = pt3(p0);
    let cp2 = pt3(p2);
    let cp1 = Vector4::from_array([mid[0].mul(w), mid[1].mul(w), mid[2].mul(w), w]);
    NurbCurve::try_new(
        2,
        vec![cp0, cp1, cp2],
        vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
    )
}

/// A degree-2 rational Bezier curve in parameter space from `p0` to `p2`
/// through the (weighted) control point `mid`, with middle weight `w`.
pub fn arc2<S: Scalar>(
    p0: Vector2<S>,
    mid: Vector2<S>,
    p2: Vector2<S>,
    w: S,
) -> GeopResult<NurbCurve2D<S>> {
    let cp0 = pt2(p0);
    let cp2 = pt2(p2);
    let cp1 = Vector3::from_array([mid[0].mul(w), mid[1].mul(w), w]);
    NurbCurve::try_new(
        2,
        vec![cp0, cp1, cp2],
        vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
    )
}

#[cfg(test)]
mod tests {
    use super::bilinear;
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    fn p<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// 4 arbitrary (non-degenerate, non-planar-aligned) corners — evaluating
    /// the resulting bilinear patch at its corners must return them exactly,
    /// and at an interior `(u, v)` must match the bilinear interpolation
    /// formula directly.
    fn check_bilinear_maps_uv_correctly<S: Scalar>() {
        let p00 = p::<S>(0.3, -1.2, 2.5);
        let p10 = p::<S>(4.1, 0.7, -0.3);
        let p11 = p::<S>(2.2, 3.3, 1.1);
        let p01 = p::<S>(-1.5, 2.0, 0.6);

        let surface = bilinear(p00, p10, p11, p01).unwrap();

        assert!(
            surface
                .evaluate(S::ZERO, S::ZERO)
                .unwrap()
                .could_be_equal(&p00)
        );
        assert!(
            surface
                .evaluate(S::ONE, S::ZERO)
                .unwrap()
                .could_be_equal(&p10)
        );
        assert!(
            surface
                .evaluate(S::ONE, S::ONE)
                .unwrap()
                .could_be_equal(&p11)
        );
        assert!(
            surface
                .evaluate(S::ZERO, S::ONE)
                .unwrap()
                .could_be_equal(&p01)
        );
    }
    #[test]
    fn bilinear_maps_uv_correctly() {
        for_all_scalars!(check_bilinear_maps_uv_correctly);
    }
}
