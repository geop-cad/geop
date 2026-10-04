mod closest;
mod curvature;
mod evaluate;
mod fit_pcurve;
mod normal;
mod offset;
mod patch;
mod project;
mod reverse;
mod revolve;
mod split;
mod translate;

use std::fmt::Display;

pub use project::clamp;

use crate::aabb::compute_aabb;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

/// A NURBS surface patch whose control points live in `D`-dimensional homogeneous space.
///
/// In practice `D = 4` for all 3-D surfaces (`(wx, wy, wz, w)` control points).
///
/// Control points are stored row-major: `control_points[i * num_v + j]` is the
/// point at u-index `i` (0 ≤ i < `num_u`) and v-index `j` (0 ≤ j < `num_v`).
#[derive(Clone, Debug)]
pub struct NurbSurface<S: Scalar, const D: usize> {
    pub degree_u: usize,
    pub degree_v: usize,
    pub num_u: usize,
    pub num_v: usize,
    pub control_points: Vec<Vector<S, D>>,
    pub knot_vector_u: Vec<S>,
    pub knot_vector_v: Vec<S>,
    /// Cached axis-aligned bounding box of the (dehomogenized) control
    /// points — see [`crate::aabb::compute_aabb`] and `NurbCurve`'s own
    /// `aabb` field doc comment.
    pub(crate) aabb: [S; 3],
}

/// 3-D NURBS surface (homogeneous control points in ℝ⁴).
pub type NurbSurface3D<S> = NurbSurface<S, 4>;

impl<S: Scalar, const D: usize> NurbSurface<S, D> {
    pub fn try_new(
        degree_u: usize,
        degree_v: usize,
        control_points: Vec<Vector<S, D>>,
        knot_vector_u: Vec<S>,
        knot_vector_v: Vec<S>,
    ) -> GeopResult<Self> {
        let len_u = knot_vector_u.len();
        let len_v = knot_vector_v.len();

        if len_u < degree_u + 2 {
            return Err(GeopError::new(
                "NurbSurface: knot_vector_u too short for the given degree",
            ));
        }
        if len_v < degree_v + 2 {
            return Err(GeopError::new(
                "NurbSurface: knot_vector_v too short for the given degree",
            ));
        }

        let num_u = len_u - degree_u - 1;
        let num_v = len_v - degree_v - 1;

        if control_points.len() != num_u * num_v {
            return Err(GeopError::new(format!(
                "NurbSurface: expected {} control points ({}×{}), got {}",
                num_u * num_v,
                num_u,
                num_v,
                control_points.len()
            )));
        }

        // See `NurbCurve::try_new`'s identical check: every weight must be
        // definitely positive, so `W(u, v) > 0` on the whole domain.
        for p in &control_points {
            if !p[D - 1].definitely_greater(S::ZERO) {
                return Err(GeopError::new(&format!(
                    "NurbSurface::try_new: control point {p:?} has a weight that is not definitely positive"
                )));
            }
        }

        let aabb = compute_aabb(&control_points);
        Ok(Self {
            degree_u,
            degree_v,
            num_u,
            num_v,
            control_points,
            knot_vector_u,
            knot_vector_v,
            aabb,
        })
    }

    /// Refresh the cached [`Self::aabb`] from the current `control_points`
    /// — see [`crate::nurb_curve::NurbCurve::recompute_aabb`]'s identical
    /// doc comment for why this is needed at all (`control_points` is
    /// `pub`, and code outside this crate does mutate it in place).
    pub fn recompute_aabb(&mut self) {
        self.aabb = compute_aabb(&self.control_points);
    }

    /// The diagonal of the box around the control points: by the convex
    /// hull property, a length the whole patch fits within — the size of
    /// the feature it is part of, for choices that should scale with it.
    pub fn size(&self) -> GeopResult<S> {
        self.aabb
            .iter()
            .take(D - 1)
            .fold(S::ZERO, |sum, axis| sum.add(axis.width().mul(axis.width())))
            .sqrt()
    }

    /// Valid parameter range in the u direction: `(u_min, u_max)`.
    pub fn domain_u(&self) -> (S, S) {
        (
            self.knot_vector_u[self.degree_u],
            self.knot_vector_u[self.num_u],
        )
    }

    /// Valid parameter range in the v direction: `(v_min, v_max)`.
    pub fn domain_v(&self) -> (S, S) {
        (
            self.knot_vector_v[self.degree_v],
            self.knot_vector_v[self.num_v],
        )
    }

    /// Where the surface stops being one polynomial piece along `u`: the
    /// ends of its domain and every distinct knot between them.
    pub fn breakpoints_u(&self) -> Vec<S> {
        crate::spline::breakpoints(&self.knot_vector_u, self.degree_u, self.num_u)
    }

    /// Like [`Self::breakpoints_u`], along `v`.
    pub fn breakpoints_v(&self) -> Vec<S> {
        crate::spline::breakpoints(&self.knot_vector_v, self.degree_v, self.num_v)
    }

    /// Whether `other` could be the very same patch: equal degrees, and
    /// knots and homogeneous control points that `could_be_equal` pairwise.
    ///
    /// A test of the representation, so it is exact where it answers `true`
    /// — two patches built the same way, e.g. two copies of one solid — and
    /// says nothing about the same surface parametrized differently.
    pub fn could_be_equal(&self, other: &Self) -> bool {
        self.degree_u == other.degree_u
            && self.degree_v == other.degree_v
            && self.num_u == other.num_u
            && self.num_v == other.num_v
            && self
                .knot_vector_u
                .iter()
                .zip(&other.knot_vector_u)
                .all(|(a, b)| a.could_be_equal(*b))
            && self
                .knot_vector_v
                .iter()
                .zip(&other.knot_vector_v)
                .all(|(a, b)| a.could_be_equal(*b))
            && self
                .control_points
                .iter()
                .zip(&other.control_points)
                .all(|(a, b)| a.could_be_equal(b))
    }

    /// Number of control points in the u direction.
    pub fn num_u(&self) -> usize {
        self.num_u
    }

    /// Number of control points in the v direction.
    pub fn num_v(&self) -> usize {
        self.num_v
    }

    /// Whether this is the [`NurbSurface::everything`] placeholder — the
    /// unsharp stand-in a face carries before it is given real geometry.
    ///
    /// A finished solid must have none: a placeholder face has no position,
    /// so nothing can be classified against it, and it will silently swallow
    /// any containment or intersection query it is handed (every comparison
    /// against `ENTIRE` succeeds). Construction code that splits faces off a
    /// starting placeholder has to consume the last one rather than leave it
    /// behind, and this is how a test says so.
    pub fn is_everything(&self) -> bool {
        self.num_u == 1
            && self.num_v == 1
            && !self.control_points[0][0].is_sharp()
            && self.knot_vector_u.iter().all(|k| !k.is_sharp())
    }

    /// A degenerate, maximally-unsharp surface: a single 1×1 control point
    /// whose every coordinate is [`Scalar::ENTIRE`], over a domain that
    /// accepts any `(u, v)`. `evaluate()` anywhere returns `ENTIRE` in every
    /// coordinate, so it `could_be_equal`s any point — a placeholder for
    /// geometry that is not yet known.
    pub fn everything() -> Self {
        let mut cp = Vector::<S, D>::everything();
        cp[D - 1] = S::ONE;
        NurbSurface {
            degree_u: 0,
            degree_v: 0,
            num_u: 1,
            num_v: 1,
            control_points: vec![cp],
            knot_vector_u: vec![S::ENTIRE, S::ENTIRE],
            knot_vector_v: vec![S::ENTIRE, S::ENTIRE],
            // Matches-anything, same as every other coordinate of this
            // placeholder — no bounding box has been established yet.
            aabb: [S::ENTIRE; 3],
        }
    }
}

impl<S: Scalar> Display for NurbSurface3D<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let p00 = self
            .evaluate(self.knot_vector_u[0], self.knot_vector_v[0])
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        let p01 = self
            .evaluate(self.knot_vector_u[0], self.knot_vector_v[self.num_v])
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        let p10 = self
            .evaluate(self.knot_vector_u[self.num_u], self.knot_vector_v[0])
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        let p11 = self
            .evaluate(
                self.knot_vector_u[self.num_u],
                self.knot_vector_v[self.num_v],
            )
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());

        write!(
            f,
            "NurbSurface((0, 0) -> {}, (1, 0) -> {}, (1, 1) -> {}, (0, 1) -> {})",
            p00, p10, p11, p01
        )
    }
}

#[cfg(test)]
mod tests {
    use super::NurbSurface;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    /// Weights must be definitely positive: zero and negative are rejected.
    fn check_non_positive_weight_is_rejected<S: Scalar>() {
        let f = S::from_f64;
        let pt = |x, y, w| Vector4::from_array([f(x), f(y), f(0.), f(w)]);
        for w in [0., -1.] {
            let result = NurbSurface::<S, 4>::try_new(
                1,
                1,
                vec![
                    pt(0., 0., w),
                    pt(0., 1., 1.),
                    pt(1., 0., 1.),
                    pt(1., 1., 1.),
                ],
                vec![f(0.), f(0.), f(1.), f(1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            );
            assert!(result.is_err(), "weight {w} accepted");
        }
    }
    #[test]
    fn non_positive_weight_is_rejected() {
        for_all_scalars!(check_non_positive_weight_is_rejected);
    }
}
