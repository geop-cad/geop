mod closest;
mod compatible;
mod evaluate;
mod interpolate;
mod length;
mod refine;
pub use interpolate::true_point_fractions;
pub use refine::ParameterRefinable;
mod reverse;
mod split;
mod sweep;
mod tangent;
mod translate;

use std::fmt::Display;

use crate::aabb::compute_aabb;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

/// Dehomogenize `control_points` (`D`-dimensional homogeneous, last component
/// the weight) into `C`-dimensional Cartesian points (`C` = `D - 1`, passed
/// explicitly since Rust's stable const generics can't express `D - 1` in a
/// single generic parameter's bound).
///
/// Infallible: every weight of a `NurbCurve`/`NurbSurface` is definitely
/// positive by construction (see `NurbCurve::try_new`), so the division
/// cannot fail. `pub(crate)`: surfaces and the intersection searches need
/// the exact same points.
pub(crate) fn dehomogenize<S: Scalar, const D: usize, const C: usize>(
    control_points: &[Vector<S, D>],
) -> Vec<Vector<S, C>> {
    control_points
        .iter()
        .map(|p| {
            let inv_w = S::ONE
                .div(p[D - 1])
                .expect("weights are definitely positive by construction");
            let mut pt = Vector::<S, C>::zero();
            for c in 0..C {
                pt[c] = p[c].mul(inv_w);
            }
            pt
        })
        .collect()
}

/// A NURBS curve whose control points live in `D`-dimensional homogeneous space.
///
/// - `D = 4`: 3-D curve  — control points are `(wx, wy, wz, w)`.
/// - `D = 3`: 2-D curve (pcurve) — control points are `(wu, wv, w)`.
#[derive(Clone, Debug)]
pub struct NurbCurve<S: Scalar, const D: usize> {
    pub degree: usize,
    pub control_points: Vec<Vector<S, D>>,
    pub knot_vector: Vec<S>,
    /// Cached axis-aligned bounding box of the (dehomogenized) control
    /// points — see [`crate::aabb::compute_aabb`]. Every constructor here
    /// fills this in once, so the intersection search's per-node prefilter
    /// (`aabb_could_overlap`, in `intersection::curve_curve`/
    /// `curve_surface`) never has to recompute it.
    pub(crate) aabb: [S; 3],
}

/// 3-D NURBS curve (homogeneous control points in ℝ⁴).
pub type NurbCurve3D<S> = NurbCurve<S, 4>;

/// 2-D parameter-space NURBS curve / pcurve (homogeneous control points in ℝ³).
pub type NurbCurve2D<S> = NurbCurve<S, 3>;

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    pub fn try_new(
        degree: usize,
        control_points: Vec<Vector<S, D>>,
        knot_vector: Vec<S>,
    ) -> GeopResult<Self> {
        let n = control_points.len();
        if knot_vector.len() != n + degree + 1 {
            return Err(GeopError::new(&format!(
                "Invalid knot vector length: expected {}, got {}",
                n + degree + 1,
                knot_vector.len()
            )));
        }
        // Every weight must be definitely positive. That is what makes
        // `W(t) = Σ w_i N_i(t) > 0` on the whole domain, which the convex hull
        // property and `contains::curve`'s division-free per-axis functions
        // `X_k(t) - p_k W(t)` rely on. Zero or negative weights have no use
        // case here, so they are rejected at construction rather than
        // surfacing as a division by zero far downstream. `split` forms
        // convex combinations of existing control points and so preserves
        // this.
        for p in &control_points {
            if !p[D - 1].definitely_greater(S::ZERO) {
                return Err(GeopError::new(&format!(
                    "NurbCurve::try_new: control point {p:?} has a weight that is not definitely positive"
                )));
            }
        }
        let aabb = compute_aabb(&control_points);
        Ok(Self {
            degree,
            control_points,
            knot_vector,
            aabb,
        })
    }

    /// Refresh the cached [`Self::aabb`] from the current `control_points`.
    ///
    /// Every constructor in this module keeps `aabb` in sync automatically,
    /// but `control_points` is a `pub` field and at least one caller outside
    /// this crate (`Model::reverse_face`, mirroring a pcurve's control
    /// points in place to flip a face) legitimately mutates it directly
    /// rather than building a new curve — call this afterwards or the
    /// cached box silently goes stale and the intersection search's
    /// `aabb_could_overlap` prefilter starts pruning real overlaps.
    pub fn recompute_aabb(&mut self) {
        self.aabb = compute_aabb(&self.control_points);
    }

    /// Valid parameter range `(start_t, end_t)` of this curve.
    pub fn domain(&self) -> (S, S) {
        let p = self.degree;
        let n = self.control_points.len() - 1;
        (self.knot_vector[p], self.knot_vector[n + 1])
    }

    /// Where the curve stops being one polynomial piece: the ends of its
    /// domain and every distinct knot between them (see
    /// [`crate::spline::breakpoints`]).
    pub fn breakpoints(&self) -> Vec<S> {
        crate::spline::breakpoints(&self.knot_vector, self.degree, self.control_points.len())
    }

    pub fn domain_as_scalar(&self) -> S {
        let (s, e) = self.domain();
        s.union(e)
    }

    /// A degenerate, maximally-unsharp curve: a single control point whose
    /// every coordinate is [`Scalar::ENTIRE`], over a domain that accepts
    /// any parameter. `evaluate()` at any `t` returns `ENTIRE` in every
    /// coordinate, so it `could_be_equal`s any point — a placeholder for
    /// geometry that is not yet known.
    pub fn everything() -> Self {
        let mut cp = Vector::<S, D>::everything();
        cp[D - 1] = S::ONE;
        NurbCurve {
            degree: 0,
            control_points: vec![cp],
            knot_vector: vec![S::ENTIRE, S::ENTIRE],
            // Matches-anything, same as every other coordinate of this
            // placeholder — no bounding box has been established yet.
            aabb: [S::ENTIRE; 3],
        }
    }
}

impl<S: Scalar> Display for NurbCurve2D<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let start = self
            .evaluate(self.domain().0)
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        let end = self
            .evaluate(self.domain().1)
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        write!(f, "NurbCurve({} -> {})", start, end)
    }
}

impl<S: Scalar> Display for NurbCurve3D<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let start = self
            .evaluate(self.domain().0)
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        let end = self
            .evaluate(self.domain().1)
            .map(|p| p.to_string())
            .unwrap_or_else(|_| "N/A".to_string());
        write!(f, "NurbCurve({} -> {})", start, end)
    }
}

#[cfg(test)]
mod tests {
    use super::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    /// Weights must be definitely positive: zero and negative are rejected.
    fn check_non_positive_weight_is_rejected<S: Scalar>() {
        let f = S::from_f64;
        for w in [0., -1.] {
            let result = NurbCurve::<S, 4>::try_new(
                1,
                vec![
                    Vector4::from_array([f(0.), f(0.), f(0.), f(w)]),
                    Vector4::from_array([f(1.), f(0.), f(0.), f(1.)]),
                ],
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
