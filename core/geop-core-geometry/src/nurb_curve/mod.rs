mod area;
pub use area::loop_signed_area;
mod closest;
mod compatible;
mod construct;
pub use construct::bezier_knots;
mod evaluate;
mod extend;
mod extrema;
mod helix;
pub use helix::{Handedness, HelixRow, arc_span, cos_sin, helix_rows, sqrt2_over_2};
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
pub(crate) use translate::transform_control_point;
pub use translate::{embed_point, flatten_point};

use std::fmt::Display;

use crate::{aabb::compute_aabb, knot_insertion::pinned_clamped_end};
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
pub(crate) fn dehomogenize<const D: usize, const C: usize>(
    control_points: &[Vector<D>],
) -> Vec<Vector<C>> {
    control_points
        .iter()
        .map(|p| {
            let inv_w = Scalar::ONE
                .div(p[D - 1])
                .expect("weights are definitely positive by construction");
            let mut pt = Vector::<C>::zero();
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
#[derive(Clone, Debug, PartialEq)]
pub struct NurbCurve<const D: usize> {
    pub degree: usize,
    pub control_points: Vec<Vector<D>>,
    pub knot_vector: Vec<Scalar>,
    /// Cached axis-aligned bounding box of the (dehomogenized) control
    /// points — see [`crate::aabb::compute_aabb`]. Every constructor here
    /// fills this in once, so the intersection search's per-node prefilter
    /// (`aabb_could_overlap`, in `intersection::curve_curve`/
    /// `curve_surface`) never has to recompute it.
    pub(crate) aabb: [Scalar; 3],
}

/// 3-D NURBS curve (homogeneous control points in ℝ⁴).
pub type NurbCurve3D = NurbCurve<4>;

/// 2-D parameter-space NURBS curve / pcurve (homogeneous control points in ℝ³).
pub type NurbCurve2D = NurbCurve<3>;

impl<const D: usize> NurbCurve<D> {
    pub fn try_new(
        degree: usize,
        control_points: Vec<Vector<D>>,
        knot_vector: Vec<Scalar>,
    ) -> GeopResult<Self> {
        let n = control_points.len();
        if knot_vector.len() != n + degree + 1 {
            return Err(GeopError::new(format!(
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
            if !p[D - 1].definitely_greater(Scalar::ZERO) {
                return Err(GeopError::new(format!(
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

    /// The box around the control points, and so around the curve (the
    /// convex hull property): one enclosure per axis, holding every control
    /// point's coordinate there. Only the first `D - 1` axes are the
    /// curve's; a 2-D curve's third is [`Scalar::ENTIRE`].
    pub fn aabb(&self) -> &[Scalar; 3] {
        &self.aabb
    }

    /// Whether the boxes around two curves' control points could overlap:
    /// `false` proves the curves apart.
    pub fn could_overlap(&self, other: &Self) -> bool {
        crate::aabb::aabb_could_overlap(&self.aabb, &other.aabb, D - 1)
    }

    /// Valid parameter range `(start_t, end_t)` of this curve.
    pub fn domain(&self) -> (Scalar, Scalar) {
        let p = self.degree;
        let n = self.control_points.len() - 1;
        (self.knot_vector[p], self.knot_vector[n + 1])
    }

    /// The diagonal of the box around the control points: by the convex
    /// hull property, a length the whole curve fits within (see
    /// `NurbSurface::size`).
    pub fn size(&self) -> GeopResult<Scalar> {
        self.aabb
            .iter()
            .take(D - 1)
            .fold(Scalar::ZERO, |sum, axis| {
                sum.add(axis.width().mul(axis.width()))
            })
            .sqrt()
    }

    /// The middle of [`Self::domain`], sharp: a parameter inside the curve
    /// for a sample, a seed or a split point. Any value in the enclosure of
    /// the middle serves as well, so it is a free choice, and sharpening it
    /// loses nothing (see AGENTS.md).
    pub fn mid_parameter(&self) -> GeopResult<Scalar> {
        let (t0, t1) = self.domain();
        Ok(t0.add(t1).div(Scalar::TWO)?.sharpen())
    }

    /// Where the curve stops being one polynomial piece: the ends of its
    /// domain and every distinct knot between them (see
    /// [`crate::spline::breakpoints`]).
    pub fn breakpoints(&self) -> Vec<Scalar> {
        crate::spline::breakpoints(&self.knot_vector, self.degree, self.control_points.len())
    }

    /// If `hat` pins the parameter sharply onto an end of the domain that
    /// is clamped (see [`crate::knot_insertion::pinned_clamped_end`]), the
    /// curve's point there: its end control point, in `C = D - 1`
    /// Cartesian coordinates.
    pub(crate) fn pinned_end<const C: usize>(&self, hat: Scalar) -> Option<Vector<C>> {
        let n = self.control_points.len();
        let first = pinned_clamped_end(hat, &self.knot_vector, n, self.degree)?;
        let end = self.control_points[if first { 0 } else { n - 1 }];
        Some(dehomogenize::<D, C>(&[end])[0])
    }

    pub fn domain_as_scalar(&self) -> Scalar {
        let (s, e) = self.domain();
        s.union(e)
    }

    /// A degenerate, maximally-unsharp curve: a single control point whose
    /// every coordinate is [`Scalar::ENTIRE`], over a domain that accepts
    /// any parameter. `evaluate()` at any `t` returns `ENTIRE` in every
    /// coordinate, so it `could_be_equal`s any point — a placeholder for
    /// geometry that is not yet known.
    pub fn everything() -> Self {
        let mut cp = Vector::<D>::everything();
        cp[D - 1] = Scalar::ONE;
        NurbCurve {
            degree: 0,
            control_points: vec![cp],
            knot_vector: vec![Scalar::ENTIRE, Scalar::ENTIRE],
            // Matches-anything, same as every other coordinate of this
            // placeholder — no bounding box has been established yet.
            aabb: [Scalar::ENTIRE; 3],
        }
    }
}

impl<const D: usize> Display for NurbCurve<D> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let at = |t: Scalar| {
            let mut p = [Scalar::ZERO; D];
            match self.evaluate_into(t, &mut p[..D - 1]) {
                Ok(()) => {
                    let coordinates: Vec<String> =
                        p[..D - 1].iter().map(Scalar::to_string).collect();
                    format!("Vector{}({})", D - 1, coordinates.join(", "))
                }
                Err(_) => "N/A".to_string(),
            }
        };
        let (start, end) = self.domain();
        write!(f, "NurbCurve({} -> {})", at(start), at(end))
    }
}

#[cfg(test)]
mod tests {
    use super::NurbCurve;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    /// Weights must be definitely positive: zero and negative are rejected.
    #[test]
    fn non_positive_weight_is_rejected() {
        let f = Scalar::from_f64;
        for w in [0., -1.] {
            let result = NurbCurve::<4>::try_new(
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
}
