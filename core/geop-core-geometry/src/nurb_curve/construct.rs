//! The simplest curves, built directly from their points: a straight line
//! ([`NurbCurve3D::line`]) and a single rational quadratic
//! ([`NurbCurve3D::arc`]), both over `[0, 1]`, and the points a clamped
//! curve starts and ends at ([`NurbCurve3D::start`]).
//!
//! Every end point is taken with weight one as it is, never multiplied by
//! it (see [`Vector3::homogeneous`]).

use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::{NurbCurve, NurbCurve2D, NurbCurve3D};

/// The knots of a single Bézier span of `degree` on `[0, 1]`: `degree + 1`
/// zeros, then as many ones.
pub fn bezier_knots<S: Scalar>(degree: usize) -> Vec<S> {
    let mut knots = vec![S::ZERO; degree + 1];
    knots.extend(std::iter::repeat_n(S::ONE, degree + 1));
    knots
}

impl<S: Scalar> NurbCurve3D<S> {
    /// The straight line from `a` to `b`, over `[0, 1]`.
    pub fn line(a: Vector3<S>, b: Vector3<S>) -> GeopResult<Self> {
        NurbCurve::try_new(
            1,
            vec![a.homogeneous(S::ONE), b.homogeneous(S::ONE)],
            bezier_knots(1),
        )
    }

    /// The rational quadratic from `p0` to `p2` over `[0, 1]`, its middle
    /// control point `mid` weighted `w`. With `mid` where the tangents of a
    /// circle at `p0` and `p2` meet and `w` the cosine of half the angle
    /// between them, it is that circle's arc exactly: for a quarter,
    /// `mid = p0 + p2 - centre` and `w =` [`super::sqrt2_over_2`].
    pub fn arc(p0: Vector3<S>, mid: Vector3<S>, p2: Vector3<S>, w: S) -> GeopResult<Self> {
        NurbCurve::try_new(
            2,
            vec![
                p0.homogeneous(S::ONE),
                mid.homogeneous(w),
                p2.homogeneous(S::ONE),
            ],
            bezier_knots(2),
        )
    }

    /// The point the curve starts at, for the clamped knot vectors every
    /// curve here has: its first control point, divided by its weight.
    pub fn start(&self) -> GeopResult<Vector3<S>> {
        self.control_points[0].dehomogenized()
    }

    /// The point the curve ends at: its last control point (see
    /// [`Self::start`]).
    pub fn end(&self) -> GeopResult<Vector3<S>> {
        self.control_points[self.control_points.len() - 1].dehomogenized()
    }
}

impl<S: Scalar> NurbCurve2D<S> {
    /// The straight line from `a` to `b`, over `[0, 1]`.
    pub fn line(a: Vector2<S>, b: Vector2<S>) -> GeopResult<Self> {
        NurbCurve::try_new(
            1,
            vec![a.homogeneous(S::ONE), b.homogeneous(S::ONE)],
            bezier_knots(1),
        )
    }

    /// The rational quadratic from `p0` to `p2` through the weighted
    /// control point `mid` (see [`NurbCurve3D::arc`]).
    pub fn arc(p0: Vector2<S>, mid: Vector2<S>, p2: Vector2<S>, w: S) -> GeopResult<Self> {
        NurbCurve::try_new(
            2,
            vec![
                p0.homogeneous(S::ONE),
                mid.homogeneous(w),
                p2.homogeneous(S::ONE),
            ],
            bezier_knots(2),
        )
    }

    /// The point the curve starts at (see [`NurbCurve3D::start`]).
    pub fn start(&self) -> GeopResult<Vector2<S>> {
        self.control_points[0].dehomogenized()
    }

    /// The point the curve ends at (see [`NurbCurve3D::start`]).
    pub fn end(&self) -> GeopResult<Vector2<S>> {
        self.control_points[self.control_points.len() - 1].dehomogenized()
    }
}
