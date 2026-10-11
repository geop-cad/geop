//! [`Profile`], the named curve chain extrude and revolve sweep, and the
//! simplest profiles: polygons, polylines and circles. The curves are the
//! kernel's own (see `geop_core_geometry::nurb_curve`). The topology itself
//! is described whole and built in one go, see [`crate::sweep`].

use geop_core_geometry::nurb_curve::{NurbCurve2D, sqrt2_over_2};
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

// ── Profiles ────────────────────────────────────────────────────────────────

/// The named curve chain extrude and revolve sweep.
pub use geop_ops::operation::Profile;

// ── Profile shapes ───────────────────────────────────────────────────────────

/// The closed polygon through `points` (and back to the first), one line
/// per side — the simplest profile loop for [`crate::extrude::extrude`].
pub fn polygon<S: Scalar>(points: &[Vector2<S>]) -> GeopResult<Vec<NurbCurve2D<S>>> {
    (0..points.len())
        .map(|i| NurbCurve2D::line(points[i], points[(i + 1) % points.len()]))
        .collect()
}

/// The open chain of lines through `points` — the simplest profile for
/// [`crate::revolve::revolve_at_oriented`].
pub fn polyline<S: Scalar>(points: &[Vector2<S>]) -> GeopResult<Vec<NurbCurve2D<S>>> {
    points
        .windows(2)
        .map(|w| NurbCurve2D::line(w[0], w[1]))
        .collect()
}

/// A circle of radius `r` around `(cx, cy)` as four quarter arcs,
/// counter-clockwise.
pub fn circle_at<S: Scalar>(cx: f64, cy: f64, r: f64) -> GeopResult<Vec<NurbCurve2D<S>>> {
    let at = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
    let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
    (0..4)
        .map(|i| {
            let (a, b) = (q[i], q[(i + 1) % 4]);
            NurbCurve2D::arc(
                at(cx + a.0, cy + a.1),
                at(cx + a.0 + b.0, cy + a.1 + b.1),
                at(cx + b.0, cy + b.1),
                sqrt2_over_2(),
            )
        })
        .collect()
}

/// What the tests of this crate share.
#[cfg(test)]
pub(crate) mod testing {
    use geop_core_geometry::nurb_curve::NurbCurve2D;
    use geop_core_math::{
        primitives::CoordinateSystem,
        scalars::Scalar,
        vector::{Vector2, Vector3},
    };
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    use super::polygon;

    pub fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    pub fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A circle of radius `r` around `(cx, cy)` as four quarter arcs,
    /// counter-clockwise (reversed: clockwise, for a hole).
    pub fn circle_at<S: Scalar>(cx: f64, cy: f64, r: f64, clockwise: bool) -> Vec<NurbCurve2D<S>> {
        let arcs = super::circle_at(cx, cy, r).unwrap();
        match clockwise {
            true => arcs.iter().rev().map(|c| c.reverse()).collect(),
            false => arcs,
        }
    }

    /// A circle of radius `r` around the origin, as four quarter arcs,
    /// counter-clockwise.
    pub fn circle<S: Scalar>(r: f64) -> Vec<NurbCurve2D<S>> {
        circle_at(0.0, 0.0, r, false)
    }

    /// The square of half side `half` around the origin, counter-clockwise.
    pub fn square<S: Scalar>(half: f64) -> Vec<NurbCurve2D<S>> {
        polygon(&[
            v2(-half, -half),
            v2(half, -half),
            v2(half, half),
            v2(-half, half),
        ])
        .unwrap()
    }

    /// The plane through `origin` spanned by `u` and `v`.
    pub fn plane<S: Scalar>(
        origin: Vector3<S>,
        u: Vector3<S>,
        v: Vector3<S>,
    ) -> CoordinateSystem<S> {
        CoordinateSystem::try_new(origin, u, v, u.prod_cross(&v)).unwrap()
    }

    /// Checks `model` is a valid manifold model.
    pub fn assert_valid<S: Scalar>(model: &Model<S>) {
        if let Err(e) = validate_manifold(&ValidationParameters::default(), model) {
            panic!("{e:?}");
        }
    }

    /// Checks `model`, which may hold sheets, is valid.
    pub fn assert_valid_sheets<S: Scalar>(model: &Model<S>) {
        if let Err(e) = validate(&ValidationParameters::default(), model) {
            panic!("{e:?}");
        }
    }

    /// Draws `model`'s topology, `resolution` samples per curve, into
    /// `outputs/{file}`, checking it draws points, lines, faces and labels.
    pub fn render<S: Scalar>(model: &Model<S>, resolution: usize, file: &str) {
        let scene = geop_ops_rasterize::debug::rasterize_topology(model, resolution).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles_transparent.is_empty());
        assert!(!scene.labels.is_empty());
        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file(&format!("outputs/{file}")).unwrap();
    }
}
