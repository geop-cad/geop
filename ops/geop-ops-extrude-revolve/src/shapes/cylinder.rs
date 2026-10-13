//! A capped circular cylinder, built two different ways: [`revolved_cylinder`]
//! (via [`revolve_at_oriented`], an exact circular cross-section) and
//! [`extruded_cylinder`] (via [`extrude`], an `n`-gon approximating one).

use crate::{
    common::{Profile, polygon, polyline},
    extrude::extrude,
    revolve::revolve_at_oriented,
    sweep::SweepLoop,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::SolidId;
use geop_ops_model::{NamedModel, Namer};

/// Which axis a cylinder's own axis runs parallel to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// A capped cylinder of `radius` and `height`, with its axis parallel to the
/// z-axis through `base_center` (the center of its *bottom* cap — the solid
/// spans `z in [base_center.z, base_center.z + height]`), as an exact
/// circular cross-section swept via [`revolve_at_oriented`].
///
/// [`revolved_cylinder_along_axis`] with `axis = Axis::Z`.
pub fn revolved_cylinder<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    base_center: Vector3<S>,
    radius: S,
    height: S,
) -> GeopResult<SolidId> {
    revolved_cylinder_along_axis(model, name, base_center, radius, height, Axis::Z)
}

/// Like [`revolved_cylinder`], but with its axis parallel to `axis` instead
/// of always z — `base_center` is still the center of the *bottom* cap
/// (the cap nearer the origin along `axis`'s own positive direction), so
/// the solid spans `[0, height]` along `axis` from there.
///
/// Built as the same 3-segment revolve profile `revolved_cylinder` always
/// used, swept via [`revolve_at_oriented`] around a `CoordinateSystem`
/// oriented to `axis`. Each orientation's `u`/`v`/`w` basis is chosen as a cyclic permutation of the
/// standard one (`(x, y, z) -> (y, z, x) -> (z, x, y)`), so `u x v = w`
/// always holds, as for the world's axes, and every axis produces a solid with outward-facing
/// normals, never one revolved "inside out".
///
/// Named as the operation `cylinder(name)`, after that profile (see
/// [`revolve_at_oriented`]): `p0`/`p3` are the centers of the top/bottom
/// caps, `c0` the top cap's radius, `c1` the side and `c2` the bottom cap's
/// radius.
pub fn revolved_cylinder_along_axis<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    base_center: Vector3<S>,
    radius: S,
    height: S,
    axis: Axis,
) -> GeopResult<SolidId> {
    let [x, y, z] = [0, 1, 2].map(Vector3::axis);
    let (u, v, w) = match axis {
        Axis::X => (y, z, x),
        Axis::Y => (z, x, y),
        Axis::Z => (x, y, z),
    };
    let coordinate_system = CoordinateSystem::try_new(base_center, u, v, w)?;

    // Top-down, as a full turn takes its profile (see
    // [`revolve_at_oriented`]). Running it bottom-up instead revolves the
    // solid inside-out: structurally perfect, with every face's normal
    // pointing into the material.
    let profile = polyline(&[
        Vector2::from_array([S::ZERO, height]),
        Vector2::from_array([radius, height]),
        Vector2::from_array([radius, S::ZERO]),
        Vector2::from_array([S::ZERO, S::ZERO]),
    ])?;
    let namer = Namer::new("cylinder", name)?;
    revolve_at_oriented(
        model,
        &namer,
        &namer.root(),
        &Profile::open(profile),
        &coordinate_system,
    )
}

/// A capped cylinder of `radius` and `height`, with its axis parallel to the
/// z-axis through `base_center`, as a regular `segments`-gon prism
/// approximating a circular cross-section, swept via `extrude`.
///
/// Its plane's `w` is `height` down along `z`, so the prism reaches down
/// from `base_center`; [`extrude`] points its faces outwards whichever way
/// `w` runs.
///
/// Named as the operation `cylinder(name)`, after the polygon's corners
/// `p0..` and sides `c0..` (see [`extrude`]).
pub fn extruded_cylinder<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    base_center: Vector3<S>,
    radius: S,
    height: S,
    segments: usize,
) -> GeopResult<SolidId> {
    if segments < 3 {
        return Err(geop_core_math::geop_error::GeopError::new(
            "extruded_cylinder: need at least 3 segments",
        ));
    }
    let u = Vector3::from_array([radius, S::ZERO, S::ZERO]);
    let v = Vector3::from_array([S::ZERO, radius, S::ZERO]);
    let w = Vector3::from_array([S::ZERO, S::ZERO, height.mul(S::from_f64(-1.0))]);
    let coordinate_system = CoordinateSystem::try_new(base_center, u, v, w)?;

    let outer: Vec<Vector2<S>> = (0..segments)
        .map(|k| {
            let angle = std::f64::consts::TAU * (k as f64) / (segments as f64);
            Vector2::from_array([S::from_f64(angle.cos()), S::from_f64(angle.sin())])
        })
        .collect();

    let namer = Namer::new("cylinder", name)?;
    let built = extrude(
        model,
        &namer,
        Some(&namer.root()),
        &coordinate_system,
        S::ZERO,
        S::ONE,
        &[SweepLoop::plain(Profile::closed(polygon(&outer)?))],
    )?;
    Ok(built.solid.expect("extruded as a solid"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::testing::render;
    use geop_core_math::for_all_scalars;
    use geop_ops_model::testing::{assert_model_valid, assert_model_valid_sheets};

    fn check_revolved_cylinder_is_valid<S: Scalar>() {
        let mut model = NamedModel::<S>::new();
        revolved_cylinder(
            &mut model,
            "t2",
            Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]),
            S::ONE,
            S::TWO,
        )
        .unwrap();
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 12);

        assert_model_valid(topology);
        render(topology, 32, "revolved_cylinder_topology.html");
    }
    #[test]
    fn revolved_cylinder_is_valid() {
        for_all_scalars!(check_revolved_cylinder_is_valid);
    }

    fn check_extruded_cylinder_is_valid<S: Scalar>() {
        let mut model = NamedModel::<S>::new();
        extruded_cylinder(
            &mut model,
            "t4",
            Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]),
            S::ONE,
            S::TWO,
            12,
        )
        .unwrap();
        let topology = model.topology();

        assert_model_valid(topology);
        render(topology, 8, "extruded_cylinder_topology.html");
        assert_eq!(topology.faces.len(), 12 + 2);
    }
    #[test]
    fn extruded_cylinder_is_valid() {
        for_all_scalars!(check_extruded_cylinder_is_valid);
    }

    /// An `X`/`Y`-axis cylinder must be structurally identical to the
    /// (already-covered) `Z`-axis one — same face count, valid, manifold —
    /// and its vertices must actually extend along the requested axis
    /// (`height`) with the circular cross-section in the *other* two
    /// coordinates, not still sitting on `z`.
    fn check_revolved_cylinder_along_axis_is_valid<S: Scalar>() {
        for (axis, extent_axis) in [(Axis::X, 0), (Axis::Y, 1), (Axis::Z, 2)] {
            let mut model = NamedModel::<S>::new();
            revolved_cylinder_along_axis(
                &mut model,
                "t1",
                Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]),
                S::ONE,
                S::TWO,
                axis,
            )
            .unwrap_or_else(|e| panic!("{axis:?}: {e}"));
            let topology = model.topology();
            assert_eq!(topology.faces.len(), 12, "{axis:?}");

            assert_model_valid(topology);

            // Every vertex's coordinate along the *other* two axes stays
            // within the radius; `extent_axis` alone reaches all the way
            // to `height`.
            let max_extent = topology
                .vertices
                .values()
                .map(|v| v.point[extent_axis].to_f64())
                .fold(0.0_f64, f64::max);
            assert!(
                (max_extent - 2.0).abs() < 1e-6,
                "{axis:?}: max extent along its own axis = {max_extent}"
            );
            for other in 0..3 {
                if other == extent_axis {
                    continue;
                }
                let max_other = topology
                    .vertices
                    .values()
                    .map(|v| v.point[other].to_f64().abs())
                    .fold(0.0_f64, f64::max);
                assert!(
                    max_other <= 1.0 + 1e-6,
                    "{axis:?}: vertex strayed to {max_other} on axis {other}, radius is 1"
                );
            }
        }
    }
    #[test]
    fn revolved_cylinder_along_axis_is_valid() {
        for_all_scalars!(check_revolved_cylinder_along_axis_is_valid);
    }

    /// A clip-sized cylinder (radius 3, 10 long) standing 50 along its own
    /// axis from the origin, along each axis, is valid. In fixed point its
    /// two caps were reported overlapping: evaluated at the wide parameter
    /// box that projecting a point near the axis onto a cap honestly gives,
    /// each cap's height came out tens wide, as wide as its distance from
    /// the origin (see `nurb_surface::evaluate`).
    fn check_revolved_cylinder_far_along_its_axis_is_valid<S: Scalar>() {
        let f = S::from_f64;
        for (axis, k) in [(Axis::X, 0), (Axis::Y, 1), (Axis::Z, 2)] {
            let mut base = [f(0.0); 3];
            base[k] = f(50.0);
            let mut model = NamedModel::<S>::new();
            revolved_cylinder_along_axis(
                &mut model,
                "clip",
                Vector3::from_array(base),
                f(3.0),
                f(10.0),
                axis,
            )
            .unwrap();
            assert_model_valid_sheets(model.topology());
        }
    }
    #[test]
    fn revolved_cylinder_far_along_its_axis_is_valid() {
        for_all_scalars!(check_revolved_cylinder_far_along_its_axis_is_valid);
    }

    /// The same cylinder far off its axis, at `(1000, -500, 50)`, is valid.
    ///
    /// In fixed point it was not, along `Y` and `Z`: validation's edge x
    /// face check of each radial edge of a cap against the two quarters of
    /// that cap it only touches at the centre (their pole, where every `u`
    /// is a solution) exhausted `curve_surface_crossings`' node budget. Its
    /// bisection split `u` below the width the fixed-point weight carries
    /// once the edge and `v` could not be split further (see
    /// `fat_line::stalled` and the curve-surface test
    /// `spoke_touching_cap_quadrant_at_its_pole_far_from_the_origin`).
    fn check_revolved_cylinder_far_from_the_origin_is_valid<S: Scalar>() {
        let f = S::from_f64;
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            let mut model = NamedModel::<S>::new();
            revolved_cylinder_along_axis(
                &mut model,
                "clip",
                Vector3::from_array([f(1000.0), f(-500.0), f(50.0)]),
                f(3.0),
                f(10.0),
                axis,
            )
            .unwrap();
            assert_model_valid_sheets(model.topology());
        }
    }
    #[test]
    fn revolved_cylinder_far_from_the_origin_is_valid() {
        for_all_scalars!(check_revolved_cylinder_far_from_the_origin_is_valid);
    }
}
