//! A "figure-8" / dumbbell solid with two rectangular holes running all the
//! way through it.
//!
//! This is just an outer footprint and two hole footprints (a dumbbell
//! outline — two square "lobes" joined by a narrow neck — with a small
//! square hole in each lobe) handed to [`extrude`], which does all the
//! actual euler-operator work (including the holes, all the way through
//! both caps and their own side walls).

use crate::{
    common::{Profile, polygon},
    extrude::{ExtrudeNames, extrude},
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::SolidId;
use geop_ops::{Namer, Part};

/// The outer boundary of the figure-8 profile, as a CCW polygon in `(u, v) ∈
/// [0, 1]^2` parameter space.
pub fn outer_polygon<S: Scalar>() -> Vec<Vector2<S>> {
    let c = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
    vec![
        c(0.0, 0.0),
        c(3.0 / 8.0, 0.0),
        c(3.0 / 8.0, 1.0 / 3.0),
        c(5.0 / 8.0, 1.0 / 3.0),
        c(5.0 / 8.0, 0.0),
        c(1.0, 0.0),
        c(1.0, 1.0),
        c(5.0 / 8.0, 1.0),
        c(5.0 / 8.0, 2.0 / 3.0),
        c(3.0 / 8.0, 2.0 / 3.0),
        c(3.0 / 8.0, 1.0),
        c(0.0, 1.0),
    ]
}

/// The two square holes (one per lobe), each as a CW polygon in `(u, v)`
/// parameter space.
pub fn hole_polygons<S: Scalar>() -> [Vec<Vector2<S>>; 2] {
    let c = |x: f64, y: f64| Vector2::from_array([S::from_f64(x), S::from_f64(y)]);
    [
        vec![
            c(0.125, 1.0 / 3.0),
            c(0.125, 2.0 / 3.0),
            c(0.25, 2.0 / 3.0),
            c(0.25, 1.0 / 3.0),
        ],
        vec![
            c(0.75, 1.0 / 3.0),
            c(0.75, 2.0 / 3.0),
            c(0.875, 2.0 / 3.0),
            c(0.875, 1.0 / 3.0),
        ],
    ]
}

/// Build the figure-8-with-2-holes solid: `outer_polygon` extruded by one
/// unit along `+z`, with `hole_polygons` cut all the way through. Returns
/// the new solid.
///
/// Named as the operation `figure8(name)`, after the outline's corners
/// `p0..` and sides `c0..` and the holes' `h0p0..`, `h1c0..` (see
/// [`ExtrudeNames`]).
pub fn figure8_profile<S: Scalar>(part: &mut Part<S>, name: &str) -> GeopResult<SolidId> {
    // Left-handed (`u x v = -w`): `extrude` extrudes a CCW `outer` polygon
    // backwards along `w`, so a CCW boundary with outward-facing normals
    // needs `w` pointing the opposite way `u x v` (i.e. a right-handed
    // basis's own `+z`) would.
    let origin = Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]);
    let u = Vector3::from_array([S::ONE, S::ZERO, S::ZERO]);
    let v = Vector3::from_array([S::ZERO, S::ONE, S::ZERO]);
    let w = Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(-1.0)]);
    let coordinate_system = CoordinateSystem::try_new(origin, u, v, w)?;

    let outer = Profile::closed(polygon(&outer_polygon::<S>())?);
    let holes = hole_polygons::<S>()
        .iter()
        .enumerate()
        .map(|(k, h)| Ok(Profile::closed(polygon(h)?).with_prefix(&format!("h{k}"))))
        .collect::<GeopResult<Vec<_>>>()?;
    let namer = Namer::new("figure8", name)?;
    extrude(
        part,
        &ExtrudeNames::single(&namer),
        &coordinate_system,
        &outer,
        &holes,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;
    use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};

    /// How many faces the figure-8 has, so the orientation counts below can
    /// be read as a fraction rather than a bare number.
    fn check_figure8_face_count<S: Scalar>() {
        let mut part = Part::<S>::new();
        figure8_profile(&mut part, "t").unwrap();
        let model = part.topology();
        assert_eq!(model.faces.len(), 22);
    }
    #[test]
    fn figure8_face_count() {
        for_all_scalars!(check_figure8_face_count);
    }

    fn check_figure8_cap_normals_point_outward<S: Scalar>() {
        let mut part = Part::<S>::new();
        figure8_profile(&mut part, "t").unwrap();
        let model = part.topology();
        for face in model.faces.values() {
            let (u0, u1) = face.surface.domain_u();
            let (v0, v1) = face.surface.domain_v();
            let mid_u = u0.add(u1).div(S::from_f64(2.0)).unwrap();
            let mid_v = v0.add(v1).div(S::from_f64(2.0)).unwrap();
            let p = face.surface.evaluate(mid_u, mid_v).unwrap();
            let n = face.surface.normal(mid_u, mid_v).unwrap();
            // Only the flat caps are exactly horizontal (z is ~constant
            // across the whole surface) — side walls aren't.
            let p_at_00 = face.surface.evaluate(u0, v0).unwrap();
            if !p[2].could_be_equal(p_at_00[2]) {
                continue;
            }
            // bottom cap (z=0): outward normal should point +z (material
            // is below, at z<0). top cap (z=-1): outward should point -z.
            let expect_positive_z = p[2].to_f64().abs() < 0.5;
            assert_eq!(
                n[2].to_f64() > 0.0,
                expect_positive_z,
                "cap at z={} has normal z-component {}",
                p[2].to_f64(),
                n[2].to_f64()
            );
        }
    }
    #[test]
    fn figure8_cap_normals_point_outward() {
        for_all_scalars!(check_figure8_cap_normals_point_outward);
    }

    fn check_figure8_profile_is_valid<S: Scalar>() {
        let mut part = Part::<S>::new();
        figure8_profile(&mut part, "t").unwrap();
        let model = part.topology();

        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, &model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, &model) {
            panic!("{e:?}");
        }
    }
    #[test]
    fn figure8_profile_is_valid() {
        for_all_scalars!(check_figure8_profile_is_valid);
    }

    fn check_rasterize_topology_figure8<S: Scalar>() {
        let mut part = Part::<S>::new();
        figure8_profile(&mut part, "t").unwrap();
        let model = part.topology();

        let scene = geop_ops_rasterize::debug::rasterize_topology(&model, 8).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles_transparent.is_empty());
        assert!(!scene.labels.is_empty());

        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file("outputs/figure8_topology.html").unwrap();
    }
    #[test]
    fn rasterize_topology_figure8() {
        for_all_scalars!(check_rasterize_topology_figure8);
    }
}
