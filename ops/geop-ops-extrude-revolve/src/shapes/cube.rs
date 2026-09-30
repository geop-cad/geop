//! An axis-aligned box, built by [`extrude`]-ing its bottom face straight up
//! — no vertex/edge is ever duplicated then welded, since `extrude` shares
//! every side wall's edges with its top/bottom caps by construction (see its
//! own module docs).

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

/// A box solid spanning `[min, max]`, built as `extrude`'s start cap (the
/// unit square in `(u, v)`, at `z = max.z`) swept down to `min.z`.
///
/// Named as the operation `cube(name)`, after that square: its corners
/// `p0..p3` (counter-clockwise from `min` as seen from above) and sides
/// `c0..c3`, see [`ExtrudeNames`].
pub fn cube_solid<S: Scalar>(
    part: &mut Part<S>,
    name: &str,
    min: Vector3<S>,
    max: Vector3<S>,
) -> GeopResult<SolidId> {
    let (x0, y0, z0) = (min[0], min[1], max[2]);
    let (x1, y1) = (max[0], max[1]);

    // The coordinate system's `u`/`v` span the bottom footprint (scaled to
    // the box's own `x`/`y` extents, so `outer` is the unit square) and `w`
    // is the extrude direction.
    let origin = Vector3::from_array([x0, y0, z0]);
    let u = Vector3::from_array([x1.sub(x0), S::ZERO, S::ZERO]);
    let v = Vector3::from_array([S::ZERO, y1.sub(y0), S::ZERO]);
    let w = Vector3::from_array([S::ZERO, S::ZERO, min[2].sub(z0)]);
    let coordinate_system = CoordinateSystem::try_new(origin, u, v, w)?;

    // CCW in `(u, v)`, so the bottom cap's outward (downward) normal comes
    // out right once `extrude` sweeps it upward.
    let outer = [
        Vector2::from_array([S::ZERO, S::ZERO]),
        Vector2::from_array([S::ONE, S::ZERO]),
        Vector2::from_array([S::ONE, S::ONE]),
        Vector2::from_array([S::ZERO, S::ONE]),
    ];

    let namer = Namer::new("cube", name)?;
    extrude(
        part,
        &ExtrudeNames::single(&namer),
        &coordinate_system,
        &Profile::closed(polygon(&outer)?),
        &[],
    )
}

#[cfg(test)]
mod tests {
    use super::cube_solid;
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};
    use geop_core_topology::contains::shell::{PointClassification, shell_contains};
    use geop_ops::Part;

    const MAX: usize = 200;
    const EPS: f64 = 1e-3;
    const SEED: u64 = 7;

    fn check_cube_has_expected_entity_counts<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t1",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        // 6 faces, 8 shared corner vertices, 12 shared edges (each with
        // exactly 2 coedges, one per adjoining face) — `extrude` shares
        // vertices/edges between its side walls and caps directly, no
        // welding needed.
        assert_eq!(model.faces.len(), 6);
        assert_eq!(model.vertices.len(), 8);
        assert_eq!(model.edges.len(), 12);
        assert_eq!(model.coedges.len(), 24);
        assert_eq!(model.shells.len(), 1);
        assert_eq!(model.solids.len(), 1);
        for face in model.faces.values() {
            assert!(face.holes.is_empty());
        }
        for edge_id in model.edges.keys() {
            assert_eq!(model.coedges_of_edge(*edge_id).len(), 2);
        }
    }
    #[test]
    fn cube_has_expected_entity_counts() {
        for_all_scalars!(check_cube_has_expected_entity_counts);
    }

    fn check_cube_center_is_inside<S: Scalar>() {
        let mut part = Part::<S>::new();
        let solid_id = cube_solid(
            &mut part,
            "t2",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let p = Vector3::from_array([S::from_f64(0.5); 3]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::Inside
        );
    }
    #[test]
    fn cube_center_is_inside() {
        for_all_scalars!(check_cube_center_is_inside);
    }

    fn check_cube_outside_point_is_outside<S: Scalar>() {
        let mut part = Part::<S>::new();
        let solid_id = cube_solid(
            &mut part,
            "t3",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let p = Vector3::from_array([S::from_f64(-5.0), S::from_f64(0.5), S::from_f64(0.5)]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::Outside
        );
    }
    #[test]
    fn cube_outside_point_is_outside() {
        for_all_scalars!(check_cube_outside_point_is_outside);
    }

    fn check_rasterize_topology_cube<S: Scalar>() {
        let mut part = Part::<S>::new();
        cube_solid(
            &mut part,
            "t4",
            Vector3::from_array([S::ZERO; 3]),
            Vector3::from_array([S::ONE; 3]),
        )
        .unwrap();
        let model = part.topology();

        let scene = geop_ops_rasterize::debug::rasterize_topology(&model, 8).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles_transparent.is_empty());
        assert!(!scene.labels.is_empty());

        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file("outputs/cube_topology.html").unwrap();
    }
    #[test]
    fn rasterize_topology_cube() {
        for_all_scalars!(check_rasterize_topology_cube);
    }
}
