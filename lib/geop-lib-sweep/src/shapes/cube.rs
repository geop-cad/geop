//! An axis-aligned box, built by [`extrude`]-ing its bottom face straight up
//! — no vertex/edge is ever duplicated then welded, since `extrude` shares
//! every side wall's edges with its top/bottom caps by construction (see its
//! own module docs).

use crate::{common::polygon, extrude::extrude, sweep::SweepLoop};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::SolidId;
use geop_ops_model::{NamedModel, Namer, Profile};

/// A box solid spanning `[min, max]`, built as `extrude`'s start cap (the
/// unit square in `(u, v)`, at `z = max.z`) swept down to `min.z`.
///
/// Named as the operation `cube(name)`, after that square: its corners
/// `p0..p3` (counter-clockwise from `min` as seen from above) and sides
/// `c0..c3`, see [`extrude`].
pub fn cube_solid(
    model: &mut NamedModel,
    name: &str,
    min: Vector3,
    max: Vector3,
) -> GeopResult<SolidId> {
    let (x0, y0, z0) = (min[0], min[1], max[2]);
    let (x1, y1) = (max[0], max[1]);

    // The coordinate system's `u`/`v` span the bottom footprint (scaled to
    // the box's own `x`/`y` extents, so `outer` is the unit square) and `w`
    // is the extrude direction.
    let origin = Vector3::from_array([x0, y0, z0]);
    let u = Vector3::from_array([x1.sub(x0), Scalar::ZERO, Scalar::ZERO]);
    let v = Vector3::from_array([Scalar::ZERO, y1.sub(y0), Scalar::ZERO]);
    let w = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, min[2].sub(z0)]);
    let coordinate_system = CoordinateSystem::try_new(origin, u, v, w)?;

    // CCW in `(u, v)`, so the bottom cap's outward (downward) normal comes
    // out right once `extrude` sweeps it upward.
    let outer = [
        Vector2::from_array([Scalar::ZERO, Scalar::ZERO]),
        Vector2::from_array([Scalar::ONE, Scalar::ZERO]),
        Vector2::from_array([Scalar::ONE, Scalar::ONE]),
        Vector2::from_array([Scalar::ZERO, Scalar::ONE]),
    ];

    let namer = Namer::new("cube", name)?;
    let built = extrude(
        model,
        &namer,
        Some(&namer.root()),
        &coordinate_system,
        &[Scalar::ZERO, Scalar::ONE],
        &[SweepLoop::plain(Profile::closed(polygon(&outer)?))],
    )?;
    Ok(built.solid.expect("extruded as a solid"))
}

#[cfg(test)]
mod tests {
    use super::cube_solid;
    use crate::common::testing::render;
    use geop_core_math::{scalars::Scalar, vector::Vector3};
    use geop_core_topology::contains::shell::{PointClassification, shell_contains};
    use geop_ops_model::NamedModel;

    const MAX: usize = 200;
    const MIN_SUBDIVISION_SIZE: f64 = 1e-3;
    const SEED: u64 = 7;

    #[test]
    fn cube_has_expected_entity_counts() {
        let mut model = NamedModel::new();
        cube_solid(
            &mut model,
            "t1",
            Vector3::from_array([Scalar::ZERO; 3]),
            Vector3::from_array([Scalar::ONE; 3]),
        )
        .unwrap();
        let topology = model.topology();

        // 6 faces, 8 shared corner vertices, 12 shared edges (each with
        // exactly 2 coedges, one per adjoining face) — `extrude` shares
        // vertices/edges between its side walls and caps directly, no
        // welding needed.
        assert_eq!(topology.faces.len(), 6);
        assert_eq!(topology.vertices.len(), 8);
        assert_eq!(topology.edges.len(), 12);
        assert_eq!(topology.coedges.len(), 24);
        assert_eq!(topology.shells.len(), 1);
        assert_eq!(topology.solids.len(), 1);
        for face in topology.faces.values() {
            assert!(face.holes.is_empty());
        }
        for edge_id in topology.edges.keys() {
            assert_eq!(topology.coedges_of_edge(*edge_id).len(), 2);
        }
    }

    #[test]
    fn cube_center_is_inside() {
        let mut model = NamedModel::new();
        let solid_id = cube_solid(
            &mut model,
            "t2",
            Vector3::from_array([Scalar::ZERO; 3]),
            Vector3::from_array([Scalar::ONE; 3]),
        )
        .unwrap();
        let topology = model.topology();
        let shell_id = topology.get_solid(solid_id).unwrap().shells[0];
        let p = Vector3::from_array([Scalar::from_f64(0.5); 3]);
        assert_eq!(
            shell_contains(
                topology,
                shell_id,
                p,
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::Inside
        );
    }

    #[test]
    fn cube_outside_point_is_outside() {
        let mut model = NamedModel::new();
        let solid_id = cube_solid(
            &mut model,
            "t3",
            Vector3::from_array([Scalar::ZERO; 3]),
            Vector3::from_array([Scalar::ONE; 3]),
        )
        .unwrap();
        let topology = model.topology();
        let shell_id = topology.get_solid(solid_id).unwrap().shells[0];
        let p = Vector3::from_array([
            Scalar::from_f64(-5.0),
            Scalar::from_f64(0.5),
            Scalar::from_f64(0.5),
        ]);
        assert_eq!(
            shell_contains(
                topology,
                shell_id,
                p,
                MAX,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
                SEED
            )
            .unwrap(),
            PointClassification::Outside
        );
    }

    #[test]
    fn rasterize_topology_cube() {
        let mut model = NamedModel::new();
        cube_solid(
            &mut model,
            "t4",
            Vector3::from_array([Scalar::ZERO; 3]),
            Vector3::from_array([Scalar::ONE; 3]),
        )
        .unwrap();
        render(model.topology(), 8, "cube_topology.html");
    }
}
