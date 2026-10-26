//! Booleans that once failed, rebuilt from their solids.

use geop_core_math::scalars::Scalar;
use geop_core_topology::{Body, SolidId, validation::validate};
use geop_lib_sweep::shapes::block;
use geop_ops_model::{BooleanOp, NamedModel, Namer};

use crate::{boolean, remesh::RemeshParams, scenes::validation};

/// The block spanning `[min, max]` with its top face bulged into a dome
/// rising `rise` above it in the middle: the top's plane raised to a
/// biquadratic, its middle control point lifted by four times `rise`. The
/// dome's rim is the top's own four edges, so the block's sides are kept.
fn domed_block(
    model: &mut NamedModel,
    name: &str,
    min: [f64; 3],
    max: [f64; 3],
    rise: f64,
) -> SolidId {
    let mut scratch = NamedModel::new();
    let solid = block(&mut scratch, name, min, max);
    let (mut spec, names) = scratch.body_record(Body::Solid(solid)).unwrap();
    let top = Scalar::from_f64(max[2]);
    let face = spec
        .faces
        .iter_mut()
        .find(|f| {
            f.surface
                .control_points
                .iter()
                .all(|p| p[2].div(p[3]).unwrap().could_be_equal(top))
        })
        .expect("a top face");
    let mut dome = face
        .surface
        .elevate_degree(true)
        .unwrap()
        .elevate_degree(false)
        .unwrap();
    assert_eq!((dome.num_u, dome.num_v), (3, 3));
    let middle = &mut dome.control_points[4];
    middle[2] = middle[2].add(Scalar::from_f64(4.0 * rise).mul(middle[3]));
    dome.recompute_aabb();
    face.surface = dome;
    model
        .build_body(spec, names)
        .unwrap()
        .solid
        .expect("a solid")
}

fn volume(model: &NamedModel, solid: SolidId) -> Scalar {
    model.topology().mass_properties(solid).unwrap().volume
}

/// A plate lying over a block whose domed top pokes up through the
/// plate's bottom: that face meets the dome in an oval inside both faces,
/// which no edge of either solid pierces, so no piercing starts a trace of
/// it. Every boolean of the two has to cut both faces along the oval.
/// Checked by validity and by the volumes: the union and the intersection
/// together hold both solids, and the difference is the block less the
/// intersection.
#[test]
fn dome_poking_through_a_plate() {
    let build = || {
        let mut model = NamedModel::new();
        let dome = domed_block(&mut model, "dome", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 0.5);
        let plate = block(&mut model, "plate", [-0.5, -0.5, 1.25], [1.5, 1.5, 2.0]);
        (model, dome, plate)
    };
    let (model, dome, plate) = build();
    let (v_dome, v_plate) = (volume(&model, dome), volume(&model, plate));
    let mut volumes = Vec::new();
    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
    ] {
        let (mut model, dome, plate) = build();
        let namer = Namer::new("boolean", "b").unwrap();
        let result = boolean(&mut model, &namer, dome, plate, op, RemeshParams::default())
            .unwrap_or_else(|e| panic!("{op:?}: {e:?}"))
            .unwrap_or_else(|| panic!("{op:?}: empty"));
        if let Err(errors) = validate(&validation(), model.topology()) {
            panic!("{op:?}: {} error(s): {}", errors.len(), errors[0]);
        }
        volumes.push(volume(&model, result));
    }
    let [union, intersection, difference] = volumes[..] else {
        unreachable!()
    };
    assert!(
        union.add(intersection).could_be_equal(v_dome.add(v_plate)),
        "{union:?} + {intersection:?} vs {v_dome:?} + {v_plate:?}"
    );
    assert!(
        difference.add(intersection).could_be_equal(v_dome),
        "{difference:?} + {intersection:?} vs {v_dome:?}"
    );
    assert!(
        intersection.definitely_greater(Scalar::ZERO),
        "{intersection:?}"
    );
}
