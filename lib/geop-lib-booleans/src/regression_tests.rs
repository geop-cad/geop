//! Booleans that once failed, rebuilt from their solids.

use geop_core_math::scalars::Scalar;
use geop_core_topology::{Body, SolidId, contains::shell::solid_lumps, validation::validate};
use geop_lib_sweep::shapes::{block, sphere_solid, v3};
use geop_ops_model::{BooleanOp, NamedModel, Namer};

use crate::{boolean, faces_classified, remesh::RemeshParams, scenes::validation};

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
/// together hold both solids, the difference is the block less the
/// intersection, and the symmetric difference the union less it.
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
        BooleanOp::SymmetricDifference,
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
    let [union, intersection, difference, symmetric] = volumes[..] else {
        unreachable!()
    };
    assert!(
        symmetric.add(intersection).could_be_equal(union),
        "{symmetric:?} + {intersection:?} vs {union:?}"
    );
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

/// The lumps of `solid`, each its outer shell's face count and how many
/// voids it holds.
fn lumps(model: &NamedModel, solid: SolidId) -> Vec<(usize, usize)> {
    let params = RemeshParams::default();
    let topology = model.topology();
    solid_lumps(
        topology,
        solid,
        params.max_nodes,
        params.curve_curve_min_subdivision_size,
        7,
    )
    .unwrap()
    .into_iter()
    .map(|(outer, voids)| (topology.get_shell(outer).unwrap().faces.len(), voids.len()))
    .collect()
}

fn combined(model: &mut NamedModel, a: SolidId, b: SolidId, op: BooleanOp) -> SolidId {
    let namer = Namer::new("boolean", "b").unwrap();
    let result = boolean(model, &namer, a, b, op, RemeshParams::default())
        .unwrap_or_else(|e| panic!("{op:?}: {e:?}"))
        .unwrap_or_else(|| panic!("{op:?}: empty"));
    if let Err(errors) = validate(&validation(), model.topology()) {
        panic!("{op:?}: {} error(s): {}", errors.len(), errors[0]);
    }
    result
}

/// A bar with a slab cut out across its middle falls in two: one solid of
/// two lumps, each a closed shell of its own, the box's six faces each.
#[test]
fn bar_cut_in_two_is_two_lumps() {
    let mut model = NamedModel::new();
    let bar = block(&mut model, "bar", [0.0, 0.0, 0.0], [4.0, 1.0, 1.0]);
    let slab = block(&mut model, "slab", [1.5, -1.0, -1.0], [2.5, 2.0, 2.0]);
    let result = combined(&mut model, bar, slab, BooleanOp::Difference);
    assert_eq!(lumps(&model, result), vec![(6, 0), (6, 0)]);
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(3.0)));
}

/// A block less a sphere inside it keeps the sphere's room as a void: one
/// lump, the block's six faces, holding one void.
#[test]
fn block_less_a_sphere_inside_it_has_a_void() {
    let mut model = NamedModel::new();
    let cube = block(&mut model, "cube", [0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
    let ball = sphere_solid(&mut model, "ball", v3(2.0, 2.0, 2.0), Scalar::ONE).unwrap();
    let result = combined(&mut model, cube, ball, BooleanOp::Difference);
    assert_eq!(lumps(&model, result), vec![(6, 1)]);
    let expected = Scalar::from_f64(64.0).sub(Scalar::PI.mul(Scalar::from_f64(4.0 / 3.0)));
    let got = volume(&model, result);
    assert!(
        got.sub(expected).abs().to_f64() < 1e-6,
        "{got:?} vs {expected:?}"
    );
}

/// Two cylinders of one radius with crossing axes meet in two ellipses,
/// which cross each other where the walls touch, at `(0, ±r, 0)`. With
/// both built as the kernel builds them, those nodes fall on the second
/// cylinder's seams; turned 45 degrees about its own axis, the second
/// cylinder has a node in the middle of a face, where the intersection
/// branches cross with the two walls tangent, on a seam of the first; with
/// the first turned 45 degrees about its axis too, inside a face of each.
#[test]
fn steinmetz_cylinders_with_a_node_inside_a_face() {
    use geop_core_math::primitives::Pose;
    use geop_lib_sweep::shapes::{Axis, revolved_cylinder_along_axis};
    let ops = [BooleanOp::Union, BooleanOp::Intersection];
    for (op, both_turned) in ops.into_iter().flat_map(|op| [(op, false), (op, true)]) {
        let mut model = NamedModel::new();
        let r = Scalar::from_f64(0.5);
        let a = revolved_cylinder_along_axis(
            &mut model,
            "a",
            v3(0.0, 0.0, -1.0),
            r,
            Scalar::TWO,
            Axis::Z,
        )
        .unwrap();
        let b = revolved_cylinder_along_axis(
            &mut model,
            "b",
            v3(-1.0, 0.0, 0.0),
            r,
            Scalar::TWO,
            Axis::X,
        )
        .unwrap();
        let turn = Pose::rotation_about(
            &v3(0.0, 0.0, 0.0),
            &v3(1.0, 0.0, 0.0),
            Scalar::PI.div(Scalar::from_f64(4.0)).unwrap(),
        )
        .unwrap();
        model
            .transform_body(Body::Solid(b), &turn.motion())
            .unwrap();
        if both_turned {
            let turn = Pose::rotation_about(
                &v3(0.0, 0.0, 0.0),
                &v3(0.0, 0.0, 1.0),
                Scalar::PI.div(Scalar::from_f64(4.0)).unwrap(),
            )
            .unwrap();
            model
                .transform_body(Body::Solid(a), &turn.motion())
                .unwrap();
        }
        let result = combined(&mut model, a, b, op);
        // Steinmetz: the intersection is 16/3 r^3.
        if op == BooleanOp::Intersection {
            let expected = 16.0 / 3.0 * 0.125;
            let got = volume(&model, result).to_f64();
            assert!((got - expected).abs() < 1e-4, "{got} vs {expected}");
        }
    }
}

/// A prism standing 2 high on a profile whose bottom side is the cubic
/// `y = x³` from `(-1, -1)` to `(1, 1)` (the rest of it up to `y = 2`), and
/// a block below the plane `y + z = 0`, the plane through the origin square
/// to `(0, 1, 1)`. The cubic edge along the prism's foot crosses that plane
/// at the origin, at its inflection, where it runs along the plane: it is
/// tangent to the plane there and still crosses it, from below to above.
/// The prism's faces cross the plane there all the same — the foot along
/// `y = 0`, the cubic wall along `z = -x³` — so the curves the plane cuts
/// end at the origin, and need a vertex there, which only that edge's
/// crossing makes.
#[test]
fn cubic_edge_crossing_a_plane_at_its_inflection() {
    use geop_core_geometry::nurb_curve::NurbCurve2D;
    use geop_core_math::{
        primitives::{CoordinateSystem, Pose},
        vector::Vector3,
    };
    use geop_lib_sweep::{SweepLoop, extrude, polyline, shapes::v2};
    use geop_ops_model::Profile;
    let build = || {
        let mut model = NamedModel::new();
        let s = Scalar::from_f64;
        let point = |x: f64, y: f64| Vector3::from_array([s(x), s(y), Scalar::ONE]);
        let cubic = NurbCurve2D::try_new(
            3,
            vec![
                point(-1.0, -1.0),
                point(-1.0 / 3.0, 1.0),
                point(1.0 / 3.0, -1.0),
                point(1.0, 1.0),
            ],
            [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0].map(s).to_vec(),
        )
        .unwrap();
        let mut profile = vec![cubic];
        profile.extend(
            polyline(&[v2(1.0, 1.0), v2(1.0, 2.0), v2(-1.0, 2.0), v2(-1.0, -1.0)]).unwrap(),
        );
        let plane = CoordinateSystem::try_new(
            v3(0.0, 0.0, 0.0),
            v3(1.0, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        )
        .unwrap();
        let namer = Namer::new("prism", "p").unwrap();
        let prism = extrude(
            &mut model,
            &namer,
            Some(&namer.root()),
            &plane,
            Scalar::ZERO,
            Scalar::TWO,
            &[SweepLoop::plain(Profile::closed(profile))],
        )
        .unwrap()
        .solid
        .unwrap();
        let cut = block(&mut model, "cut", [-3.0, -3.0, -3.0], [3.0, 0.0, 3.0]);
        let turn = Pose::rotation_about(
            &v3(0.0, 0.0, 0.0),
            &v3(1.0, 0.0, 0.0),
            Scalar::PI.div(Scalar::from_f64(4.0)).unwrap(),
        )
        .unwrap();
        model
            .transform_body(Body::Solid(cut), &turn.motion())
            .unwrap();
        (model, prism, cut)
    };
    // The plane cuts a wedge off the prism, below the foot's `y = 0` and
    // the wall's `z = -x³`, for `x < 0`: both what is left and what is cut
    // off are some of the prism, not all of it nor none.
    let (model, prism, _) = build();
    let whole = volume(&model, prism);
    for op in [BooleanOp::Difference, BooleanOp::Intersection] {
        let (mut model, prism, cut) = build();
        let result = combined(&mut model, prism, cut, op);
        let v = volume(&model, result);
        assert!(
            v.definitely_greater(Scalar::ZERO) && v.definitely_less(whole),
            "{op:?}: {v:?} of {whole:?}"
        );
    }
}

/// A disc of radius 4 with `n` slots cut into its rim one after another,
/// each a block across it at a turn of its own: what a gear's teeth ask of
/// one body, boolean after boolean. With how many faces the last cut
/// classified at points of their own.
fn slotted_disc(n: usize) -> (NamedModel, SolidId, usize) {
    use geop_core_math::primitives::Pose;
    use geop_lib_sweep::shapes::cylinder;
    let mut model = NamedModel::new();
    let mut disc = cylinder(&mut model, "disc", [0.0, 0.0, 0.0], 4.0, 1.0);
    let mut last = 0;
    for k in 0..n {
        let slot = block(
            &mut model,
            &format!("slot{k}"),
            [3.5, -0.1, -1.0],
            [5.0, 0.1, 2.0],
        );
        let angle = Scalar::from_f64(std::f64::consts::TAU * (k as f64 + 0.5) / n as f64);
        let turn = Pose::rotation_about(&v3(0.0, 0.0, 0.0), &v3(0.0, 0.0, 1.0), angle).unwrap();
        model
            .transform_body(Body::Solid(slot), &turn.motion())
            .unwrap();
        let before = faces_classified();
        let namer = Namer::new("boolean", &format!("cut{k}")).unwrap();
        let params = RemeshParams::default();
        disc = boolean(
            &mut model,
            &namer,
            disc,
            slot,
            BooleanOp::Difference,
            params,
        )
        .unwrap()
        .unwrap();
        last = faces_classified() - before;
    }
    (model, disc, last)
}

/// A slot cut into a disc that already has many classifies the faces it
/// touches, not every face of the disc: the last of twelve slots as many
/// as the last of four. The disc's faces apart from the slot take their
/// side from their neighbours. The disc cut is valid, its volume less the
/// slots'.
#[test]
fn a_slot_cut_into_a_slotted_disc_classifies_the_faces_it_touches() {
    let (_, _, four) = slotted_disc(4);
    let (model, disc, twelve) = slotted_disc(12);
    assert!(
        twelve <= four,
        "{twelve} faces classified for the 12th slot, {four} for the 4th"
    );
    if let Err(errors) = validate(&validation(), model.topology()) {
        panic!("{} error(s): {}", errors.len(), errors[0]);
    }
    // Each slot takes 0.2 wide, 1 high, from radius 3.5 to the rim: a
    // rectangle 0.2 by 0.5 less the sliver between the chord and the rim.
    let disc_volume = Scalar::PI.mul(Scalar::from_f64(16.0));
    let got = volume(&model, disc).to_f64();
    let removed = disc_volume.to_f64() - got;
    assert!(
        removed > 12.0 * 0.2 * 0.49 && removed < 12.0 * 0.2 * 0.5,
        "{removed}"
    );
}
