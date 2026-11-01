//! Booleans that once failed, rebuilt from their solids.

use geop_core_math::scalars::Scalar;
use geop_core_topology::{Body, SolidId, contains::shell::solid_lumps, validation::validate};
use geop_lib_sweep::shapes::{block, cylinder, extruded_cylinder, sphere_solid, v3};
use geop_ops_model::{BooleanOp, NamedModel, Namer};

use crate::{boolean, faces_classified, imprint_on_face, remesh::RemeshParams, scenes::validation};

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
            &[Scalar::ZERO, Scalar::TWO],
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

/// Making one what a cut left in two asks only of the faces the cut touches:
/// each slot of twelve as many pairs as each slot of four, the disc's other
/// faces, whose edges it was given, not asked again.
#[test]
fn a_slot_cut_into_a_slotted_disc_compares_the_surfaces_it_touches() {
    let compared = |n: usize| {
        let before = geop_core_topology::merge::surfaces_compared();
        slotted_disc(n);
        (geop_core_topology::merge::surfaces_compared() - before) / n
    };
    let (four, twelve) = (compared(4), compared(12));
    assert!(four > 0);
    assert!(twelve <= four, "{twelve} pairs a slot of 12, {four} of 4");
}

/// The slotted disc with thirty-two slots, eight to a quarter of its rim.
/// The 32nd cut once left the slot's footprint on the cap's quarter
/// straddling the slot, its corner on the rim `4e-10` below the cap: each
/// cut of the rim had carried the width of its parameter into the pieces,
/// three times wider per cut.
#[test]
fn a_disc_with_thirty_two_slots() {
    let (model, _, _) = slotted_disc(32);
    if let Err(errors) = validate(&validation(), model.topology()) {
        panic!("{} error(s): {}", errors.len(), errors[0]);
    }
}

/// The widest control point coordinate of any edge of `solid` that
/// `which` picks, and the edge's name.
fn widest_edge(
    model: &NamedModel,
    solid: SolidId,
    which: impl Fn(&geop_core_topology::Curve3) -> bool,
) -> (f64, String) {
    let topology = model.topology();
    let mut widest = (0.0, String::new());
    for face in topology.solid_faces(solid).unwrap() {
        for edge in topology.face_edges(face) {
            let curve = &topology.get_edge(edge).unwrap().curve;
            if !which(curve) {
                continue;
            }
            for point in &curve.control_points {
                for k in 0..4 {
                    let width = point[k].width().to_f64();
                    if width > widest.0 {
                        widest = (width, model.name_of(edge).unwrap_or("?").to_string());
                    }
                }
            }
        }
    }
    widest
}

/// The lines a slot's sides meet the disc's side in are traced, between
/// the points where they cross its rims. The traced line of the eleventh
/// slot's side `c0` was `1.7e-12` wide, its ends `6.3e-13`, crossings of
/// the rims with the slot's side, which its turn had left `1.3e-13` wide:
/// the turn's quaternion, unit already as cosine and sine, was normalized
/// again, honestly, and came out four times as wide, and the matrix with
/// it. Kept as it is, the ends are under `5e-13`. The line is still about
/// `1.5e-12`: the corrector's points along it are as wide as its ends,
/// and the fit widened to hold them three to four times that.
#[test]
fn the_lines_where_slots_meet_a_disc_side_stay_narrow() {
    let (model, disc, _) = slotted_disc(16);
    let (width, edge) = widest_edge(&model, disc, |curve| curve.degree != 2);
    assert!(width < 2e-12, "{edge}: {width:e}");
    let topology = model.topology();
    let e = topology.get_edge(model.edge_id(&edge).unwrap()).unwrap();
    for v in [e.start_vertex, e.end_vertex] {
        let p = topology.vertices[&v].point;
        let width = (0..3).map(|k| p[k].width().to_f64()).fold(0.0, f64::max);
        assert!(width < 5e-13, "{:?}: {width:e}", model.name_of(v));
    }
}

/// The rims of a disc are cut twice by every slot, so with sixteen slots
/// each quarter of them eight times over: their pieces are as narrow as a
/// rim cut once, to the rounding of each cut. When a cut carried the width
/// of its parameter into the pieces, the bottom rim's quarter
/// `cylinder(disc,p2,q3)` came out `1.2e-8` wide.
#[test]
fn the_rims_of_a_slotted_disc_stay_as_narrow_as_cut_once() {
    let (model, disc, _) = slotted_disc(16);
    let (width, edge) = widest_edge(&model, disc, |curve| curve.degree == 2);
    assert!(width < 1e-13, "{edge}: {width:e}");
}

/// How many faces, edges and vertices the model holds.
fn counts(model: &NamedModel) -> (usize, usize, usize) {
    let topology = model.topology();
    (
        topology.faces.len(),
        topology.edges.len(),
        topology.vertices.len(),
    )
}

/// Two blocks standing flush, one on the end of the other: four walls in
/// two planes each, made one box.
#[test]
fn blocks_united_end_to_end_are_one_box() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block(&mut model, "b", [1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let result = combined(&mut model, a, b, BooleanOp::Union);
    assert_eq!(counts(&model), (6, 12, 8));
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(2.0)));
}

/// The same, overlapping: each block's walls run on past the other's end
/// before the boolean drops what lies inside.
#[test]
fn overlapping_blocks_united_are_one_box() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block(&mut model, "b", [0.5, 0.0, 0.0], [1.5, 1.0, 1.0]);
    let result = combined(&mut model, a, b, BooleanOp::Union);
    assert_eq!(counts(&model), (6, 12, 8));
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(1.5)));
}

/// A taller block overlapping a block, flush along three sides: what is
/// left of the taller one's wall is an L, split off the part lying on the
/// first's wall under a piece's name, and runs on into the first's wall.
/// The wall's name finds what it became, as a lone piece's does.
#[test]
fn a_wall_trimmed_and_run_into_another_keeps_its_name() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block(&mut model, "b", [0.5, 0.0, 0.0], [1.5, 1.0, 2.0]);
    let topology = model.topology();
    let mut names = Vec::new();
    for solid in [a, b] {
        for face in topology.solid_faces(solid).unwrap() {
            names.push(model.name_of(face).unwrap().to_string());
        }
    }
    combined(&mut model, a, b, BooleanOp::Union);
    let lost: Vec<_> = (names.iter())
        .filter(|name| model.face_id(name).is_err())
        .collect();
    // Of the twelve only the first's far end, wholly inside the second.
    assert_eq!(lost.len(), 1, "{lost:?}");
}

/// `model`'s block `[0, 3] x [0, 1] x [0, 1]` with its top divided in three
/// across `x = 1.5` and `x = 2`, as a step splitting a face does: the pieces
/// and the edges between them are what the part was given.
fn block_with_a_divided_top(model: &mut NamedModel) -> SolidId {
    let a = block(model, "a", [0.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
    let topology = model.topology();
    let top = (topology.solid_faces(a).unwrap().into_iter())
        .find(|f| {
            (topology.get_face(*f).unwrap().surface.control_points.iter())
                .all(|p| p[2].div(p[3]).unwrap().could_be_equal(Scalar::ONE))
        })
        .unwrap();
    let knife = block(model, "knife", [1.5, -1.0, 0.0], [2.0, 2.0, 2.0]);
    let namer = Namer::new("imprint", "i").unwrap();
    imprint_on_face(model, &namer, top, &[knife.into()], RemeshParams::default()).unwrap();
    a
}

/// A face a step divided stays divided through a boolean elsewhere on the
/// body: the edge between the pieces lies on one surface, but the boolean
/// did not leave it there, so it has no say over it.
#[test]
fn a_face_a_step_divided_stays_divided_through_a_boolean_elsewhere() {
    let mut model = NamedModel::new();
    let a = block_with_a_divided_top(&mut model);
    let (faces, edges, _) = counts(&model);
    assert!(faces > 6, "the top is divided: {faces} faces");
    let pocket = block(&mut model, "pocket", [0.25, 0.25, -1.0], [0.5, 0.75, 0.5]);
    combined(&mut model, a, pocket, BooleanOp::Difference);
    // The pocket's four walls and its ceiling, its rim four edges and its
    // uprights and ceiling eight.
    assert_eq!(counts(&model).0, faces + 5);
    assert_eq!(counts(&model).1, edges + 12);
}

/// A plate on the end of that block, flush with the divided top: it runs on
/// into the piece it stands beside, and that piece stays parted from the
/// other, along the edge the step drew.
#[test]
fn a_flush_union_does_not_undo_a_step_dividing_the_face_beside_it() {
    let mut model = NamedModel::new();
    let a = block_with_a_divided_top(&mut model);
    let (faces, ..) = counts(&model);
    let plate = block(&mut model, "plate", [3.0, 0.0, 0.0], [4.0, 1.0, 1.0]);
    let result = combined(&mut model, a, plate, BooleanOp::Union);
    // Of the plate's six only its far end is a face of its own, in place of
    // the block's end, gone inside.
    assert_eq!(counts(&model).0, faces);
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(4.0)));
}

/// A plate standing flush with one side of the block under it only: that
/// side is one face, every other face of either stays, a step all round.
#[test]
fn a_plate_flush_with_one_side_is_one_wall_there() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let b = block(&mut model, "b", [0.0, 0.5, 1.0], [1.0, 1.5, 2.0]);
    let result = combined(&mut model, a, b, BooleanOp::Union);
    // The block's six, and the plate's top and three sides: its bottom is
    // gone and its side at x = 0 is one with the block's.
    assert_eq!(counts(&model).0, 10);
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(5.0)));
}

/// Two pockets of one depth overlapping: the second's floor runs on from
/// the first's, one floor.
#[test]
fn two_pockets_of_one_depth_share_one_floor() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [4.0, 4.0, 2.0]);
    let first = block(&mut model, "first", [1.0, 1.0, 1.0], [2.5, 3.0, 3.0]);
    let pocket = combined(&mut model, a, first, BooleanOp::Difference);
    let second = block(&mut model, "second", [2.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
    let result = combined(&mut model, pocket, second, BooleanOp::Difference);
    // The block's six, a floor, and four walls round the one pocket: those
    // at y = 1 and y = 3 run on from the first into the second.
    assert_eq!(counts(&model).0, 11);
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(28.0)));
}

/// A slot across a block's top leaves the top in two; a block filling it
/// flush makes all three one again, though the two pieces meet only
/// through what filled the slot.
#[test]
fn a_slot_filled_flush_is_a_whole_top_again() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
    let slot = block(&mut model, "slot", [1.0, -1.0, 0.5], [2.0, 2.0, 2.0]);
    let cut = combined(&mut model, a, slot, BooleanOp::Difference);
    let fill = block(&mut model, "fill", [1.0, 0.0, 0.5], [2.0, 1.0, 1.0]);
    let result = combined(&mut model, cut, fill, BooleanOp::Union);
    assert_eq!(counts(&model), (6, 12, 8));
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(3.0)));
}

/// A plug filling a bored hole flush: its caps lie in the box's top and
/// bottom, and the hole in each goes with the edges round it.
#[test]
fn a_plugged_bore_is_a_box_again() {
    let mut model = NamedModel::new();
    let (r, six) = (Scalar::from_f64(0.5), 6);
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let bore = extruded_cylinder(
        &mut model,
        "bore",
        v3(1.0, 1.0, 2.0),
        r,
        Scalar::from_f64(3.0),
        six,
    )
    .unwrap();
    let drilled = combined(&mut model, a, bore, BooleanOp::Difference);
    let plug =
        extruded_cylinder(&mut model, "plug", v3(1.0, 1.0, 1.0), r, Scalar::ONE, six).unwrap();
    let result = combined(&mut model, drilled, plug, BooleanOp::Union);
    assert_eq!(counts(&model), (6, 12, 8));
    assert!(volume(&model, result).could_be_equal(Scalar::from_f64(4.0)));
}

/// The same with a revolved plug, whose caps are four sectors parametrized
/// by radius and turn: they lie in the box's plane, but no shift and scale of
/// `(u, v)` takes one to the other (`NurbSurface3D::carrying`), so they stay
/// sectors in the box's holes. Valid and whole, only not one face: what it
/// takes to make them one is pcurves mapped through a rational map.
#[test]
fn a_revolved_plugs_caps_stay_sectors() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let bore = cylinder(&mut model, "bore", [1.0, 1.0, -1.0], 0.5, 3.0);
    let drilled = combined(&mut model, a, bore, BooleanOp::Difference);
    let plug = cylinder(&mut model, "plug", [1.0, 1.0, 0.0], 0.5, 1.0);
    let result = combined(&mut model, drilled, plug, BooleanOp::Union);
    assert_eq!(counts(&model).0, 6 + 8);
    let got = volume(&model, result);
    assert!(got.could_be_equal(Scalar::from_f64(4.0)), "{got:?}");
}

/// A bar turned in the plane of a block's top, its own top flush with it:
/// one plane, but its `(u, v)` turned against the block's, which
/// `NurbSurface3D::carrying` does not map (each parameter has to go to one
/// parameter). The tops stay apart, and what is built is whole.
#[test]
fn a_top_turned_in_its_plane_stays_a_face_of_its_own() {
    use geop_core_math::primitives::Pose;
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [-1.0, -1.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block(&mut model, "b", [-2.0, -0.25, 0.0], [2.0, 0.25, 1.0]);
    let turn = Pose::rotation_about(
        &v3(0.0, 0.0, 0.0),
        &v3(0.0, 0.0, 1.0),
        Scalar::from_f64(0.5),
    )
    .unwrap();
    model
        .transform_body(Body::Solid(b), &turn.motion())
        .unwrap();
    let result = combined(&mut model, a, b, BooleanOp::Union);
    let topology = model.topology();
    let tops = (topology.solid_faces(result).unwrap().iter())
        .filter(|f| {
            (topology
                .get_face(**f)
                .unwrap()
                .surface
                .control_points
                .iter())
            .all(|p| p[2].div(p[3]).unwrap().could_be_equal(Scalar::ONE))
        })
        .count();
    assert!(tops > 1, "{tops}");
}

/// The names of what a union makes one: every face's name of either block
/// still finds its face, but for the two faces between them, gone inside;
/// the walls' names of both find the one wall. An edge between two faces
/// made one is gone and asking for it says so by name, and an edge joined
/// to one running on from it is found by the name of either piece.
#[test]
fn names_of_what_a_union_made_one_still_find_it() {
    let mut model = NamedModel::new();
    let a = block(&mut model, "a", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = block(&mut model, "b", [1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let topology = model.topology();
    let mut face_names = Vec::new();
    let mut edge_names = Vec::new();
    for solid in [a, b] {
        for face in topology.solid_faces(solid).unwrap() {
            face_names.push(model.name_of(face).unwrap().to_string());
            for edge in topology.face_edges(face) {
                edge_names.push(model.name_of(edge).unwrap().to_string());
            }
        }
    }
    edge_names.sort();
    edge_names.dedup();
    combined(&mut model, a, b, BooleanOp::Union);
    let faces: Vec<_> = (face_names.iter())
        .filter_map(|name| model.face_id(name).ok())
        .collect();
    assert_eq!(faces.len(), 10, "{face_names:?}");
    let distinct: std::collections::BTreeSet<_> = faces.iter().collect();
    assert_eq!(distinct.len(), 6);
    let edges: Vec<_> = (edge_names.iter())
        .filter_map(|name| model.edge_id(name).ok())
        .collect();
    let distinct: std::collections::BTreeSet<_> = edges.iter().collect();
    // Twelve edges of the box, all found: four along x, each of a piece of
    // either block, and the eight round its ends. The four round the seam
    // are gone, by the names of both blocks.
    assert_eq!(distinct.len(), 12, "{edge_names:?}");
    assert_eq!(edges.len(), 16, "{edge_names:?}");
    let gone = (edge_names.iter())
        .find(|name| model.edge_id(name).is_err())
        .expect("an edge between the faces made one");
    let message = model.edge_id(gone).unwrap_err().to_string();
    assert!(message.contains(gone.as_str()), "{message}");
}

/// Two cylinders of one radius stacked: each quarter of the wall runs on
/// into the one above, one surface, and the rim between them goes, its
/// corners with it. The quarters stay quarters: each is a patch of its own,
/// by design.
#[test]
fn stacked_cylinders_are_one_cylinder() {
    let mut model = NamedModel::new();
    let a = cylinder(&mut model, "a", [0.0, 0.0, 0.0], 1.0, 1.0);
    let b = cylinder(&mut model, "b", [0.0, 0.0, 1.0], 1.0, 1.0);
    let whole = {
        let mut model = NamedModel::new();
        cylinder(&mut model, "whole", [0.0, 0.0, 0.0], 1.0, 2.0);
        counts(&model)
    };
    let result = combined(&mut model, a, b, BooleanOp::Union);
    assert_eq!(counts(&model), whole);
    let expected = Scalar::PI.mul(Scalar::from_f64(2.0));
    let got = volume(&model, result);
    assert!(
        got.sub(expected).abs().to_f64() < 1e-6,
        "{got:?} vs {expected:?}"
    );
}

/// A block standing out of a cylinder's wall, of the cylinder's height: its
/// top and bottom lie in the caps' planes and are one with them. The wall
/// is cut by the block, and of its quarters, patches of their own by design
/// that lie on one circle, none is made one with another.
#[test]
fn a_cylinders_quarters_stay_apart() {
    let mut model = NamedModel::new();
    let a = cylinder(&mut model, "a", [0.0, 0.0, 0.0], 1.0, 1.0);
    let b = block(&mut model, "b", [0.5, -0.25, 0.0], [2.0, 0.25, 1.0]);
    let result = combined(&mut model, a, b, BooleanOp::Union);
    let topology = model.topology();
    let curved = (topology.solid_faces(result).unwrap().iter())
        .filter(|f| topology.get_face(**f).unwrap().surface.degree_u == 2)
        .count();
    assert!(curved >= 4, "{curved} curved faces");
    // The caps and the block's top and bottom are two faces, and the block's
    // three other sides are there too.
    assert_eq!(counts(&model).0, curved + 2 + 3);
}
