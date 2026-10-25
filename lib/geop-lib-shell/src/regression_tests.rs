//! Shells of bodies users built that once failed.

use geop_core_math::scalars::Scalar;
use geop_core_topology::{Body, FaceId, SolidId};
use geop_lib_booleans::{RemeshParams, boolean};
use geop_lib_sweep::shapes::{revolved_cylinder, v3};
use geop_ops_model::{
    BooleanOp, NamedModel, Namer,
    testing::{assert_valid, middle},
};

use crate::shell;

/// A revolved cylinder of radius 1 and height 2 on the origin, drilled
/// through along `z` at `(x, 0)` with a hole of `radius`: the solid
/// `drilled(d)`. Its caps are four quarters each round the axis's pole.
/// The drill is `standing_out` of the caps, or ends flush with them: flush,
/// the rims are the drill's own arcs; standing out, they are traced.
fn drilled_revolved_cylinder(x: f64, radius: f64, standing_out: bool) -> (NamedModel, SolidId) {
    let mut part = NamedModel::new();
    let body = revolved_cylinder(
        &mut part,
        "c",
        v3(0., 0., 0.),
        Scalar::from_f64(1.),
        Scalar::from_f64(2.),
    )
    .unwrap();
    let (base, height) = if standing_out { (-0.5, 3.) } else { (0., 2.) };
    let drill = revolved_cylinder(
        &mut part,
        "h",
        v3(x, 0., base),
        Scalar::from_f64(radius),
        Scalar::from_f64(height),
    )
    .unwrap();
    let namer = Namer::new("drilled", "d").unwrap();
    let solid = boolean(
        &mut part,
        &namer,
        body,
        drill,
        BooleanOp::Difference,
        RemeshParams::default(),
    )
    .unwrap()
    .expect("a drilled cylinder");
    assert_valid(&part);
    (part, solid)
}

/// The heights of the middles of the edges of face `f`.
fn edge_heights(part: &NamedModel, f: FaceId) -> Vec<Scalar> {
    let model = part.topology();
    model
        .face_edges(f)
        .map(|e| middle(model.get_edge(e).unwrap())[2])
        .collect()
}

/// The faces of `solid` whose every edge lies at height `z`: a cap.
fn faces_at_height(part: &NamedModel, solid: SolidId, z: f64) -> Vec<FaceId> {
    let z = Scalar::from_f64(z);
    part.topology()
        .body_faces(Body::Solid(solid))
        .unwrap()
        .into_iter()
        .filter(|&f| edge_heights(part, f).iter().all(|h| h.could_be_equal(z)))
        .collect()
}

/// The faces of `solid` with an edge at height 0 and one at height 2: the
/// sides of the cylinder and of its hole.
fn walls(part: &NamedModel, solid: SolidId) -> Vec<FaceId> {
    part.topology()
        .body_faces(Body::Solid(solid))
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let heights = edge_heights(part, f);
            [0., 2.].iter().all(|&z| {
                heights
                    .iter()
                    .any(|h| h.could_be_equal(Scalar::from_f64(z)))
            })
        })
        .collect()
}

/// The sides of the cylinder: its walls further than the hole's from `(x, 0)`.
fn outer_side(part: &NamedModel, solid: SolidId, x: f64) -> Vec<FaceId> {
    let model = part.topology();
    walls(part, solid)
        .into_iter()
        .filter(|&f| {
            model.face_edges(f).any(|e| {
                let m = middle(model.get_edge(e).unwrap());
                let (dx, dy) = (m[0].to_f64() - x, m[1].to_f64());
                dx * dx + dy * dy > 0.5
            })
        })
        .collect()
}

/// `part`'s `solid` shelled 0.1 thick, open at `open`, which must be valid.
fn shelled_valid(mut part: NamedModel, solid: SolidId, open: &[FaceId]) -> (NamedModel, SolidId) {
    let namer = Namer::new("shell", "s").unwrap();
    let solid = shell(&mut part, &namer, solid, open, Scalar::from_f64(0.1))
        .unwrap_or_else(|e| panic!("{e:?}"));
    assert_valid(&part);
    (part, solid)
}

#[test]
fn drilled_revolved_cylinder_shelled_closed() {
    let (part, solid) = drilled_revolved_cylinder(0., 0.3, false);
    shelled_valid(part, solid, &[]);
}

#[test]
fn drilled_revolved_cylinder_opened_at_its_side() {
    let (part, solid) = drilled_revolved_cylinder(0., 0.3, false);
    let side = outer_side(&part, solid, 0.);
    assert_eq!(side.len(), 4, "{side:?}");
    shelled_valid(part, solid, &side);
}

#[test]
fn drilled_revolved_cylinder_opened_at_its_top() {
    let (part, solid) = drilled_revolved_cylinder(0., 0.3, false);
    let top = faces_at_height(&part, solid, 2.);
    assert_eq!(top.len(), 4, "{top:?}");
    shelled_valid(part, solid, &top);
}

/// The hole's wall, 0.25 from its axis at `x = 0.4`, moves to 0.35 from
/// it: the inner copy of the vertex where the hole's rim crosses the cap's
/// seam lands 0.05 from the cap's pole, and the steps towards it pass
/// closer still.
#[test]
fn off_axis_drilled_revolved_cylinder_shelled_closed() {
    let (part, solid) = drilled_revolved_cylinder(0.4, 0.25, false);
    shelled_valid(part, solid, &[]);
}

#[test]
fn off_axis_drilled_revolved_cylinder_opened_at_its_side() {
    let (part, solid) = drilled_revolved_cylinder(0.4, 0.25, false);
    let side = outer_side(&part, solid, 0.4);
    assert_eq!(side.len(), 4, "{side:?}");
    shelled_valid(part, solid, &side);
}

#[test]
fn off_axis_drilled_revolved_cylinder_opened_at_its_top() {
    let (part, solid) = drilled_revolved_cylinder(0.4, 0.25, false);
    let top = faces_at_height(&part, solid, 2.);
    assert!(!top.is_empty());
    shelled_valid(part, solid, &top);
}

/// The hole's wall, 0.3 from its axis at `x = 0.4`, moves to 0.4 from it:
/// onto the cap's axis. The inner copy of the vertex where the rim crosses
/// the cap's seam is the inner copy of the cap's pole, and the seam between
/// them has no inner copy: refused, naming the seam. (It once failed in
/// `meet`, at a normal of the cap read a hair off its pole.)
#[test]
fn hole_wall_moved_onto_the_axis_of_a_revolved_cap_is_refused() {
    for open_top in [false, true] {
        let (mut part, solid) = drilled_revolved_cylinder(0.4, 0.3, false);
        let open = if open_top {
            faces_at_height(&part, solid, 2.)
        } else {
            Vec::new()
        };
        let namer = Namer::new("shell", "s").unwrap();
        let error = shell(&mut part, &namer, solid, &open, Scalar::from_f64(0.1)).unwrap_err();
        let message = error.root_message();
        assert!(
            message.contains("too thick") && message.contains("cylinder(c,c"),
            "{error:?}"
        );
    }
}

/// A drill standing out of the caps leaves rims the boolean traced, where
/// the drill's side crosses the caps. The boolean once fitted a curve
/// through its march there, which was neither straight nor circular to the
/// shell; the rim is a circle, a plane across a cylinder around its
/// normal, and the boolean now splices that arc. Shelled like the flush
/// drill.
#[test]
fn drill_standing_out_leaves_a_traced_rim_the_shell_offsets() {
    let (part, solid) = drilled_revolved_cylinder(0.4, 0.25, true);
    let faces = |part: &NamedModel, solid| {
        part.topology()
            .body_faces(Body::Solid(solid))
            .unwrap()
            .len()
    };
    let before = faces(&part, solid);
    let (part, shelled) = shelled_valid(part, solid, &[]);
    // Every face and its inner copy.
    assert_eq!(faces(&part, shelled), 2 * before);
}
