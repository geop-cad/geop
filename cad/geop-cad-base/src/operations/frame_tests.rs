//! Reference frames: what a face, an edge or a point gives where a mate puts
//! one, and what a pick for one finds where the pointer is.

use geop_core_math::{
    primitives::{CoordinateSystem, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    EntityRef, NoFiles, Part,
    operation::{Aspects, Role},
    ui::{PartView, Pointer, Reach},
};

use crate::examples;

/// The plate of [`examples::box_with_drill_hole`]: `[0, 2] x [0, 2] x [0, 1]`,
/// with a hole of radius 0.4 round `(1, 1)` from its top down to `z = 0.5`.
fn plate() -> Part<S> {
    examples::box_with_drill_hole().build(&NoFiles).unwrap()
}

fn v(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// Where `frame` is, and the way its `z` axis runs, as plain numbers.
fn placed(frame: &CoordinateSystem<S>) -> ([f64; 3], [f64; 3]) {
    let plain = |p: &Vector3<S>| [0, 1, 2].map(|k| p[k].to_f64());
    (plain(frame.origin()), plain(frame.w()))
}

#[track_caller]
fn assert_close(a: [f64; 3], b: [f64; 3]) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9),
        "{a:?} is not {b:?}"
    );
}

/// The frame `on` gives at `at`, as numbers: where, and its `z`, which may
/// point either way along an axis.
#[track_caller]
fn assert_frame(
    part: &Part<S>,
    on: &EntityRef,
    at: Option<&EntityRef>,
    origin: [f64; 3],
    z: [f64; 3],
) {
    let frame = Aspects::frame_on(on, at, part).unwrap();
    let (found, axis) = placed(&frame);
    assert_close(found, origin);
    let along = (0..3).map(|k| axis[k] * z[k]).sum::<f64>();
    assert!(
        (along.abs() - 1.0).abs() < 1e-9,
        "{axis:?} does not run along {z:?}"
    );
}

/// The vertex of `part` at `p`.
fn vertex_at(part: &Part<S>, p: [f64; 3]) -> EntityRef {
    let model = part.topology();
    let (&id, _) = model
        .vertices
        .iter()
        .find(|(_, vertex)| vertex.point.could_be_equal(&v(p)))
        .unwrap_or_else(|| panic!("no vertex at {p:?}"));
    EntityRef::Vertex {
        name: part.name_of(id).unwrap().to_string(),
    }
}

/// The edge of `part` whose middle is at `p`.
fn edge_at(part: &Part<S>, p: [f64; 3]) -> EntityRef {
    let model = part.topology();
    let (&id, _) = model
        .edges
        .iter()
        .find(|(_, edge)| {
            let (t0, t1) = edge.curve.domain();
            edge.curve
                .evaluate(S::interpolate(t0, t1, S::from_f64(0.5)))
                .unwrap()
                .could_be_equal(&v(p))
        })
        .unwrap_or_else(|| panic!("no edge with its middle at {p:?}"));
    EntityRef::Edge {
        name: part.name_of(id).unwrap().to_string(),
    }
}

fn top() -> EntityRef {
    EntityRef::Face {
        name: "extrude(box,end)".into(),
    }
}

/// A planar face has its frame at its middle, `z` along its normal; where it
/// sits at a corner or a side, that point, with the same `z`.
#[test]
fn a_planar_face_gives_its_middle_or_the_corner_or_side_it_is_near() {
    let part = plate();
    assert_frame(&part, &top(), None, [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
    let corner = vertex_at(&part, [0.0, 0.0, 1.0]);
    assert_frame(
        &part,
        &top(),
        Some(&corner),
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    );
    let side = edge_at(&part, [1.0, 0.0, 1.0]);
    assert_frame(&part, &top(), Some(&side), [1.0, 0.0, 1.0], [0.0, 0.0, 1.0]);
    // Out of the solid: up, on the top.
    let frame = Aspects::frame_on(&top(), None, &part).unwrap();
    assert!(placed(&frame).1[2] > 0.99);
}

/// A round face has its frame on its axis, half way along it — or as far
/// along as a corner or side it is near — `z` along the axis; a circular
/// edge at its center, `z` along its axis.
#[test]
fn round_things_give_their_axis() {
    let part = plate();
    let wall = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .find(|name| name.starts_with("extrude(hole,hole_sketch"))
        .unwrap()
        .to_string();
    let wall = EntityRef::Face { name: wall };
    assert_frame(&part, &wall, None, [1.0, 1.0, 0.75], [0.0, 0.0, 1.0]);
    let rim = vertex_at(&part, [1.4, 1.0, 1.0]);
    assert_frame(&part, &wall, Some(&rim), [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
    let model = part.topology();
    let rim_edge = model
        .edges
        .iter()
        .find(|(_, e)| {
            e.curve
                .as_arc()
                .unwrap()
                .is_some_and(|arc| arc.circle.center.could_be_equal(&v([1.0, 1.0, 1.0])))
        })
        .map(|(&id, _)| EntityRef::Edge {
            name: part.name_of(id).unwrap().to_string(),
        })
        .expect("the hole's rim");
    assert_frame(&part, &rim_edge, None, [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
}

/// A straight edge has its frame at its middle, `z` along it — or at the
/// end it is near; a point's has the world's axes.
#[test]
fn a_straight_edge_gives_its_middle_or_its_end_and_a_point_the_world() {
    let part = plate();
    let edge = edge_at(&part, [1.0, 0.0, 1.0]);
    assert_frame(&part, &edge, None, [1.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let end = vertex_at(&part, [0.0, 0.0, 1.0]);
    assert_frame(&part, &edge, Some(&end), [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let corner = vertex_at(&part, [2.0, 2.0, 0.0]);
    let frame = Aspects::frame_on(&corner, None, &part).unwrap();
    let (at, z) = placed(&frame);
    assert_close(at, [2.0, 2.0, 0.0]);
    assert_close(z, [0.0, 0.0, 1.0]);
    assert_close(placed_x(&frame), [1.0, 0.0, 0.0]);
}

fn placed_x(frame: &CoordinateSystem<S>) -> [f64; 3] {
    [0, 1, 2].map(|k| frame.u()[k].to_f64())
}

/// Looking straight down at `(x, y)`, as the viewport's reach is 0.009 there.
fn down(x: f64, y: f64) -> Pointer<S> {
    Pointer {
        ray: Ray::try_new(v([x, y, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

/// What a pick for a frame finds, as the frame's entity and where it sits.
fn picked(part: &Part<S>, x: f64, y: f64) -> (EntityRef, Option<EntityRef>) {
    let view = PartView::of(part).unwrap();
    let hit = view
        .pick(&down(x, y), &[Role::Frame], None)
        .unwrap_or_else(|| panic!("nothing to put a frame on at ({x}, {y})"));
    match hit.entity {
        EntityRef::Frame { on, at } => (*on, at.map(|at| *at)),
        other => panic!("{other} is no frame"),
    }
}

/// A pick for a frame over a face finds the face — at its middle where the
/// pointer is far from every corner and side — and, near a corner or a side
/// of it, there: the corner before the side.
#[test]
fn a_pick_over_a_face_puts_the_frame_at_the_corner_or_side_it_is_near() {
    let part = plate();
    let (on, at) = picked(&part, 1.0, 1.7);
    assert_eq!(on, top());
    assert_eq!(at, None);
    let (on, at) = picked(&part, 0.5, 0.012);
    assert_eq!(on, top());
    assert_eq!(at, Some(edge_at(&part, [1.0, 0.0, 1.0])));
    let (on, at) = picked(&part, 0.02, 0.02);
    assert_eq!(on, top());
    assert_eq!(at, Some(vertex_at(&part, [0.0, 0.0, 1.0])));
}

/// A pick for a frame right on a straight edge finds it, at its middle —
/// near an end of it, there; right on a corner, the face there, at it.
#[test]
fn a_pick_over_an_edge_puts_the_frame_at_its_middle_or_the_end_it_is_near() {
    let part = plate();
    let edge = edge_at(&part, [1.0, 0.0, 1.0]);
    let (on, at) = picked(&part, 1.0, 0.0);
    assert_eq!((on, at), (edge.clone(), None));
    let (on, at) = picked(&part, 0.2, 0.0);
    assert_eq!((on, at), (edge, Some(vertex_at(&part, [0.0, 0.0, 1.0]))));
}
