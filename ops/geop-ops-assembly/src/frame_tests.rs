//! Reference frames: what a face, an edge or a point gives where a mate puts
//! one, and what a pick for one finds where the pointer is.

use geop_core_editor::view::{PartView, drawn};
use geop_core_editor::{
    editing::{Pointer, Reach},
    selectable::Selectable,
};
use geop_core_math::{
    primitives::{CoordinateSystem, Ray},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, part::Part};
use geop_lib_fixtures::{drilled_box, registry};
use geop_ops_model::selection::Selected;
use geop_ops_model::{
    NamedModel, model,
    testing::{Built, edge_at, model_of, vertex_at},
};

use geop_ops_model::mate_entity::MateEntity;

/// The drilled box (see [`drilled_box`]): `[0, 2] x [0, 2] x [0, 1]`, with
/// a hole of radius 0.4 round `(1, 1)` from its top down to `z = 0.5`, as
/// built.
fn plate() -> Part {
    drilled_box().built(registry).unwrap()
}

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// Where `frame` is, and the way its `z` axis runs, as plain numbers.
fn placed(frame: &CoordinateSystem) -> ([f64; 3], [f64; 3]) {
    let plain = |p: &Vector3| [0, 1, 2].map(|k| p[k].to_f64());
    (plain(frame.origin()), plain(frame.w()))
}

#[track_caller]
fn assert_close(a: [f64; 3], b: [f64; 3]) {
    geop_ops_model::testing::assert_close(a, b, 1e-9);
}

/// The frame `on`, put at `at`, gives in `part`.
fn frame_of(part: &Part, on: Selected, at: Option<Selected>) -> CoordinateSystem {
    let entity = MateEntity {
        at,
        ..MateEntity::on(on)
    };
    entity.frame(&part.current()).unwrap()
}

/// The frame `on` gives at `at`, as numbers: where, and its `z`, which may
/// point either way along an axis.
#[track_caller]
fn assert_frame(part: &Part, on: &Selected, at: Option<&Selected>, origin: [f64; 3], z: [f64; 3]) {
    let frame = frame_of(part, on.clone(), at.cloned());
    let (found, axis) = placed(&frame);
    assert_close(found, origin);
    let along = (0..3).map(|k| axis[k] * z[k]).sum::<f64>();
    assert!(
        (along.abs() - 1.0).abs() < 1e-9,
        "{axis:?} does not run along {z:?}"
    );
}

fn face(name: &str) -> Selected {
    Selected::Face(ElementReference::new(model(), name))
}

fn edge(name: &str) -> Selected {
    Selected::Edge(ElementReference::new(model(), name))
}

fn vertex(name: &str) -> Selected {
    Selected::Vertex(ElementReference::new(model(), name))
}

fn top() -> Selected {
    face("block(box,end)")
}

/// A planar face has its frame at its middle, `z` along its normal; where it
/// sits at a corner or a side, that point, with the same `z`.
#[test]
fn a_planar_face_gives_its_middle_or_the_corner_or_side_it_is_near() {
    let built = plate();
    let part = model_of(&built);
    assert_frame(&built, &top(), None, [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
    let corner = vertex(&vertex_at(&part, [0.0, 0.0, 1.0]));
    assert_frame(
        &built,
        &top(),
        Some(&corner),
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    );
    let side = edge(&edge_at(&part, [1.0, 0.0, 1.0]));
    assert_frame(
        &built,
        &top(),
        Some(&side),
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    );
    // Out of the solid: up, on the top.
    let frame = frame_of(&built, top(), None);
    assert!(placed(&frame).1[2] > 0.99);
}

/// A round face has its frame on its axis, half way along it — or as far
/// along as a corner or side it is near — `z` along the axis; a circular
/// edge at its center, `z` along its axis.
#[test]
fn round_things_give_their_axis() {
    let built = plate();
    let part = model_of(&built);
    let wall = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .find(|name| name.starts_with("cylinder(hole,c1"))
        .unwrap()
        .to_string();
    let wall = face(&wall);
    assert_frame(&built, &wall, None, [1.0, 1.0, 0.75], [0.0, 0.0, 1.0]);
    let rim = vertex(&vertex_at(&part, [1.4, 1.0, 1.0]));
    assert_frame(&built, &wall, Some(&rim), [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
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
        .map(|(&id, _)| edge(part.name_of(id).unwrap()))
        .expect("the hole's rim");
    assert_frame(&built, &rim_edge, None, [1.0, 1.0, 1.0], [0.0, 0.0, 1.0]);
}

/// A straight edge has its frame at its middle, `z` along it — or at the
/// end it is near; a point's has the world's axes.
#[test]
fn a_straight_edge_gives_its_middle_or_its_end_and_a_point_the_world() {
    let built = plate();
    let part = model_of(&built);
    let side = edge(&edge_at(&part, [1.0, 0.0, 1.0]));
    assert_frame(&built, &side, None, [1.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let end = vertex(&vertex_at(&part, [0.0, 0.0, 1.0]));
    assert_frame(&built, &side, Some(&end), [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let corner = vertex(&vertex_at(&part, [2.0, 2.0, 0.0]));
    let frame = frame_of(&built, corner, None);
    let (at, z) = placed(&frame);
    assert_close(at, [2.0, 2.0, 0.0]);
    assert_close(z, [0.0, 0.0, 1.0]);
    assert_close(placed_x(&frame), [1.0, 0.0, 0.0]);
}

fn placed_x(frame: &CoordinateSystem) -> [f64; 3] {
    [0, 1, 2].map(|k| frame.u()[k].to_f64())
}

/// Looking down at `(x, y)` from above the plate.
fn from_above(x: f64, y: f64) -> Pointer {
    Pointer {
        ray: Ray::try_new(v([x, y, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube {
            radius: Scalar::from_f64(0.01),
        },
    }
}

/// The model of `part`, drawn: what a frame is picked in.
fn drawn_model(part: &Part) -> PartView {
    let mut view = PartView::new();
    model_of(part).draw("model", &mut view).unwrap();
    view.finish()
}

/// What a pick for a frame finds, as the frame's element and where it sits.
fn picked(view: &PartView, x: f64, y: f64) -> (Selected, Option<Selected>) {
    let hit = view
        .pick(&from_above(x, y), &[drawn::FRAME.to_string()])
        .unwrap_or_else(|| panic!("nothing to put a frame on at ({x}, {y})"));
    let entity = MateEntity::from_picked(&hit.element)
        .unwrap_or_else(|| panic!("{:?} is no frame", hit.element));
    assert_eq!(entity.part(), "", "the part itself is picked in");
    (entity.on, entity.at)
}

/// A pick for a frame over a face finds the face — at its middle where the
/// pointer is far from every corner and side — and, near a corner or a side
/// of it, there: the corner before the side.
#[test]
fn a_pick_over_a_face_puts_the_frame_at_the_corner_or_side_it_is_near() {
    let built = plate();
    let part: NamedModel = model_of(&built);
    let view = drawn_model(&built);
    let (on, at) = picked(&view, 1.0, 1.7);
    assert_eq!(on, top());
    assert_eq!(at, None);
    let (on, at) = picked(&view, 0.5, 0.012);
    assert_eq!(on, top());
    assert_eq!(at, Some(edge(&edge_at(&part, [1.0, 0.0, 1.0]))));
    let (on, at) = picked(&view, 0.02, 0.02);
    assert_eq!(on, top());
    assert_eq!(at, Some(vertex(&vertex_at(&part, [0.0, 0.0, 1.0]))));
}

/// A pick for a frame right on a straight edge finds it, at its middle —
/// near an end of it, there; right on a corner, the face there, at it.
#[test]
fn a_pick_over_an_edge_puts_the_frame_at_its_middle_or_the_end_it_is_near() {
    let built = plate();
    let part = model_of(&built);
    let view = drawn_model(&built);
    let side = edge(&edge_at(&part, [1.0, 0.0, 1.0]));
    assert_eq!(picked(&view, 1.0, 0.0), (side.clone(), None));
    assert_eq!(
        picked(&view, 0.2, 0.0),
        (side, Some(vertex(&vertex_at(&part, [0.0, 0.0, 1.0]))))
    );
}
