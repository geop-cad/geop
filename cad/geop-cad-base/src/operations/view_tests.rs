//! Picking with a [`PartView`]: what a pointer is over, as drawn.

use crate::examples::n;
use geop_core_math::{
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    EntityRef, ORIGIN, Part, PlacedSketch,
    operation::Role,
    ui::{PartView, Pointer, Reach},
};
use geop_ops_extrude_revolve::shapes::cube_solid;
use geop_ops_sketch::Sketch;

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// A pointer from `origin` along `dir`, reaching 0.009 units.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    Pointer {
        ray: Ray::try_new(
            v(origin[0], origin[1], origin[2]),
            v(dir[0], dir[1], dir[2]),
        )
        .unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 5.0], [0.0, 0.0, -1.0])
}

/// The unit cube, named `cube(c)`.
fn cube() -> Part<S> {
    let mut part = Part::new();
    cube_solid(&mut part, "c", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    part
}

/// What `pointer` picks that can fill one of `roles`.
fn picked(view: &PartView<S>, pointer: &Pointer<S>, roles: &[Role]) -> Option<EntityRef> {
    view.pick(pointer, roles, None).map(|h| h.entity)
}

/// A cube's vertices, edges and faces.
const ANY: &[Role] = &[Role::Point, Role::Edge, Role::Plane];

/// A face is hit where the ray enters the part; a solid on any of its
/// faces; and nothing where the ray misses.
#[test]
fn faces_and_solids() {
    let view = PartView::of(&cube()).unwrap();
    let hit = view.pick(&down(0.5, 0.5), &[Role::Plane], None).unwrap();
    assert!(matches!(hit.entity, EntityRef::Face { .. }));
    assert!(hit.point[2].could_be_equal(S::ONE), "{hit:?}");
    assert!(matches!(
        picked(&view, &down(0.5, 0.5), &[Role::Solid]),
        Some(EntityRef::Solid { .. })
    ));
    assert_eq!(picked(&view, &down(5.0, 5.0), &[Role::Plane]), None);
}

/// The smallest entity near the pointer wins — a corner, else an edge,
/// else the face — but never one hidden behind the face in front.
#[test]
fn the_smallest_visible_entity_wins() {
    let view = PartView::of(&cube()).unwrap();
    let kind = |pointer: Pointer<S>| match picked(&view, &pointer, ANY) {
        Some(EntityRef::Vertex { .. }) => "vertex",
        Some(EntityRef::Edge { .. }) => "edge",
        Some(EntityRef::Face { .. }) => "face",
        other => panic!("{other:?}"),
    };
    assert_eq!(kind(down(1.005, 1.005)), "vertex");
    assert_eq!(kind(down(0.5, 0.995)), "edge");
    assert_eq!(kind(down(0.5, 0.5)), "face");
    // From the side, through the front face: the back face's edges lie
    // right behind the pointer, and are not picked.
    assert_eq!(kind(pointer([0.5, -5.0, 0.995], [0.0, 1.0, 0.0])), "edge");
    assert_eq!(kind(pointer([0.5, -5.0, 0.5], [0.0, 1.0, 0.0])), "face");
}

/// A sketch is hit inside its closed region and on its curves, not beside
/// them; of two sketches, the nearer one wins.
#[test]
fn sketches() {
    let mut square = Sketch::new();
    let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|c| square.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..4 {
        square.add_line(p[i], p[(i + 1) % 4]);
    }
    let placed = |z: f64| PlacedSketch {
        plane: CoordinateSystem::try_new(
            v(0.0, 0.0, z),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.0, 0.0, 1.0),
        )
        .unwrap(),
        sketch: square.clone(),
    };
    let mut part = Part::<S>::new();
    part.add_sketch(placed(0.0), "low").unwrap();
    part.add_sketch(placed(1.0), "high").unwrap();
    let view = PartView::of(&part).unwrap();
    let sketch = |pointer: Pointer<S>| picked(&view, &pointer, &[Role::Sketch]);
    let named = |name: &str| Some(EntityRef::Sketch { name: name.into() });
    assert_eq!(sketch(down(0.5, 0.5)), named("high"));
    assert_eq!(sketch(down(1.005, 0.5)), named("high"));
    assert_eq!(sketch(down(1.5, 0.5)), None);
    assert_eq!(
        sketch(pointer([0.5, 0.5, -5.0], [0.0, 0.0, 1.0])),
        named("low")
    );
}

/// Datums are hit as drawn — a plane as a square around the drawing's
/// center — and only for the roles they can fill.
#[test]
fn datums() {
    let mut part = cube();
    let frame = |z: f64| {
        CoordinateSystem::try_new(
            v(0.0, 0.0, z),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.0, 0.0, 1.0),
        )
        .unwrap()
    };
    part.add_datum(
        Datum {
            kind: DatumKind::Plane,
            frame: frame(3.0),
        },
        "above",
    )
    .unwrap();
    let view = PartView::of(&part).unwrap();
    let datum = Some(EntityRef::datum("above"));
    // Beside the cube, but within the square drawn around its center: √3
    // across, the cube's diagonal.
    assert_eq!(picked(&view, &down(1.2, 0.5), &[Role::Plane]), datum);
    assert_eq!(picked(&view, &down(1.2, 0.5), &[Role::Point]), None);
    // In front of the cube's top, the plane is nearer.
    assert_eq!(picked(&view, &down(0.5, 0.5), &[Role::Plane]), datum);
    assert_eq!(picked(&view, &down(50.0, 0.5), &[Role::Plane]), None);
}

/// A frame datum — here the origin every part has — is hit before
/// anything else: its ball, as the whole frame, else an axis, else a
/// plane's square.
#[test]
fn frames() {
    let view = PartView::of(&cube()).unwrap();
    let frame = [Role::Point, Role::Line, Role::Plane];
    // Ten reaches of 0.009 each: the axes reach 0.09 out.
    assert_eq!(
        picked(&view, &down(0.0, 0.0), &frame),
        Some(EntityRef::datum(ORIGIN))
    );
    assert_eq!(
        picked(&view, &down(0.06, 0.0), &frame),
        Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Axis(FrameAxis::X)
        ))
    );
    assert_eq!(
        picked(&view, &down(0.04, 0.04), &frame),
        Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Plane(FrameAxis::Z)
        ))
    );
    // Looking for what no part of a frame can be, the cube under it.
    assert!(matches!(
        picked(&view, &down(0.04, 0.04), &[Role::Solid]),
        Some(EntityRef::Solid { .. })
    ));
}

/// A sketch's lines are picked as lines and its points as points — the
/// nearer of two sketches' — and, within a sketch, only that sketch's.
#[test]
fn sketch_lines() {
    let mut square = Sketch::new();
    let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|c| square.add_point(n(c[0]), n(c[1])))
        .collect();
    let right = square.add_line(p[1], p[2]);
    for i in [0, 2, 3] {
        square.add_line(p[i], p[(i + 1) % 4]);
    }
    let placed = |z: f64| PlacedSketch {
        plane: CoordinateSystem::try_new(
            v(0.0, 0.0, z),
            v(1.0, 0.0, 0.0),
            v(0.0, 1.0, 0.0),
            v(0.0, 0.0, 1.0),
        )
        .unwrap(),
        sketch: square.clone(),
    };
    let mut part = Part::<S>::new();
    part.add_sketch(placed(0.0), "low").unwrap();
    part.add_sketch(placed(1.0), "high").unwrap();
    let view = PartView::of(&part).unwrap();
    let line = |sketch: &str| {
        Some(EntityRef::SketchCurve {
            sketch: sketch.into(),
            curve: right,
        })
    };
    assert_eq!(
        picked(&view, &down(1.005, 0.5), &[Role::Line]),
        line("high")
    );
    let low = EntityRef::Sketch { name: "low".into() };
    let within = view
        .pick(&down(1.005, 0.5), &[Role::Line], Some(&low))
        .map(|h| h.entity);
    assert_eq!(within, line("low"));
    // Inside the square, no line is near.
    assert_eq!(picked(&view, &down(0.5, 0.5), &[Role::Line]), None);
    // Its corners are points, of the nearer sketch.
    assert_eq!(
        picked(&view, &down(1.005, 0.995), &[Role::Point]),
        Some(EntityRef::SketchPoint {
            sketch: "high".into(),
            point: p[2],
        })
    );
}

/// Seen at a glancing angle, an edge of the face the ray hits is picked
/// wherever the pointer is within reach of it: aimed at the cube's top 0.1
/// short of its far edge, from nearly level with the top, the ray passes
/// a thousandth from the edge, within its reach of about 0.005 — but meets
/// it 0.1 further along than the top, twenty reaches. The face does not
/// hide its own edge. (The near top edge it passes 0.008 above, out of
/// reach.)
#[test]
fn edge_of_the_face_hit_at_a_glancing_angle() {
    let part = cube();
    let view = PartView::of(&part).unwrap();
    let (origin, aim) = ([0.5, -10.0, 1.1], [0.5, 0.9, 1.0]);
    let glancing = Pointer {
        ray: Ray::try_new(
            v(origin[0], origin[1], origin[2]),
            v(aim[0] - origin[0], aim[1] - origin[1], aim[2] - origin[2]),
        )
        .unwrap(),
        reach: Reach::Cone {
            slope: S::from_f64(0.0005),
        },
    };
    let Some(EntityRef::Edge { name }) = picked(&view, &glancing, &[Role::Edge]) else {
        panic!("no edge picked");
    };
    let edge = part.edge_id(&name).unwrap();
    let model = part.topology();
    let e = model.get_edge(edge).unwrap();
    for vertex in [e.start_vertex, e.end_vertex] {
        let p = model.get_vertex(vertex).unwrap().point;
        assert!(
            p[1].could_be_equal(S::ONE) && p[2].could_be_equal(S::ONE),
            "{name} is not the far top edge"
        );
    }
}
