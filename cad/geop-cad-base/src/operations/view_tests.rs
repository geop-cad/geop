//! Picking with a [`PartView`]: what a pointer is over, as drawn.

use geop_core_math::{
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_sketch::Sketch;
use geop_ops::{
    EntityRef, ORIGIN, Part, PlacedSketch,
    ui::{PartView, Pointer, Reach, Target},
};
use geop_ops_extrude_revolve::cube_solid;

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

/// What `pointer` picks among `targets`, and its kind.
fn picked(view: &PartView<S>, pointer: &Pointer<S>, targets: &[Target]) -> Option<EntityRef> {
    view.pick(pointer, targets).map(|h| h.entity)
}

const ANY: &[Target] = &[Target::Vertex, Target::Edge, Target::Face];

/// A face is hit where the ray enters the part; a solid on any of its
/// faces; and nothing where the ray misses.
#[test]
fn faces_and_solids() {
    let view = PartView::of(&cube()).unwrap();
    let hit = view.pick(&down(0.5, 0.5), &[Target::Face]).unwrap();
    assert!(matches!(hit.entity, EntityRef::Face { .. }));
    assert!(hit.point[2].could_be_equal(S::ONE), "{hit:?}");
    assert!(matches!(
        picked(&view, &down(0.5, 0.5), &[Target::Solid]),
        Some(EntityRef::Solid { .. })
    ));
    assert_eq!(picked(&view, &down(5.0, 5.0), &[Target::Face]), None);
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
        .map(|c| square.add_point(c[0], c[1]))
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
    let sketch = |pointer: Pointer<S>| picked(&view, &pointer, &[Target::Sketch]);
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
/// center — and only those of the kinds looked for.
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
    assert_eq!(
        picked(&view, &down(1.2, 0.5), &[Target::Datum(DatumKind::Plane)]),
        datum
    );
    assert_eq!(
        picked(&view, &down(1.2, 0.5), &[Target::Datum(DatumKind::Point)]),
        None
    );
    // In front of the cube's top, the plane is nearer.
    assert_eq!(
        picked(
            &view,
            &down(0.5, 0.5),
            &[Target::Face, Target::Datum(DatumKind::Plane)]
        ),
        datum
    );
    assert_eq!(
        picked(&view, &down(50.0, 0.5), &[Target::Datum(DatumKind::Plane)]),
        None
    );
}

/// A frame datum — here the origin every part has — is hit before
/// anything else: its ball, as the whole frame, else an axis, else a
/// plane's square.
#[test]
fn frames() {
    let view = PartView::of(&cube()).unwrap();
    let frame = [
        Target::Datum(DatumKind::Point),
        Target::Datum(DatumKind::Axis),
        Target::Datum(DatumKind::Plane),
        Target::Face,
    ];
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
    // Without the frame's kinds, the cube's face under it.
    assert!(matches!(
        picked(&view, &down(0.04, 0.04), &[Target::Face]),
        Some(EntityRef::Face { .. })
    ));
}
