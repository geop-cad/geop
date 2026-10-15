//! Picking with a [`PartView`]: what a pointer is over, as drawn.

use crate::operations::testing::{origin_axis, origin_plane, pointer};
use geop_core_editor::{
    editing::{Pointer, Reach},
    selectable,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{CoordinateSystem, DatumKind, FrameAxis, Ray},
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::{
    element::{ElementReference, PickedElement},
    part::{Part, Reader},
    values::Design,
};
use geop_ops_datums::ORIGIN;
use geop_ops_extrude_revolve::{
    shapes::{block_part, v3},
    testing::n,
};
use geop_ops_model::{NamedModel, Solid, testing::given_part};
use geop_ops_sketch::{PlacedSketch, Sketch, sketch};
use geop_ops_view::{PartView, ViewDatum, drawn};

fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 5.0], [0.0, 0.0, -1.0])
}

/// The unit cube, named `cube(c)`.
fn cube() -> NamedModel<S> {
    block_part("c", [0.0; 3], [1.0; 3])
}

fn given_sketch(_: &Reader, s: &PlacedSketch<Design>) -> GeopResult<PlacedSketch<Design>> {
    Ok(s.clone())
}

/// `model`, with the datums and `sketches` by name: a part a program of one
/// step would have built.
fn built(model: &NamedModel<S>, sketches: &[(&str, PlacedSketch<S>)]) -> Part {
    let mut part: Part = given_part(model).unwrap();
    for (name, placed) in sketches {
        let placed = PlacedSketch {
            name: name.to_string(),
            ..placed.clone()
        };
        part.define_target(&sketch(name), &placed, given_sketch)
            .unwrap();
    }
    part
}

/// `part`, as drawn.
fn drawn(part: &Part) -> PartView {
    crate::part::draw(&part.current()).unwrap()
}

/// What `pointer` picks of the kinds `accepts` ([`drawn`]).
fn picked(view: &PartView, pointer: &Pointer<S>, accepts: &[&str]) -> Option<PickedElement> {
    let accepts: Vec<String> = accepts.iter().map(|k| k.to_string()).collect();
    view.pick(pointer, &accepts).map(|h| h.element)
}

/// The kind of what `pointer` picks of the kinds `accepts`.
fn kind(view: &PartView, pointer: &Pointer<S>, accepts: &[&str]) -> Option<String> {
    picked(view, pointer, accepts).map(|p| p.kind)
}

/// A cube's vertices, edges and faces.
const ANY: &[&str] = &[drawn::VERTEX, drawn::EDGE, drawn::FACE];

/// A face is hit where the ray enters the part; a solid on any of its
/// faces; and nothing where the ray misses.
#[test]
fn faces_and_solids() {
    let part = built(&cube(), &[]);
    let view = drawn(&part);
    let accepts = vec![drawn::FACE.to_string()];
    let hit = view.pick(&down(0.5, 0.5), &accepts).unwrap();
    assert_eq!(hit.element.kind, drawn::FACE);
    assert!(hit.point[2].could_be_equal(S::ONE), "{hit:?}");
    let solid = selectable::pick::<ElementReference<NamedModel<Design>, Solid>, _>(
        &view,
        &part.current(),
        &down(0.5, 0.5),
    );
    assert!(solid.is_some());
    assert_eq!(picked(&view, &down(5.0, 5.0), &[drawn::FACE]), None);
}

/// The smallest entity near the pointer wins — a corner, else an edge,
/// else the face — but never one hidden behind the face in front.
#[test]
fn the_smallest_visible_entity_wins() {
    let view = drawn(&built(&cube(), &[]));
    let kind = |pointer: Pointer<S>| kind(&view, &pointer, ANY).unwrap();
    assert_eq!(kind(down(1.005, 1.005)), drawn::VERTEX);
    assert_eq!(kind(down(0.5, 0.995)), drawn::EDGE);
    assert_eq!(kind(down(0.5, 0.5)), drawn::FACE);
    // From the side, through the front face: the back face's edges lie
    // right behind the pointer, and are not picked.
    assert_eq!(
        kind(pointer([0.5, -5.0, 0.995], [0.0, 1.0, 0.0])),
        drawn::EDGE
    );
    assert_eq!(
        kind(pointer([0.5, -5.0, 0.5], [0.0, 1.0, 0.0])),
        drawn::FACE
    );
}

/// A square sketch, and its right side.
fn square() -> (
    Sketch,
    Vec<geop_core_sketch::PointId>,
    geop_core_sketch::CurveId,
) {
    let mut square = Sketch::new();
    let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|c| square.add_point(n(c[0]), n(c[1])))
        .collect();
    let right = square.add_line(p[1], p[2]);
    for i in [0, 2, 3] {
        square.add_line(p[i], p[(i + 1) % 4]);
    }
    (square, p, right)
}

/// `sketch` on the plane `z` up.
fn placed(sketch: &Sketch, z: f64) -> PlacedSketch<S> {
    PlacedSketch {
        name: String::new(),
        plane: CoordinateSystem::try_new(
            v3(0.0, 0.0, z),
            v3(1.0, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        )
        .unwrap(),
        sketch: sketch.clone(),
    }
}

/// A sketch is hit inside its closed region and on its curves, not beside
/// them; of two sketches, the nearer one wins.
#[test]
fn sketches() {
    let (square, _, _) = square();
    let view = drawn(&built(
        &NamedModel::new(),
        &[
            ("low", placed(&square, 0.0)),
            ("high", placed(&square, 1.0)),
        ],
    ));
    let sketch_of =
        |pointer: Pointer<S>| picked(&view, &pointer, &[drawn::SKETCH]).map(|p| p.target);
    let named = |name: &str| Some(sketch(name).name().to_string());
    assert_eq!(sketch_of(down(0.5, 0.5)), named("high"));
    assert_eq!(sketch_of(down(1.005, 0.5)), named("high"));
    assert_eq!(sketch_of(down(1.5, 0.5)), None);
    assert_eq!(
        sketch_of(pointer([0.5, 0.5, -5.0], [0.0, 0.0, 1.0])),
        named("low")
    );
}

/// Datums are hit as drawn — a plane as a square around the drawing's
/// center — and only for the kinds they can be. (The plane is drawn
/// directly, not added by a step.)
#[test]
fn datums() {
    let frame = |z: f64| {
        CoordinateSystem::try_new(
            v3(0.0, 0.0, z),
            v3(1.0, 0.0, 0.0),
            v3(0.0, 1.0, 0.0),
            v3(0.0, 0.0, 1.0),
        )
        .unwrap()
    };
    let mut view = drawn(&built(&cube(), &[]));
    view.datums.push(ViewDatum {
        name: "above".into(),
        kind: DatumKind::Plane,
        frame: frame(3.0),
        components: Vec::new(),
    });
    let datum = Some("above".to_string());
    let name =
        |pointer: Pointer<S>, accepts: &[&str]| picked(&view, &pointer, accepts).map(|p| p.name);
    // Beside the cube, but within the square drawn around its center: √3
    // across, the cube's diagonal.
    assert_eq!(name(down(1.2, 0.5), &[drawn::DATUM]), datum);
    assert_eq!(name(down(1.2, 0.5), &[drawn::VERTEX]), None);
    // In front of the cube's top, the plane is nearer.
    assert_eq!(name(down(0.5, 0.5), &[drawn::DATUM, drawn::FACE]), datum);
    assert_eq!(name(down(50.0, 0.5), &[drawn::DATUM]), None);
}

/// A frame datum — here the origin every part has — is hit before
/// anything else: its ball, as the whole frame, else an axis, else a
/// plane's square.
#[test]
fn frames() {
    let part = built(&cube(), &[]);
    let view = drawn(&part);
    let frame = [drawn::DATUM, drawn::VERTEX, drawn::EDGE, drawn::FACE];
    let name =
        |pointer: Pointer<S>, accepts: &[&str]| picked(&view, &pointer, accepts).map(|p| p.name);
    // Ten reaches of 0.009 each: the axes reach 0.09 out.
    assert_eq!(name(down(0.0, 0.0), &frame), Some(ORIGIN.to_string()));
    assert_eq!(
        name(down(0.06, 0.0), &frame),
        Some(origin_axis(FrameAxis::X).name().to_string())
    );
    assert_eq!(
        name(down(0.04, 0.04), &frame),
        Some(origin_plane(FrameAxis::Z).name().to_string())
    );
    // Looking for what no part of a frame can be, the cube under it.
    let solid = selectable::pick::<ElementReference<NamedModel<Design>, Solid>, _>(
        &view,
        &part.current(),
        &down(0.04, 0.04),
    );
    assert!(solid.is_some());
}

/// A sketch's lines are picked as lines and its points as points — the
/// nearer of two sketches'.
#[test]
fn sketch_lines() {
    let (square, p, right) = square();
    let view = drawn(&built(
        &NamedModel::new(),
        &[
            ("low", placed(&square, 0.0)),
            ("high", placed(&square, 1.0)),
        ],
    ));
    let element = |sketch_name: &str, kind: &str, name: String| PickedElement {
        target: sketch(sketch_name).name().to_string(),
        kind: kind.to_string(),
        name,
    };
    assert_eq!(
        picked(&view, &down(1.005, 0.5), &[drawn::SKETCH_CURVE]),
        Some(element("high", drawn::SKETCH_CURVE, right.to_string()))
    );
    // Inside the square, no line is near.
    assert_eq!(picked(&view, &down(0.5, 0.5), &[drawn::SKETCH_CURVE]), None);
    // Its corners are points, of the nearer sketch.
    assert_eq!(
        picked(&view, &down(1.005, 0.995), &[drawn::SKETCH_POINT]),
        Some(element("high", drawn::SKETCH_POINT, p[2].to_string()))
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
    let view = drawn(&built(&part, &[]));
    let (origin, aim) = ([0.5, -10.0, 1.1], [0.5, 0.9, 1.0]);
    let glancing = Pointer {
        ray: Ray::try_new(
            v3(origin[0], origin[1], origin[2]),
            v3(aim[0] - origin[0], aim[1] - origin[1], aim[2] - origin[2]),
        )
        .unwrap(),
        reach: Reach::Cone {
            slope: S::from_f64(0.0005),
        },
    };
    let edge = picked(&view, &glancing, &[drawn::EDGE]).expect("no edge picked");
    let name = edge.name;
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
