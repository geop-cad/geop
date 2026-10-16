//! Sketches as drawn ([`SketchDrawer`]): picked inside their closed
//! regions, on their curves and at their points, the nearer of two first.

use geop_core_editor::{
    editing::Pointer,
    registry::Registry,
    testing::pointer,
    view::{PartView, drawn},
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::CoordinateSystem,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{
    element::PickedElement, part::Part, reader::Reader, runner::Runner, values::Design,
};
use geop_core_sketch::{CurveId, PointId};

use crate::{PlacedSketch, Sketch, sketch};

fn down(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 5.0], [0.0, 0.0, -1.0])
}

fn given_sketch(_: &Reader, s: &PlacedSketch<Design>) -> GeopResult<PlacedSketch<Design>> {
    Ok(s.clone())
}

/// A part of `sketches` by name, each added by a step of its name, drawn
/// as an editor draws a part, by the drawers `register` adds.
fn drawn(sketches: &[(&str, PlacedSketch<S>)]) -> PartView {
    let mut part = Part::default();
    let runner = Runner::new(Default::default());
    for (name, placed) in sketches {
        let placed = PlacedSketch {
            name: name.to_string(),
            ..placed.clone()
        };
        (runner.run_step(&mut part, name, |ctx| {
            ctx.produce(&sketch(name), &placed, given_sketch)
        }))
        .unwrap();
    }
    let registry = crate::register(Registry::default()).unwrap();
    registry.draw(&part.current()).unwrap()
}

/// What `pointer` picks of the kinds `accepts`.
fn picked(view: &PartView, pointer: &Pointer<S>, accepts: &[&str]) -> Option<PickedElement> {
    let accepts: Vec<String> = accepts.iter().map(|k| k.to_string()).collect();
    view.pick(pointer, &accepts).map(|h| h.element)
}

/// A square sketch, its corners, and its right side.
fn square() -> (Sketch, Vec<PointId>, CurveId) {
    let mut square = Sketch::new();
    let p: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|c| square.add_point(S::from_f64(c[0]), S::from_f64(c[1])))
        .collect();
    let right = square.add_line(p[1], p[2]);
    for i in [0, 2, 3] {
        square.add_line(p[i], p[(i + 1) % 4]);
    }
    (square, p, right)
}

/// `sketch` on the plane `z` up.
fn placed(sketch: &Sketch, z: f64) -> PlacedSketch<S> {
    let v = |x: f64, y: f64, z: f64| Vector3::from_array([x, y, z].map(S::from_f64));
    PlacedSketch {
        name: String::new(),
        plane: CoordinateSystem::world_at(v(0.0, 0.0, z)),
        sketch: sketch.clone(),
    }
}

/// A sketch is hit inside its closed region and on its curves, not beside
/// them; of two sketches, the nearer one wins.
#[test]
fn sketches() {
    let (square, _, _) = square();
    let view = drawn(&[
        ("low", placed(&square, 0.0)),
        ("high", placed(&square, 1.0)),
    ]);
    let sketch_of =
        |pointer: Pointer<S>| picked(&view, &pointer, &[drawn::SKETCH]).map(|p| p.target);
    let named = |name: &str| Some(sketch(name).location().clone());
    assert_eq!(sketch_of(down(0.5, 0.5)), named("high"));
    assert_eq!(sketch_of(down(1.005, 0.5)), named("high"));
    assert_eq!(sketch_of(down(1.5, 0.5)), None);
    assert_eq!(
        sketch_of(pointer([0.5, 0.5, -5.0], [0.0, 0.0, 1.0])),
        named("low")
    );
}

/// A sketch's lines are picked as lines and its points as points — the
/// nearer of two sketches'.
#[test]
fn sketch_lines() {
    let (square, p, right) = square();
    let view = drawn(&[
        ("low", placed(&square, 0.0)),
        ("high", placed(&square, 1.0)),
    ]);
    let element = |sketch_name: &str, kind: &str, name: String| PickedElement {
        target: sketch(sketch_name).location().clone(),
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
