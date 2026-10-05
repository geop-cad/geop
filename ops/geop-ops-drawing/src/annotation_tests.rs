//! Annotations: measured in the views they name, drawn on the sheet and
//! written out, and refused where a view cannot show what they measure.

use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
use geop_ops::{Part, RefId};
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::{
    Along, Annotation, DrawingArgs, DrawnView, EdgeShape, Layout, Leader, Pickable, Target,
    ViewKind, compose, layout, sheet::Layer, to_dxf, writer_tests::read_dxf,
};

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
}

const FRONT: DrawnView = DrawnView::View(ViewKind::Front);
const TOP: DrawnView = DrawnView::View(ViewKind::Top);

/// A 20 x 10 x 30 box beside a cylinder of radius 5 standing on it.
fn part() -> Part<S> {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "box", v(0.0, 0.0, 0.0), v(20.0, 10.0, 30.0)).unwrap();
    revolved_cylinder(
        &mut part,
        "pin",
        v(40.0, 5.0, 0.0),
        S::from_f64(5.0),
        S::from_f64(10.0),
    )
    .unwrap();
    part
}

fn args(annotations: Vec<Annotation>) -> DrawingArgs {
    DrawingArgs {
        views: vec![ViewKind::Front, ViewKind::Top, ViewKind::Iso],
        annotations,
        ..Default::default()
    }
}

/// The vertices `view` of `layout` shows, by name, where they are.
fn vertices(layout: &Layout, view: DrawnView) -> Vec<(String, [f64; 2])> {
    layout
        .candidates
        .iter()
        .filter(|c| c.view == view)
        .filter_map(|c| match &c.what {
            Pickable::Point {
                target: Target::Vertex { name },
                at,
            } => Some((name.clone(), *at)),
            _ => None,
        })
        .collect()
}

/// The edges `view` of `layout` shows of shape `shape`, by name.
fn edges(layout: &Layout, view: DrawnView, shape: fn(&EdgeShape) -> bool) -> Vec<String> {
    let mut names: Vec<String> = layout
        .candidates
        .iter()
        .filter(|c| c.view == view)
        .filter_map(|c| match &c.what {
            Pickable::Edge { name, shape: s, .. } if shape(s) => Some(name.clone()),
            _ => None,
        })
        .collect();
    names.dedup();
    names
}

/// The text the dimension `annotation` shows on `part`'s drawing, laid out
/// as `laid`: an annotation does not move the views.
fn value(part: &Part<S>, laid: &Layout, annotation: Annotation) -> String {
    annotation
        .drawn(part, laid)
        .unwrap()
        .text
        .map(|t| t.text)
        .unwrap_or_default()
}

/// Each kind of annotation measures what its view shows: the box's front
/// diagonal and its width from the front, the pin's radius and diameter
/// from above, a right angle between two of the box's edges; a centre mark
/// draws two centre lines, a note its text and a leader. All of it is
/// written into the DXF file.
#[test]
fn every_annotation_measures_its_view() {
    let part = part();
    let laid = layout(&part, &args(Vec::new()), "", &[]).unwrap();
    // The box's corner at `(x, y, z)`, by name.
    let at = |x: f64, y: f64, z: f64| {
        let model = part.topology();
        let mut names: Vec<&str> = part
            .names()
            .iter()
            .filter(|(id, _)| match id {
                RefId::Vertex(id) => {
                    let p = model.get_vertex(*id).unwrap().point;
                    [x, y, z]
                        .iter()
                        .zip(p.to_array())
                        .all(|(&c, q)| q.could_be_equal(S::from_f64(c)))
                }
                _ => false,
            })
            .map(|(_, name)| name)
            .collect();
        names.sort();
        Target::Vertex {
            name: names[0].to_string(),
        }
    };
    assert!(
        vertices(&laid, FRONT).len() >= 4,
        "the front view's corners can be picked"
    );
    let distance = |along| Annotation::Distance {
        view: FRONT,
        from: at(0.0, 0.0, 0.0),
        to: at(20.0, 0.0, 30.0),
        along,
        label: [0.0, 12.0],
    };
    assert_eq!(value(&part, &laid, distance(Along::Aligned)), "36.06");
    assert_eq!(value(&part, &laid, distance(Along::Horizontal)), "20");
    assert_eq!(value(&part, &laid, distance(Along::Vertical)), "30");

    let circle = edges(&laid, TOP, |s| matches!(s, EdgeShape::Circle { .. }));
    assert!(!circle.is_empty(), "the pin's rim is seen round from above");
    let radius = |diameter| Annotation::Radius {
        view: TOP,
        edge: circle[0].clone(),
        diameter,
        label: [8.0, 8.0],
    };
    assert_eq!(value(&part, &laid, radius(false)), "R5");
    assert_eq!(value(&part, &laid, radius(true)), "⌀10");

    let lines = edges(&laid, FRONT, |s| matches!(s, EdgeShape::Line { .. }));
    let square = lines
        .iter()
        .flat_map(|a| lines.iter().map(move |b| (a, b)))
        .find_map(|(a, b)| {
            let angle = Annotation::Angle {
                view: FRONT,
                a: a.clone(),
                b: b.clone(),
                label: [3.0, 3.0],
            };
            angle.drawn(&part, &laid).ok().map(|d| d.text.unwrap().text)
        })
        .expect("two of the box's edges meet");
    assert_eq!(square, "90°");

    let all = vec![
        distance(Along::Aligned),
        radius(true),
        Annotation::CenterMark {
            view: TOP,
            edge: circle[0].clone(),
        },
        Annotation::Note {
            text: "BREAK EDGES".into(),
            leader: Some(Leader {
                view: FRONT,
                target: at(20.0, 0.0, 30.0),
            }),
            label: [15.0, 15.0],
        },
    ];
    let sheet = compose(&part, &args(all), "", &[]).unwrap();
    let entities = read_dxf(&to_dxf(&sheet));
    let texts: Vec<&str> = entities
        .iter()
        .filter(|(k, l, _)| k == "TEXT" && l == "DIMENSIONS")
        .map(|(_, _, t)| t.as_str())
        .collect();
    for text in ["36.06", "%%c10", "BREAK EDGES"] {
        assert!(texts.contains(&text), "{text} is not written: {texts:?}");
    }
    // The pin's own centre mark from above, and the one added.
    let centre_lines = sheet
        .strokes
        .iter()
        .filter(|s| s.layer == Layer::Center)
        .count();
    assert!(centre_lines >= 4, "{centre_lines} centre lines");
}

/// An annotation that cannot be drawn is refused, saying why: in the
/// isometric view, which shows no length truly; of a circle its view does
/// not see round; in a view the drawing no longer has.
#[test]
fn annotations_a_view_cannot_show_are_refused() {
    let part = part();
    let laid = layout(&part, &args(Vec::new()), "", &[]).unwrap();
    let circle = edges(&laid, TOP, |s| matches!(s, EdgeShape::Circle { .. }))[0].clone();
    let radius = |view| Annotation::Radius {
        view,
        edge: circle.clone(),
        diameter: false,
        label: [5.0, 5.0],
    };
    let refused = |annotation: Annotation| {
        compose(&part, &args(vec![annotation]), "", &[])
            .unwrap_err()
            .to_string()
    };
    assert!(refused(radius(DrawnView::View(ViewKind::Iso))).contains("isometric"));
    assert!(refused(radius(FRONT)).contains("round"));
    assert!(refused(radius(DrawnView::View(ViewKind::Right))).contains("no right view"));
    assert!(refused(radius(DrawnView::Section)).contains("section"));
}

/// An annotation is saved by the names of its entities and its view.
#[test]
fn annotations_are_saved_by_name() {
    let annotation = Annotation::Distance {
        view: FRONT,
        from: Target::Vertex { name: "a".into() },
        to: Target::Center { edge: "e".into() },
        along: Along::Horizontal,
        label: [1.0, 2.0],
    };
    let json = serde_json::to_value(&annotation).unwrap();
    assert_eq!(
        json,
        serde_json::json!({
            "type": "distance",
            "view": "front",
            "from": {"type": "vertex", "name": "a"},
            "to": {"type": "center", "edge": "e"},
            "along": "horizontal",
            "label": [1.0, 2.0],
        })
    );
    let back: Annotation = serde_json::from_value(json).unwrap();
    assert_eq!(back, annotation);
}
