use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
use geop_core_topology::FaceId;
use geop_ops::Part;
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::{LineKind, ProjectedView, ViewKind, ViewOptions, project_view};

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
}

fn all_faces(part: &Part<S>) -> Vec<FaceId> {
    let model = part.topology();
    let mut faces: Vec<FaceId> = model.faces.keys().copied().collect();
    faces.sort_by_key(|f| f.0);
    faces
}

fn view(part: &Part<S>, kind: ViewKind, options: ViewOptions) -> ProjectedView<S> {
    project_view(
        part.topology(),
        &all_faces(part),
        &kind.frame().unwrap(),
        &options,
    )
    .unwrap()
}

fn count(view: &ProjectedView<S>, visible: bool) -> usize {
    view.lines.iter().filter(|l| l.visible == visible).count()
}

/// Each line as `(kind, visible, from, to)` on the paper, for a failure to
/// show.
fn summary(view: &ProjectedView<S>) -> Vec<String> {
    view.lines
        .iter()
        .map(|l| {
            let (a, b) = l.curve.domain();
            let (p, q) = (l.curve.evaluate(a).unwrap(), l.curve.evaluate(b).unwrap());
            format!(
                "{:?} {} ({:.3}, {:.3}) -> ({:.3}, {:.3})",
                l.kind,
                if l.visible { "visible" } else { "hidden" },
                p[0].to_f64(),
                p[1].to_f64(),
                q[0].to_f64(),
                q[1].to_f64()
            )
        })
        .collect()
}

fn assert_extents(view: &ProjectedView<S>, expected: [f64; 4]) {
    let e = view.extents().unwrap();
    for k in 0..4 {
        assert!(
            e[k].could_be_equal(S::from_f64(expected[k])),
            "extents {e:?}, expected {expected:?}"
        );
    }
}

/// A 2 x 1 x 3 box seen from the front is a 2 x 3 rectangle: four visible
/// lines, and the edges behind it, running along them, are not drawn.
#[test]
fn box_front_view_is_a_rectangle() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "box", v(0.0, 0.0, 0.0), v(2.0, 1.0, 3.0)).unwrap();
    let front = view(&part, ViewKind::Front, ViewOptions::default());
    assert_eq!(count(&front, true), 4, "{:#?}", summary(&front));
    assert_eq!(count(&front, false), 0);
    assert_extents(&front, [0.0, 0.0, 2.0, 3.0]);

    let top = view(&part, ViewKind::Top, ViewOptions::default());
    assert_eq!(count(&top, true), 4);
    assert_extents(&top, [0.0, 0.0, 2.0, 1.0]);
    let right = view(&part, ViewKind::Right, ViewOptions::default());
    assert_eq!(count(&right, true), 4);
    assert_extents(&right, [0.0, 0.0, 1.0, 3.0]);

    // From a corner, three faces show: nine edges are visible, and the three
    // at the far corner hidden — cut where they pass behind visible ones.
    let iso = view(&part, ViewKind::Iso, ViewOptions::default());
    let edges = |visible: bool| {
        let mut edges: Vec<_> = iso
            .lines
            .iter()
            .filter(|l| l.visible == visible)
            .map(|l| l.edge.unwrap())
            .collect();
        edges.sort_by_key(|e| e.0);
        edges.dedup();
        edges
    };
    let (visible, hidden) = (edges(true), edges(false));
    assert_eq!(visible.len(), 9, "{:#?}", summary(&iso));
    assert_eq!(hidden.len(), 3, "{:#?}", summary(&iso));
    assert!(visible.iter().all(|e| !hidden.contains(e)));
}

/// A standing cylinder seen from the side shows its two silhouettes, and
/// its circles edge on as lines; from above, a circle and nothing else.
#[test]
fn cylinder_side_view_has_silhouettes() {
    let mut part = Part::<S>::new();
    revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.0), S::ONE, S::TWO).unwrap();
    let front = view(&part, ViewKind::Front, ViewOptions::default());
    let silhouettes = front
        .lines
        .iter()
        .filter(|l| l.kind == LineKind::Silhouette && l.visible)
        .count();
    assert_eq!(silhouettes, 2, "{:#?}", summary(&front));
    assert_eq!(count(&front, false), 0, "{:#?}", summary(&front));
    assert_extents(&front, [-1.0, 0.0, 1.0, 2.0]);

    let top = view(&part, ViewKind::Top, ViewOptions::default());
    assert!(
        top.lines
            .iter()
            .all(|l| l.kind == LineKind::Edge && l.visible)
    );
    assert_extents(&top, [-1.0, -1.0, 1.0, 1.0]);
}

/// A shaft of radius 3, threaded M6 from its top 8 down, drawn as drafting
/// draws an external thread: from the front two thin visible lines at the
/// minor diameter, the length of the thread; from above three quarters of
/// a thin circle at it; labelled once.
#[test]
fn a_threaded_shaft_draws_its_thread() {
    use crate::{
        DrawingArgs, compose,
        sheet::{Layer, Shape, Stroke},
    };
    use geop_core_geometry::{nurb_curve::Handedness, shape::Axis};
    use geop_ops::CosmeticThread;

    let mut part = Part::<S>::new();
    revolved_cylinder(
        &mut part,
        "shaft",
        v(0.0, 0.0, 0.0),
        S::from_f64(3.0),
        S::from_f64(10.0),
    )
    .unwrap();
    let minor = 4.917;
    part.add_thread(
        "thread",
        CosmeticThread {
            designation: "M6x1".into(),
            face: "the shaft's side".into(),
            axis: Axis::try_new(v(0.0, 0.0, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
            radius: S::from_f64(3.0),
            major_diameter: 6.0,
            minor_diameter: minor,
            pitch: 1.0,
            length: 8.0,
            internal: false,
            handedness: Handedness::Right,
        },
    )
    .unwrap();
    let args = DrawingArgs {
        views: vec![ViewKind::Front, ViewKind::Top],
        scale: Some(1.0),
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &[]).unwrap();
    let thread = |s: &&Stroke| s.layer == Layer::Thread;
    let lines: Vec<(f64, f64)> = sheet
        .strokes
        .iter()
        .filter(thread)
        .filter_map(|s| match s.shape {
            Shape::Line(a, b) => Some((a[0], (a[1] - b[1]).abs())),
            _ => None,
        })
        .collect();
    assert_eq!(lines.len(), 2, "{lines:?}");
    assert!(lines.iter().all(|(_, length)| (length - 8.0).abs() < 1e-9));
    assert!(
        ((lines[0].0 - lines[1].0).abs() - minor).abs() < 1e-9,
        "{lines:?}"
    );
    let arcs: Vec<f64> = sheet
        .strokes
        .iter()
        .filter(thread)
        .filter_map(|s| match s.shape {
            Shape::Arc { radius, .. } => Some(radius),
            _ => None,
        })
        .collect();
    assert_eq!(arcs.len(), 1, "{arcs:?}");
    assert!((arcs[0] - minor / 2.0).abs() < 1e-9, "{arcs:?}");
    assert_eq!(sheet.labels.iter().filter(|l| l.text == "M6x1").count(), 1);
}
