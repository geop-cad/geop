//! Datums built on a solid, and sketched on.

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{EntityRef, ORIGIN, Operation, Part};
use geop_ops_datums::{AddDatum, AddDatumArgs, Construction, Role, inspect_selection};

use crate::examples;

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

fn face(name: &str) -> EntityRef {
    EntityRef::Face { name: name.into() }
}
fn edge(name: &str) -> EntityRef {
    EntityRef::Edge { name: name.into() }
}
fn vertex(name: &str) -> EntityRef {
    EntityRef::Vertex { name: name.into() }
}
fn axis(axis: FrameAxis) -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Axis(axis))
}
fn base(normal: FrameAxis) -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(normal))
}

/// The 2 x 2 x 1 box with a hole of radius 0.4 drilled 0.5 deep into
/// the middle of its top.
fn drilled_box() -> Part<S> {
    examples::box_with_drill_hole().build().unwrap()
}

/// The datum built from `selection` by `construction` in `part`.
fn datum(part: &Part<S>, selection: Vec<EntityRef>, construction: Construction) -> Datum<S> {
    let args = AddDatumArgs {
        selection,
        construction,
    };
    let part = AddDatum.apply(part.clone(), "d", &args).unwrap();
    part.datum(part.datum_id("d").unwrap()).unwrap().clone()
}

/// Sketch data is solved, not exact: agreeing to 1e-9 is agreeing.
fn assert_at(frame: &CoordinateSystem<S>, origin: [f64; 3], w: [f64; 3]) {
    let [x, y, z] = origin;
    let off = frame.origin().sub(&v(x, y, z)).norm().to_f64();
    assert!(off < 1e-9, "origin of {frame} is {off} off {origin:?}");
    let [x, y, z] = w;
    let w = v(x, y, z).normalize().unwrap();
    let off = frame.w().sub(&w).norm().to_f64();
    assert!(off < 1e-9, "w of {frame} is {off} off {w:?}");
}

/// Whether `p` lies on the plane (a frame whose `w` is its normal) —
/// to within what solved sketch data is.
fn on_plane(frame: &CoordinateSystem<S>, p: [f64; 3]) -> bool {
    let [x, y, z] = p;
    Plane {
        point: *frame.origin(),
        normal: *frame.w(),
    }
    .signed_distance(&v(x, y, z))
    .to_f64()
    .abs()
        < 1e-9
}

/// What a selection fits follows from the shape of what is selected,
/// not only its kind.
#[test]
fn selections_fit_by_shape() {
    let part = drilled_box();
    let fits = |selection: Vec<EntityRef>| inspect_selection(&part, &selection).fits;

    let top = fits(vec![face("extrude(box,end)")]);
    assert!(top.contains(&"offset"), "{top:?}");
    assert!(!top.contains(&"axis_of"), "{top:?}");
    // The hole's wall is round, not flat.
    let wall = fits(vec![face("extrude(hole,hole_sketch,c1)")]);
    assert_eq!(wall, ["axis_of"]);
    // A straight edge is a line and an edge; a circular one has a
    // center and an axis.
    let straight = fits(vec![edge("extrude(box,outline,c4,end)")]);
    for method in ["along_line", "edge_point", "tangent", "normal_to_edge"] {
        assert!(straight.contains(&method), "{method}: {straight:?}");
    }
    assert!(!straight.contains(&"center"), "{straight:?}");
    let rim = fits(vec![edge("extrude(hole,hole_sketch,c1,start)")]);
    for method in ["center", "axis_of", "edge_point"] {
        assert!(rim.contains(&method), "{method}: {rim:?}");
    }
    assert!(!rim.contains(&"along_line"), "{rim:?}");
    // In any order.
    let point_plane = fits(vec![
        base(FrameAxis::Z),
        vertex("extrude(box,outline,p2,end)"),
    ]);
    for method in ["project_on_plane", "perpendicular", "parallel_plane"] {
        assert!(point_plane.contains(&method), "{method}: {point_plane:?}");
    }
    assert!(
        fits(vec![
            EntityRef::datum(ORIGIN),
            EntityRef::datum(ORIGIN),
            EntityRef::datum(ORIGIN)
        ])
        .contains(&"three_points")
    );
    assert!(fits(Vec::new()).is_empty());
    // Nothing fits what the part does not have.
    let missing = inspect_selection(&part, &[face("nowhere")]);
    assert!(missing.fits.is_empty());
    assert_eq!(missing.roles, [Vec::<Role>::new()]);
}

#[test]
fn points() {
    let part = drilled_box();
    let corner = vertex("extrude(box,outline,p2,end)");
    let d = datum(
        &part,
        vec![corner.clone()],
        Construction::Point {
            x: 0.5,
            y: 0.0,
            z: 1.0,
        },
    );
    assert_eq!(d.kind, DatumKind::Point);
    assert_at(&d.frame, [2.5, 2.0, 2.0], [0., 0., 1.]);
    // The top's front edge runs from (2, 0, 1) back to (0, 0, 1); a
    // point on it has its z axis along it.
    let on_edge = AddDatumArgs {
        selection: vec![edge("extrude(box,outline,c4,end)")],
        construction: Construction::EdgePoint { position: 0.25 },
    };
    let with_point = AddDatum.apply(part.clone(), "on_edge", &on_edge).unwrap();
    let d = with_point
        .datum(with_point.datum_id("on_edge").unwrap())
        .unwrap();
    assert_at(&d.frame, [1.5, 0.0, 1.0], [-1., 0., 0.]);
    // Offset from a datum point: along its own axes.
    let d = datum(
        &with_point,
        vec![EntityRef::datum("on_edge")],
        Construction::Point {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
    );
    assert_at(&d.frame, [1.0, 0.0, 1.0], [-1., 0., 0.]);

    let d = datum(
        &part,
        vec![EntityRef::datum(ORIGIN), corner.clone()],
        Construction::Midpoint {},
    );
    assert_at(&d.frame, [1.0, 1.0, 0.5], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::Center {},
    );
    assert_at(&d.frame, [1.0, 1.0, 1.0], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![corner.clone(), base(FrameAxis::Z)],
        Construction::ProjectOnPlane {},
    );
    assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![axis(FrameAxis::X), corner.clone()],
        Construction::ProjectOnLine {},
    );
    assert_at(&d.frame, [2.0, 0.0, 0.0], [1., 0., 0.]);
    let d = datum(
        &part,
        vec![edge("extrude(box,outline,p2)"), base(FrameAxis::Z)],
        Construction::LinePlane {},
    );
    assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![axis(FrameAxis::Z), edge("extrude(box,outline,c4,start)")],
        Construction::LineLine {},
    );
    assert_at(&d.frame, [0.0, 0.0, 0.0], [0., -1., 0.]);
    let d = datum(
        &part,
        vec![
            face("extrude(box,end)"),
            face("extrude(box,outline,c5)"),
            face("extrude(box,outline,c6)"),
        ],
        Construction::ThreePlanes {},
    );
    assert_at(&d.frame, [2.0, 2.0, 1.0], [0., 0., 1.]);
}

#[test]
fn axes() {
    let part = drilled_box();
    let corner = vertex("extrude(box,outline,p2,end)");
    let d = datum(
        &part,
        vec![EntityRef::datum(ORIGIN), corner.clone()],
        Construction::TwoPoints {},
    );
    assert_eq!(d.kind, DatumKind::Axis);
    assert_at(&d.frame, [0., 0., 0.], [2., 2., 1.]);
    let d = datum(
        &part,
        vec![edge("extrude(box,outline,p1)")],
        Construction::AlongLine {},
    );
    assert_at(&d.frame, [2., 0., 0.], [0., 0., 1.]);
    // The hole's wall turns around the vertical through its center.
    let d = datum(
        &part,
        vec![face("extrude(hole,hole_sketch,c1#2)")],
        Construction::AxisOf {},
    );
    assert!(datum_line(&d).could_contain(&v(1., 1., 7.)));
    let d = datum(
        &part,
        vec![base(FrameAxis::X), base(FrameAxis::Y)],
        Construction::PlanePlane {},
    );
    assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
    // The perpendicular dropped from the corner onto the xy plane.
    let d = datum(
        &part,
        vec![corner.clone(), base(FrameAxis::Z)],
        Construction::Perpendicular {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![corner.clone(), axis(FrameAxis::X)],
        Construction::Parallel {},
    );
    assert_at(&d.frame, [2., 2., 1.], [1., 0., 0.]);
    let d = datum(
        &part,
        vec![corner.clone(), axis(FrameAxis::X)],
        Construction::PerpendicularToLine {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., -2., -1.]);
    let d = datum(
        &part,
        vec![axis(FrameAxis::X), axis(FrameAxis::Y)],
        Construction::Bisector { other: false },
    );
    assert_at(&d.frame, [0., 0., 0.], [1., 1., 0.]);
    let d = datum(
        &part,
        vec![axis(FrameAxis::X), axis(FrameAxis::Y)],
        Construction::Bisector { other: true },
    );
    assert_at(&d.frame, [0., 0., 0.], [1., -1., 0.]);
    // Between two parallel edges: halfway.
    let d = datum(
        &part,
        vec![
            edge("extrude(box,outline,p0)"),
            edge("extrude(box,outline,p2)"),
        ],
        Construction::Bisector { other: false },
    );
    let off = datum_line(&d).project(&v(1., 1., 0.)).sub(&v(1., 1., 0.));
    assert!(off.norm().to_f64() < 1e-9, "{off:?}");
    let d = datum(
        &part,
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::Tangent { position: 0.0 },
    );
    assert_at(&d.frame, [1.4, 1.0, 1.0], [0., 1., 0.]);
}

fn datum_line(d: &Datum<S>) -> geop_core_geometry::shape::Axis<S> {
    geop_core_geometry::shape::Axis::try_new(*d.frame.origin(), *d.frame.w()).unwrap()
}

#[test]
fn planes() {
    let part = drilled_box();
    let top = face("extrude(box,end)");
    let d = datum(
        &part,
        vec![top.clone()],
        Construction::Offset { distance: 0.5 },
    );
    assert_eq!(d.kind, DatumKind::Plane);
    assert_at(&d.frame, [0., 0., 1.5], [0., 0., 1.]);
    // Between the box's top and bottom, which face apart.
    let d = datum(
        &part,
        vec![top.clone(), face("extrude(box,start)")],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [1., 1., 0.5]));
    assert_at(&d.frame, [0., 0., 0.5], [0., 0., 1.]);
    // Between two planes facing the same way.
    let lifted = AddDatum
        .apply(
            part.clone(),
            "lifted",
            &AddDatumArgs {
                selection: vec![top.clone()],
                construction: Construction::Offset { distance: 1.0 },
            },
        )
        .unwrap();
    let d = datum(
        &lifted,
        vec![top.clone(), EntityRef::datum("lifted")],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [5., 5., 1.5]));
    // Halving the angle between two sides that meet at an edge.
    let d = datum(
        &part,
        vec![
            face("extrude(box,outline,c4)"),
            face("extrude(box,outline,c5)"),
        ],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [2., 0., 0.]) && on_plane(&d.frame, [1., 1., 0.]));
    let d = datum(
        &part,
        vec![
            EntityRef::datum(ORIGIN),
            vertex("extrude(box,outline,p1,start)"),
            vertex("extrude(box,outline,p2,end)"),
        ],
        Construction::ThreePoints {},
    );
    assert!(
        on_plane(&d.frame, [0., 0., 0.])
            && on_plane(&d.frame, [2., 0., 0.])
            && on_plane(&d.frame, [2., 2., 1.])
    );
    // Hinged on the top's front edge, turned up by 90 degrees: the
    // front face's plane.
    let d = datum(
        &part,
        vec![top.clone(), edge("extrude(box,outline,c4,end)")],
        Construction::Angle { angle: 90.0 },
    );
    assert!(on_plane(&d.frame, [0.5, 0., 0.]) && on_plane(&d.frame, [0.5, 0., 7.]));
    let d = datum(
        &part,
        vec![axis(FrameAxis::Z), vertex("extrude(box,outline,p1,start)")],
        Construction::LinePoint {},
    );
    assert!(on_plane(&d.frame, [5., 0., 3.]));
    let d = datum(
        &part,
        vec![
            edge("extrude(box,outline,p0)"),
            edge("extrude(box,outline,p2)"),
        ],
        Construction::TwoLines {},
    );
    assert!(on_plane(&d.frame, [1., 1., 3.]));
    let d = datum(
        &part,
        vec![top.clone(), EntityRef::datum(ORIGIN)],
        Construction::ParallelPlane {},
    );
    assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
    let d = datum(
        &part,
        vec![axis(FrameAxis::Y), vertex("extrude(box,outline,p2,end)")],
        Construction::NormalToLine {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., 1., 0.]);
    let d = datum(
        &part,
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::NormalToEdge { position: 0.0 },
    );
    assert!(on_plane(&d.frame, [1.4, 1., 1.]) && on_plane(&d.frame, [1., 1., 1.]));
}

/// A datum is a named entity, and a sketch can be placed on a datum
/// plane: its frame, exactly.
#[test]
fn sketches_go_on_datum_planes() {
    let program = examples::boss_on_reference_plane();
    let part = program.build::<S>().unwrap();
    part.check_names().unwrap();
    let placed = part.sketch(part.sketch_id("boss_sketch").unwrap()).unwrap();
    let lifted = part.datum(part.datum_id("lifted").unwrap()).unwrap();
    assert!(placed.plane.origin().could_be_equal(lifted.frame.origin()));
    assert!(placed.plane.u().could_be_equal(lifted.frame.u()));
    assert_at(&placed.plane, [0., 0., 1.5], [0., 0., 1.]);
}

/// A coordinate system through three points: at the first, x towards the
/// second, xy through the third. It is used as a point — its origin — and
/// by its axes, like the origin itself.
#[test]
fn frames() {
    let part = drilled_box();
    let args = AddDatumArgs {
        selection: vec![
            vertex("extrude(box,outline,p1,end)"),
            vertex("extrude(box,outline,p2,end)"),
            vertex("extrude(box,outline,p0,start)"),
        ],
        construction: Construction::FrameThreePoints {},
    };
    let part = AddDatum.apply(part, "cs", &args).unwrap();
    let d = part.datum(part.datum_id("cs").unwrap()).unwrap();
    assert_eq!(d.kind, DatumKind::Frame);
    // x from (2, 0, 1) to (2, 2, 1); the third point (0, 0, 0) tilts the
    // xy plane down towards -x.
    assert_at(&d.frame, [2., 0., 1.], [-1., 0., 2.]);
    assert!(d.frame.u().sub(&v(0., 1., 0.)).norm().to_f64() < 1e-9);

    let fit = inspect_selection(&part, &[EntityRef::datum("cs")]);
    assert_eq!(fit.roles, [vec![Role::Point]]);
    // Its axes carry an offset point, as a datum point's do.
    let d = datum(
        &part,
        vec![EntityRef::datum("cs")],
        Construction::Point {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
    );
    assert_at(&d.frame, [2., 1., 1.], [-1., 0., 2.]);

    // The first two points coincide: there is no x to go by.
    let args = AddDatumArgs {
        selection: vec![
            EntityRef::datum(ORIGIN),
            EntityRef::datum(ORIGIN),
            vertex("extrude(box,outline,p2,end)"),
        ],
        ..args
    };
    let Err(e) = AddDatum.apply(part, "degenerate", &args) else {
        panic!("a frame with no x axis");
    };
    assert!(e.to_string().contains("one line"), "{e}");
}
