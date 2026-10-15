//! Datums built on a solid, and sketched on.

use crate::{AddDatumArgs, Construction, Role, Selected, Shape, fitting_constructions};
use geop_core_geometry::shape::Plane;
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::element::ElementReference;
use geop_ops_datums::{AnyDatum, Datums, ORIGIN, component_name, datums, read_datums};
use geop_ops_extrude_revolve::shapes::v3;
use geop_ops_model::model;
use geop_ops_parameters::{
    BuildPart, PartReader, ProgramFile, Registry, Runner, Settings, StateValues,
};
use geop_ops_sketch::{PlacedSketch, sketch_element, whole_sketch};

use geop_ops_extrude_revolve::testing::{drilled_box as drilled, runner_of};

use crate::AddDatum;

fn face(name: &str) -> Selected {
    Selected::Face(ElementReference::new(model(), name))
}
fn edge(name: &str) -> Selected {
    Selected::Edge(ElementReference::new(model(), name))
}
fn vertex(name: &str) -> Selected {
    Selected::Vertex(ElementReference::new(model(), name))
}
fn datum(name: impl Into<String>) -> Selected {
    Selected::Datum(ElementReference::<Datums, AnyDatum>::new(datums(), name))
}
fn base(normal: FrameAxis) -> Selected {
    datum(component_name(ORIGIN, DatumComponent::Plane(normal)))
}
fn axis(along: FrameAxis) -> Selected {
    datum(component_name(ORIGIN, DatumComponent::Axis(along)))
}

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<Registry> {
    geop_ops_extrude_revolve::testing::registry()?.with(AddDatum)
}

/// A program, the runner that built it and the part it built: a datum
/// step added to it is built on what the part kept.
struct Part {
    program: ProgramFile,
    runner: Runner,
    part: BuildPart,
}

impl Part {
    /// `program`, built.
    fn of(program: ProgramFile) -> Self {
        let runner = runner_of(registry().unwrap());
        let mut part = BuildPart::default();
        (program.run(&runner, &mut part, &Settings::new(), &StateValues::new())).unwrap();
        Part {
            program,
            runner,
            part,
        }
    }

    /// The program with the datum step `id` from `selection` by
    /// `construction` added, built — or why it fails.
    fn try_with(
        &mut self,
        id: &str,
        selection: Vec<Selected>,
        construction: Construction,
    ) -> GeopResult<ProgramFile> {
        let mut program = self.program.clone();
        let args = AddDatumArgs {
            selection,
            construction,
        };
        program.push(id, args);
        program.run(&self.runner, &mut self.part, &Settings::new(), &StateValues::new())?;
        Ok(program)
    }

    /// The datum `id`, as the program last run built it.
    fn datum_named(&self, id: &str) -> Datum<S> {
        read_datums(&self.part.current()).unwrap().get(id).unwrap()
    }

    /// The datum built from `selection` by `construction`, or why it fails.
    fn try_datum(
        &mut self,
        selection: Vec<Selected>,
        construction: Construction,
    ) -> GeopResult<Datum<S>> {
        self.try_with("d", selection, construction)?;
        Ok(self.datum_named("d"))
    }

    /// The datum built from `selection` by `construction`.
    fn datum(&mut self, selection: Vec<Selected>, construction: Construction) -> Datum<S> {
        self.try_datum(selection, construction).unwrap()
    }

    /// It with the datum step `id` added for good.
    fn with(mut self, id: &str, selection: Vec<Selected>, construction: Construction) -> Self {
        self.program = self.try_with(id, selection, construction).unwrap();
        self
    }

    /// The part as the program — without a datum step tried since — has it.
    fn seen(&mut self) -> PartReader<'_> {
        let (runner, part) = (&self.runner, &mut self.part);
        (self.program.run(runner, part, &Settings::new(), &StateValues::new())).unwrap();
        self.part.current()
    }

    /// The constructions that fit `selection`.
    fn fits(&mut self, selection: &[Selected]) -> Vec<&'static str> {
        fitting_constructions(&self.seen(), selection)
    }

    /// What `entity` can be used as.
    fn aspects(&mut self, entity: &Selected) -> GeopResult<Shape> {
        entity.shape(&self.seen())
    }
}

/// The 2 x 2 x 1 box with a hole of radius 0.4 drilled 0.5 deep into
/// the middle of its top.
fn drilled_box() -> Part {
    Part::of(drilled())
}

/// Sketch data is solved, not exact: agreeing to 1e-9 is agreeing.
fn assert_at(frame: &CoordinateSystem<S>, origin: [f64; 3], w: [f64; 3]) {
    let [x, y, z] = origin;
    let off = frame.origin().sub(&v3(x, y, z)).norm().to_f64();
    assert!(off < 1e-9, "origin of {frame} is {off} off {origin:?}");
    let [x, y, z] = w;
    let w = v3(x, y, z).normalize().unwrap();
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
    .signed_distance(&v3(x, y, z))
    .to_f64()
    .abs()
        < 1e-9
}

/// What a selection fits follows from the shape of what is selected,
/// not only its kind.
#[test]
fn selections_fit_by_shape() {
    let mut part = drilled_box();
    let mut fits = |selection: Vec<Selected>| part.fits(&selection);

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
            datum(ORIGIN),
            datum(ORIGIN),
            datum(ORIGIN)
        ])
        .contains(&"three_points")
    );
    assert!(fits(Vec::new()).is_empty());
    // Nothing fits what the part does not have.
    assert!(fits(vec![face("nowhere")]).is_empty());
    assert!(part.aspects(&face("nowhere")).is_err());
}

/// A sketch's points and lines are points and lines to build on: the
/// drilled box's outline — a 2 x 2 square on the xy plane — gives a line
/// through two of its corners.
#[test]
fn sketch_points_and_lines() {
    let mut part = drilled_box();
    let placed: PlacedSketch<S> = part.seen().retrieve_element(&whole_sketch("outline")).unwrap().clone();
    let corner = |x: f64, y: f64| {
        let (&point, _) = placed
            .sketch
            .points
            .iter()
            .find(|(_, p)| (p.x.to_f64() - x).abs() < 1e-9 && (p.y.to_f64() - y).abs() < 1e-9)
            .unwrap();
        Selected::SketchPoint(sketch_element("outline", point))
    };
    let d = part.datum(
        vec![corner(0.0, 0.0), corner(2.0, 2.0)],
        Construction::TwoPoints {},
    );
    assert_at(&d.frame, [0.0, 0.0, 0.0], [1.0, 1.0, 0.0]);
    let (&line, _) = placed.sketch.curves.iter().next().unwrap();
    let roles = (part.aspects(&Selected::SketchCurve(sketch_element("outline", line))))
        .unwrap()
        .roles();
    assert_eq!(roles, [Role::Line, Role::Curve]);
}

#[test]
fn points() {
    let mut part = drilled_box();
    let corner = vertex("extrude(box,outline,p2,end)");
    let d = part.datum(
        vec![corner.clone()],
        Construction::Point {
            x: 0.5.into(),
            y: 0.0.into(),
            z: 1.0.into(),
        },
    );
    assert_eq!(d.kind, DatumKind::Point);
    assert_at(&d.frame, [2.5, 2.0, 2.0], [0., 0., 1.]);
    // The top's front edge runs from (2, 0, 1) back to (0, 0, 1); a
    // point on it has its z axis along it.
    let mut with_point = drilled_box().with(
        "on_edge",
        vec![edge("extrude(box,outline,c4,end)")],
        Construction::EdgePoint {
            position: 0.25.into(),
        },
    );
    let d = with_point.datum_named("on_edge");
    assert_at(&d.frame, [1.5, 0.0, 1.0], [-1., 0., 0.]);
    // Offset from a datum point: along its own axes.
    let d = with_point.datum(
        vec![datum("on_edge")],
        Construction::Point {
            x: 0.0.into(),
            y: 0.0.into(),
            z: 0.5.into(),
        },
    );
    assert_at(&d.frame, [1.0, 0.0, 1.0], [-1., 0., 0.]);

    let d = part.datum(
        vec![datum(ORIGIN), corner.clone()],
        Construction::Midpoint {},
    );
    assert_at(&d.frame, [1.0, 1.0, 0.5], [0., 0., 1.]);
    let d = part.datum(
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::Center {},
    );
    assert_at(&d.frame, [1.0, 1.0, 1.0], [0., 0., 1.]);
    let d = part.datum(
        vec![corner.clone(), base(FrameAxis::Z)],
        Construction::ProjectOnPlane {},
    );
    assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
    let d = part.datum(
        vec![axis(FrameAxis::X), corner.clone()],
        Construction::ProjectOnLine {},
    );
    assert_at(&d.frame, [2.0, 0.0, 0.0], [1., 0., 0.]);
    let d = part.datum(
        vec![
            edge("extrude(box,outline,p2)"),
            base(FrameAxis::Z),
        ],
        Construction::LinePlane {},
    );
    assert_at(&d.frame, [2.0, 2.0, 0.0], [0., 0., 1.]);
    let d = part.datum(
        vec![
            axis(FrameAxis::Z),
            edge("extrude(box,outline,c4,start)"),
        ],
        Construction::LineLine {},
    );
    assert_at(&d.frame, [0.0, 0.0, 0.0], [0., -1., 0.]);
    let d = part.datum(
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
    let mut part = drilled_box();
    let corner = vertex("extrude(box,outline,p2,end)");
    let d = part.datum(
        vec![datum(ORIGIN), corner.clone()],
        Construction::TwoPoints {},
    );
    assert_eq!(d.kind, DatumKind::Axis);
    assert_at(&d.frame, [0., 0., 0.], [2., 2., 1.]);
    let d = part.datum(
        vec![edge("extrude(box,outline,p1)")],
        Construction::AlongLine {},
    );
    assert_at(&d.frame, [2., 0., 0.], [0., 0., 1.]);
    // The hole's wall turns around the vertical through its center.
    let d = part.datum(
        vec![face("extrude(hole,hole_sketch,c1#2)")],
        Construction::AxisOf {},
    );
    assert!(datum_line(&d).could_contain(&v3(1., 1., 7.)));
    let d = part.datum(
        vec![base(FrameAxis::X), base(FrameAxis::Y)],
        Construction::PlanePlane {},
    );
    assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
    // The perpendicular dropped from the corner onto the xy plane.
    let d = part.datum(
        vec![corner.clone(), base(FrameAxis::Z)],
        Construction::Perpendicular {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., 0., 1.]);
    let d = part.datum(
        vec![corner.clone(), axis(FrameAxis::X)],
        Construction::Parallel {},
    );
    assert_at(&d.frame, [2., 2., 1.], [1., 0., 0.]);
    let d = part.datum(
        vec![corner.clone(), axis(FrameAxis::X)],
        Construction::PerpendicularToLine {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., -2., -1.]);
    let d = part.datum(
        vec![axis(FrameAxis::X), axis(FrameAxis::Y)],
        Construction::Bisector { other: false },
    );
    assert_at(&d.frame, [0., 0., 0.], [1., 1., 0.]);
    let d = part.datum(
        vec![axis(FrameAxis::X), axis(FrameAxis::Y)],
        Construction::Bisector { other: true },
    );
    assert_at(&d.frame, [0., 0., 0.], [1., -1., 0.]);
    // Between two parallel edges: halfway.
    let d = part.datum(
        vec![
            edge("extrude(box,outline,p0)"),
            edge("extrude(box,outline,p2)"),
        ],
        Construction::Bisector { other: false },
    );
    let off = datum_line(&d).project(&v3(1., 1., 0.)).sub(&v3(1., 1., 0.));
    assert!(off.norm().to_f64() < 1e-9, "{off:?}");
    let d = part.datum(
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::Tangent {
            position: 0.0.into(),
        },
    );
    assert_at(&d.frame, [1.4, 1.0, 1.0], [0., 1., 0.]);
}

fn datum_line(d: &Datum<S>) -> geop_core_geometry::shape::Axis<S> {
    geop_core_geometry::shape::Axis::try_new(*d.frame.origin(), *d.frame.w()).unwrap()
}

#[test]
fn planes() {
    let mut part = drilled_box();
    let top = face("extrude(box,end)");
    let d = part.datum(
        vec![top.clone()],
        Construction::Offset {
            distance: 0.5.into(),
        },
    );
    assert_eq!(d.kind, DatumKind::Plane);
    assert_at(&d.frame, [0., 0., 1.5], [0., 0., 1.]);
    // Between the box's top and bottom, which face apart.
    let d = part.datum(
        vec![top.clone(), face("extrude(box,start)")],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [1., 1., 0.5]));
    assert_at(&d.frame, [0., 0., 0.5], [0., 0., 1.]);
    // Between two planes facing the same way.
    let mut lifted = drilled_box().with(
        "lifted",
        vec![top.clone()],
        Construction::Offset {
            distance: 1.0.into(),
        },
    );
    let d = lifted.datum(
        vec![top.clone(), datum("lifted")],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [5., 5., 1.5]));
    // Halving the angle between two sides that meet at an edge.
    let d = part.datum(
        vec![
            face("extrude(box,outline,c4)"),
            face("extrude(box,outline,c5)"),
        ],
        Construction::Midplane { other: false },
    );
    assert!(on_plane(&d.frame, [2., 0., 0.]) && on_plane(&d.frame, [1., 1., 0.]));
    let d = part.datum(
        vec![
            datum(ORIGIN),
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
    let d = part.datum(
        vec![top.clone(), edge("extrude(box,outline,c4,end)")],
        Construction::Angle { angle: 90.0.into() },
    );
    assert!(on_plane(&d.frame, [0.5, 0., 0.]) && on_plane(&d.frame, [0.5, 0., 7.]));
    let d = part.datum(
        vec![
            axis(FrameAxis::Z),
            vertex("extrude(box,outline,p1,start)"),
        ],
        Construction::LinePoint {},
    );
    assert!(on_plane(&d.frame, [5., 0., 3.]));
    let d = part.datum(
        vec![
            edge("extrude(box,outline,p0)"),
            edge("extrude(box,outline,p2)"),
        ],
        Construction::TwoLines {},
    );
    assert!(on_plane(&d.frame, [1., 1., 3.]));
    let d = part.datum(
        vec![top.clone(), datum(ORIGIN)],
        Construction::ParallelPlane {},
    );
    assert_at(&d.frame, [0., 0., 0.], [0., 0., 1.]);
    let d = part.datum(
        vec![
            axis(FrameAxis::Y),
            vertex("extrude(box,outline,p2,end)"),
        ],
        Construction::NormalToLine {},
    );
    assert_at(&d.frame, [2., 2., 1.], [0., 1., 0.]);
    let d = part.datum(
        vec![edge("extrude(hole,hole_sketch,c1,start)")],
        Construction::NormalToEdge {
            position: 0.0.into(),
        },
    );
    assert!(on_plane(&d.frame, [1.4, 1., 1.]) && on_plane(&d.frame, [1., 1., 1.]));
}

/// A coordinate system through three points: at the first, x towards the
/// second, xy through the third. It is used as a point — its origin — and
/// by its axes, like the origin itself.
#[test]
fn frames() {
    let part = drilled_box();
    let mut part = part.with(
        "cs",
        vec![
            vertex("extrude(box,outline,p1,end)"),
            vertex("extrude(box,outline,p2,end)"),
            vertex("extrude(box,outline,p0,start)"),
        ],
        Construction::FrameThreePoints {},
    );
    let d = part.datum_named("cs");
    assert_eq!(d.kind, DatumKind::Frame);
    // x from (2, 0, 1) to (2, 2, 1); the third point (0, 0, 0) tilts the
    // xy plane down towards -x.
    assert_at(&d.frame, [2., 0., 1.], [-1., 0., 2.]);
    assert!(d.frame.u().sub(&v3(0., 1., 0.)).norm().to_f64() < 1e-9);

    let shape = part.aspects(&datum("cs")).unwrap();
    assert_eq!(shape.roles(), [Role::Point]);
    assert!(shape.frame.is_some());
    // Its axes carry an offset point, as a datum point's do.
    let d = part.datum(
        vec![datum("cs")],
        Construction::Point {
            x: 1.0.into(),
            y: 0.0.into(),
            z: 0.0.into(),
        },
    );
    assert_at(&d.frame, [2., 1., 1.], [-1., 0., 2.]);

    // The first two points coincide: there is no x to go by.
    let selection = vec![
        datum(ORIGIN),
        datum(ORIGIN),
        vertex("extrude(box,outline,p2,end)"),
    ];
    let Err(e) = part.try_datum(selection, Construction::FrameThreePoints {}) else {
        panic!("a frame with no x axis");
    };
    assert!(e.to_string().contains("one line"), "{e}");
}
