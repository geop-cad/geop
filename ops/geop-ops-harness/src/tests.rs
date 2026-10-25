//! Routes laid between connectors and through circular edges: their
//! lengths against the analytical ones, their bends checked, their bundles
//! valid solids.
//!
//! A connector here is the frame on a short straight edge of a wire, at
//! its middle, its `z` along the edge — a free point a vertex of a wire on
//! its own. The points are resolved in a part's model, in the design
//! scalar, and the route through them laid in a model of its own, in each
//! scalar.

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    for_all_scalars,
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, values::Design};
use geop_core_topology::{
    build::{BodySpec, EdgeSpec},
    validation::{ValidationParameters, validate},
};
use geop_lib_sweep::shapes::{revolved_cylinder, v3};
use geop_ops_model::datums::{ORIGIN, component_name, datums};
use geop_ops_model::{
    BodyNames, NamedModel, model,
    testing::{assert_valid, given_part},
};
use geop_ops_reference_geometry::Selected;

use crate::{
    Cable, RouteArgs, Wire, WireSize,
    operation::lay,
    route::{Heading, RoutePath, Waypoint},
};

/// A part's model, whose points a route runs through.
type Part = NamedModel<Design>;

/// A part with a connector — a frame the cable leaves along its `z` axis
/// — named `name` at `at` for every `(name, at, z)`: the edge `name` of a
/// wire from `at - z` to `at + z`, whose frame sits at its middle.
fn connectors(part: &mut Part, frames: &[(&str, Vector3<Design>, Vector3<Design>)]) {
    for (name, at, z) in frames {
        let (start, end) = (at.sub(z), at.add(z));
        let spec = BodySpec {
            vertices: vec![start, end],
            edges: vec![EdgeSpec {
                curve: NurbCurve3D::line(start, end).unwrap(),
                start: 0,
                end: 1,
            }],
            faces: Vec::new(),
            shells: Vec::new(),
            solid: false,
        };
        let names = BodyNames {
            vertices: vec![format!("{name}.start"), format!("{name}.end")],
            edges: vec![name.to_string()],
            faces: Vec::new(),
            solid: None,
        };
        part.build_body(spec, names).unwrap();
    }
}

/// A free point named `name` at `at`: a wire of the vertex alone.
fn point(part: &mut Part, name: &str, at: Vector3<Design>) {
    let spec = BodySpec {
        vertices: vec![at],
        edges: Vec::new(),
        faces: Vec::new(),
        shells: Vec::new(),
        solid: false,
    };
    let names = BodyNames {
        vertices: vec![name.to_string()],
        edges: Vec::new(),
        faces: Vec::new(),
        solid: None,
    };
    part.build_body(spec, names).unwrap();
}

/// The connector `name`: the frame on its edge.
fn connector(name: &str) -> Selected {
    Selected::Edge(ElementReference::new(model(), name))
}

/// The edge `name`, a circular one: what a route passes along its axis.
fn rim(name: &str) -> Selected {
    Selected::Edge(ElementReference::new(model(), name))
}

/// The vertex `name`: a free point.
fn vertex(name: &str) -> Selected {
    Selected::Vertex(ElementReference::new(model(), name))
}

/// One wire `d` across: a bundle `d / sqrt(fill)` across.
fn args(through: Vec<Selected>, d: f64, bend_factor: f64) -> RouteArgs {
    RouteArgs {
        through,
        wires: vec![Wire {
            size: WireSize::Diameter(d),
            ..Wire::new("w1")
        }],
        fill: 1.0,
        bend_factor,
        service_loop: 0.0,
    }
}

/// The points `through` in `part`, as a part of the model and the datums
/// resolves them, in `S`.
fn waypoints<S: Scalar>(part: &Part, through: &[Selected]) -> GeopResult<Vec<Waypoint<S>>> {
    let built = given_part(part)?;
    let reader = built.current();
    let cast = |v: Vector3<Design>| v.map(|c| c.cast::<S>());
    (through.iter().enumerate())
        .map(|(i, entity)| {
            let waypoint = Waypoint::of(&reader, entity, i)?;
            Ok(Waypoint {
                point: cast(waypoint.point),
                heading: match waypoint.heading {
                    Heading::Free => Heading::Free,
                    Heading::Axis(d) => Heading::Axis(cast(d)),
                    Heading::Exit(d) => Heading::Exit(cast(d)),
                },
                label: waypoint.label,
            })
        })
        .collect()
}

/// The route `r` of `args` through its points in `part`, laid in a model
/// of its own, and its cable.
fn route<S: Scalar>(part: &Part, args: &RouteArgs) -> GeopResult<(NamedModel<S>, Cable<S>)> {
    let waypoints = waypoints(part, &args.through)?;
    let mut laid = NamedModel::new();
    let cable = lay(&mut laid, "r", args, &waypoints)?;
    Ok((laid, cable))
}

/// Why routing `args` in `part` is refused.
#[track_caller]
fn refusal<S: Scalar>(part: &Part, args: &RouteArgs) -> String {
    match route::<S>(part, args) {
        Ok(_) => panic!("{args:?} was routed"),
        Err(e) => e.root_message().to_string(),
    }
}

/// Between two connectors facing each other, the route runs straight: as
/// long as they are apart, every wire cut to that and a service loop at
/// each end, the bundle a straight cylinder.
fn check_straight_route_between_facing_connectors<S: Scalar>() {
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("end", v3(120.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0)),
        ],
    );
    let mut args = args(vec![connector("start"), connector("end")], 2.0, 5.0);
    args.service_loop = 15.0;
    args.wires.push(Wire {
        size: WireSize::Awg {
            gauge: 20,
            insulation: 0.3,
        },
        colour: "#ff0000".into(),
        ..Wire::new("w2")
    });
    let (part, cable) = route::<S>(&part, &args).unwrap();
    assert_valid(&part);
    assert_eq!(
        part.topology().faces.len(),
        4 + 2,
        "a cylinder of four quarters, capped"
    );
    assert!(
        cable.length.could_be_equal(S::from_f64(120.0)),
        "{:?}",
        cable.length
    );
    assert!(cable.length.width().to_f64() < 1e-6, "{:?}", cable.length);
    assert!(cable.tightest_bend.is_none());
    assert_eq!(cable.wires.len(), 2);
    for wire in &cable.wires {
        assert!(
            wire.cut_length.could_be_equal(S::from_f64(150.0)),
            "{wire:?}"
        );
    }
    assert_eq!(cable.wires[1].gauge, Some(20));
    let d2 = cable.wires[1].diameter;
    assert!((cable.diameter - (4.0 + d2 * d2).sqrt()).abs() < 1e-12);
}
#[test]
fn straight_route_between_facing_connectors() {
    for_all_scalars!(check_straight_route_between_facing_connectors);
}

/// From a connector heading `x` to one a quarter turn round, heading `y`,
/// at `(R, R)`: the route is the quarter circle of radius `R` — two eighths,
/// meeting halfway — `pi R / 2` long, bending with radius `R`.
fn check_quarter_turn_is_a_quarter_circle<S: Scalar>() {
    let r = 40.0;
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("end", v3(r, r, 0.0), v3(0.0, -1.0, 0.0)),
        ],
    );
    let (part, cable) = route::<S>(
        &part,
        &args(vec![connector("start"), connector("end")], 2.0, 5.0),
    )
    .unwrap();
    assert_valid(&part);
    let quarter = S::PI.mul(S::from_f64(r / 2.0));
    assert!(cable.length.could_be_equal(quarter), "{:?}", cable.length);
    let scalar = std::any::type_name::<S>();
    // Fixed point at this scale holds the arcs' centres to a few millionths.
    // The connectors' directions are their edges', normalized, and so not
    // exact: the length comes out up to two ten-thousandths wide.
    assert!(
        cable.length.width().to_f64() < 5e-4,
        "{scalar}: {:?}",
        cable.length
    );
    let tightest = cable.tightest_bend.unwrap();
    assert!(tightest.could_be_equal(S::from_f64(r)), "{tightest:?}");
    // Halfway round, the bundle's centre passes `R (1 - 1/sqrt 2)` off
    // either axis: its section there is centred on the circle.
    let half = r * (1.0 - std::f64::consts::FRAC_1_SQRT_2);
    let centre = v3::<S>(r - half, half, 0.0);
    let model = part.topology();
    for p in 0..4 {
        let vertex = part.vertex_id(&format!("route(r,p{p},m0)")).unwrap();
        let at = model.get_vertex(vertex).unwrap().point;
        let off = at.sub(&centre).norm();
        assert!(
            off.could_be_equal(S::ONE),
            "{at:?} is {off:?} off the centre"
        );
    }
}
#[test]
fn quarter_turn_is_a_quarter_circle() {
    for_all_scalars!(check_quarter_turn_is_a_quarter_circle);
}

/// The same quarter turn, but too tight for the bundle: refused, saying
/// between which points it bends, with what radius, and what is allowed.
fn check_too_tight_a_bend_is_refused<S: Scalar>() {
    let r = 8.0;
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("end", v3(r, r, 0.0), v3(0.0, -1.0, 0.0)),
        ],
    );
    // A bundle 2 across, bent no tighter than 5 diameters: radius 10.
    let message = refusal::<S>(
        &part,
        &args(vec![connector("start"), connector("end")], 2.0, 5.0),
    );
    assert!(
        message.contains("may bend no tighter than radius 10.000 (5 × its diameter)"),
        "{message}"
    );
    assert!(
        message.contains("with radius 8.000 between point 1 (edge start) and point 2 (edge end)"),
        "{message}"
    );
}
#[test]
fn too_tight_a_bend_is_refused() {
    for_all_scalars!(check_too_tight_a_bend_is_refused);
}

/// A bend exactly as tight as allowed is no violation.
fn check_a_bend_at_the_limit_is_allowed<S: Scalar>() {
    let r = 10.0;
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("end", v3(r, r, 0.0), v3(0.0, -1.0, 0.0)),
        ],
    );
    let (part, _) = route::<S>(
        &part,
        &args(vec![connector("start"), connector("end")], 2.0, 5.0),
    )
    .unwrap();
    assert_valid(&part);
}
#[test]
fn a_bend_at_the_limit_is_allowed() {
    for_all_scalars!(check_a_bend_at_the_limit_is_allowed);
}

/// A clip: a cylinder named `name` along `z`, of radius 3 and 10 long,
/// standing on `base` — and the name of its circular edge round `base`.
fn clip(part: &mut Part, name: &str, base: Vector3<Design>) -> String {
    let (radius, height) = (Design::from_f64(3.0), Design::from_f64(10.0));
    revolved_cylinder(part, name, base, radius, height).unwrap();
    let model = part.topology();
    model
        .edges
        .iter()
        .find_map(|(&id, edge)| {
            let arc = edge.curve.as_arc().ok()??;
            arc.circle
                .center
                .could_be_equal(&base)
                .then(|| part.name_of(id).unwrap().to_string())
        })
        .expect("the cylinder has a circular edge round its base")
}

/// Whether `value` encloses `expected`, a value worked out in `f64` — up
/// to that working's own rounding.
fn encloses<S: Scalar>(value: S, expected: f64) -> bool {
    (value.to_f64() - expected).abs() <= value.width().to_f64() / 2.0 + 1e-12 * expected.abs()
}

/// Through a clip: the route passes through the centre of its circular
/// edge, along its axis — here off the line between the connectors, so it
/// swerves in two S-bends to get there.
fn check_route_through_a_clip<S: Scalar>() {
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, -55.0), v3(0.0, 0.0, 1.0)),
            ("end", v3(0.0, 0.0, 45.0), v3(0.0, 0.0, -1.0)),
        ],
    );
    let rim = clip(&mut part, "clip", v3(10.0, 0.0, 0.0));
    let through = vec![connector("start"), self::rim(&rim), connector("end")];
    let (part, cable) = route::<S>(&part, &args(through, 2.0, 5.0)).unwrap();
    assert_valid(&part);
    let model = part.topology();
    // The bundle's section at the clip is centred on the rim's centre,
    // square to its axis.
    for p in 0..4 {
        let vertex = part.vertex_id(&format!("route(r,p{p},w1)")).unwrap();
        let at = model.get_vertex(vertex).unwrap().point;
        assert!(at[2].could_be_equal(S::ZERO), "{at:?}");
        let off = at.sub(&v3(10.0, 0.0, 0.0)).norm();
        assert!(off.could_be_equal(S::ONE), "{at:?}");
    }
    // Each half swerves 10 sideways over 55 and 45, along z at both ends:
    // two equal arcs turning by `2 atan(10 / L)` each, of radius
    // `L / (2 sin)` — the shorter half bends tighter.
    let turn = |l: f64| 2.0 * (10.0 / l).atan();
    let radius = |l: f64| l / (2.0 * turn(l).sin());
    let tightest = cable.tightest_bend.unwrap();
    assert!(
        encloses(tightest, radius(45.0)),
        "{tightest:?} vs {}",
        radius(45.0)
    );
    let expected = [55.0, 45.0].map(|l| 2.0 * turn(l) * radius(l)).iter().sum();
    assert!(
        encloses(cable.length, expected),
        "{:?} vs {expected}",
        cable.length
    );
}
#[test]
fn route_through_a_clip() {
    for_all_scalars!(check_route_through_a_clip);
}

/// Datum points and vertices leave the direction free: between two of
/// them the route is straight, and a free middle point is passed turning
/// as much before it as after.
fn check_free_points_choose_their_directions<S: Scalar>() {
    let mut part = Part::new();
    for (name, at) in [
        ("a", v3(0.0, 0.0, 0.0)),
        ("b", v3(50.0, 0.0, 0.0)),
        ("c", v3(100.0, 30.0, 0.0)),
    ] {
        point(&mut part, name, at);
    }
    let (_, cable) = route::<S>(&part, &args(vec![vertex("a"), vertex("b")], 2.0, 5.0)).unwrap();
    assert!(cable.length.could_be_equal(S::from_f64(50.0)));

    let through = vec![vertex("a"), vertex("b"), vertex("c")];
    let (bent, _) = route::<S>(&part, &args(through.clone(), 2.0, 5.0)).unwrap();
    assert_valid(&bent);
    // The middle point is passed along the bisector of the chords to its
    // neighbours; each free end bends into it in one arc.
    let waypoints: Vec<Waypoint<S>> = waypoints(&part, &through).unwrap();
    let path = RoutePath::through(&waypoints).unwrap();
    let measure = path.measure().unwrap();
    assert_eq!(measure.bends.len(), 4);
    // Mirror images about the bisector: the two arcs of each half are of
    // one circle.
    for half in [0, 2] {
        let (a, b) = (&measure.bends[half].radius, &measure.bends[half + 1].radius);
        assert!(a.could_be_equal(*b), "{a:?} vs {b:?}");
    }
}
#[test]
fn free_points_choose_their_directions() {
    for_all_scalars!(check_free_points_choose_their_directions);
}

/// What a route cannot run through, or cannot do, is refused, saying
/// which point and why.
fn check_unsupported_routes_are_refused<S: Scalar>() {
    let mut part = Part::new();
    connectors(
        &mut part,
        &[
            ("start", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("same", v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)),
            ("end", v3(100.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0)),
        ],
    );
    let rim = clip(&mut part, "clip", v3(10.0, 0.0, 0.0));
    for (name, at) in [
        ("below", v3(10.0, -40.0, 0.0)),
        ("above", v3(10.0, 60.0, 0.0)),
    ] {
        point(&mut part, name, at);
    }
    let refused = |through: Vec<Selected>| refusal::<S>(&part, &args(through, 2.0, 5.0));
    let axis = component_name(ORIGIN, DatumComponent::Axis(FrameAxis::Z));
    let axis = Selected::Datum(ElementReference::new(datums(), &axis));
    let message = refused(vec![connector("start"), axis]);
    assert!(
        message.contains("point 2 (datum origin:z) is an axis"),
        "{message}"
    );
    assert!(message.contains("runs through points"), "{message}");

    let message = refused(vec![
        connector("start"),
        connector("same"),
        connector("end"),
    ]);
    assert!(
        message.contains("point 1 (edge start) and point 2 (edge same) could be one point"),
        "{message}"
    );

    let message = refused(vec![connector("start")]);
    assert!(message.contains("two at least"), "{message}");

    // A clip whose axis lies square to the way the route goes through it.
    let message = refused(vec![vertex("below"), self::rim(&rim), vertex("above")]);
    assert!(
        message.contains(&format!(
            "the axis of point 2 (edge {rim}) could lie square to the route"
        )),
        "{message}"
    );

    let mut no_wires = args(vec![connector("start"), connector("end")], 2.0, 5.0);
    no_wires.wires.clear();
    let message = refusal::<S>(&part, &no_wires);
    assert!(message.contains("no wires"), "{message}");
}
#[test]
fn unsupported_routes_are_refused() {
    for_all_scalars!(check_unsupported_routes_are_refused);
}

/// Every combination of a start connector's heading, an end connector's
/// place and heading, and what lies between — nothing, a free point, a
/// coordinate system — routed: each either builds a valid bundle at least
/// as long as the straight lines between its points, and no shorter than
/// the polyline through points sampled along it, or is refused by the
/// route itself, saying why. Never does it fail anywhere deeper.
#[test]
#[ignore = "slow: a sweep of 432 waypoint configurations, five minutes — run with `cargo test -- --ignored`"]
fn waypoint_configurations_route_or_are_refused() {
    type S = Design;
    let directions: [Vector3<S>; 6] = [
        v3(1.0, 0.0, 0.0),
        v3(0.0, 1.0, 0.0),
        v3(0.0, 0.0, 1.0),
        v3(-1.0, 0.0, 0.0),
        v3(0.0, -1.0, 0.0),
        v3(0.6, 0.0, 0.8),
    ];
    let ends = [
        v3(60.0, 0.0, 0.0),
        v3(40.0, 30.0, 0.0),
        v3(10.0, -20.0, 35.0),
        v3(-30.0, 25.0, 15.0),
    ];
    // Where a middle point is, and the z axis of the frame there, if any.
    type Middle = Option<(Vector3<S>, Option<Vector3<S>>)>;
    let middles: [Middle; 3] = [
        None,
        Some((v3(25.0, 15.0, 10.0), None)),
        Some((v3(20.0, -10.0, 5.0), Some(v3(0.0, 0.0, 1.0)))),
    ];
    let (mut built, mut refused) = (0, 0);
    for start in &directions {
        for end in &ends {
            for end_heading in &directions {
                for middle in &middles {
                    let mut part = Part::new();
                    connectors(
                        &mut part,
                        &[
                            ("start", v3(0.0, 0.0, 0.0), *start),
                            ("end", *end, *end_heading),
                        ],
                    );
                    let mut through = vec![connector("start")];
                    let mut points = vec![v3::<S>(0.0, 0.0, 0.0)];
                    if let Some((at, heading)) = middle {
                        match heading {
                            Some(z) => {
                                connectors(&mut part, &[("middle", *at, *z)]);
                                through.push(connector("middle"));
                            }
                            None => {
                                point(&mut part, "middle", *at);
                                through.push(vertex("middle"));
                            }
                        }
                        points.push(*at);
                    }
                    through.push(connector("end"));
                    points.push(*end);
                    let case =
                        format!("{start:?} -> {middle:?} -> {end:?} heading {end_heading:?}");
                    let args = args(through, 1.0, 1.0);
                    let (laid, cable) = match route::<S>(&part, &args) {
                        Ok(built) => built,
                        Err(e) => {
                            let message = e.root_message();
                            assert!(message.starts_with("route: "), "{case}: {e:?}");
                            assert!(message.contains("point"), "{case}: {message}");
                            refused += 1;
                            continue;
                        }
                    };
                    built += 1;
                    let params = ValidationParameters::default();
                    if let Err(e) = validate(&params, laid.topology()) {
                        panic!("{case}: {e:?}");
                    }
                    let chords: f64 = points
                        .windows(2)
                        .map(|w| w[1].sub(&w[0]).norm().to_f64())
                        .sum();
                    assert!(
                        cable.length.upper().to_f64() >= chords,
                        "{case}: {:?}",
                        cable.length
                    );
                    let waypoints: Vec<Waypoint<S>> = waypoints(&part, &args.through).unwrap();
                    let path = RoutePath::through(&waypoints).unwrap();
                    let sampled: f64 = path
                        .chain
                        .curves
                        .iter()
                        .map(|curve| {
                            let at =
                                |k: usize| curve.evaluate(S::from_f64(k as f64 / 64.0)).unwrap();
                            (0..64)
                                .map(|k| at(k + 1).sub(&at(k)).norm().to_f64())
                                .sum::<f64>()
                        })
                        .sum();
                    assert!(
                        cable.length.upper().to_f64() >= sampled,
                        "{case}: {:?} < {sampled}",
                        cable.length
                    );
                    assert!(
                        cable.length.to_f64() - sampled < 1e-3 * cable.length.to_f64(),
                        "{case}: {:?} vs sampled {sampled}",
                        cable.length
                    );
                }
            }
        }
    }
    assert!(built > refused, "{built} built, {refused} refused");
}
