//! Routes laid between datum frames and through circular edges: their
//! lengths against the analytical ones, their bends checked, their bundles
//! valid solids.

use geop_core_math::{
    for_all_scalars,
    geop_error::GeopResult,
    primitives::{CoordinateSystem, Datum, DatumKind},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, Operation, Part};
use geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder;

use crate::{
    Route, RouteArgs, Wire, WireSize,
    route::{RoutePath, Waypoint},
};

fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// A part with a connector — a coordinate system the cable leaves along
/// its `z` axis — named `name` at `at` for every `(name, at, z)`.
fn connectors<S: Scalar>(part: &mut Part<S>, frames: &[(&str, Vector3<S>, Vector3<S>)]) {
    for (name, at, z) in frames {
        let u = if z[0].to_f64().abs() < 0.9 {
            v(1.0, 0.0, 0.0)
        } else {
            v(0.0, 1.0, 0.0)
        };
        let u = u.sub(&z.prod_scalar(z.prod_dot(&u))).normalize().unwrap();
        let frame = CoordinateSystem::try_new(*at, u, z.prod_cross(&u), *z).unwrap();
        part.add_datum(
            Datum {
                kind: DatumKind::Frame,
                frame,
            },
            *name,
        )
        .unwrap();
    }
}

fn datum(name: &str) -> EntityRef {
    EntityRef::datum(name)
}

/// One wire `d` across: a bundle `d / sqrt(fill)` across.
fn args(through: Vec<EntityRef>, d: f64, bend_factor: f64) -> RouteArgs {
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

fn route<S: Scalar>(part: Part<S>, args: &RouteArgs) -> GeopResult<Part<S>> {
    Route.apply(part, "r", args, &NoFiles)
}

/// Why routing `args` in `part` is refused.
#[track_caller]
fn refusal<S: Scalar>(part: Part<S>, args: &RouteArgs) -> String {
    match route(part, args) {
        Ok(_) => panic!("{args:?} was routed"),
        Err(e) => e.root_message().to_string(),
    }
}

#[track_caller]
fn assert_valid<S: Scalar>(part: &Part<S>) {
    part.check_names().unwrap();
    let params = ValidationParameters::default();
    let scalar = std::any::type_name::<S>();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("in {scalar}: {e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("in {scalar}: {e:?}");
    }
}

/// Between two connectors facing each other, the route runs straight: as
/// long as they are apart, every wire cut to that and a service loop at
/// each end, the bundle a straight cylinder.
fn check_straight_route_between_facing_connectors<S: Scalar>() {
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("end", v(120.0, 0.0, 0.0), v(-1.0, 0.0, 0.0)),
        ],
    );
    let mut args = args(vec![datum("start"), datum("end")], 2.0, 5.0);
    args.service_loop = 15.0;
    args.wires.push(Wire {
        size: WireSize::Awg {
            gauge: 20,
            insulation: 0.3,
        },
        colour: "#ff0000".into(),
        ..Wire::new("w2")
    });
    let part = route(part, &args).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 4 + 2, "a cylinder of four quarters, capped");
    let cable = part.cable("route(r)").unwrap();
    assert!(cable.length.could_be_equal(S::from_f64(120.0)), "{:?}", cable.length);
    assert!(cable.length.width().to_f64() < 1e-6, "{:?}", cable.length);
    assert!(cable.tightest_bend.is_none());
    assert_eq!(cable.wires.len(), 2);
    for wire in &cable.wires {
        assert!(wire.cut_length.could_be_equal(S::from_f64(150.0)), "{wire:?}");
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
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("end", v(r, r, 0.0), v(0.0, -1.0, 0.0)),
        ],
    );
    let part = route(part, &args(vec![datum("start"), datum("end")], 2.0, 5.0)).unwrap();
    assert_valid(&part);
    let cable = part.cable("route(r)").unwrap();
    let quarter = S::PI.mul(S::from_f64(r / 2.0));
    assert!(cable.length.could_be_equal(quarter), "{:?}", cable.length);
    let scalar = std::any::type_name::<S>();
    // Fixed point at this scale holds the arcs' centres to a few millionths.
    assert!(cable.length.width().to_f64() < 1e-4, "{scalar}: {:?}", cable.length);
    let tightest = cable.tightest_bend.unwrap();
    assert!(tightest.could_be_equal(S::from_f64(r)), "{tightest:?}");
    // Halfway round, the bundle's centre passes `R (1 - 1/sqrt 2)` off
    // either axis: its section there is centred on the circle.
    let half = r * (1.0 - std::f64::consts::FRAC_1_SQRT_2);
    let centre = v::<S>(r - half, half, 0.0);
    let model = part.topology();
    for p in 0..4 {
        let vertex = part.vertex_id(&format!("route(r,p{p},m0)")).unwrap();
        let at = model.get_vertex(vertex).unwrap().point;
        let off = at.sub(&centre).norm();
        assert!(off.could_be_equal(S::ONE), "{at:?} is {off:?} off the centre");
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
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("end", v(r, r, 0.0), v(0.0, -1.0, 0.0)),
        ],
    );
    // A bundle 2 across, bent no tighter than 5 diameters: radius 10.
    let message = refusal(part, &args(vec![datum("start"), datum("end")], 2.0, 5.0));
    assert!(
        message.contains("may bend no tighter than radius 10.000 (5 × its diameter)"),
        "{message}"
    );
    assert!(
        message.contains("with radius 8.000 between point 1 (start) and point 2 (end)"),
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
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("end", v(r, r, 0.0), v(0.0, -1.0, 0.0)),
        ],
    );
    let part = route(part, &args(vec![datum("start"), datum("end")], 2.0, 5.0)).unwrap();
    assert_valid(&part);
}
#[test]
fn a_bend_at_the_limit_is_allowed() {
    for_all_scalars!(check_a_bend_at_the_limit_is_allowed);
}

/// A clip: a cylinder named `name` along `z`, of radius 3 and 10 long,
/// standing on `base` — and the name of its circular edge round `base`.
fn clip<S: Scalar>(part: &mut Part<S>, name: &str, base: Vector3<S>) -> String {
    revolved_cylinder(part, name, base, S::from_f64(3.0), S::from_f64(10.0)).unwrap();
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
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, -55.0), v(0.0, 0.0, 1.0)),
            ("end", v(0.0, 0.0, 45.0), v(0.0, 0.0, -1.0)),
        ],
    );
    let rim = clip(&mut part, "clip", v(10.0, 0.0, 0.0));
    let through = vec![
        datum("start"),
        EntityRef::Edge { name: rim.clone() },
        datum("end"),
    ];
    let part = route(part, &args(through, 2.0, 5.0)).unwrap();
    assert_valid(&part);
    let model = part.topology();
    // The bundle's section at the clip is centred on the rim's centre,
    // square to its axis.
    for p in 0..4 {
        let vertex = part.vertex_id(&format!("route(r,p{p},w1)")).unwrap();
        let at = model.get_vertex(vertex).unwrap().point;
        assert!(at[2].could_be_equal(S::ZERO), "{at:?}");
        let off = at.sub(&v(10.0, 0.0, 0.0)).norm();
        assert!(off.could_be_equal(S::ONE), "{at:?}");
    }
    // Each half swerves 10 sideways over 55 and 45, along z at both ends:
    // two equal arcs turning by `2 atan(10 / L)` each, of radius
    // `L / (2 sin)` — the shorter half bends tighter.
    let cable = part.cable("route(r)").unwrap();
    let turn = |l: f64| 2.0 * (10.0 / l).atan();
    let radius = |l: f64| l / (2.0 * turn(l).sin());
    let tightest = cable.tightest_bend.unwrap();
    assert!(encloses(tightest, radius(45.0)), "{tightest:?} vs {}", radius(45.0));
    let expected = [55.0, 45.0].map(|l| 2.0 * turn(l) * radius(l)).iter().sum();
    assert!(encloses(cable.length, expected), "{:?} vs {expected}", cable.length);
}
#[test]
fn route_through_a_clip() {
    for_all_scalars!(check_route_through_a_clip);
}

/// Datum points and vertices leave the direction free: between two of
/// them the route is straight, and a free middle point is passed turning
/// as much before it as after.
fn check_free_points_choose_their_directions<S: Scalar>() {
    let mut part = Part::<S>::new();
    for (name, at) in [("a", v(0.0, 0.0, 0.0)), ("b", v(50.0, 0.0, 0.0)), ("c", v(100.0, 30.0, 0.0))] {
        part.add_datum(
            Datum {
                kind: DatumKind::Point,
                frame: CoordinateSystem::world_at(at),
            },
            name,
        )
        .unwrap();
    }
    let straight = route(part.clone(), &args(vec![datum("a"), datum("b")], 2.0, 5.0)).unwrap();
    let cable = straight.cable("route(r)").unwrap();
    assert!(cable.length.could_be_equal(S::from_f64(50.0)));

    let bent = route(part, &args(vec![datum("a"), datum("b"), datum("c")], 2.0, 5.0)).unwrap();
    assert_valid(&bent);
    // The middle point is passed along the bisector of the chords to its
    // neighbours; each free end bends into it in one arc.
    let waypoints: Vec<Waypoint<S>> = ["a", "b", "c"]
        .iter()
        .enumerate()
        .map(|(i, n)| Waypoint::of(&bent, &datum(n), i).unwrap())
        .collect();
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
    let mut part = Part::<S>::new();
    connectors(
        &mut part,
        &[
            ("start", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("same", v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)),
            ("end", v(100.0, 0.0, 0.0), v(-1.0, 0.0, 0.0)),
        ],
    );
    let rim = clip(&mut part, "clip", v(10.0, 0.0, 0.0));
    for (name, at) in [("below", v(10.0, -40.0, 0.0)), ("above", v(10.0, 60.0, 0.0))] {
        part.add_datum(
            Datum {
                kind: DatumKind::Point,
                frame: CoordinateSystem::world_at(at),
            },
            name,
        )
        .unwrap();
    }
    let refused = |through: Vec<EntityRef>| {
        refusal(part.clone(), &args(through, 2.0, 5.0))
    };
    let axis = EntityRef::datum_component(
        "origin",
        geop_core_math::primitives::DatumComponent::Axis(
            geop_core_math::primitives::FrameAxis::Z,
        ),
    );
    let message = refused(vec![datum("start"), axis]);
    assert!(message.contains("point 2 (origin z axis)"), "{message}");
    assert!(message.contains("runs through points"), "{message}");

    let message = refused(vec![datum("start"), datum("same"), datum("end")]);
    assert!(
        message.contains("point 1 (start) and point 2 (same) could be one point"),
        "{message}"
    );

    let message = refused(vec![datum("start")]);
    assert!(message.contains("two at least"), "{message}");

    // A clip whose axis lies square to the way the route goes through it.
    let message = refused(vec![
        datum("below"),
        EntityRef::Edge { name: rim.clone() },
        datum("above"),
    ]);
    assert!(
        message.contains(&format!("the axis of point 2 ({rim}) could lie square to the route")),
        "{message}"
    );

    let mut no_wires = args(vec![datum("start"), datum("end")], 2.0, 5.0);
    no_wires.wires.clear();
    let message = refusal(part.clone(), &no_wires);
    assert!(message.contains("no wires"), "{message}");
}
#[test]
fn unsupported_routes_are_refused() {
    for_all_scalars!(check_unsupported_routes_are_refused);
}

