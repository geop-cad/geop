//! Joints, their limits and the couplings between them.

use super::*;

/// A joint of `kind` between the ground's connector at `at` along `axis`
/// and body `body`'s at its own origin along the same axis, both
/// coordinates starting at zero.
fn joint(kind: JointKind<S>, body: usize, at: [f64; 3], axis: [f64; 3]) -> Joint<S> {
    Joint {
        kind,
        a: end(None, at, axis),
        b: end(Some(body), [0.0; 3], axis),
        angle: Coordinate::free(n(0.0)),
        distance: Coordinate::free(n(0.0)),
    }
}

fn revolute(min: Option<f64>, max: Option<f64>) -> JointKind<S> {
    JointKind::Revolute {
        min: min.map(n),
        max: max.map(n),
    }
}

fn slider(min: Option<f64>, max: Option<f64>) -> JointKind<S> {
    JointKind::Slider {
        min: min.map(n),
        max: max.map(n),
    }
}

fn joints(bodies: Vec<Body<S>>, joints: Vec<Joint<S>>, couplings: Vec<Coupling<S>>) -> Assembly<S> {
    Assembly {
        bodies,
        constraints: Vec::new(),
        joints,
        couplings,
        scale: n(1.0),
    }
}

/// A revolute joint lets its body only turn about its axis: dragged up and
/// across, the crank's tip stays in its plane, at its radius, and the
/// joint's angle says how far it turned.
#[test]
fn a_revolute_joint_turns_only_about_its_axis() {
    let mut assembly = joints(
        vec![body([0.0; 3])],
        vec![joint(revolute(None, None), 0, [0.0; 3], Z)],
        Vec::new(),
    );
    let report = assembly
        .solve(&[drag(0, [2.0, 0.0, 0.0], [0.0, 3.0, 5.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    let tip = at(&assembly.bodies[0].pose, [2.0, 0.0, 0.0]);
    assert!(close(tip, [0.0, 2.0, 0.0], 1e-3), "{tip:?} {report:?}");
    let angle = assembly.joints[0].angle.value.to_f64();
    assert!((angle - 90.0).abs() < 0.1, "{angle}");
}

/// Dragged past its limit, a revolute joint stops at it: the crank turns
/// to 45° and no further, and the report says the joint is at its limit.
/// Dragged back, it follows again.
#[test]
fn a_revolute_joint_stops_at_its_limits() {
    let mut assembly = joints(
        vec![body([0.0; 3])],
        vec![joint(revolute(Some(-30.0), Some(45.0)), 0, [0.0; 3], Z)],
        Vec::new(),
    );
    let report = assembly
        .solve(&[drag(0, [2.0, 0.0, 0.0], [0.0, 2.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    assert_eq!(report.at_limit, [(0, Motion::Turn)]);
    let half = 2.0_f64.sqrt();
    let tip = at(&assembly.bodies[0].pose, [2.0, 0.0, 0.0]);
    assert!(close(tip, [half, half, 0.0], 1e-9), "{tip:?}");
    assert_eq!(assembly.joints[0].angle.value.to_f64(), 45.0);
    assert!(!assembly.joints[0].angle.held, "held for that solve only");

    let report = assembly
        .solve(&[drag(0, [2.0, 0.0, 0.0], [2.0, 0.5, 0.0])])
        .unwrap();
    assert!(report.converged && report.at_limit.is_empty(), "{report:?}");
    let angle = assembly.joints[0].angle.value.to_f64();
    let expected = 0.5_f64.atan2(2.0).to_degrees();
    assert!((angle - expected).abs() < 0.1, "{angle} vs {expected}");

    let report = assembly
        .solve(&[drag(0, [2.0, 0.0, 0.0], [0.0, -2.0, 0.0])])
        .unwrap();
    assert_eq!(report.at_limit, [(0, Motion::Turn)]);
    assert_eq!(assembly.joints[0].angle.value.to_f64(), -30.0);
}

/// A joint's value held puts the body there: the crank set to 120° turns to
/// it, and the value stays exactly as it was set.
#[test]
fn a_held_joint_value_places_its_body() {
    let mut j = joint(revolute(None, None), 0, [0.0; 3], Z);
    j.angle = Coordinate {
        value: n(120.0),
        held: true,
    };
    let mut assembly = joints(vec![body([0.0; 3])], vec![j], Vec::new());
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let tip = at(&assembly.bodies[0].pose, [1.0, 0.0, 0.0]);
    assert!(close(tip, [-0.5, 0.75_f64.sqrt(), 0.0], 1e-9), "{tip:?}");
    assert_eq!(assembly.joints[0].angle.value.to_f64(), 120.0);
}

/// A slider moves its body only along its axis, and stops at its limits.
#[test]
fn a_slider_stops_at_its_limits() {
    let x = [1.0, 0.0, 0.0];
    let mut assembly = joints(
        vec![body([0.0; 3])],
        vec![joint(slider(Some(-1.0), Some(2.0)), 0, [0.0; 3], x)],
        Vec::new(),
    );
    let report = assembly
        .solve(&[drag(0, [0.0; 3], [1.5, 3.0, -1.0])])
        .unwrap();
    assert!(report.converged && report.at_limit.is_empty(), "{report:?}");
    assert!(
        close(position(&assembly.bodies[0].pose), [1.5, 0.0, 0.0], 1e-3),
        "{:?}",
        assembly.bodies[0].pose
    );
    let report = assembly
        .solve(&[drag(0, [0.0; 3], [5.0, 3.0, 0.0])])
        .unwrap();
    assert_eq!(report.at_limit, [(0, Motion::Slide)]);
    assert!(close(
        position(&assembly.bodies[0].pose),
        [2.0, 0.0, 0.0],
        1e-9
    ));
    assert!(close(
        assembly.bodies[0].pose.euler_degrees(),
        [0.0; 3],
        1e-9
    ));
    assembly
        .solve(&[drag(0, [0.0; 3], [-4.0, 0.0, 0.0])])
        .unwrap();
    assert_eq!(assembly.joints[0].distance.value.to_f64(), -1.0);
}

/// Two gears, 2:1, turning opposite ways: the driver dragged a quarter turn
/// turns the follower an eighth turn back.
#[test]
fn a_gear_pair_turns_its_follower_by_its_ratio() {
    let mut assembly = joints(
        vec![body([0.0; 3]), body([3.0, 0.0, 0.0])],
        vec![
            joint(revolute(None, None), 0, [0.0; 3], Z),
            joint(revolute(None, None), 1, [3.0, 0.0, 0.0], Z),
        ],
        vec![Coupling {
            kind: CouplingKind::Gear {
                ratio: n(2.0),
                reverse: true,
            },
            a: 0,
            b: 1,
        }],
    );
    let report = assembly
        .solve(&[drag(0, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    let (driver, follower) = (
        assembly.joints[0].angle.value.to_f64(),
        assembly.joints[1].angle.value.to_f64(),
    );
    assert!((driver - 90.0).abs() < 0.1, "{driver}");
    assert!((follower + driver / 2.0).abs() < 1e-6, "{follower}");
    let turned = assembly.bodies[1].pose.euler_degrees()[2];
    assert!((turned - follower).abs() < 1e-6, "{turned} vs {follower}");
}

/// A rack and pinion: the pinion, of pitch radius 0.5, dragged a quarter
/// turn moves the rack a quarter of the pitch circle along its axis.
#[test]
fn a_rack_follows_its_pinion() {
    let x = [1.0, 0.0, 0.0];
    let mut assembly = joints(
        vec![body([0.0; 3]), body([0.0, -0.5, 0.0])],
        vec![
            joint(revolute(None, None), 0, [0.0; 3], Z),
            joint(slider(None, None), 1, [0.0, -0.5, 0.0], x),
        ],
        vec![Coupling {
            kind: CouplingKind::RackPinion {
                radius: n(0.5),
                reverse: true,
            },
            a: 0,
            b: 1,
        }],
    );
    let report = assembly
        .solve(&[drag(0, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    let turned = assembly.joints[0].angle.value.to_f64();
    let moved = assembly.joints[1].distance.value.to_f64();
    assert!((turned - 90.0).abs() < 0.1, "{turned}");
    assert!((moved + 0.5 * turned.to_radians()).abs() < 1e-6, "{moved}");
    let rack = position(&assembly.bodies[1].pose);
    assert!(close(rack, [moved, -0.5, 0.0], 1e-9), "{rack:?}");
}

/// A screw — one cylindrical joint, its turn coupled to its slide — moves
/// its lead per turn: set to a turn and a half, it has moved one and a half
/// leads along its axis.
#[test]
fn a_screw_advances_its_lead_per_turn() {
    let mut j = joint(JointKind::Cylindrical, 0, [0.0; 3], Z);
    j.angle = Coordinate {
        value: n(540.0),
        held: true,
    };
    let mut assembly = joints(
        vec![body([0.0; 3])],
        vec![j],
        vec![Coupling {
            kind: CouplingKind::Screw {
                lead: n(0.2),
                reverse: false,
            },
            a: 0,
            b: 0,
        }],
    );
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let distance = assembly.joints[0].distance.value.to_f64();
    assert!((distance - 0.3).abs() < 1e-9, "{distance}");
    assert!(close(
        position(&assembly.bodies[0].pose),
        [0.0, 0.0, 0.3],
        1e-9
    ));
}

/// The degrees of freedom left: a free body has six, one on a revolute
/// joint one, one on a cylindrical joint two, and one fastened none.
#[test]
fn the_freedom_of_each_body_is_reported() {
    let mut fixed = body([0.0, 9.0, 0.0]);
    fixed.free = false;
    let assembly = joints(
        vec![
            body([0.0; 3]),
            body([3.0, 0.0, 0.0]),
            body([6.0, 0.0, 0.0]),
            body([9.0, 0.0, 0.0]),
            fixed,
        ],
        vec![
            joint(revolute(None, None), 1, [3.0, 0.0, 0.0], Z),
            joint(JointKind::Cylindrical, 2, [6.0, 0.0, 0.0], Z),
            joint(JointKind::Fastened, 3, [9.0, 0.0, 0.0], Z),
        ],
        Vec::new(),
    );
    let freedom = assembly.freedom().unwrap();
    assert_eq!(freedom.bodies, [6, 1, 2, 0, 0]);
    assert_eq!(freedom.total, 9);
}

/// Mates that conflict are named: a crank on a revolute joint cannot also
/// keep its tip 3 from the axis it turns about at radius 2 — those two,
/// and not the mate that has nothing to do with it.
#[test]
fn conflicting_mates_are_named() {
    let mut assembly = joints(
        vec![body([0.0; 3]), body([5.0, 0.0, 0.0])],
        vec![joint(revolute(None, None), 0, [0.0; 3], Z)],
        Vec::new(),
    );
    assembly.constraints = vec![
        Constraint {
            kind: Kind::Coincident,
            a: on(1, point([0.0; 3])),
            b: ground(point([5.0, 0.0, 0.0])),
        },
        Constraint {
            kind: Kind::Distance { value: n(3.0) },
            a: on(0, point([2.0, 0.0, 0.0])),
            b: ground(line([0.0; 3], Z)),
        },
    ];
    let report = assembly.clone().solve(&[]).unwrap();
    assert!(!report.converged);
    assert_eq!(assembly.conflicting().unwrap(), [1, 2]);
}

/// A joint between two features of one body, or a coupling of a joint
/// that does not turn, is refused, saying why.
#[test]
fn joints_that_hold_nothing_are_refused() {
    let mut same = joint(revolute(None, None), 0, [0.0; 3], Z);
    same.a.body = Some(0);
    let err = joints(vec![body([0.0; 3])], vec![same], Vec::new())
        .solve(&[])
        .unwrap_err()
        .to_string();
    assert!(err.contains("both ends"), "{err}");
    let err = joints(
        vec![body([0.0; 3]), body([1.0, 0.0, 0.0])],
        vec![
            joint(slider(None, None), 0, [0.0; 3], Z),
            joint(revolute(None, None), 1, [1.0, 0.0, 0.0], Z),
        ],
        vec![Coupling {
            kind: CouplingKind::Gear {
                ratio: n(1.0),
                reverse: false,
            },
            a: 0,
            b: 1,
        }],
    )
    .solve(&[])
    .unwrap_err()
    .to_string();
    assert!(err.contains("needs joint 0 to turn"), "{err}");
}
