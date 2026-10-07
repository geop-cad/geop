use geop_core_math::scalars::ScalInF64;

use super::*;

type S = ScalInF64;

const Z: [f64; 3] = [0.0, 0.0, 1.0];

fn n(x: f64) -> S {
    S::from_f64(x)
}

fn v(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<S> {
    Pose::from_euler(v(position), degrees.map(S::from_f64)).unwrap()
}

/// Where `pose` puts the point `p`.
fn at(pose: &Pose<S>, p: [f64; 3]) -> [f64; 3] {
    let q = pose.apply(&v(p));
    [0, 1, 2].map(|k| q[k].to_f64())
}

fn position(pose: &Pose<S>) -> [f64; 3] {
    at(pose, [0.0; 3])
}

fn body(position: [f64; 3]) -> Body<S> {
    Body {
        pose: pose(position, [0.0; 3]),
        free: true,
        center: v([0.0; 3]),
    }
}

fn on(body: usize, geometry: Geometry<S>) -> Feature<S> {
    Feature {
        body: Some(body),
        geometry,
    }
}

fn ground(geometry: Geometry<S>) -> Feature<S> {
    Feature {
        body: None,
        geometry,
    }
}

fn point(at: [f64; 3]) -> Geometry<S> {
    Geometry::Point { at: v(at) }
}

fn line(point: [f64; 3], direction: [f64; 3]) -> Geometry<S> {
    Geometry::Line {
        point: v(point),
        direction: v(direction),
    }
}

fn plane(point: [f64; 3], normal: [f64; 3]) -> Geometry<S> {
    Geometry::Plane {
        point: v(point),
        normal: v(normal),
    }
}

fn close(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < tol)
}

fn drag(body: usize, local: [f64; 3], target: [f64; 3]) -> Pull<S> {
    Pull::Point {
        body,
        local: v(local),
        target: v(target),
    }
}

/// A turning joint: `local` axis of body `b` on the `world` axis of the
/// ground (or of body `other`), the body kept in the `z = 0` plane.
fn pivot(
    kind_body: usize,
    local: [f64; 3],
    other: Option<usize>,
    at: [f64; 3],
) -> [Constraint<S>; 1] {
    [Constraint {
        kind: Kind::Concentric,
        a: on(kind_body, line(local, Z)),
        b: Feature {
            body: other,
            geometry: line(at, Z),
        },
    }]
}

/// A joint's end on `body` (or the ground), at `origin` along `axis`, its
/// turns measured from the body's own axis least along it.
fn end(body: Option<usize>, origin: [f64; 3], axis: [f64; 3]) -> JointEnd<S> {
    JointEnd {
        body,
        connector: Connector::new(v(origin), v(axis), None).unwrap(),
    }
}

fn flat(b: usize) -> Constraint<S> {
    Constraint {
        kind: Kind::Coincident,
        a: on(b, plane([0.0; 3], Z)),
        b: ground(plane([0.0; 3], Z)),
    }
}

/// A point mated to a fixed point moves the body there, without turning
/// it.
#[test]
fn a_body_moves_onto_its_mate() {
    let mut assembly = Assembly {
        bodies: vec![body([3.0, 1.0, 0.0])],
        constraints: vec![Constraint {
            kind: Kind::Coincident,
            a: on(0, point([1.0, 0.0, 0.0])),
            b: ground(point([0.0, 0.0, 2.0])),
        }],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let pose = assembly.bodies[0].pose;
    assert!(close(at(&pose, [1.0, 0.0, 0.0]), [0.0, 0.0, 2.0], 1e-8));
}

/// A peg in a hole: concentric axes and seated faces leave it free only to
/// turn about the axis and the solve finds the nearest such place.
#[test]
fn a_peg_seats_in_its_hole() {
    let mut assembly = Assembly {
        bodies: vec![Body {
            pose: pose([0.3, -0.2, 1.5], [5.0, -4.0, 30.0]),
            free: true,
            center: v([0.0, 0.0, 0.5]),
        }],
        constraints: vec![
            Constraint {
                kind: Kind::Concentric,
                a: on(0, line([0.0; 3], Z)),
                b: ground(line([0.0; 3], Z)),
            },
            Constraint {
                kind: Kind::Coincident,
                a: on(0, plane([0.0; 3], Z)),
                b: ground(plane([0.0, 0.0, 1.0], Z)),
            },
        ],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let pose = assembly.bodies[0].pose;
    assert!(close(position(&pose), [0.0, 0.0, 1.0], 1e-8), "{pose:?}");
    // Still turned about the axis as it was, roughly: the solve moves it as
    // little as it can.
    assert!((pose.euler_degrees()[2] - 30.0).abs() < 5.0, "{pose:?}");
}

/// Dragging a free body by a point of it moves it, without turning it.
#[test]
fn a_free_body_follows_a_drag_without_turning() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3])],
        constraints: vec![],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    assembly
        .solve(&[drag(0, [1.0, 1.0, 0.0], [3.0, 1.0, 0.0])])
        .unwrap();
    let pose = assembly.bodies[0].pose;
    assert!(close(position(&pose), [2.0, 0.0, 0.0], 1e-3), "{pose:?}");
    assert!(close(pose.euler_degrees(), [0.0; 3], 1e-1), "{pose:?}");
}

/// Dragging the tip of a crank turns it about its axis.
#[test]
fn a_crank_turns_when_dragged() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3])],
        constraints: [pivot(0, [0.0; 3], None, [0.0; 3]).to_vec(), vec![flat(0)]].concat(),
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly
        .solve(&[drag(0, [1.0, 0.0, 0.0], [0.0, 2.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    let tip = at(&assembly.bodies[0].pose, [1.0, 0.0, 0.0]);
    assert!(close(tip, [0.0, 1.0, 0.0], 1e-3), "{tip:?}");
}

/// A four-bar linkage — a kinematic loop of three moving bodies — follows
/// its crank: dragging the crank's tip turns the crank, and the coupler and
/// the rocker move with it so the loop stays closed.
#[test]
fn a_four_bar_linkage_follows_its_crank() {
    // Crank 0 about (0, 0), its tip at local (1, 0); coupler 1 from the
    // crank's tip, its far end at local (2, 2); rocker 2 about (3, 0), its
    // tip at local (0, 2).
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3]), body([1.0, 0.0, 0.0]), body([3.0, 0.0, 0.0])],
        constraints: [
            pivot(0, [0.0; 3], None, [0.0; 3]).to_vec(),
            pivot(1, [0.0; 3], Some(0), [1.0, 0.0, 0.0]).to_vec(),
            pivot(2, [0.0; 3], None, [3.0, 0.0, 0.0]).to_vec(),
            pivot(1, [2.0, 2.0, 0.0], Some(2), [0.0, 2.0, 0.0]).to_vec(),
            vec![flat(0), flat(1), flat(2)],
        ]
        .concat(),
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(3.0),
    };
    assert!(assembly.report().unwrap().converged);
    let report = assembly
        .solve(&[drag(0, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    let crank_tip = at(&assembly.bodies[0].pose, [1.0, 0.0, 0.0]);
    assert!(close(crank_tip, [0.0, 1.0, 0.0], 1e-2), "{crank_tip:?}");
    let coupler_start = at(&assembly.bodies[1].pose, [0.0; 3]);
    assert!(close(coupler_start, crank_tip, 1e-6));
    let coupler_end = at(&assembly.bodies[1].pose, [2.0, 2.0, 0.0]);
    let rocker_tip = at(&assembly.bodies[2].pose, [0.0, 2.0, 0.0]);
    assert!(close(coupler_end, rocker_tip, 1e-6));
}

/// A pose pull keeps a body where it was put, so it is the other bodies
/// that move to meet it — all but the sliver their damping holds against
/// the pull, a ten-thousandth of the way.
#[test]
fn a_pose_pull_makes_the_others_give_way() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3]), body([2.0, 0.0, 0.0])],
        constraints: vec![Constraint {
            kind: Kind::Coincident,
            a: on(0, point([1.0, 0.0, 0.0])),
            b: on(1, point([0.0; 3])),
        }],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let target = assembly.bodies[1].pose;
    let report = assembly.solve(&[Pull::Pose { body: 1, target }]).unwrap();
    assert!(report.converged, "{report:?}");
    assert!(close(
        position(&assembly.bodies[1].pose),
        [2.0, 0.0, 0.0],
        1e-3
    ));
    assert!(close(
        position(&assembly.bodies[0].pose),
        [1.0, 0.0, 0.0],
        1e-3
    ));
}

/// Dragging a body moves a body mated to it only as far as the mate
/// needs: one whose point lies on the dragged body's plane follows the
/// plane, but does not slide along it or turn — though nothing else holds
/// it.
#[test]
fn an_under_constrained_body_stays_near_where_it_was() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3]), body([0.3, -0.4, 1.0])],
        constraints: vec![Constraint {
            kind: Kind::Coincident,
            a: on(0, plane([0.0, 0.0, 1.0], Z)),
            b: on(1, point([0.0; 3])),
        }],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    assert!(assembly.report().unwrap().converged);
    for step in 1..=5 {
        let lift = 0.4 * step as f64;
        let report = assembly
            .solve(&[drag(0, [0.0; 3], [0.0, 0.0, lift])])
            .unwrap();
        assert!(report.converged, "{report:?}");
        let follower = assembly.bodies[1].pose;
        let dragged = assembly.bodies[0].pose;
        assert!(
            close(position(&follower), [0.3, -0.4, 1.0 + lift], 1e-3),
            "step {step}: {:?} {:?} {report:?}",
            position(&follower),
            position(&dragged),
        );
        assert!(
            close(follower.euler_degrees(), [0.0; 3], 1e-2),
            "{follower:?}"
        );
    }
}

/// Constraints that cannot all hold are reported, the bodies left as near
/// to meeting them as they get.
#[test]
fn conflicting_constraints_are_reported() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3])],
        constraints: vec![
            Constraint {
                kind: Kind::Distance { value: n(1.0) },
                a: on(0, point([0.0; 3])),
                b: ground(point([0.0; 3])),
            },
            Constraint {
                kind: Kind::Distance { value: n(3.0) },
                a: on(0, point([0.0; 3])),
                b: ground(point([0.0; 3])),
            },
        ],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(!report.converged);
    assert_eq!(report.failed, vec![0, 1]);
}

/// Distances and angles between planes and lines.
#[test]
fn distances_and_angles_hold() {
    let mut assembly = Assembly {
        bodies: vec![Body {
            pose: pose([0.2, 0.1, 0.4], [10.0, 20.0, 0.0]),
            free: true,
            center: v([0.0; 3]),
        }],
        constraints: vec![
            Constraint {
                kind: Kind::Distance { value: n(2.0) },
                a: on(0, plane([0.0; 3], Z)),
                b: ground(plane([0.0; 3], Z)),
            },
            Constraint {
                kind: Kind::Angle { value: n(30.0) },
                a: on(0, line([0.0; 3], [1.0, 0.0, 0.0])),
                b: ground(line([0.0; 3], [1.0, 0.0, 0.0])),
            },
        ],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let pose = assembly.bodies[0].pose;
    assert!((position(&pose)[2].abs() - 2.0).abs() < 1e-8, "{pose:?}");
    let x = at(&pose, [1.0, 0.0, 0.0]);
    let angle = (x[0] - position(&pose)[0]).acos().to_degrees();
    assert!((angle - 30.0).abs() < 1e-6, "{angle}");
}

/// An angle mate closes two lines to parallel at 0° and 180°, and opens
/// them from nearly parallel: one row of slope one per turn where the angle
/// is not 0° or 180°, held as parallel lines are where it is. The length of
/// a cross product had no slope at parallel lines, so a solve could not
/// finish there. (From exactly parallel lines, no direction to open in is
/// any better than another: `d·e` is stationary there, and the mate stays
/// unmet.)
#[test]
fn an_angle_closes_to_parallel_and_opens_from_near_it() {
    let solve = |start: f64, value: f64| {
        let mut assembly = Assembly {
            bodies: vec![Body {
                pose: pose([0.0; 3], [0.0, 0.0, start]),
                free: true,
                center: v([0.0; 3]),
            }],
            constraints: vec![Constraint {
                kind: Kind::Angle { value: n(value) },
                a: on(0, line([0.0; 3], [1.0, 0.0, 0.0])),
                b: ground(line([0.0; 3], [1.0, 0.0, 0.0])),
            }],
            joints: Vec::new(),
            couplings: Vec::new(),
            scale: n(1.0),
        };
        let report = assembly.solve(&[]).unwrap();
        assert!(report.converged, "{start}° to {value}°: {report:?}");
        assert!(report.iterations < 20, "{start}° to {value}°: {report:?}");
        let x = at(&assembly.bodies[0].pose, [1.0, 0.0, 0.0]);
        x[0].clamp(-1.0, 1.0).acos().to_degrees()
    };
    for (start, value) in [(2.0, 40.0), (25.0, 0.0), (160.0, 180.0), (90.0, 30.0)] {
        let angle = solve(start, value);
        assert!(
            (angle - value).abs() < 1e-6,
            "{start}° to {value}°: {angle}°"
        );
    }
}

#[test]
fn an_angle_at_a_point_is_refused() {
    let assembly = Assembly {
        bodies: vec![body([0.0; 3])],
        constraints: vec![Constraint {
            kind: Kind::Angle { value: n(30.0) },
            a: on(0, point([0.0; 3])),
            b: ground(line([0.0; 3], Z)),
        }],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    assert!(assembly.validate().is_err());
}

/// A pose pull towards where a turned body already is leaves it there: the
/// pull compares the body's axes with the target's, each once.
#[test]
fn a_pose_pull_leaves_a_turned_body_where_it_is() {
    let pose = pose([2.0, -1.0, 0.5], [20.0, -35.0, 90.0]);
    let mut assembly = Assembly {
        bodies: vec![Body {
            pose,
            free: true,
            center: v([0.5; 3]),
        }],
        constraints: vec![],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    assembly
        .solve(&[Pull::Pose {
            body: 0,
            target: pose,
        }])
        .unwrap();
    let moved = assembly.bodies[0].pose;
    for p in [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 1.0]] {
        assert!(close(at(&moved, p), at(&pose, p), 1e-12), "{moved:?}");
    }
}

/// A drag converges in few iterations: it runs once per frame of a drag,
/// so the pull phase stops once the pull can get no closer, rather than
/// using up its whole budget.
#[test]
fn a_drag_converges_quickly() {
    let mut assembly = Assembly {
        bodies: vec![body([0.0; 3])],
        constraints: [pivot(0, [0.0; 3], None, [0.0; 3]).to_vec(), vec![flat(0)]].concat(),
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.0),
    };
    let report = assembly
        .solve(&[drag(0, [1.0, 0.0, 0.0], [0.0, 2.0, 0.0])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    assert!(report.iterations < 20, "{report:?}");
}

/// The residuals' Jacobian — through the dual-quaternion motion of turned
/// bodies — is the true one: it matches central differences, at the start
/// of a solve (every variable zero) and away from it.
#[test]
fn jacobians_match_finite_differences() {
    let turned = |position: [f64; 3], degrees: [f64; 3]| Body {
        pose: pose(position, degrees),
        free: true,
        center: v([0.3, -0.2, 0.1]),
    };
    let assembly = Assembly {
        bodies: vec![
            turned([0.5, 0.2, -0.1], [20.0, -35.0, 80.0]),
            turned([1.5, -0.4, 0.3], [-60.0, 10.0, 170.0]),
        ],
        constraints: vec![
            Constraint {
                kind: Kind::Concentric,
                a: on(0, line([0.1, 0.0, 0.0], Z)),
                b: on(1, line([0.0, 0.2, 0.0], [1.0, 0.0, 0.0])),
            },
            Constraint {
                kind: Kind::Distance { value: n(0.7) },
                a: on(0, point([1.0, 0.0, 0.0])),
                b: ground(plane([0.0; 3], Z)),
            },
        ],
        joints: vec![Joint {
            kind: JointKind::Cylindrical,
            a: end(Some(0), [0.2, 0.1, 0.0], [0.0, 1.0, 1.0]),
            b: end(Some(1), [0.0, -0.3, 0.4], [1.0, 0.0, 0.5]),
            angle: Coordinate::free(n(25.0)),
            distance: Coordinate::free(n(0.3)),
        }],
        couplings: vec![Coupling {
            kind: CouplingKind::Screw {
                lead: n(0.4),
                reverse: true,
            },
            a: 0,
            b: 0,
        }],
        scale: n(2.0),
    };
    let pulls = [(
        geop_core_math::solvers::system::Pull::Point {
            param: 1,
            local: v([0.5, 0.5, 0.0]),
            target: v([2.0, 1.0, 0.0]),
        },
        n(0.1),
    )];
    let residuals = assembly.residuals().unwrap();
    let (params, mobility) = assembly.parameters(&residuals).unwrap();
    let all = residuals.all();
    let evaluate = |x: &[f64]| {
        let x: Vec<S> = x.iter().map(|&v| S::from_f64(v)).collect();
        let e = geop_core_math::solvers::system::evaluate(
            &params,
            &mobility,
            &all,
            assembly.scale,
            &x,
            &pulls,
            false,
        )
        .unwrap();
        let mid = |v: &[S]| v.iter().map(|v| v.to_f64()).collect::<Vec<_>>();
        (
            mid(&e.sum.values),
            e.sum.jacobian.iter().map(|r| mid(r)).collect::<Vec<_>>(),
        )
    };
    for x in [
        vec![0.0; 14],
        (0..14).map(|i| 0.05 * (i as f64 - 6.0)).collect(),
    ] {
        let (_, jacobian) = evaluate(&x);
        for i in 0..14 {
            let h = 1e-6;
            let mut plus = x.clone();
            plus[i] += h;
            let mut minus = x.clone();
            minus[i] -= h;
            let (rp, rm) = (evaluate(&plus).0, evaluate(&minus).0);
            for (k, row) in jacobian.iter().enumerate() {
                let fd = (rp[k] - rm[k]) / (2.0 * h);
                assert!(
                    (row[i] - fd).abs() < 1e-5 * (1.0 + fd.abs()),
                    "residual {k}, variable {i} at {x:?}: {} vs {fd}",
                    row[i]
                );
            }
        }
    }
}

/// From far off, a solve moves a body as little as it can: onto the axis
/// it is mated to and to the distance from the base, without turning it
/// over on the way.
#[test]
fn a_far_body_is_not_turned_over() {
    let mut assembly = Assembly {
        bodies: vec![Body {
            pose: pose([5.0, 4.0, 3.0], [0.0; 3]),
            free: true,
            center: v([0.5; 3]),
        }],
        constraints: vec![
            Constraint {
                kind: Kind::Concentric,
                a: on(0, line([0.0; 3], Z)),
                b: ground(line([0.0; 3], Z)),
            },
            Constraint {
                kind: Kind::Distance { value: n(1.5) },
                a: on(0, plane([0.0; 3], Z)),
                b: ground(plane([0.0; 3], Z)),
            },
        ],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(1.7),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    let pose = assembly.bodies[0].pose;
    assert!(close(position(&pose), [0.0, 0.0, 1.5], 1e-7), "{pose:?}");
    assert!(close(at(&pose, Z), [0.0, 0.0, 2.5], 1e-7), "{pose:?}");
}

/// A body nothing holds goes exactly where it is dragged: moved, not turned
/// — however far, and wherever on it it is grabbed.
#[test]
fn a_free_body_goes_where_it_is_dragged() {
    let mut assembly = Assembly {
        bodies: vec![body([3.5, 1.0, 0.0])],
        constraints: vec![],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(10.0),
    };
    let report = assembly
        .solve(&[drag(0, [0.3, 0.2, 0.5], [4.8, 1.7, 0.5])])
        .unwrap();
    let pose = assembly.bodies[0].pose;
    assert!(
        close(position(&pose), [4.5, 1.5, 0.0], 1e-9),
        "{pose:?} {report:?}"
    );
    assert!(
        close(pose.euler_degrees(), [0.0; 3], 1e-6),
        "{pose:?} {report:?}"
    );
}

/// A chain of two links on a fixed one, each on the pin of the one before,
/// folded back on itself — the second link turned a half turn, its far end
/// over the first link's pin — and its far end dragged a little: a singular
/// configuration (the links in line), which a drag still answers in few
/// steps.
#[test]
fn a_folded_chain_is_dragged_in_few_steps() {
    let pin = |body: usize, local: [f64; 3], other: Option<usize>, at: [f64; 3]| Constraint {
        kind: Kind::Concentric,
        a: on(body, line(local, Z)),
        b: Feature {
            body: other,
            geometry: line(at, Z),
        },
    };
    let mut assembly = Assembly {
        bodies: vec![
            body([3.0, 0.0, 0.0]),
            Body {
                pose: pose([6.0, 0.0, 0.0], [0.0, 0.0, 180.0]),
                free: true,
                center: v([1.5, 0.0, 0.1]),
            },
        ],
        constraints: vec![
            pin(0, [0.0; 3], None, [3.0, 0.0, 0.0]),
            pin(1, [0.0; 3], Some(0), [3.0, 0.0, 0.0]),
        ],
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(4.0),
    };
    assembly.bodies[0].center = v([1.5, 0.0, 0.1]);
    assert!(assembly.report().unwrap().converged);
    let report = assembly
        .solve(&[drag(1, [2.5, 0.0, 0.2], [3.512, -0.156, 0.12])])
        .unwrap();
    assert!(report.converged, "{report:?}");
    assert!(report.iterations < 20, "{report:?}");
}

/// A 6-axis arm built from mates — each link's axis concentric with the
/// one before's and its origin on that one's end face — its tip dragged to
/// `targets` random points one after the other. Returns, per drag, the
/// report.
fn drag_mated_arm(targets: usize) -> Vec<SolveReport<S>> {
    let axes = [
        Z,
        [0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ];
    let mut constraints = Vec::new();
    for (i, &axis) in axes.iter().enumerate() {
        let before = |geometry: Geometry<S>| Feature {
            body: i.checked_sub(1),
            geometry,
        };
        let top = if i == 0 { [0.0; 3] } else { [0.0, 0.0, 1.0] };
        constraints.push(Constraint {
            kind: Kind::Concentric,
            a: on(i, line([0.0; 3], axis)),
            b: before(line(top, axis)),
        });
        constraints.push(Constraint {
            kind: Kind::Coincident,
            a: on(i, point([0.0; 3])),
            b: before(plane(top, axis)),
        });
    }
    let mut assembly = Assembly::new(
        (0..6)
            .map(|i| Body {
                center: v([0.0, 0.0, 0.5]),
                ..body([0.0, 0.0, i as f64])
            })
            .collect(),
        constraints,
        n(6.0),
    );
    let mut seed: u64 = 12345;
    let mut random = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (seed >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    };
    (0..targets)
        .map(|_| {
            let target = [random() * 2.0, random() * 2.0, 1.0 + random() * 2.0];
            assembly.solve(&[drag(5, [0.0, 0.0, 1.0], target)]).unwrap()
        })
        .collect()
}

/// The drags of `reports` whose mates the constrained minimization did not
/// meet by itself — left for the least-squares pass after it — or that took
/// every step it may.
fn unmet<T: Scalar>(reports: &[SolveReport<T>]) -> Vec<String> {
    reports
        .iter()
        .enumerate()
        .filter(|(_, r)| {
            !r.converged
                || r.phases.len() > 1
                || r.phases
                    .iter()
                    .any(|p| p.stop == geop_core_math::solvers::least_squares::Stop::Budget)
        })
        .map(|(k, r)| format!("drag {k}: {r:?}"))
        .collect()
}

/// A mated 6-axis arm follows its tip dragged about: every drag ends with
/// every mate holding, met by the constrained minimization itself.
/// Concentric axes hold by two rows of constant rank; as a cross product,
/// three rows that lose one as the axes close, the first drag already
/// failed. Drag 6 once ran its turn variables off towards a half turn, and
/// drag 10 wandered off the mates by whole lengths (see `Placed` and
/// `least_squares::corrected`).
#[test]
fn a_mated_arm_follows_its_tip() {
    let unmet = unmet(&drag_mated_arm(12));
    assert!(unmet.is_empty(), "{}", unmet.join("\n"));
}

/// [`a_mated_arm_follows_its_tip`], through 40 drags, none taking every
/// step it may. (Drags 9, 11 and 33 once spent every step on the
/// preferences once the mates held and the tip was there: the model's
/// curvature left out the constraints' where it is negative, which a BFGS
/// estimate cannot learn — see `least_squares`.)
#[test]
#[ignore = "slow: a mated arm dragged 40 times — run with `cargo test -- --ignored`"]
fn a_mated_arm_follows_its_tip_through_every_drag() {
    let unmet = unmet(&drag_mated_arm(40));
    assert!(unmet.is_empty(), "{}", unmet.join("\n"));
}

#[path = "joints_tests.rs"]
mod joints;

/// Bodies no constraint ties together are solved group by group: a fixed
/// plate with 300 screws on it, half of them lifted off, and the last
/// screw standing on the one before it. Each lifted one comes down onto
/// the plate where it is, those already on it are not moved at all, and
/// the solve takes as many steps as one screw's — a dense solve of every
/// screw at once took tens of seconds for a few hundred.
#[test]
fn independent_bodies_are_solved_apart() {
    let screws = 300;
    let mut bodies = vec![Body {
        free: false,
        ..body([0.0; 3])
    }];
    let mut constraints = Vec::new();
    for i in 0..screws {
        let lifted = if i % 2 == 0 { 0.3 } else { 0.0 };
        bodies.push(body([i as f64, 0.0, 1.0 + lifted]));
        // The last on the one before: its bottom on that one's top.
        let below = match i + 1 == screws {
            false => on(0, plane([0.0, 0.0, 1.0], Z)),
            true => on(i, plane([0.0, 0.0, 1.0], Z)),
        };
        constraints.push(Constraint {
            kind: Kind::Coincident,
            a: on(i + 1, plane([0.0; 3], Z)),
            b: below,
        });
    }
    let mut assembly = Assembly {
        bodies: bodies.clone(),
        constraints,
        joints: Vec::new(),
        couplings: Vec::new(),
        scale: n(10.0),
    };
    let report = assembly.solve(&[]).unwrap();
    assert!(report.converged, "{report:?}");
    assert!(report.iterations < 20, "{report:?}");
    for i in 0..screws {
        let (before, after) = (&bodies[i + 1].pose, &assembly.bodies[i + 1].pose);
        let z = if i + 1 == screws { 2.0 } else { 1.0 };
        if i % 2 == 1 && i + 2 < screws {
            assert_eq!(before, after, "screw {i} was on the plate already");
        } else {
            assert!(close(position(after), [i as f64, 0.0, z], 1e-9), "{i}");
        }
    }
}
