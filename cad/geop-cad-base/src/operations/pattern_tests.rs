//! Patterns, mirrors and moves of bodies built by programs: holes drilled
//! by patterning a tool body, symmetric parts mirrored from a half, names
//! of the copies, and sweeps over counts and awkward angles.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_pattern::{
    CircularPatternArgs, Direction, LinearPatternArgs, MirrorArgs, MoveBodyArgs, Spacing,
};
use geop_ops_rasterize::{rasterize, stl::stl_triangles};
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        let messages: Vec<String> = errors.iter().map(|e| format!("{e:?}")).collect();
        panic!(
            "{} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:?}");
    }
    part.check_names().unwrap();
}

/// The volume the solid named `solid` encloses, from its mesh as drawn.
fn volume(part: &Part<S>, solid: &str) -> f64 {
    let id = part.solid_id(solid).unwrap();
    let faces = part.topology().body_faces(Body::Solid(id)).unwrap();
    let raster = rasterize(part.topology(), 64).unwrap();
    stl_triangles(&raster, &faces)
        .iter()
        .map(|t| {
            let [a, b, c] = t.corners.map(|p| p.map(f64::from));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

fn z_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z))
}

fn polygon(corners: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
    s
}

fn circle(center: [f64; 2], radius: f64) -> Sketch {
    let mut s = Sketch::new();
    let c = s.add_point(n(center[0]), n(center[1]));
    s.add_circle(c, n(radius));
    s
}

/// A sketch on the Z plane, and `name` its extrude, as `extent` says, a
/// new body.
fn extruded(program: &mut Program, name: &str, sketch: Sketch, extent: Extents) {
    let sketch_name = format!("{name}_sketch");
    program.push(
        &sketch_name,
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch,
            ..Default::default()
        },
    );
    program.push(
        name,
        ExtrudeArgs {
            sketch: sketch_name,
            extent,
            face: false,
            combine: Combine::NewBody,
        },
    );
}

fn solid(name: &str) -> Vec<EntityRef> {
    vec![EntityRef::Solid { name: name.into() }]
}

fn axis(a: FrameAxis) -> Option<EntityRef> {
    Some(EntityRef::datum_component(ORIGIN, DatumComponent::Axis(a)))
}

/// A plate 8 x 2 x 0.5, and a pin of radius 0.3 at (1, 1) through it — a
/// tool kept as a body of its own.
fn plate_and_pin() -> Program {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        polygon(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    let mut through = Extents::blind(2.0);
    through.symmetric = true;
    extruded(&mut program, "pin", circle([1.0, 1.0], 0.3), through);
    program
}

/// Builds `program`, with every step read back from its JSON first: what
/// a program file holds builds the same part.
fn build(program: &Program) -> Part<S> {
    let json = serde_json::to_string(program).unwrap();
    let read: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(&read, program);
    read.build::<S>(&NoFiles).unwrap()
}

/// The pin patterned four times two apart and cut from the plate — itself
/// and every copy: a row of four holes through it.
#[test]
fn row_of_holes_cut_by_a_patterned_tool() {
    let mut program = plate_and_pin();
    program.push(
        "holes",
        LinearPatternArgs {
            bodies: solid("extrude(pin)"),
            first: Direction {
                along: axis(FrameAxis::X),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(2.0),
            },
            second: None,
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
    let expected = 8.0 * 2.0 * 0.5 - 4.0 * PI * 0.3 * 0.3 * 0.5;
    let got = volume(&part, "linear_pattern(holes)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
    // The wall of the third copy's hole is the pin's side, copied — cut
    // where the plate's faces cross it, and named by the cut after it.
    let wall = part.names().iter().any(|(id, name)| {
        matches!(id, geop_ops::RefId::Face(_))
            && name.starts_with("combine(holes,3,linear_pattern(holes,3,extrude(pin,pin_sketch,c1")
    });
    assert!(wall, "no face of the third hole");
}

/// The pin turned six times around a hole's axis through the middle of a
/// square plate: a bolt circle.
#[test]
fn bolt_circle_cut_by_a_patterned_tool() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        polygon(&[[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]]),
        Extents::blind(0.5),
    );
    let mut through = Extents::blind(2.0);
    through.symmetric = true;
    extruded(&mut program, "pin", circle([2.0, 0.0], 0.3), through);
    program.push(
        "bolts",
        CircularPatternArgs {
            bodies: solid("extrude(pin)"),
            axis: axis(FrameAxis::Z),
            reversed: false,
            count: 6.0.into(),
            angle: Spacing::extent(360.0),
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let expected = 36.0 * 0.5 - 6.0 * PI * 0.3 * 0.3 * 0.5;
    let got = volume(&part, "circular_pattern(bolts)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
}

/// An L standing on the `yz` plane, mirrored in it and joined to itself: a
/// symmetric T, twice the L.
#[test]
fn half_mirrored_and_joined() {
    let mut program = Program::new();
    let l = [
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 0.5],
        [0.5, 0.5],
        [0.5, 3.0],
        [0.0, 3.0],
    ];
    extruded(&mut program, "half", polygon(&l), Extents::blind(1.0));
    program.push(
        "m",
        MirrorArgs {
            bodies: solid("extrude(half)"),
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            combine: Combine::Union {
                target: "extrude(half)".into(),
            },
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["mirror(m)"]);
    let got = volume(&part, "mirror(m)");
    assert!((got - 2.0 * 2.25).abs() < 1e-6, "{got}");
}

/// An L away from the plane, mirrored as a new body: the image's corners
/// are the L's mirrored — its foot pointing the other way — and it faces
/// out.
#[test]
fn asymmetric_body_mirrored() {
    let mut program = Program::new();
    let l = [
        [1.0, 0.0],
        [3.0, 0.0],
        [3.0, 0.5],
        [1.5, 0.5],
        [1.5, 3.0],
        [1.0, 3.0],
    ];
    extruded(&mut program, "l", polygon(&l), Extents::blind(1.0));
    program.push(
        "m",
        MirrorArgs {
            bodies: solid("extrude(l)"),
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            combine: Combine::NewBody,
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let image = part.solid_id("mirror(m,extrude(l))").unwrap();
    let model = part.topology();
    let mut corners: Vec<[i64; 3]> = model
        .iter_body_vertices(image)
        .unwrap()
        .map(|v| {
            let p = model.get_vertex(v).unwrap().point;
            [0, 1, 2].map(|k| (p[k].to_f64() * 2.0).round() as i64)
        })
        .collect();
    corners.sort();
    let mut expected: Vec<[i64; 3]> = l
        .iter()
        .flat_map(|c| [0, 2].map(|z| [(-c[0] * 2.0) as i64, (c[1] * 2.0) as i64, z]))
        .collect();
    expected.sort();
    assert_eq!(corners, expected);
    let got = volume(&part, "mirror(m,extrude(l))");
    assert!((got - 2.25).abs() < 1e-6, "{got}");
}

/// A pin moved into place and cut from the plate, rather than drawn
/// there: the plate with one hole at (5, 1).
#[test]
fn tool_moved_into_place_and_cut() {
    let mut program = plate_and_pin();
    program.push(
        "mv",
        MoveBodyArgs {
            bodies: solid("extrude(pin)"),
            translation: [4.0, 0.0, 0.0],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let expected = 8.0 - PI * 0.09 * 0.5;
    let got = volume(&part, "move(mv)");
    assert!((got - expected).abs() < 1e-3, "{got} vs {expected}");
}

/// A circle of boxes, up to 24 of them, at awkward angles, each valid,
/// and joined where they overlap.
#[test]
#[ignore = "slow: circular patterns of up to 24 copies at awkward angles — run with `cargo test -- --ignored`"]
fn circular_patterns_sweep() {
    for (count, angle, joined) in [
        (24, Spacing::extent(360.0), false),
        (7, Spacing::step(37.3), false),
        (5, Spacing::extent(-251.7), false),
        (2, Spacing::step(13.0), true),
        (12, Spacing::extent(360.0), true),
    ] {
        let mut program = Program::new();
        extruded(
            &mut program,
            "b",
            polygon(&[[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]),
            Extents::blind(1.0),
        );
        program.push(
            "r",
            CircularPatternArgs {
                bodies: solid("extrude(b)"),
                axis: axis(FrameAxis::Z),
                reversed: false,
                count: (count as f64).into(),
                angle: angle.clone(),
                combine: if joined {
                    Combine::Union {
                        target: "extrude(b)".into(),
                    }
                } else {
                    Combine::NewBody
                },
            },
        );
        let part = program
            .build::<S>(&NoFiles)
            .unwrap_or_else(|e| panic!("{count} at {angle:?}: {e:?}"));
        assert_valid(&part);
        let solids = part.solid_names();
        assert_eq!(solids.len(), if joined { 1 } else { count }, "{solids:?}");
        if !joined {
            for name in &solids {
                let got = volume(&part, name);
                assert!((got - 0.8).abs() < 1e-6, "{name}: {got}");
            }
        }
    }
}

/// Three boxes, each turned 13° further about `z` than the last, drawn
/// where they are rather than patterned, and joined one after the other —
/// what a circular pattern joined to its seed does.
#[test]
fn three_turned_boxes_joined() {
    let mut program = Program::new();
    for (k, name) in ["a", "b", "c"].into_iter().enumerate() {
        let (s, c) = (13.0 * k as f64).to_radians().sin_cos();
        let corners: Vec<[f64; 2]> = [[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]
            .iter()
            .map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1]])
            .collect();
        extruded(&mut program, name, polygon(&corners), Extents::blind(1.0));
    }
    program.push(
        "ab",
        geop_ops_booleans::BooleanArgs {
            a: "extrude(a)".into(),
            b: "extrude(b)".into(),
            op: geop_ops_booleans::boolean::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_booleans::BooleanArgs {
            a: "boolean(ab)".into(),
            b: "extrude(c)".into(),
            op: geop_ops_booleans::boolean::BooleanOp::Union,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
}

/// The same three boxes, the last a copy of the first turned 26°.
///
/// The second join, (a ∪ b) ∪ c, once failed imprinting c's bottom edge
/// onto b's bottom cap. b's inner top edge lies in c's top plane and ends
/// on c's inner side; the piercing search met c's cap only at that end
/// vertex, and its last box, 1.4e-4 short of the end and 1.4e-5 outside
/// the cap, was split at as a crossing — Newton cannot refine a curve
/// lying in the surface's tangent plane. That left vertices 5e-5 wide, and
/// a pcurve through one too wide for the imprint to tell its ring's area
/// from zero. Drawn boxes passed only because their caps' patches extend
/// past the edge: there it lay on the patch and was imprinted. Only a
/// transversal crossing is now a piercing.
#[test]
fn three_turned_boxes_one_a_turned_copy_joined() {
    let mut program = Program::new();
    for (k, name) in ["a", "b"].into_iter().enumerate() {
        let (s, c) = (13.0 * k as f64).to_radians().sin_cos();
        let corners: Vec<[f64; 2]> = [[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]
            .iter()
            .map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1]])
            .collect();
        extruded(&mut program, name, polygon(&corners), Extents::blind(1.0));
    }
    program.push(
        "c",
        MoveBodyArgs {
            bodies: solid("extrude(a)"),
            translation: [0.0; 3],
            axis: axis(FrameAxis::Z),
            angle: 26.0,
            copy: true,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "ab",
        geop_ops_booleans::BooleanArgs {
            a: "extrude(a)".into(),
            b: "extrude(b)".into(),
            op: geop_ops_booleans::boolean::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_booleans::BooleanArgs {
            a: "boolean(ab)".into(),
            b: "move(c,extrude(a))".into(),
            op: geop_ops_booleans::boolean::BooleanOp::Union,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let (width, edge) = widest_pcurve(&part);
    assert!(width < 1e-9, "{edge}: {width:e}");
}

/// The widest control point of any pcurve of `part`, and the edge it
/// belongs to.
fn widest_pcurve(part: &Part<S>) -> (f64, String) {
    let model = part.topology();
    let mut widest = (0.0, String::new());
    for coedge in model.coedges.values() {
        let width = coedge
            .pcurve
            .control_points
            .iter()
            .flat_map(|p| [p[0], p[1], p[2]])
            .map(|c| c.width().to_f64())
            .fold(0.0, f64::max);
        if width > widest.0 {
            let name = match coedge.geometry {
                geop_core_topology::CoedgeGeometry::Edge(e) => {
                    part.name_of(e).unwrap_or("?").to_string()
                }
                geop_core_topology::CoedgeGeometry::Vertex(v) => {
                    part.name_of(v).unwrap_or("?").to_string()
                }
            };
            widest = (width, name);
        }
    }
    widest
}

/// A box joined to a copy of itself turned 13° about `z`: the edges the
/// join splits keep pcurves as tight as the same join of two boxes drawn
/// where they are.
#[test]
fn turned_copy_joined_stays_tight() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "b",
        polygon(&[[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]),
        Extents::blind(1.0),
    );
    program.push(
        "r",
        CircularPatternArgs {
            bodies: solid("extrude(b)"),
            axis: axis(FrameAxis::Z),
            reversed: false,
            count: 2.0.into(),
            angle: Spacing::step(13.0),
            combine: Combine::Union {
                target: "extrude(b)".into(),
            },
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let (width, edge) = widest_pcurve(&part);
    assert!(width < 1e-9, "{edge}: {width:e}");
}

/// Rows and grids of up to 24 holes cut from a long plate by a patterned
/// pin, and copies of the plate turned at awkward angles about one of its
/// edges and shifted: every result valid, holding what it should.
#[test]
#[ignore = "slow: rows and grids of up to 24 holes, copies turned at awkward angles — run with `cargo test -- --ignored`"]
fn linear_patterns_and_moves_sweep() {
    for (along, across) in [(24, 1), (6, 4), (3, 3)] {
        let mut program = Program::new();
        extruded(
            &mut program,
            "plate",
            polygon(&[[0.0, 0.0], [30.0, 0.0], [30.0, 6.0], [0.0, 6.0]]),
            Extents::blind(0.5),
        );
        let mut through = Extents::blind(2.0);
        through.symmetric = true;
        extruded(&mut program, "pin", circle([1.0, 1.0], 0.3), through);
        program.push(
            "holes",
            LinearPatternArgs {
                bodies: solid("extrude(pin)"),
                first: Direction {
                    along: axis(FrameAxis::X),
                    reversed: false,
                    count: (along as f64).into(),
                    spacing: Spacing::step(1.2),
                },
                second: (across > 1).then(|| Direction {
                    along: axis(FrameAxis::Y),
                    reversed: false,
                    count: (across as f64).into(),
                    spacing: Spacing::extent(1.3 * (across - 1) as f64),
                }),
                combine: Combine::Difference {
                    target: "extrude(plate)".into(),
                },
            },
        );
        let part = program
            .build::<S>(&NoFiles)
            .unwrap_or_else(|e| panic!("{along} x {across}: {e:?}"));
        assert_valid(&part);
        let holes = (along * across) as f64;
        let expected = 30.0 * 6.0 * 0.5 - holes * PI * 0.09 * 0.5;
        let got = volume(&part, "linear_pattern(holes)");
        assert!(
            (got - expected).abs() < 1e-2,
            "{along} x {across}: {got} vs {expected}"
        );
    }
    for degrees in [1.0, 17.3, 89.99, 133.7, 271.1] {
        let mut program = plate_and_pin();
        program.push(
            "mv",
            MoveBodyArgs {
                bodies: solid("extrude(plate)"),
                translation: [0.3, -1.7, 2.2],
                axis: Some(EntityRef::Edge {
                    name: "extrude(plate,plate_sketch,p0)".into(),
                }),
                angle: degrees,
                copy: true,
                combine: Combine::NewBody,
            },
        );
        let part = program
            .build::<S>(&NoFiles)
            .unwrap_or_else(|e| panic!("{degrees}°: {e:?}"));
        assert_valid(&part);
        let got = volume(&part, "move(mv,extrude(plate))");
        assert!((got - 8.0).abs() < 1e-6, "{degrees}°: {got}");
    }
}
