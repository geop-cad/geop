//! Example programs, written in Rust against the operation types.
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use crate::{
    AddDatumArgs, AddSketchArgs, Combine, Construction, EntityRef, ExtrudeArgs, Program,
    RevolveArgs, WorldAxis,
};
use geop_core_sketch::{Constraint, CurveId, PointId, Sketch};

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
fn polygon(sketch: &mut Sketch, corners: &[[f64; 2]]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(c[0], c[1]))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// Solves `sketch`, which the examples all constrain fully.
fn solved(mut sketch: Sketch) -> Sketch {
    let report = sketch.solve().expect("example sketches are valid");
    assert!(
        report.converged,
        "example sketch does not solve: {report:?}"
    );
    sketch
}

/// A `width` x `depth` rectangle with its first corner at `origin`, drawn
/// roughly and fully constrained.
fn rectangle(sketch: &mut Sketch, origin: [f64; 2], width: f64, depth: f64) -> Vec<CurveId> {
    let [x, y] = origin;
    // Deliberately a little off: the constraints decide the shape.
    let (p, l) = polygon(
        sketch,
        &[
            [x + 0.05, y - 0.02],
            [x + width, y + 0.03],
            [x + width - 0.04, y + depth],
            [x, y + depth + 0.01],
        ],
    );
    sketch.constrain(Constraint::Fix { point: p[0], x, y });
    sketch.constrain(Constraint::Horizontal { line: l[0] });
    sketch.constrain(Constraint::Horizontal { line: l[2] });
    sketch.constrain(Constraint::Vertical { line: l[1] });
    sketch.constrain(Constraint::Vertical { line: l[3] });
    sketch.constrain(Constraint::Length {
        curve: l[0],
        value: width,
    });
    sketch.constrain(Constraint::Length {
        curve: l[1],
        value: depth,
    });
    l
}

/// A circle of `radius` around `center`, fully constrained.
fn circle(sketch: &mut Sketch, center: [f64; 2], radius: f64) -> CurveId {
    let c = sketch.add_point(center[0], center[1]);
    let circle = sketch.add_circle(c, radius * 1.1);
    sketch.constrain(Constraint::Fix {
        point: c,
        x: center[0],
        y: center[1],
    });
    sketch.constrain(Constraint::Radius {
        curve: circle,
        value: radius,
    });
    circle
}

/// A 2 x 2 x 1 box with a blind hole drilled into its top: a rectangle
/// sketched on the Z plane and extruded up (`box`), and a circle sketched on
/// the box's end cap `extrude(box,end)` and extruded back into it, cutting
/// it out of the box (`hole`) — the drilled box is `extrude(hole)`.
pub fn box_with_drill_hole() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 2.0, 2.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: EntityRef::Plane {
                normal: WorldAxis::Z,
            },
            sketch: solved(outline),
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: "outline".into(),
            distance: 1.0,
            symmetric: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [1.0, 1.0], 0.4);
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: EntityRef::Face {
                name: "extrude(box,end)".into(),
            },
            sketch: solved(hole),
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            distance: -0.5,
            symmetric: false,
            combine: Combine::Difference {
                target: "extrude(box)".into(),
            },
        },
    );
    program
}

/// A stepped shaft revolved around the world z-axis, cross-drilled through
/// its thinner end: a half section sketched on the X plane, closed by the
/// axis itself (`shaft`), and a circle on the Y plane extruded through both
/// sides of it and cut out of it (`bore`) — the drilled shaft is
/// `extrude(bore)`.
pub fn cross_drilled_shaft() -> Program {
    let mut program = Program::new();

    // Sketch x runs along world y, sketch y along world z.
    let mut section = Sketch::new();
    let (p, l) = polygon(
        &mut section,
        &[
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.6, 1.0],
            [0.6, 3.0],
            [0.0, 3.0],
        ],
    );
    let axis = l[5];
    section.constrain(Constraint::Fix {
        point: p[0],
        x: 0.0,
        y: 0.0,
    });
    section.constrain(Constraint::Vertical { line: axis });
    for (line, horizontal) in [
        (l[0], true),
        (l[1], false),
        (l[2], true),
        (l[3], false),
        (l[4], true),
    ] {
        section.constrain(if horizontal {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
    section.constrain(Constraint::Length {
        curve: l[0],
        value: 1.0,
    });
    section.constrain(Constraint::Length {
        curve: l[1],
        value: 1.0,
    });
    section.constrain(Constraint::Length {
        curve: l[3],
        value: 2.0,
    });
    section.constrain(Constraint::Length {
        curve: l[4],
        value: 0.6,
    });
    program.push(
        "section",
        AddSketchArgs {
            plane: EntityRef::Plane {
                normal: WorldAxis::X,
            },
            sketch: solved(section),
        },
    );
    program.push(
        "shaft",
        RevolveArgs {
            sketch: "section".into(),
            axis,
            combine: Combine::NewBody,
        },
    );

    // Sketch x runs along world x, sketch y along world -z: a bore across
    // the thin end, at z = 2.2.
    let mut bore = Sketch::new();
    circle(&mut bore, [0.0, -2.2], 0.25);
    program.push(
        "bore_sketch",
        AddSketchArgs {
            plane: EntityRef::Plane {
                normal: WorldAxis::Y,
            },
            sketch: solved(bore),
        },
    );
    program.push(
        "bore",
        ExtrudeArgs {
            sketch: "bore_sketch".into(),
            distance: 3.0,
            symmetric: true,
            combine: Combine::Difference {
                target: "revolve(shaft)".into(),
            },
        },
    );
    program
}

/// Two separate plates from one sketch of two regions, one with a round
/// hole, extruded symmetrically into a single solid of two shells.
pub fn two_plates() -> Program {
    let mut program = Program::new();
    let mut plates = Sketch::new();
    rectangle(&mut plates, [0.0, 0.0], 1.5, 1.0);
    rectangle(&mut plates, [2.0, 0.0], 1.0, 1.0);
    circle(&mut plates, [0.75, 0.5], 0.3);
    program.push(
        "plates_sketch",
        AddSketchArgs {
            plane: EntityRef::Plane {
                normal: WorldAxis::Z,
            },
            sketch: solved(plates),
        },
    );
    program.push(
        "plates",
        ExtrudeArgs {
            sketch: "plates_sketch".into(),
            distance: 0.25,
            symmetric: true,
            combine: Combine::NewBody,
        },
    );
    program
}

/// A box with a round boss standing on it, sketched on a reference plane:
/// the plane half a unit above the box's top (`lifted`, offset from
/// `extrude(box,end)`), a circle sketched on it, and extruded back down
/// through the gap and into the box, joined to it (`boss`) — the part is
/// `extrude(boss)`.
pub fn boss_on_reference_plane() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 2.0, 2.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: EntityRef::Plane {
                normal: WorldAxis::Z,
            },
            sketch: solved(outline),
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: "outline".into(),
            distance: 1.0,
            symmetric: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "lifted",
        AddDatumArgs {
            selection: vec![EntityRef::Face {
                name: "extrude(box,end)".into(),
            }],
            construction: Construction::Offset { distance: 0.5 },
        },
    );
    let mut boss = Sketch::new();
    circle(&mut boss, [1.0, 1.0], 0.5);
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: EntityRef::Datum {
                name: "lifted".into(),
            },
            sketch: solved(boss),
        },
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: "boss_sketch".into(),
            distance: -0.75,
            symmetric: false,
            combine: Combine::Union {
                target: "extrude(box)".into(),
            },
        },
    );
    program
}

/// Every example, by name.
pub fn all() -> Vec<(&'static str, Program)> {
    vec![
        ("box_with_drill_hole", box_with_drill_hole()),
        ("cross_drilled_shaft", cross_drilled_shaft()),
        ("two_plates", two_plates()),
        ("boss_on_reference_plane", boss_on_reference_plane()),
    ]
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
    use geop_core_part::{Part, PartDescription, RefId};
    use geop_core_topology::{
        contains::shell::{PointClassification, shell_contains},
        validation::{ValidationParameters, validate},
    };

    use super::*;

    fn outputs_dir() -> std::path::PathBuf {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../outputs/parts");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Every named entity's exact geometry, by name: vertex points, edge
    /// curves and face surfaces as their full `Debug` enclosures — equal only
    /// if the two parts agree to the last bit of every interval.
    fn geometry_by_name(part: &Part<S>) -> BTreeMap<String, String> {
        let model = part.topology();
        part.names()
            .iter()
            .filter_map(|(id, name)| {
                let geometry = match id {
                    RefId::Vertex(v) => format!("{:?}", model.get_vertex(v).ok()?.point),
                    RefId::Edge(e) => format!("{:?}", model.get_edge(e).ok()?.curve),
                    RefId::Face(f) => format!("{:?}", model.get_face(f).ok()?.surface),
                    RefId::Solid(_) | RefId::Sketch(_) | RefId::Datum(_) => return None,
                };
                Some((name.to_string(), geometry))
            })
            .collect()
    }

    /// Builds `program`, writes it and a description of the part it builds
    /// to `outputs/parts/`, reads the program back from its JSON, and
    /// requires the read-back program to be the same program and to build
    /// the very same part: every name, every piece of topology between
    /// names, and every bit of geometry.
    fn build_and_round_trip(name: &str, program: &Program) -> Part<S> {
        let part = program.apply(Part::<S>::new()).unwrap();
        let validation = ValidationParameters::default();
        if let Err(errors) = validate(&validation, part.topology()) {
            panic!(
                "{name}: {} validation error(s): {}",
                errors.len(),
                errors[0]
            );
        }

        let json = program.to_json().unwrap();
        let dir = outputs_dir();
        std::fs::write(dir.join(format!("{name}.program.json")), &json).unwrap();
        let description = PartDescription::of(&part).unwrap();
        std::fs::write(
            dir.join(format!("{name}.part.json")),
            serde_json::to_string_pretty(&description).unwrap(),
        )
        .unwrap();
        geop_ops_rasterize::rasterize_model(part.topology(), 16)
            .unwrap()
            .save_to_file(dir.join(format!("{name}.html")).to_str().unwrap())
            .unwrap();

        let read_back = Program::from_json(&json).unwrap();
        assert_eq!(
            &read_back, program,
            "{name}: JSON round trip changed the program"
        );
        assert_eq!(
            read_back.to_json().unwrap(),
            json,
            "{name}: JSON is not stable"
        );

        let rebuilt = read_back.apply(Part::<S>::new()).unwrap();
        assert_eq!(
            PartDescription::of(&rebuilt).unwrap(),
            description,
            "{name}: the read-back program built a different part"
        );
        assert_eq!(
            geometry_by_name(&rebuilt),
            geometry_by_name(&part),
            "{name}: the read-back program built different geometry"
        );
        part
    }

    fn inside(part: &Part<S>, solid: &str, p: [f64; 3]) -> PointClassification {
        let model = part.topology();
        let solid = part.solid_id(solid).unwrap();
        let point = Vector3::from_array(p.map(S::from_f64));
        let mut result = PointClassification::Outside;
        for &shell in &model.get_solid(solid).unwrap().shells {
            match shell_contains(model, shell, point, 2000, S::from_f64(1e-6), 7).unwrap() {
                PointClassification::Outside => {}
                other => result = other,
            }
        }
        result
    }

    #[test]
    fn box_with_drill_hole_round_trips() {
        let part = build_and_round_trip("box_with_drill_hole", &box_with_drill_hole());
        let description = PartDescription::of(&part).unwrap();

        // One solid, named after the step that cut the hole.
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(hole)"]
        );
        // The box's top keeps its name, and now has the hole in it.
        let top = &description.faces["extrude(box,end)"];
        assert_eq!(top.holes.len(), 1, "{top:?}");
        // The hole's bottom is the hole tool's end cap; its wall is the
        // four quarters swept by the circle.
        assert!(description.faces.contains_key("extrude(hole,end)"));
        let circle = box_with_drill_hole().steps[2].clone();
        let crate::PartOperation::AddSketch(args) = circle.operation else {
            unreachable!()
        };
        let circle_id = *args.sketch.curves.keys().next().unwrap();
        for piece in ["", "#1", "#2", "#3"] {
            let wall = format!("extrude(hole,hole_sketch,{circle_id}{piece})");
            assert!(description.faces.contains_key(&wall), "no face {wall}");
        }

        assert_eq!(
            inside(&part, "extrude(hole)", [0.3, 0.3, 0.5]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(hole)", [1.0, 1.0, 0.8]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(hole)", [1.0, 1.0, 0.3]),
            PointClassification::Inside
        );
    }

    #[test]
    fn cross_drilled_shaft_round_trips() {
        let part = build_and_round_trip("cross_drilled_shaft", &cross_drilled_shaft());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(bore)"]
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.8, 0.5]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.0, 2.2]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.0, 2.7]),
            PointClassification::Inside
        );
    }

    #[test]
    fn two_plates_round_trip() {
        let part = build_and_round_trip("two_plates", &two_plates());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids["extrude(plates)"].len(),
            2,
            "one shell per plate"
        );
        assert_eq!(
            inside(&part, "extrude(plates)", [0.2, 0.2, 0.0]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(plates)", [0.75, 0.5, 0.0]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(plates)", [2.5, 0.5, 0.1]),
            PointClassification::Inside
        );
    }

    #[test]
    fn boss_on_reference_plane_round_trips() {
        let part = build_and_round_trip("boss_on_reference_plane", &boss_on_reference_plane());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(boss)"]
        );
        assert_eq!(description.datums, ["lifted"]);
        // The boss stands on the box: from the box's top up to the plane.
        assert_eq!(
            inside(&part, "extrude(boss)", [1.0, 1.0, 1.3]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [1.0, 1.0, 1.6]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [0.2, 0.2, 1.3]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [0.2, 0.2, 0.5]),
            PointClassification::Inside
        );
    }

    /// The names of a program's entities don't depend on its numbers: a
    /// taller box with a wider hole has the very same names.
    #[test]
    fn names_survive_a_change_of_dimensions() {
        let names = |program: &Program| {
            let part = program.apply(Part::<S>::new()).unwrap();
            let description = PartDescription::of(&part).unwrap();
            (
                description.faces.keys().cloned().collect::<Vec<_>>(),
                description.edges.keys().cloned().collect::<Vec<_>>(),
                description.vertices.keys().cloned().collect::<Vec<_>>(),
            )
        };
        let original = box_with_drill_hole();
        let mut edited = original.clone();
        for step in &mut edited.steps {
            match &mut step.operation {
                crate::PartOperation::Extrude(args) if step.id == "box" => args.distance = 1.5,
                crate::PartOperation::AddSketch(args) if step.id == "hole_sketch" => {
                    for c in args.sketch.constraints.values_mut() {
                        if let Constraint::Radius { value, .. } = c {
                            *value = 0.6;
                        }
                    }
                    args.sketch.solve().unwrap();
                }
                _ => {}
            }
        }
        assert_ne!(original, edited);
        assert_eq!(names(&edited), names(&original));
    }

    /// An unknown name is reported, not guessed at.
    #[test]
    fn referring_to_a_missing_entity_fails() {
        let mut program = box_with_drill_hole();
        let crate::PartOperation::AddSketch(args) = &mut program.steps[2].operation else {
            unreachable!()
        };
        args.plane = EntityRef::Face {
            name: "extrude(box,side)".into(),
        };
        let Err(err) = program.apply(Part::<S>::new()) else {
            panic!("a sketch on a face that doesn't exist was placed somewhere");
        };
        assert!(format!("{err:?}").contains("extrude(box,side)"), "{err:?}");
    }

    /// Step ids have to be unique: every name a step creates is built from
    /// its id.
    #[test]
    fn duplicate_step_ids_are_rejected() {
        let mut program = box_with_drill_hole();
        program.steps[3].id = "box".into();
        assert!(program.apply(Part::<S>::new()).is_err());
    }
}
