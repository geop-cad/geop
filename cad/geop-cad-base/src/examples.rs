//! Example programs, written in Rust against the operation types.
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_sketch::{Constraint, CurveId, PointId, Sketch, point::P2};
use geop_ops::{EntityRef, ORIGIN};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{ExtrudeArgs, RevolveArgs};
use geop_ops_sketch::AddSketchArgs;

use crate::Program;

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
fn polygon(sketch: &mut Sketch, corners: &[P2]) -> (Vec<PointId>, Vec<CurveId>) {
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
fn rectangle(sketch: &mut Sketch, origin: P2, width: f64, depth: f64) -> Vec<CurveId> {
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
fn circle(sketch: &mut Sketch, center: P2, radius: f64) -> CurveId {
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
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
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

/// A 40 x 40 x 10 mm bracket with a 6 mm hole through its middle: a
/// rectangle sketched on the Z plane and extruded up (`block`), and a
/// circle sketched on the block's end cap and extruded back through its
/// full thickness, cutting it out of the block (`hole`). Unlike
/// `box_with_drill_hole`, the cut ends exactly on the bottom face, so the
/// hole goes all the way through. The program the landing page's agent demo
/// transcript writes, step for step.
pub fn bracket() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 40.0, 40.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            sketch: solved(outline),
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "outline".into(),
            distance: 10.0,
            symmetric: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [20.0, 20.0], 3.0);
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: EntityRef::Face {
                name: "extrude(block,end)".into(),
            },
            sketch: solved(hole),
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            distance: -10.0,
            symmetric: false,
            combine: Combine::Difference {
                target: "extrude(block)".into(),
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
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::X)),
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
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
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
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
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
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
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
            plane: EntityRef::datum("lifted"),
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

/// A hand-drawn handle-shaped outline (an irregular blob, splined, with a
/// smaller splined region inside it — as a free-hand sketch in the web
/// editor draws, unconstrained) extruded, then cross-drilled with a
/// circular hole near one end.
///
/// Reproduces a real user bug report (2026-09-27) with the exact reported
/// coordinates, unconstrained, rather than a re-derived "clean" equivalent
/// that might not carry the same numerical case at all: the hole's boolean
/// difference failed with a degenerate-face error from
/// `face_interior_point`. (A second report, of the same outline with a
/// slightly different hole center, left a non-watertight hole instead; its
/// coordinates were not recorded.)
pub fn handle_with_hole() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let outer_points = [
        (0.01007682018456503, 0.8939399106232252),
        (-0.9693901017551558, 0.8213868052943569),
        (-1.00566665441959, 0.3175457960661055),
        (-0.9452057333121998, -0.3555857922628385),
        (0.09472210973491128, -0.9481028191152622),
        (0.695300592734987, -0.3878316168534466),
        (1.384555093359235, -0.4523232660346628),
        (1.6233271012047248, -0.9224957048864859),
        (1.9214381194506664, 0.15401630544608172),
        (1.2092840203075834, 0.8578895429712221),
    ]
    .map(|(x, y)| outline.add_point(x, y));
    let mut outer_loop = outer_points.to_vec();
    outer_loop.push(outer_points[0]);
    outline.add_spline(outer_loop);

    let inner_points = [
        (0.27854547786607586, 0.568500810276078),
        (-0.3241022513094386, 0.561767316095346),
        (-0.3645032163938306, 0.00962079327532167),
        (0.03950643445008967, -0.4246895813818926),
        (0.507484280010964, -0.11831559615858644),
        (0.8643594715897602, 0.12072344725739975),
        (1.1202322504575766, -0.1452495728815144),
        (1.2919363520662426, 0.18805838906471978),
        (1.0427970673791584, 0.470865144655464),
    ]
    .map(|(x, y)| outline.add_point(x, y));
    let mut inner_loop = inner_points.to_vec();
    inner_loop.push(inner_points[0]);
    outline.add_spline(inner_loop);

    program.push(
        "outline",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            sketch: solved(outline),
        },
    );
    program.push(
        "handle",
        ExtrudeArgs {
            sketch: "outline".into(),
            distance: 1.0,
            symmetric: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    let center = hole.add_point(0.32989396295411244, -0.5632891678453777);
    hole.add_circle(center, 0.31727744879927977);
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            sketch: solved(hole),
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            distance: 2.44,
            symmetric: true,
            combine: Combine::Difference {
                target: "extrude(handle)".into(),
            },
        },
    );
    program
}

/// A rounded-top luggage tag: two vertical sides and a horizontal bottom,
/// closed by a semicircular top tangent to both sides, with a hang-hole
/// through the top (concentric with it) and a rectangular window in the
/// body, inset 0.15 from each of the two sides and the bottom.
pub fn luggage_tag() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let p0 = outline.add_point(-0.7051397478635735, 0.7809633733011653);
    let p1 = outline.add_point(-0.7051397478646341, -0.6729821470985564);
    let p4 = outline.add_point(0.695311597099524, -0.6729821470973941);
    let p7 = outline.add_point(0.695311597101934, 0.7809633732995341);
    let left = outline.add_line(p0, p1);
    let bottom = outline.add_line(p1, p4);
    let right = outline.add_line(p4, p7);
    let top = outline.add_arc_with_sweep(p7, p0, std::f64::consts::PI);
    outline.constrain(Constraint::Vertical { line: left });
    outline.constrain(Constraint::Horizontal { line: bottom });
    outline.constrain(Constraint::Vertical { line: right });
    outline.constrain(Constraint::Tangent { a: top, b: right });
    outline.constrain(Constraint::Tangent { a: top, b: left });

    let hole_center = outline.add_point(-0.004914075380876311, 0.7809633733016882);
    let hole = outline.add_circle(hole_center, 0.30711303824557856);
    outline.constrain(Constraint::Concentric { a: hole, b: top });

    let w_tl = outline.add_point(-0.5551397478558091, 0.08482972707824256);
    let w_tr = outline.add_point(0.5453115971039223, 0.08482972707824256);
    let w_br = outline.add_point(0.5453115971014348, -0.5229821471055477);
    let w_bl = outline.add_point(-0.5551397478598076, -0.5229821471049918);
    let w_top = outline.add_line(w_tl, w_tr);
    let w_right = outline.add_line(w_tr, w_br);
    let w_bottom = outline.add_line(w_br, w_bl);
    let w_left = outline.add_line(w_bl, w_tl);
    outline.constrain(Constraint::Horizontal { line: w_top });
    outline.constrain(Constraint::Vertical { line: w_right });
    outline.constrain(Constraint::Horizontal { line: w_bottom });
    outline.constrain(Constraint::Vertical { line: w_left });
    outline.constrain(Constraint::PointLineDistance {
        point: w_br,
        line: right,
        value: 0.15,
    });
    outline.constrain(Constraint::PointLineDistance {
        point: w_bl,
        line: left,
        value: 0.15,
    });
    outline.constrain(Constraint::PointLineDistance {
        point: w_br,
        line: bottom,
        value: 0.15,
    });

    program.push(
        "outline",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            sketch: solved(outline),
        },
    );
    program.push(
        "tag",
        ExtrudeArgs {
            sketch: "outline".into(),
            distance: 1.0 / 3.0,
            symmetric: true,
            combine: Combine::NewBody,
        },
    );
    program
}

/// A box with a round hole through it, and a triangle sketched on its side
/// face, revolved around one of its own edges and joined to the box: a double
/// cone whose axis lies in the side face, so that both apexes sit on it and
/// half the cone stands out of the box.
///
/// Reproduces a real user bug report (2026-09-30) with the exact reported
/// coordinates and sketch ids: the `revolve1` union failed in
/// `remesh_edges_x_faces` with "the spokes ... are not coplanar: an apex has
/// no single normal" from `NurbSurface::pole_normal`: `trace_one_side` asked
/// for the cone's normal at its apex, to find which way a curve leaves it.
pub fn revolved_cone_on_box() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let p0 = outline.add_point(-2.046526714311965, 1.182776884816516);
    let p1 = outline.add_point(2.2456482324612383, -1.2774916901086213);
    let p2 = outline.add_point(2.2456482324612383, 1.182776884816516);
    let p3 = outline.add_point(-2.046526714311965, -1.2774916901086213);
    let top = outline.add_line(p0, p2);
    outline.constrain(Constraint::Horizontal { line: top });
    let right = outline.add_line(p2, p1);
    outline.constrain(Constraint::Vertical { line: right });
    let bottom = outline.add_line(p1, p3);
    outline.constrain(Constraint::Horizontal { line: bottom });
    let left = outline.add_line(p3, p0);
    outline.constrain(Constraint::Vertical { line: left });
    let center = outline.add_point(-0.6692695471128323, 0.04905776553659026);
    outline.add_circle(center, 0.771466957205246);
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            sketch: outline,
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            distance: 1.8900000000000001,
            symmetric: false,
            combine: Combine::NewBody,
        },
    );

    let mut triangle = Sketch::new();
    let q0 = triangle.add_point(-0.2654953575323612, 1.4039697276195047);
    let q1 = triangle.add_point(-0.8683910652620968, 0.8073854454124275);
    let axis = triangle.add_line(q0, q1);
    let q3 = triangle.add_point(1.9210929721682684, 0.8073854454124275);
    let base = triangle.add_line(q1, q3);
    triangle.constrain(Constraint::Horizontal { line: base });
    triangle.add_line(q3, q0);
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: EntityRef::Face {
                name: format!("extrude(extrude1,sketch1,{right})"),
            },
            sketch: triangle,
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: "sketch2".into(),
            axis,
            combine: Combine::Union {
                target: "extrude(extrude1)".into(),
            },
        },
    );
    program
}

/// Every example, by name.
pub fn all() -> Vec<(&'static str, Program)> {
    vec![
        ("box_with_drill_hole", box_with_drill_hole()),
        ("bracket", bracket()),
        ("cross_drilled_shaft", cross_drilled_shaft()),
        ("two_plates", two_plates()),
        ("boss_on_reference_plane", boss_on_reference_plane()),
        ("handle_with_hole", handle_with_hole()),
        ("luggage_tag", luggage_tag()),
        ("revolved_cone_on_box", revolved_cone_on_box()),
    ]
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
    use geop_core_topology::{
        contains::shell::{PointClassification, shell_contains},
        validation::{ValidationParameters, validate},
    };
    use geop_ops::{Part, PartDescription, RefId};

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
        let part = program.build::<S>().unwrap();
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
        geop_ops_rasterize::rasterize(part.topology(), 16)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue)
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

        let rebuilt = read_back.build::<S>().unwrap();
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
        assert_eq!(description.datums, ["lifted", "origin"]);
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
    /// The hole goes all the way through: both the top and the bottom face
    /// carry it as a hole, it has no bottom of its own, and its axis is
    /// outside the solid at every height.
    #[test]
    fn bracket_round_trips() {
        let part = build_and_round_trip("bracket", &bracket());
        let description = PartDescription::of(&part).unwrap();

        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(hole)"]
        );
        for cap in ["extrude(block,start)", "extrude(block,end)"] {
            let face = &description.faces[cap];
            assert_eq!(face.holes.len(), 1, "{cap}: {face:?}");
        }
        assert!(!description.faces.contains_key("extrude(hole,end)"));
        for z in [1.0, 5.0, 9.0] {
            assert_eq!(
                inside(&part, "extrude(hole)", [20.0, 20.0, z]),
                PointClassification::Outside,
                "z = {z}"
            );
        }
        assert_eq!(
            inside(&part, "extrude(hole)", [5.0, 5.0, 5.0]),
            PointClassification::Inside
        );
    }

    #[test]
    fn names_survive_a_change_of_dimensions() {
        let names = |program: &Program| {
            let part = program.build::<S>().unwrap();
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
        let Err(err) = program.build::<S>() else {
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
        assert!(program.build::<S>().is_err());
    }

    /// Reproduces the real bug report described on `handle_with_hole`.
    ///
    /// Used to fail in the "hole" step's `Difference`, with
    /// `face_interior_point` finding no interior point on a side wall of the
    /// inner spline. The wall's loop carried a spur whose pcurve ran across
    /// the whole face: `fit_pcurve` seeded its first Newton projection from
    /// the patch's parametric middle, and on this strongly curved wall that
    /// converged to a foot point clamped against the far domain bound. The
    /// walk is now seeded where the curve actually starts on the surface.
    #[test]
    fn handle_with_hole_round_trips() {
        build_and_round_trip("handle_with_hole", &handle_with_hole());
    }

    #[test]
    fn luggage_tag_round_trips() {
        build_and_round_trip("luggage_tag", &luggage_tag());
    }

    /// Reproduces the real bug report described on `revolved_cone_on_box`.
    #[test]
    fn revolved_cone_on_box_round_trips() {
        let part = build_and_round_trip("revolved_cone_on_box", &revolved_cone_on_box());
        // The side face is the plane x = 2.2456, with sketch x along world y
        // and sketch y along world z. The triangle's centroid, turned a
        // quarter around the axis, lies 0.654 off that plane on either side.
        for (p, expected) in [
            ([2.9, -0.198, 1.471], PointClassification::Inside),
            ([1.59, -0.198, 1.471], PointClassification::Inside),
            ([2.9, 1.0, 0.3], PointClassification::Outside),
            ([-0.669, 0.049, 1.0], PointClassification::Outside),
            ([1.5, -1.0, 1.0], PointClassification::Inside),
        ] {
            assert_eq!(inside(&part, "revolve(revolve1)", p), expected, "{p:?}");
        }
    }
}
