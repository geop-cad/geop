//! Example programs, written in Rust against the operation types.
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::Program, values::Design};
use geop_core_sketch::{CurveId, PointId};
use geop_lib_sweep::Orientation;
use geop_ops_boolean::SplitArgs;
use geop_ops_datums::{ORIGIN, component_name, datum, origin_axis, origin_plane};
use geop_ops_extrude_revolve::{
    Extent, Extents, ExtrudeArgs, LoftArgs, Output, PathRef, SweepArgs,
};
use geop_ops_hole::{HoleArgs, HoleCentre, HoleKind, Standard, iso::Fit};
use geop_ops_model::element;
use geop_ops_model::{Combine, solid};
use geop_ops_pattern::{Direction, LinearPatternArgs, PatternDirection, Seeds, Spacing, feature};
use geop_ops_reference_geometry::{AddDatumArgs, Construction, Selected};
use geop_ops_sketch::{AddSketchArgs, Constraint, Sketch, SketchPlane, sketch, whole_sketch};
use geop_ops_sketch3d::{AddSketch3dArgs, CurveSource, Sketch3d};
use geop_ops_surface::NetworkSurfaceArgs;

use geop_core_editor::registry::{Example, ExampleSet};

/// `[x, y]`, as an example is written down.
type P2 = [f64; 2];

/// A number of an example, as design data.
pub(crate) fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// The flat face `name` of the part, as a plane to sketch on.
fn on_face(name: impl Into<String>) -> SketchPlane {
    SketchPlane::Face(element(name))
}

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// A pose of an example: at `position`, turned by `degrees` (see
/// [`Pose::from_euler`]).
#[allow(
    dead_code,
    reason = "pruned with the other shared helpers once every example has moved"
)]
pub(crate) fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n))
        .expect("an example's rotation is one")
}

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
pub(crate) fn polygon(sketch: &mut Sketch, corners: &[P2]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// Solves `sketch`, which the examples all constrain fully.
pub(crate) fn solved(mut sketch: Sketch) -> Sketch {
    let report = sketch.solve().expect("example sketches are valid");
    assert!(
        report.converged,
        "example sketch does not solve: {report:?}"
    );
    sketch
}

/// A `width` x `depth` rectangle with its first corner at `origin`, drawn
/// roughly and fully constrained.
pub(crate) fn rectangle(sketch: &mut Sketch, origin: P2, width: f64, depth: f64) -> Vec<CurveId> {
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
    sketch.constrain(Constraint::Fix {
        point: p[0],
        x: n(x),
        y: n(y),
    });
    sketch.constrain(Constraint::Horizontal { line: l[0] });
    sketch.constrain(Constraint::Horizontal { line: l[2] });
    sketch.constrain(Constraint::Vertical { line: l[1] });
    sketch.constrain(Constraint::Vertical { line: l[3] });
    sketch.constrain(Constraint::Length {
        curve: l[0],
        value: n(width),
    });
    sketch.constrain(Constraint::Length {
        curve: l[1],
        value: n(depth),
    });
    l
}

/// A circle of `radius` around `center`, fully constrained.
pub(crate) fn circle(sketch: &mut Sketch, center: P2, radius: f64) -> CurveId {
    let c = sketch.add_point(n(center[0]), n(center[1]));
    let circle = sketch.add_circle(c, n(radius * 1.1));
    sketch.constrain(Constraint::Fix {
        point: c,
        x: n(center[0]),
        y: n(center[1]),
    });
    sketch.constrain(Constraint::Radius {
        curve: circle,
        value: n(radius),
    });
    circle
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
            plane: Some(on_origin(FrameAxis::Z)),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(1.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push(
        "lifted",
        AddDatumArgs {
            selection: vec![Selected::Face(element("extrude(box,end)"))],
            construction: Construction::Offset {
                distance: 0.5.into(),
            },
        },
    );
    let mut boss = Sketch::new();
    circle(&mut boss, [1.0, 1.0], 0.5);
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(datum("lifted"))),
            sketch: solved(boss),
            ..Default::default()
        },
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: whole_sketch("boss_sketch"),
            extent: Extents::blind(-0.75),
            output: Output::Solid(Combine::join("extrude(box)")),
        },
    );
    program
}

/// An example drawn in the editor, kept as the program it saved (in
/// `src/drawn/`): its sketches as they were drawn, which no code here says
/// better. Its tests build it like every other example, so a change to the
/// format that breaks it is found there: save it again from the editor.
fn drawn(text: &str) -> Program {
    Program::from_json(text).expect("a drawn example is a program")
}

/// Every example, as the editor offers them (see [`all`]).
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: all(),
        workspaces: Vec::new(),
    }
}

/// The NACA 2412 section, chord 1 from its leading edge at the origin to
/// its trailing edge at `(1, 0)`: the upper side from the trailing edge to
/// the leading edge, then the lower side back — points of the four-digit
/// formula, cosine spaced, used as the control points of a spline each.
const NACA_2412_UPPER: [P2; 15] = [
    [1.0, 0.0],
    [0.9876, 0.0026],
    [0.9509, 0.0101],
    [0.8917, 0.0214],
    [0.8129, 0.0349],
    [0.7182, 0.049],
    [0.6123, 0.062],
    [0.5006, 0.0723],
    [0.3886, 0.0784],
    [0.2813, 0.0782],
    [0.1853, 0.071],
    [0.1056, 0.0576],
    [0.0464, 0.0399],
    [0.0107, 0.0201],
    [0.0, 0.0],
];
const NACA_2412_LOWER: [P2; 15] = [
    [0.0, 0.0],
    [0.0144, -0.0176],
    [0.0526, -0.0306],
    [0.1126, -0.0387],
    [0.1912, -0.0422],
    [0.2848, -0.0416],
    [0.3889, -0.0384],
    [0.4994, -0.0334],
    [0.6102, -0.027],
    [0.7157, -0.0202],
    [0.8106, -0.0138],
    [0.8901, -0.0082],
    [0.9501, -0.0038],
    [0.9873, -0.001],
    [1.0, 0.0],
];

/// The NACA 2412 section of chord `chord`, its leading edge at `(x, 0)`,
/// drawn in a plane of the origin's `y` axis — which draws in `(x, -z)`,
/// so the section is drawn upside down to stand the right way up.
fn naca_2412(chord: f64, x: f64) -> Sketch {
    let mut s = Sketch::new();
    let point = |s: &mut Sketch, p: P2| s.add_point(n(x + chord * p[0]), n(-chord * p[1]));
    let trailing = point(&mut s, NACA_2412_UPPER[0]);
    let leading = point(&mut s, NACA_2412_LOWER[0]);
    let side = |s: &mut Sketch, points: &[P2], first: PointId, last: PointId| {
        let mut control = vec![first];
        for &p in &points[1..points.len() - 1] {
            control.push(point(s, p));
        }
        control.push(last);
        s.add_spline(control);
    };
    side(&mut s, &NACA_2412_UPPER, trailing, leading);
    side(&mut s, &NACA_2412_LOWER, leading, trailing);
    s
}

/// A wing: the NACA 2412 section, chord 1 at the root, lofted into the
/// same section of chord 0.6 at the tip, 3 out along `y` and swept back
/// 0.3.
pub fn airfoil_wing() -> Program {
    let mut program = Program::new();
    program.push(
        "root",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::Y)),
            sketch: naca_2412(1.0, 0.0),
            ..Default::default()
        },
    );
    program.push(
        "span",
        AddDatumArgs {
            selection: vec![Selected::Datum(datum(component_name(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )))],
            construction: Construction::Offset {
                distance: 3.0.into(),
            },
        },
    );
    program.push(
        "tip",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(datum("span"))),
            sketch: naca_2412(0.6, 0.3),
            ..Default::default()
        },
    );
    program.push(
        "wing",
        LoftArgs {
            profiles: vec![whole_sketch("root"), whole_sketch("tip")],
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// A horn: a circle swept along a straight path, its size set by a rail —
/// a spline flaring out beside the path, which the circle's point on it
/// follows.
pub fn horn() -> Program {
    let base = |axis| Some(on_origin(axis));
    let mut program = Program::new();
    let mut mouth = Sketch::new();
    let c = mouth.add_point(n(0.0), n(0.0));
    mouth.add_circle(c, n(0.5));
    program.push(
        "mouth",
        AddSketchArgs {
            plane: base(FrameAxis::X),
            sketch: mouth,
            ..Default::default()
        },
    );
    let mut axis = Sketch::new();
    let (a, b) = (
        axis.add_point(n(0.0), n(0.0)),
        axis.add_point(n(4.0), n(0.0)),
    );
    axis.add_line(a, b);
    program.push(
        "axis",
        AddSketchArgs {
            plane: base(FrameAxis::Z),
            sketch: axis,
            ..Default::default()
        },
    );
    let mut flare = Sketch::new();
    let points =
        [[0.0, 0.5], [1.5, 0.4], [3.0, 0.9], [4.0, 1.6]].map(|[x, y]| flare.add_point(n(x), n(y)));
    flare.add_spline(points.to_vec());
    program.push(
        "flare",
        AddSketchArgs {
            plane: base(FrameAxis::Z),
            sketch: flare,
            ..Default::default()
        },
    );
    program.push(
        "horn",
        SweepArgs {
            profile: whole_sketch("mouth"),
            path: Some(PathRef::Sketch(ElementReference::whole(sketch("axis")))),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: vec![PathRef::Sketch(ElementReference::whole(sketch("flare")))],
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// An 8 x 3 x 0.5 plate with a row of four holes: a circle sketched on the
/// Z plane and cut through the plate (`hole`), and that feature done again
/// three more times 2 apart along `x` (`holes`) — the drilled plate is
/// `linear_pattern(holes)`, and the holes follow the hole when it changes.
pub fn patterned_plate() -> Program {
    let mut program = Program::new();
    let z_plane = || on_origin(FrameAxis::Z);
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 8.0, 3.0);
    program.push("outline", AddSketchArgs::on(z_plane(), solved(outline)));
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(0.5),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let mut hole = Sketch::new();
    circle(&mut hole, [1.0, 1.5], 0.3);
    program.push("hole_sketch", AddSketchArgs::on(z_plane(), solved(hole)));
    let mut through = Extents::blind(2.0);
    through.symmetric = true;
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: through,
            output: Output::Solid(Combine::cut("extrude(plate)")),
        },
    );
    program.push(
        "holes",
        LinearPatternArgs {
            seeds: Seeds::features(vec![feature("hole")]),
            first: Direction {
                along: Some(PatternDirection::Datum(origin_axis(FrameAxis::X))),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(2.0),
            },
            second: None,
        },
    );
    program
}

/// A 60 x 40 x 10 mm mounting plate: four counterbored M5 holes through
/// its corners, at the points of one sketch on its top (`bolts`), and a
/// tapped M6 hole 8 deep in its middle (`tapped`), which carries the
/// cosmetic thread M6x1 — the result is `hole(tapped)`.
pub fn hole_plate() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 60.0, 40.0);
    program.push(
        "outline",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(10.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let points = |at: &[P2]| {
        let mut sketch = Sketch::new();
        let ids: Vec<PointId> = at
            .iter()
            .map(|p| sketch.add_point(n(p[0]), n(p[1])))
            .collect();
        for (id, p) in ids.into_iter().zip(at) {
            sketch.constrain(Constraint::Fix {
                point: id,
                x: n(p[0]),
                y: n(p[1]),
            });
        }
        solved(sketch)
    };
    let top = || on_face("extrude(plate,end)");
    program.push(
        "corners",
        AddSketchArgs::on(
            top(),
            points(&[[8.0, 8.0], [52.0, 8.0], [52.0, 32.0], [8.0, 32.0]]),
        ),
    );
    program.push(
        "bolts",
        HoleArgs {
            face: Some(element("extrude(plate,end)")),
            points: vec![HoleCentre::Sketch(whole_sketch("corners"))],
            kind: HoleKind::Counterbore,
            standard: Standard::Iso {
                size: "M5".into(),
                fit: Fit::Normal,
            },
            end: Extent::ThroughAll,
            drill_point: false,
            thread_length: None,
        },
    );
    program.push("middle", AddSketchArgs::on(top(), points(&[[30.0, 20.0]])));
    program.push(
        "tapped",
        HoleArgs {
            face: Some(element("extrude(plate,end)")),
            points: vec![HoleCentre::Sketch(whole_sketch("middle"))],
            kind: HoleKind::Tapped,
            standard: Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
            end: Extent::blind(8.0),
            drill_point: true,
            thread_length: None,
        },
    );
    program
}

/// A motor flange 30 across and 2.5 thick, drawn the way the sketch tools
/// draw it: its bore an offset of its rim, 10 in, so the wall follows the
/// rim; and its bolt circle one hole patterned six times round its middle,
/// so the copies follow the first hole.
pub fn motor_flange() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    let middle = outline.add_point(n(0.0), n(0.0));
    outline.constrain(Constraint::Fix {
        point: middle,
        x: n(0.0),
        y: n(0.0),
    });
    let rim = outline.add_circle(middle, n(16.0));
    outline.constrain(Constraint::Radius {
        curve: rim,
        value: n(15.0),
    });
    outline
        .offset(&[rim], n(10.0), geop_core_sketch::offset::Corners::Round)
        .expect("a circle offsets inwards by less than its radius");
    let bolt = circle(&mut outline, [11.0, 0.0], 1.2);
    outline
        .pattern(
            &[bolt],
            &geop_core_sketch::copies::Step::Round {
                center: middle,
                angle: n(std::f64::consts::PI / 3.0),
            },
            6,
        )
        .expect("a hole off the middle turns round it");
    program.push(
        "outline",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "flange",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(2.5),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// A doubly curved panel, a saddle: a UV surface (`panel`) through three
/// ribs along `x` and three spines along `z`, each a spline of a 3-D
/// sketch through points of the saddle — the ribs' and the spines' meeting
/// where they cross; and a block under it (`block`, up along `y`) split
/// along it (`shaped`) into a piece with the saddle for its top and one
/// with it for its bottom. Drawn in tens of millimetres: the panel is 40 by
/// 30.
pub fn curved_panel() -> Program {
    let height = |x: f64, z: f64| 0.6 + 0.08 * (x - 2.0).powi(2) - 0.12 * (z - 1.5).powi(2);
    let mut program = Program::new();
    let mut splines = |name: &str, lines: Vec<Vec<[f64; 2]>>| -> Vec<CurveSource> {
        let mut sketch = Sketch3d::new();
        let mut curves = Vec::new();
        for line in lines {
            let points = line
                .iter()
                .map(|&[x, z]| {
                    let at = [x, height(x, z), z].map(|c| n(10.0 * c));
                    sketch.add_point(Vector3::from_array(at))
                })
                .collect();
            let curve = sketch.add_spline(points);
            curves.push(CurveSource::Edge(element(format!(
                "sketch3d({name},{curve})"
            ))));
        }
        program.push(
            name,
            AddSketch3dArgs {
                sketch,
                references: Vec::new(),
            },
        );
        curves
    };
    let ribs = splines(
        "ribs",
        [0.0, 1.5, 3.0]
            .iter()
            .map(|&y| [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| [x, y]).to_vec())
            .collect(),
    );
    let spines = splines(
        "spines",
        [0.0, 2.0, 4.0]
            .iter()
            .map(|&x| [0.0, 0.75, 1.5, 2.25, 3.0].map(|y| [x, y]).to_vec())
            .collect(),
    );
    program.push(
        "panel",
        NetworkSurfaceArgs {
            u_curves: ribs,
            v_curves: spines,
        },
    );
    let mut outline = Sketch::new();
    // Sketch `y` runs along world `-z`.
    rectangle(&mut outline, [5.0, -25.0], 30.0, 20.0);
    program.push(
        "block_sketch",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::Y)),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: whole_sketch("block_sketch"),
            extent: Extents::blind(15.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push(
        "shaped",
        SplitArgs {
            solid: solid("extrude(block)"),
            face: element("network(panel)"),
        },
    );
    program
}

/// Every example, by name, each with what makes its program: listing them
/// builds nothing — an editor lists them when it starts, and making one
/// may solve sketches, or build a part to name its faces.
pub fn all() -> Vec<Example<Program>> {
    vec![
        ("turbine_blade", || {
            drawn(include_str!("drawn/turbine_blade.geop"))
        }),
        ("cool_flange", || {
            drawn(include_str!("drawn/cool_flange.geop"))
        }),
        ("edge_part", || drawn(include_str!("drawn/edge_part.geop"))),
        ("conebox", || drawn(include_str!("drawn/conebox.geop"))),
        ("boss_on_reference_plane", boss_on_reference_plane),
        ("airfoil_wing", airfoil_wing),
        ("patterned_plate", patterned_plate),
        ("horn", horn),
        ("hole_plate", hole_plate),
        ("motor_flange", motor_flange),
        ("curved_panel", curved_panel),
    ]
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
    use geop_core_topology::contains::shell::{PointClassification, shell_contains};
    use geop_ops_datums::read_datums;
    use geop_ops_extrude_revolve::testing::model_of;
    use geop_ops_model::{NamedModel, PartDescription, RefId};
    use std::collections::BTreeMap;

    use crate::registry::testing::{Built, check_valid};

    use super::*;

    fn outputs_dir() -> std::path::PathBuf {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../outputs/parts");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Every named entity's exact geometry, by name: vertex points, edge
    /// curves and face surfaces as their full `Debug` enclosures — equal only
    /// if the two parts agree to the last bit of every interval.
    fn geometry_by_name(part: &NamedModel<S>) -> BTreeMap<String, String> {
        let model = part.topology();
        part.names()
            .iter()
            .filter_map(|(id, name)| {
                let geometry = match id {
                    RefId::Vertex(v) => format!("{:?}", model.get_vertex(v).ok()?.point),
                    RefId::Edge(e) => format!("{:?}", model.get_edge(e).ok()?.curve),
                    RefId::Face(f) => format!("{:?}", model.get_face(f).ok()?.surface),
                    RefId::Solid(_) => return None,
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
    fn build_and_round_trip(name: &str, program: &Program) -> NamedModel<S> {
        let part = program.build_model().unwrap();
        if let Err(report) = check_valid(&part) {
            panic!("{name}: {report}");
        }

        let json = program.to_json().unwrap();
        let dir = outputs_dir();
        std::fs::write(dir.join(format!("{name}.geop")), &json).unwrap();
        let description = PartDescription::of(&part).unwrap();
        std::fs::write(
            dir.join(format!("{name}.part.json")),
            serde_json::to_string_pretty(&description).unwrap(),
        )
        .unwrap();
        geop_core_rasterize::rasterize(part.topology(), 16)
            .unwrap()
            .scene(|_| geop_core_rasterize::debug::Color10::Blue)
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

        let rebuilt = read_back.build_model().unwrap();
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

    fn inside(part: &NamedModel<S>, solid: &str, p: [f64; 3]) -> PointClassification {
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

    /// The examples drawn in the editor build valid parts and round-trip,
    /// as the ones written here do.
    #[test]
    fn drawn_examples_round_trip() {
        let drawn = ["turbine_blade", "cool_flange", "edge_part", "conebox"];
        for (name, program) in all().into_iter().filter(|(n, _)| drawn.contains(n)) {
            build_and_round_trip(name, &program());
        }
    }

    #[test]
    fn boss_on_reference_plane_round_trips() {
        let part = build_and_round_trip("boss_on_reference_plane", &boss_on_reference_plane());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(boss)"]
        );
        // The datum the boss is sketched on is one of the part's.
        let built = boss_on_reference_plane().built().unwrap();
        let datums = read_datums(&built.current()).unwrap();
        assert_eq!(datums.names(), ["lifted"]);
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

    /// The curved panel is a valid face, through the middle of its middle
    /// rib and spine, where they cross; it splits the block under it in
    /// two, a point above the saddle in one and one below it in the other.
    #[test]
    fn curved_panel_round_trips() {
        let part = build_and_round_trip("curved_panel", &curved_panel());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.len(),
            2,
            "{:?}",
            description.solids.keys()
        );
        for (p, expected) in [
            ([20.0, 10.0, 15.0], [true, false]),
            ([20.0, 2.0, 15.0], [false, true]),
        ] {
            let inside = ["split(shaped,0)", "split(shaped,1)"]
                .map(|solid| inside(&part, solid, p) == PointClassification::Inside);
            assert!(
                inside == expected || inside == [expected[1], expected[0]],
                "at {p:?}: {inside:?}"
            );
        }
        let face = part.face_id("network(panel)").unwrap();
        let surface = &part.topology().get_face(face).unwrap().surface;
        let middle = Vector3::from_array([20.0, 6.0, 15.0].map(S::from_f64));
        assert!(
            geop_core_geometry::contains::surface::surface_could_contain(
                surface,
                &middle,
                10_000,
                S::from_f64(1e-7)
            )
            .unwrap()
            .is_some()
        );
    }

    /// The motor flange is a ring with six bolt holes through it: solid in
    /// its wall, open in its bore and in every hole, the copies round the
    /// bolt circle as much as the first.
    #[test]
    fn motor_flange_round_trips() {
        let part = build_and_round_trip("motor_flange", &motor_flange());
        let solid = "extrude(flange)";
        let at = |angle: f64, r: f64| {
            let a = angle.to_radians();
            [r * a.cos(), r * a.sin(), 1.25]
        };
        for (p, expected) in [
            (at(30.0, 11.0), PointClassification::Inside),
            (at(0.0, 7.0), PointClassification::Inside),
            (at(0.0, 0.0), PointClassification::Outside),
            (at(0.0, 11.0), PointClassification::Outside),
            (at(120.0, 11.0), PointClassification::Outside),
            (at(300.0, 11.0), PointClassification::Outside),
            (at(0.0, 16.0), PointClassification::Outside),
        ] {
            assert_eq!(inside(&part, solid, p), expected, "at {p:?}");
        }
    }

    /// Reproduces the real bug report described on `revolved_cone_on_box`.
    #[test]
    fn airfoil_wing_round_trips() {
        let part = build_and_round_trip("airfoil_wing", &airfoil_wing());
        assert_eq!(part.solid_names(), ["loft(wing)"]);
    }

    #[test]
    fn patterned_plate_round_trips() {
        let part = build_and_round_trip("patterned_plate", &patterned_plate());
        assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
        for (p, expected) in [
            ([5.0, 1.5, 0.25], PointClassification::Outside),
            ([5.0, 1.9, 0.25], PointClassification::Inside),
            ([7.0, 1.5, 0.25], PointClassification::Outside),
            ([6.0, 1.5, 0.25], PointClassification::Inside),
        ] {
            assert_eq!(inside(&part, "linear_pattern(holes)", p), expected, "{p:?}");
        }
    }

    /// A datum is a named entity, and a sketch can be placed on a datum
    /// plane: its frame, exactly.
    #[test]
    fn sketches_go_on_datum_planes() {
        let part = boss_on_reference_plane().built().unwrap();
        model_of(&part).check_names().unwrap();
        let built = part.current();
        let lifted = read_datums(&built).unwrap().get("lifted").unwrap();
        let placed = built
            .retrieve_element(&whole_sketch("boss_sketch"))
            .unwrap();
        assert!(placed.plane.origin().could_be_equal(lifted.frame.origin()));
        assert!(placed.plane.u().could_be_equal(lifted.frame.u()));
        // Sketch data is solved, not exact: agreeing to 1e-9 is agreeing.
        let off =
            |a: &Vector3<S>, b: [f64; 3]| a.sub(&Vector3::from_array(b.map(n))).norm().to_f64();
        assert!(
            off(placed.plane.origin(), [0.0, 0.0, 1.5]) < 1e-9,
            "{}",
            placed.plane
        );
        assert!(
            off(placed.plane.w(), [0.0, 0.0, 1.0]) < 1e-9,
            "{}",
            placed.plane
        );
    }
}
