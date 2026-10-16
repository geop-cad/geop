//! The examples of datums an editor offers: parts sketched on datum planes
//! — a boss on a plane above a box, a wing lofted between sections on
//! planes apart.

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_program::program::Program;
use geop_core_sketch::PointId;
use geop_ops_datums::{ORIGIN, component_name, datum, origin_plane};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, LoftArgs, Output};
use geop_ops_model::{Combine, element};
use geop_ops_sketch::{
    AddSketchArgs, Sketch, SketchPlane,
    figures::{P2, circle, n, rectangle, solved},
    whole_sketch,
};

use crate::{AddDatumArgs, Construction, Selected};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![
            ("boss_on_reference_plane", boss_on_reference_plane),
            ("airfoil_wing", airfoil_wing),
        ],
        workspaces: Vec::new(),
    }
}

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
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
/// 0.3 — the tip's section drawn on a datum plane offset from the root's.
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
