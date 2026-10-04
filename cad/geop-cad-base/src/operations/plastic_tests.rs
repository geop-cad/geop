//! The plastic features in programs, on an enclosure: a 2 x 2 x 1 box
//! shelled 0.2 thick, open at its top — drafted, ribbed, given a lip or a
//! groove.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_fillet::FilletArgs;
use geop_ops_plastic::{DraftArgs, GrooveArgs, LipArgs, RibArgs, RibDirection, RibSide};
use geop_ops_shell::ShellArgs;
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    assert_valid_case(part, "");
}

/// [`assert_valid`], saying which `case` of a sweep it is.
fn assert_valid_case(part: &Part<S>, case: &str) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        panic!(
            "{case}: {} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{case}: {errors:?}");
    }
}

fn z_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z))
}

fn face(name: &str) -> EntityRef {
    EntityRef::Face { name: name.into() }
}

/// The 2 x 2 x 1 box `box`: its walls `extrude(box,outline,c4)` (towards
/// `-y`), `c5` (`+x`), `c6` (`+y`) and `c7` (`-x`), its bottom
/// `extrude(box,start)` and top `extrude(box,end)`.
fn boxed() -> Program {
    let mut program = Program::new();
    let mut s = Sketch::new();
    let corners = [[0., 0.], [2., 0.], [2., 2.], [0., 2.]];
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..4 {
        s.add_line(p[i], p[(i + 1) % 4]);
    }
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: s,
            ..Default::default()
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

const WALLS: [&str; 4] = ["c4", "c5", "c6", "c7"];

pub(super) fn wall(c: &str) -> String {
    format!("extrude(box,outline,{c})")
}

/// The box, shelled 0.2 thick open at its top: the solid `shell(s)`, its
/// inner walls `shell(s,extrude(box,outline,c4))` and so on, its rim
/// `shell(s,extrude(box,end))`.
pub(super) fn enclosure() -> Program {
    let mut program = boxed();
    program.push(
        "s",
        ShellArgs {
            solid: "extrude(box)".into(),
            faces: vec!["extrude(box,end)".into()],
            thickness: 0.2,
        },
    );
    program
}

fn draft(faces: &[String], degrees: f64) -> DraftArgs {
    DraftArgs {
        faces: faces.to_vec(),
        neutral: Some(face("extrude(box,start)")),
        angle: degrees,
        reversed: false,
    }
}

/// The enclosure's outside drafted 3 degrees about its bottom, pulled up
/// out of the mould.
#[test]
fn enclosure_outside_drafted() {
    let mut program = enclosure();
    let walls: Vec<String> = WALLS.iter().map(|c| wall(c)).collect();
    program.push("d", draft(&walls, 3.0));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    // The top of the +x wall leans in, the bottom stays.
    let top = part.vertex_id("extrude(box,outline,p2,end)").unwrap();
    let x = part.topology().get_vertex(top).unwrap().point[0].to_f64();
    assert!((x - (2.0 - 3f64.to_radians().tan())).abs() < 1e-12, "{x}");
    assert!(part.solid_id("draft(d)").is_ok());
}

/// The enclosure's inside drafted 1 degree: the cavity widens upwards.
#[test]
fn enclosure_inside_drafted() {
    let mut program = enclosure();
    let walls: Vec<String> = WALLS
        .iter()
        .map(|c| format!("shell(s,{})", wall(c)))
        .collect();
    program.push("d", draft(&walls, 1.0));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
}

/// A round port through the +x wall: the cylinder it leaves is re-trimmed
/// where the drafted wall now cuts it.
#[test]
fn enclosure_with_a_port_drafted() {
    let mut program = enclosure();
    let mut s = Sketch::new();
    // On the +x wall, `u` runs along `y` and `v` along `z`.
    let c = s.add_point(n(1.0), n(0.5));
    s.add_circle(c, n(0.2));
    program.push(
        "port_sketch",
        AddSketchArgs {
            plane: Some(face(&wall("c5"))),
            sketch: s,
            ..Default::default()
        },
    );
    program.push(
        "port",
        ExtrudeArgs {
            sketch: "port_sketch".into(),
            extent: Extents::blind(-0.5),
            face: false,
            combine: Combine::Difference {
                target: "shell(s)".into(),
            },
        },
    );
    program.push("d", draft(&[wall("c5")], 5.0));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
}

/// A wall next to a fillet is refused, naming the fillet's face.
#[test]
fn wall_next_to_a_fillet_is_refused() {
    let mut program = boxed();
    program.push(
        "f",
        FilletArgs {
            edges: vec!["extrude(box,outline,p1)".into()],
            radius: 0.2,
        },
    );
    program.push("d", draft(&[wall("c4")], 3.0));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("drafting next to a fillet is refused");
    };
    let message = format!("{error}");
    assert!(message.contains("tangentially"), "{message}");
    assert!(message.contains("fillet(f,"), "{message}");
}

const RIM: &str = "shell(s,extrude(box,end))";

/// The inner edge of the rim along the wall `c`.
fn inner_edge(c: &str) -> String {
    format!("shell(s,extrude(box,outline,{c},end))")
}

/// A lip all along the inside of the rim, and on a second enclosure the
/// groove that takes it.
#[test]
fn enclosure_lip_and_groove() {
    let mut program = enclosure();
    program.push(
        "l",
        LipArgs {
            face: RIM.into(),
            edges: Vec::new(),
            width: 0.08,
            height: 0.1,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(part.solid_id("lip(l)").is_ok());

    let mut program = enclosure();
    program.push(
        "g",
        GrooveArgs {
            face: RIM.into(),
            edges: Vec::new(),
            width: 0.08,
            height: 0.1,
            clearance: 0.02,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(part.solid_id("groove(g)").is_ok());
}

/// A lip along two inner edges of the rim, ending at corners of the rim:
/// its square ends would lie on the next walls, along their edges, so it is
/// refused, naming the corner.
#[test]
fn lip_ending_at_a_corner_is_refused() {
    let mut program = enclosure();
    program.push(
        "l",
        LipArgs {
            face: RIM.into(),
            edges: vec![inner_edge("c5"), inner_edge("c6")],
            width: 0.08,
            height: 0.1,
        },
    );
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("a lip ending at a corner is refused");
    };
    let message = format!("{error}");
    assert!(message.contains("a corner of face"), "{message}");
}

/// Edges that do not form one chain are refused, by name.
#[test]
fn lip_along_opposite_edges_is_refused() {
    let mut program = enclosure();
    program.push(
        "l",
        LipArgs {
            face: RIM.into(),
            edges: vec![inner_edge("c4"), inner_edge("c6")],
            width: 0.08,
            height: 0.1,
        },
    );
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("a lip along two separate edges is refused");
    };
    let message = format!("{error}");
    assert!(message.contains("not one chain"), "{message}");
}

/// The enclosure with a reference plane through its middle, `middle`, at
/// `y = 1` — offset from the `-y` wall, whose plane runs `u` along `x` and
/// `v` along `z` — and on it the sketch `rib_sketch` of a line across the
/// cavity at height 0.7, from `x = 0.5` to `x = 1.5`.
pub(super) fn enclosure_with_rib_sketch() -> Program {
    let mut program = enclosure();
    program.push(
        "middle",
        AddDatumArgs {
            selection: vec![face(&wall("c4"))],
            construction: Construction::Offset { distance: -1.0 },
        },
    );
    let mut s = Sketch::new();
    let a = s.add_point(n(0.5), n(0.7));
    let b = s.add_point(n(1.5), n(0.7));
    s.add_line(a, b);
    program.push(
        "rib_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum("middle")),
            sketch: s,
            ..Default::default()
        },
    );
    program
}

fn rib(direction: RibDirection, side: RibSide, flipped: bool) -> RibArgs {
    RibArgs {
        sketch: "rib_sketch".into(),
        solid: "shell(s)".into(),
        thickness: 0.05,
        side,
        direction,
        flipped,
    }
}

/// A rib from a line across the cavity, grown parallel to its sketch down
/// to the floor, its ends run on into the walls.
#[test]
fn enclosure_rib() {
    let mut program = enclosure_with_rib_sketch();
    program.push("r", rib(RibDirection::Parallel, RibSide::Symmetric, true));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(part.solid_id("rib(r)").is_ok());
}

/// The same line grown the other way, up out of the open top, meets
/// nothing that stops it: refused.
#[test]
fn rib_out_of_the_top_is_refused() {
    let mut program = enclosure_with_rib_sketch();
    program.push("r", rib(RibDirection::Parallel, RibSide::First, false));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("a rib out of the top is refused");
    };
    let message = format!("{error}");
    assert!(
        message.contains("without meeting it all along"),
        "{message}"
    );
}

/// A rib, grown normal to its sketch — across the cavity from the `-y`
/// wall to the `+y` one — with its line running across the cavity.
#[test]
fn enclosure_rib_normal_to_its_sketch() {
    let mut program = enclosure();
    program.push(
        "level",
        AddDatumArgs {
            selection: vec![face("shell(s,extrude(box,start))")],
            construction: Construction::Offset { distance: 0.5 },
        },
    );
    let mut s = Sketch::new();
    let a = s.add_point(n(0.5), n(1.0));
    let b = s.add_point(n(1.5), n(1.0));
    s.add_line(a, b);
    program.push(
        "rib_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum("level")),
            sketch: s,
            ..Default::default()
        },
    );
    program.push("r", rib(RibDirection::Normal, RibSide::Symmetric, true));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
}

/// The sweep: ribs of several thicknesses, either side, both directions;
/// lips and grooves of several sizes; drafts of several angles.
#[test]
#[ignore = "slow: rib, lip, groove and draft sweep — run with `cargo test -- --ignored`"]
fn plastic_sweep() {
    for thickness in [0.02, 0.05, 0.1] {
        for side in [RibSide::Symmetric, RibSide::First, RibSide::Second] {
            let mut program = enclosure_with_rib_sketch();
            let mut args = rib(RibDirection::Parallel, side, true);
            args.thickness = thickness;
            program.push("r", args);
            let part = program
                .build::<S>(&NoFiles)
                .unwrap_or_else(|e| panic!("rib {thickness} {side:?}: {e}"));
            assert_valid_case(&part, &format!("rib {thickness} {side:?}"));
        }
    }
    for (width, height, clearance) in [(0.08, 0.05, 0.0), (0.1, 0.2, 0.01), (0.15, 0.1, 0.03)] {
        let mut program = enclosure();
        program.push(
            "l",
            LipArgs {
                face: RIM.into(),
                edges: Vec::new(),
                width,
                height,
            },
        );
        assert_valid_case(
            &program.build::<S>(&NoFiles).unwrap(),
            &format!("lip {width} {height}"),
        );
        let mut program = enclosure();
        program.push(
            "g",
            GrooveArgs {
                face: RIM.into(),
                edges: Vec::new(),
                width,
                height,
                clearance,
            },
        );
        let part = program
            .build::<S>(&NoFiles)
            .unwrap_or_else(|e| panic!("groove {width} {height} {clearance}: {e}"));
        assert_valid_case(&part, &format!("groove {width} {height} {clearance}"));
    }
    for angle in [0.5, 1.0, 3.0, 7.0, 10.0, -3.0] {
        let mut program = enclosure();
        let walls: Vec<String> = WALLS.iter().map(|c| wall(c)).collect();
        program.push("d", draft(&walls, angle));
        let part = program
            .build::<S>(&NoFiles)
            .unwrap_or_else(|e| panic!("draft {angle}: {e}"));
        assert_valid_case(&part, &format!("draft {angle}"));
    }
}

/// A groove for a lip 0.05 wide and high, no clearance: its cut reaches
/// 0.05 past the enclosure's inner corners, less than the boolean's tracing
/// stride (0.1).
///
/// The trace along the inner wall `x = 0.2`, leaving the cut's inner corner
/// `(0.2, 1.8, 0.95)`, once ended at `(0.2, 1.85, 0.95)` one stride later —
/// a vertex on both the wall's and the cut floor's planes, but 0.05
/// *behind* the trace's start, outside the wall — and spliced a spur out of
/// the wall to it. The march now only ends at a vertex ahead of it.
#[test]
fn narrow_groove() {
    let mut program = enclosure();
    program.push(
        "g",
        GrooveArgs {
            face: RIM.into(),
            edges: Vec::new(),
            width: 0.05,
            height: 0.05,
            clearance: 0.0,
        },
    );
    assert_valid(&program.build::<S>(&NoFiles).unwrap());
}

/// Drafted 15 degrees, the enclosure's 0.2 thick walls would lean in by
/// 0.27 at the top, past their inner side: refused, naming the rim.
#[test]
fn too_steep_a_draft_is_refused() {
    let mut program = enclosure();
    let walls: Vec<String> = WALLS.iter().map(|c| wall(c)).collect();
    program.push("d", draft(&walls, 15.0));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("a draft crossing the walls is refused");
    };
    let message = format!("{error}");
    assert!(message.contains("too steep"), "{message}");
    assert!(message.contains(RIM), "{message}");
}
