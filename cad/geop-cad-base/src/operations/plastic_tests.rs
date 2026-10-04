//! The plastic features in programs, on an enclosure: a 2 x 2 x 1 box
//! shelled 0.2 thick, open at its top — drafted, ribbed, given a lip or a
//! groove.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_fillet::FilletArgs;
use geop_ops_plastic::DraftArgs;
use geop_ops_shell::ShellArgs;
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        panic!(
            "{} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:?}");
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

fn wall(c: &str) -> String {
    format!("extrude(box,outline,{c})")
}

/// The box, shelled 0.2 thick open at its top: the solid `shell(s)`, its
/// inner walls `shell(s,extrude(box,outline,c4))` and so on, its rim
/// `shell(s,extrude(box,end))`.
fn enclosure() -> Program {
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
