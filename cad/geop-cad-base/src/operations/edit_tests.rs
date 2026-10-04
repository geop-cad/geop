//! The edit operations run as programs: deleting bodies, extracting faces
//! — and splitting with them — and projecting sketches onto faces.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::ScalInF64 as S;
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::{Combine, SplitArgs};
use geop_ops_edit::{DeleteBodyArgs, ExtractFaceArgs, ProjectCurveArgs};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
    part.check_names().unwrap();
}

fn on_xy(sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Plane(FrameAxis::Z),
        )),
        sketch,
        ..Default::default()
    }
}

/// An extrude of `sketch` as a new body, from `from` to `to` along its
/// plane's normal.
fn extrude(sketch: &str, from: f64, to: f64) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: sketch.into(),
        extent: Extents {
            side1: Extent::blind(to),
            symmetric: false,
            side2: (from != 0.0).then_some(Extent::blind(-from)),
            reversed: false,
        },
        face: false,
        combine: Combine::NewBody,
    }
}

/// A rectangle from `lo` to `hi`, its corners counter-clockwise: the ids
/// of its sides, the first along the bottom.
fn rectangle(s: &mut Sketch, lo: [f64; 2], hi: [f64; 2]) -> Vec<geop_core_sketch::CurveId> {
    let corners = [
        [lo[0], lo[1]],
        [hi[0], lo[1]],
        [hi[0], hi[1]],
        [lo[0], hi[1]],
    ];
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect()
}

/// A block's side wall, extracted, cuts a smaller block standing through
/// it in two; deleting the first block and the extracted face leaves
/// only the halves.
#[test]
fn extract_a_wall_split_with_it_and_delete_the_rest() {
    let mut big = Sketch::new();
    let sides = rectangle(&mut big, [0.0, 0.0], [1.0, 1.0]);
    let mut small = Sketch::new();
    rectangle(&mut small, [0.5, 0.2], [1.5, 0.8]);
    let wall = format!("extrude(a,big,{})", sides[1]);
    let extracted = format!("extract(x,{wall})");

    let mut program = Program::new();
    program.push("big", on_xy(big));
    program.push("a", extrude("big", -1.0, 1.0));
    program.push("small", on_xy(small));
    program.push("b", extrude("small", -0.5, 0.5));
    program.push("x", ExtractFaceArgs { face: wall.clone() });
    program.push(
        "halves",
        SplitArgs {
            solid: "extrude(b)".into(),
            face: extracted.clone(),
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.sheet_face_names(), [extracted.as_str()]);
    assert_eq!(
        part.solid_names(),
        ["extrude(a)", "split(halves,0)", "split(halves,1)"]
    );
    assert!(part.face_id(&wall).is_ok(), "the wall itself stays");

    program.push(
        "clean",
        DeleteBodyArgs {
            bodies: vec![
                EntityRef::Solid {
                    name: "extrude(a)".into(),
                },
                EntityRef::Face { name: extracted },
            ],
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.solid_names(), ["split(halves,0)", "split(halves,1)"]);
}

/// A block of `[-1, 1]^2 x [0, 1]` with the sketch `mark` on the plane
/// `z = 0` projected onto its top.
fn project_onto_top(mark: Sketch) -> Program {
    let mut block = Sketch::new();
    rectangle(&mut block, [-1.0, -1.0], [1.0, 1.0]);
    let mut program = Program::new();
    program.push("block_sketch", on_xy(block));
    program.push("block", extrude("block_sketch", 0.0, 1.0));
    program.push("mark", on_xy(mark));
    program.push(
        "p",
        ProjectCurveArgs {
            sketch: "mark".into(),
            face: "extrude(block,end)".into(),
        },
    );
    program
}

/// A circle projected up onto a block's top cuts a disc out of it: a face
/// of its own, inside a hole of the rest of the top.
#[test]
fn project_a_circle_onto_a_block_top() {
    let mut mark = Sketch::new();
    let c = mark.add_point(n(0.2), n(0.1));
    mark.add_circle(c, n(0.4));
    let part = project_onto_top(mark).build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.topology().faces.len(), 7);
    let top = part.face_id("extrude(block,end)").unwrap();
    assert_eq!(part.topology().get_face(top).unwrap().holes.len(), 1);
}

/// A line projected onto a block's top divides it in two, and leaves the
/// bottom — on the line's own plane — as it was.
#[test]
fn project_a_line_across_a_block_top() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(2.0));
    mark.add_line(a, b);
    let part = project_onto_top(mark).build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 7);
    assert_eq!(
        part.topology()
            .get_face(part.face_id("extrude(block,start)").unwrap())
            .unwrap()
            .holes
            .len(),
        0
    );
}

/// A line ending on the top divides nothing, and is refused.
#[test]
fn project_a_line_ending_on_the_face_fails() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(0.0));
    mark.add_line(a, b);
    let Err(error) = project_onto_top(mark).build::<S>(&NoFiles) else {
        panic!("projected");
    };
    assert!(error.root_message().contains("end inside"), "{error:?}");
}
