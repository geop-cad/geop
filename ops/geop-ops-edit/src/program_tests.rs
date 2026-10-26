//! The edit operations run as steps on a block: projecting sketches onto
//! its top. Deleting bodies and extracting faces, and splitting with those,
//! are run by `geop-ops-boolean`'s tests.

use crate::{ProjectCurve, ProjectCurveArgs};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::element::ElementReference;
use geop_lib_sweep::shapes::block_part;
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_origin},
    whole_sketch,
};
use geop_ops_model::{
    NamedModel, model,
    testing::{assert_valid, error, face_towards, given_part, model_of, run_step},
};
use geop_ops_sketch::{AddSketch, AddSketchArgs};

/// The block the sketches are projected onto: `[-1, 1]^2 x [0, 1]`.
fn block() -> NamedModel {
    block_part("block", [-1.0, -1.0, 0.0], [1.0, 1.0, 1.0])
}

/// The name of the face of `block` facing `normal`.
fn face_of(block: &NamedModel, normal: [f64; 3]) -> String {
    let solid = block.solid_id("cube(block)").unwrap();
    let face = face_towards(block, solid, normal);
    block.name_of(face).unwrap().to_string()
}

/// The block with the sketch `mark` on the plane `z = 0` projected onto
/// its top.
fn project_onto_top(mark: Sketch) -> GeopResult<NamedModel> {
    let block = block();
    let top = face_of(&block, [0.0, 0.0, 1.0]);
    let mut part = given_part(&block)?;
    let on_xy = AddSketchArgs::on(on_origin(FrameAxis::Z), mark);
    run_step::<AddSketch>(&mut part, "mark", &on_xy)?;
    let project = ProjectCurveArgs {
        sketch: whole_sketch("mark"),
        face: ElementReference::new(model(), top),
    };
    run_step::<ProjectCurve>(&mut part, "p", &project)?;
    Ok(model_of(&part))
}

/// A circle projected up onto a block's top cuts a disc out of it: a face
/// of its own, inside a hole of the rest of the top.
#[test]
fn project_a_circle_onto_a_block_top() {
    let mut mark = Sketch::new();
    let c = mark.add_point(n(0.2), n(0.1));
    mark.add_circle(c, n(0.4));
    let part = project_onto_top(mark).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.topology().faces.len(), 7);
    let top = part.face_id(&face_of(&block(), [0.0, 0.0, 1.0])).unwrap();
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
    let part = project_onto_top(mark).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 7);
    let bottom = part.face_id(&face_of(&block(), [0.0, 0.0, -1.0])).unwrap();
    assert_eq!(part.topology().get_face(bottom).unwrap().holes.len(), 0);
}

/// A line ending on the top divides nothing, and is refused.
#[test]
fn project_a_line_ending_on_the_face_fails() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(0.0));
    mark.add_line(a, b);
    let error = error(project_onto_top(mark));
    assert!(error.root_message().contains("end inside"), "{error:?}");
}

// ── Direct edits ────────────────────────────────────────────────────────────

mod direct {
    use geop_core_program::{operation_registry::OperationRegistry, program::Program};
    use geop_lib_fixtures::{PrimitiveArgs, bodies::cylinder, drilled_box};
    use geop_ops_model::{
        element,
        testing::{Built, assert_valid, assert_volume},
    };

    use crate::{MoveFace, MoveFaceArgs, OffsetFace, OffsetFaceArgs, ReplaceFace, ReplaceFaceArgs};

    fn registry() -> geop_core_math::geop_error::GeopResult<OperationRegistry> {
        geop_lib_fixtures::registry()?
            .with(MoveFace)?
            .with(OffsetFace)?
            .with(ReplaceFace)
    }

    /// The top of a box drilled from the top, pulled up a quarter: the
    /// hole's wall grows with it.
    #[test]
    fn drilled_box_top_moved_up() {
        let mut program = drilled_box();
        program.push(
            "m",
            MoveFaceArgs {
                faces: vec![element("block(box,end)")],
                translation: [0.0, 0.0, 0.25],
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let hole = std::f64::consts::PI * 0.4 * 0.4;
        assert_volume(&part, "move_face(m)", 1.25 * 4.0 - 0.75 * hole, 2e-2);
        assert!(part.face_id("block(box,end)").is_ok());
    }

    /// A cylinder's wall offset out by a half: a cylinder of radius 1.5.
    #[test]
    fn cylinder_wall_offset() {
        let mut program = cylinder();
        let part = program.build_model(registry).unwrap();
        let walls: Vec<_> = part
            .topology()
            .faces
            .iter()
            .filter(|(_, f)| f.surface.as_plane().unwrap().is_none())
            .map(|(&id, _)| element(part.name_of(id).unwrap()))
            .collect();
        program.push(
            "o",
            OffsetFaceArgs {
                faces: walls,
                distance: 0.5,
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let volume = std::f64::consts::PI * 1.5 * 1.5 * 1.5;
        assert_volume(&part, "offset_face(o)", volume, 5e-2);
    }

    /// A box's top put on the top of a taller box beside it: as tall.
    #[test]
    fn box_top_replaced_by_a_higher_one() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push("b", PrimitiveArgs::block([3.0, 0.0, 0.0], [4.0, 1.0, 2.0]));
        program.push(
            "r",
            ReplaceFaceArgs {
                face: element("block(a,end)"),
                with: element("block(b,end)"),
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        assert_volume(&part, "replace_face(r)", 2.0, 1e-5);
    }

    /// The top pushed down through the bottom is refused: the sides would
    /// vanish.
    #[test]
    fn box_top_pushed_through_is_refused() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push(
            "o",
            OffsetFaceArgs {
                faces: vec![element("block(a,end)")],
                distance: -1.5,
            },
        );
        let error = program.build_model(registry).unwrap_err();
        assert!(format!("{error}").contains("turns around"), "{error}");
    }
}
