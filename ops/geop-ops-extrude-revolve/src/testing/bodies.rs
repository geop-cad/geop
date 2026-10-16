//! The bodies the stress sweeps of blends and shells run over: boxes,
//! cylinders and spheres, whole and with parts cut off, added or drilled —
//! and bodies with free-form edges, a boss on a sphere and a pipe tee.
//! Each is a program building one solid, named as each says.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::{program::Program, values::Design};
use geop_ops_model::{Combine, NamedModel, testing::check_valid};
use geop_ops_sketch::{Sketch, SketchPlane, sketch_element, whole_sketch};

use crate::{Extents, Output, RevolveArgs, RevolveAxis};

use super::{
    Built, both_ways, disc, extrude_sketch, n, on_face, on_origin, outline, rectangle, sketch,
};

/// The one solid `program` builds, which is valid, and its name.
pub fn one_solid(program: &Program) -> (NamedModel<Design>, String) {
    let part = program.build_model(super::registry).unwrap();
    let solids = part.solid_names();
    let [solid] = solids.as_slice() else {
        panic!("not one solid: {solids:?}");
    };
    if let Err(e) = check_valid(&part) {
        panic!("the body itself is invalid: {e}");
    }
    let solid = solid.clone();
    (part, solid)
}

/// The top face of the solid `extrude(body)`.
fn top() -> SketchPlane {
    on_face("extrude(body,end)")
}

/// A box of 2 x 2 x 1 on the ground: the solid `extrude(body)`.
pub fn block() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "body",
        on_origin(FrameAxis::Z),
        rectangle(0.0, 0.0, 2.0, 2.0),
        Extents::blind(1.0),
        Combine::NewBody,
    );
    program
}

/// A cylinder of radius 1 and height 1.5 around the z axis: the solid
/// `extrude(body)`.
pub fn cylinder() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "body",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 1.0),
        Extents::blind(1.5),
        Combine::NewBody,
    );
    program
}

/// A sphere of radius 1 around the origin, a half disc turned round the y
/// axis: the solid `revolve(body)`.
pub fn sphere() -> Program {
    let mut s = Sketch::new();
    let top = s.add_point(n(0.0), n(1.0));
    let bottom = s.add_point(n(0.0), n(-1.0));
    s.add_arc(bottom, top, n(std::f64::consts::PI));
    let axis = s.add_line(top, bottom);
    let mut program = Program::new();
    program.push("body_sketch", sketch(on_origin(FrameAxis::Z), s));
    program.push(
        "body",
        RevolveArgs {
            sketch: whole_sketch("body_sketch"),
            axis: Some(RevolveAxis::Sketch(sketch_element("body_sketch", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// The block with a corner cut off.
pub fn block_corner_cut_off() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "corner",
        on_origin(FrameAxis::Z),
        outline(&[[1.5, 2.5], [2.5, 1.5], [2.5, 2.5]]),
        Extents::blind(1.0),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a pocket 0.4 deep in its top.
pub fn block_pocketed() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "pocket",
        top(),
        rectangle(0.5, 0.5, 1.5, 1.5),
        Extents::blind(-0.4),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a step cut along one side.
pub fn block_stepped() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "step",
        on_origin(FrameAxis::Z),
        rectangle(-0.5, 1.5, 1.0, 2.5),
        Extents::blind(0.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a round hole through it.
pub fn block_drilled() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([1.0, 1.0], 0.4),
        Extents::blind(1.0),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The block with a round boss on its top.
pub fn block_with_boss() -> Program {
    let mut program = block();
    extrude_sketch(
        &mut program,
        "boss",
        top(),
        disc([1.0, 1.0], 0.4),
        Extents::blind(0.5),
        Combine::join("extrude(body)"),
    );
    program
}

/// The cylinder with a flat cut along it at `x = 0.6`.
pub fn cylinder_with_flat() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "flat",
        on_origin(FrameAxis::Z),
        rectangle(0.6, -2.0, 2.0, 2.0),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The cylinder drilled along its axis.
pub fn cylinder_drilled() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The cylinder drilled along a line beside its axis.
pub fn cylinder_drilled_off_axis() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.4, 0.0], 0.2),
        Extents::blind(1.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The cylinder with a round pocket in its top.
pub fn cylinder_pocketed() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "pocket",
        top(),
        disc([0.0, 0.0], 0.5),
        Extents::blind(-0.5),
        Combine::cut("extrude(body)"),
    );
    program
}

/// The cylinder with a round boss on its top.
pub fn cylinder_with_boss() -> Program {
    let mut program = cylinder();
    extrude_sketch(
        &mut program,
        "boss",
        top(),
        disc([0.0, 0.0], 0.4),
        Extents::blind(0.5),
        Combine::join("extrude(body)"),
    );
    program
}

/// The sphere, its half below `z = 0` cut off.
pub fn hemisphere() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "half",
        on_origin(FrameAxis::Z),
        rectangle(-2.0, -2.0, 2.0, 2.0),
        Extents::blind(-2.0),
        Combine::cut("revolve(body)"),
    );
    program
}

/// The sphere drilled through along `z`.
pub fn sphere_drilled() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "hole",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        both_ways(2.0),
        Combine::cut("revolve(body)"),
    );
    program
}

/// The sphere with its side beyond `x = 0.5` cut off.
pub fn sphere_with_side_cut_off() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "side",
        on_origin(FrameAxis::Z),
        rectangle(0.5, -2.0, 2.0, 2.0),
        both_ways(2.0),
        Combine::cut("revolve(body)"),
    );
    program
}

/// A sphere with a cylinder standing on it, around its axis: the two meet
/// in a free-form rim — no face of revolution with a straight meridian on
/// one side of it — which a rolling ball rounds.
pub fn sphere_with_boss() -> Program {
    let mut program = sphere();
    extrude_sketch(
        &mut program,
        "boss",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.4),
        Extents::blind(1.5),
        Combine::join("revolve(body)"),
    );
    program
}

/// A pipe along `x` with a narrower branch standing on it: the two meet in
/// a saddle, a free-form edge round which a rolling ball rolls from face to
/// face of both.
pub fn pipe_tee() -> Program {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "pipe",
        on_origin(FrameAxis::X),
        disc([0.0, 0.0], 0.5),
        both_ways(1.0),
        Combine::NewBody,
    );
    extrude_sketch(
        &mut program,
        "branch",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.3),
        Extents::blind(1.0),
        Combine::join("extrude(pipe)"),
    );
    program
}
