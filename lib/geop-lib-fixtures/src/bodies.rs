//! The bodies the stress sweeps of blends and shells run over: boxes,
//! cylinders and spheres, whole and with parts cut off, added or drilled —
//! and bodies with free-form edges, a boss on a sphere and a pipe tee.
//! Each is a program of [`Primitive`] steps building one solid, named as
//! each says.

use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_model::{
    BooleanOp, Combine, NamedModel, solid,
    testing::{Built, check_valid},
};

use crate::{PrimitiveArgs, Shape};

/// The one solid `program` builds, which is valid, and its name.
pub fn one_solid(program: &Program) -> (NamedModel, String) {
    let part = program.build_model(crate::registry).unwrap();
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

/// `body` as the step `body`, and `features` after it, each as its step,
/// its shape combined by its operation with the solid the step before
/// left.
fn body_with(body: PrimitiveArgs, features: Vec<(&str, BooleanOp, Shape)>) -> Program {
    let mut program = Program::new();
    let mut last = format!("{}(body)", body.shape.kind());
    program.push("body", body);
    for (step, op, shape) in features {
        let kind = shape.kind();
        let combine = Combine::With {
            op,
            target: solid(&last),
        };
        program.push(step, PrimitiveArgs::new(shape).combined(combine));
        last = format!("{kind}({step})");
    }
    program
}

/// A box of 2 x 2 x 1 on the ground: the solid `block(body)`.
pub fn block() -> Program {
    body_with(PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]), vec![])
}

/// A cylinder of radius 1 and height 1.5 around the z axis: the solid
/// `cylinder(body)`.
pub fn cylinder() -> Program {
    body_with(PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5), vec![])
}

/// A sphere of radius 1 around the origin: the solid `sphere(body)`.
pub fn sphere() -> Program {
    body_with(PrimitiveArgs::sphere([0.0; 3], 1.0), vec![])
}

/// The block with a corner cut off.
pub fn block_corner_cut_off() -> Program {
    body_with(
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        vec![("corner", BooleanOp::Difference, PrimitiveArgs::prism(&[[1.5, 2.5], [2.5, 1.5], [2.5, 2.5]], 0.0, 1.0).shape)],
    )
}

/// The block with a pocket 0.4 deep in its top.
pub fn block_pocketed() -> Program {
    body_with(
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        vec![("pocket", BooleanOp::Difference, PrimitiveArgs::block([0.5, 0.5, 0.6], [1.5, 1.5, 1.0]).shape)],
    )
}

/// The block with a step cut along one side.
pub fn block_stepped() -> Program {
    body_with(
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        vec![("step", BooleanOp::Difference, PrimitiveArgs::block([-0.5, 1.5, 0.0], [1.0, 2.5, 0.5]).shape)],
    )
}

/// The block with a round hole through it.
pub fn block_drilled() -> Program {
    body_with(
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        vec![("hole", BooleanOp::Difference, PrimitiveArgs::cylinder_z([1.0, 1.0, 0.0], 0.4, 1.0).shape)],
    )
}

/// The block with a round boss on its top.
pub fn block_with_boss() -> Program {
    body_with(
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
        vec![("boss", BooleanOp::Union, PrimitiveArgs::cylinder_z([1.0, 1.0, 1.0], 0.4, 0.5).shape)],
    )
}

/// The cylinder with a flat cut along it at `x = 0.6`.
pub fn cylinder_with_flat() -> Program {
    body_with(
        PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5),
        vec![("flat", BooleanOp::Difference, PrimitiveArgs::block([0.6, -2.0, 0.0], [2.0, 2.0, 1.5]).shape)],
    )
}

/// The cylinder drilled along its axis.
pub fn cylinder_drilled() -> Program {
    body_with(
        PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5),
        vec![(
            "hole",
            BooleanOp::Difference, PrimitiveArgs::cylinder_z([0.0; 3], 0.3, 1.5).shape,
        )],
    )
}

/// The cylinder drilled along a line beside its axis.
pub fn cylinder_drilled_off_axis() -> Program {
    body_with(
        PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5),
        vec![("hole", BooleanOp::Difference, PrimitiveArgs::cylinder_z([0.4, 0.0, 0.0], 0.2, 1.5).shape)],
    )
}

/// The cylinder with a round pocket in its top.
pub fn cylinder_pocketed() -> Program {
    body_with(
        PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5),
        vec![("pocket", BooleanOp::Difference, PrimitiveArgs::cylinder_z([0.0, 0.0, 1.0], 0.5, 0.5).shape)],
    )
}

/// The cylinder with a round boss on its top.
pub fn cylinder_with_boss() -> Program {
    body_with(
        PrimitiveArgs::cylinder_z([0.0; 3], 1.0, 1.5),
        vec![("boss", BooleanOp::Union, PrimitiveArgs::cylinder_z([0.0, 0.0, 1.5], 0.4, 0.5).shape)],
    )
}

/// The sphere, its half below `z = 0` cut off.
pub fn hemisphere() -> Program {
    body_with(
        PrimitiveArgs::sphere([0.0; 3], 1.0),
        vec![("half", BooleanOp::Difference, PrimitiveArgs::block([-2.0, -2.0, -2.0], [2.0, 2.0, 0.0]).shape)],
    )
}

/// The sphere drilled through along `z`.
pub fn sphere_drilled() -> Program {
    body_with(
        PrimitiveArgs::sphere([0.0; 3], 1.0),
        vec![("hole", BooleanOp::Difference, PrimitiveArgs::cylinder_z([0.0, 0.0, -2.0], 0.3, 4.0).shape)],
    )
}

/// The sphere with its side beyond `x = 0.5` cut off.
pub fn sphere_with_side_cut_off() -> Program {
    body_with(
        PrimitiveArgs::sphere([0.0; 3], 1.0),
        vec![("side", BooleanOp::Difference, PrimitiveArgs::block([0.5, -2.0, -2.0], [2.0, 2.0, 2.0]).shape)],
    )
}

/// A sphere with a cylinder standing on it, around its axis: the two meet
/// in a free-form rim — no face of revolution with a straight meridian on
/// one side of it — which a rolling ball rounds.
pub fn sphere_with_boss() -> Program {
    body_with(
        PrimitiveArgs::sphere([0.0; 3], 1.0),
        vec![(
            "boss",
            BooleanOp::Union, PrimitiveArgs::cylinder_z([0.0; 3], 0.4, 1.5).shape,
        )],
    )
}

/// A pipe along `x` with a narrower branch standing on it: the two meet in
/// a saddle, a free-form edge round which a rolling ball rolls from face to
/// face of both. The solid `cylinder(branch)`.
pub fn pipe_tee() -> Program {
    let mut program = Program::new();
    program.push(
        "pipe",
        PrimitiveArgs::cylinder([-1.0, 0.0, 0.0], FrameAxis::X, 0.5, 2.0),
    );
    program.push(
        "branch",
        PrimitiveArgs::cylinder_z([0.0; 3], 0.3, 1.0).join("cylinder(pipe)"),
    );
    program
}

/// Every body, named: what the sweeps run over.
pub fn all() -> Vec<(&'static str, Program)> {
    vec![
        ("block", block()),
        ("cylinder", cylinder()),
        ("sphere", sphere()),
        ("block_corner_cut_off", block_corner_cut_off()),
        ("block_pocketed", block_pocketed()),
        ("block_stepped", block_stepped()),
        ("block_drilled", block_drilled()),
        ("block_with_boss", block_with_boss()),
        ("cylinder_with_flat", cylinder_with_flat()),
        ("cylinder_drilled", cylinder_drilled()),
        ("cylinder_drilled_off_axis", cylinder_drilled_off_axis()),
        ("cylinder_pocketed", cylinder_pocketed()),
        ("cylinder_with_boss", cylinder_with_boss()),
        ("hemisphere", hemisphere()),
        ("sphere_drilled", sphere_drilled()),
        ("sphere_with_side_cut_off", sphere_with_side_cut_off()),
        ("sphere_with_boss", sphere_with_boss()),
        ("pipe_tee", pipe_tee()),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Primitive;
    use geop_core_program::operation::Operation;

    /// Every body builds one valid solid, named by its last step.
    #[test]
    fn every_body_is_one_valid_solid() {
        for (name, program) in all() {
            let (_, solid) = one_solid(&program);
            let last = program.steps.last().unwrap();
            assert_eq!(last.operation, Primitive::KIND, "{name}");
            assert!(solid.ends_with(&format!("({})", last.id)), "{name}: {solid}");
        }
    }
}
