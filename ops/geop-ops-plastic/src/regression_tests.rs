//! Programs from tutorials and bug reports, rebuilt as the smallest
//! programs found that still fail.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{element::ElementReference, operation_registry::OperationRegistry};
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::{
    model, solid,
    testing::{Built, assert_valid},
};
use geop_ops_shell::{Shell, ShellArgs};

use crate::{Groove, GrooveArgs, Lip, LipArgs, testing::boxed};

/// The operations these programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?.with(Fillet)?)
        .with(Shell)?
        .with(Lip)?
        .with(Groove)
}

/// An enclosure with rounded corners (the Fusion 360 "enclosure with a
/// lip" tutorial): the 2 x 2 x 1 box of [`boxed`], its four upright edges
/// rounded by 0.5, shelled 0.2 thick open at its top, and a lip 0.08 wide
/// and 0.1 high raised along the whole inner edge of the rim — and, for
/// the mating half, a groove there instead. Both are refused:
///
/// > edge shell(s,fillet(r,extrude(box,outline,p3),fillet,st1)) is neither
/// > straight nor circular: a lip or groove runs along straight and
/// > circular edges
///
/// That edge is the inner copy of the blend's top edge, the quarter circle
/// of radius 0.3 where the inner wall's round corner meets the rim. The
/// same enclosure drawn with its corners as arcs in the outline, extruded
/// and shelled, takes the lip: the arcs' copies are circles. Rounded by the
/// fillet, the corner's edges are fitted curves the shell offsets as
/// fitted curves, and the lip does not see that they are circles.
#[test]
#[ignore = "bug: a lip or groove on the rim of a box with rounded upright edges, shelled, is refused: \"edge shell(s,fillet(r,..,fillet,st1)) is neither straight nor circular\" — the blend's quarter circles are fitted curves; corners drawn as sketch arcs take the lip"]
fn lip_and_groove_on_a_rounded_enclosure() {
    let mut program = boxed();
    program.push(
        "r",
        FilletArgs::constant(
            (0..4)
                .map(|p| format!("extrude(box,outline,p{p})"))
                .collect(),
            0.5,
        ),
    );
    program.push(
        "s",
        ShellArgs {
            solid: solid("fillet(r)"),
            faces: vec![ElementReference::new(model(), "extrude(box,end)")],
            thickness: 0.2.into(),
            other_faces: Vec::new(),
            other_thickness: 0.2.into(),
            outward: false,
        },
    );
    let rim = ElementReference::new(model(), "shell(s,extrude(box,end))");
    let mut lipped = program.clone();
    lipped.push(
        "l",
        LipArgs {
            face: rim.clone(),
            edges: Vec::new(),
            width: 0.08,
            height: 0.1,
        },
    );
    assert_valid(&lipped.build_model(registry).unwrap());
    program.push(
        "g",
        GrooveArgs {
            face: rim,
            edges: Vec::new(),
            width: 0.08,
            height: 0.1,
            clearance: 0.01,
        },
    );
    assert_valid(&program.build_model(registry).unwrap());
}
