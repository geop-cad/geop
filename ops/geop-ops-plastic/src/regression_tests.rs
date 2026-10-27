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
/// the mating half, a groove there instead. Both were refused:
///
/// > edge shell(s,fillet(r,extrude(box,outline,p3),fillet,st1)) is neither
/// > straight nor circular: a lip or groove runs along straight and
/// > circular edges
///
/// That edge is the inner copy of the blend's top edge, where the inner
/// wall's round corner meets the rim: the shell traces it as a fitted
/// cubic. The band a groove is cut along now offsets any curve (a cubic
/// fitted to its offset, proven near it), and where the chain runs on
/// smoothly is told by the walls' normals at the vertex, as a tangent
/// chain is: the groove builds.
///
/// The lip still refuses, by name. Its face along the edge stands flush
/// with the wall: the traced edge swept up, a surface a hair off the
/// wall's own, carried on past the rim. The union then exhausts its
/// curve-surface search on the lip's top edge lying in the wall's surface
/// (measured: 20000 nodes, 8311 pairs pending). A lip along such an edge
/// needs its face to be the wall's surface itself.
#[test]
#[ignore = "slow: shells a filleted box — run with `cargo test -- --ignored`"]
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
    let err = lipped.build_model(registry).unwrap_err().to_string();
    assert!(
        err.contains("a lip runs along straight and circular edges"),
        "{err}"
    );
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

/// The enclosure of [`crate::testing::enclosure`] — walls 0.2 thick — with a lip, and
/// for the mating half a groove, 0.3 wide: wider than the rim they stand
/// on. Both build, valid, without a word:
///
/// - the lip runs 0.3 out from the rim's inner edge, so it overhangs the
///   outer wall by 0.1 all round, a ledge outside the part: the point
///   `(-0.05, 1, 1.05)`, beside the wall just above the rim, is inside
///   `lip(l)`;
/// - the groove is cut 0.31 wide and so takes the whole rim away, the
///   wall lowered by the groove's depth all round instead of grooved:
///   `(0.02, 1, 0.95)`, in the outer skin of the wall just under the rim,
///   is outside `groove(g)`.
///
/// A lip or groove as wide as the rim or wider is no lip or groove; it
/// should be refused by name, as a lip ending at a corner is.
#[test]
#[ignore = "bug: a lip or groove wider than the rim builds silently — the lip overhangs the outer wall as a ledge, the groove removes the wall's whole top — instead of being refused"]
fn lip_and_groove_wider_than_the_rim() {
    use geop_core_topology::contains::shell::PointClassification::{Inside, Outside};
    use geop_ops_model::testing::classify;
    let rim = || ElementReference::new(model(), "shell(s,extrude(box,end))");
    let mut lipped = crate::testing::enclosure();
    lipped.push(
        "l",
        LipArgs {
            face: rim(),
            edges: Vec::new(),
            width: 0.3,
            height: 0.1,
        },
    );
    let mut wrong = Vec::new();
    if let Ok(part) = lipped.build_model(registry) {
        let beside = classify(&part, "lip(l)", [-0.05, 1.0, 1.05]);
        if beside != Outside {
            wrong.push(format!("the lip overhangs the wall: beside it {beside:?}"));
        }
    }
    let mut grooved = crate::testing::enclosure();
    grooved.push(
        "g",
        GrooveArgs {
            face: rim(),
            edges: Vec::new(),
            width: 0.3,
            height: 0.1,
            clearance: 0.01,
        },
    );
    if let Ok(part) = grooved.build_model(registry) {
        let skin = classify(&part, "groove(g)", [0.02, 1.0, 0.95]);
        if skin != Inside {
            wrong.push(format!("the groove takes the wall's outer skin: {skin:?}"));
        }
    }
    assert!(wrong.is_empty(), "{wrong:?}");
}
