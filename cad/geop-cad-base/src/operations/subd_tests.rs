//! SubD bodies in programs: built from their cage, and worked on further
//! by the other operations like any solid.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::{BooleanArgs, Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};
use geop_ops_subd::{Cage, Mirror, SubdArgs};

use crate::Program;
use crate::examples::n;

/// `validate_manifold` runs `validate` first: once is enough.
fn assert_valid(part: &Part<S>) {
    if let Err(report) = super::regression_tests::check_valid(part) {
        panic!("{report}");
    }
}

/// A rounded blob — the limit of a box cage two units a side around the
/// origin — united with a block reaching into it from one corner: one
/// valid solid.
#[test]
fn subd_body_united_with_a_box() {
    let mut program = Program::new();
    program.push(
        "blob",
        SubdArgs {
            cage: Cage::cuboid([2.0, 2.0, 2.0]),
            mirror: Mirror::None,
        },
    );
    let mut sketch = Sketch::new();
    let corners = [[0.1, 0.15], [1.4, 0.15], [1.4, 1.35], [0.1, 1.35]];
    let p: Vec<_> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        sketch.add_line(p[i], p[(i + 1) % p.len()]);
    }
    program.push(
        "block_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch,
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "block_sketch".into(),
            extent: Extents::blind(1.4),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "join",
        BooleanArgs {
            a: "subd(blob)".into(),
            b: "extrude(block)".into(),
            op: BooleanOp::Union,
        },
    );
    let part = program
        .build::<S>(&NoFiles)
        .unwrap_or_else(|e| panic!("{e}"));
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1, "{:?}", part.solid_names());
}
