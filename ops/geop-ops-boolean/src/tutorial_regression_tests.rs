//! Programs from tutorials and bug reports, rebuilt in Rust: the split of a
//! solid that fails where a real one asks for it.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_origin},
    whole_sketch,
};
use geop_ops_model::{
    model, solid,
    testing::{Built, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{Split, SplitArgs};

/// The operations these programs are written in: given solids, and the
/// sketch and extrusion of the face a solid is split with.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Primitive)?
        .with(Split)
}

/// A mould or a bearing block is split at a plane through its bore: a
/// 2 x 2 x 2 block with a hole of radius 0.5 drilled through it along `z`,
/// split by the plane `z = 1` — a line sketched on the Y plane and
/// extruded both ways into a face wider than the block.
///
/// From the SolidWorks "split" and the Onshape "split part" tutorials: a
/// body with a hole is split along a plane through the hole into two
/// halves, each keeping half the hole.
///
/// Once refused with "split: the face ends inside the solid", which was
/// right: the face was extruded 3 symmetric, 1.5 either way, and ended at
/// `y = 1.5` inside the block 2 deep, through the hole's far rim. Naming
/// the edge in the refusal showed it to be the face's own end edge. The
/// face now reaches 3 either way; the ring the section makes with the hole
/// splits as it should.
#[test]
fn split_a_block_across_its_through_hole() {
    let mut program = Program::new();
    program.push(
        "block",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]),
    );
    program.push(
        "hole",
        PrimitiveArgs::cylinder_z([1.0, 1.0, -1.0], 0.5, 4.0).cut("block(block)"),
    );
    // On the Y plane sketch `y` runs along world `-z`: the line at sketch
    // `y = -1` lies in the plane `z = 1`.
    let mut line = Sketch::new();
    let a = line.add_point(n(-1.0), n(-1.0));
    let b = line.add_point(n(3.0), n(-1.0));
    let cut = line.add_line(a, b);
    program.push(
        "cut_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Y), line),
    );
    program.push(
        "cut",
        ExtrudeArgs {
            sketch: whole_sketch("cut_sketch"),
            extent: Extents {
                side1: Extent::blind(6.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Faces { up_to: None },
            draft: 0.0,
        },
    );
    program.push(
        "halves",
        SplitArgs {
            solid: solid("cylinder(hole)"),
            face: ElementReference::new(model(), format!("extrude(cut,cut_sketch,{cut})")),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let halves: Vec<_> = (part.solid_names().into_iter())
        .filter(|name| name.starts_with("split("))
        .collect();
    assert_eq!(halves.len(), 2, "{halves:?}");
    let model = part.topology();
    let volume: f64 = halves
        .iter()
        .map(|name| {
            let solid = part.solid_id(name).unwrap();
            model.mass_properties(solid).unwrap().volume.to_f64()
        })
        .sum();
    let expected = 8.0 - std::f64::consts::PI * 0.25 * 2.0;
    assert!((volume - expected).abs() < 1e-6, "{volume} vs {expected}");
}
